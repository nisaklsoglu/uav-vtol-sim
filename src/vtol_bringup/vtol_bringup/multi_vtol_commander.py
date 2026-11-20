#!/usr/bin/env python3
"""SUAS 2027 & TEKNOFEST Full Mission Commander for Dual ArduPlane QuadPlane."""

import json
import math
import os
from collections import deque
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandInt, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from vtol_bringup.ballistics import compute_ballistic_drop, spawn_payload_box


class Phase(Enum):
    WAIT_FCU = auto()
    SET_GUIDED = auto()
    WAIT_GUIDED = auto()
    ARM = auto()
    WAIT_ARMED = auto()
    SPOOL_UP = auto()
    TAKEOFF = auto()
    WAIT_ALTITUDE = auto()
    NAVIGATING = auto()
    INTERCEPT_TARGET = auto()
    PAYLOAD_DROP = auto()
    RTL_LAND = auto()
    WAIT_LANDED = auto()
    MISSION_COMPLETE = auto()
    FAILED = auto()


R_EARTH = 6378137.0


def gps_to_enu(lat: float, lon: float, home_lat: float, home_lon: float) -> Tuple[float, float]:
    """Convert (lat, lon) to local ENU (East=x, North=y) meters relative to home."""
    d_lat = math.radians(lat - home_lat)
    d_lon = math.radians(lon - home_lon)
    east_x = R_EARTH * d_lon * math.cos(math.radians(home_lat))
    north_y = R_EARTH * d_lat
    return east_x, north_y


def enu_to_gps(east_x: float, north_y: float, home_lat: float, home_lon: float) -> Tuple[float, float]:
    """Convert local ENU (East=x, North=y) meters back to (lat, lon) relative to home."""
    d_lat = north_y / R_EARTH
    d_lon = east_x / (R_EARTH * math.cos(math.radians(home_lat)))
    lat = home_lat + math.degrees(d_lat)
    lon = home_lon + math.degrees(d_lon)
    return lat, lon


class VehicleCommander:
    SPOOL_UP_SEC = 2.0
    SERVICE_TIMEOUT_SEC = 5.0
    RETRY_PERIOD_SEC = 2.0
    MAX_RETRIES = 20
    STATUSTEXT_KEEP = 12

    # SUAS 2027 Task 5: Minimum 150 ft AGL (~45.7m) & 50 ft (~15.2m) accuracy
    MIN_DROP_ALT_M = 48.0
    DROP_RADIUS_M = 15.0
    DROP_STABILIZE_SEC = 2.0

    # MAVLink constants for ArduPlane Guided Reposition
    MAV_CMD_DO_REPOSITION = 192
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

    def __init__(
        self,
        node: Node,
        ns: str,
        cb_group: ReentrantCallbackGroup,
        mission_cfg: Dict,
    ):
        self.node = node
        self.ns = ns
        self.log = node.get_logger()

        veh_cfg = mission_cfg.get(ns, {})
        self.takeoff_alt_m = float(veh_cfg.get('takeoff_alt_m', 20.0))
        self.enable_nfz = bool(mission_cfg.get('enable_nfz', True))
        self.enable_ballistic_drop = bool(veh_cfg.get('enable_ballistic_drop', mission_cfg.get('enable_ballistic_drop', False)))
        if 'drop_radius_m' in mission_cfg:
            self.DROP_RADIUS_M = float(mission_cfg['drop_radius_m'])
        self.climb_verify_m = self.takeoff_alt_m * 0.8
        self.wp_radius_m = float(mission_cfg.get('wp_radius_m', 35.0))
        self.total_laps = int(veh_cfg.get('laps', 2))
        self.raw_waypoints: List[Dict] = veh_cfg.get('waypoints', [])

        self.phase = Phase.WAIT_FCU
        self.state = State()
        self.odom: Optional[Odometry] = None
        self.home_pose: Optional[PoseStamped] = None
        self.home_gps: Optional[NavSatFix] = None
        self.curr_gps: Optional[NavSatFix] = None
        self._status_texts: deque[str] = deque(maxlen=self.STATUSTEXT_KEEP)
        self._fcu_alive = False

        # Waypoints stored as (enu_x, enu_y, rel_alt, lat, lon)
        self.waypoints: List[Tuple[float, float, float, float, float]] = []
        self.current_wp_idx: int = 0
        self.current_lap: int = 1
        self.prev_wp_enu: Tuple[float, float, float] = (0.0, 0.0, self.takeoff_alt_m)
        self._last_nav_log: Optional[Time] = None
        self._last_wait_log: Optional[Time] = None

        # Dynamic Target / Payload Drop state (SUAS Task 4 & 5)
        self.payload_dropped: bool = False
        self.intercept_target: Optional[Tuple[float, float, float, float, float]] = None
        self._drop_deadline: Optional[Time] = None

        self._pending_future = None
        self._pending_kind: Optional[str] = None
        self._call_started: Optional[Time] = None
        self._retries = 0
        self._spool_deadline: Optional[Time] = None
        self._next_retry_time: Optional[Time] = None
        self._arm_service_ok = False
        self._post_land_reset = False

        # Case 2: Statik NFZ (No-Fly Zone) & Inter-UAV Separation
        self.obstacles: List[Dict] = mission_cfg.get('obstacles', [])
        self.detour_wp: Optional[Tuple[float, float, float, float, float]] = None
        self.avoiding_uav: bool = False

        prefix = f'/{ns}/mavros'
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.state_sub = node.create_subscription(
            State, f'{prefix}/state', self._on_state, state_qos, callback_group=cb_group
        )
        self.odom_sub = node.create_subscription(
            Odometry,
            f'{prefix}/global_position/local',
            self._on_odom,
            sensor_qos,
            callback_group=cb_group,
        )
        self.gps_sub = node.create_subscription(
            NavSatFix,
            f'{prefix}/global_position/global',
            self._on_gps,
            sensor_qos,
            callback_group=cb_group,
        )
        self.statustext_sub = node.create_subscription(
            StatusText,
            f'{prefix}/statustext/recv',
            self._on_statustext,
            sensor_qos,
            callback_group=cb_group,
        )

        # Perception / Target Detection Interrupt Subscriber
        # Publish NavSatFix (lat, lon, alt) or local ENU offset to /<ns>/target_gps to trigger intercept!
        self.target_sub = node.create_subscription(
            NavSatFix,
            f'/{ns}/target_gps',
            self._on_target_detected,
            10,
            callback_group=cb_group,
        )

        self.set_mode_cli = node.create_client(
            SetMode, f'{prefix}/set_mode', callback_group=cb_group
        )
        self.arming_cli = node.create_client(
            CommandBool, f'{prefix}/cmd/arming', callback_group=cb_group
        )
        self.takeoff_cli = node.create_client(
            CommandTOL, f'{prefix}/cmd/takeoff', callback_group=cb_group
        )
        self.cmd_int_cli = node.create_client(
            CommandInt, f'{prefix}/cmd/command_int', callback_group=cb_group
        )

    def _on_state(self, msg: State) -> None:
        self.state = msg
        if msg.connected:
            self._fcu_alive = True

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self._fcu_alive = True
        if self.home_pose is None:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.home_pose = pose

    def _on_gps(self, msg: NavSatFix) -> None:
        self._fcu_alive = True
        self.curr_gps = msg
        if self.home_gps is None and msg.status.status >= 0 and abs(msg.latitude) > 1e-5:
            self.home_gps = msg
            self.log.info(
                f'[{self.ns}] Home GPS kaydedildi: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}'
            )

    def _on_statustext(self, msg: StatusText) -> None:
        text = (msg.text or '').strip()
        if not text:
            return
        self._fcu_alive = True
        self._status_texts.append(text)
        lower = text.lower()
        if any(
            k in lower
            for k in ('prearm', 'arm:', 'arming', 'failed', 'reject', 'ekf', 'gps', 'transition')
        ):
            self.log.warn(f'[{self.ns}] STATUSTEXT: {text}')

    def _on_target_detected(self, msg: NavSatFix) -> None:
        """Triggered when camera/YOLO node detects the stranded hiker / drop target."""
        if self.payload_dropped or self.phase != Phase.NAVIGATING or self.home_gps is None:
            return

        h_lat = float(self.home_gps.latitude)
        h_lon = float(self.home_gps.longitude)

        # Support both global GPS (lat > 1.0 or < -1.0) and local ENU meter offsets (if lat/lon small)
        if abs(msg.latitude) > 1.0:
            t_lat = float(msg.latitude)
            t_lon = float(msg.longitude)
            ex, ny = gps_to_enu(t_lat, t_lon, h_lat, h_lon)
        else:
            ex = float(msg.latitude)
            ny = float(msg.longitude)
            t_lat, t_lon = enu_to_gps(ex, ny, h_lat, h_lon)

        # Enforce SUAS Task 5 minimum 150 ft AGL (>= 48.0 m) safety floor
        current_target_alt = self.waypoints[self.current_wp_idx][2] if self.waypoints else 50.0
        drop_alt = max(float(msg.altitude) if msg.altitude > 0.0 else current_target_alt, self.MIN_DROP_ALT_M)

        self.intercept_target = (ex, ny, drop_alt, t_lat, t_lon)
        self.log.warn(
            f'[{self.ns}] [HEDEF TESPİT EDİLDİ!] Rota kesiliyor -> Dağcı/Hedef koordinatına '
            f'yöneliniyor: ENU=({ex:.1f}m, {ny:.1f}m), İrtifa={drop_alt:.1f}m (>=150ft AGL)'
        )
        pos = self._position_enu()
        from_enu = pos if pos is not None else self.prev_wp_enu
        self._send_wp_with_overshoot(from_enu, self.intercept_target)
        self.phase = Phase.INTERCEPT_TARGET

    def _now(self) -> Time:
        return self.node.get_clock().now()

    def _position_enu(self) -> Optional[Tuple[float, float, float]]:
        if self.curr_gps is not None and self.home_gps is not None:
            ex, ny = gps_to_enu(
                self.curr_gps.latitude,
                self.curr_gps.longitude,
                self.home_gps.latitude,
                self.home_gps.longitude,
            )
            alt = float(self.odom.pose.pose.position.z) if self.odom else self.takeoff_alt_m
            return (ex, ny, alt)
        if self.odom is not None and self.home_pose is not None:
            p = self.odom.pose.pose.position
            hp = self.home_pose.pose.position
            return (float(p.x - hp.x), float(p.y - hp.y), float(p.z))
        return None

    def _build_waypoints(self) -> None:
        if self.home_gps is None:
            self.log.error(f'[{self.ns}] Home GPS henüz yok, rota oluşturulamadı!')
            return

        h_lat = float(self.home_gps.latitude)
        h_lon = float(self.home_gps.longitude)

        self.waypoints.clear()
        for wp in self.raw_waypoints:
            if 'lat' in wp and 'lon' in wp:
                lat = float(wp['lat'])
                lon = float(wp['lon'])
                ex, ny = gps_to_enu(lat, lon, h_lat, h_lon)
                z = float(wp.get('alt', self.takeoff_alt_m))
            else:
                ex = float(wp.get('x', 0.0))
                ny = float(wp.get('y', 0.0))
                z = float(wp.get('z', self.takeoff_alt_m))
                lat, lon = enu_to_gps(ex, ny, h_lat, h_lon)
            self.waypoints.append((ex, ny, z, lat, lon))

        self.current_wp_idx = 0
        self.current_lap = 1
        pos = self._position_enu()
        self.prev_wp_enu = pos if pos is not None else (0.0, 0.0, self.takeoff_alt_m)
        self.log.info(
            f'[{self.ns}] Hazırlanan Global Rota: {len(self.waypoints)} WP, {self.total_laps} Tur, Yarıçap: {self.wp_radius_m}m'
        )

    def _check_static_obstacles(
        self,
        pos: Tuple[float, float, float],
        target_wp: Tuple[float, float, float, float, float],
    ) -> Optional[Tuple[float, float, float, float, float]]:
        """Rota cizgisi bir NFZ silindirini kesiyorsa guvenli teget kacis noktasi (detour) dondurur."""
        if not self.enable_nfz or not self.obstacles or self.home_gps is None:
            return None

        px, py = pos[0], pos[1]
        tx, ty, tz = target_wp[0], target_wp[1], target_wp[2]
        vx, vy = tx - px, ty - py
        seg_len = math.hypot(vx, vy)
        if seg_len < 1.0:
            return None

        ux, uy = vx / seg_len, vy / seg_len
        for obs in self.obstacles:
            ox, oy = float(obs['x']), float(obs['y'])
            r_safe = float(obs['radius']) + 20.0

            # Engelin rota dogrultusundaki izdusumu
            proj = (ox - px) * ux + (oy - py) * uy
            if proj < 0.0 or proj > seg_len:
                continue

            # Engele olan dik mesafe ve ucağın engele mesafesi
            closest_x = px + proj * ux
            closest_y = py + proj * uy
            perp_dist = math.hypot(ox - closest_x, oy - closest_y)
            dist_to_obs = math.hypot(ox - px, oy - py)

            if perp_dist < r_safe and dist_to_obs < 110.0:
                # Engelin sol/sag dik vektorunden disari dogru 35m ofsetli kacis noktasi
                nx, ny = -uy, ux
                if (closest_x - ox) * nx + (closest_y - oy) * ny < 0.0:
                    nx, ny = -nx, -ny
                dx = ox + nx * (float(obs['radius']) + 35.0)
                dy = oy + ny * (float(obs['radius']) + 35.0)
                d_lat, d_lon = enu_to_gps(dx, dy, float(self.home_gps.latitude), float(self.home_gps.longitude))
                self.log.warn(
                    f"[{self.ns}] [NFZ KAÇINMA] {obs.get('name', 'NFZ')} tespit edildi! "
                    f"Teğet noktaya yöneliniyor: ENU=({dx:.1f}m, {dy:.1f}m)"
                )
                return (dx, dy, tz, d_lat, d_lon)
        return None

    def _compute_cross_track_error(
        self,
        curr: Tuple[float, float, float],
        wp_from: Tuple[float, float, float],
        wp_to: Tuple[float, float, float],
    ) -> float:
        dx = wp_to[0] - wp_from[0]
        dy = wp_to[1] - wp_from[1]
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-3:
            return math.hypot(curr[0] - wp_to[0], curr[1] - wp_to[1])
        return abs(dy * curr[0] - dx * curr[1] + wp_to[0] * wp_from[1] - wp_to[1] * wp_from[0]) / seg_len

    def _send_wp_with_overshoot(
        self,
        wp_from_enu: Tuple[float, float, float],
        target_wp: Tuple[float, float, float, float, float],
        overshoot_m: float = 80.0,
    ) -> None:
        """ArduPlane'in loiter cemberine girmeden gercek WP uzerinden duz gecmesi icin hedefi 80m uzatir."""
        dx = target_wp[0] - wp_from_enu[0]
        dy = target_wp[1] - wp_from_enu[1]
        dist = math.hypot(dx, dy)
        if dist > 1.0 and self.home_gps is not None:
            vx = target_wp[0] + (dx / dist) * overshoot_m
            vy = target_wp[1] + (dy / dist) * overshoot_m
            v_lat, v_lon = enu_to_gps(vx, vy, float(self.home_gps.latitude), float(self.home_gps.longitude))
            self._send_reposition_once(v_lat, v_lon, target_wp[2])
        else:
            self._send_reposition_once(target_wp[3], target_wp[4], target_wp[2])

    def _send_reposition_once(self, lat: float, lon: float, rel_alt: float) -> None:
        """Send one-shot MAV_CMD_DO_REPOSITION (192) so ArduPlane never resets FW transition."""
        if not self.cmd_int_cli.service_is_ready():
            self.log.warn(f'[{self.ns}] command_int servisi hazır değil!')
            return
        req = CommandInt.Request()
        req.broadcast = False
        req.frame = self.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        req.command = self.MAV_CMD_DO_REPOSITION
        req.current = 0
        req.autocontinue = 0
        req.param1 = -1.0  # default cruise speed
        req.param2 = 1.0   # MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
        req.param3 = 0.0
        req.param4 = 0.0
        req.x = int(round(lat * 1e7))
        req.y = int(round(lon * 1e7))
        req.z = float(rel_alt)
        self.cmd_int_cli.call_async(req)

    def _log_recent_statustext(self, context: str) -> None:
        if not self._status_texts:
            return
        joined = ' | '.join(list(self._status_texts)[-5:])
        self.log.warn(f'[{self.ns}] {context} — recent STATUSTEXT: {joined}')

    def _can_retry(self) -> bool:
        if self._pending_future is not None:
            return False
        if self._next_retry_time is None:
            return True
        return self._now() >= self._next_retry_time

    def _schedule_retry(self) -> None:
        self._retries += 1
        self._next_retry_time = self._now() + Duration(seconds=self.RETRY_PERIOD_SEC)
        if self._retries > self.MAX_RETRIES:
            if self.phase == Phase.TAKEOFF and any('already flying' in t.lower() for t in self._status_texts):
                self.log.warn(f'[{self.ns}] Araç zaten uçuşta görünüyor, doğrudan NAVIGATING fazına geçiliyor.')
                self._retries = 0
                self._build_waypoints()
                if self.waypoints:
                    wp0 = self.waypoints[0]
                    self._send_reposition_once(wp0[3], wp0[4], wp0[2])
                    self.phase = Phase.NAVIGATING
                else:
                    self.phase = Phase.RTL_LAND
                return
            self.log.error(
                f'[{self.ns}] exceeded {self.MAX_RETRIES} retries in {self.phase.name}'
            )
            self._log_recent_statustext('giving up')
            self.phase = Phase.FAILED

    def _clear_call(self) -> None:
        self._pending_future = None
        self._pending_kind = None
        self._call_started = None

    def _start_call(self, kind: str, future) -> None:
        self._pending_kind = kind
        self._pending_future = future
        self._call_started = self._now()

    def _poll_pending(self) -> Optional[bool]:
        if self._pending_future is None:
            return None

        kind = self._pending_kind or 'service'

        if not self._pending_future.done():
            elapsed = (self._now() - self._call_started).nanoseconds * 1e-9
            if elapsed > self.SERVICE_TIMEOUT_SEC:
                self.log.warn(
                    f'[{self.ns}] {kind} timed out after {self.SERVICE_TIMEOUT_SEC:.1f}s'
                )
                self._clear_call()
                self._schedule_retry()
                return False
            return None

        try:
            result = self._pending_future.result()
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f'[{self.ns}] {kind} exception: {exc}')
            self._clear_call()
            self._schedule_retry()
            return False

        self._clear_call()

        if kind.startswith('set_mode'):
            success = bool(getattr(result, 'mode_sent', False))
        else:
            success = bool(getattr(result, 'success', False))

        if success:
            self._retries = 0
            self._next_retry_time = None
            self.log.info(f'[{self.ns}] {kind} service OK')
            return True

        self.log.warn(f'[{self.ns}] {kind} rejected (success=False)')
        if kind.startswith('arming'):
            self._log_recent_statustext('ARM rejected by FCU')
        self._schedule_retry()
        return False

    def _call_set_mode(self, mode_str: str) -> None:
        if not self.set_mode_cli.service_is_ready():
            self._schedule_retry()
            return
        req = SetMode.Request()
        req.custom_mode = mode_str
        self.log.info(f'[{self.ns}] requesting mode {mode_str} (attempt {self._retries + 1})')
        self._start_call(f'set_mode({mode_str})', self.set_mode_cli.call_async(req))

    def _call_arm(self) -> None:
        if not self.arming_cli.service_is_ready():
            self._schedule_retry()
            return
        req = CommandBool.Request()
        req.value = True
        self.log.info(f'[{self.ns}] requesting ARM (attempt {self._retries + 1})')
        self._arm_service_ok = False
        self._start_call('arming', self.arming_cli.call_async(req))

    def _call_takeoff(self) -> None:
        if not self.takeoff_cli.service_is_ready():
            self._schedule_retry()
            return
        req = CommandTOL.Request()
        req.altitude = float(self.takeoff_alt_m)
        self.log.info(
            f'[{self.ns}] requesting takeoff to {self.takeoff_alt_m:.1f} m '
            f'(attempt {self._retries + 1})'
        )
        self._start_call('takeoff', self.takeoff_cli.call_async(req))

    def tick(self) -> None:
        if self.phase == Phase.FAILED:
            return

        if self.phase == Phase.MISSION_COMPLETE:
            if not self._post_land_reset and self.state.mode == 'QRTL' and self._can_retry():
                self._post_land_reset = True
                self._call_set_mode('GUIDED')
            return

        pending = self._poll_pending()

        if pending is None and self._pending_future is not None:
            return

        if self.phase == Phase.WAIT_FCU:
            if (self.state.connected or self._fcu_alive) and self.home_gps is not None:
                pos = self._position_enu()
                if pos is not None and pos[2] >= self.climb_verify_m:
                    self.log.info(
                        f'[{self.ns}] Araç zaten havada ({pos[2]:.1f}m) -> Doğrudan NAVIGATING!'
                    )
                    self._build_waypoints()
                    if self.waypoints:
                        wp0 = self.waypoints[0]
                        self._send_reposition_once(wp0[3], wp0[4], wp0[2])
                        self.phase = Phase.NAVIGATING
                    else:
                        self.phase = Phase.RTL_LAND
                    return

                self.log.info(f'[{self.ns}] FCU ve Home GPS hazır -> SET_GUIDED')
                self.phase = Phase.SET_GUIDED
            else:
                now = self._now()
                if (
                    self._last_wait_log is None
                    or (now - self._last_wait_log).nanoseconds * 1e-9 >= 3.0
                ):
                    self._last_wait_log = now
                    self.log.info(
                        f'[{self.ns}] WAIT_FCU bekleniyor... '
                        f'(fcu_alive={self._fcu_alive}, home_gps={self.home_gps is not None})'
                    )
            return

        if self.phase == Phase.SET_GUIDED:
            if self.state.mode == 'GUIDED' or pending is True:
                self.phase = Phase.WAIT_GUIDED
                return
            if self._can_retry():
                self._call_set_mode('GUIDED')
            return

        if self.phase == Phase.WAIT_GUIDED:
            if self.state.mode == 'GUIDED' or not self.state.connected:
                self.log.info(f'[{self.ns}] GUIDED moduna geçildi -> ARM')
                self.phase = Phase.ARM
            elif self._can_retry():
                self.phase = Phase.SET_GUIDED
            return

        if self.phase == Phase.ARM:
            if self.state.connected and self.state.mode not in ('GUIDED', ''):
                self._arm_service_ok = False
                self.phase = Phase.SET_GUIDED
                return
            if pending is True:
                self._arm_service_ok = True
                self.phase = Phase.WAIT_ARMED
                return
            if self._arm_service_ok and self.state.armed:
                self.phase = Phase.WAIT_ARMED
                return
            if self._can_retry():
                self._call_arm()
            return

        if self.phase == Phase.WAIT_ARMED:
            if self._arm_service_ok and (self.state.armed or not self.state.connected):
                self.log.info(f'[{self.ns}] Araç ARM oldu! — spool-up {self.SPOOL_UP_SEC:.1f}s')
                self._spool_deadline = self._now() + Duration(seconds=self.SPOOL_UP_SEC)
                self.phase = Phase.SPOOL_UP
                return
            if pending is False or (self._arm_service_ok and not self.state.armed):
                if self._can_retry():
                    self._arm_service_ok = False
                    self.phase = Phase.ARM
            return

        if self.phase == Phase.SPOOL_UP:
            if self._spool_deadline is not None and self._now() >= self._spool_deadline:
                self.phase = Phase.TAKEOFF
            return

        if self.phase == Phase.TAKEOFF:
            if pending is True:
                self.log.info(f'[{self.ns}] Takeoff komutu kabul edildi')
                self.phase = Phase.WAIT_ALTITUDE
                return
            if self._can_retry():
                self._call_takeoff()
            return

        if self.phase == Phase.WAIT_ALTITUDE:
            pos = self._position_enu()
            now = self._now()
            if (
                self._last_wait_log is None
                or (now - self._last_wait_log).nanoseconds * 1e-9 >= 2.0
            ):
                self._last_wait_log = now
                curr_z = pos[2] if pos is not None else -1.0
                self.log.info(
                    f'[{self.ns}] Tırmanılıyor: İrtifa={curr_z:.1f}m / Hedef={self.climb_verify_m:.1f}m'
                )
            if pos is not None and pos[2] >= self.climb_verify_m:
                self.log.info(
                    f'[{self.ns}] Tırmanış doğrulandı ({pos[2]:.1f} m) -> Rota Takibi (NAVIGATING) başlıyor!'
                )
                self._build_waypoints()
                if not self.waypoints:
                    self.phase = Phase.RTL_LAND
                    return
                wp0 = self.waypoints[0]
                self._send_reposition_once(wp0[3], wp0[4], wp0[2])
                self.phase = Phase.NAVIGATING
            return

        if self.phase == Phase.NAVIGATING:
            pos = self._position_enu()
            if pos is None or not self.waypoints:
                return

            target_wp = self.waypoints[self.current_wp_idx]

            # Statik NFZ kacisma aktifse once detour noktasini tamamla
            if self.detour_wp is not None:
                d_dist = math.hypot(self.detour_wp[0] - pos[0], self.detour_wp[1] - pos[1])
                detour_radius = max(self.wp_radius_m, 65.0)
                if d_dist <= detour_radius:
                    self.log.info(
                        f'[{self.ns}] [NFZ KAÇINMA] Engel geçildi (mesafe={d_dist:.1f}m) -> '
                        f'Ana WP {self.current_wp_idx + 1} hedefine dönülüyor.'
                    )
                    self.prev_wp_enu = (self.detour_wp[0], self.detour_wp[1], self.detour_wp[2])
                    self.detour_wp = None
                    self._send_reposition_once(target_wp[3], target_wp[4], target_wp[2])
                return

            detour = self._check_static_obstacles(pos, target_wp)
            if detour is not None:
                self.detour_wp = detour
                self._send_wp_with_overshoot(pos, detour, overshoot_m=45.0)
                return
            dist_2d = math.hypot(target_wp[0] - pos[0], target_wp[1] - pos[1])
            xte = self._compute_cross_track_error(
                pos, self.prev_wp_enu, (target_wp[0], target_wp[1], target_wp[2])
            )

            now = self._now()
            if (
                self._last_nav_log is None
                or (now - self._last_nav_log).nanoseconds * 1e-9 >= 2.0
            ):
                self._last_nav_log = now
                self.log.info(
                    f'[{self.ns}] [Tur {self.current_lap}/{self.total_laps} | '
                    f'WP {self.current_wp_idx + 1}/{len(self.waypoints)}] '
                    f'Kalan Mesafe: {dist_2d:.1f}m | İrtifa: {pos[2]:.1f}/{target_wp[2]:.1f}m | '
                    f'XTE (Sapma): {xte:.2f}m'
                )

            if dist_2d <= self.wp_radius_m:
                self.log.info(
                    f'[{self.ns}] >>> WP {self.current_wp_idx + 1} ULAŞILDI! (Sapma XTE: {xte:.2f}m) <<<'
                )
                self.prev_wp_enu = (target_wp[0], target_wp[1], target_wp[2])
                self.current_wp_idx += 1

                if self.current_wp_idx >= len(self.waypoints):
                    if self.current_lap < self.total_laps:
                        self.current_lap += 1
                        self.current_wp_idx = 0
                        next_wp = self.waypoints[0]
                        self._send_reposition_once(next_wp[3], next_wp[4], next_wp[2])
                        self.log.info(
                            f'[{self.ns}] === Tur {self.current_lap}/{self.total_laps} Başlıyor ==='
                        )
                    else:
                        self.log.info(
                            f'[{self.ns}] Tüm rota turları tamamlandı! Otonom QRTL İnişi başlatılıyor...'
                        )
                        self.phase = Phase.RTL_LAND
                else:
                    next_wp = self.waypoints[self.current_wp_idx]
                    self._send_reposition_once(next_wp[3], next_wp[4], next_wp[2])
            return

        if self.phase == Phase.INTERCEPT_TARGET:
            pos = self._position_enu()
            if pos is None or self.intercept_target is None:
                return

            tx, ty, tz, _, _ = self.intercept_target
            dist_2d = math.hypot(tx - pos[0], ty - pos[1])

            now = self._now()
            if (
                self._last_nav_log is None
                or (now - self._last_nav_log).nanoseconds * 1e-9 >= 1.5
            ):
                self._last_nav_log = now
                self.log.info(
                    f'[{self.ns}] [HEDEF YAKLAŞMA - TASK 5] Hedefe Mesafe: {dist_2d:.1f}m | '
                    f'İrtifa: {pos[2]:.1f}m (Min: {self.MIN_DROP_ALT_M:.1f}m)'
                )

            if self.enable_ballistic_drop:
                vel_enu = (0.0, 0.0, 0.0)
                if self.odom is not None:
                    q = self.odom.pose.pose.orientation
                    v = self.odom.twist.twist.linear
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    vx_e = v.x * math.cos(yaw) - v.y * math.sin(yaw)
                    vy_e = v.x * math.sin(yaw) + v.y * math.cos(yaw)
                    vel_enu = (vx_e, vy_e, v.z)
                bal = compute_ballistic_drop(pos, vel_enu, (tx, ty))
                if int(self._now().nanoseconds / 1e9) % 2 == 0:
                    self.log.info(f'[{self.ns}] [BALİSTİK] Hata: {bal.impact_error_m:.1f}m | Lead: {bal.lead_distance_m:.1f}m | Hız: {bal.ground_speed_mps:.1f}m/s')
                if bal.impact_error_m <= self.DROP_RADIUS_M and pos[2] >= 45.0:
                    box_name = f'{self.ns}_payload_box'
                    spawn_payload_box(box_name, pos)
                    self.log.warn(
                        f'[{self.ns}] >>> BALİSTİK BIRAKMA TETİKLENDİ! '
                        f'Uçak Mesafe: {dist_2d:.1f}m | Lead: {bal.lead_distance_m:.1f}m | '
                        f'Hız: {bal.ground_speed_mps:.1f}m/s | Düşüş Süresi: {bal.fall_time_s:.2f}s | '
                        f'Tahmini Düşüş Hatası: {bal.impact_error_m:.2f}m <= {self.DROP_RADIUS_M:.1f}m <<<'
                    )
                    self._drop_deadline = self._now() + Duration(seconds=self.DROP_STABILIZE_SEC)
                    self.phase = Phase.PAYLOAD_DROP
                    return
            if dist_2d <= self.DROP_RADIUS_M and pos[2] >= 45.0:
                self.log.warn(
                    f'[{self.ns}] >>> HEDEF ÜZERİNDE! (Mesafe: {dist_2d:.1f}m <= 50ft, '
                    f'İrtifa: {pos[2]:.1f}m >= 150ft AGL) -> FAYDALI YÜK BIRAKILIYOR! <<<'
                )
                self._drop_deadline = self._now() + Duration(seconds=self.DROP_STABILIZE_SEC)
                self.phase = Phase.PAYLOAD_DROP
            return

        if self.phase == Phase.PAYLOAD_DROP:
            if self._drop_deadline is not None and self._now() >= self._drop_deadline:
                self.payload_dropped = True
                self.intercept_target = None
                resume_wp = self.waypoints[self.current_wp_idx]
                self.log.info(
                    f'[{self.ns}] Faydalı yük başarıyla bırakıldı! '
                    f'Kaldığı yerden (WP {self.current_wp_idx + 1}) rotaya devam ediliyor...'
                )
                self._send_reposition_once(resume_wp[3], resume_wp[4], resume_wp[2])
                self.phase = Phase.NAVIGATING
            return

        if self.phase == Phase.RTL_LAND:
            if pending is True:
                self.log.info(
                    f'[{self.ns}] QRTL komutu aktif -> Başlangıç noktasına dönülüp dikey iniş yapılıyor...'
                )
                self.phase = Phase.WAIT_LANDED
                return
            if self._can_retry():
                self._call_set_mode('QRTL')
            return

        if self.phase == Phase.WAIT_LANDED:
            pos = self._position_enu()
            alt = pos[2] if pos is not None else 0.0
            now = self._now()
            if (
                self._last_wait_log is None
                or (now - self._last_wait_log).nanoseconds * 1e-9 >= 3.0
            ):
                self._last_wait_log = now
                self.log.info(f'[{self.ns}] QRTL İniş sürecinde... Anlık İrtifa: {alt:.1f}m')

            if (self.state.connected and not self.state.armed and alt < 2.0) or alt < 0.4:
                self.log.info(
                    f'[{self.ns}] === OTONOM İNİŞ TAMAMLANDI! GÖREV BAŞARIYLA BİTTİ ==='
                )
                self.phase = Phase.MISSION_COMPLETE
            return


class MultiVtolCommander(Node):
    def __init__(self):
        super().__init__('multi_vtol_commander')
        self.declare_parameter('case', '')
        self._cb_group = ReentrantCallbackGroup()
        mission_cfg = self._load_mission_config()
        self.enable_separation = bool(mission_cfg.get('enable_separation', True))

        self.vehicles = [
            VehicleCommander(self, 'vtol_1', self._cb_group, mission_cfg),
            VehicleCommander(self, 'vtol_2', self._cb_group, mission_cfg),
        ]
        self._timer = self.create_timer(0.1, self._on_timer, callback_group=self._cb_group)
        self.get_logger().info(
            'Multi-VTOL SUAS 2027 Commander ready — Takeoff -> FW Nav -> Target Drop -> QRTL Land'
        )

    def _load_mission_config(self) -> Dict:
        try:
            share_dir = get_package_share_directory('vtol_bringup')
            case_param = str(self.get_parameter('case').value or '').strip().lower()
            case_map = {
                'case1': 'case1_navigation.json',
                'case1_navigation': 'case1_navigation.json',
                'case2': 'case2_obstacles_separation.json',
                'case2_obstacles_separation': 'case2_obstacles_separation.json',
                'case3': 'case3_target_drop.json',
                'case3_target_drop': 'case3_target_drop.json',
                'case4': 'case4_target_drop.json',
                'case4_target_drop': 'case4_target_drop.json',
            }
            if case_param in case_map:
                cfg_path = os.path.join(share_dir, 'config', 'cases', case_map[case_param])
            else:
                cfg_path = os.path.join(share_dir, 'config', 'suas_mission.json')
            if os.path.exists(cfg_path):
                with open(cfg_path, 'r', encoding='utf-8') as f:
                    self.get_logger().info(f'Görev dosyası yüklendi: {cfg_path}')
                    return json.load(f)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'suas_mission.json okunamadı, varsayılan rota kullanılacak: {exc}')

        return {
            'wp_radius_m': 55.0,
            'vtol_1': {
                'takeoff_alt_m': 20.0,
                'laps': 2,
                'waypoints': [
                    {'x': 150.0, 'y': 0.0, 'z': 46.0},
                    {'x': 150.0, 'y': 150.0, 'z': 46.0},
                    {'x': 0.0, 'y': 150.0, 'z': 46.0},
                    {'x': 0.0, 'y': 0.0, 'z': 46.0},
                ],
            },
            'vtol_2': {
                'takeoff_alt_m': 30.0,
                'laps': 2,
                'waypoints': [
                    {'x': 150.0, 'y': -50.0, 'z': 60.0},
                    {'x': 150.0, 'y': -200.0, 'z': 60.0},
                    {'x': 0.0, 'y': -200.0, 'z': 60.0},
                    {'x': 0.0, 'y': -50.0, 'z': 60.0},
                ],
            },
        }

    def _check_inter_uav_separation(self) -> None:
        if not getattr(self, 'enable_separation', True) or len(self.vehicles) < 2:
            return
        v1, v2 = self.vehicles[0], self.vehicles[1]
        active_phases = (Phase.NAVIGATING, Phase.INTERCEPT_TARGET)
        if v1.phase not in active_phases or v2.phase not in active_phases:
            return
        if v1.curr_gps is None or v2.curr_gps is None:
            return
        p1, p2 = v1._position_enu(), v2._position_enu()
        if p1 is None or p2 is None:
            return

        dx, dy = gps_to_enu(
            v2.curr_gps.latitude, v2.curr_gps.longitude,
            v1.curr_gps.latitude, v1.curr_gps.longitude
        )
        horiz_dist = math.hypot(dx, dy)
        vert_dist = abs(p1[2] - p2[2])

        if not v2.avoiding_uav and horiz_dist < 45.0 and vert_dist < 12.0:
            v2.avoiding_uav = True
            target_wp = v2.waypoints[v2.current_wp_idx]
            safe_alt = max(p1[2], target_wp[2]) + 15.0
            self.get_logger().warn(
                f'[SEPARATION] vtol_1 - vtol_2 yakınlaşma! '
                f'(Yatay: {horiz_dist:.1f}m, Dikey: {vert_dist:.1f}m) -> vtol_2 {safe_alt:.1f}m irtifaya tırmanıyor.'
            )
            v2._send_reposition_once(target_wp[3], target_wp[4], safe_alt)
        elif v2.avoiding_uav and horiz_dist > 65.0:
            v2.avoiding_uav = False
            target_wp = v2.waypoints[v2.current_wp_idx]
            self.get_logger().info(
                f'[SEPARATION] Güvenli ayrım sağlandı ({horiz_dist:.1f}m) -> vtol_2 normal irtifaya ({target_wp[2]:.1f}m) dönüyor.'
            )
            v2._send_reposition_once(target_wp[3], target_wp[4], target_wp[2])

    def _on_timer(self) -> None:
        self._check_inter_uav_separation()
        for vehicle in self.vehicles:
            try:
                vehicle.tick()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'[{vehicle.ns}] tick error: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiVtolCommander()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
