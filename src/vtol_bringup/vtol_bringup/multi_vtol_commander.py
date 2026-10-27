#!/usr/bin/env python3
"""Independent dual-VTOL takeoff commander for MAVROS / ArduPilot QuadPlane."""

from collections import deque
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time


class Phase(Enum):
    WAIT_FCU = auto()
    SET_GUIDED = auto()
    WAIT_GUIDED = auto()
    ARM = auto()
    WAIT_ARMED = auto()
    SPOOL_UP = auto()
    TAKEOFF = auto()
    WAIT_ALTITUDE = auto()
    HOLD = auto()
    FAILED = auto()


class VehicleCommander:
    """Per-vehicle state machine. One vehicle's delay never blocks the other."""

    TAKEOFF_ALT_M = 15.0
    CLIMB_VERIFY_M = 12.0
    SPOOL_UP_SEC = 2.0
    SERVICE_TIMEOUT_SEC = 5.0
    RETRY_PERIOD_SEC = 2.0
    MAX_RETRIES = 12
    STATUSTEXT_KEEP = 12

    def __init__(self, node: Node, ns: str, cb_group: ReentrantCallbackGroup):
        self.node = node
        self.ns = ns
        self.log = node.get_logger()

        self.phase = Phase.WAIT_FCU
        self.state = State()
        self.odom: Optional[Odometry] = None
        self.home_pose: Optional[PoseStamped] = None
        self._status_texts: deque[str] = deque(maxlen=self.STATUSTEXT_KEEP)

        self._pending_future = None
        self._pending_kind: Optional[str] = None
        self._call_started: Optional[Time] = None
        self._retries = 0
        self._spool_deadline: Optional[Time] = None
        self._next_retry_time: Optional[Time] = None
        self._arm_service_ok = False
        self._hold_pose = PoseStamped()

        prefix = f'/{ns}/mavros'
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.state_sub = node.create_subscription(
            State, f'{prefix}/state', self._on_state, 10, callback_group=cb_group
        )
        self.odom_sub = node.create_subscription(
            Odometry,
            f'{prefix}/global_position/local',
            self._on_odom,
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
        self.setpoint_pub = node.create_publisher(
            PoseStamped, f'{prefix}/setpoint_position/local', 10
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

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        if self.home_pose is None:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.home_pose = pose

    def _on_statustext(self, msg: StatusText) -> None:
        text = (msg.text or '').strip()
        if not text:
            return
        self._status_texts.append(text)
        lower = text.lower()
        if any(k in lower for k in ('prearm', 'arm:', 'arming', 'failed', 'reject', 'ekf', 'gps')):
            self.log.warn(f'[{self.ns}] STATUSTEXT: {text}')

    def _now(self) -> Time:
        return self.node.get_clock().now()

    def _altitude(self) -> Optional[float]:
        if self.odom is None:
            return None
        return float(self.odom.pose.pose.position.z)

    def _log_recent_statustext(self, context: str) -> None:
        if not self._status_texts:
            self.log.warn(f'[{self.ns}] {context} — no recent STATUSTEXT')
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
        """Return True/False when a pending service call finishes; None if idle/waiting."""
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

        if kind.startswith('arming'):
            success = bool(getattr(result, 'success', False))
        elif kind.startswith('set_mode'):
            success = bool(getattr(result, 'mode_sent', False))
        else:
            success = bool(getattr(result, 'success', False))

        if success:
            self._retries = 0
            self._next_retry_time = None
            self.log.info(f'[{self.ns}] {kind} service OK (success/mode_sent=True)')
            return True

        self.log.warn(f'[{self.ns}] {kind} rejected (success=False)')
        if kind.startswith('arming'):
            self._log_recent_statustext('ARM rejected by FCU')
        self._schedule_retry()
        return False

    def _call_set_guided(self) -> None:
        if not self.set_mode_cli.service_is_ready():
            self.log.info(f'[{self.ns}] waiting for set_mode service...')
            return
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        self.log.info(f'[{self.ns}] requesting GUIDED (attempt {self._retries + 1})')
        self._start_call('set_mode(GUIDED)', self.set_mode_cli.call_async(req))

    def _call_arm(self) -> None:
        if not self.arming_cli.service_is_ready():
            self.log.info(f'[{self.ns}] waiting for arming service...')
            return
        req = CommandBool.Request()
        req.value = True
        self.log.info(f'[{self.ns}] requesting ARM (attempt {self._retries + 1})')
        self._arm_service_ok = False
        self._start_call('arming', self.arming_cli.call_async(req))

    def _call_takeoff(self) -> None:
        if not self.takeoff_cli.service_is_ready():
            self.log.info(f'[{self.ns}] waiting for takeoff service...')
            return
        req = CommandTOL.Request()
        req.altitude = float(self.TAKEOFF_ALT_M)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        self.log.info(
            f'[{self.ns}] requesting takeoff to {self.TAKEOFF_ALT_M:.1f} m '
            f'(attempt {self._retries + 1})'
        )
        self._start_call('takeoff', self.takeoff_cli.call_async(req))

    def _update_hold_pose(self) -> None:
        pose = PoseStamped()
        pose.header.stamp = self._now().to_msg()
        pose.header.frame_id = 'map'
        if self.home_pose is not None:
            pose.pose.position.x = self.home_pose.pose.position.x
            pose.pose.position.y = self.home_pose.pose.position.y
            pose.pose.orientation = self.home_pose.pose.orientation
        elif self.odom is not None:
            pose.pose.position.x = self.odom.pose.pose.position.x
            pose.pose.position.y = self.odom.pose.pose.position.y
            pose.pose.orientation = self.odom.pose.pose.orientation
        pose.pose.position.z = self.TAKEOFF_ALT_M
        self._hold_pose = pose

    def _publish_setpoint(self) -> None:
        self._hold_pose.header.stamp = self._now().to_msg()
        self.setpoint_pub.publish(self._hold_pose)

    def tick(self) -> None:
        if self.phase == Phase.FAILED:
            return

        pending = self._poll_pending()

        if self.phase in (Phase.WAIT_ALTITUDE, Phase.HOLD):
            self._publish_setpoint()

        if pending is None and self._pending_future is not None:
            return

        if self.phase == Phase.WAIT_FCU:
            if self.state.connected:
                self.log.info(f'[{self.ns}] FCU connected')
                self.phase = Phase.SET_GUIDED
            return

        if self.phase == Phase.SET_GUIDED:
            if self.state.mode == 'GUIDED':
                self.phase = Phase.WAIT_GUIDED
                return
            if pending is True:
                # mode_sent only — wait until state.mode confirms GUIDED
                self.phase = Phase.WAIT_GUIDED
                return
            if self._can_retry():
                self._call_set_guided()
            return

        if self.phase == Phase.WAIT_GUIDED:
            if self.state.mode == 'GUIDED':
                self.log.info(f'[{self.ns}] mode confirmed GUIDED')
                self.phase = Phase.ARM
            elif self._can_retry():
                self.phase = Phase.SET_GUIDED
            return

        if self.phase == Phase.ARM:
            if self.state.mode != 'GUIDED':
                self.log.warn(
                    f'[{self.ns}] mode={self.state.mode} before ARM; recovering GUIDED'
                )
                self._arm_service_ok = False
                self.phase = Phase.SET_GUIDED
                return
            # Never treat telemetry-only armed as success without our service OK,
            # and never advance on service OK alone without state.armed.
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
            if self.state.mode != 'GUIDED':
                self.log.warn(
                    f'[{self.ns}] left GUIDED before arm latch (mode={self.state.mode}); '
                    're-selecting GUIDED'
                )
                self._arm_service_ok = False
                self.phase = Phase.SET_GUIDED
                return
            if self._arm_service_ok and self.state.armed:
                self.log.info(
                    f'[{self.ns}] Araç ARM oldu! (service success + state.armed) '
                    f'— spool-up {self.SPOOL_UP_SEC:.1f}s'
                )
                self._spool_deadline = self._now() + Duration(seconds=self.SPOOL_UP_SEC)
                self.phase = Phase.SPOOL_UP
                return
            if pending is False or (self._arm_service_ok and not self.state.armed):
                # Service claimed success but FCU never armed — Pre-Arm etc.
                if self._arm_service_ok and not self.state.armed and self._can_retry():
                    self.log.warn(
                        f'[{self.ns}] arming reported success but state.armed=False; retrying'
                    )
                    self._log_recent_statustext('ARM not latched')
                    self._arm_service_ok = False
                    self.phase = Phase.ARM
                elif not self._arm_service_ok and self._can_retry():
                    self.phase = Phase.ARM
            elif self._can_retry() and not self._arm_service_ok:
                self.phase = Phase.ARM
            return

        if self.phase == Phase.SPOOL_UP:
            if self.state.mode != 'GUIDED':
                self.log.warn(f'[{self.ns}] left GUIDED during spool-up; recovering')
                self._arm_service_ok = False
                self.phase = Phase.SET_GUIDED
                return
            if not self.state.armed:
                self.log.warn(f'[{self.ns}] disarmed during spool-up; re-arming')
                self._arm_service_ok = False
                self.phase = Phase.ARM
                return
            if self._spool_deadline is not None and self._now() >= self._spool_deadline:
                self.phase = Phase.TAKEOFF
            return

        if self.phase == Phase.TAKEOFF:
            if self.state.mode != 'GUIDED':
                self.log.warn(f'[{self.ns}] left GUIDED before takeoff; recovering')
                self._arm_service_ok = False
                self.phase = Phase.SET_GUIDED
                return
            if not self.state.armed:
                self.log.warn(f'[{self.ns}] not armed before takeoff; re-arming')
                self._arm_service_ok = False
                self.phase = Phase.ARM
                return
            if pending is True:
                self.log.info(f'[{self.ns}] Takeoff komutu kabul edildi')
                self._update_hold_pose()
                self.phase = Phase.WAIT_ALTITUDE
                return
            if self._can_retry():
                self._call_takeoff()
            return

        if self.phase == Phase.WAIT_ALTITUDE:
            alt = self._altitude()
            if alt is not None and alt >= self.CLIMB_VERIFY_M:
                self.log.info(
                    f'[{self.ns}] climb verified at {alt:.1f} m '
                    f'(>= {self.CLIMB_VERIFY_M:.1f} m) — holding setpoint'
                )
                self._update_hold_pose()
                self.phase = Phase.HOLD
            return

        if self.phase == Phase.HOLD:
            return


class MultiVtolCommander(Node):
    def __init__(self):
        super().__init__('multi_vtol_commander')
        self._cb_group = ReentrantCallbackGroup()
        self.vehicles = [
            VehicleCommander(self, 'vtol_1', self._cb_group),
            VehicleCommander(self, 'vtol_2', self._cb_group),
        ]
        self._timer = self.create_timer(0.1, self._on_timer, callback_group=self._cb_group)
        self.get_logger().info(
            'Multi-VTOL commander ready — independent GUIDED→ARM→spool→takeoff FSMs'
        )

    def _on_timer(self) -> None:
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
