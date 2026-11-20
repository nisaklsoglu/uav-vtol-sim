import math
import subprocess
from dataclasses import dataclass
from typing import Tuple

GRAVITY = 9.81


@dataclass
class BallisticResult:
    fall_time_s: float
    ground_speed_mps: float
    lead_distance_m: float
    predicted_impact_xy: Tuple[float, float]
    impact_error_m: float


def compute_ballistic_drop(
    pos_enu: Tuple[float, float, float],
    vel_enu: Tuple[float, float, float],
    target_xy: Tuple[float, float],
    g: float = GRAVITY,
) -> BallisticResult:
    """
    Uçağın anlık ENU konumu (x, y, z) ve hızına (vx, vy, vz) göre
    yük şu an bırakılırsa düşeceği noktayı ve hedefe olan hatayı hesaplar.
    """
    x, y, z = pos_enu
    vx, vy, vz = vel_enu
    tx, ty = target_xy

    h = max(z, 0.1)
    # Dikey hız (vz) varsa onu da hesaba katan düşüş süresi: h + vz*t - 0.5*g*t^2 = 0
    discriminant = vz * vz + 2.0 * g * h
    fall_time_s = (vz + math.sqrt(max(discriminant, 0.0))) / g

    ground_speed = math.hypot(vx, vy)
    lead_distance = ground_speed * fall_time_s

    pred_x = x + vx * fall_time_s
    pred_y = y + vy * fall_time_s
    impact_error = math.hypot(tx - pred_x, ty - pred_y)

    return BallisticResult(
        fall_time_s=fall_time_s,
        ground_speed_mps=ground_speed,
        lead_distance_m=lead_distance,
        predicted_impact_xy=(pred_x, pred_y),
        impact_error_m=impact_error,
    )


def spawn_payload_box(
    box_name: str,
    pos_enu: Tuple[float, float, float],
    world_name: str = "quadplane_dual_runway",
) -> None:
    """
    Gazebo Harmonic sahnesinde uçağın 0.8 m altında fiziksel bir faydalı yük kutusu oluşturur.
    Non-blocking çalışır, ROS 2 döngüsünü kilitlemez.
    """
    x, y, z = pos_enu
    spawn_z = max(z - 0.8, 0.5)

    sdf = (
        f"<sdf version='1.8'>"
        f"<model name='{box_name}'>"
        f"<pose>{x:.2f} {y:.2f} {spawn_z:.2f} 0 0 0</pose>"
        f"<link name='link'>"
        f"<inertial><mass>1.5</mass></inertial>"
        f"<collision name='col'><geometry><box><size>0.4 0.4 0.4</size></box></geometry></collision>"
        f"<visual name='vis'><geometry><box><size>0.4 0.4 0.4</size></box></geometry>"
        f"<material><ambient>1 0.2 0 1</ambient><diffuse>1 0.2 0 1</diffuse></material></visual>"
        f"</link></model></sdf>"
    )

    req = f'sdf: "{sdf}"'
    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
