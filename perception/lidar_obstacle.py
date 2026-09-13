"""lidar_obstacle.py -- LiDAR-clearance-per-richting, in BASE_LINK's eigen
conventie (niet de rauwe laser-frame), voor Fase 5's wander-executor.

De +180-correctie hieronder is niet aangenomen maar hergebruikt van een al
tf2_echo-bevestigde meting elders in dit project: lidar_overlay.py's
STATIC_YAW = math.pi voor de base_link->laser_scan_fix-transform (zie
muto_front_convention.py/muto_localization_approach_pivot_2026-08-21-memory).
Puur berekenlogica, geen rclpy-afhankelijkheid -- zelfde scheiding als
depth_obstacle.py/behavior.py.
"""
import math

LASER_TO_BASE_LINK_YAW_DEG = 180.0  # tf2_echo-bevestigd, zie module-docstring


def estimate_clearance_by_heading(ranges, angle_min_rad: float, angle_increment_rad: float,
                                    range_min_m: float, range_max_m: float,
                                    heading_step_deg: float = 15.0, useful_range_m: float = 4.0):
    """ranges in de rauwe /scan_fixed-laserframe (msg.ranges, msg.angle_min/
    angle_increment/range_min/range_max ongewijzigd doorgegeven). Output al
    omgerekend naar base_link's eigen conventie -- direct combineerbaar met
    depth_obstacle.py's output (die al in base_link-conventie is, zie diens
    module-docstring: camera-bearing bleek 12 sep 2026 al zonder correctie te
    kloppen, face_follow_controller-test)."""
    buckets = {}
    for i, r in enumerate(ranges):
        if r is None or math.isnan(r) or math.isinf(r) or not (range_min_m < r < range_max_m):
            continue
        r = min(r, useful_range_m)
        laser_deg = math.degrees(angle_min_rad + i * angle_increment_rad)
        base_deg = laser_deg + LASER_TO_BASE_LINK_YAW_DEG
        base_deg = ((base_deg + 180.0) % 360.0) - 180.0  # normaliseer naar [-180, 180]
        bucket = round(base_deg / heading_step_deg) * heading_step_deg
        buckets.setdefault(bucket, []).append(r)
    # Conservatief: het minimum binnen elke richting-bucket, niet een gemiddelde --
    # één echt obstakel in een bucket moet die richting als "niet vrij" laten gelden.
    return {h: min(vals) for h, vals in buckets.items()}
