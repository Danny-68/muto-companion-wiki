"""lidar_obstacle.py -- LiDAR-clearance-per-richting, in BASE_LINK's eigen
conventie (niet de rauwe laser-frame), voor Fase 5's wander-executor.

TEKEN-FIX 13 sep 2026: de eerdere +180-correctie was overgenomen van
lidar_overlay.py's STATIC_YAW voor de base_link->laser_scan_fix-transform --
maar /scan_fixed (wat deze functie daadwerkelijk ontvangt) heeft frame_id
"laser", een ANDER, apart kind-frame van base_link (bevestigd via
`ros2 topic echo /scan_fixed --field header`), niet "laser_scan_fix" (dat op
zijn beurt een kind van "laser" is, niet van base_link). Live opnieuw gemeten
met het EXACTE, uit de topic-header bevestigde framenaam
(`ros2 run tf2_ros tf2_echo base_link laser`, herhaald en consistent):
rotatie is 0 graden, geen 180. Dit was precies de "nog niet betrouwbaar
vastgestelde hoek-relatie" uit muto_wander_frontcheck_bug_2026-09-12 --
destijds alleen gemitigeerd met een richting-onafhankelijke veiligheidsgrens,
nooit echt gefixed. Live bevestigd: met deze fix koos de wander-executor
voor het eerst daadwerkelijk een reeel vrije richting i.p.v. te blijven
ronddraaien zonder vooruitgang (zie muto_lidar_heading_fix_2026-09-13-memory).
"""
import math

LASER_TO_BASE_LINK_YAW_DEG = 0.0  # tf2_echo-bevestigd tegen het juiste "laser"-frame, zie module-docstring


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
