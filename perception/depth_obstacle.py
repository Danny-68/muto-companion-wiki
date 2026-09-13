"""depth_obstacle.py -- Fase 5-vervolg (12 sep 2026): dieptecamera als tweede
obstakel-sensor naast LiDAR, per architectuurbeslissing 7 (SLAM/AMCL
geparkeerd, lokale obstakelvermijding + camera-checks is nu de standaard-
aanpak, zie muto_slam_parked_2026-09-12-memory).

Waarom een tweede sensor: de LiDAR is 2D/planar op één vaste montagehoogte --
hij ziet niets boven of onder dat vlak (tafelranden, overhangende obstakels,
glas). De Astra-dieptecamera (`/camera/depth/image_raw`, 16UC1 = uint16 in
millimeters, HFOV ~58 graden) dekt een verticale band en kan dat soort
obstakels wel zien, al is het gezichtsveld veel smaller dan de LiDAR's 360
graden -- dus dit is bewust een AANVULLING vooral voorwaarts, geen vervanging.

Puur berekenlogica hier, geen ROS/rclpy-afhankelijkheid -- zelfde scheiding
als behavior.py (perceptie/besturing gescheiden, makkelijk offline te testen).
De caller (nog te bouwen wander-executor) voedt een ruwe depth-frame
(numpy uint16, mm) + de camera-intrinsics aan.
"""
import math

INVALID_DEPTH_MM = 0  # Astra/OpenNI2-conventie: 0 = geen geldige meting
MAX_USEFUL_DEPTH_MM = 4000  # verder dan 4m is voor obstakelvermijding niet interessant


def estimate_clearance_by_heading(depth_mm, fx: float, cx: float,
                                    heading_step_deg: float = 15.0,
                                    vertical_band_frac: float = 0.4,
                                    min_valid_fraction: float = 0.2):
    """Rekent een ruwe depth-frame (2D numpy-array, uint16, mm) om naar
    {heading_deg: clearance_m} voor de headings die binnen de camera's HFOV
    vallen -- zelfde vorm als check_full_clearance.py/choose_heading()
    verwachten van LiDAR-data, zodat ze direct te combineren zijn (zie
    merge_lidar_and_depth_clearance()).

    Voor elke heading-kolom-band wordt het 10e percentiel van de geldige
    diepte-metingen gebruikt (niet het strikte minimum) -- robuuster tegen
    losse ruis-pixels die anders een valse "praktisch nul"-clearance geven,
    maar nog steeds conservatief genoeg om echte obstakels niet te missen.
    """
    height, width = depth_mm.shape
    half_hfov_deg = math.degrees(math.atan2(width - cx, fx))
    neg_half_hfov_deg = math.degrees(math.atan2(-cx, fx))

    y0 = int(height * (0.5 - vertical_band_frac / 2))
    y1 = int(height * (0.5 + vertical_band_frac / 2))
    band = depth_mm[y0:y1, :]

    result = {}
    heading = math.ceil(neg_half_hfov_deg / heading_step_deg) * heading_step_deg
    while heading <= half_hfov_deg:
        # kolom-index voor deze heading: x = cx + fx * tan(heading)
        x_center = cx + fx * math.tan(math.radians(heading))
        col_half_width = max(1, int(fx * math.tan(math.radians(heading_step_deg / 2))))
        x0 = max(0, int(x_center - col_half_width))
        x1 = min(width, int(x_center + col_half_width))
        if x1 <= x0:
            heading += heading_step_deg
            continue

        strip = band[:, x0:x1]
        valid = strip[(strip > INVALID_DEPTH_MM) & (strip <= MAX_USEFUL_DEPTH_MM * 2)]
        total_px = strip.size
        if total_px == 0 or valid.size < min_valid_fraction * total_px:
            heading += heading_step_deg
            continue  # te weinig geldige data voor deze richting -- niet raden

        valid_sorted = sorted(valid.tolist())
        p10 = valid_sorted[max(0, int(0.10 * len(valid_sorted)) - 1)]
        result[round(heading)] = min(p10 / 1000.0, MAX_USEFUL_DEPTH_MM / 1000.0)
        heading += heading_step_deg

    return result


def merge_lidar_and_depth_clearance(lidar_clearance: dict, depth_clearance: dict) -> dict:
    """Combineert twee {heading_deg: clearance_m}-dicts conservatief: waar
    beide een waarde hebben, het minimum (het meest voorzichtige antwoord);
    waar er maar één is, die waarde; LiDAR dekt 360 graden, dieptecamera
    typisch alleen het voorwaartse HFOV-segment eromheen."""
    merged = dict(lidar_clearance)
    for heading, depth_val in depth_clearance.items():
        if heading in merged:
            merged[heading] = min(merged[heading], depth_val)
        else:
            merged[heading] = depth_val
    return merged
