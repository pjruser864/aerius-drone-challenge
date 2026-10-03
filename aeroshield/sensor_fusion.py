import math
from config import WEIGHTS

def clamp(x, a=0.0, b=100.0):
    try:
        return max(a, min(b, float(x)))
    except (TypeError, ValueError):
        return a

def angle_diff(a, b):
    d = abs((float(a) - float(b)) % 360.0)
    return min(d, 360.0 - d)

def radar_score(r):
    confidence = clamp(r.get("confidence", 0) * 100)
    speed = clamp(r.get("speed", 0) / 25 * 100)
    distance = clamp((500 - r.get("distance", 999)) / 500 * 100)
    return clamp(0.45 * confidence + 0.30 * speed + 0.25 * distance)

def gps_score(g):
    accuracy = clamp((30 - g.get("accuracy", 999)) / 30 * 100)
    direction = 100 if g.get("movement_direction") is not None else 40
    return clamp(0.70 * accuracy + 0.30 * direction)

def rf_score(rf_list, radar):
    if not rf_list:
        return 0.0
    values = []
    for item in rf_list:
        strength = clamp((-item.get("signal_strength", -100) - 20) / 60 * 100)
        stability = clamp(item.get("stability", 0) * 100)
        match = clamp(100 - angle_diff(item.get("direction", 0), radar.get("angle", 0)) / 180 * 100)
        values.append(0.45 * strength + 0.30 * stability + 0.25 * match)
    return clamp(max(values))

def vision_score(v):
    return clamp(v.get("confidence", 0) * 100)

def behavior_score(r, g):
    speed = clamp(r.get("speed", 0) / 25 * 100)
    altitude = clamp(r.get("altitude", 0) / 150 * 100)
    vertical = clamp(abs(g.get("vertical_speed", 0)) / 10 * 100)
    return clamp(0.50 * speed + 0.25 * altitude + 0.25 * vertical)

def disagreement(radar, gps, rf_list, vision):
    issues = []
    if gps.get("accuracy", 999) > 15:
        issues.append("GPS LOW ACCURACY")
    if rf_list:
        best = min(angle_diff(x.get("direction", 0), radar.get("angle", 0)) for x in rf_list)
        if best > 60:
            issues.append("RADAR/RF DIRECTION MISMATCH")
    if vision.get("confidence", 0) < 0.50:
        issues.append("VISION LOW CONFIDENCE")
    if radar.get("confidence", 0) < 0.50:
        issues.append("RADAR LOW CONFIDENCE")
    return issues

def fuse(radar, gps, rf_list, vision):
    components = {
        "radar": radar_score(radar),
        "gps": gps_score(gps),
        "rf": rf_score(rf_list, radar),
        "vision": vision_score(vision),
        "behavior": behavior_score(radar, gps),
    }
    total = clamp(sum(components[k] * WEIGHTS[k] for k in components))
    issues = disagreement(radar, gps, rf_list, vision)
    best_rf_angle = min(
        (angle_diff(x.get("direction", 0), radar.get("angle", 0)) for x in rf_list),
        default=180,
    )
    confirmed = (
        radar.get("confidence", 0) >= 0.70
        and vision.get("confidence", 0) >= 0.70
        and (not rf_list or best_rf_angle <= 45)
        and gps.get("accuracy", 999) <= 15
    )
    return total, components, confirmed, issues
