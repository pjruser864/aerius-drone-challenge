import math
from config import PREDICTION_HORIZON

def compass(degrees):
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return dirs[int((float(degrees) + 22.5) // 45) % 8]

def predict(radar, gps):
    heading = math.radians(radar.get("heading", 0))
    speed = max(0.0, float(radar.get("speed", 0)))
    horizon = PREDICTION_HORIZON

    north_m = speed * math.cos(heading) * horizon
    east_m = speed * math.sin(heading) * horizon
    lat = float(gps.get("latitude", 0))
    lon = float(gps.get("longitude", 0))
    cos_lat = max(0.2, math.cos(math.radians(lat)))

    return {
        "north_m": north_m,
        "east_m": east_m,
        "predicted_latitude": lat + north_m / 111320,
        "predicted_longitude": lon + east_m / (111320 * cos_lat),
        "horizon": horizon,
        "direction": compass(radar.get("heading", 0)),
    }

def motion(history, radar):
    if not history:
        return "STABLE"
    previous = history[-1]
    old_d = previous.get("distance", radar.get("distance", 0))
    new_d = radar.get("distance", old_d)
    delta = new_d - old_d
    if delta < -2:
        return "APPROACHING"
    if delta > 2:
        return "MOVING AWAY"
    return "STABLE"
