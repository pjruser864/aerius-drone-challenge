import math

SCENARIOS = ["NORMAL", "SUSPICIOUS", "HIGH RISK", "SENSOR DISAGREEMENT"]

def make(t, scenario):
    presets = {
        "NORMAL": (520, 7, 0.72, 95),
        "SUSPICIOUS": (380, 11, 0.82, 110),
        "HIGH RISK": (160, 18, 0.95, 55),
        "SENSOR DISAGREEMENT": (250, 14, 0.86, 120),
    }
    dist, speed, confidence, heading = presets[scenario]
    dist = max(70, dist - t * (1.2 if scenario == "NORMAL" else 2.8))
    speed += 1.5 * math.sin(t / 4)
    heading_now = heading + 3 * math.sin(t / 6)

    radar = {
        "track_id": 7,
        "distance": dist,
        "angle": heading,
        "altitude": 80 + 5 * math.sin(t / 5),
        "speed": speed,
        "heading": heading_now,
        "confidence": confidence,
    }

    gps_accuracy = 5 if scenario != "SENSOR DISAGREEMENT" else 24
    gps = {
        "latitude": 28.613 + 0.00003 * t,
        "longitude": 77.209 + 0.00004 * t,
        "altitude": radar["altitude"],
        "accuracy": gps_accuracy,
        "movement_direction": heading,
        "vertical_speed": 1.2 * math.sin(t / 4),
    }

    rf_direction = heading if scenario != "SENSOR DISAGREEMENT" else heading + 105
    rf = [
        {
            "frequency": "2.4 GHz",
            "signal_strength": -42 if scenario != "NORMAL" else -72,
            "direction": rf_direction,
            "stability": 0.88 if scenario != "NORMAL" else 0.35,
        },
        {
            "frequency": "5.8 GHz",
            "signal_strength": -67,
            "direction": 80,
            "stability": 0.45,
        },
    ]

    vision_conf = (
        0.93 if scenario in ("HIGH RISK", "SUSPICIOUS")
        else 0.65 if scenario == "NORMAL"
        else 0.38
    )

    vision = {
        "object_type": "UAV / AIRBORNE OBJECT",
        "confidence": vision_conf,
        "position": {"x": 240, "y": 180},
    }

    secondary = {
        "track_id": 12,
        "distance": 300 + 20 * math.sin(t / 5),
        "angle": 250 + 8 * math.sin(t / 6),
        "altitude": 60,
        "speed": 6.5,
        "heading": 250,
        "confidence": 0.63,
    }

    telemetry = {
        "battery": max(20, 82 - 0.04 * t),
        "velocity": speed,
        "vertical_speed": gps["vertical_speed"],
        "flight_time": int(t),
        "motion_status": "AIRBORNE / MOVING",
    }

    environment = {
        "wind_speed": 12,
        "wind_direction": 240,
        "visibility": 70,
        "temperature": 31,
        "humidity": 60,
    }

    return {
        "tracks": [radar, secondary],
        "radar": radar,
        "gps": gps,
        "rf": rf,
        "vision": vision,
        "telemetry": telemetry,
        "environment": environment,
    }
