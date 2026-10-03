# AEROSHIELD

Counter-UAV intelligence prototype using simulated Radar, GPS, RF, Vision, Environment and UAV telemetry.

## Run
```bash
cd ~/aerius/aeroshield
python3 main.py
```

Open:
http://127.0.0.1:8080

## Demonstration
NORMAL -> SUSPICIOUS -> HIGH RISK -> SENSOR DISAGREEMENT

The dashboard includes:
- animated 2D radar sweep
- multiple tracks
- target pulse/highlight
- predicted trajectory
- sensor-fusion scores
- movement behavior
- sensor disagreement detection
- threat classification
- decision support

The included simulator is an input adapter because the competition problem statement supplied sensor fields but no live API/socket schema. It must not be presented as the organizers' live feed.

The current system is an explainable algorithmic sensor-fusion prototype, not a trained ML model.
