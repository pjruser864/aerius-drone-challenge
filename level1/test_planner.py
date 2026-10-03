from planner import plan_direction


drone_position = (0.0, 0.0)
drone_velocity = (5.0, 0.0)

target_position = (100.0, 0.0)

hazards = [
    {
        "position": (40.0, 0.0),
        "velocity": (-10.0, 0.0),
    }
]

result = plan_direction(
    drone_position=drone_position,
    drone_velocity=drone_velocity,
    target_position=target_position,
    hazards=hazards,
    safety_distance=15.0,
    prediction_time=3.0,
)

print("Chosen direction:", result["direction"])

print(
    "Dangerous hazards:",
    len(result["dangerous_hazards"])
)
