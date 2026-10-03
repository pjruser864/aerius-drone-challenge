from avoidance import hazard_is_dangerous


# Drone moving toward +X
drone_position = (0.0, 0.0)
drone_velocity = (5.0, 0.0)

# Hazard approaching from +X
hazard_position = (40.0, 0.0)
hazard_velocity = (-10.0, 0.0)

result = hazard_is_dangerous(
    drone_position,
    drone_velocity,
    hazard_position,
    hazard_velocity,
    safety_distance=15.0,
    prediction_time=3.0,
)

print("Dangerous:", result["dangerous"])
print(
    "Time to closest approach:",
    result["time_to_closest"]
)
print(
    "Predicted separation:",
    result["predicted_separation"]
)
