import time

from planner import plan_direction


drone_position = [0.0, 0.0]
drone_velocity = [5.0, 0.0]

target_position = (100.0, 0.0)

hazards = [
    {
        "position": [40.0, 0.0],
        "velocity": [-10.0, 0.0],
    }
]

dt = 0.2

for step in range(100):

    result = plan_direction(
        drone_position=tuple(drone_position),
        drone_velocity=tuple(drone_velocity),
        target_position=target_position,
        hazards=hazards,
        safety_distance=15.0,
        prediction_time=3.0,
    )

    direction_x, direction_y = result["direction"]

    speed = 5.0

    drone_velocity[0] = direction_x * speed
    drone_velocity[1] = direction_y * speed

    drone_position[0] += drone_velocity[0] * dt
    drone_position[1] += drone_velocity[1] * dt

    for hazard in hazards:
        hazard["position"][0] += hazard["velocity"][0] * dt
        hazard["position"][1] += hazard["velocity"][1] * dt

    print(
        f"t={step * dt:5.1f}s | "
        f"drone=({drone_position[0]:6.1f}, "
        f"{drone_position[1]:6.1f}) | "
        f"hazard=({hazards[0]['position'][0]:6.1f}, "
        f"{hazards[0]['position'][1]:6.1f}) | "
        f"dangerous={bool(result['dangerous_hazards'])}"
    )

    time.sleep(0.05)
