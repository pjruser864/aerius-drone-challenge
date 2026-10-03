import math


def closest_approach(
    drone_position,
    drone_velocity,
    hazard_position,
    hazard_velocity,
    prediction_time=3.0,
):
    """
    Calculate the predicted minimum separation between
    drone and hazard during prediction_time.
    """

    rx = hazard_position[0] - drone_position[0]
    ry = hazard_position[1] - drone_position[1]

    rvx = hazard_velocity[0] - drone_velocity[0]
    rvy = hazard_velocity[1] - drone_velocity[1]

    relative_speed_squared = rvx * rvx + rvy * rvy

    if relative_speed_squared < 1e-6:
        t_closest = 0.0
    else:
        t_closest = -(
            rx * rvx + ry * rvy
        ) / relative_speed_squared

        t_closest = max(
            0.0,
            min(t_closest, prediction_time)
        )

    closest_x = rx + rvx * t_closest
    closest_y = ry + rvy * t_closest

    separation = math.hypot(
        closest_x,
        closest_y
    )

    return t_closest, separation


def avoidance_direction(
    drone_position,
    hazard_position,
):
    """
    Produce a lateral direction around the hazard.
    """

    dx = drone_position[0] - hazard_position[0]
    dy = drone_position[1] - hazard_position[1]

    distance = math.hypot(dx, dy)

    if distance < 1e-6:
        return 0.0, 1.0

    # Perpendicular direction.
    return (
        -dy / distance,
        dx / distance,
    )


def hazard_is_dangerous(
    drone_position,
    drone_velocity,
    hazard_position,
    hazard_velocity,
    safety_distance,
    prediction_time,
):
    """
    Determine whether predicted closest approach
    violates the safety distance.
    """

    time_to_closest, predicted_separation = closest_approach(
        drone_position,
        drone_velocity,
        hazard_position,
        hazard_velocity,
        prediction_time,
    )

    return {
        "dangerous": predicted_separation < safety_distance,
        "time_to_closest": time_to_closest,
        "predicted_separation": predicted_separation,
    }
