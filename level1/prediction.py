import math


def relative_motion(
    drone_position,
    drone_velocity,
    hazard_position,
    hazard_velocity,
):
    """
    Calculate relative position and velocity.

    All values use the same local coordinate system.
    """

    relative_position = (
        hazard_position[0] - drone_position[0],
        hazard_position[1] - drone_position[1],
    )

    relative_velocity = (
        hazard_velocity[0] - drone_velocity[0],
        hazard_velocity[1] - drone_velocity[1],
    )

    return (
        relative_position,
        relative_velocity,
    )


def closest_approach(
    drone_position,
    drone_velocity,
    hazard_position,
    hazard_velocity,
    horizon,
):
    """
    Calculate predicted closest separation within horizon.
    """

    relative_position, relative_velocity = (
        relative_motion(
            drone_position,
            drone_velocity,
            hazard_position,
            hazard_velocity,
        )
    )

    rx, ry = relative_position
    vx, vy = relative_velocity

    speed_squared = (
        vx * vx
        + vy * vy
    )

    if speed_squared < 1e-9:

        time = 0.0

    else:

        time = -(
            rx * vx
            + ry * vy
        ) / speed_squared

        time = max(
            0.0,
            min(time, horizon),
        )

    closest_x = (
        rx + vx * time
    )

    closest_y = (
        ry + vy * time
    )

    separation = math.hypot(
        closest_x,
        closest_y,
    )

    return {
        "time": time,
        "separation": separation,
    }


def predicted_position(
    position,
    velocity,
    time,
):
    """
    Predict position assuming constant velocity.
    """

    return (
        position[0] + velocity[0] * time,
        position[1] + velocity[1] * time,
    )
