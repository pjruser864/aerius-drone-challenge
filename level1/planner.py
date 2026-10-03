import math

from navigation import direction_to_target
from prediction import closest_approach


def normalize(x, y):
    magnitude = math.hypot(x, y)

    if magnitude < 1e-9:
        return 0.0, 0.0

    return x / magnitude, y / magnitude


def angle_of(x, y):
    return math.atan2(y, x)


def vector_from_angle(angle):
    return math.cos(angle), math.sin(angle)


def angular_difference(a, b):
    difference = a - b

    while difference > math.pi:
        difference -= 2.0 * math.pi

    while difference < -math.pi:
        difference += 2.0 * math.pi

    return abs(difference)


def evaluate_candidate(
    drone_position,
    drone_velocity,
    candidate_direction,
    cruise_speed,
    hazards,
    safety_distance,
    horizon,
):
    """
    Evaluate one possible horizontal flight direction.

    The candidate velocity is used to predict where the drone would
    travel if it continued in this direction at cruise speed.
    """

    candidate_velocity = (
        candidate_direction[0] * cruise_speed,
        candidate_direction[1] * cruise_speed,
    )

    minimum_separation = float("inf")

    total_risk = 0.0

    safe = True

    for hazard in hazards:

        result = closest_approach(
            drone_position=drone_position,
            drone_velocity=candidate_velocity,
            hazard_position=hazard["position"],
            hazard_velocity=hazard["velocity"],
            horizon=horizon,
        )

        separation = result["separation"]

        minimum_separation = min(
            minimum_separation,
            separation,
        )

        if separation < safety_distance:
            safe = False

            ratio = (
                (safety_distance - separation)
                / safety_distance
            )

            total_risk += ratio * ratio

    return {
        "safe": safe,
        "minimum_separation": minimum_separation,
        "risk": total_risk,
    }


def generate_candidate_angles(target_angle):
    """
    Generate possible flight directions around the target direction.

    The target direction itself is included.

    Additional directions allow the drone to move around incoming
    hazards instead of only attempting to fly directly toward the
    destination.
    """

    candidate_angles = []

    # Fine search around the target direction.
    for degrees in range(-90, 91, 10):

        angle = target_angle + math.radians(degrees)

        candidate_angles.append(angle)

    # Explicit full lateral directions.
    candidate_angles.append(
        target_angle + math.pi / 2.0
    )

    candidate_angles.append(
        target_angle - math.pi / 2.0
    )

    return candidate_angles


def plan_direction(
    drone_position,
    drone_velocity,
    target_position,
    hazards,
    safety_distance,
    prediction_time,
    cruise_speed=2.0,
):
    """
    Select the safest useful direction toward the target.

    Decision hierarchy:

    1. Prefer candidates that remain outside the safety distance.
    2. Among safe candidates, prefer the direction closest to the
       target direction.
    3. If no safe candidate exists, choose the candidate providing
       the greatest predicted separation.
    """

    target_dx, target_dy = direction_to_target(
        drone_position[0],
        drone_position[1],
        target_position[0],
        target_position[1],
    )

    target_angle = angle_of(
        target_dx,
        target_dy,
    )

    candidate_angles = generate_candidate_angles(
        target_angle
    )

    evaluated = []

    for angle in candidate_angles:

        direction = vector_from_angle(angle)

        result = evaluate_candidate(
            drone_position=drone_position,
            drone_velocity=drone_velocity,
            candidate_direction=direction,
            cruise_speed=cruise_speed,
            hazards=hazards,
            safety_distance=safety_distance,
            horizon=prediction_time,
        )

        direction_angle_difference = angular_difference(
            angle,
            target_angle,
        )

        evaluated.append(
            {
                "direction": direction,
                "safe": result["safe"],
                "minimum_separation": (
                    result["minimum_separation"]
                ),
                "risk": result["risk"],
                "angle_difference": (
                    direction_angle_difference
                ),
            }
        )

    # ---------------------------------------------------------
    # CASE 1:
    # At least one completely safe candidate exists.
    #
    # PRIMARY OBJECTIVE:
    # Stay safe.
    #
    # SECONDARY OBJECTIVE:
    # Stay as close as possible to the destination direction.
    #
    # This prevents harmless distant hazards from causing
    # unnecessary 90-degree turns.
    # ---------------------------------------------------------

    safe_candidates = [
        candidate
        for candidate in evaluated
        if candidate["safe"]
    ]

    if safe_candidates:

        safe_candidates.sort(
            key=lambda candidate: (
                candidate["angle_difference"],
                -candidate["minimum_separation"],
            )
        )

        best = safe_candidates[0]

        return {
            "direction": best["direction"],
            "dangerous_hazards": hazards,
            "emergency": False,
            "safe_candidate": True,
            "predicted_minimum_separation": (
                best["minimum_separation"]
            ),
        }

    # ---------------------------------------------------------
    # CASE 2:
    # No completely safe candidate exists.
    #
    # This means that under the current prediction model and
    # candidate velocities, every tested path enters the
    # development safety radius.
    #
    # In this situation maximize separation first.
    # ---------------------------------------------------------

    evaluated.sort(
        key=lambda candidate: (
            -candidate["minimum_separation"],
            candidate["risk"],
            candidate["angle_difference"],
        )
    )

    best = evaluated[0]

    return {
        "direction": best["direction"],
        "dangerous_hazards": hazards,
        "emergency": True,
        "safe_candidate": False,
        "predicted_minimum_separation": (
            best["minimum_separation"]
        ),
    }
