import os
import sys
import math

# Add the parent level1 directory to Python's import path.
LEVEL1_DIR = os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))
)

if LEVEL1_DIR not in sys.path:
    sys.path.insert(0, LEVEL1_DIR)

from planner import plan_direction


SAFETY_DISTANCE = 15.0
PREDICTION_TIME = 4.0
CRUISE_SPEED = 2.0


def direction_angle(direction):
    return math.degrees(
        math.atan2(direction[1], direction[0])
    )


def run_test(
    name,
    drone_position,
    drone_velocity,
    target,
    hazards,
):
    print()
    print("=" * 60)
    print(name)
    print("=" * 60)

    result = plan_direction(
        drone_position=drone_position,
        drone_velocity=drone_velocity,
        target_position=target,
        hazards=hazards,
        safety_distance=SAFETY_DISTANCE,
        prediction_time=PREDICTION_TIME,
        cruise_speed=CRUISE_SPEED,
    )

    direction = result["direction"]

    print(
        f"Direction              : "
        f"({direction[0]:.3f}, {direction[1]:.3f})"
    )

    print(
        f"Direction angle        : "
        f"{direction_angle(direction):.1f} deg"
    )

    print(
        f"Emergency              : "
        f"{result['emergency']}"
    )

    print(
        f"Safe candidate         : "
        f"{result['safe_candidate']}"
    )

    print(
        f"Predicted minimum sep. : "
        f"{result['predicted_minimum_separation']:.2f} m"
    )

    assert math.isfinite(direction[0])
    assert math.isfinite(direction[1])

    magnitude = math.hypot(
        direction[0],
        direction[1],
    )

    assert abs(magnitude - 1.0) < 1e-6

    return result


def test_1_no_hazards():

    result = run_test(
        "TEST 1 — No hazards",

        drone_position=(0.0, 0.0),

        drone_velocity=(0.0, 0.0),

        target=(100.0, 0.0),

        hazards=[],
    )

    direction = result["direction"]

    assert direction[0] > 0.99
    assert abs(direction[1]) < 0.05

    print("PASS")


def test_2_hazard_from_front():

    result = run_test(
        "TEST 2 — Hazard approaching from front",

        drone_position=(0.0, 0.0),

        drone_velocity=(0.0, 0.0),

        target=(100.0, 0.0),

        hazards=[
            {
                "position": (30.0, 0.0),
                "velocity": (-10.0, 0.0),
            }
        ],
    )

    direction = result["direction"]

    assert abs(direction[1]) > 0.05

    print("PASS")


def test_3_hazard_from_upper_front():

    result = run_test(
        "TEST 3 — Hazard approaching from upper-front",

        drone_position=(0.0, 0.0),

        drone_velocity=(0.0, 0.0),

        target=(100.0, 0.0),

        hazards=[
            {
                "position": (35.0, 20.0),
                "velocity": (-10.0, -4.0),
            }
        ],
    )

    direction = result["direction"]

    assert math.isfinite(direction[0])
    assert math.isfinite(direction[1])

    print("PASS")


def test_4_multiple_hazards():

    result = run_test(
        "TEST 4 — Multiple hazards",

        drone_position=(0.0, 0.0),

        drone_velocity=(0.0, 0.0),

        target=(100.0, 0.0),

        hazards=[
            {
                "position": (35.0, 15.0),
                "velocity": (-10.0, -3.0),
            },
            {
                "position": (35.0, -15.0),
                "velocity": (-10.0, 3.0),
            },
            {
                "position": (55.0, 25.0),
                "velocity": (-8.0, -4.0),
            },
        ],
    )

    direction = result["direction"]

    assert math.isfinite(direction[0])
    assert math.isfinite(direction[1])

    print("PASS")


def test_5_hazard_behind():

    result = run_test(
        "TEST 5 — Hazard behind drone",

        drone_position=(0.0, 0.0),

        drone_velocity=(0.0, 0.0),

        target=(100.0, 0.0),

        hazards=[
            {
                "position": (-30.0, 0.0),
                "velocity": (-10.0, 0.0),
            }
        ],
    )

    direction = result["direction"]

    assert direction[0] > 0.5

    print("PASS")


def test_6_hazard_far_away():

    result = run_test(
        "TEST 6 — Far-away hazard",

        drone_position=(0.0, 0.0),

        drone_velocity=(0.0, 0.0),

        target=(100.0, 0.0),

        hazards=[
            {
                "position": (200.0, 100.0),
                "velocity": (-10.0, 0.0),
            }
        ],
    )

    direction = result["direction"]

    assert direction[0] > 0.9

    print("PASS")


def main():

    print()
    print("AERIUS LEVEL 1 — PLANNER VALIDATION")
    print()

    test_1_no_hazards()

    test_2_hazard_from_front()

    test_3_hazard_from_upper_front()

    test_4_multiple_hazards()

    test_5_hazard_behind()

    test_6_hazard_far_away()

    print()
    print("=" * 60)
    print("ALL PLANNER TESTS PASSED")
    print("=" * 60)


if __name__ == "__main__":
    main()
