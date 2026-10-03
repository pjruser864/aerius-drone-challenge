#!/usr/bin/env python3

import csv
import math
import os
import time

from pymavlink import mavutil


# ============================================================
# AERIUS LEVEL 1 - FINAL AUTONOMOUS CONTROLLER
# ============================================================

CONNECTION = "udp:127.0.0.1:14551"

TAKEOFF_ALTITUDE = 5.0

# Local-NED mission distance used for our local SITL test.
# In the official environment, change this to the supplied
# start/destination configuration if organizers provide one.
MISSION_DISTANCE = 100.0

TARGET_TOLERANCE = 2.5

CRUISE_SPEED = 2.0
SLOW_SPEED = 0.8

CONTROL_RATE = 10.0

LIDAR_FILE = "/tmp/aerius_lidar.csv"

LIDAR_MAX_RANGE = 30.0
LIDAR_MIN_RANGE = 0.20

# Conservative obstacle distances.
SLOW_DISTANCE = 8.0
AVOID_DISTANCE = 5.0
EMERGENCY_DISTANCE = 2.5

TAKEOFF_TIMEOUT = 30
MISSION_TIMEOUT = 180
LAND_TIMEOUT = 45


# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def angle_difference(a, b):
    return wrap_angle(a - b)


# ------------------------------------------------------------
# Telemetry
# ------------------------------------------------------------

class Telemetry:

    def __init__(self, vehicle):
        self.vehicle = vehicle

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.yaw = 0.0

        self.last_position_time = 0.0
        self.last_attitude_time = 0.0

    def update(self, timeout=0.05):

        msg = self.vehicle.recv_match(
            type=[
                "LOCAL_POSITION_NED",
                "ATTITUDE"
            ],
            blocking=True,
            timeout=timeout
        )

        if msg is None:
            return

        msg_type = msg.get_type()

        if msg_type == "LOCAL_POSITION_NED":

            self.x = float(msg.x)
            self.y = float(msg.y)
            self.z = float(msg.z)

            self.vx = float(msg.vx)
            self.vy = float(msg.vy)
            self.vz = float(msg.vz)

            self.last_position_time = time.time()

        elif msg_type == "ATTITUDE":

            self.yaw = float(msg.yaw)

            self.last_attitude_time = time.time()


# ------------------------------------------------------------
# LiDAR
# ------------------------------------------------------------

class Lidar:

    def __init__(self, path):

        self.path = path

        self.ranges = [
            LIDAR_MAX_RANGE
            for _ in range(16)
        ]

        self.valid = False

    def read(self):

        try:

            with open(self.path, "r") as f:

                line = f.readline().strip()

            if not line:
                return False

            values = line.split(",")

            if len(values) != 16:
                return False

            ranges = []

            for value in values:

                try:
                    r = float(value)
                except ValueError:
                    r = LIDAR_MAX_RANGE

                if not math.isfinite(r):
                    r = LIDAR_MAX_RANGE

                r = clamp(
                    r,
                    LIDAR_MIN_RANGE,
                    LIDAR_MAX_RANGE
                )

                ranges.append(r)

            self.ranges = ranges
            self.valid = True

            return True

        except Exception:
            self.valid = False
            return False


# ------------------------------------------------------------
# LiDAR geometry
#
# 16 beams:
#
# index 0  = -180°
# index 1  = -157.5°
# index 2  = -135°
# ...
# index 8  = 0°
# ...
# index 15 = +157.5°
#
# The LiDAR is mounted with zero-angle along its forward axis.
# ------------------------------------------------------------

def lidar_angle(index):

    return -math.pi + (
        index * (2.0 * math.pi / 16.0)
    )


def obstacle_clearance(lidar, direction):

    """
    Returns conservative clearance in the requested
    body-frame direction.

    direction:
        0 rad = forward
        +rad  = positive LiDAR angular direction
        -rad  = negative LiDAR angular direction
    """

    best = LIDAR_MAX_RANGE

    for i, distance in enumerate(lidar.ranges):

        beam = lidar_angle(i)

        difference = abs(
            angle_difference(beam, direction)
        )

        # Consider beams within roughly +/- 33.75 degrees
        # of the candidate direction.
        if difference <= math.radians(33.75):

            # Weight central beams more heavily by using
            # the minimum actual clearance.
            best = min(best, distance)

    return best


def forward_clearance(lidar):

    return obstacle_clearance(lidar, 0.0)


# ------------------------------------------------------------
# Navigation
# ------------------------------------------------------------

def world_to_body(vx, vy, yaw):

    """
    Convert local-NED velocity into body-frame direction.

    Body:
        x = forward
        y = right
    """

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    body_x = cy * vx + sy * vy
    body_y = -sy * vx + cy * vy

    return body_x, body_y


def body_to_world(vx, vy, yaw):

    """
    Convert body-frame velocity to local-NED velocity.
    """

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    world_x = cy * vx - sy * vy
    world_y = sy * vx + cy * vy

    return world_x, world_y


# ------------------------------------------------------------
# Candidate direction planner
# ------------------------------------------------------------

def choose_body_direction(
    lidar,
    desired_body_angle
):

    # Candidate directions around the desired direction.
    offsets_deg = [
        0,
        -22.5,
        22.5,
        -45,
        45,
        -67.5,
        67.5,
        -90,
        90,
        -112.5,
        112.5,
        -135,
        135,
        157.5,
        -157.5,
        180
    ]

    candidates = []

    for offset_deg in offsets_deg:

        angle = wrap_angle(
            desired_body_angle +
            math.radians(offset_deg)
        )

        clearance = obstacle_clearance(
            lidar,
            angle
        )

        candidates.append(
            (
                angle,
                clearance,
                abs(math.radians(offset_deg))
            )
        )

    # First attempt: directions with strong clearance.
    safe = [
        c for c in candidates
        if c[1] >= AVOID_DISTANCE
    ]

    if safe:

        # Prefer:
        # 1. highest clearance
        # 2. smallest deviation from target direction
        safe.sort(
            key=lambda c: (
                -c[1],
                c[2]
            )
        )

        # Avoid making the vehicle unnecessarily
        # deviate when the target direction is clear.
        desired_clearance = obstacle_clearance(
            lidar,
            desired_body_angle
        )

        if desired_clearance >= SLOW_DISTANCE:
            return desired_body_angle

        return safe[0][0]

    # Emergency case:
    # choose the direction with maximum clearance.
    candidates.sort(
        key=lambda c: (
            -c[1],
            c[2]
        )
    )

    return candidates[0][0]


# ------------------------------------------------------------
# MAVLink controller
# ------------------------------------------------------------

class Controller:

    def __init__(self, vehicle):

        self.vehicle = vehicle

    def set_guided(self):

        mode_id = self.vehicle.mode_mapping().get("GUIDED")

        if mode_id is None:
            raise RuntimeError("GUIDED mode unavailable")

        self.vehicle.set_mode(mode_id)

        time.sleep(1)

    def arm(self):

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0
        )

        self.vehicle.motors_armed_wait()

    def takeoff(self, altitude):

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude
        )

    def send_velocity_ned(
        self,
        vx,
        vy,
        vz
    ):

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.vehicle.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0,
            0,
            0,
            0,
            0
        )

    def land(self):

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )


# ------------------------------------------------------------
# Main mission
# ------------------------------------------------------------

def main():

    print("=" * 60)
    print(" AERIUS LEVEL 1")
    print(" FINAL LiDAR AUTONOMOUS NAVIGATION")
    print("=" * 60)

    print("\nConnecting to ArduPilot...")

    vehicle = mavutil.mavlink_connection(
        CONNECTION,
        autoreconnect=True
    )

    vehicle.wait_heartbeat()

    print(
        "Connected: system=%d component=%d"
        % (
            vehicle.target_system,
            vehicle.target_component
        )
    )

    telemetry = Telemetry(vehicle)
    lidar = Lidar(LIDAR_FILE)
    controller = Controller(vehicle)

    # --------------------------------------------------------
    # GUIDED
    # --------------------------------------------------------

    print("Setting GUIDED...")

    controller.set_guided()

    # --------------------------------------------------------
    # ARM
    # --------------------------------------------------------

    print("Arming...")

    controller.arm()

    print("ARMED")

    # --------------------------------------------------------
    # TAKEOFF
    # --------------------------------------------------------

    print(
        "Taking off to %.1f m..."
        % TAKEOFF_ALTITUDE
    )

    controller.takeoff(TAKEOFF_ALTITUDE)

    takeoff_start = time.time()

    while time.time() - takeoff_start < TAKEOFF_TIMEOUT:

        telemetry.update()

        altitude = -telemetry.z

        print(
            "ALT %.2f m"
            % altitude
        )

        if altitude >= TAKEOFF_ALTITUDE * 0.85:

            print("Takeoff complete.")

            break

        time.sleep(0.2)

    else:

        print("ERROR: Takeoff timeout.")
        controller.land()
        return

    # --------------------------------------------------------
    # Mission target
    # --------------------------------------------------------

    start_x = telemetry.x
    start_y = telemetry.y

    target_x = start_x + MISSION_DISTANCE
    target_y = start_y

    print()
    print("MISSION")
    print(
        "Start : %.2f %.2f"
        % (start_x, start_y)
    )
    print(
        "Target: %.2f %.2f"
        % (target_x, target_y)
    )
    print()

    # --------------------------------------------------------
    # Autonomous navigation
    # --------------------------------------------------------

    mission_start = time.time()

    last_print = 0

    while time.time() - mission_start < MISSION_TIMEOUT:

        telemetry.update()

        lidar.read()

        dx = target_x - telemetry.x
        dy = target_y - telemetry.y

        distance_to_target = math.hypot(
            dx,
            dy
        )

        if distance_to_target <= TARGET_TOLERANCE:

            print()
            print("=" * 60)
            print("DESTINATION REACHED")
            print("=" * 60)

            controller.send_velocity_ned(
                0,
                0,
                0
            )

            time.sleep(1)

            print("Landing...")

            controller.land()

            time.sleep(LAND_TIMEOUT)

            print("MISSION COMPLETE")

            return

        # ----------------------------------------------------
        # Desired direction in local NED
        # ----------------------------------------------------

        desired_world_angle = math.atan2(
            dy,
            dx
        )

        # Convert desired NED direction to body frame.
        desired_body_x, desired_body_y = world_to_body(
            math.cos(desired_world_angle),
            math.sin(desired_world_angle),
            telemetry.yaw
        )

        desired_body_angle = math.atan2(
            desired_body_y,
            desired_body_x
        )

        # ----------------------------------------------------
        # LiDAR avoidance
        # ----------------------------------------------------

        front = forward_clearance(lidar)

        selected_body_angle = choose_body_direction(
            lidar,
            desired_body_angle
        )

        selected_world_x, selected_world_y = body_to_world(
            math.cos(selected_body_angle),
            math.sin(selected_body_angle),
            telemetry.yaw
        )

        # ----------------------------------------------------
        # Speed control
        # ----------------------------------------------------

        if front <= EMERGENCY_DISTANCE:

            speed = SLOW_SPEED

        elif front <= AVOID_DISTANCE:

            speed = SLOW_SPEED

        elif front <= SLOW_DISTANCE:

            # Smooth speed reduction.
            ratio = (
                front - AVOID_DISTANCE
            ) / (
                SLOW_DISTANCE - AVOID_DISTANCE
            )

            ratio = clamp(
                ratio,
                0.0,
                1.0
            )

            speed = (
                SLOW_SPEED
                +
                (CRUISE_SPEED - SLOW_SPEED) * ratio
            )

        else:

            speed = CRUISE_SPEED

        # ----------------------------------------------------
        # Velocity command
        # ----------------------------------------------------

        vx = selected_world_x * speed
        vy = selected_world_y * speed

        controller.send_velocity_ned(
            vx,
            vy,
            0
        )

        # ----------------------------------------------------
        # Status
        # ----------------------------------------------------

        if time.time() - last_print >= 0.5:

            min_range = min(lidar.ranges)

            if min_range >= LIDAR_MAX_RANGE:
                min_text = "CLEAR"
            else:
                min_text = "%.2f m" % min_range

            print(
                "DIST %.1f m | "
                "LiDAR %s | "
                "FRONT %.2f m | "
                "SPD %.2f | "
                "BODY %.1f°"
                % (
                    distance_to_target,
                    min_text,
                    front,
                    speed,
                    math.degrees(selected_body_angle)
                )
            )

            last_print = time.time()

        time.sleep(
            1.0 / CONTROL_RATE
        )

    # --------------------------------------------------------
    # Timeout safety
    # --------------------------------------------------------

    print()
    print("=" * 60)
    print("MISSION TIMEOUT")
    print("Landing for safety.")
    print("=" * 60)

    controller.send_velocity_ned(
        0,
        0,
        0
    )

    time.sleep(1)

    controller.land()


if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:

        print("\nInterrupted. Landing...")

    except Exception as e:

        print("\nERROR:", e)

    finally:

        print("Controller terminated.")
