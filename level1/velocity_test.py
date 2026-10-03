from pymavlink import mavutil
import time

CONNECTION = "udp:127.0.0.1:14551"

master = mavutil.mavlink_connection(CONNECTION)

print("Waiting for heartbeat...")
master.wait_heartbeat()

print("Connected.")

print("Setting GUIDED mode...")
master.set_mode_apm("GUIDED")
time.sleep(2)

print("Arming...")
master.arducopter_arm()
master.motors_armed_wait()

print("Armed.")

print("Taking off to 5 m...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0, 5
)

time.sleep(8)

print("Sending forward velocity...")

for _ in range(20):

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        # Velocity only.
        0b0000111111000111,

        0, 0, 0,

        2.0,   # vx = 2 m/s
        0.0,   # vy
        0.0,   # vz

        0, 0, 0,
        0, 0
    )

    time.sleep(0.5)

print("Stopping.")

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,

    0b0000111111000111,

    0, 0, 0,

    0, 0, 0,

    0, 0, 0,
    0, 0
)

time.sleep(2)

print("Landing...")

master.set_mode_apm("LAND")

master.motors_disarmed_wait()

print("Velocity test complete.")
