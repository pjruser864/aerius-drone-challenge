from pymavlink import mavutil


class DroneController:

    def __init__(self, connection):

        self.connection = connection

    def set_velocity(
        self,
        vx,
        vy,
        vz=0.0,
    ):
        """
        Set local-NED velocity.

        vx = North
        vy = East
        vz = Down
        """

        type_mask = (
            0b0000111111000111
        )

        self.connection.mav.set_position_target_local_ned_send(

            0,

            self.connection.target_system,

            self.connection.target_component,

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
            0,
        )

    def stop(self):

        self.set_velocity(
            0.0,
            0.0,
            0.0,
        )
