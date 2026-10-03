from pymavlink import mavutil


class DroneTelemetry:

    def __init__(self, connection):
        self.connection = connection

    def get_local_state(self):
        """
        Return the latest LOCAL_POSITION_NED state.

        x: north
        y: east
        z: down

        Velocity uses the same NED convention.
        """

        msg = self.connection.recv_match(
            type="LOCAL_POSITION_NED",
            blocking=True,
            timeout=2,
        )

        if msg is None:
            return None

        return {
            "position": (
                float(msg.x),
                float(msg.y),
                float(msg.z),
            ),

            "velocity": (
                float(msg.vx),
                float(msg.vy),
                float(msg.vz),
            ),
        }

    def get_position(self):

        state = self.get_local_state()

        if state is None:
            return None

        return state["position"]

    def get_velocity(self):

        state = self.get_local_state()

        if state is None:
            return None

        return state["velocity"]
