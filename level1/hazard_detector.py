"""
AERIUS LEVEL 1 HAZARD DETECTOR

This module deliberately separates the navigation system from
the official hazard-data interface.

The rulebook specifies that virtual hazards approach the drone
from different directions at approximately 10 m/s, but it does
not specify the exact MAVLink message, sensor API, plugin,
topic, or data structure used to expose those hazards.

Therefore this class is the integration point for the official
Gazebo environment.

DO NOT invent an official hazard interface until the organizers'
environment is available.
"""


class HazardDetector:

    def __init__(self, connection):

        self.connection = connection

    def get_hazards(self):

        """
        Return currently detected hazards.

        Expected format:

        [
            {
                "position": (x, y),
                "velocity": (vx, vy),
            },

            ...
        ]

        Position and velocity must use the same local coordinate
        convention as the drone telemetry.

        IMPORTANT:
        This base implementation intentionally returns no hazards.

        It is NOT the final competition detector.

        The method must be connected to the organizers'
        actual Gazebo/MAVLink hazard interface once provided.
        """

        return []
