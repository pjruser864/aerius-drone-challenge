"""
AERIUS LEVEL 1 CONFIGURATION

Important:
The values below are DEVELOPMENT/TEST values unless explicitly
specified by the competition rulebook.

The Aerius rulebook specifies approximately 100 m altitude and
approximately 10 m/s hazard speed, but does not specify a numerical
safe-separation distance or exact destination coordinates.
"""

# ------------------------------------------------------------
# MAVLink
# ------------------------------------------------------------

CONNECTION = "udp:127.0.0.1:14551"


# ------------------------------------------------------------
# DEVELOPMENT TEST SETTINGS
# ------------------------------------------------------------

# Use 5 m locally so that tests are easy to run in SITL.
# Official mission altitude is approximately 100 m.
TEST_ALTITUDE = 5.0

# Local test mission distance.
TEST_MISSION_DISTANCE = 100.0


# ------------------------------------------------------------
# NAVIGATION
# ------------------------------------------------------------

TARGET_TOLERANCE = 3.0

CRUISE_SPEED = 2.0


# ------------------------------------------------------------
# PREDICTION
# ------------------------------------------------------------

PREDICTION_HORIZON = 4.0

PREDICTION_STEP = 0.25


# ------------------------------------------------------------
# SAFETY
# ------------------------------------------------------------

# DEVELOPMENT VALUE ONLY.
#
# The competition rulebook does NOT specify a numerical
# safe-separation distance.
SAFETY_DISTANCE = 15.0


# ------------------------------------------------------------
# MISSION
# ------------------------------------------------------------

MISSION_TIMEOUT = 120.0
