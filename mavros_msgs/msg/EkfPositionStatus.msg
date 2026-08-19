# Bounded EKF horizontal-position status.
#
# Derived from the ArduPilot-specific EKF_STATUS_REPORT MAVLink message
# (ardupilotmega dialect, https://mavlink.io/en/messages/ardupilotmega.html).
# Exposes only the flags needed to gate a vehicle-neutral position-source
# health projection; the full EKF_STATUS_REPORT flag/variance surface is not
# published here.

std_msgs/Header header

# True if the EKF's absolute horizontal position estimate is good
# (EKF_STATUS_FLAGS::POS_HORIZ_ABS).
bool pos_horiz_abs

# True if the EKF's relative horizontal position estimate is good
# (EKF_STATUS_FLAGS::POS_HORIZ_REL).
bool pos_horiz_rel

# True if the EKF has never been healthy (EKF_STATUS_FLAGS::UNINITIALIZED).
bool uninitialized
