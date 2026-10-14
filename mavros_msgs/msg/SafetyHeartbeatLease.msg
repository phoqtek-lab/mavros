# Companion supervisor liveness lease. This gates the MAVROS GCS heartbeat;
# it never grants mode, source, arming, or navigation authority.
std_msgs/Header header
uint8[16] supervisor_boot_id
uint64 sequence
builtin_interfaces/Duration ttl
bool command_loop_healthy
