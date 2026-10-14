# Read-only projection of ArduPilot EKF_STATUS_REPORT. Missing fields are
# represented explicitly; this message is not a source-selection command.
std_msgs/Header header
uint16 flags
float32 velocity_variance
float32 pos_horiz_variance
float32 pos_vert_variance
float32 compass_variance
float32 terrain_alt_variance
float32 airspeed_variance
bool variances_valid
uint32 time_boot_ms
uint8 source_system
uint8 source_component
uint64 receive_sequence
uint64 connection_epoch
