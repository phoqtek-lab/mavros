# Validated read-only source witness decoded by the safety supervisor.
std_msgs/Header header
uint8 schema_version
uint8 source_set
uint8 gps_index
uint16 witness_sequence
uint32 time_boot_ms
uint8 source_system
uint8 source_component
uint64 receive_sequence
uint64 connection_epoch
bool known
uint64 reason_mask
# FC Lua witness says source set 1 has no configured ExternalNav velocity
# dependency. This is configuration provenance only; GPS receiver identity and
# provenance are qualified separately by the supervisor.
bool source1_ekf_independent
# FC-side GPS timestamp advanced recently in the unextrapolated GPS week/time-
# of-week API. GPS_RAW_INT.time_usec is not sufficient: ArduPilot's MAV GPS
# backend may jitter-correct/extrapolate it while the input measurement repeats.
bool gps_measurement_fresh
