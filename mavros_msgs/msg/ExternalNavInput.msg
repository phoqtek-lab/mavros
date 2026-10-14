# Safety-profile external navigation input. This keeps reset and quality
# provenance alongside nav_msgs/Odometry before MAVLink ODOMETRY conversion.
nav_msgs/Odometry odometry
uint8 reset_counter
int8 quality
uint64 producer_epoch
# Lineage is explicit so a delayed sample from a previous supervisor/VIO
# instance cannot be mistaken for the current producer after a restart.
uint8[16] supervisor_boot_id
uint64 permit_revision
uint64 vio_epoch
bool producer_reset
