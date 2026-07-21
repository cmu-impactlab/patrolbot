std_msgs/Header header

# Increments whenever the Pi bridge establishes a new TCP hardware session.
uint64 session_generation
bool link_connected
float32 telemetry_age

bool hardware_state_valid
int16 charge_state
bool charge_state_valid
bool motors_enabled
bool estop_pressed
uint32 flags
uint32 fault_flags
uint32 stall_value

# False until every physical bumper segment has been mapped and configured.
bool bumpers_valid
bool front_bumper_pressed
bool rear_bumper_pressed
