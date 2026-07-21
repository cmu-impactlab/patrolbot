std_msgs/Header header

bool ready_for_undock
bool link_connected
bool hardware_state_valid
bool charge_state_valid
bool motors_enabled
bool estop_pressed
bool bumpers_valid
bool sonar_fresh
bool rear_clear
bool odom_fresh
bool stationary
bool localization_fresh
bool localization_valid
bool dock_pose_valid
bool command_path_valid
bool navigation_idle

uint64 session_generation
int16 charge_state
uint32 fault_flags
float32 base_state_age
float32 sonar_age
float32 odom_age
float32 localization_age
string manager_state
string blocking_reason
