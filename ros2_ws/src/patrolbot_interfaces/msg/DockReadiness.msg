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

# Undock readiness and docking inhibition are separate facts with separate
# causes, so they get separate fields. After a successful undock,
# ready_for_undock is false because the robot is already clear, while
# automatic_docking_allowed is false because the redock inhibition is running.
# Redock inhibition never blocks undocking.
string dock_state
bool physically_clear
bool automatic_docking_allowed
bool redock_inhibited
float32 redock_inhibit_remaining
bool undock_profile_commissioned
