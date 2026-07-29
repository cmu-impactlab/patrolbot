std_msgs/Header header

# Increments whenever the Pi bridge establishes a new TCP hardware session.
# There is exactly one such counter: enabling the motors does NOT power-cycle
# the base link (verified against the SBC journal for 2026-07-28, which shows a
# single TCP session across thirteen MOTOR_ENABLE operations), so there is no
# separate base-power generation to track.
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

# Persistent dock observer, owned by the SBC. Ordinary motion is permitted only
# in CLEAR_CONFIRMED. charge_state above is evidence feeding this state, never
# an interlock in its own right, so a stale re-asserted charge level after a
# proven departure no longer blocks navigation.
# UNKNOWN | DOCKED_CONFIRMED | DEPARTING | CLEAR_CONFIRMED | FAULT_STOPPED
string dock_state
bool dock_state_valid
# Active undock phase; 0/IDLE when no operation is running.
uint8 dock_phase
string dock_phase_name
bool undock_active
uint16 undock_release_attempts
float32 minimum_rear_range
bool rear_sonar_usable

# Auto-docking inhibition, published so higher-level battery or mission logic
# cannot immediately redock the robot. This is advisory state from the SBC: the
# SBC has no autonomous dock operation, so the Pi dock coordinator is what
# actually enforces it. It never gates undocking.
bool redock_inhibited
float32 redock_inhibit_remaining
# False until the SBC's rear-sonar indices, E-stop semantics and fault masks
# have been physically validated. Production undocking is refused while false.
bool undock_profile_commissioned
