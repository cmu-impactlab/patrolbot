std_msgs/Header header

# Serial link to the SimpleBGC controller is up and framing correctly.
bool connected
bool motors_on

# Ground truth from the board's IMU, in degrees. These are what the dashboard
# should display; they are not dead reckoning.
float32 imu_pan
float32 imu_tilt
float32 imu_roll

# What the board is currently trying to reach, in degrees.
float32 target_pan
float32 target_tilt
float32 target_roll

# Travel limits enforced by the driver, in degrees. Published so a client can
# render how much travel is left without hardcoding the mount's geometry.
float32 pan_min
float32 pan_max
float32 tilt_min
float32 tilt_max
float32 roll_min
float32 roll_max

# True while the driver is holding because no valid command arrived in time.
bool command_timeout

# Empty when healthy; a short human-readable reason otherwise.
string fault
