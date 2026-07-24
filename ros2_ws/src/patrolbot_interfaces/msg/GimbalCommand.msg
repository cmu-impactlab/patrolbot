std_msgs/Header header

# Camera gimbal command. Deliberately not a geometry_msgs/Twist: this message
# can never be mistaken for, or routed into, the robot velocity chain.

uint8 MODE_HOLD=0
uint8 MODE_RATE=1
uint8 MODE_ANGLE=2

uint8 mode

# In MODE_RATE these are degrees/second; in MODE_ANGLE they are degrees.
# Ignored entirely in MODE_HOLD.
float32 pan
float32 tilt
float32 roll

# False means "treat this as a hold regardless of the values above". This is
# where the deadman lives: a publisher that loses its operator keeps sending
# invalid commands rather than going silent, so the driver can tell the
# difference between "hold still" and "the publisher died".
bool valid

# "joy" or "web". Recorded for audit only. Arbitration is the mux's job and
# must never depend on this field.
string source
