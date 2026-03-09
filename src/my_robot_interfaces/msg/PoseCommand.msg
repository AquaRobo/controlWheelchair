# PoseCommand.msg - Extended version
float64 x
float64 y
float64 z

# Option 1: Euler angles
float64 roll
float64 pitch
float64 yaw

# Option 2: Quaternion
float64 qx
float64 qy
float64 qz
float64 qw

bool use_quaternion  # True to use quaternion, False to use Euler