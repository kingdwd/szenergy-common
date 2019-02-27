# Refer to https://drive.google.com/drive/folders/0B8gK3ayIgglxeDBGNDF5U2thcFU
# CAN Message ID 10
std_msgs/Header header            # header of msg (stamp, etc.)
# Feedback from low-level controller
bool            is_autonomous_mode  # Indicates that the vehicle is in autonomous drive mode [T/F]
float64         throttle_position   # Indicates throttle position [%]
float64         vehicle_speed       # Indicates current velocity [km/h]
float64         steer_angle         # Indicates current steer angle [deg]
