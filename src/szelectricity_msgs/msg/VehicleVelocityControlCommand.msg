# Refer to https://drive.google.com/drive/folders/0B8gK3ayIgglxeDBGNDF5U2thcFU
# CAN Message ID 10
std_msgs/Header header            # header of msg (stamp, etc.)
# Input to low-level controller
bool            set_autonomous_mode # Indicates that the vehicle is in autonomous drive mode [T/F]
float64         ref_vehicle_speed   # Indicates reference vehicle speed [km/h]
float64         ref_steer_angle     # Indicates reference steer angle [deg]
