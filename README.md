# ABU_Nav

ABU R0b0c0n 2O24 navigator node. Using 3 VL53L1X ToF sensors.

# Dependencies

- geometry_msgs
- rclcpp
- std_msgs
- mecanum_controller

# Hardware requirement
- Linux based PC with GPIO control
- x3 VL53L1X ToF sensor
- mecanum_controller compatible odom

# Sub
- /teammode (std_msgs/msg/String)

# Pub
- /cmd_vel (geometry_msgs/msg/Twist)