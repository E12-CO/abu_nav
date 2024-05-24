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
- /abu_nav_stat (std_msgs/msg/String)

# Example test
```
ros2 topic pub --once /teammode std_msgs/msg/String 'data: Red,Start'
```
```
ros2 topic pub --once /teammode std_msgs/msg/String 'data: Blue,RETRY'
```
```
ros2 topic pub --once /teammode std_msgs/msg/String 'data: stop,stop'
```

# Status publish
on topic ```/abu_nav_stat``` will publish the string ```DONE``` when the robot reached the area 3. This will be used to trigger the Area 3 node to start the ball mission.