#!/bin/bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.25
angular:
  z: 0.3
" &
sleep 5.24
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
angular:
  z: 0.0
" --once
