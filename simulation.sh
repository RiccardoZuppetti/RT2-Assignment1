#!/bin/bash
gnome-terminal --tab --title="roscore" -- bash -c "source ros.sh; roscore"
gnome-terminal --tab --title="ros1_assignment" -- bash -c "sleep 1; source ros.sh; roslaunch rt2_assignment_temp sim2.launch"
gnome-terminal --tab --title="ros_bridge" -- bash -c "source ros12.sh; ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="ros2_assignment" -- bash -c "source ros2.sh; ros2 launch rt2_assignment1 ros2_launch.py"