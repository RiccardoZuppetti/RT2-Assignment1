# Research Track II - First Assignment - ROS2 Branch

This package contains two components written in ROS2, [position_service](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/position_service.cpp) and [state_machine](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/state_machine.cpp).

The component [position_service](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/position_service.cpp) implements a server that replies as response a feasible pose of the robot: since the considered space has two dimensions, the client that requests this service receives an x-coordinate, a y-coordinate and an orientation theta, in which the point (x, y) and the orientation are chosen among a certain range of values.

The component [state_machine](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/state_machine.cpp) implements the server for the user_interface client. On top, it also implements a client for both the position_server and the go_to_point.

# How to Run

Inside your ROS workspace clone this ROS package

```
git clone -b temp https://github.com/RiccardoZuppetti/RT2-Assignment1.git rt2_assignment
```

Then build the workspace using the `catkin_make` command.

Create a ROS2 workspace and clone inside it this package

```
git clone -b ros2 https://github.com/RiccardoZuppetti/RT2-Assignment1.git rt2_assignment1
```

and then build the workspace

```
colcon build --symlink-install --packages-skip ros1_bridge
```

In the same ROS2 workspace clone this package

```
git clone https://github.com/ros2/ros1_bridge.git
```

and then build just the ROS1 bridge

```
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

At this point move to the `/root` folder and create the following `.sh` files

```
nano ros.sh
#!/bin/bash
source /root/<name_of_your_ros_workspace>/devel/setup.bash
```

```
nano ros2.sh
#!/bin/bash
source /root/<name_of_your_ros2_workspace>/install/setup.bash
```

```
nano ros12.h
#!/bin/bash
source /root/<name_of_your_ros_workspace>/devel/setup.bash
source /root/<name_of_your_ros2_workspace>/install/local_setup.bash
```

To run this project it is necessary to open three terminals.

In the first one `source ros.sh`, move to the ROS workspace and digit

```
roslaunch rt2_assignment sim.launch
```

In the second one `source ros12.sh`, move to the ROS2 workspace and digit

```
ros2 run ros1_bridge dynamic_bridge
```

In the last terminal `source ros2.sh`, move to the ROS2 workspace and digit

```
ros2 launch rt2_assignment1 sim_launch.py
```

# Robot behaviour

Once the simulation is booted, the robot is positioned in the (0, 0) point, with a 0 orientation and a null velocity: the robot remains in this state until the user starts the robot motion. When the motion has been started, the robot reaches a random position in the environment, with a random orientation. If the user wants to stop the robot motion, the robot stops and the previous goals will be canceled. If the user wants to restart the robot motion, the robot will move towards a new goal. If the robot reaches the goal and the user doesn't want to stop its motion, the robot will automatically reach a new goal position.

# Software architecture and architectural choices

This project has been implemented using ROS Noetic and ROS2 Foxy. The nodes (ROS Noetic) have been written in Python, while the components (ROS2 Foxy) are implemented in C++. Concerning the architecture used, the state_machine node controls the entire execution, since it can receive inputs from the user_interface, can send a request for a random position to the position_service, and finally can invoke the go_to_point service in order to reach this random position. The communication between these nodes is guaranteed by the ros1_bridge.
