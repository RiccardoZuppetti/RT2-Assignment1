# Research Track II - First Assignment - Ros2 branch

This package contains two nodes written in ROS2, [position_service](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/position_service.cpp) and [state_machine](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/state_machine.cpp).

The node [position_service](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/position_service.cpp) implements a server that replies as response a feasible pose of the robot: since the considered space has two dimensions, the client that requests this service receives an x-coordinate, a y-coordinate and an orientation theta, in which the point (x, y) and the orientation are chosen among a certain range of values.

The node [state_machine](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/ros2/src/state_machine.cpp) implements the server for the user_interface client. On top, it also implements a client for both the position_server and the go_to_point.

# How to Run

Verify that you have installed ROS Noetic. Create a ROS Workspace and clone this ROS package inside it

```
git clone -b main https://github.com/RiccardoZuppetti/RT2-Assignment1.git rt2_assignment1
```

Then build the workspace using the `catkin_make` command.

Modify the `.bashrc` file, using the command `gedit ~/.bashrc`, sourcing the ROS environment, the ROS workspace and exporting the correct directory of `CoppeliaSim_Edu_V4_2_0_Ubuntu20_04`.

To run this project it is necessary to open two terminals.

In the first terminal move to the created ROS workspace and digit

```
roslaunch rt2_assignment1 sim.launch 
```

In the second terminal move to the `CoppeliaSim_Edu_V4_2_0_Ubuntu20_04` folder and digit

```
./coppeliaSim.sh
```

Once CoppeliaSim started the "prova_scene.ttt" must be opened, which is inside the package cloned. Then, the CoppeliaSim simulation must be booted.
When the simulation has started, it is possible to move the robot pressing (in the first terminal opened) the "1" key: in this way, the robot will move towards a random position and orientation. To stop the robot movement it is necessary to press the "0" key.

# Robot behaviour

Once the simulation is booted, the robot is positioned in the (0, 0) point, with a 0 orientation and a null velocity: the robot remains in this state until the user starts the robot motion. When the motion has been started, the robot reaches a random position in the environment, with a random orientation. If the user wants to stop the robot motion, the robot will stop as soon as the goal position has been reached.

# Software architecture and architectural choices

Both nodes go_to_point and user_interface are implemented in Python, and so it is important to make the associated scripts executable. The nodes position_service and state_machine are implemented in C++. Concerning the architecture used, the state_machine node controls the entire execution, since it can receive inputs from the user_interface, can send a request for a random position to the position_service, and finally can invoke the go_to_point service in order to reach this random position.
