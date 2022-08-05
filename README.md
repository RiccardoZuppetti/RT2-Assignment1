# Research Track II - First Assignment - Action Branch

This package is composed of four nodes, two servers and one action. The nodes are [go_to_point](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/scripts/go_to_point.py), [user_interface](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/scripts/user_interface.py), [position_service](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/src/position_service.cpp), [state_machine](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/src/state_machine.cpp). The servers are [Command](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/srv/Command.srv) and [RandomPosition](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/srv/RandomPosition.srv). The action is [Planning](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/action/Planning.action).

This branch contains the same content of the main one, with the difference that here the [go_to_point](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/scripts/go_to_point.py) node is modelled as a ROS action server, introducing the possibility to cancel the goal and stop the robot at any time. It implements the algorithm to control the robot: it subscribes to the `/odom` topic, in order to retrieve the current position and orientation of the robot, and publishes on the `/cmd_vel` topic. The control algorithm is structured in different phases: at first, the robot orients itself towards the goal position that should be reached; then, it moves in a straight direction towards the goal; once the goal is reached, the robot rotates itself according to the provided random orientation. 

The node [user_interface](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/scripts/user_interface.py) reads the inputs given by the user: depending on the input received it can recall the go_to_point service, through which the robot can perform a motion or can immediately stop if the user want to stop the robot.

The node [position_service](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/src/position_service.cpp) implements a server that receives as input a feasible pose of the robot: since the considered space has two dimensions, it receives an x-coordinate, a y-coordinate and an orientation theta, in which the point (x, y) and the orientation are chosen among a certain range of values.

The node [state_machine](https://github.com/RiccardoZuppetti/RT2-Assignment1/blob/action/src/state_machine.cpp) implements the server for the user_interface client. On top, it also implements a client for both the position_server and the go_to_point. If the user want that the robot moves, and so the robot is not moving until now, this node requests a random position to the position_server, that will be then passed to the go_to_point. If instead the robot is already moving, the action is set as succeeded. If the user want to stop the robot that is moving, then a request to cancel the goal is sent to the action server. If the goal has been reached and the user want that the robot moves, then a new random position will be requested.

# How to Run

Verify that you have installed ROS Noetic. Create a ROS Workspace and clone this ROS package inside it

```
git clone -b action https://github.com/RiccardoZuppetti/RT2-Assignment1.git rt2_assignment1
```

Then build the workspace using the `catkin_make` command.

Modify the `.bashrc` file, using the command `gedit ~/.bashrc`, sourcing the ROS environment and the ROS workspace.

To run this project open a terminal, move to the created ROS workspace and digit

```
roslaunch rt2_assignment1 sim.launch 
```

# Robot behaviour

Once the simulation is booted, the robot is positioned in the (0, 0) point, with a 0 orientation and a null velocity: the robot remains in this state until the user starts the robot motion. When the motion starts, the robot moves toward the goal. If the user wants to stop the robot, it will immediately stop (in the current position, and not when the goal has been reached), and the previous goals will be canceled. If the user wants to restard the robot motion, it will move toward a new goal.

# Software architecture and architectural choices

Both nodes go_to_point and user_interface are implemented in Python, and so it is important to make the associated scripts executable. The nodes position_service and state_machine are implemented in C++. Concerning the architecture used, the state_machine node controls the entire execution, since it can receive inputs from the user_interface, can send a request for a random position to the position_service, and finally can invoke the go_to_point service in order to reach this random position.

# System limitations and improvements

With the current implementation, if the user wants to stop and then restart the robot motion, as said before, all the previous goals will be canceled, and then a new goal is given: for this reason it is not possible to continue toward the previous goal. To overcome this limitation the current goal position must be saved, so that it is resumed when the robot moves again.
