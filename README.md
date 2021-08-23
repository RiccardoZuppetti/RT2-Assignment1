# Research Track II - First Assignment

The purpose of the assignment is to develop a simulation of a robot in a 3-dimensional environment, using the CoppeliaSim simulator. For the robot it was decided to adopt the Pioneer P3DX model.

The package is composed of four nodes, and three servers. The nodes are go_to_point, user_interface, position_service, state_machine. The servers are Command and RandomPosition and Position.

The node go_to_point implements a server, when it is called it receives a request to reach a point with a given orientation, it then implements the algorithm to control the robot.
The node user_interface reads from input from the client and calls the server that implements the calls to the go_to_point.
The node position_service implements a server that receives as input the ranges for the feasible position along x and y and for the feasible orientation and returns a goal position, a point (x,y) and an orientation chosen randomly in the interval.
The node state_machine implements the server for the user_interface client and also implements the client for both the position_server and the go_to_point.

Regarding the simulation, it is created a scene, called "prova_scene.ttt". Seeing the script it is possible to notice that there is a publisher to the topic /odom (used to publish the current position of the robot) and there is also a subscriber to the topic /cmd_vel (used to retrieve the velocities of the two wheels of the robot). On top, there is the function "velocity" that set, using the differential form, the velocities of the two wheels of the used model, as soon as new data are available on the topic /cmd_vel.

# How to Run

Verify that you have installed ROS Noetic. Create a ROS Workspace and clone this ROS package inside it. Then build the workspace using the catkin_make command.

Modify the .bashrc file, using the command gedit ~/.bashrc, and sourcing the ROS environment, the ROS workspace and exporting the correct directory of CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.

To run this problem it is necessary to open two terminals.

Inside the first terminal go into the ROS workspace that you've just created and digit
- roslaunch rt2_assignment1 sim.launch 

In the second terminal go in the folder of CoppeliaSim_Edu_V4_2_0_Ubuntu20_04 and digit
- ./coppeliaSim.sh

Once the CoppeliaSim software started in order to open the correct scene go in File, upper left corner, Open scene and select the scene "prova_scene.ttt" (that is inside the package cloned).
Then press the "Play" button in the CoppeliaSim software and, when the simulation has started, start to move the robot, pressing (from the first terminal opened) "1" to start the motion to a random position and orientation, or "0" to stop the robot as soon as it reaches the goal.