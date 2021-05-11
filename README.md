ROS First Project 2021
Student: Davide Mornatta 10657647

FOLDERS STRUCTURE: (name of the package: "first_project")
- cfg: contains the config file with the parameter needed to change the approximation method at run time.
- launch: contains the requested launch file that starts the node "odometry"
- msg: contains the provided custom message for the input data of the wheels, and the custom odometry message that is published as specifications
- src: main folder, it contains the node C++ implementation as "odometry.cpp"
- srv: contain the two requested services declaration, with the expected calling input 
- CMakeLists.txt, package.xml: usual needed files to specify dependencies, build instructions, executables and so on

PARAMETERS:
- approx: parameter that declares the desired approximation method. It has two possible values, 0 and 1, meaning respectively "Euler Approximation" and "Runge-Kutta Approximation". It has 0 as default value. It can be dynamicly reconfigurated.

TF TREE:
odom --> base_link

CUSTOM MESSAGE:
Apart from the provided custom message, it is used another custom message called CustomOdometry, that follows the specifications, so it is built like:
nav_msgs/Odometry odom
std_msgs/String method

DESCRIPTION:
As requested, the node can be started using the launch file "launcher.launch".
The project has been structured as a single node "odometry" that contains a class modeling the SCOUT robot and the node that subscribes to the input topics.
The scout_odometry class contains pose and velocity of the robot, methods to calculate odometry in both the approximations, and callback methods in order to use dynamic reconfiguration and services. Finally, a method to publish the custom odometry.

In order to use the dynamic reconfiguration, it should be used:
rosrun dynamic_reconfiguration dynparam set odometry approx [0 or 1]

Furthermore, in order to use the two services:
- If you want to use the reset service: rosservice call reset_pose (set the current pose to all 0s)
- If you want to use the teleport service: rosservice call teleport_pose [float64 x] [float64 y] [float64 theta]

ADDITIONAL INFO:
The gear ratio and the apparent baseline have been calibrated comparing the odometry with the manufacturer one. To determinate the values, i have observed the difference in behaviour and through trial and error i obtained some valid values even in the long run.



