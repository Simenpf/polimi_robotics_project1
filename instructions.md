# Team Members ( Name Surname (student ID) )
* Ingvild Fleisje ()
* Kristian Borgen (10877857)
* Simen Fløtaker  (10845390)

# File Descriptions
* `bags/`
  * The provided bags for robot ground-truth and encoder data
* `cfg/`
  * `parameters.cfg`: Configuration file used for dynamic reconfiguring of parameters.
* `include/`
  * `encoder_computations.h`: Header file of helper functions for transformations on encoder data.
* `launch/`
  * `project.launch`: Launch file used to initate all nodes and parameters used in the project.
* `msg/`
  * `Wheel_speed_msg.msg`: File defining the structure of the custom message for wheel rpms.
* `src/`
  * `command_node.cpp`: A node that subscribes to encoder messages and uses this to compute linear and angular robot velocities, published as commands for control.
  * `control_node.cpp`: A node that subscribes to velocity commands, and computes the inverse kinematics to get the required wheel speeds. These wheel speeds are published.
  * `encoder_computations.h`: A set of helper functions for transforming encoder data to robot velocities.
  * `odometry_node.cpp`: A node that subscribes to encoder messages, computes the odometry and publishes this odometry. Also maintains the TF tree.
* `srv/`
  * `Reset.srv`: File defining the structure of the custom service for resetting the robot odometry.
* `CMakeLists.txt`: File containing instructions of source files and libraries needed for building the project.
* `graph_of_project_nodes_and_topics.svg`: An image showing the interaction between nodes and topics in this project
* `instructions.md`: This file, containing information about the project.
* `package.xml`: File defining properties of the ros-package (there is only one package in the project).
* `plotjuggler_setup_GT_comparison.xml`: A plotjuggler configuration file. Can be loaded into plotjuggler to visualize the odometry results.
* `plotjuggler_setup_rpm_comparison.xml`: A plotjuggler configuration file. Can be loaded into plotjuggler to visualize the control results.

# ROS Parameters Name and Meaning
## The parameters of the robot hardware:
* `r`: Radius of the robot wheels. 
* `l`: Lenght from center of robot to center of wheel along x axis.
* `w`: Width from center of robot to center of wheel along y axis.
* `T`: Gear ratio between motor axel and wheel axel.
* `N`: Number of ticks per revolution of the wheel encoders.

## The initial values of the robot odometry (there are three sets of them corresponding to the three bags):
* `x_init`: The initial x-position of the robot.
* `y_init`: The initial y-position of the robot.
* `theta_init`: The initial rotation of the robot.

## Parameters for program configuration:
* `integrator`: Defines whether the odometry is computed using Euler method or Runge-Kutta of second order. Dynamically reconfigurable.
* `enable_parameter_tuning`: Defines whether to allow dynamical reconfiguration of the parameters `r`, `l` and `w`, used in the odometry calculation.

# Structure of the TF Tree

# Structure of Custom Message
Custom message for publishing desired RPM of all wheels:
* `Header header`:  Header
* `float64 rpm_fl`: RPM of front left wheel
* `float64 rpm_fr`: RPM of front right wheel
* `float64 rpm_rr`: RPM of rear right wheel 
* `float64 rpm_rl`: RPM of rear left wheel

# Instructions For Use
## Launching the Project
The project can be launched by running `roslaunch "path to"/project.launch`.
This will start all the nodes, and initialize the needed parameters.

## Visualizing the Odometry and RPM results
Layouts for plotjuggler has been made for easy visualization of the results. There is a layout for comparing the ground truth to the odometry found by integration. There is also a layout for comparing the wheel RPMs calculated directly from encoder data, and the RPMs calculated using inverse kinematics on robot velocities. 
To visualize the odometry:
  * Launch plotjuggler by running `rosrun plotjuggler plotjuggler`
  * Click the load layout button and select `plotjuggler_setup_GT_comparison.xml`
  * In the pop-up window select `/odom` and `/robot/pose`
To visualize the RPM:
  * Launch plotjuggler by running `rosrun plotjuggler plotjuggler`
  * Click the load layout button and select `plotjuggler_setup_rpm_comparison.xml`
  * In the pop-up window select `/wheels_rpm` and `/sensor_rpm`

It is important to note that since the plots are comparing the odometry (given in odom frame) to the ground truth (given in world frame), there will be a slight offset between the two if the ground truth (bag) does not have zero initial conditions.

## Tuning Parameters and Changing Integrator
Dynamic reconfigure has been used to allow for easier tuning of parameters and changing of odometry integrator-method. The default values of the dynamically reconfigurable parameters are the ones found after tuning. To tune/change parameters and change integrator dynamically, use: rosrun `rqt_reconfigure rqt_reconfigure`. Note that the parameter `enable_parameter_tuning` in the launch file must be set true for the tuning to affect the odometry.

## Resetting Odometry with Custom Service
The service for resetting the odometry can be called from the command-line, with three arguments in the following way: `rosservice call /reset x_value y_value theta_value`
This will move the odometry frame to the requested pose.



# Additional Information
## Explanation of Parameter Tuning Process
To tune the parameters `r`, `l` and `w` we added the possibility to have them dynamically reconfigurable (`T` and `N` was not tuned as they will contribute in the same factor as `r`, thus only `r` needs to be tuned. Although this does of course not guarantee that the values correspond with the actual values, just that they will work for the odometry computations). This way the parameter values could be changed with sliders, while looking at the plotjuggler plots.

Since `l` and `w` always contrbutes to computations as a sum, there is nothing to be gained by tuning them independently in this case. As there is no way to know what relationship between them is correct, given the data we have. Therefore, only `l` was tuned.

Our process for the tuning was as follows:
1. Since `l` and `w` only affect the integration of theta `r` could be tuned independenty using bag 1, as it contains no rotations. By changing `r` until the odometry matched the ground truth in the linear movement along the robot-frame's x-axis.
2. Using the tuned `r`, we could tune `l` by ensuring that the angular velocity matches the ground truth angular velocity in bag2 and bag3. 

## Nodes and Topics
To get a better overview of the nodes and topics of this project, and their interaction, a graph has been generated. This is the image `graph_of_project_nodes_and_topics.svg`:
<img src="./graph_of_project_nodes_and_topics.svg">

