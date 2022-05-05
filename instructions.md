# Team Members ( Name Surname (student ID) )
* Ingvild Fleisje ()
* Kristian Borgen (10877857)
* Simen Fløtaker  (10845390)

# File Descriptions
* `bags/`
  * The provided bags for robot ground-truth
* `cfg/`
  * `parameters.cfg`: Configuration file used for dynamic reconfiguring of parameters.
* `include/`
  * `encoder_computations.h`: Header file of helper functions for transformations on encoder data.
* `launch/`
  * `project.launch`: Launch file used to initate all nodes and parameters used in the project.
* `msg/`
  * `Wheel_speed_msg.msg`: File defining the structure of the custom message for wheel rpms.
* `src/`
  * `command_node.cpp`: A node that subscribes to encoder messages and uses this to compute linear and angular velocities, published as commands for control.
  * `control_node.cpp`: A node that subscribes to velocity commands, and computes the inverse kinematics to get the required wheel speeds. These wheel speeds are published.
  * `encoder_computations.h`: A set of helper functions for transforming encoder data to robot velocities.
  * `odometry_node.cpp`: A node that subscribes to encoder messages, computes the odometry and publishes this odometry.
* `srv/`
  * `Reset.srv`: File defining the structure of the custom service for resetting the robot odometry.
* `CMakeLists.txt`: File containing instructions of source files and libraries needed for building the project.
* `instructions.md`: This file, containing information about the project.
* `package.xml`: File defining properties of the ros-package (there is only one package in the project).
* `plotjuggler_setup_GT_comparison.xml`: A plotjuggler configuration file. Can be loaded into plotjuggler to visualize the project results.

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
* `integrator`: Defines whether the odometry is computed using Euler method or Runge-Kutta of second order.

# Structure of the TF Tree

