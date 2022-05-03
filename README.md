## Initial setup to get started
* Put project in catkin_ws/src.
* Every terminal you open move to catkin_ws root and run `source devel/setup.bash`
* Run `catkin_make` (The first time the project is pulled, and every time the project is changed)
* Before running code (in a separate terminal, from catkin_ws): `roscore` 
* To run code (from catkin_ws root): `roslaunch src/project1/launch/project.launch`

## Handy commands
* To see all current topics: `rostopic list`
* To live-log a topic in terminal:`rostopic echo "topic name from rostopic list"`
* To use dynamic reconfiguring from GUI: `rosrun rqt_reconfigure rqt_reconfigure`

## Data visualization using plotjuggler
* Install plotjuggler by running `sudo apt install ros-melodic-plotjuggler-ros`
* Launch plotjuggler by running `rosrun plotjuggler plotjuggler`
* A GUI shall open. Run the code with roslaunch to start the publishers you want to visualize data from. In plotjuggler, in the "Streaming" section, start a ROS topic subscriber and check off the desired topics. In the "Timeseries" section, expand the topics until desired signals are found. Click and drag into graph.
* Plotjuggler xml file can be loaded into GUI (see "Layout" in "Files" section)to get nice plotting setup.

## TODO
* Put extra stuff on published messages (like filling the header etc)?
* Why is there no action in heading in bag 1?
* Look at post on slack/ros-help about CMAKE add_dependencies() for services, dynamic reconfigure and custom messages.
* Set up TF tree with world, odom and base_link?
* Split our code into packages? Kinda feels like he wants that
* Clean up node and package name and type. In launch file, in main (ros::init), in parameters.cfg (and in the generated include and use in main) 
* Test using the reset service, if not done already