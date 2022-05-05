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
* Write instructions file 
* Make TF tree
* Optimize params
* Refactor 
  - control_node
  - ros::init names
  - consider moving some global variables to main and use boost::bind?
* Consider adding initial pose for each bag in launch file. Comment out the ones not used.
* Consider filling seq_nr in header on messages to be published. Double check frame_ids.
* make classes of sub + pubs, see slides.
* should reset move odom frame to reset the requested pos and set base_link to 0? 
* smoother initalization of computeOdometry (prev variables and flag)
* decide to use this everywhere or not
* decide to put spinning loop in class or not
* Remove command for playing bag from launch