## Initial setup to get started
* Put project in catkin_ws/src.
* Every terminal you open move to catkin_ws root and run `source devel/setup.bash`
* Run `catkin_make` (The first time the project is pulled, and every time the project is changed)
* Before running code (in a separate terminal, from catkin_ws): `roscore` 
* To run code (from catkin_ws root): `roslaunch src/project1/launch/project.launch`

## Handy commands
* To see all current topics: `rostopic list`
* To live-log a topic in terminal:`rostopic echo "topic name from rostopic list"`
