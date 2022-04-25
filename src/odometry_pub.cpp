#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

double r;
double l;
double w;
double T;
int N;

double y = 0;
double x = 0;

enum Wheels {front_left, front_right, rear_left, rear_right};

double vxFromWheelSpeeds(std::vector<double> speeds){
    return -(r/4)*(speeds[front_left]+speeds[front_right]+speeds[rear_left]+speeds[rear_right]);
}

double vyFromWheelSpeeds(std::vector<double> speeds){
    return (r/4)*(speeds[front_left]-speeds[front_right]-speeds[rear_left]+speeds[rear_right]);
    
}
void encoderDataCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  std::vector<double> positions = msg->position;
  std::vector<double> speeds = msg->velocity;
  double vx = vxFromWheelSpeeds(speeds);
  double vy = vyFromWheelSpeeds(speeds);
  y+=vy;
  x+=vx;

  ROS_INFO("x: [%f]", x);
  ROS_INFO("y: [%f]", y);
  

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Retrieve parameters set in launch file
  ros::param::get("r",r);
  ros::param::get("l",l);
  ros::param::get("w",w);
  ros::param::get("T",T);
  ros::param::get("N",N);


  ros::Publisher odometry_pub = n.advertise<geometry_msgs::TwistStamped>("chatter", 1000);
  ros::Subscriber encoder_sub = n.subscribe("wheel_states", 1000, encoderDataCallback);


  ros::Rate loop_rate(100);

  while (ros::ok()) {

    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = 10;

    


    odometry_pub.publish(msg);


    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
