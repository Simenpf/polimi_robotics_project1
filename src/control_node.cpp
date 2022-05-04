#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/Wheel_speed_msg.h"
#include <sstream>
#include "math.h"

// Declare variables to be retrieved from parameter-server
double r;
double l;
double w;
double T;
int N;


// Variables 
ros::Publisher control_pub;

// Calculates rpm (wheel speeds) from odometry (body twist)
std::vector<double> rpmFromOdometry(double vx, double vy, double omega){
  // Computing wheel speed (rad/s)
  double omega_fl = (1/r) * ( (-l-w) * omega + vx - vy );
  double omega_fr = (1/r) * ( (l+w)  * omega + vx + vy );
  double omega_rl = (1/r) * ( (-l-w) * omega + vx + vy );
  double omega_rr = (1/r) * ( (l+w)  * omega + vx - vy );

  // Converting to rpm
  double rpm_fl = (omega_fl*60)/(2*M_PI);
  double rpm_fr = (omega_fr*60)/(2*M_PI);
  double rpm_rr = (omega_rr*60)/(2*M_PI);
  double rpm_rl = (omega_rl*60)/(2*M_PI);


  std::vector<double> rpm = {rpm_fl, rpm_fr, rpm_rr, rpm_rl};
  return rpm;
}

void callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // Gather data from the odometry message
  double vx = msg->twist.linear.x;
  double vy = msg->twist.linear.y;
  double omega = msg->twist.angular.z;

  // Calculate rpm from odometry
  std::vector<double> rpm = rpmFromOdometry(vx, vy, omega);

  project1::Wheel_speed_msg rpm_msg;
    
  rpm_msg.rpm_fl = rpm[0];
  rpm_msg.rpm_fr = rpm[1];
  rpm_msg.rpm_rr = rpm[2];
  rpm_msg.rpm_rl = rpm[3];

  control_pub.publish(rpm_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rpm");
  ros::NodeHandle nh;

  // Retrieve parameters set in launch file
  ros::param::get("r",r);
  ros::param::get("l",l);
  ros::param::get("w",w);
  ros::param::get("T",T); //motor or wheel?
  //ros::param::get("N",N);
  

  // Define control publisher and odometry subscriber
  control_pub = nh.advertise<project1::Wheel_speed_msg>("wheels_rpm", 1000);
  ros::Subscriber odometry_sub = nh.subscribe("cmd_vel", 1000, callback);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
