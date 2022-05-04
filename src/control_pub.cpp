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
enum Odometry {omega, vx, vy};
ros::Publisher control_pub;

// Calculates rpm (wheel speeds) from odometry (body twist)
std::vector<double> rpmFromOdometry(std::vector<double> odometry){
  
  double rpm_fl = (1/r)*((-l-w)*odometry[omega]+odometry[vx]-odometry[vy]);
  double rpm_fr = (1/r)*((l+w)*odometry[omega]+odometry[vx]+odometry[vy]);
  double rpm_rr = (1/r)*((-l-w)*odometry[omega]+odometry[vx]+odometry[vy]);
  double rpm_rl = (1/r)*((l+w)*odometry[omega]+odometry[vx]-odometry[vy]);
  std::vector<double> rpm = {rpm_fl, rpm_fr, rpm_rr, rpm_rl};
  return rpm;
}

void callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  
  // Gather data from the odometry message
  double vx = msg->twist.linear.x;
  double vy = msg->twist.linear.y;
  double omega = msg->twist.angular.z;

  std::vector<double> odometry = {vx, vy, omega}; //temp

  // Calculate rpm from odometry
  std::vector<double> rpm = rpmFromOdometry(odometry);

  project1::Wheel_speed_msg rpm_msg;
    
  rpm_msg.rpm_fr = rpm[0];
  rpm_msg.rpm_fl = rpm[1];
  rpm_msg.rpm_rr = rpm[2];
  rpm_msg.rpm_rl = rpm[3];

  control_pub.publish(rpm_msg);
    

  // Prints for debug
  ROS_INFO("rpm front left: [%f]",rpm[0]);
  ROS_INFO("rpm front right: [%f]",rpm[1]);
  ROS_INFO("rpm rear right: [%f]",rpm[2]);
  ROS_INFO("rpm rear left: [%f]",rpm[3]);
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
  ros::Publisher control_pub = nh.advertise<project1::Wheel_speed_msg>("wheels_rpm", 1000);
  ros::Subscriber odometry_sub = nh.subscribe("cmd_vel", 1000, callback);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
