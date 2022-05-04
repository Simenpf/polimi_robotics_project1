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
double x;
double y;
double omega;

//Variables 
std::vector<double> prev_odometry(3,0.0);
bool no_previous_odometry_msg = true;
ros::Time prev_time;
enum Odometry {omega, vx, vy};
enum BodyTwist {omega_vz, v_bx, v_by};
ros::Publisher control_pub;

// Calculate body twist from odometry
std::vector<double> bodyTwistFromOdometry(std::vector<double> odometry){
  double omega_bz = odometry[omega_bz];
  double v_bx = cos(omega)*body_twist[v_bx]+sin(omega)*body_twist[v_by]);
  double v_by = -sin(omega)*body_twist[v_bx]+cos(omega)*body_twist[v_by]);
  std::vector<double> body_twist = {omega_bz, v_bx, v_by};
  return body_twist;
}
// Calculates rpm (wheel speeds) from body twist
std::vector<double> rpmFromBodyTwist(std::vector<double> body_twist){
  
  double rpm_fl = (1/r)*((-l-w)*body_twist[omega_bz]+body_twist[v_bx]-body_twist[v_by]);
  double rpm_fr = (1/r)*((l+w)*body_twist[omega_bz]+body_twist[v_bx]+body_twist[v_by]);
  double rpm_rr = (1/r)*((-l-w)*body_twist[omega_bz]+body_twist[v_bx]+body_twist[v_by]);
  double rpm_rl = (1/r)*((l+w)*body_twist[omega_bz]+body_twist[v_bx]-body_twist[v_by]);
  std::vector<double> rpm = {rpm_fl, rpm_fr, rpm_rr, rpm_rl};
  return rpm;
}

void callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  
  if(no_previous_odometry_msg){
    //todo
    return;
  }
  
  
  // Gather data from the odometry message
  //TODO
  //std::vector<double> new_odometry = msg->whattowritehere;
  std::vector<double> new_odometry(3, 0.0); //temp

  //Init 
  std::vector<double> rpm(4,0.0);
  ros::Time new_time = msg->header.stamp;
  double d_t = (new_time-prev_time).toSec();
  
  // Calculate rpm from odometry
  for (int i = 0; i < rpm.size(); i++){
    
  }
  std::vector<double> rpm = rpmFromOdometry(new_odometry);

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
  //ros::param::get("T",T);
  //ros::param::get("N",N);
   

  // Define control publisher and odometry subscriber
  ros::Publisher control_pub = nh.advertise<project1::Wheel_speed_msg>("wheels_rpm", 1000);
  ros::Subscriber odometry_sub = nh.subscribe("cmd_vel", 1000, callback);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    
    project1::Wheel_speed_msg rpm_msg;
    
    std::vector<double> rpm = rpmFromOdometry(odometry);
 
    rpm_msg.rpm_fr = rpm[0];
    rpm_msg.rpm_fl = rpm[1];
    rpm_msg.rpm_rr = rpm[2];
    rpm_msg.rpm_rl = rpm[3];

    control_pub.publish(rpm_msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
