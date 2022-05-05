#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/Wheel_speed_msg.h"
#include <sstream>
#include "math.h"


class Control_node {
private:
  double r;
  double l;
  double w;
  double T;
  int N;
  ros::NodeHandle n;
  ros::Publisher control_pub;
  ros::Subscriber odometry_sub;


  // Calculates rpm (wheel speeds) from odometry (body twist)
  std::vector<double> rpmFromOdometry(double vx, double vy, double omega){
    // Computing wheel speeds (rad/s)
    double omega_fl = (1/this->r) * ( (-this->l-this->w) * omega + vx - vy );
    double omega_fr = (1/this->r) * ( (+this->l+this->w) * omega + vx + vy );
    double omega_rl = (1/this->r) * ( (-this->l-this->w) * omega + vx + vy );
    double omega_rr = (1/this->r) * ( (+this->l+this->w) * omega + vx - vy );

    // Converting to rpm
    double rpm_fl = (omega_fl*60)/(2*M_PI);
    double rpm_fr = (omega_fr*60)/(2*M_PI);
    double rpm_rr = (omega_rr*60)/(2*M_PI);
    double rpm_rl = (omega_rl*60)/(2*M_PI);


    std::vector<double> rpm = {rpm_fl, rpm_fr, rpm_rr, rpm_rl};
    return rpm;
  }

public:
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
  Control_node(double r, double l, double w, double T) {
    this->control_pub = this->n.advertise<project1::Wheel_speed_msg>("wheels_rpm", 1000);
    this->odometry_sub = this->n.subscribe("cmd_vel", 1000, &Control_node::callback, this);
    this->r = r;
    this->l = l;
    this->w = w;
    this->T = T;
  }
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "rpm");
  
  // Retrieve parameters set in launch file
  double r;
  double l;
  double w;
  double T;
  ros::param::get("r",r);
  ros::param::get("l",l);
  ros::param::get("w",w);
  ros::param::get("T",T); //motor or wheel?

  Control_node cntr_node(r,l,w,T);
  



  ros::Rate loop_rate(100);
  while (ros::ok()) {
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
