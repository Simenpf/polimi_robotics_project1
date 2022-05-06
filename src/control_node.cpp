#include "ros/ros.h"
#include "math.h"

// Message types for pubs / subs
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/Wheel_speed_msg.h"


class Control_node {
  private:
  ros::NodeHandle n;
  ros::Subscriber odometry_sub;
  ros::Publisher control_pub;
  
  double r;  // Robot wheel radius                                             
  double l;  // Lenght from center of robot to center of wheel along x axis
  double w;  // Width from center of robot to center of wheel along y axis  

  // Calculates rpm (wheel speeds) from body velocites (body twist)
  std::vector<double> rpmFromVelocities(double vx, double vy, double omega){
    // Computing wheel speeds (rad/s)
    double omega_fl = (1/r) * ( (-l-w) * omega + vx - vy );
    double omega_fr = (1/r) * ( (+l+w) * omega + vx + vy );
    double omega_rl = (1/r) * ( (-l-w) * omega + vx + vy );
    double omega_rr = (1/r) * ( (+l+w) * omega + vx - vy );

    // Converting to rpm
    double rpm_fl = (omega_fl*60) / (2*M_PI);
    double rpm_fr = (omega_fr*60) / (2*M_PI);
    double rpm_rl = (omega_rl*60) / (2*M_PI);
    double rpm_rr = (omega_rr*60) / (2*M_PI);

    return {rpm_fl, rpm_fr, rpm_rr, rpm_rl};
  }

  /* 
  Computes the required wheel speeds to achive the commanded velocities from sub (cmd_vel)
  Publishes these speeds on topic (wheel_speeds)*/
  void commandCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    // Gather data from the command message
    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;
    double omega = msg->twist.angular.z;

    // Calculate rpm from velocities
    std::vector<double> rpm = rpmFromVelocities(vx, vy, omega);

    // Publish rpms
    project1::Wheel_speed_msg rpm_msg;
    rpm_msg.header.stamp = msg->header.stamp;
    rpm_msg.rpm_fl = rpm[0];
    rpm_msg.rpm_fr = rpm[1];
    rpm_msg.rpm_rr = rpm[2];
    rpm_msg.rpm_rl = rpm[3];

    control_pub.publish(rpm_msg);
  }


  public:

  Control_node(){
    // Retrieve parameters set in launch file
    ros::param::get("r",r);
    ros::param::get("l",l);
    ros::param::get("w",w);

    // Initialize publisher and subscriber
    control_pub = n.advertise<project1::Wheel_speed_msg>("wheels_rpm", 1000);
    odometry_sub = n.subscribe("cmd_vel", 1000, &Control_node::commandCallback, this);
  }

  void run_main(){

    ros::Rate loop_rate(100);

    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "control_node");
  
  Control_node node;
  
  node.run_main();

  return 0;
}
