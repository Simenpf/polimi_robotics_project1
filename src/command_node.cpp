#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

#include "encoder_computations.h"

// Declare global parameters to be initialized from parameter-server
double r;        // Robot wheel radius                                             
double l;        // Lenght from center of robot to center of wheel along x axis
double w;        // Width from center of robot to center of wheel along y axis  
double T;        // Robot wheel gear ratio                                       
int N;           // Robot wheel encoder ticks per revolution                                                                                 

// Declare global variables and flags
bool no_previous_encoder_msg = true;
ros::Publisher cmd_pub;


void computeCommand(const sensor_msgs::JointState::ConstPtr& msg) {
  // Declare static variables
  static ros::Time prev_time;
  static std::vector<double> prev_wheel_ticks(4,0.0);
  
  // Cannot compute wheel speeds after only one measurement
  if(no_previous_encoder_msg){
    prev_time = msg->header.stamp;
    prev_wheel_ticks = msg->position;
    no_previous_encoder_msg = false;
    return;
  }

  // Gather data from the encoder message
  std::vector<double> new_wheel_ticks = msg->position;

  // Initiate temporary variables
  std::vector<double> speeds(4,0.0);
  ros::Time new_time = msg->header.stamp;
  double d_t = (new_time-prev_time).toSec();
  
  // Calculate wheel speeds from encoder ticks
  for (int i = 0; i < speeds.size(); i++){
    double d_tick = new_wheel_ticks[i] - prev_wheel_ticks[i];
    speeds[i] = wheelSpeedFromTicks(d_tick, d_t, N, T);
  }

  // Update global variables
  prev_wheel_ticks = new_wheel_ticks;
  prev_time = new_time;

  // Calculate odometry
  double vx = vxFromWheelSpeeds(speeds, r);
  double vy = vyFromWheelSpeeds(speeds, r);
  double omega = omegaFromWheelSpeeds(speeds, r, l, w);

  // Publish results
  geometry_msgs::TwistStamped cmd_msg;
  cmd_msg.header.stamp = new_time;
  cmd_msg.header.frame_id = "base_link";
  cmd_msg.twist.linear.x  = vx;
  cmd_msg.twist.linear.y  = vy;
  cmd_msg.twist.angular.z = omega;
  cmd_pub.publish(cmd_msg);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Set global parameters from launch file
  ros::param::get("r",r);
  ros::param::get("l",l);
  ros::param::get("w",w);
  ros::param::get("T",T);
  ros::param::get("N",N);

  // Define command publisher and encoder subscriber
  cmd_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  ros::Subscriber encoder_sub = n.subscribe("wheel_states", 1000, computeCommand);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
