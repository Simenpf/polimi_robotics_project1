#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

#include "encoder_computations.h"

class Command_node {
private:
  ros::NodeHandle n; 
  ros::Subscriber encoder_sub;
  ros::Publisher cmd_pub;
  bool no_previous_encoder_msg = true;
  double r;        // Robot wheel radius                                             
  double l;        // Lenght from center of robot to center of wheel along x axis
  double w;        // Width from center of robot to center of wheel along y axis  
  double T;        // Robot wheel gear ratio                                       
  int N;           // Robot wheel encoder ticks per revolution 
  ros::Time prev_time;
  std::vector<double> prev_wheel_ticks; //(4,0.0) removed

public:
  void computeCommand(const sensor_msgs::JointState::ConstPtr& msg) {
    // Cannot compute wheel speeds after only one measurement
    if(this->no_previous_encoder_msg){
      this->prev_time = msg->header.stamp;
      this->prev_wheel_ticks = msg->position;
      this->no_previous_encoder_msg = false;
      return;
    }
    
    // Gather data from the encoder message
    std::vector<double> new_wheel_ticks = msg->position;
  
    // Initiate temporary variables
    std::vector<double> speeds(4,0.0);
    ros::Time new_time = msg->header.stamp;
    double d_t = (new_time-this->prev_time).toSec();
    
    // Calculate wheel speeds from encoder ticks
    for (int i = 0; i < speeds.size(); i++){
      double d_tick = new_wheel_ticks[i] - this->prev_wheel_ticks[i];
      speeds[i] = wheelSpeedFromTicks(d_tick, d_t, this->N, this->T);
    }
  
    // Update global variables
    this->prev_wheel_ticks = new_wheel_ticks;
    this->prev_time        = new_time;
    
    // Calculate odometry
    double vx = vxFromWheelSpeeds(speeds, this->r);
    double vy = vyFromWheelSpeeds(speeds, this->r);
    double omega = omegaFromWheelSpeeds(speeds, this->r, this->l, this->w);
  
    // Publish results
    geometry_msgs::TwistStamped cmd_msg;
    cmd_msg.header.stamp = new_time;
    cmd_msg.header.frame_id = "base_link";
    cmd_msg.twist.linear.x  = vx;
    cmd_msg.twist.linear.y  = vy;
    cmd_msg.twist.angular.z = omega;
    cmd_pub.publish(cmd_msg);
  }

  Command_node(double r, double l, double w, double T, int N){
    this->encoder_sub = this->n.subscribe("wheel_states", 1000, &Command_node::computeCommand, this);
    this->cmd_pub = this-> n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
    this->r = r;                       
    this->l = l;
    this->w = w;
    this->T = T;                     
    this->N = N;   
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  // Set global parameters from launch file
  double r;
  double l;
  double w;
  double T;
  int N;
  ros::param::get("r",r);
  ros::param::get("l",l);
  ros::param::get("w",w);
  ros::param::get("T",T);
  ros::param::get("N",N);

  // Create a node instance
  Command_node cmd_node(r,l,w,T,N);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
