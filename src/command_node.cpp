#include "ros/ros.h"
#include "encoder_computations.h"

// Message types for pubs / subs
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "project1/Wheel_speed_msg.h"


class Command_node {
  private:
  ros::NodeHandle n; 
  ros::Subscriber encoder_sub;
  ros::Publisher cmd_pub;
  ros::Publisher rpm_pub;
  ros::Time prev_time;
  std::vector<double> prev_wheel_ticks;
  bool no_previous_encoder_msg = true;
  bool publishing_true_rpm = true;

  double r;  // Robot wheel radius                                             
  double l;  // Lenght from center of robot to center of wheel along x axis
  double w;  // Width from center of robot to center of wheel along y axis  
  double T;  // Robot wheel gear ratio                                       
  int N;     // Robot wheel encoder ticks per revolution 

  void publish_true_rpm(std::vector<double> speeds) {
    double rpm_fl = (speeds[0]*60)/(2*M_PI);
    double rpm_fr = (speeds[1]*60)/(2*M_PI);
    double rpm_rl = (speeds[2]*60)/(2*M_PI);
    double rpm_rr = (speeds[3]*60)/(2*M_PI);

    project1::Wheel_speed_msg rpm_msg;
    rpm_msg.rpm_fr = rpm_fr;
    rpm_msg.rpm_fl = rpm_fl;
    rpm_msg.rpm_rr = rpm_rr;
    rpm_msg.rpm_rl = rpm_rl;

    rpm_pub.publish(rpm_msg);
  }

  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
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
  
    if (publishing_true_rpm){
      publish_true_rpm(speeds);
    }


    // Update global variables
    prev_wheel_ticks = new_wheel_ticks;
    prev_time        = new_time;
    
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

  public:
  Command_node(double r, double l, double w, double T, int N):
  r{r}, l{l}, w{w},T{T},N{N} {
    encoder_sub = n.subscribe("wheel_states", 1000, &Command_node::encoderCallback, this);
    cmd_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
    rpm_pub = n.advertise<project1::Wheel_speed_msg>("true_rpm", 1000);  
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  // Retrieve parameters set in launch file
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

  // Create node instance
  Command_node cmd_node(r,l,w,T,N);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
