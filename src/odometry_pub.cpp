#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include "math.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "project1/Reset.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

// Declare variables to be retrieved from parameter-server
double r;
double l;
double w;
double T;
int N;
double y;
double x;
double theta;

// Initialize and declare global variables and flags
std::vector<double> prev_wheel_ticks(4,0.0);
bool no_previous_encoder_msg = true;
ros::Time prev_time;
enum Wheels {front_left, front_right, rear_left, rear_right};
ros::Publisher odometry_pub;
ros::Publisher pose_pub;



double vxFromWheelSpeeds(std::vector<double> speeds){
    return (r/4) * (speeds[front_left]+speeds[front_right]+speeds[rear_left]+speeds[rear_right]);
}

double vyFromWheelSpeeds(std::vector<double> speeds){
    return (r/4) * (-speeds[front_left]+speeds[front_right]+speeds[rear_left]-speeds[rear_right]);   
}

double omegaFromWheelSpeeds(std::vector<double> speeds){
  return (r / (4*(l+w)) ) * (-speeds[front_left]+speeds[front_right]-speeds[rear_left]+speeds[rear_right]);
}

double wheelSpeedFromTicks(double d_tick, double d_t){
  return (d_tick*2*M_PI)/(N*T*d_t);
}

void encoderDataCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // Cannot compute wheel speeds after only one measurement
  if(no_previous_encoder_msg){
    prev_time = msg->header.stamp;
    prev_wheel_ticks = msg->position;
    no_previous_encoder_msg = false;
    return;
  }

  // Gather data from the encoder message
  std::vector<double> new_wheel_ticks = msg->position;
  std::vector<double> true_speeds = msg->velocity;        //unused??

  // Initiate temporary variables
  std::vector<double> speeds(4,0.0);
  ros::Time new_time = msg->header.stamp;
  double d_t = (new_time-prev_time).toSec();
  
  // Calculate wheel speeds from encoder ticks
  for (int i = 0; i < speeds.size(); i++){
    double d_tick = new_wheel_ticks[i] - prev_wheel_ticks[i];
    speeds[i] = wheelSpeedFromTicks(d_tick, d_t);
  }

  // Update global variables
  prev_wheel_ticks = new_wheel_ticks;
  prev_time = new_time;


  // Calculate odometry
  double vx = vxFromWheelSpeeds(speeds);
  double vy = vyFromWheelSpeeds(speeds);
  double omega = omegaFromWheelSpeeds(speeds);

  // Euler integration
  x += (vx*cos(theta) - vy*sin(theta)) * d_t;
  y += (vx*sin(theta) + vy*cos(theta)) * d_t;

  // RK2 integration
  double angle = theta + omega*d_t/2;
  // x += (vx*cos(angle) - vy*sin(angle)) * d_t;
  // y += (vx*sin(angle) + vy*cos(angle)) * d_t;
  theta += omega*d_t;

  // Publish results
  geometry_msgs::TwistStamped odometry_msg;
  odometry_msg.header.stamp = new_time;
  odometry_msg.twist.linear.x  = vx;
  odometry_msg.twist.linear.y  = vy;
  odometry_msg.twist.angular.z = omega;
  odometry_pub.publish(odometry_msg);

  tf2::Quaternion quat_tf; 
  quat_tf.setEuler(0, 0, theta);

  nav_msgs::Odometry pose_msg;
  pose_msg.header.stamp = new_time;
  // Set any frame_ids?
  pose_msg.pose.pose.position.x = x;
  pose_msg.pose.pose.position.y = y;
  pose_msg.pose.pose.position.z = 0.0;
  pose_msg.pose.pose.orientation = tf2::toMsg(quat_tf);
  pose_pub.publish(pose_msg);

  // Broadcast TF
  static tf2_ros::TransformBroadcaster tf_br;
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = new_time;
  tfStamped.header.frame_id = "odom";
  tfStamped.child_frame_id = "base_link";
  tfStamped.transform.translation.x = x;
  tfStamped.transform.translation.y = y;
  tfStamped.transform.translation.z = 0.0;
  tfStamped.transform.rotation = tf2::toMsg(quat_tf);
  tf_br.sendTransform(tfStamped);
  
  // Display results for debugging
  // ROS_INFO("x: [%f]", x);
  // ROS_INFO("y: [%f]", y);
  // ROS_INFO("theta: [%f]", theta);
}

bool reset_callback(project1::Reset::Request  &req, project1::Reset::Response &res) {
  x = req.new_x;
  y = req.new_y;
  theta = req.new_theta;
  ROS_INFO("x,y and theta reset...");
  return true;
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
  ros::param::get("x_init",x);
  ros::param::get("y_init",y);
  ros::param::get("theta_init",theta);


  // Define odometry publisher and encoder subscriber
  odometry_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_val", 1000);
  pose_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
  ros::Subscriber encoder_sub = n.subscribe("wheel_states", 1000, encoderDataCallback);

  //Define reset service handler
  ros::ServiceServer service = 
      n.advertiseService<project1::Reset::Request, 
                         project1::Reset::Response>("reset", reset_callback);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
