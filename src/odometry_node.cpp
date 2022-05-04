#include "ros/ros.h"

// Message types for pubs / subs
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

// Includes for reset service
#include "project1/Reset.h"

// Includes for dynamic parameter reconfiguring
#include "project1/parametersConfig.h"
#include "dynamic_reconfigure/server.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

// Includes for quaternion operations
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "encoder_computations.h"

enum Integrator_method {Euler, RK2};

// Declare global parameters to be initialized from parameter-server
double r;         // Robot wheel radius                                         
double l;         // Lenght from center of robot to center of wheel along x axis
double w;         // Width from center of robot to center of wheel along y axis 
double T;         // Robot wheel gear ratio                                     
int N;            // Robot wheel encoder ticks per revolution                    
double y;         // Robot x pos
double x;         // Robot y pos
double theta;     // Robot heading

// Declare global variables and flags
bool no_previous_encoder_msg = true;
int integrator;
ros::Publisher odometry_pub;
ros::Publisher rpm_pub;                 // TEMP
#include "project1/Wheel_speed_msg.h"   // TEMP


void computeOdometry(const sensor_msgs::JointState::ConstPtr& msg) {
  // Declare static variables
  static ros::Time prev_time;
  static std::vector<double> prev_wheel_ticks(4,0.0);
  static int integrator;
  
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

  //*** Temp - For RPM debugging ****//
  project1::Wheel_speed_msg rpm_msg;
  double rpm_fl = (speeds[0]*60)/(2*M_PI);
  double rpm_fr = (speeds[1]*60)/(2*M_PI);
  double rpm_rl = (speeds[2]*60)/(2*M_PI);
  double rpm_rr = (speeds[3]*60)/(2*M_PI);

  rpm_msg.rpm_fr = rpm_fr;
  rpm_msg.rpm_fl = rpm_fl;
  rpm_msg.rpm_rr = rpm_rr;
  rpm_msg.rpm_rl = rpm_rl;

  rpm_pub.publish(rpm_msg);
  //***********************//

  // Update global variables
  prev_wheel_ticks = new_wheel_ticks;
  prev_time = new_time;

  // Calculate odometry
  double vx = vxFromWheelSpeeds(speeds, r);
  double vy = vyFromWheelSpeeds(speeds, r);
  double omega = omegaFromWheelSpeeds(speeds, r, l, w);

  if (integrator == Integrator_method::Euler){
    x += (vx*cos(theta) - vy*sin(theta)) * d_t;
    y += (vx*sin(theta) + vy*cos(theta)) * d_t;
    theta += omega*d_t;
  } 
  else if (integrator == Integrator_method::RK2){
    double angle = theta + omega*d_t/2;
    x += (vx*cos(angle) - vy*sin(angle)) * d_t;
    y += (vx*sin(angle) + vy*cos(angle)) * d_t;
    theta += omega*d_t;
  }

  // Convert heading to correct quaternion type
  tf2::Quaternion quat_tf; 
  quat_tf.setEuler(0, 0, theta);
  geometry_msgs::Quaternion heading_quat = tf2::toMsg(quat_tf);
  
  // Publish results
  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.stamp = new_time;
  odometry_msg.header.frame_id = "base_link";
  odometry_msg.pose.pose.position.x = x;
  odometry_msg.pose.pose.position.y = y;
  odometry_msg.pose.pose.position.z = 0.0;
  odometry_msg.pose.pose.orientation = heading_quat;
  odometry_pub.publish(odometry_msg);

  // Broadcast TF
  static tf2_ros::TransformBroadcaster tf_br;
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = new_time;
  tfStamped.header.frame_id = "odom";
  tfStamped.child_frame_id = "base_link";
  tfStamped.transform.translation.x = x;
  tfStamped.transform.translation.y = y;
  tfStamped.transform.translation.z = 0.0;
  tfStamped.transform.rotation = heading_quat;
  tf_br.sendTransform(tfStamped);
}

bool resetCallback(project1::Reset::Request  &req, project1::Reset::Response &res){
  x = req.new_x;
  y = req.new_y;
  theta = req.new_theta;
  ROS_INFO("x,y and theta reset...");
  return true;
}

void paramCallback(project1::parametersConfig& config, uint32_t level){
  integrator = config.integrator;
  if (config.integrator == Integrator_method::Euler){
    ROS_INFO("Changing to Euler integration method");
  }
  else if (config.integrator == Integrator_method::RK2){
    ROS_INFO("Changing to RK2 integration method");
  }
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
  odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
  ros::Subscriber encoder_sub = n.subscribe("wheel_states", 1000, computeOdometry);
  rpm_pub = n.advertise<project1::Wheel_speed_msg>("true_rpm", 1000);

  //Define reset service handler
  ros::ServiceServer service = 
      n.advertiseService<project1::Reset::Request, 
                         project1::Reset::Response>("reset", resetCallback);

  // Define dynamic parameter reconfigurer
  dynamic_reconfigure::Server<project1::parametersConfig> dynServer;
  dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
  f = boost::bind(&paramCallback, _1, _2);
  dynServer.setCallback(f);


  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
