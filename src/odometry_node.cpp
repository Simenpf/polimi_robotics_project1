#include "ros/ros.h"

// Includes for pub / sub message types
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

// Includes for TF broadcasting
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

// Includes for reset service
#include "project1/Reset.h"

// Includes for dynamic parameter reconfiguring
#include "project1/parametersConfig.h"
#include "dynamic_reconfigure/server.h"

// Includes for quaternion operations
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "encoder_computations.h"

enum Integrator_method {Euler, RK2};

class Odometry_node {
  private:
  ros::NodeHandle n;
  ros::Subscriber encoder_sub;
  ros::Publisher odometry_pub;
  tf2_ros::TransformBroadcaster tf_br;

  ros::ServiceServer service;
  dynamic_reconfigure::Server<project1::parametersConfig> dynServer;
  dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
 
  // Pose variables w.r.t odom frame
  double x;                         // Robot x pos   
  double y;                         // Robot y pos   
  double theta;                     // Robot heading
 
  // Dynamic reconfigurable parameters, see cfg/parameters.cfg for default values
  int integrator;                   // Odometry integration method
  double r;                         // Robot wheel radius                                         
  double l;                         // Lenght from center of robot to center of wheel along x axis
  double w;                         // Width from center of robot to center of wheel along y axis 
  double T;                         // Robot wheel gear ratio                                     
  int N;                            // Robot wheel encoder ticks per revolution

  // Variables for odometry computation
  bool no_previous_encoder_msg;
  ros::Time prev_measurement_time;
  std::vector<double> prev_wheel_ticks;
  
  /*
  Computes odometry from encoder ticks
  Publishes results as nav_msgs/Odometry on topic odom
  Broadcasts corresponding odom->base_link TF */
  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg){
    
    // Cannot compute wheel speeds after only one measurement
    if(no_previous_encoder_msg){
      prev_measurement_time = msg->header.stamp;
      prev_wheel_ticks = msg->position;
      no_previous_encoder_msg = false;
      return;
    }

    // Gather data from the encoder message
    std::vector<double> new_wheel_ticks = msg->position;

    // Initiate temporary variables
    std::vector<double> speeds(4,0.0);
    ros::Time new_time = msg->header.stamp;
    double d_t = (new_time-prev_measurement_time).toSec();

    // Calculate wheel speeds from encoder ticks
    for (int i = 0; i < speeds.size(); i++){
      double d_tick = new_wheel_ticks[i] - prev_wheel_ticks[i];
      speeds[i] = wheelSpeedFromTicks(d_tick, d_t, N, T);
    }

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

    // Update variables
    prev_wheel_ticks = new_wheel_ticks;
    prev_measurement_time = new_time;

    // Publish results
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = new_time;
    odometry_msg.header.frame_id = "odom";
    odometry_msg.pose.pose.position.x = x;
    odometry_msg.pose.pose.position.y = y;
    odometry_msg.pose.pose.position.z = 0.0;
    odometry_msg.pose.pose.orientation = heading_quat;
    odometry_pub.publish(odometry_msg);

    // Broadcast TF
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

  /*
  Callback for reset service.
  Moves odom frame to requested pose TF. 
  Resets x, y, theta to 0 and broadcasts corresponding odom->baselink TF.*/
  bool resetCallback(project1::Reset::Request &req, project1::Reset::Response &res){
    
    // Extract requested heading quaternion
    tf2::Quaternion quat_tf; 
    quat_tf.setEuler(0, 0, req.new_theta);
    geometry_msgs::Quaternion heading_quat = tf2::toMsg(quat_tf);

    // Move odom frame to requested pose
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.frame_id = "world";
    odom_tf.child_frame_id = "odom";
    odom_tf.transform.translation.x = req.new_x;
    odom_tf.transform.translation.y = req.new_y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = heading_quat;
    tf_br.sendTransform(odom_tf);

    // Reset odometry variables and base_link frame
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    quat_tf.setEuler(0, 0, theta);
    heading_quat = tf2::toMsg(quat_tf);

    geometry_msgs::TransformStamped baselink_tf;
    baselink_tf.header.frame_id = "odom";
    baselink_tf.child_frame_id = "base_link";
    baselink_tf.transform.translation.x = x;
    baselink_tf.transform.translation.y = y;
    baselink_tf.transform.translation.z = 0.0;
    baselink_tf.transform.rotation = heading_quat;
    tf_br.sendTransform(baselink_tf);

    ROS_INFO("Odometry is reset to: \nx: [%f] \ny: [%f] \ntheta: [%f]", req.new_x, req.new_y, req.new_theta);
    return true;
  }

  /*
  Callback for dynamic parameter reconfiguring.*/
  void paramCallback(project1::parametersConfig& config, uint32_t level){
    integrator = config.integrator;
    r = config.r;
    l = config.l;
    w = config.w;
    T = config.T;
    N = config.N;

    ROS_INFO("Parameter Update in Odometry Node:");
    ROS_INFO("**********************************");
    if (config.integrator == Integrator_method::Euler){
      ROS_INFO("Integrator: Euler");
    }
    else if (config.integrator == Integrator_method::RK2){
      ROS_INFO("Integrator: RK2");
    }
    ROS_INFO("r: [%f]",r);
    ROS_INFO("l: [%f]",l);
    ROS_INFO("w: [%f]",w);
    ROS_INFO("T: [%f]",T);
    ROS_INFO("N: [%i]",N);
    ROS_INFO("**********************************");
  }


  public:

  Odometry_node(){
    x     = 0.0;
    y     = 0.0;
    theta = 0.0;

    no_previous_encoder_msg = true;
    
    odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    encoder_sub  = n.subscribe("wheel_states", 1000, &Odometry_node::encoderCallback, this);
  
    service = n.advertiseService("reset", &Odometry_node::resetCallback, this);
   
    f = boost::bind(&Odometry_node::paramCallback, this, _1, _2);
    dynServer.setCallback(f);
  }

  void run_main(){
    double x_init;
    double y_init;
    double theta_init;
  
    // Retrieve parameters set in launch file
    ros::param::get("r",r);
    ros::param::get("l",l);
    ros::param::get("w",w);
    ros::param::get("T",T);
    ros::param::get("N",N);
    ros::param::get("x_init",x_init);
    ros::param::get("y_init",y_init);
    ros::param::get("theta_init",theta_init);
  
    // Convert heading to correct quaternion type
    tf2::Quaternion quat_tf; 
    quat_tf.setEuler(0, 0, theta_init);
    geometry_msgs::Quaternion heading_quat = tf2::toMsg(quat_tf);
  
    // Broadcast world->odom TF based on init parameters
    geometry_msgs::TransformStamped tfStamped;
    tfStamped.header.frame_id = "world";
    tfStamped.child_frame_id = "odom";
    tfStamped.transform.translation.x = x_init;
    tfStamped.transform.translation.y = y_init;
    tfStamped.transform.translation.z = 0.0;
    tfStamped.transform.rotation = heading_quat;
    tf_br.sendTransform(tfStamped);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_node");

  Odometry_node node;

  node.run_main();

  return 0;
}
