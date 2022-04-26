#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include "math.h"

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



double vxFromWheelSpeeds(std::vector<double> speeds){
    return (r/4)*(speeds[front_left]+speeds[front_right]+speeds[rear_left]+speeds[rear_right]);
}

double vyFromWheelSpeeds(std::vector<double> speeds){
    return (r/4)*(-speeds[front_left]+speeds[front_right]+speeds[rear_left]-speeds[rear_right]);   
}

double omegaFromWheelSpeeds(std::vector<double> speeds){
  return (r/(4*(l+w)))*(-speeds[front_left]+speeds[front_right]-speeds[rear_left]+speeds[rear_right]);
}

double wheelSpeedFromTicks(double d_tick, double d_t){
  return (d_tick*2*M_PI)/(N*T*d_t);
}

void encoderDataCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // Gather data from the encoder message
  std::vector<double> new_wheel_ticks = msg->position;
  std::vector<double> true_speeds = msg->velocity;

  // Initiate temporary variables
  std::vector<double> speeds(4,0.0);
  ros::Time new_time = ros::Time::now();
  double d_t = (new_time-prev_time).toSec();

  // Calculate wheel speeds from encoder ticks
  for (int i = 0; i < speeds.size(); i++){
    double d_tick = new_wheel_ticks[i] - prev_wheel_ticks[i];
    speeds[i] = wheelSpeedFromTicks(d_tick,d_t);
  }

  // Update global variables and flags  
  prev_wheel_ticks = new_wheel_ticks;
  prev_time = new_time;
  if(no_previous_encoder_msg){
    no_previous_encoder_msg = false;
    return;
  }

  // Calculate odometry
  double vx = vxFromWheelSpeeds(speeds);
  double vy = vyFromWheelSpeeds(speeds);
  double omega = omegaFromWheelSpeeds(speeds);

  // Euler integration
  x += (vx*cos(theta) - vy*sin(theta)) * d_t;
  y += (vx*sin(theta) + vy*cos(theta)) * d_t;
  theta += omega*d_t;

  // TODO: publish pos, compare with GT using e.g. rqt_graph

  // RK2 integration


  // Publish results
  geometry_msgs::TwistStamped out_msg;
  out_msg.twist.linear.x  = vx;
  out_msg.twist.linear.y  = vy;
  out_msg.twist.angular.z = omega;
  odometry_pub.publish(out_msg);

  // Display results for debugging
  ROS_INFO("x: [%f]", x);
  ROS_INFO("y: [%f]", y);
  ROS_INFO("theta: [%f]", theta);
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  prev_time = ros::Time::now();

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
  ros::Subscriber encoder_sub = n.subscribe("wheel_states", 1000, encoderDataCallback);


  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
