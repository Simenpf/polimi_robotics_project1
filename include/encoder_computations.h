#include <vector>

// Calculate robot velocity [m/s] in local x direction from wheel speeds [rad/s]
// 'speeds' have the structure [front_left, front_right, rear_left, rear_right]
double vxFromWheelSpeeds(std::vector<double> speeds, double wheel_radius);

// Calculate robot velocity [m/s] in local y direction from wheel speeds [rad/s]
// 'spees' have the structure [front_left, front_right, rear_left, rear_right]
double vyFromWheelSpeeds(std::vector<double> speeds, double wheel_radius);

// Calculate robot angular velocity [rad/s] from wheel speeds [rad/s]
// 'speeds' have the structure [front_left, front_right, rear_left, rear_right]
// 'l': length [m] from center of robot to center of wheel along x axis
// 'w': width [m] from center of robot to center of wheel along y axis
double omegaFromWheelSpeeds(std::vector<double> speeds, double wheel_radius, double l, double w);

// Convert encoder ticks 'd_tick' over time 'd_t' [s] to wheel speed [m/s]
// 'N': encoder ticks per revolution
// 'T': gear ratio
double wheelSpeedFromTicks(double d_tick, double d_t, int N, double T);