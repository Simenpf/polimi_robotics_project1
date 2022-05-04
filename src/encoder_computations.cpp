#include "encoder_computations.h"
#include "math.h"

enum Wheels {front_left, front_right, rear_left, rear_right};

double vxFromWheelSpeeds(std::vector<double> speeds, double wheel_radius){
    return (wheel_radius/4) * (speeds[front_left]+speeds[front_right]+speeds[rear_left]+speeds[rear_right]);
}

double vyFromWheelSpeeds(std::vector<double> speeds, double wheel_radius){
    return (wheel_radius/4) * (-speeds[front_left]+speeds[front_right]+speeds[rear_left]-speeds[rear_right]);   
}

double omegaFromWheelSpeeds(std::vector<double> speeds, double wheel_radius, double l, double w){
  return (wheel_radius / (4*(l+w)) ) * (-speeds[front_left]+speeds[front_right]-speeds[rear_left]+speeds[rear_right]);
}

double wheelSpeedFromTicks(double d_tick, double d_t, int N, double T){
  return (d_tick*2*M_PI)/(N*T*d_t);
}