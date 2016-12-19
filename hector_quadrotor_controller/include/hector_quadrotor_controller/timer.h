#ifndef HECTOR_QUADROTOR_CONTROLLER_TIMER
#define HECTOR_QUADROTOR_CONTROLLER_TIMER

#include <ros/node_handle.h>

namespace hector_quadrotor_controller{

class Timer
{
public:
  struct states{
   states();
   bool started;
   double value;
   double value_integral;
  } states_;

public:
    Timer();
    ~Timer();

void start();
bool count(const double Duration, const ros::Duration& dt);
bool pulse(const double Duration, const ros::Duration& dt);
void integrate(const ros::Duration& dt);
void reset();
};

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_TIMER
