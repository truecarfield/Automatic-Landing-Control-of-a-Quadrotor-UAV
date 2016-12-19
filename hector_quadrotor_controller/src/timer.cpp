#include <hector_quadrotor_controller/timer.h>
#include <limits>

#include <hector_quadrotor_controller/pid.h>
namespace hector_quadrotor_controller{

Timer::states::states()
    : started(false)
    , value(0.0)
    , value_integral(0.0)
{
}

Timer::Timer()
{}

Timer::~Timer()
{}

void Timer::start()
{
    if (!states_.started) {states_.started = true;}
}

bool Timer::count(const double Duration, const ros::Duration& dt)
{
    bool result;
    if (states_.started) {states_.value += dt.toSec();} else {states_.value = 0.0;}
    if (states_.value >= Duration) { result = true; states_.value = 0.0; states_.started = false;} else {result = false;}
    return result;
}

bool Timer::pulse(const double Duration, const ros::Duration& dt)
{
    bool result;
    if (states_.started)
    {states_.value += dt.toSec(); states_.value_integral += dt.toSec();}
    else
    {states_.value = 0.0; states_.value_integral = 0.0;}
    if (states_.value >= Duration) { result = true; states_.value = 0.0;} else {result = false;}
    return result;
}

void Timer::reset()
{
    if (states_.started) {states_= states();}
}

}
