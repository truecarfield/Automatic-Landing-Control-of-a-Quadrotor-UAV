#ifndef HECTOR_QUADROTOR_CONTROLLER_PID_H
#define HECTOR_QUADROTOR_CONTROLLER_PID_H

#include <ros/node_handle.h>

namespace hector_quadrotor_controller {

class PID
{
public:
  struct parameters {
    parameters();
    bool enabled;
    double time_constant;
    double k_p;
    double k_i;
    double k_d;
    double k_dd; // by ZD
    double limit_i;
    double limit_output;
  } parameters_;   // The struct parameters has a variable with the name parameters_

  struct state {
    state();
    double p, i, d, dd;  // by ZD
    double input, dinput;
    double dx, ddx;
  } state_;

public:
  PID();
  PID(const parameters& parameters);
  ~PID();

  void init(const ros::NodeHandle &param_nh);
  void reset();

  double update(double input, double x, double dx, const ros::Duration& dt);
  double update(double error, double dx, const ros::Duration& dt);

  double update_attitude(double input, double x, double dx, const ros::Duration& dt, const int& mode);                              // by ZD
  double update_attitude(double error, double dx, const ros::Duration& dt, const int& mode);                                               // by ZD
  double update_pos(double error, double d_error, double dd_error, const ros::Duration& dt, const int& mode);                                    // by ZD
  double update_test(double error, double d_error, double dd_error, const ros::Duration& dt);                                    // by ZD

  double getFilteredControlError(double& filtered_error, double time_constant, const ros::Duration& dt);
};

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_PID_H
