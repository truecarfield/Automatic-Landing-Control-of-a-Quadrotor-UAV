//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
#include "ros/ros.h"

#include "tf/tf.h"     //by ZD
#include "tf/transform_listener.h"
#include "eigen3/Eigen/Eigen"
#include <fstream>
#include <iostream>
//#include "random_numbers/random_numbers.h"
#include <ostream>
#include <hector_quadrotor_controller/quadrotor_interface.h>
#include <hector_quadrotor_controller/pid.h>
#include <hector_quadrotor_controller/timer.h> // by ZD
//#include <hector_quadrotor_controller/eiquadprog.hpp> // by ZD
#include <hector_quadrotor_controller/trajectory_prediction.hpp> // by ZD

#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <limits>

namespace hector_quadrotor_controller {

using namespace controller_interface;

class echoListener  // by ZD
 {
 public:
   tf::TransformListener tf;

   //constructor with name
   echoListener()
   {
    };
   ~echoListener()
   {
    };
private:
};

class LandingController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  LandingController()
  {}

  ~LandingController()
  {}

  bool init(QuadrotorInterface *interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {   
    // Wait for up to one second for the first transforms to become avaiable.   // by ZD
    echoListener_st_.tf.waitForTransform("base_stabilized", "heart", ros::Time(), ros::Duration(3.0));
    echoListener_w_.tf.waitForTransform("world", "base_link", ros::Time(), ros::Duration(3.0));
    echoListener_cam_.tf.waitForTransform("front_cam_optical_frame", "heart", ros::Time(), ros::Duration(3.0));
    echoListener_tar_w_.tf.waitForTransform("world", "heart", ros::Time(), ros::Duration(3.0));

    // example
    // using namespace Eigen;
    // foo<Eigen::VectorXd, Eigen::MatrixXd>();

    // get interface handles
    pose_  = interface->getPose();
    twist_  = interface->getTwist();
    //motor_status_=interface->getMotorStatus(); // by ZD
    imu_ = interface->getSensorImu(); // by ZD
    acceleration_  = interface->getAcceleration();
    twist_input_   = interface->addInput<TwistCommandHandle>("twist");
    wrench_output_ = interface->addOutput<WrenchCommandHandle>("wrench"); // The shared_ptr saved in the map as "wrench" will be changed into shared_ptr<WrenchCommandHandle>, and get the thing it points         by ZD
    node_handle_ = root_nh;

    // subscribe to commanded twist (geometry_msgs/TwistStamped) and cmd_vel (geometry_msgs/Twist)
    twist_subscriber_ = node_handle_.subscribe<geometry_msgs::TwistStamped>("command/twist", 1, boost::bind(&LandingController::twistCommandCallback, this, _1));  // it seems that it is not useds  by ZD
    cmd_vel_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&LandingController::cmd_velCommandCallback, this, _1));  // This one is from joystick
    noise_subscriber_ = node_handle_.subscribe<std_msgs::Float64>("gaussian_noise", 1, boost::bind(&LandingController::gaussian_noiseCallback, this, _1)); // by ZD
    //motor_status_subscriber_=node_handle_.subscribe<hector_uav_msgs::MotorStatus>("motor_command_status", 1, boost::bind(&LandingController::motor_command_statusCallback, this, _1)); // by ZD;

    // engage/shutdown service servers
    engage_service_server_ = node_handle_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("engage", boost::bind(&LandingController::engageCallback, this, _1, _2));    // it seems that this line has no use       by ZD
    shutdown_service_server_ = node_handle_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("shutdown", boost::bind(&LandingController::shutdownCallback, this, _1, _2));  // it seems that this line has no use       by ZD

    // initialize PID controllers
    pid_.linear.x.init(ros::NodeHandle(controller_nh, "linear/xy"));
    pid_.linear.y.init(ros::NodeHandle(controller_nh, "linear/xy"));
    pid_.linear.z.init(ros::NodeHandle(controller_nh, "linear/z"));
    pid_.angular.x.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.y.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.z.init(ros::NodeHandle(controller_nh, "angular/z"));

    pid_.angular_pos.x.init(ros::NodeHandle(controller_nh, "angular/xy")); // by ZD
    pid_.angular_pos.y.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular_pos.z.init(ros::NodeHandle(controller_nh, "angular_pos/z"));
    pid_.angular_lyapnov.x.init(ros::NodeHandle(controller_nh, "angular_lyapnov/xy"));
    pid_.angular_lyapnov.y.init(ros::NodeHandle(controller_nh, "angular_lyapnov/xy"));

    pid_.linear_pos.x.init(ros::NodeHandle(controller_nh, "linear_pos/xy"));
    pid_.linear_pos.y.init(ros::NodeHandle(controller_nh, "linear_pos/xy"));
    pid_.linear_pos.z.init(ros::NodeHandle(controller_nh, "linear_pos/z"));

    // load other parameters
    controller_nh.getParam("auto_engage", auto_engage_ = true);
    controller_nh.getParam("limits/load_factor", load_factor_limit = 1.5);
    controller_nh.getParam("limits/force/z", limits_.force.z);
    controller_nh.getParam("limits/torque/xy", limits_.torque.x);
    controller_nh.getParam("limits/torque/xy", limits_.torque.y);
    controller_nh.getParam("limits/torque/z", limits_.torque.z);
    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

    C_wxy = 0.12;
    C_wz = 0.1;
    C_mxy = 0.074156208000000;
    C_mz = 0.050643264000000;
    C_drag_T << C_mxy, 0.0,     0.0,
                          0.0,      C_mxy, 0.0,
                          0.0,      0.0,     C_mz;

    pi = 3.1415926;
    gravity = 9.8065;

    // get parameters for motor propulsion model
    force_per_voltage = 0.559966216;
    torque_per_voltage = 7.98598e-3;
    lever = 0.275;
    motor_command_.force.assign(4, 0.0);
    motor_command_.torque.assign(4, 0.0);
    motor_command_.frequency.clear();
    motor_command_.voltage.assign(4, 0.0);

    // get mass and inertia from QuadrotorInterface
    interface->getMassAndInertia(mass_, inertia_);
    inertia_Eigen_<< inertia_[0], 0.0,             0.0,      // by ZD
                                 0.0,             inertia_[1], 0.0,
                                 0.0,             0.0,             inertia_[2];
             RPY_old << 0.0,   0.0,   0.0;
    dot_Omega_c << 0.0,   0.0,   0.0;
              T_cmdf  << 100.0, 0.0,   0.0,
                                  0.0,  100.0,  0.0,
                                  0.0,  0.0,    100.0;
              T_cmdf_1  << 20.0, 0.0,   0.0,
                                  0.0,  20.0,  0.0,
                                  0.0,  0.0,    20.0;
              T_cmdf_2  << 20.0, 0.0,   0.0,
                                      0.0,  20.0, 0.0,
                                      0.0,  0.0,  2.0;
              c4 <<10.0, 0.0, 0.0,
                        0.0, 10.0, 0.0,
                        0.0, 0.0, 10.0;

    P_test_e << 0.0,0.0,5.0;

    mode = 0;

    timer_test_started=false;
    timer_.timer_test_.reset();
    timer_.timer_target_lost_.reset();
    timer_.timer_tracking_permission_.reset();
    timer_.timer_predictor_tar_.reset();
    timer_.timer_landing_permitssion_.reset();

    tracking_permitted_ = false;
    trajectory_predictor_tar_x_.reset();
    trajectory_predictor_tar_y_.reset();

    once_and_for_all = false;
    trajectory_tar_predicted_=false;
    landed_ = true;

//    rise_done = false;
//    control_done = false;
//    overshoot_done = false;
//    overshoot_done = false;
//    control_about_done = false;

    SwitchMethod[0] = &LandingController::TwistControl;
    SwitchMethod[1] = &LandingController::TrackingControl;
    SwitchMethod[2] = &LandingController::PathFollowing;
    SwitchMethod[3] = &LandingController::LandingControl;

    command_given_in_stabilized_frame_ = false;

    return true;
  }

  void reset()
  {
    // timer_.timer_test_.reset();
    // timer_test_started=false;
    timer_.timer_test_.reset();
    timer_.timer_target_lost_.reset();
    timer_.timer_tracking_permission_.reset();
    timer_.timer_predictor_tar_.reset();
    timer_.timer_landing_permitssion_.reset();

    tracking_permitted_ = false;
    trajectory_predictor_tar_x_.reset();
    trajectory_predictor_tar_y_.reset();

    once_and_for_all = false;
    trajectory_tar_predicted_=false;
    landed_ = true;

    pid_.linear.x.reset();
    pid_.linear.y.reset();
    pid_.linear.z.reset();
    pid_.angular.x.reset();
    pid_.angular.y.reset();
    pid_.angular.z.reset();

    wrench_.wrench.force.x  = 0.0;
    wrench_.wrench.force.y  = 0.0;
    wrench_.wrench.force.z  = 0.0;
    wrench_.wrench.torque.x = 0.0;
    wrench_.wrench.torque.y = 0.0;
    wrench_.wrench.torque.z = 0.0;

    motor_command_.voltage[1]=0.0;
    motor_command_.voltage[2]=0.0;
    motor_command_.voltage[3]=0.0;
    motor_command_.voltage[4]=0.0;

    Omega_c << 0.0, 0.0, 0.0;

    linear_z_control_error_ = 0.0;
    motors_running_ = false;

    motor_command_.force.assign(4, 0.0);
    motor_command_.torque.assign(4, 0.0);
    motor_command_.frequency.clear();
    motor_command_.voltage.assign(4, 0.0);
  }

  void twistCommandCallback(const geometry_msgs::TwistStampedConstPtr& command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    command_ = *command;
    if (command_.header.stamp.isZero()) command_.header.stamp = ros::Time::now();
    command_given_in_stabilized_frame_ = false;

    // start controller if it not running
    if (!isRunning()) this->startRequest(command_.header.stamp);   // this = *LandingController
  }

  void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr& command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    command_.twist = *command;
    command_.header.stamp = ros::Time::now();
    command_given_in_stabilized_frame_ = true;

    // start controller if it not running
    if (!isRunning()) this->startRequest(command_.header.stamp);
  }

  void gaussian_noiseCallback(const std_msgs::Float64::ConstPtr& noise_signal) // by ZD
  {
    noise = (noise_signal->data);
  }

  //  void motor_command_statusCallback(const hector_uav_msgs::MotorStatusConstPtr& motor_command_status) // by ZD
  //  {
  //    motor_command_status_.voltage = motor_command_status->voltage;
  //    motor_command_status_.current = motor_command_status->current;
  //    motor_command_status_.frequency = motor_command_status->frequency;
  //  }

  //void imuCallback(const sensor_msgs::ImuConstPtr &imu) {   // by ZD
  //  imu_ = *imu;
  // }

  bool engageCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("landing_controller", "Engaging motors!");
    motors_running_ = true;
    return true;
  }

  bool shutdownCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("landing_controller", "Shutting down motors!");
    motors_running_ = false;
    return true;
  }

  void starting(const ros::Time &time)
  {
    reset();
    wrench_output_->start();
  }

  void stopping(const ros::Time &time)
  {
    wrench_output_->stop();
  }

  void TwistControl(const ros::Duration& period)  // by ZD
  {
      /* Speed controller */
      acceleration_command.x = pid_.linear.x.update(command.linear.x, twist.linear.x, acceleration_->acceleration().x, period);
      acceleration_command.y = pid_.linear.y.update(command.linear.y, twist.linear.y, acceleration_->acceleration().y, period);
     // acceleration_command.z = pid_.linear.z.update(command.linear.z, twist.linear.z, acceleration_->acceleration().z, period) + gravity;
      acceleration_command_body = pose_->toBody(acceleration_command);

      // Temporary testing program
      double twist_c_z = pid_.linear_pos.z.update_test(7.0 - quadrotor_pos_w.getZ(), 0.0 - twist.linear.z, 0.0, period); // by ZD
      if (twist_c_z > 2.0) {twist_c_z = 2.0;}
      if (twist_c_z < -2.0) {twist_c_z = -2.0;}
      acceleration_command.z = pid_.linear.z.update(twist_c_z, twist.linear.z, acceleration_->acceleration().z, period) + gravity;

      ROS_DEBUG_STREAM_NAMED("landing_controller", "twist.linear:               [" << twist.linear.x << " " << twist.linear.y << " " << twist.linear.z << "]");
      ROS_DEBUG_STREAM_NAMED("landing_controller", "twist_body.angular:         [" << twist_body.angular.x << " " << twist_body.angular.y << " " << twist_body.angular.z << "]");
      ROS_DEBUG_STREAM_NAMED("landing_controller", "twist_command.linear:       [" << command.linear.x << " " << command.linear.y << " " << command.linear.z << "]");
      ROS_DEBUG_STREAM_NAMED("landing_controller", "twist_command.angular:      [" << command.angular.x << " " << command.angular.y << " " << command.angular.z << "]");
      ROS_DEBUG_STREAM_NAMED("landing_controller", "acceleration:               [" << acceleration_->acceleration().x << " " << acceleration_->acceleration().y << " " << acceleration_->acceleration().z << "]");
      ROS_DEBUG_STREAM_NAMED("landing_controller", "acceleration_command_world: [" << acceleration_command.x << " " << acceleration_command.y << " " << acceleration_command.z << "]");
      ROS_DEBUG_STREAM_NAMED("landing_controller", "acceleration_command_body:  [" << acceleration_command_body.x << " " << acceleration_command_body.y << " " << acceleration_command_body.z << "]");

      wrench_.wrench.torque.x = inertia_[0] * pid_.angular.x.update( -acceleration_command_body.y / gravity, roll, twist_body.angular.x, period);  // twist_body.angular.x is the pitch RATE!!!! Not angle Kp = 10, Kd = 5, Ki = 5 time_const = 0.01
      wrench_.wrench.torque.y = inertia_[1] * pid_.angular.y.update( acceleration_command_body.x / gravity, pitch, twist_body.angular.y, period);
      //wrench_.wrench.torque.x = inertia_[0] * pid_.angular.x.update( -acceleration_command_body.y / gravity, 0.0, twist_body.angular.x, period);  // original
      //wrench_.wrench.torque.y = inertia_[1] * pid_.angular.y.update( acceleration_command_body.x / gravity, 0.0, twist_body.angular.y, period);
      wrench_.wrench.torque.z = inertia_[2] * pid_.angular.z.update( command.angular.z, twist.angular.z, 0.0, period); // original
      wrench_.wrench.force.x  = 0.0;
      wrench_.wrench.force.y  = 0.0;
      wrench_.wrench.force.z  = mass_ * ((acceleration_command.z - gravity) * load_factor + gravity);
  }

  void TrackingControl(const ros::Duration& period)
  {

      delta_P << P_test_e(0) - quadrotor_pos_w.getX(),   P_test_e(1) - quadrotor_pos_w.getY(),  P_test_e(2) - quadrotor_pos_w.getZ();
      // delta_P << 5.0 - quadrotor_pos_w.getX(),  5.0 - quadrotor_pos_w.getY(), 5.0  - quadrotor_pos_w.getZ();
      /* PT2 filter */
      dotdot_delta_P_f = -2.0*T_cmdf_2*dot_delta_P_f - T_cmdf_2*T_cmdf_2*(delta_P_f - delta_P);
      dot_delta_P_f += dotdot_delta_P_f*period.toSec();
      delta_P_f  += dot_delta_P_f*period.toSec();
      //dot_delta_P = dot_delta_P_f;
      dot_delta_P += -T_cmdf_2*(dot_delta_P - dot_delta_P_f)*period.toSec();
      //dot_delta_P = (delta_P_f - delta_P_old)/period.toSec();
      //delta_P_old =delta_P_f;

      /* Trajectory and landing controller */
      LinAccel_c(0) = pid_.linear_pos.x.update_pos( delta_P_f(0), dot_delta_P_f(0), 0.0, period, mode);// + C_mxy*twist.linear.x*sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y + twist.linear.z*twist.linear.z);
      LinAccel_c(1) = pid_.linear_pos.y.update_pos( delta_P_f(1), dot_delta_P_f(1), 0.0, period, mode);// + C_mxy*twist.linear.y*sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y + twist.linear.z*twist.linear.z);
      double twist_c_z = pid_.linear_pos.z.update_pos(delta_P_f(2), dot_delta_P_f(2), 0.0, period, mode);
      if (twist_c_z > 2.0) {twist_c_z = 2.0;}
      if (twist_c_z < -2.0) {twist_c_z = -2.0;}
      LinAccel_c(2) = pid_.linear.z.update(twist_c_z, twist.linear.z, acceleration_->acceleration().z, period);

      RPY_c(0) = asin((LinAccel_c(0)*sin(RPY_c(2)) - LinAccel_c(1)*cos(RPY_c(2)))/sqrt( LinAccel_c(0)*LinAccel_c(0) + LinAccel_c(1)*LinAccel_c(1) + (gravity)*(gravity)));
      RPY_c(1) = atan2(LinAccel_c(0)*cos(RPY_c(2)) + LinAccel_c(1)*sin(RPY_c(2)), gravity); // In this one there is no Accel(2) part in the denominator
//      if (delta_P_st(0)*delta_P_st(0) + delta_P_st(1)*delta_P_st(1) > 1.0)
//      {
//            RPY_c(2) = yaw+atan2(relativ_pos_st.getY(), relativ_pos_st.getX());
            RPY_c(2) = 0.0;
//      }
//      else
//     {
//           RPY_c(2) = yaw + yaw_tar;
//           RPY_c(2) = 0.0;
//     }

     /* RPY_c limitation
     double l_star_front = 0.0, l_star_back = 0.0, l = 0.13, P = 0.0, Alpha_front = pi/2.5, Alpha_back = pi - Alpha_front, Beta = 64.5*0.5*pi/180,   Gamma_front = Alpha_front - Beta + pi/2,   Gamma_back = Alpha_back - Beta + pi/2 -Beta, pitch_max = 0.0, pitch_min = 0.0;
     if (tracking_permitted_)
     {
          P = sqrt(relativ_pos_st.getX()*relativ_pos_st.getX() + relativ_pos_st.getZ()*relativ_pos_st.getZ());
          l_star_front = l*cos(Gamma_front) + sqrt(P*P - sin(Gamma_front)*sin(Gamma_front)*l*l);
          pitch_max  = acos((P*P + l*l - l_star_front*l_star_front)/(2*P*l)) - atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ());
          if (pitch_max > pi/2) {pitch_max = pi/2;}
          if (pitch_max < 0.0) {pitch_max = 0;}
          // std::cout<<"pitch_max = "<<acos((P*P + l*l - l_star_front*l_star_front)/(2*P*l)) <<" - "<< atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ()) <<" = " << pitch_max <<std::endl;

          l_star_back = l*cos(Gamma_back) + sqrt(P*P - sin(Gamma_back)*sin(Gamma_back)*l*l);
          pitch_min = acos((P*P + l*l - l_star_back*l_star_back)/(2*P*l)) + atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ());
          if (pitch_min > pi/2) {pitch_min = pi/2;}
          if (pitch_min < 0.0) {pitch_min = 0.0;}

        // if (RPY_c(1) > pitch_max) {RPY_c(1) = pitch_max;}
        // if (RPY_c(1) < -pitch_min) {RPY_c(1) = -pitch_min;}
     }

     /* check how well the attitude control is done
     RPY_c << 0.0, 0.0, 0.0;
     if (time.toSec() > 5.0)
     {
         RPY_c << 0.0, 0.3, 0.0;
         if (fabs(RPY_c(1) - pitch) <= RPY_c(1)*0.05 && !rise_done)
         {
              rise_done = true;
              rise_time = time.toSec() - 5.0;
              std::cout<<"rise_time is: "<<rise_time<<std::endl;
         }

         if (rise_done && fabs(RPY_c(1) - pitch) <= RPY_c(1)*0.05 && !control_done && !control_about_done)
         {
              settling_time = time.toSec() - 5.0;
              control_about_done = true;
         }
         if (control_about_done && fabs(RPY_c(1) - pitch) > RPY_c(1)*0.05) {control_about_done = false;}
         if (control_about_done && time.toSec() > 10.0 && fabs(RPY_c(1) - pitch) <= RPY_c(1)*0.05 && !control_done)
         {
              control_done = true;
              std::cout<<"settling_time is: "<<settling_time<<std::endl;
         }

         if (pitch - RPY_c(1) > 0.0) {overshoot = true;}
         if (rise_done  && overshoot && pitch - RPY_c(1) < error_old && !overshoot_done)
         {
             overshoot_done = true;
             overshoot_rate = error_old/RPY_c(1)*100;
             std::cout<<"overshoot_rate is: "<<overshoot_rate<<" %"<<std::endl;
         }
        error_old = pitch - RPY_c(1);
    }

       /* Lyapnov attitude controller */
       W << 1.0,   sin(roll)*tan(pitch),   cos(roll)*tan(pitch),
                 0.0,   cos(roll),                    -sin(roll),
                 0.0,   sin(roll)/cos(pitch),   cos(roll)/cos(pitch);

       c1 << 35.0, 0.0, 0.0,  // 35
                 0.0, 35.0, 0.0,
                 0.0, 0.0, 0.0;
       c2 << 35.0, 0.0, 0.0,  // 35
                 0.0, 35.0, 0.0,
                 0.0, 0.0, 0.0;
       c3 << 50.0, 0.0, 0.0,
                 0.0, 50.0, 0.0,
                 0.0, 0.0, 10.0;

      //dot_RPY = W*Omega;

     // T_drag = C_drag_T*Omega*Omega.norm();
     z1 = RPY - RPY_c;
     z1_int += z1*period.toSec();
     z1_old = z1;
     Omega_d = W.inverse()*( dot_RPY - c1*z1);
     // dot_Omega_c = (Omega_d - Omega_c)/period.toSec();
     dot_Omega_c = -T_cmdf*(Omega_c - Omega_d);
     Omega_c += dot_Omega_c*period.toSec();
     //Omega_c = Omega_d;

     z2 = Omega - Omega_c;
     Ypsilon += (-c1*Ypsilon + W*(Omega_c - Omega_d))*period.toSec();
     z1_mod = z1 - Ypsilon;

     /* Control effort of Lyapnov attitude controller */
     Torque = Omega.cross(inertia_Eigen_*Omega) + inertia_Eigen_*dot_Omega_c - inertia_Eigen_*(W.transpose())*z1_mod - inertia_Eigen_*c2*z2 - inertia_Eigen_*W.transpose()*c3*z1_int;

     /* Control effort of hybrid Lyapnov attitude controller */
     // horizonal: Kp = 10, Kd = 0.2, Ki = 1, settling time = 0.122 s, overshoot = 0.6%, vertical: Kp = 15, Kd = 2, Ki = 2;  if mode != 1 then no integration will be made (anti-windup)

     // Torque_star << pid_.angular_lyapnov.x.update_attitude(RPY_c(0), roll, Omega(0), period, mode), pid_.angular_lyapnov.y.update_attitude(RPY_c(1), pitch, Omega(1), period, mode), 0.0;
     // Torque = Omega.cross(inertia_Eigen_*Omega) - inertia_Eigen_*c4*Omega + c4*Torque_star;
     Torque(2) = pid_.angular_pos.z.update_attitude(RPY_c(2), yaw, dot_RPY(2), period, mode);

     wrench_.wrench.torque.x = Torque(0);
     wrench_.wrench.torque.y = Torque(1);
     wrench_.wrench.torque.z = Torque(2);
     wrench_.wrench.force.x  = 0.0;
     wrench_.wrench.force.y  = 0.0;
     //wrench_.wrench.force.z  = mass_ * (pid_.linear.z.update(command.linear.z, twist.linear.z, acceleration_->acceleration().z, period) * load_factor + gravity);
     wrench_.wrench.force.z  = mass_ * (LinAccel_c(2) * load_factor + gravity);
  }

  void PathFollowing(const ros::Duration& period)
  {
      /* However the absolute target position will be nolonger calculated via relative position but predicted polynom */
      P_tar_e_temp << 0.0, 0.0, 0.0;
      for (int i = 0; i < Coefs_tar_x.size(); i++)
      {
           P_tar_e_temp(0) += Coefs_tar_x[i]*pow(timer_.timer_predictor_tar_.states_.value_integral, i);
      }
      for (int i = 0; i < Coefs_tar_y.size(); i++)
      {
           P_tar_e_temp(1) += Coefs_tar_y[i]*pow(timer_.timer_predictor_tar_.states_.value_integral, i);
           //std::cout<<"Term "<<i<<" is "<<Coefs_tar_y[i]*pow(timer_.timer_predictor_tar_.states_.value_integral, i)<<",  P_tar_e_temp1(1) is "<< P_tar_e_temp(1) <<std::endl;
      }
      time_z += period.toSec();
      P_tar_e_temp(2) = 1.15 - Coef_z*(time_z - 3)*(time_z - 3);
      P_tar_e = P_tar_e_temp;
      delta_P = P_tar_e - P_e;

      /* Alternatively the target position in 3 seconds later can be predicted only once */
      /* if (!once_and_for_all)
      {
          for (int i = 0; i < Coefs_tar_x.size(); i++)
          {
              P_tar_e_once_(0) += Coefs_tar_x[i]*pow(time_following_start + 3.0, i);
          }
          for (int i = 0; i < Coefs_tar_y.size(); i++)
          {
              P_tar_e_once_(1) += Coefs_tar_y[i]*pow(time_following_start + 3.0, i);
          }
          P_tar_e = P_e;
          twist_cmd_once_ = (P_tar_e_once_ - P_e)/3;
          once_and_for_all = true;
          std::cout<<"P_tar_e once is "<<std::endl;
          std::cout<<P_tar_e_once_<<std::endl;
          std::cout<<"P_e is "<<std::endl;
          std::cout<<P_e<<std::endl;
          std::cout<<"P_tar_e - P_e is "<<std::endl;
          std::cout<<P_tar_e_once_ - P_e<<std::endl;
          std::cout<<"delta_P is "<<std::endl;
          std::cout<<delta_P<<std::endl;
      }
      P_tar_e(0) = P_following_start_(0) + twist_cmd_once_(0)*period.toSec();
      P_tar_e(1) = P_following_start_(1) + twist_cmd_once_(1)*period.toSec();
      P_tar_e(2) = delta_P(2); */

      /* Path following last for 3 seconds */
      if (timer_.timer_predictor_tar_.pulse(3.0, period))
      {
          std::cout<<"P_tar_e"<<std::endl;
          std::cout<<P_tar_e(0)<<std::endl;
          std::cout<<"P_tar_real"<<std::endl;
          std::cout<<P_tar_real(0)<<std::endl;
          mode = 3;
          timer_.timer_predictor_tar_.reset();
          P_tar_e_temp << 0.0, 0.0, 0.0;
          ROS_INFO_NAMED("landing_controller", "Followed the predicted path for 3 sec, target is in the field of view, switching to landing mode");
      }

      /* PT2 filter */
      dotdot_delta_P_f = -2.0*T_cmdf_2*dot_delta_P_f - T_cmdf_2*T_cmdf_2*(delta_P_f - delta_P);
      dot_delta_P_f += dotdot_delta_P_f*period.toSec();
      delta_P_f  += dot_delta_P_f*period.toSec();
      //dot_delta_P = dot_delta_P_f;
      dot_delta_P += -T_cmdf_2*(dot_delta_P - dot_delta_P_f)*period.toSec();
      //dot_delta_P = (delta_P_f - delta_P_old)/period.toSec();
      //delta_P_old =delta_P_f;

      /* Trajectory and landing controller */
      LinAccel_c(0) = pid_.linear_pos.x.update_pos( delta_P_f(0), dot_delta_P_f(0), 0.0, period, mode);// + C_mxy*twist.linear.x*sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y + twist.linear.z*twist.linear.z);
      LinAccel_c(1) = pid_.linear_pos.y.update_pos( delta_P_f(1), dot_delta_P_f(1), 0.0, period, mode);// + C_mxy*twist.linear.y*sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y + twist.linear.z*twist.linear.z);

      double twist_c_z = pid_.linear_pos.z.update_pos(delta_P_f(2), dot_delta_P_f(2), 0.0, period, mode);
      if (twist_c_z > 2.0) {twist_c_z = 2.0;}
      if (twist_c_z < -2.0) {twist_c_z = -2.0;}
      LinAccel_c(2) = pid_.linear.z.update(twist_c_z, twist.linear.z, acceleration_->acceleration().z, period);

      if (delta_P_st(0)*delta_P_st(0) + delta_P_st(1)*delta_P_st(1) > 1.0)
      {
            RPY_c(2) = yaw+atan2(relativ_pos_st.getY(), relativ_pos_st.getX());
            //RPY_c(2) = 0.0;
      }
      else
     {
           RPY_c(2) = yaw + yaw_tar;
           //RPY_c(2) = 0.0;
     }
     RPY_c(0) = asin((LinAccel_c(0)*sin(RPY_c(2)) - LinAccel_c(1)*cos(RPY_c(2)))/sqrt( LinAccel_c(0)*LinAccel_c(0) + LinAccel_c(1)*LinAccel_c(1) + (gravity)*(gravity)));
     RPY_c(1) = atan2(LinAccel_c(0)*cos(RPY_c(2)) + LinAccel_c(1)*sin(RPY_c(2)), gravity); // In this one there is no Accel(2) part in the denominator

     // pitch limitation
//     double l_star_front = 0.0, l_star_back = 0.0, l = 0.13, P = 0.0, Alpha_front = pi/2.5, Alpha_back = pi - Alpha_front, Beta = 85*0.5*pi/180,   Gamma_front = Alpha_front - Beta + pi/2,   Gamma_back = Alpha_back - Beta + pi/2 -Beta, pitch_max = 0.0, pitch_min = 0.0;
//     if (tracking_permitted_)
//     {
//          P = sqrt(relativ_pos_st.getX()*relativ_pos_st.getX() + relativ_pos_st.getZ()*relativ_pos_st.getZ());
//          l_star_front = l*cos(Gamma_front) + sqrt(P*P - sin(Gamma_front)*sin(Gamma_front)*l*l);
//          pitch_max  = acos((P*P + l*l - l_star_front*l_star_front)/(2*P*l)) - atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ());
//          if (pitch_max > pi/2) {pitch_max = pi/2;}
//          if (pitch_max < 0.0) {pitch_max = 0;}
//          // std::cout<<"pitch_max = "<<acos((P*P + l*l - l_star_front*l_star_front)/(2*P*l)) <<" - "<< atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ()) <<" = " << pitch_max <<std::endl;

//          l_star_back = l*cos(Gamma_back) + sqrt(P*P - sin(Gamma_back)*sin(Gamma_back)*l*l);
//          pitch_min = acos((P*P + l*l - l_star_back*l_star_back)/(2*P*l)) + atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ());
//          if (pitch_min > pi/2) {pitch_min = pi/2;}
//          if (pitch_min < 0.0) {pitch_min = 0.0;}

//          if (RPY_c(1) > pitch_max) {RPY_c(1) = pitch_max;}
//          //if (RPY_c(1) < -pitch_min) {RPY_c(1) = -pitch_min;}
//     }

       /* Lyapnov attitude controller */
       W << 1.0,   sin(roll)*tan(pitch),   cos(roll)*tan(pitch),
                 0.0,   cos(roll),                    -sin(roll),
                 0.0,   sin(roll)/cos(pitch),   cos(roll)/cos(pitch);

       c1 << 35.0, 0.0, 0.0,  // 25   30
                 0.0, 35.0, 0.0,
                 0.0, 0.0, 0.0;
       c2 << 35.0, 0.0, 0.0,  // 50   30
                 0.0, 35.0, 0.0,
                 0.0, 0.0, 0.0;
       c3 << 50.0, 0.0, 0.0,
                 0.0, 50.0, 0.0,
                 0.0, 0.0, 10.0;

      //dot_RPY = W*Omega;

     // T_drag = C_drag_T*Omega*Omega.norm();
     z1 = RPY - RPY_c;
     z1_int += z1*period.toSec();
     z1_old = z1;
     Omega_d = W.inverse()*( dot_RPY - c1*z1);
     // dot_Omega_c = (Omega_d - Omega_c)/period.toSec();
     dot_Omega_c = -T_cmdf*(Omega_c - Omega_d);
     Omega_c += dot_Omega_c*period.toSec();
     //Omega_c = Omega_d;

     z2 = Omega - Omega_c;
     Ypsilon += (-c1*Ypsilon + W*(Omega_c - Omega_d))*period.toSec();
     z1_mod = z1 - Ypsilon;

     /* Control effort of Lyapnov attitude controller */
    Torque = Omega.cross(inertia_Eigen_*Omega) + inertia_Eigen_*dot_Omega_c - inertia_Eigen_*(W.transpose())*z1_mod - inertia_Eigen_*c2*z2 - inertia_Eigen_*W.transpose()*c3*z1_int;

     /* Control effort of hybrid Lyapnov attitude controller */
     // horizonal: Kp = 10, Kd = 0.2, Ki = 1, settling time = 0.122 s, overshoot = 0.6%, vertical: Kp = 15, Kd = 2, Ki = 2;  if mode != 1 then no integration will be made (anti-windup)

    // Torque_star << pid_.angular_lyapnov.x.update_attitude(RPY_c(0), roll, Omega(0), period, mode), pid_.angular_lyapnov.y.update_attitude(RPY_c(1), pitch, Omega(1), period, mode), 0.0;
     // Torque = Omega.cross(inertia_Eigen_*Omega) - inertia_Eigen_*c4*Omega + c4*Torque_star;
     Torque(2) = pid_.angular_pos.z.update_attitude(RPY_c(2), yaw, dot_RPY(2), period, mode);

     wrench_.wrench.torque.x = Torque(0);
     wrench_.wrench.torque.y = Torque(1);
     wrench_.wrench.torque.z = Torque(2);
     wrench_.wrench.force.x  = 0.0;
     wrench_.wrench.force.y  = 0.0;
     //wrench_.wrench.force.z  = mass_ * (LinAccel_c(0)*(sin(pitch)*cos(yaw)*cos(roll) + sin(yaw)*sin(roll)) + LinAccel_c(1)*(sin(pitch)*sin(yaw)*cos(roll) - cos(yaw)*sin(roll)) + (LinAccel_c(2) + gravity)*load_factor);
     //wrench_.wrench.force.z  = mass_ * (LinAccel_c(2) * load_factor + gravity);
     wrench_.wrench.force.z  = mass_ * (LinAccel_c(2) * load_factor + gravity);
  }

  void LandingControl(const ros::Duration& period)
  {
      //delta_P << delta_P_temp.x, delta_P_temp.y, delta_P_temp.z;
      if (!target_found_) {delta_P = P_tar_e - P_e; delta_P(2) = P_tar_e(2) - quadrotor_pos_w.getZ();}

      /* PT2 filter */
      dotdot_delta_P_f = -2.0*T_cmdf_2*dot_delta_P_f - T_cmdf_2*T_cmdf_2*(delta_P_f - delta_P);
      dot_delta_P_f += dotdot_delta_P_f*period.toSec();
      delta_P_f  += dot_delta_P_f*period.toSec();
      //dot_delta_P = dot_delta_P_f;
      dot_delta_P += -T_cmdf_2*(dot_delta_P - dot_delta_P_f)*period.toSec();
      //dot_delta_P = (delta_P_f - delta_P_old)/period.toSec();
      //delta_P_old =delta_P_f;

      if (mode == 3 && trajectory_tar_predicted_)
      {timer_.timer_landing_permitssion_.start();}

      /* Trajectory and landing controller */
      LinAccel_c(0) = pid_.linear_pos.x.update_pos( delta_P_f(0), dot_delta_P_f(0), 0.0, period, mode);// + C_mxy*twist.linear.x*sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y + twist.linear.z*twist.linear.z);
      LinAccel_c(1) = pid_.linear_pos.y.update_pos( delta_P_f(1), dot_delta_P_f(1), 0.0, period, mode);// + C_mxy*twist.linear.y*sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y + twist.linear.z*twist.linear.z);

      //LinAccel_c(2) = pid_.linear_pos.z.update_pos(delta_P_f(2) + 1.0, dot_delta_P_f(2), 0.0, period, mode);
      double twist_c_z = pid_.linear_pos.z.update_pos(delta_P_f(2) + 1.0, dot_delta_P_f(2), 0.0, period, mode);
      //double twist_c_z = 2.0;
      if (twist_c_z > 2.0) {twist_c_z = 2.0;}
      if (twist_c_z < -2.0) {twist_c_z = -2.0;}

      if (timer_.timer_landing_permitssion_.states_.started && sqrt(delta_P(0)*delta_P(0) + delta_P(1)*delta_P(1)) <= 0.1)
      // if (timer_.timer_landing_permitssion_.states_.started)
      {
          if(timer_.timer_landing_permitssion_.count(1.5, period))
          {
             land_permitted_= true;
          }
          if (landed_== false && sqrt(relativ_pos_st.getX()*relativ_pos_st.getX()+ relativ_pos_st.getY()*relativ_pos_st.getY() + relativ_pos_st.getZ()*relativ_pos_st.getZ()) < 0.19)
          {motors_running_ = false; landed_ = true; ROS_INFO_NAMED("landing_controller", "Landed, shutting down motors!");}

      }
      else{timer_.timer_landing_permitssion_.reset();}
      if (land_permitted_){twist_c_z = -0.3;}

      LinAccel_c(2) = pid_.linear.z.update(twist_c_z, twist.linear.z, acceleration_->acceleration().z, period);

      if (delta_P_st(0)*delta_P_st(0) + delta_P_st(1)*delta_P_st(1) > 1.0)
      {
            RPY_c(2) = yaw+atan2(relativ_pos_st.getY(), relativ_pos_st.getX());
            //RPY_c(2) = 0.0;
      }
      else
     {
           RPY_c(2) = yaw + yaw_tar;
           //RPY_c(2) = 0.0;
     }
     RPY_c(0) = asin((LinAccel_c(0)*sin(RPY_c(2)) - LinAccel_c(1)*cos(RPY_c(2)))/sqrt( LinAccel_c(0)*LinAccel_c(0) + LinAccel_c(1)*LinAccel_c(1) + (gravity)*(gravity)));
     RPY_c(1) = atan2(LinAccel_c(0)*cos(RPY_c(2)) + LinAccel_c(1)*sin(RPY_c(2)), gravity); // In this one there is no Accel(2) part in the denominator

     // pitch limitation
     double l_star_front = 0.0, l_star_back = 0.0, l = 0.13, P = 0.0, Alpha_front = pi/2.5, Alpha_back = pi - Alpha_front, Beta = 85*0.5*pi/180,   Gamma_front = Alpha_front - Beta + pi/2,   Gamma_back = Alpha_back - Beta + pi/2 -Beta, pitch_max = 0.0, pitch_min = 0.0;
     if (tracking_permitted_)
     {
          P = sqrt(relativ_pos_st.getX()*relativ_pos_st.getX() + relativ_pos_st.getZ()*relativ_pos_st.getZ());
          l_star_front = l*cos(Gamma_front) + sqrt(P*P - sin(Gamma_front)*sin(Gamma_front)*l*l);
          pitch_max  = acos((P*P + l*l - l_star_front*l_star_front)/(2*P*l)) - atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ());
          if (pitch_max > pi/2) {pitch_max = pi/2;}
          if (pitch_max < 0.0) {pitch_max = 0;}
          // std::cout<<"pitch_max = "<<acos((P*P + l*l - l_star_front*l_star_front)/(2*P*l)) <<" - "<< atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ()) <<" = " << pitch_max <<std::endl;

          l_star_back = l*cos(Gamma_back) + sqrt(P*P - sin(Gamma_back)*sin(Gamma_back)*l*l);
          pitch_min = acos((P*P + l*l - l_star_back*l_star_back)/(2*P*l)) + atan2(relativ_pos_st.getX(), -relativ_pos_st.getZ());
          if (pitch_min > pi/2) {pitch_min = pi/2;}
          if (pitch_min < 0.0) {pitch_min = 0.0;}

          if (RPY_c(1) > pitch_max) {RPY_c(1) = pitch_max;}
          //if (RPY_c(1) < -pitch_min) {RPY_c(1) = -pitch_min;}
     }

     /* Control effort of hybrid Lyapnov attitude controller */
     // horizonal: Kp = 10, Kd = 0.2, Ki = 1, settling time = 0.122 s, overshoot = 0.6%, vertical: Kp = 15, Kd = 2, Ki = 2;  if mode != 1 then no integration will be made (anti-windup)

     Torque_star << pid_.angular_lyapnov.x.update_attitude(RPY_c(0), roll, Omega(0), period, mode), pid_.angular_lyapnov.y.update_attitude(RPY_c(1), pitch, Omega(1), period, mode), 0.0;
     Torque = Omega.cross(inertia_Eigen_*Omega) - inertia_Eigen_*c4*Omega + c4*Torque_star;
     Torque(2) = pid_.angular_pos.z.update_attitude(RPY_c(2), yaw, dot_RPY(2), period, mode);

     wrench_.wrench.torque.x = Torque(0);
     wrench_.wrench.torque.y = Torque(1);
     wrench_.wrench.torque.z = Torque(2);
     wrench_.wrench.force.x  = 0.0;
     wrench_.wrench.force.y  = 0.0;
     //wrench_.wrench.force.z  = mass_ * (LinAccel_c(0)*(sin(pitch)*cos(yaw)*cos(roll) + sin(yaw)*sin(roll)) + LinAccel_c(1)*(sin(pitch)*sin(yaw)*cos(roll) - cos(yaw)*sin(roll)) + (LinAccel_c(2) + gravity)*load_factor);
     wrench_.wrench.force.z  = mass_ * (LinAccel_c(2) * load_factor + gravity);
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    // Get twist command input
    if (twist_input_->connected() && twist_input_->enabled()) {  // If twist_input is connected and enabled then no more signals from other sources(like joystick) will be received by ZD
      command_.twist = twist_input_->getCommand();
      command_given_in_stabilized_frame_ = false;                     // stabilized frame is the inertial frame only transformed with yaw   by ZD
    }

    // Get current state and command for Twist-Control
    command = command_.twist;
    twist = twist_->twist();
    twist_body;
    twist_body.linear =  pose_->toBody(twist.linear);
    twist_body.angular = pose_->toBody(twist.angular);

    // Transform to world coordinates if necessary (yaw only)
    if (command_given_in_stabilized_frame_) {
      double yaw = pose_->getYaw();  //  -pi <= yaw <= pi, calculated from orientation.z
      Twist transformed = command;
      transformed.linear.x  = cos(yaw) * command.linear.x  - sin(yaw) * command.linear.y;
      transformed.linear.y  = sin(yaw) * command.linear.x  + cos(yaw) * command.linear.y;
      transformed.angular.x = cos(yaw) * command.angular.x - sin(yaw) * command.angular.y;
      transformed.angular.y = sin(yaw) * command.angular.x + cos(yaw) * command.angular.y;
      command = transformed;
    }

    // Get load factor
    load_factor = 1. / (  pose_->pose().orientation.w * pose_->pose().orientation.w  // [w, x, y, z]T is quaternion, so the system does have a inertial frame, load factor = lift/gravity    by ZD
                                   - pose_->pose().orientation.x * pose_->pose().orientation.x
                                   - pose_->pose().orientation.y * pose_->pose().orientation.y
                                  + pose_->pose().orientation.z * pose_->pose().orientation.z );
    // Note: load_factor could be NaN or Inf...?
    if (load_factor_limit > 0.0 && !(load_factor < load_factor_limit)) load_factor = load_factor_limit;

    // Auto engage/shutdown
    if (auto_engage_) {
      if (!motors_running_ && command.linear.z > 0.1 && load_factor > 0.0) {
        motors_running_ = true;
        ROS_INFO_NAMED("landing_controller", "Engaging motors!");
      } else if (motors_running_ && command.linear.z < -0.1 /* && (twist.linear.z > -0.1 && twist.linear.z < 0.1) */) {
        double shutdown_limit = 0.25 * std::min(command.linear.z, -0.5);
        if (linear_z_control_error_ > 0.0) linear_z_control_error_ = 0.0; // positive control errors should not affect shutdown
        if (pid_.linear.z.getFilteredControlError(linear_z_control_error_, 5.0, period) < shutdown_limit) {                    // The linear_z_control_error_ will keep integrating until its value is smaller than shutdown_limit = -0.5(if command.z = -2)
          motors_running_ = false;
          ROS_INFO_NAMED("landing_controller", "Shutting down motors!");
        } else {
          ROS_DEBUG_STREAM_NAMED("landing_controller", "z control error = " << linear_z_control_error_ << " >= " << shutdown_limit);
        }
      } else {
        linear_z_control_error_ = 0.0;
      }

      // flip over?
      if (motors_running_ && load_factor < 0.0) {
        motors_running_ = false;
        ROS_WARN_NAMED("landing_controller", "Shutting down motors due to flip over!");
      }
    }

    // Get motor status

    // Get current measurements for Lyapnov-Control   by ZD
    Imu imu = imu_->imu();
    pose_->getEulerRPY(roll, pitch, yaw);
    RPY << roll, pitch, yaw;
    Omega << imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z;
   // Omega << twist_body.angular.x,  twist_body.angular.y,  twist_body.angular.z;
   // dot_RPY = (RPY - RPY_old)/(period.toSec());

    /* Reading relative position between target and the UAV.  by ZD  */
    echoListener_st_.tf.lookupTransform("base_stabilized", "heart", ros::Time(), echo_transform_st);   // by ZD
    echoListener_w_.tf.lookupTransform("world", "base_link", ros::Time(), echo_transform_w);   // by ZD
    echoListener_cam_.tf.lookupTransform("front_cam_optical_frame", "heart", ros::Time(), echo_transform_cam);
    echoListener_tar_w_.tf.lookupTransform("world", "heart", ros::Time(), echo_transform_tar_w);
    echo_transform_st.getBasis().getRPY(roll_tar, pitch_tar, yaw_tar);
    //tf::Quaternion q = echo_transform.getRotation();
    relativ_pos_st = echo_transform_st.getOrigin();
    quadrotor_pos_w = echo_transform_w.getOrigin();
    relativ_pos_cam = echo_transform_cam.getOrigin();

    P_tar_real_temp = echo_transform_tar_w.getOrigin();
    P_tar_real << P_tar_real_temp.getX(), P_tar_real_temp.getY(), P_tar_real_temp.getZ();

    /* Analyse the image data */
    noise *= fabs(relativ_pos_st.getZ())*0.1*0.25; // noise is related with height,
    delta_P_st << relativ_pos_st.getX() + noise, relativ_pos_st.getY() + noise, relativ_pos_st.getZ() + noise;
    delta_P_temp.x  = cos(yaw) * delta_P_st(0) - sin(yaw) * delta_P_st(1);
    delta_P_temp.y  = sin(yaw) * delta_P_st(0) + cos(yaw) * delta_P_st(1);
    delta_P_temp.z = delta_P_st(2);
    delta_P << delta_P_temp.x, delta_P_temp.y, delta_P_temp.z; // delta_P will be updated if target is within fov

    /* Check if target is in the fov */
    if (atan2(fabs(relativ_pos_cam.getY()), fabs(relativ_pos_cam.getZ())) < 90*0.5*pi/180 && atan2(fabs(relativ_pos_cam.getX()), fabs(relativ_pos_cam.getZ())) < 120*0.5*pi/180)
    {
        /* If target is not in the fov before */
       if (!target_found_)
       {
           target_found_  = true;
           ROS_INFO_NAMED("landing_controller", "Target captured by camera!");

           /* Reset timers */
           timer_.timer_target_lost_.reset();
           if (mode == 0)
           {
               timer_.timer_tracking_permission_.start();
               timer_.timer_predictor_tar_.start();
           }
       }

       /* Calculation of absolute quadrotor(P_e) */
       P_e(0) += twist.linear.x*period.toSec();
       P_e(1) += twist.linear.y*period.toSec();
       P_e(2) = quadrotor_pos_w.getZ();

       /* If prediction timer started, then begin recording target position data */
       if (mode != 2)
       {
           P_tar_e = P_e + delta_P;  // Target position record

           /* Sample the target absolute position */
           if (mode == 0)
           {
               if (timer_.timer_predictor_tar_.pulse(0.005, period))
               {
                   time_tar.push_back(timer_.timer_predictor_tar_.states_.value_integral); x_tar.push_back(P_tar_e(0)); y_tar.push_back(P_tar_e(1));
               }
           }
       }
    }
    else  // If target is not in the fov
    {
        yaw_tar = yaw_tar_old; // If target is not in the fov, then yaw of target remains unchanged

        if(target_found_) // If the target is in the fov before
        {
           target_found_ = false;
           tracking_permitted_= false;
           ROS_INFO_NAMED("landing_controller", "Target lost! ");

           /* reset timers */
           timer_.timer_tracking_permission_.reset();
           timer_.timer_target_lost_.start();

           /* reset target trajectory sampling, if the target trajectory is still not predicted when target get lost */
           if (!trajectory_tar_predicted_ && mode == 0)
           {
               timer_.timer_predictor_tar_.reset();
               P_e << 0.0, 0.0, -quadrotor_pos_w.getZ(); P_tar_e << 0.0, 0.0, 0.0;
               time_tar.clear(); x_tar.clear(); y_tar.clear();
               time_tar.push_back(0.0); x_tar.push_back(0.0); y_tar.push_back(0.0);
           }
        }

        /* The computation of absolute quadrotor position will continue before target completely lost */
        if (mode != 0)
        {
            P_e(0) += twist.linear.x*period.toSec();
            P_e(1) += twist.linear.y*period.toSec();
            P_e(2) = quadrotor_pos_w.getZ();
        }

        /* If target gets lost over 5 seconds, then switch back to joystick mode*/
        if (mode != 2) // The counting won't continue in path following mode
        {
            /* If target get lost over 5 second outs when not in mode 2, then it will be regarded as completely lost */
            if (timer_.timer_target_lost_.count(5.0, period))
            {
                mode = 0;
                pid_.linear.x.reset();
                pid_.linear.y.reset();
                pid_.linear.z.reset();
                pid_.angular.x.reset();
                pid_.angular.y.reset();
                pid_.angular.z.reset();
                ROS_INFO_NAMED("landing_controller", "Target lost completely, switching to joystick mode!");

                // Reset the trajectory predictor if target completely lost
                trajectory_tar_predicted_ = false;
                timer_.timer_predictor_tar_.reset();
                P_e << 0.0, 0.0, quadrotor_pos_w.getZ(); P_tar_e << 0.0, 0.0, 0.0;
                time_tar.clear(); x_tar.clear(); y_tar.clear();
                time_tar.push_back(0.0); x_tar.push_back(0.0); y_tar.push_back(0.0);
            }
        }
        /* Relative position stops updating while target is not in the fov */
    }

    /* yaw carry over */
    if (yaw_old > 3.0 && yaw < - 3.0)
    { yaw += 2*pi;}
    if (yaw_old < -3.0 && yaw > 3.0)
    { yaw -= 2*pi;}
    yaw_old = yaw;

//    /* Tracking permission after keeping target in the fov for 3 seconds */
//    if (timer_.timer_tracking_permission_.count(3.0, period))
//    {
//        tracking_permitted_ = true;
//        ROS_INFO_NAMED("landing_controller", "Tracking permitted...");
//        // mode = 3;
//        mode = 1;
//    }

//   //  Attitude test
//     if (!timer_.timer_test_.states_.started)
//     {timer_.timer_test_.start();}
//     if (timer_.timer_test_.pulse(5.0, period) && timer_.timer_test_.states_.value_integral < 6.0)
//      {
//           {ROS_INFO_NAMED("landing_controller", "test mode begin"); mode = 1;
//           RPY_c << 0.3, 0.0, 0.0;}
//      }
//     else if (timer_.timer_test_.pulse(5.0, period) && timer_.timer_test_.states_.value_integral < 11.0)
//     {
//           RPY_c << 0.3, 0.3, 0.0;
//     }
//     else if (timer_.timer_test_.pulse(5.0, period) && timer_.timer_test_.states_.value_integral < 16.0)
//     {
//          RPY_c << 0.3, 0.3, 1.0;
//     }
//       if (time.toSec() > 5.0 && mode == 0)
//        {
//             ROS_INFO_NAMED("landing_controller", "test mode begin"); mode = 1;
//        }
//         else if (time.toSec() > 10.0 && RPY_c(1) == 0.0)
//         {
//               RPY_c << 0.6, 0.6, 0.0;
//         }
//         else if (time.toSec() > 15.0)
//         {
//              RPY_c << 0.6, 0.6, 1.0;
//         }

//   // Trajectory test 1
//    if (mode_test == 0)
//      {P_test_e << P_test_e(0) + 1.0*period.toSec(), P_test_e(1), P_test_e(2) + 1.0*period.toSec(); if (P_test_e(0) >= 5.0){mode_test=1;}}
//    if (mode_test == 1)
//      {P_test_e << P_test_e(0), P_test_e(1) + 1.0*period.toSec(), P_test_e(2) -1.0*period.toSec(); if (P_test_e(1) >= 5.0){mode_test=2;}}
//    if (mode_test == 2)
//      {P_test_e << P_test_e(0) - 1.0*period.toSec(), P_test_e(1), P_test_e(2) + 1.0*period.toSec(); if (P_test_e(0) <= 0.0){mode_test=3;}}
//    if (mode_test == 3)
//      {P_test_e << P_test_e(0), P_test_e(1) - 1.0*period.toSec(), P_test_e(2) - 1.0*period.toSec(); if (P_test_e(1) <= 0.0){mode_test=0;}}

//   // Trajectory test 2
//      P_test_e << 5.0*sin(0.32*time.toSec()), 3.0*cos(0.32*time.toSec()), 5.0 + sin(0.32*time.toSec())  ;

//     std::cout<<"value "<<timer_.timer_test_.states_.value<<std::endl;
//     std::cout<<"mode "<<mode<<std::endl;

    //If over 300 samples of target absolute position is obtained, then start prediction
    if (time_tar.size() >= 5/period.toSec() && !trajectory_tar_predicted_ && mode == 0 && motors_running_)
    {
        trajectory_predictor_tar_x_.polyfit<double>(time_tar, x_tar, 3, true);
        trajectory_predictor_tar_x_.getFactor(Coefs_tar_x);
        std::cout<<"Trajectory quality x is "<<trajectory_predictor_tar_x_.getR_square()<<std::endl;
        trajectory_predictor_tar_y_.polyfit<double>(time_tar, y_tar, 3, true);
        trajectory_predictor_tar_y_.getFactor(Coefs_tar_y);
        std::cout<<"Trajectory quality y is "<<trajectory_predictor_tar_y_.getR_square()<<std::endl;

        Coef_z = (1.15 - P_e(2))/9.0; // z = Ceof_z*t², at t = 3.0, the trajectory reach the bottom
        time_z = 0.0; P_start_z = P_e(2);

        ROS_INFO_NAMED("landing_controller", "Target trajectory predicted, start blind tracking ...");
        mode = 2;
        time_following_start = timer_.timer_predictor_tar_.states_.value_integral;

        trajectory_tar_predicted_ = true;
    }

    // Update output          here the twist should be the velocity, all calculations are done in world frame
    if (motors_running_) {

        // so lang as motor is running, the landing process is not over. by ZD
        landed_=false;

       /* Switching mode */
       (this->*SwitchMethod[mode])(period);

       if (limits_.force.z > 0.0 && wrench_.wrench.force.z > limits_.force.z) wrench_.wrench.force.z = limits_.force.z;
       if (wrench_.wrench.force.z <= std::numeric_limits<double>::min()) wrench_.wrench.force.z = std::numeric_limits<double>::min();
       if (limits_.torque.x > 0.0) {
       if (wrench_.wrench.torque.x >  limits_.torque.x) wrench_.wrench.torque.x =  limits_.torque.x;
       if (wrench_.wrench.torque.x < -limits_.torque.x) wrench_.wrench.torque.x = -limits_.torque.x;
       }
       if (limits_.torque.y > 0.0) {
         if (wrench_.wrench.torque.y >  limits_.torque.y) wrench_.wrench.torque.y =  limits_.torque.y;
         if (wrench_.wrench.torque.y < -limits_.torque.y) wrench_.wrench.torque.y = -limits_.torque.y;
       }
       if (limits_.torque.z > 0.0) {
         if (wrench_.wrench.torque.z >  limits_.torque.z) wrench_.wrench.torque.z =  limits_.torque.z;
         if (wrench_.wrench.torque.z < -limits_.torque.z) wrench_.wrench.torque.z = -limits_.torque.z;
       }

       // motor propulsion calculations
       double nominal_thrust_per_motor = wrench_.wrench.force.z / 4.0;
       motor_command_.force[0] =  nominal_thrust_per_motor - wrench_.wrench.torque.y / 2.0 / lever;
       motor_command_.force[1] =  nominal_thrust_per_motor + wrench_.wrench.torque.y / 2.0 / lever;
       motor_command_.force[2] =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 2.0 / lever;
       motor_command_.force[3] =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 2.0 / lever;
       double nominal_torque_per_motor = wrench_.wrench.torque.z / 4.0;
       motor_command_.voltage[0] = motor_command_.force[0] / force_per_voltage + nominal_torque_per_motor / torque_per_voltage;
       if (motor_command_.voltage[0] < 1.5) { motor_command_.voltage[0] = 1.5; }
       if (motor_command_.voltage[0] > 15.0) { motor_command_.voltage[0] = 15.0; }
       motor_command_.voltage[1] = motor_command_.force[1] / force_per_voltage + nominal_torque_per_motor / torque_per_voltage;
       if (motor_command_.voltage[1] < 1.5) { motor_command_.voltage[1] = 1.5; }
       if (motor_command_.voltage[1] > 15.0) { motor_command_.voltage[1] = 15.0; }
       motor_command_.voltage[2] = motor_command_.force[2] / force_per_voltage - nominal_torque_per_motor / torque_per_voltage;
       if (motor_command_.voltage[2] < 1.5) { motor_command_.voltage[2] = 1.5; }
       if (motor_command_.voltage[2] > 15.0) { motor_command_.voltage[2] = 15.0; }
       motor_command_.voltage[3] = motor_command_.force[3] / force_per_voltage - nominal_torque_per_motor / torque_per_voltage;
       if (motor_command_.voltage[3] < 1.5) { motor_command_.voltage[3] = 1.5; }
       if (motor_command_.voltage[3] > 15.0) { motor_command_.voltage[3] = 15.0; }


       ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.force:       [" << wrench_.wrench.force.x << " " << wrench_.wrench.force.y << " " << wrench_.wrench.force.z << "]");
       ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.torque:      [" << wrench_.wrench.torque.x << " " << wrench_.wrench.torque.y << " " << wrench_.wrench.torque.z << "]");

    } else {
      reset();
      //fout.close();
      mode = 0;
      trajectory_tar_predicted_ = false;
      land_permitted_= false;
    }

    delta_P_old = delta_P;
    yaw_tar_old = yaw_tar;
    RPY_old = RPY; // by ZD

    // Data Recording
    if (!fout.is_open())
    {
        fout.open("/home/ding/Records.txt");
        fout<<"1time, 2LinAccel_c_x, 3LinAccel_c_y, 4LinAccel_c_z, 5Accel_x, 6Accel_y, 7Accel_z, 8delta_P, 9delta_P_st, 10delta_P_f, 11dot_delta_P_f, 12dot_delta_P, 13-twist.linear.x, 14-twist.linear.y, 15-twist.linear.z, 16roll_c, 17pitch_c, 18yaw_c, 19roll, 20pitch, 21yaw, 22torque_x, 23torque_y, 24torque_z, 25dot_roll, 26dot_pitch, 27dot_yaw, 28p, 29q, 30r, 31motorvoltage1, 32motorvoltage2, 33motorvoltage3, 34motorvoltage4, 35P_tar_realx, 36P_tar_realy, 37P_tar_realz, 38P_tar_ex, 39P_tar_ey, 40P_tar_ez, 41Pnx, 42Pny, 43Pnz, 44P_testx, 45P_testy, 46P_testz, 47x_pre, 48y_pre, 49z_pre, 50detV_x, 51detV_y, 52detV_z"<<std::endl;
    }
    fout<<time.toSec()<<", "<<LinAccel_c(0)<<", "<<LinAccel_c(1)<<", "<<LinAccel_c(2)<<", "<<acceleration_->acceleration().x<<", "<<acceleration_->acceleration().y<<", "<<acceleration_->acceleration().z<<", "<<delta_P(2)<<", "<<delta_P_st(2)<<", "<< -delta_P_f(2)<<", "<<dot_delta_P_f(2)<<", "<<dot_delta_P(2)<<", "<<twist.linear.x<<", "<<twist.linear.y<<", "<<twist.linear.z<<", "<<RPY_c(0)<<", "<<RPY_c(1)<<", "<<RPY_c(2)<<", "<<roll<<", "<<pitch<<", "<<yaw<<", "<<wrench_.wrench.torque.x<<", "<<wrench_.wrench.torque.y<<", "<<wrench_.wrench.torque.z<<", "<<dot_RPY(0)<<", "<<dot_RPY(1)<<", "<<dot_RPY(2)<<", "<<Omega(0)<<", "<<Omega(1)<<", "<<Omega(2)<<", "<<motor_command_.voltage[0]<<", "<<motor_command_.voltage[1]<<", "<<motor_command_.voltage[2]<<", "<<motor_command_.voltage[3]<<", "<<P_tar_real(0)<<", "<<P_tar_real(1)<<", "<<P_tar_real(2)<<", "<<P_tar_e(0)<<", "<<P_tar_e(1)<<", "<<P_tar_e(2)<<", "<<quadrotor_pos_w.getX()<<", "<<quadrotor_pos_w.getY()<<", "<<quadrotor_pos_w.getZ()<<", "<<P_test_e(0)<<", "<<P_test_e(1)<<", "<<P_test_e(2)<<", "<<P_tar_e_temp(0)<<", "<<P_tar_e_temp(1)<<", "<<P_tar_e_temp(2)<<", "<<dot_delta_P_f(0)<<", "<<dot_delta_P_f(1)<<", "<<dot_delta_P_f(2)<<std::endl;


    // set wrench output
    wrench_.header.stamp = time;
    wrench_.header.frame_id = base_link_frame_;
    wrench_output_->setCommand(wrench_.wrench);
  }

private:
  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  ImuHandlePtr imu_;   // by ZD
  //MotorStatusHandlePtr motor_command_status_; // by ZD
  AccelerationHandlePtr acceleration_;
  TwistCommandHandlePtr twist_input_;
  WrenchCommandHandlePtr wrench_output_;

  ros::NodeHandle node_handle_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber cmd_vel_subscriber_;

  ros::Subscriber imu_subscriber_;
  ros::Subscriber noise_subscriber_;
  //ros::Subscriber motor_command_status_subscriber_;
  ros::ServiceServer engage_service_server_;
  ros::ServiceServer shutdown_service_server_;

  geometry_msgs::TwistStamped command_;
  geometry_msgs::WrenchStamped wrench_;
  bool command_given_in_stabilized_frame_;
  std::string base_link_frame_;

  struct {
    struct {
      PID x;
      PID y;
      PID z;
    } linear, angular, angular_lyapnov, angular_pos, linear_pos; // by ZD
  } pid_;

  bool timer_test_started;
  struct { // by ZD
      Timer timer_test_;
      Timer timer_tracking_permission_;
      Timer timer_target_lost_;
      Timer timer_predictor_tar_;
      Timer timer_landing_permitssion_;
  } timer_;

  geometry_msgs::Wrench limits_;
  bool auto_engage_;
  double load_factor_limit, load_factor, gravity;
  double mass_;
  double inertia_[3];
  Eigen::Matrix3d inertia_Eigen_;

  bool motors_running_;
  double linear_z_control_error_;
  boost::mutex command_mutex_;

  Twist command, twist, twist_body;
  Vector3 acceleration_command, acceleration_command_body;

  std::string source_frameid, target_frameid;
  tf::StampedTransform echo_transform_st, echo_transform_w, echo_transform_cam, echo_transform_tar_w;
  echoListener echoListener_st_, echoListener_w_, echoListener_cam_, echoListener_tar_w_;
  tf::Vector3 relativ_pos_cam, relativ_pos_st, quadrotor_pos_w;
  Vector3 delta_P_temp;

  double pitch, roll, yaw, yaw_old, pi; // by ZD
  double pitch_tar, roll_tar, yaw_tar, yaw_tar_old; // orientation of the target in the quadrotor's body frame by ZD
  Eigen::Vector3d T_drag, RPY_c, RPY, dot_RPY, RPY_old, z1, z1_mod, z2, z1_old, z1_int, Omega, dot_Omega_c, Omega_c, Omega_d, Ypsilon, Torque, Torque_f, Torque_star; // parameters for attitude control
  Eigen::Vector3d Trajectory, delta_P, delta_P_st, delta_P_old, dot_delta_P, delta_P_f, dot_delta_P_f,  dotdot_delta_P_f, LinAccel_c, P_tar_e_temp;
  Eigen::Matrix3d W, W_inv, c1, c2, c3, c4, T_cmdf, T_cmdf_1, T_cmdf_2, C_drag_T, R_mat;

  // trajectory of target prediction and planning issue
  bool trajectory_tar_predicted_, once_and_for_all, trajectory_generated_;
  Fit trajectory_predictor_tar_x_, trajectory_predictor_tar_y_;
  std::vector<double> time_tar, x_tar, y_tar, Coefs_tar_x, Coefs_tar_y;
  double yaw_tar_e, time_following_start, Coef_z, time_z, P_start_z;
  Eigen::Vector3d P_tar_e, P_e, P_tar_e_once_, twist_cmd_once_, P_following_start_, P_tar_real, P_test_e;
  tf::Vector3 P_tar_real_temp;

  // motor propulsion model by ZD
  hector_uav_msgs::MotorCommand motor_command_;
  double force_per_voltage;     // coefficient for linearized volts to force conversion for a single motor [N / V]
  double torque_per_voltage;    // coefficient for linearized volts to force conversion for a single motor [Nm / V]
  double lever;                 // the lever arm from origin to the motor axes (symmetry assumption) [m]

  // variable for tests
  int mode_test;

  double C_wxy;
  double C_wz;
  double C_mxy;
  double C_mz;
  std::ofstream fout;
  double noise;

  // pitch_c limitation and mode switch
  int mode;
  bool tracking_demand_;
  bool target_found_;
  bool target_lost_;
  bool tracking_permitted_;
  bool land_permitted_;
  bool landed_;

  void (LandingController::*SwitchMethod[4])(const ros::Duration& period);

  // test
  double overshoot_rate, error_old, settling_time, rise_time;
  bool overshoot, overshoot_done, rise_done, control_done, control_about_done;
};

} // namespace hector_quadrotor_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller::LandingController, controller_interface::ControllerBase)
