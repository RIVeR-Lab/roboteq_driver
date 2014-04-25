#ifndef ROBOTEQ_CONTROLLER_HW_
#define ROBOTEQ_CONTROLLER_HW_

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <roboteq_driver/roboteq_motor_controller.h>
#include <boost/thread/recursive_mutex.hpp>
#include <safety_interface/safety_interface.h>

namespace roboteq_driver{
using namespace boost;

class RoboteqControllerHW
{
 public:
  RoboteqControllerHW(ros::NodeHandle n, std::string port, std::string actuator_1_name, double maxRPM1, int ppr1, std::string actuator_2_name, double maxRPM2, int ppr2, hardware_interface::ActuatorStateInterface state_interface, hardware_interface::VelocityActuatorInterface& vel_interface, safety_interface::SafetyInterface& safety_interface);

  ~RoboteqControllerHW();
  void reconnect();
  void close();
  void read();
  void write();

 private:
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  RoboteqMotorController controller;
  std::string port;
  void connection_check(const ros::TimerEvent& e);
  ros::Timer reconnect_timer;
  recursive_timed_mutex controller_mutex;
  safety_interface::SafetyInterface& safety_interface;
};

}

#endif
