#include <roboteq_driver/roboteq_controller_hw.h>

namespace roboteq_driver{

  RoboteqControllerHW::RoboteqControllerHW(std::string port, std::string actuator_1_name, double maxRPM1, int ppr1, std::string actuator_2_name, double maxRPM2, int ppr2, hardware_interface::ActuatorStateInterface state_interface, hardware_interface::VelocityActuatorInterface& vel_interface): controller(maxRPM1, maxRPM2, ppr1, ppr2){
    hardware_interface::ActuatorStateHandle state_handle_1(actuator_1_name, &pos[0], &vel[0], &eff[0]);
    state_interface.registerHandle(state_handle_1);

    hardware_interface::ActuatorStateHandle state_handle_2(actuator_2_name, &pos[1], &vel[1], &eff[1]);
    state_interface.registerHandle(state_handle_2);

    hardware_interface::ActuatorHandle vel_handle_1(state_interface.getHandle(actuator_1_name), &cmd[0]);
    vel_interface.registerHandle(vel_handle_1);

    hardware_interface::ActuatorHandle vel_handle_2(state_interface.getHandle(actuator_2_name), &cmd[1]);
    vel_interface.registerHandle(vel_handle_2);

    controller.open(port);
    controller.setMotorMode(0, RoboteqMotorController::MOTOR_MODE_RPM);
    controller.setMotorMode(1, RoboteqMotorController::MOTOR_MODE_RPM);
  }
  RoboteqControllerHW::~RoboteqControllerHW(){
    controller.close();
  }

  void RoboteqControllerHW::read(){
    controller.getPosition(0, pos[0]);
    controller.getPosition(1, pos[1]);
    controller.getVelocity(0, vel[0]);
    controller.getVelocity(1, vel[1]);
  }
  
  void RoboteqControllerHW::write(){
    //TODO convert cmd to RPM
    controller.setRPM(0, cmd[0]);
    controller.setRPM(1, cmd[1]);
  }

}
