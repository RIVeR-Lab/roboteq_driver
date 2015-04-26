#include <roboteq_driver/roboteq_controller_hw.h>

namespace roboteq_driver{
  using namespace device_driver;

  RoboteqControllerHW::RoboteqControllerHW(ros::NodeHandle n, std::string port, std::string actuator_1_name, double maxRPM1, int ppr1, std::string actuator_2_name, double maxRPM2, int ppr2, hardware_interface::ActuatorStateInterface state_interface, hardware_interface::VelocityActuatorInterface& vel_interface): controller(maxRPM1, maxRPM2, ppr1, ppr2), port(port) {
    hardware_interface::ActuatorStateHandle state_handle_1(actuator_1_name, &pos[0], &vel[0], &eff[0]);
    state_interface.registerHandle(state_handle_1);

    hardware_interface::ActuatorStateHandle state_handle_2(actuator_2_name, &pos[1], &vel[1], &eff[1]);
    state_interface.registerHandle(state_handle_2);

    hardware_interface::ActuatorHandle vel_handle_1(state_interface.getHandle(actuator_1_name), &cmd[0]);
    vel_interface.registerHandle(vel_handle_1);

    hardware_interface::ActuatorHandle vel_handle_2(state_interface.getHandle(actuator_2_name), &cmd[1]);
    vel_interface.registerHandle(vel_handle_2);

    reconnect();
    reconnect_timer = n.createTimer(ros::Duration(1), &RoboteqControllerHW::connection_check, this);
  }
  RoboteqControllerHW::~RoboteqControllerHW(){
    close();
  }


  void RoboteqControllerHW::close(){
    unique_lock<recursive_timed_mutex> lock(controller_mutex);
    try{
      controller.close();
    } catch(Exception& e){
      ROS_ERROR_STREAM("Roboteq driver got error closing: "<<e.what());
    }
  }
  void RoboteqControllerHW::connection_check(const ros::TimerEvent& e){
    unique_lock<recursive_timed_mutex> lock(controller_mutex);
    if(!controller.is_connected()){
      ROS_WARN("Reconnecting to roboteq...");
      reconnect();
    }
  }
  void RoboteqControllerHW::reconnect(){
    unique_lock<recursive_timed_mutex> lock(controller_mutex);
    try{
      controller.close();
      controller.open(port);
      controller.setMotorMode(1, RoboteqMotorController::MOTOR_MODE_RPM);
      controller.setMotorMode(2, RoboteqMotorController::MOTOR_MODE_RPM);
      controller.setCurrentTrigger(1, 750, 1000);
      controller.setCurrentTrigger(2, 750, 1000);
      controller.saveToEEPROM();
    } catch(Exception& e){
      ROS_ERROR_STREAM("Roboteq driver got error reconnecting: "<<e.what());
    }
  }
  void RoboteqControllerHW::read(){
    unique_lock<recursive_timed_mutex> lock(controller_mutex, try_to_lock);
    if(lock && controller.is_connected()){
      try{
        double pos_rot[2];
        double vel_rpm[2];
        controller.getPosition(1, pos_rot[0]);
        controller.getPosition(2, pos_rot[1]);
        controller.getVelocity(1, vel_rpm[0]);
        controller.getVelocity(2, vel_rpm[1]);
        pos[0] = pos_rot[0]*2*M_PI;
        pos[1] = pos_rot[1]*2*M_PI;
        vel[0] = vel_rpm[0]/60*2*M_PI;
        vel[1] = vel_rpm[1]/60*2*M_PI;
      } catch(Exception& e){
        ROS_ERROR_STREAM_THROTTLE(1, "Roboteq driver got error reading state: "<<e.what());
        close();
      }
    }
  }
#define RAD_PER_SEC_TO_RPM (60/(2*M_PI))
  void RoboteqControllerHW::write(){
    unique_lock<recursive_timed_mutex> lock(controller_mutex, try_to_lock);
    if(lock && controller.is_connected()){
      try{
	controller.setRPM(1, cmd[0]*RAD_PER_SEC_TO_RPM);
	controller.setRPM(2, cmd[1]*RAD_PER_SEC_TO_RPM);
      } catch(Exception& e){
        ROS_ERROR_STREAM_THROTTLE(1, "Roboteq driver got error writing command: "<<e.what());
        close();
      }
    }
  }

}
