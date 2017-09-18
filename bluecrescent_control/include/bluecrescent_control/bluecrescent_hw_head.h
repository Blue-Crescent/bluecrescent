#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.h>

	
enum ROTATION {ROLL=1,PITCH=2,YAW=0};
enum ARM {SHOULDER=0,ELBOW,WRIST};
enum SIDE {LEFT=0,RIGHT};

namespace bluecrescent_control{
  double head_cmd_[3];
  double head_pos_[3];
  
class BlueCrescent_head : public hardware_interface::RobotHW
{
public:

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void motor_release();
  void motor_lock(uint8_t num);
  void cwstep(uint8_t num);
  void ccwstep(uint8_t num);

  void read(ros::Time, ros::Duration);
  void write(ros::Time, ros::Duration);

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double head_vel_[3];
  double head_eff_[3];

public:
  BlueCrescent_head()
  {
    // connect and register the joint state interface
    //signal(SIGINT, bluecrescent_control::mySigIntHandler);
  
  //debug  fd_mux = wiringPiI2CSetup(0x70);
  //debug  wiringPiI2CWriteReg8(fd_mux, 0x0 ,0x04);
  //debug
  //debug  fd_M0[0] = wiringPiI2CSetup(drv8830_addr_M0[0]);
  //debug  fd_M0[1] = wiringPiI2CSetup(drv8830_addr_M0[1]);
  //debug  fd_M1[0] = wiringPiI2CSetup(drv8830_addr_M1[0]);
  //debug  fd_M1[1] = wiringPiI2CSetup(drv8830_addr_M1[1]);
    printf("This is bluecrescnet_hw_head\n");
  
  
    hardware_interface::JointStateHandle state_handle_head_roll("head_roll", &bluecrescent_control::head_pos_[ROLL], &head_vel_[ROLL], &head_eff_[ROLL]);
    jnt_state_interface.registerHandle(state_handle_head_roll);
  
    hardware_interface::JointHandle pos_handle_head_roll(jnt_state_interface.getHandle("head_roll"), &bluecrescent_control::head_cmd_[ROLL]);
    jnt_pos_interface.registerHandle(pos_handle_head_roll);
    
    hardware_interface::JointStateHandle state_handle_head_yaw("head_yaw", &bluecrescent_control::head_pos_[YAW], &head_vel_[YAW], &head_eff_[YAW]);
    jnt_state_interface.registerHandle(state_handle_head_yaw);
  
    hardware_interface::JointHandle pos_handle_head_yaw(jnt_state_interface.getHandle("head_yaw"), &bluecrescent_control::head_cmd_[YAW]);
    jnt_pos_interface.registerHandle(pos_handle_head_yaw);
  
    
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
  
  }

};
}
