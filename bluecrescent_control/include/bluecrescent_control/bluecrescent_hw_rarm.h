#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>

enum ROTATION {ROLL=1,PITCH=2,YAW=0};
enum ARM {SHOULDER=0,ELBOW,WRIST};
enum SIDE {LEFT=0,RIGHT};

namespace combined_robot_hw_tests{
  double rarm_cmd_[3][3];
  double rarm_pos_[3][3];
  double rarm_vel_[3][3];
  double rarm_eff_[3][3];
class BlueCrescent_rarm : public hardware_interface::RobotHW
{
public:
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read(ros::Time, ros::Duration);

  void write(ros::Time, ros::Duration);

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
//
//  double head_cmd_[3];
//  double head_pos_[3];
//  double head_vel_[3];
//  double head_eff_[3];
//

public:
  BlueCrescent_rarm()
  {
    // connect and register the joint state interface
  
  //debug  fd_mux = wiringPiI2CSetup(0x70);
  //debug  wiringPiI2CWriteReg8(fd_mux, 0x0 ,0x04);
  //debug
  //debug  fd_M0[0] = wiringPiI2CSetup(drv8830_addr_M0[0]);
  //debug  fd_M0[1] = wiringPiI2CSetup(drv8830_addr_M0[1]);
  //debug  fd_M1[0] = wiringPiI2CSetup(drv8830_addr_M1[0]);
  //debug  fd_M1[1] = wiringPiI2CSetup(drv8830_addr_M1[1]);
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_roll("arm_shoulder_right_roll", &combined_robot_hw_tests::rarm_pos_[SHOULDER][ROLL], &combined_robot_hw_tests::rarm_vel_[SHOULDER][ROLL], &combined_robot_hw_tests::rarm_eff_[SHOULDER][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_roll(jnt_state_interface.getHandle("arm_shoulder_right_roll"), &combined_robot_hw_tests::rarm_cmd_[SHOULDER][ROLL]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_roll);
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_pitch("arm_shoulder_right_pitch", &combined_robot_hw_tests::rarm_pos_[SHOULDER][PITCH], &combined_robot_hw_tests::rarm_vel_[SHOULDER][PITCH], &combined_robot_hw_tests::rarm_eff_[SHOULDER][PITCH]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_pitch);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_pitch(jnt_state_interface.getHandle("arm_shoulder_right_pitch"), &combined_robot_hw_tests::rarm_cmd_[SHOULDER][PITCH]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_pitch);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_yaw("arm_elbow_right_yaw", &combined_robot_hw_tests::rarm_pos_[ELBOW][YAW], &combined_robot_hw_tests::rarm_vel_[ELBOW][YAW], &combined_robot_hw_tests::rarm_eff_[ELBOW][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_yaw(jnt_state_interface.getHandle("arm_elbow_right_yaw"), &combined_robot_hw_tests::rarm_cmd_[ELBOW][YAW]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_yaw);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_roll("arm_elbow_right_roll", &combined_robot_hw_tests::rarm_pos_[ELBOW][ROLL], &combined_robot_hw_tests::rarm_vel_[ELBOW][ROLL], &combined_robot_hw_tests::rarm_eff_[ELBOW][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_roll(jnt_state_interface.getHandle("arm_elbow_right_roll"), &combined_robot_hw_tests::rarm_cmd_[ELBOW][ROLL]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_roll);

  hardware_interface::JointStateHandle state_handle_arm_wrist_right_yaw("arm_wrist_right_yaw", &combined_robot_hw_tests::rarm_pos_[WRIST][YAW], &combined_robot_hw_tests::rarm_vel_[WRIST][YAW], &combined_robot_hw_tests::rarm_eff_[WRIST][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_wrist_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_wrist_right_yaw(jnt_state_interface.getHandle("arm_wrist_right_yaw"), &combined_robot_hw_tests::rarm_cmd_[WRIST][YAW]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_wrist_right_yaw);
  
  
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);
  
  
  }
};

}
