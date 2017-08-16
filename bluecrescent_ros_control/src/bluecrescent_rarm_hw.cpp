#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <bluecrescent_ros_control/bluecrescent_rarm_hw.h>
#include <iostream> // for debug
#include <math.h>

namespace bluecrescent_ros_control 
{
bluecrescent_rarm_hw::bluecrescent_rarm_hw()
{

  // connect and register the joint state interface
  // ARM right
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_roll("arm_shoulder_right_roll", &arm_pos_[SHOULDER][RIGHT][ROLL], &arm_vel_[SHOULDER][RIGHT][ROLL], &arm_eff_[SHOULDER][RIGHT][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_roll);

  hardware_interface::JointHandle arm_pos_handle_arm_shoulder_right_roll(jnt_state_interface.getHandle("arm_shoulder_right_roll"), &arm_cmd_[SHOULDER][RIGHT][ROLL]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_shoulder_right_roll);
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_pitch("arm_shoulder_right_pitch", &arm_pos_[SHOULDER][RIGHT][PITCH], &arm_vel_[SHOULDER][RIGHT][PITCH], &arm_eff_[SHOULDER][RIGHT][PITCH]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_pitch);

  hardware_interface::JointHandle arm_pos_handle_arm_shoulder_right_pitch(jnt_state_interface.getHandle("arm_shoulder_right_pitch"), &arm_cmd_[SHOULDER][RIGHT][PITCH]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_shoulder_right_pitch);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_yaw("arm_elbow_right_yaw", &arm_pos_[ELBOW][RIGHT][YAW], &arm_vel_[ELBOW][RIGHT][YAW], &arm_eff_[ELBOW][RIGHT][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_yaw);

  hardware_interface::JointHandle arm_pos_handle_arm_elbow_right_yaw(jnt_state_interface.getHandle("arm_elbow_right_yaw"), &arm_cmd_[ELBOW][RIGHT][YAW]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_elbow_right_yaw);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_roll("arm_elbow_right_roll", &arm_pos_[ELBOW][RIGHT][ROLL], &arm_vel_[ELBOW][RIGHT][ROLL], &arm_eff_[ELBOW][RIGHT][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_roll);

  hardware_interface::JointHandle arm_pos_handle_arm_elbow_right_roll(jnt_state_interface.getHandle("arm_elbow_right_roll"), &arm_cmd_[ELBOW][RIGHT][ROLL]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_elbow_right_roll);

  hardware_interface::JointStateHandle state_handle_arm_wrist_right_yaw("arm_wrist_right_yaw", &arm_pos_[WRIST][RIGHT][YAW], &arm_vel_[WRIST][RIGHT][YAW], &arm_eff_[WRIST][RIGHT][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_wrist_right_yaw);

  hardware_interface::JointHandle arm_pos_handle_arm_wrist_right_yaw(jnt_state_interface.getHandle("arm_wrist_right_yaw"), &arm_cmd_[WRIST][RIGHT][YAW]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_wrist_right_yaw);
  
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);

}
bool  bluecrescent_rarm_hw::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	return true;
}

void bluecrescent_rarm_hw::read(const ros::Time& time,const ros::Duration& period)
{
}

void bluecrescent_rarm_hw::write(const ros::Time& time,const ros::Duration& period)
{
 
  // Real Robot functionality coding here...
  // below code is simulating real robot delay.	
  //pos_[0] = pos_[0] + 0.01* (cmd_[0] - pos_[0]);
  //head_pos_[ROLL] = head_cmd_[ROLL];// + 0.01*(head_cmd_[ROLL] - head_pos_[ROLL]);
  //head_pos_[YAW] = head_cmd_[YAW];// + 0.01*(head_cmd_[YAW] - head_pos_[YAW]);
  ////ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  //// Dump cmd_ from MoveIt!, current simulated real robot pos_.
  //printf("%lf,%lf\n",head_pos_[ROLL],head_cmd_[ROLL]);
}
}
