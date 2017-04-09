#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <bluecrescent_control/bluecrescent_hw.h>
#include <iostream> // for debug
#include <math.h>


BlueCrescent::BlueCrescent()
{
  // connect and register the joint state interface

  // Head

  hardware_interface::JointStateHandle state_handle_head_roll("head_roll", &head_pos_[ROLL], &head_vel_[ROLL], &head_eff_[ROLL]);
  jnt_state_interface.registerHandle(state_handle_head_roll);

  hardware_interface::JointHandle pos_handle_head_roll(jnt_state_interface.getHandle("head_roll"), &head_cmd_[ROLL]);
  jnt_pos_interface.registerHandle(pos_handle_head_roll);
  
  hardware_interface::JointStateHandle state_handle_head_yaw("head_yaw", &head_pos_[YAW], &head_vel_[YAW], &head_eff_[YAW]);
  jnt_state_interface.registerHandle(state_handle_head_yaw);

  hardware_interface::JointHandle pos_handle_head_yaw(jnt_state_interface.getHandle("head_yaw"), &head_cmd_[YAW]);
  jnt_pos_interface.registerHandle(pos_handle_head_yaw);
 
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
  
  // ARM left

  hardware_interface::JointStateHandle state_handle_arm_shoulder_left_roll("arm_shoulder_left_roll", &arm_pos_[SHOULDER][LEFT][ROLL], &arm_vel_[SHOULDER][LEFT][ROLL], &arm_eff_[SHOULDER][LEFT][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_left_roll);

  hardware_interface::JointHandle arm_pos_handle_arm_shoulder_left_roll(jnt_state_interface.getHandle("arm_shoulder_left_roll"), &arm_cmd_[SHOULDER][LEFT][ROLL]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_shoulder_left_roll);

  hardware_interface::JointStateHandle state_handle_arm_shoulder_left_pitch("arm_shoulder_left_pitch", &arm_pos_[SHOULDER][LEFT][PITCH], &arm_vel_[SHOULDER][LEFT][PITCH], &arm_eff_[SHOULDER][LEFT][PITCH]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_left_pitch);

  hardware_interface::JointHandle arm_pos_handle_arm_shoulder_left_pitch(jnt_state_interface.getHandle("arm_shoulder_left_pitch"), &arm_cmd_[SHOULDER][LEFT][PITCH]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_shoulder_left_pitch);

  hardware_interface::JointStateHandle state_handle_arm_elbow_left_yaw("arm_elbow_left_yaw", &arm_pos_[ELBOW][LEFT][YAW], &arm_vel_[ELBOW][LEFT][YAW], &arm_eff_[ELBOW][LEFT][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_left_yaw);

  hardware_interface::JointHandle arm_pos_handle_arm_elbow_left_yaw(jnt_state_interface.getHandle("arm_elbow_left_yaw"), &arm_cmd_[ELBOW][LEFT][YAW]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_elbow_left_yaw);

  hardware_interface::JointStateHandle state_handle_arm_elbow_left_roll("arm_elbow_left_roll", &arm_pos_[ELBOW][LEFT][ROLL], &arm_vel_[ELBOW][LEFT][ROLL], &arm_eff_[ELBOW][LEFT][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_left_roll);

  hardware_interface::JointHandle arm_pos_handle_arm_elbow_left_roll(jnt_state_interface.getHandle("arm_elbow_left_roll"), &arm_cmd_[ELBOW][LEFT][ROLL]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_elbow_left_roll);

  hardware_interface::JointStateHandle state_handle_arm_wrist_left_yaw("arm_wrist_left_yaw", &arm_pos_[WRIST][LEFT][YAW], &arm_vel_[WRIST][LEFT][YAW], &arm_eff_[WRIST][LEFT][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_wrist_left_yaw);

  hardware_interface::JointHandle arm_pos_handle_arm_wrist_left_yaw(jnt_state_interface.getHandle("arm_wrist_left_yaw"), &arm_cmd_[WRIST][LEFT][YAW]);
  jnt_pos_interface.registerHandle(arm_pos_handle_arm_wrist_left_yaw);
  
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);

}

void BlueCrescent::read(ros::Time time, ros::Duration period)
{
}

void BlueCrescent::write(ros::Time time, ros::Duration period)
{
 
  // Real Robot functionality coding here...
  // below code is simulating real robot delay.	
  //pos_[0] = pos_[0] + 0.01* (cmd_[0] - pos_[0]);
  head_pos_[ROLL] = head_cmd_[ROLL];// + 0.01*(head_cmd_[ROLL] - head_pos_[ROLL]);
  head_pos_[YAW] = head_cmd_[YAW];// + 0.01*(head_cmd_[YAW] - head_pos_[YAW]);
  //ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  printf("%lf,%lf\n",head_pos_[ROLL],head_cmd_[ROLL]);
}

