#include <bluecrescent_control/LARMHW.h>

namespace bluecrescent_control{

LARMHW::LARMHW()
  {
}
void LARMHW::motor_release(){
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,0x18);
#endif
}
void LARMHW::motor_lock(uint8_t num){
	printf("MOTOR LOCKED!\n");
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
#endif
}
void LARMHW::cwstep(uint8_t num){
	lrotate(step[num].A);
	lrotate(step[num].nA);
	lrotate(step[num].B);
	lrotate(step[num].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
#endif
}
void LARMHW::ccwstep(uint8_t num){
	rrotate(step[num].A);
	rrotate(step[num].nA);
	rrotate(step[num].B);
	rrotate(step[num].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
#endif
}

bool LARMHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
  
	printf("This is LARMHW\n");
	
	stepcnt[0]=0; stepcnt[1]=0; stepcnt[2]=0; stepcnt[3]=0;
  	drv8830_addr_M0[0] = 0x60;
  	drv8830_addr_M0[1] = 0x61;
  	drv8830_addr_M1[0] = 0x62;
  	drv8830_addr_M1[1] = 0x63;

#ifndef NO_WIRINGPI
   	fd_M0[0] = wiringPiI2CSetup(drv8830_addr_M0[0]);
   	fd_M0[1] = wiringPiI2CSetup(drv8830_addr_M0[1]);
   	fd_M1[0] = wiringPiI2CSetup(drv8830_addr_M1[0]);
   	fd_M1[1] = wiringPiI2CSetup(drv8830_addr_M1[1]); 
#endif
	
	step[0].A=0xC1;
	step[0].nA=(0x1C << 1);
	step[0].B=0x70;
	step[0].nB=(0x07 << 1);
	
	step[1].A=0xC1;
	step[1].nA=(0x1C << 1);
	step[1].B=0x70;
	step[1].nB=(0x07 << 1);

	step[2].A=0xC1;
	step[2].nA=(0x1C << 1);
	step[2].B=0x70;
	step[2].nB=(0x07 << 1);

	step[3].A=0xC1;
	step[3].nA=(0x1C << 1);
	step[3].B=0x70;
	step[3].nB=(0x07 << 1);
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_left_roll("arm_shoulder_left_roll", &larm_pos_[SHOULDER][ROLL], &larm_vel_[SHOULDER][ROLL], &larm_eff_[SHOULDER][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_left_roll);

  hardware_interface::JointHandle larm_pos_handle_arm_shoulder_left_roll(jnt_state_interface.getHandle("arm_shoulder_left_roll"), &larm_cmd_[SHOULDER][ROLL]);
  jnt_pos_interface.registerHandle(larm_pos_handle_arm_shoulder_left_roll);
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_left_pitch("arm_shoulder_left_pitch", &larm_pos_[SHOULDER][PITCH], &larm_vel_[SHOULDER][PITCH], &larm_eff_[SHOULDER][PITCH]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_left_pitch);

  hardware_interface::JointHandle larm_pos_handle_arm_shoulder_left_pitch(jnt_state_interface.getHandle("arm_shoulder_left_pitch"), &larm_cmd_[SHOULDER][PITCH]);
  jnt_pos_interface.registerHandle(larm_pos_handle_arm_shoulder_left_pitch);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_left_yaw("arm_elbow_left_yaw", &larm_pos_[ELBOW][YAW], &larm_vel_[ELBOW][YAW], &larm_eff_[ELBOW][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_left_yaw);

  hardware_interface::JointHandle larm_pos_handle_arm_elbow_left_yaw(jnt_state_interface.getHandle("arm_elbow_left_yaw"), &larm_cmd_[ELBOW][YAW]);
  jnt_pos_interface.registerHandle(larm_pos_handle_arm_elbow_left_yaw);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_left_roll("arm_elbow_left_roll", &larm_pos_[ELBOW][ROLL], &larm_vel_[ELBOW][ROLL], &larm_eff_[ELBOW][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_left_roll);

  hardware_interface::JointHandle larm_pos_handle_arm_elbow_left_roll(jnt_state_interface.getHandle("arm_elbow_left_roll"), &larm_cmd_[ELBOW][ROLL]);
  jnt_pos_interface.registerHandle(larm_pos_handle_arm_elbow_left_roll);

  hardware_interface::JointStateHandle state_handle_arm_wrist_left_yaw("arm_wrist_left_yaw", &larm_pos_[WRIST][YAW], &larm_vel_[WRIST][YAW], &larm_eff_[WRIST][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_wrist_left_yaw);

  hardware_interface::JointHandle larm_pos_handle_arm_wrist_left_yaw(jnt_state_interface.getHandle("arm_wrist_left_yaw"), &larm_cmd_[WRIST][YAW]);
  jnt_pos_interface.registerHandle(larm_pos_handle_arm_wrist_left_yaw);
  
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);
  return true;
}

void LARMHW::read(ros::Time& time, ros::Duration& period)
{
}

void LARMHW::write(ros::Time& time, ros::Duration& period)
{
}
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::LARMHW, hardware_interface::RobotHW)
