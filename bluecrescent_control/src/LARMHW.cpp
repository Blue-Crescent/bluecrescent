#include <bluecrescent_control/LARMHW.h>

namespace bluecrescent_control{

LARMHW::LARMHW()
{
  // connect and register the joint state interface

//debug  fd_mux = wiringPiI2CSetup(0x70);
//debug  wiringPiI2CWriteReg8(fd_mux, 0x0 ,0x04);
//debug
	step[SHOULDER][ROLL].A   = 0xC1;
	step[SHOULDER][ROLL].nA  = (0x1C << 1);
	step[SHOULDER][ROLL].B   = 0x70;
	step[SHOULDER][ROLL].nB  = (0x07 << 1);
        
	step[SHOULDER][PITCH].A  = 0xC1;
	step[SHOULDER][PITCH].nA = (0x1C << 1);
	step[SHOULDER][PITCH].B  = 0x70;
	step[SHOULDER][PITCH].nB = (0x07 << 1);

	step[ELBOW][ROLL].A      = 0xC1;
	step[ELBOW][ROLL].nA     = (0x1C << 1);
	step[ELBOW][ROLL].B      = 0x70;
	step[ELBOW][ROLL].nB     = (0x07 << 1);

	step[ELBOW][YAW].A       = 0xC1;
	step[ELBOW][YAW].nA      = (0x1C << 1);
	step[ELBOW][YAW].B       = 0x70;
	step[ELBOW][YAW].nB      = (0x07 << 1);

	step[WRIST][YAW].A       = 0xC1;
	step[WRIST][YAW].nA      = (0x1C << 1);
	step[WRIST][YAW].B       = 0x70;
	step[WRIST][YAW].nB      = (0x07 << 1);

	stepcnt[SHOULDER][ROLL] = 0;
	stepcnt[SHOULDER][PITCH] = 0;
	stepcnt[ELBOW][ROLL] = 0;
	stepcnt[ELBOW][YAW] = 0;
	stepcnt[WRIST][YAW] = 0;
  	
	drv8830_addr[SHOULDER][ROLL][0] = 0x60;
	drv8830_addr[SHOULDER][ROLL][1] = 0x61;
	drv8830_addr[SHOULDER][PITCH][0] = 0x62;
	drv8830_addr[SHOULDER][PITCH][1] = 0x63;
	drv8830_addr[ELBOW][ROLL][0] = 0x64;
	drv8830_addr[ELBOW][ROLL][1] = 0x65;
	drv8830_addr[ELBOW][YAW][0] = 0x66;
	drv8830_addr[ELBOW][YAW][1] = 0x67;
	drv8830_addr[WRIST][YAW][0] = 0x68;
	drv8830_addr[WRIST][YAW][1] = 0x69;

}

void LARMHW::motor_release(){
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[SHOULDER][ROLL][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[SHOULDER][ROLL][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[SHOULDER][PITCH][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[SHOULDER][PITCH][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW][ROLL][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW][ROLL][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW][YAW][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW][YAW][1],CONTROL,0x18);
#endif
}
void LARMHW::motor_lock(uint8_t arm,uint8_t joint){
#ifndef NO_WIRINGPI
//	wiringPiI2CWriteReg8(fd[arm][joint][0],CONTROL,A(arm,joint));
//	wiringPiI2CWriteReg8(fd[arm][joint][1],CONTROL,B(arm,joint));
#endif
}
void LARMHW::LARMHW::cwstep(uint8_t arm,uint8_t joint){
	lrotate(step[arm][joint].A);
	lrotate(step[arm][joint].nA);
	lrotate(step[arm][joint].B);
	lrotate(step[arm][joint].nB);
#ifndef NO_WIRINGPI
//	wiringPiI2CWriteReg8(fd[arm][joint][0],CONTROL,A(arm,joint));
//	wiringPiI2CWriteReg8(fd[arm][joint][1],CONTROL,B(arm,joint));
#endif
}
void LARMHW::LARMHW::ccwstep(uint8_t arm,uint8_t joint){
	rrotate(step[arm][joint].A);
	rrotate(step[arm][joint].nA);
	rrotate(step[arm][joint].B);
	rrotate(step[arm][joint].nB);
#ifndef NO_WIRINGPI
//	wiringPiI2CWriteReg8(fd[arm][joint][0],CONTROL,A(arm,joint));
//	wiringPiI2CWriteReg8(fd[arm][joint][1],CONTROL,B(arm,joint));
#endif
}

bool LARMHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
 	printf("This is LARMHW\n");

#ifndef NO_WIRINGPI
   	fd[SHOULDER][ROLL][0] = wiringPiI2CSetup(drv8830_addr[SHOULDER][ROLL][0]); 
   	fd[SHOULDER][ROLL][1] = wiringPiI2CSetup(drv8830_addr[SHOULDER][ROLL][1]); 
   	fd[SHOULDER][PITCH][0] = wiringPiI2CSetup(drv8830_addr[SHOULDER][PITCH][0]); 
   	fd[SHOULDER][PITCH][1] = wiringPiI2CSetup(drv8830_addr[SHOULDER][PITCH][1]); 
   	fd[ELBOW][ROLL][0] = wiringPiI2CSetup(drv8830_addr[ELBOW][ROLL][0]); 
   	fd[ELBOW][ROLL][1] = wiringPiI2CSetup(drv8830_addr[ELBOW][ROLL][1]); 
   	fd[ELBOW][YAW][0] = wiringPiI2CSetup(drv8830_addr[ELBOW][YAW][0]); 
   	fd[ELBOW][YAW][1] = wiringPiI2CSetup(drv8830_addr[ELBOW][YAW][1]); 
   	fd[WRIST][YAW][0] = wiringPiI2CSetup(drv8830_addr[WRIST][YAW][0]); 
   	fd[WRIST][YAW][1] = wiringPiI2CSetup(drv8830_addr[WRIST][YAW][1]); 
#endif
	
// SHOULDER ROLL PITCH
// ELBOW ROLL YAW
// WRIST YAW

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
  
  
void LARMHW::read(const ros::Time& time,const ros::Duration& period)
{
}

void LARMHW::write(const ros::Time& time,const ros::Duration& period)
{

  int larm_step_cmd_[3][3];

  larm_step_cmd_[SHOULDER][ROLL]  = (int) RAD2STEP(larm_cmd_[SHOULDER][ROLL]);
  larm_step_cmd_[SHOULDER][PITCH] = (int) RAD2STEP(larm_cmd_[SHOULDER][PITCH]);
  larm_step_cmd_[ELBOW][ROLL]     = (int) RAD2STEP(larm_cmd_[ELBOW][ROLL]);
  larm_step_cmd_[ELBOW][YAW]      = (int) RAD2STEP(larm_cmd_[ELBOW][YAW]);
  larm_step_cmd_[WRIST][YAW]      = (int) RAD2STEP(larm_cmd_[WRIST][YAW]);


  // BASIC TRAJECTORY
  // SHOULDER ROLL
  if(stepcnt[SHOULDER][ROLL]<larm_step_cmd_[SHOULDER][ROLL]){
  	LARMHW::cwstep(SHOULDER,ROLL);
	stepcnt[SHOULDER][ROLL]++;
  }else if(stepcnt[SHOULDER][ROLL]>larm_step_cmd_[SHOULDER][ROLL]){
 	LARMHW::ccwstep(SHOULDER,ROLL);
	stepcnt[SHOULDER][ROLL]--;
  }else{
	  //////motor_release();
  }
  // SHOULDER PITCH
  if(stepcnt[SHOULDER][PITCH]<larm_step_cmd_[SHOULDER][PITCH]){
  	LARMHW::cwstep(SHOULDER,PITCH);
	stepcnt[SHOULDER][PITCH]++;
  }else if(stepcnt[SHOULDER][PITCH]>larm_step_cmd_[SHOULDER][PITCH]){
 	LARMHW::ccwstep(SHOULDER,PITCH);
	stepcnt[SHOULDER][PITCH]--;
  }else{
	  //////motor_release();
  }
  // ELBOW ROLL
  if(stepcnt[ELBOW][ROLL]<larm_step_cmd_[ELBOW][ROLL]){
  	LARMHW::cwstep(ELBOW,ROLL);
	stepcnt[ELBOW][ROLL]++;
  }else if(stepcnt[ELBOW][ROLL]>larm_step_cmd_[ELBOW][ROLL]){
 	LARMHW::ccwstep(ELBOW,ROLL);
	stepcnt[ELBOW][ROLL]--;
  }else{
	  //////motor_release();
  }
  // ELBOW YAW
  if(stepcnt[ELBOW][YAW]<larm_step_cmd_[ELBOW][YAW]){
  	LARMHW::cwstep(ELBOW,YAW);
	stepcnt[ELBOW][YAW]++;
  }else if(stepcnt[ELBOW][YAW]>larm_step_cmd_[ELBOW][YAW]){
 	LARMHW::ccwstep(ELBOW,YAW);
	stepcnt[ELBOW][YAW]--;
  }else{
	  //////motor_release();
  }
  // WRIST YAW
  if(stepcnt[WRIST][YAW]<larm_step_cmd_[WRIST][YAW]){
  	LARMHW::cwstep(WRIST,YAW);
	stepcnt[WRIST][YAW]++;
  }else if(stepcnt[WRIST][YAW]>larm_step_cmd_[WRIST][YAW]){
 	LARMHW::ccwstep(WRIST,YAW);
	stepcnt[WRIST][YAW]--;
  }else{
	  //////motor_release();
  }

  //printstep(SHOULDER,ROLL);
  //printstep(SHOULDER,YAW);

  larm_pos_[SHOULDER][ROLL]  = (int) STEP2RAD(stepcnt[SHOULDER][ROLL]  );
  larm_pos_[SHOULDER][PITCH] = (int) STEP2RAD(stepcnt[SHOULDER][PITCH] );
  larm_pos_[ELBOW][ROLL]     = (int) STEP2RAD(stepcnt[ELBOW][ROLL]     );
  larm_pos_[ELBOW][YAW]      = (int) STEP2RAD(stepcnt[ELBOW][YAW]      );
  larm_pos_[WRIST][YAW]      = (int) STEP2RAD(stepcnt[WRIST][YAW]      );

//ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  //ROS_DEBUG("%lf,%lf,%d,%d ",larm_pos_[SHOULDER][ROLL],larm_cmd_[SHOULDER][ROLL],stepcnt[SHOULDER][ROLL],larm_step_cmd_[SHOULDER][ROLL]);
  //ROS_DEBUG("%lf,%lf,%d,%d\n",larm_pos_[SHOULDER][PITCH],larm_cmd_[SHOULDER][PITCH],stepcnt[SHOULDER][PITCH],larm_step_cmd_[SHOULDER][PITCH]);
  
}
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::LARMHW, hardware_interface::RobotHW)
