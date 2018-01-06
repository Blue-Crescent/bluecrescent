#include <bluecrescent_control/RARMHW.h>

namespace bluecrescent_control{

RARMHW::RARMHW()
{
  // connect and register the joint state interface

//debug  fd_mux = wiringPiI2CSetup(0x70);
//debug  wiringPiI2CWriteReg8(fd_mux, 0x0 ,0x04);
//debug
//debug  fd_M0[0] = wiringPiI2CSetup(drv8830_addr_M0[0]);
//debug  fd_M0[1] = wiringPiI2CSetup(drv8830_addr_M0[1]);
//debug  fd_M1[0] = wiringPiI2CSetup(drv8830_addr_M1[0]);
//debug  fd_M1[1] = wiringPiI2CSetup(drv8830_addr_M1[1]);
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

void RARMHW::motor_release(){
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
void RARMHW::motor_lock(uint8_t arm,uint8_t joint){
	printf("MOTOR LOCKED!\n");
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[arm][joint][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[arm][joint][1],CONTROL,B(arm,joint));
#endif
}
void RARMHW::cwstep(uint8_t arm,uint8_t joint){
	lrotate(step[arm][joint].A);
	lrotate(step[arm][joint].nA);
	lrotate(step[arm][joint].B);
	lrotate(step[arm][joint].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[arm][joint][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[arm][joint][1],CONTROL,B(arm,joint));
#endif
}
void RARMHW::ccwstep(uint8_t arm,uint8_t joint){
	rrotate(step[arm][joint].A);
	rrotate(step[arm][joint].nA);
	rrotate(step[arm][joint].B);
	rrotate(step[arm][joint].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[arm][joint][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[arm][joint][1],CONTROL,B(arm,joint));
#endif
}

bool RARMHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
 	printf("This is RARMHW\n");

#ifndef NO_WIRINGPI
   	fd[SHOULDER][ROLL][0]  = wiringPiI2CSetup(drv8830_addr[SHOULDER][ROLL][0]);
   	fd[SHOULDER][ROLL][1]  = wiringPiI2CSetup(drv8830_addr[SHOULDER][ROLL][1]);
   	fd[SHOULDER][PITCH][0] = wiringPiI2CSetup(drv8830_addr[SHOULDER][PITCH][0]);
   	fd[SHOULDER][PITCH][1] = wiringPiI2CSetup(drv8830_addr[SHOULDER][PITCH][1]);
   	fd[ELBOW][ROLL][0]     = wiringPiI2CSetup(drv8830_addr[ELBOW][ROLL][0]);
   	fd[ELBOW][ROLL][1]     = wiringPiI2CSetup(drv8830_addr[ELBOW][ROLL][1]);
   	fd[ELBOW][YAW][0]      = wiringPiI2CSetup(drv8830_addr[ELBOW][YAW][0]);
   	fd[ELBOW][YAW][1]      = wiringPiI2CSetup(drv8830_addr[ELBOW][YAW][1]);
   	fd[WRIST][YAW][0]      = wiringPiI2CSetup(drv8830_addr[WRIST][YAW][0]);
   	fd[WRIST][YAW][1]      = wiringPiI2CSetup(drv8830_addr[WRIST][YAW][1]);
#endif
	
// SHOULDER ROLL PITCH
// ELBOW ROLL YAW
// WRIST YAW

  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_roll("arm_shoulder_right_roll", &rarm_pos_[SHOULDER][ROLL], &rarm_vel_[SHOULDER][ROLL], &rarm_eff_[SHOULDER][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_roll(jnt_state_interface.getHandle("arm_shoulder_right_roll"), &rarm_cmd_[SHOULDER][ROLL]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_roll);
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_pitch("arm_shoulder_right_pitch", &rarm_pos_[SHOULDER][PITCH], &rarm_vel_[SHOULDER][PITCH], &rarm_eff_[SHOULDER][PITCH]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_pitch);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_pitch(jnt_state_interface.getHandle("arm_shoulder_right_pitch"), &rarm_cmd_[SHOULDER][PITCH]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_pitch);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_yaw("arm_elbow_right_yaw", &rarm_pos_[ELBOW][YAW], &rarm_vel_[ELBOW][YAW], &rarm_eff_[ELBOW][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_yaw(jnt_state_interface.getHandle("arm_elbow_right_yaw"), &rarm_cmd_[ELBOW][YAW]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_yaw);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_roll("arm_elbow_right_roll", &rarm_pos_[ELBOW][ROLL], &rarm_vel_[ELBOW][ROLL], &rarm_eff_[ELBOW][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_roll(jnt_state_interface.getHandle("arm_elbow_right_roll"), &rarm_cmd_[ELBOW][ROLL]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_roll);

  hardware_interface::JointStateHandle state_handle_arm_wrist_right_yaw("arm_wrist_right_yaw", &rarm_pos_[WRIST][YAW], &rarm_vel_[WRIST][YAW], &rarm_eff_[WRIST][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_wrist_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_wrist_right_yaw(jnt_state_interface.getHandle("arm_wrist_right_yaw"), &rarm_cmd_[WRIST][YAW]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_wrist_right_yaw);
  
  
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);

  return true;
}
  
  
void RARMHW::read(const ros::Time& time,const ros::Duration& period)
{
}

void RARMHW::write(const ros::Time& time,const ros::Duration& period)
{

  int rarm_step_cmd_[3][3];

  rarm_step_cmd_[SHOULDER][ROLL]  = (int) RAD2STEP(rarm_cmd_[SHOULDER][ROLL]);
  rarm_step_cmd_[SHOULDER][PITCH] = (int) RAD2STEP(rarm_cmd_[SHOULDER][PITCH]);
  rarm_step_cmd_[ELBOW][ROLL]     = (int) RAD2STEP(rarm_cmd_[ELBOW][ROLL]);
  rarm_step_cmd_[ELBOW][YAW]      = (int) RAD2STEP(rarm_cmd_[ELBOW][YAW]);
  rarm_step_cmd_[WRIST][YAW]      = (int) RAD2STEP(rarm_cmd_[WRIST][YAW]);


  // BASIC TRAJECTORY
  // SHOULDER ROLL
  if(stepcnt[SHOULDER][ROLL]<rarm_step_cmd_[SHOULDER][ROLL]){
  	cwstep(SHOULDER,ROLL);
	stepcnt[SHOULDER][ROLL]++;
  }else if(stepcnt[SHOULDER][ROLL]>rarm_step_cmd_[SHOULDER][ROLL]){
 	ccwstep(SHOULDER,ROLL);
	stepcnt[SHOULDER][ROLL]--;
  }else{
	  //////motor_release();
  }
  // SHOULDER PITCH
  if(stepcnt[SHOULDER][PITCH]<rarm_step_cmd_[SHOULDER][PITCH]){
  	cwstep(SHOULDER,PITCH);
	stepcnt[SHOULDER][PITCH]++;
  }else if(stepcnt[SHOULDER][PITCH]>rarm_step_cmd_[SHOULDER][PITCH]){
 	ccwstep(SHOULDER,PITCH);
	stepcnt[SHOULDER][PITCH]--;
  }else{
	  //////motor_release();
  }
  // ELBOW ROLL
  if(stepcnt[ELBOW][ROLL]<rarm_step_cmd_[ELBOW][ROLL]){
  	cwstep(ELBOW,ROLL);
	stepcnt[ELBOW][ROLL]++;
  }else if(stepcnt[ELBOW][ROLL]>rarm_step_cmd_[ELBOW][ROLL]){
 	ccwstep(ELBOW,ROLL);
	stepcnt[ELBOW][ROLL]--;
  }else{
	  //////motor_release();
  }
  // ELBOW YAW
  if(stepcnt[ELBOW][YAW]<rarm_step_cmd_[ELBOW][YAW]){
  	cwstep(ELBOW,YAW);
	stepcnt[ELBOW][YAW]++;
  }else if(stepcnt[ELBOW][YAW]>rarm_step_cmd_[ELBOW][YAW]){
 	ccwstep(ELBOW,YAW);
	stepcnt[ELBOW][YAW]--;
  }else{
	  //////motor_release();
  }
  // WRIST YAW
  if(stepcnt[WRIST][YAW]<rarm_step_cmd_[WRIST][YAW]){
  	cwstep(WRIST,YAW);
	stepcnt[WRIST][YAW]++;
  }else if(stepcnt[WRIST][YAW]>rarm_step_cmd_[WRIST][YAW]){
 	ccwstep(WRIST,YAW);
	stepcnt[WRIST][YAW]--;
  }else{
	  //////motor_release();
  }

  //printstep(SHOULDER,ROLL);
  //printstep(SHOULDER,YAW);


 // No Feedback exists, dummy Feedback
  rarm_pos_[SHOULDER][ROLL]  = (int) STEP2RAD(stepcnt[SHOULDER][ROLL]  );
  rarm_pos_[SHOULDER][PITCH] = (int) STEP2RAD(stepcnt[SHOULDER][PITCH] );
  rarm_pos_[ELBOW][ROLL]     = (int) STEP2RAD(stepcnt[ELBOW][ROLL]     );
  rarm_pos_[ELBOW][YAW]      = (int) STEP2RAD(stepcnt[ELBOW][YAW]      );
  rarm_pos_[WRIST][YAW]      = (int) STEP2RAD(stepcnt[WRIST][YAW]      );

//ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  //ROS_DEBUG("%lf,%lf,%d,%d ",rarm_pos_[SHOULDER][ROLL],rarm_cmd_[SHOULDER][ROLL],stepcnt[SHOULDER][ROLL],rarm_step_cmd_[SHOULDER][ROLL]);
  //ROS_DEBUG("%lf,%lf,%d,%d\n",rarm_pos_[SHOULDER][PITCH],rarm_cmd_[SHOULDER][PITCH],stepcnt[SHOULDER][PITCH],rarm_step_cmd_[SHOULDER][PITCH]);
  
}
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::RARMHW, hardware_interface::RobotHW)
