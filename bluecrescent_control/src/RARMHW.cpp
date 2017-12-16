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
 printf("This is bluecrescnet_hw_rarm\n");

}

void RARMHW::motor_release(){
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,0x18);
#endif
}
void RARMHW::motor_lock(uint8_t num){
	printf("MOTOR LOCKED!\n");
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
#endif
}
void RARMHW::cwstep(uint8_t num){
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
void RARMHW::ccwstep(uint8_t num){
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

bool RARMHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
	
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
//  // Real Robot functionality coding here...
//  // below code is simulating real robot delay.	
//
//  int larm_step_cmd_[4];
//
//  larm_step_cmd_[ROLL] =(int) (PI_step * larm_cmd_[ROLL]/PI);
//  larm_step_cmd_[YAW] =(int) (PI_step * larm_cmd_[YAW]/PI);
//
//  if(stepcnt[ROLL]<larm_step_cmd_[ROLL]){
//  	cwstep(ROLL);
//	stepcnt[ROLL]++;
//  }else if(stepcnt[ROLL]>larm_step_cmd_[ROLL]){
// 	ccwstep(ROLL);
//	stepcnt[ROLL]--;
//  }else{
//	  //////motor_release();
//  }
//  if(stepcnt[YAW]<larm_step_cmd_[YAW]){
// 	ccwstep(YAW);
//	stepcnt[YAW]++;
//  }else if(stepcnt[YAW]>larm_step_cmd_[YAW]){
//  	cwstep(YAW);
//	stepcnt[YAW]--;
//  }else{
//	  //motor_release();
//  }
//
//  printstep(ROLL);
//  printstep(YAW);
//
//  larm_pos_[ROLL] =(int) stepcnt[ROLL] * PI / PI_step;
//  larm_pos_[YAW] =(int) stepcnt[YAW] * PI / PI_step;
//
////ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
//  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
//  printf("%lf,%lf,%d,%d ",larm_pos_[ROLL],larm_cmd_[ROLL],stepcnt[ROLL],larm_step_cmd_[ROLL]);
//  printf("%lf,%lf,%d,%d\n",larm_pos_[YAW],larm_cmd_[YAW],stepcnt[YAW],larm_step_cmd_[YAW]);
//  
//  //larm_pos_[ROLL] = larm_cmd_[ROLL];// + 0.01*(larm_cmd_[ROLL] - larm_pos_[ROLL]);
//  //larm_pos_[YAW] = larm_cmd_[YAW];// + 0.01*(larm_cmd_[YAW] - larm_pos_[YAW]);
}
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::RARMHW, hardware_interface::RobotHW)
