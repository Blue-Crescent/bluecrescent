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
	step[SHOULDER_R].A   = 0xC1;
	step[SHOULDER_R].nA  = (0x1C << 1);
	step[SHOULDER_R].B   = 0x70;
	step[SHOULDER_R].nB  = (0x07 << 1);

	step[SHOULDER_P].A  = 0xC1;
	step[SHOULDER_P].nA = (0x1C << 1);
	step[SHOULDER_P].B  = 0x70;
	step[SHOULDER_P].nB = (0x07 << 1);

	step[ELBOW_R].A      = 0xC1;
	step[ELBOW_R].nA     = (0x1C << 1);
	step[ELBOW_R].B      = 0x70;
	step[ELBOW_R].nB     = (0x07 << 1);

	step[ELBOW_Y].A       = 0xC1;
	step[ELBOW_Y].nA      = (0x1C << 1);
	step[ELBOW_Y].B       = 0x70;
	step[ELBOW_Y].nB      = (0x07 << 1);

	step[WRIST_Y].A       = 0xC1;
	step[WRIST_Y].nA      = (0x1C << 1);
	step[WRIST_Y].B       = 0x70;
	step[WRIST_Y].nB      = (0x07 << 1);

	stepcnt[SHOULDER_R] = 0;
	stepcnt[SHOULDER_P] = 0;
	stepcnt[ELBOW_R] = 0;
	stepcnt[ELBOW_Y] = 0;
	stepcnt[WRIST_Y] = 0;

	drv8830_addr[SHOULDER_R][0] = 0x60;
	drv8830_addr[SHOULDER_R][1] = 0x61;
	drv8830_addr[SHOULDER_P][0] = 0x62;
	drv8830_addr[SHOULDER_P][1] = 0x63;
	drv8830_addr[ELBOW_R][0] = 0x64;
	drv8830_addr[ELBOW_R][1] = 0x65;
	drv8830_addr[ELBOW_Y][0] = 0x66;
	drv8830_addr[ELBOW_Y][1] = 0x67;
	drv8830_addr[WRIST_Y][0] = 0x68;
	drv8830_addr[WRIST_Y][1] = 0x69;

}

void RARMHW::motor_release(){
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[SHOULDER_R][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[SHOULDER_R][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[SHOULDER_P][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[SHOULDER_P][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW_R][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW_R][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW_Y][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ELBOW_Y][1],CONTROL,0x18);
#endif
}
void RARMHW::motor_lock(uint8_t joint){
#ifndef NO_WIRINGPI
//	wiringPiI2CWriteReg8(fd[joint][0],CONTROL,A(arm,joint));
//	wiringPiI2CWriteReg8(fd[joint][1],CONTROL,B(arm,joint));
#endif
}
void RARMHW::cwstep(uint8_t joint){
	lrotate(step[joint].A);
	lrotate(step[joint].nA);
	lrotate(step[joint].B);
	lrotate(step[joint].nB);
#ifndef NO_WIRINGPI
//	wiringPiI2CWriteReg8(fd[joint][0],CONTROL,A(arm,joint));
//	wiringPiI2CWriteReg8(fd[joint][1],CONTROL,B(arm,joint));
#endif
}
void RARMHW::ccwstep(uint8_t joint){
	rrotate(step[joint].A);
	rrotate(step[joint].nA);
	rrotate(step[joint].B);
	rrotate(step[joint].nB);
#ifndef NO_WIRINGPI
//	wiringPiI2CWriteReg8(fd[joint][0],CONTROL,A(arm,joint));
//	wiringPiI2CWriteReg8(fd[joint][1],CONTROL,B(arm,joint));
#endif
}

bool RARMHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
 	printf("This is RARMHW\n");

#ifndef NO_WIRINGPI
   	fd[SHOULDER_R][0]  = wiringPiI2CSetup(drv8830_addr[SHOULDER_R][0]);
   	fd[SHOULDER_R][1]  = wiringPiI2CSetup(drv8830_addr[SHOULDER_R][1]);
   	fd[SHOULDER_P][0] = wiringPiI2CSetup(drv8830_addr[SHOULDER_P][0]);
   	fd[SHOULDER_P][1] = wiringPiI2CSetup(drv8830_addr[SHOULDER_P][1]);
   	fd[ELBOW_R][0]     = wiringPiI2CSetup(drv8830_addr[ELBOW_R][0]);
   	fd[ELBOW_R][1]     = wiringPiI2CSetup(drv8830_addr[ELBOW_R][1]);
   	fd[ELBOW_Y][0]      = wiringPiI2CSetup(drv8830_addr[ELBOW_Y][0]);
   	fd[ELBOW_Y][1]      = wiringPiI2CSetup(drv8830_addr[ELBOW_Y][1]);
   	fd[WRIST_Y][0]      = wiringPiI2CSetup(drv8830_addr[WRIST_Y][0]);
   	fd[WRIST_Y][1]      = wiringPiI2CSetup(drv8830_addr[WRIST_Y][1]);
#endif

// SHOULDER ROLL PITCH
// ELBOW ROLL YAW
// WRIST YAW

  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_roll("arm_shoulder_right_roll", &rarm_pos_[SHOULDER_R], &rarm_vel_[SHOULDER_R], &rarm_eff_[SHOULDER_R]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_roll(jnt_state_interface.getHandle("arm_shoulder_right_roll"), &rarm_cmd_[SHOULDER_R]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_roll);

  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_pitch("arm_shoulder_right_pitch", &rarm_pos_[SHOULDER_P], &rarm_vel_[SHOULDER_P], &rarm_eff_[SHOULDER_P]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_pitch);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_pitch(jnt_state_interface.getHandle("arm_shoulder_right_pitch"), &rarm_cmd_[SHOULDER_P]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_pitch);

  hardware_interface::JointStateHandle state_handle_arm_elbow_right_yaw("arm_elbow_right_yaw", &rarm_pos_[ELBOW_Y], &rarm_vel_[ELBOW_Y], &rarm_eff_[ELBOW_Y]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_yaw(jnt_state_interface.getHandle("arm_elbow_right_yaw"), &rarm_cmd_[ELBOW_Y]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_yaw);

  hardware_interface::JointStateHandle state_handle_arm_elbow_right_roll("arm_elbow_right_roll", &rarm_pos_[ELBOW_R], &rarm_vel_[ELBOW_R], &rarm_eff_[ELBOW_R]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_roll(jnt_state_interface.getHandle("arm_elbow_right_roll"), &rarm_cmd_[ELBOW_R]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_roll);

  hardware_interface::JointStateHandle state_handle_arm_wrist_right_yaw("arm_wrist_right_yaw", &rarm_pos_[WRIST_Y], &rarm_vel_[WRIST_Y], &rarm_eff_[WRIST_Y]);
  jnt_state_interface.registerHandle(state_handle_arm_wrist_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_wrist_right_yaw(jnt_state_interface.getHandle("arm_wrist_right_yaw"), &rarm_cmd_[WRIST_Y]);
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

  int rarm_step_cmd_[5];

  rarm_step_cmd_[SHOULDER_R]  = (int) RAD2STEP(rarm_cmd_[SHOULDER_R]);
  rarm_step_cmd_[SHOULDER_P] = (int) RAD2STEP(rarm_cmd_[SHOULDER_P]);
  rarm_step_cmd_[ELBOW_R]     = (int) RAD2STEP(rarm_cmd_[ELBOW_R]);
  rarm_step_cmd_[ELBOW_Y]      = (int) RAD2STEP(rarm_cmd_[ELBOW_Y]);
  rarm_step_cmd_[WRIST_Y]      = (int) RAD2STEP(rarm_cmd_[WRIST_Y]);


  // BASIC TRAJECTORY
  // SHOULDER ROLL
  if(stepcnt[SHOULDER_R]<rarm_step_cmd_[SHOULDER_R]){
  	cwstep(SHOULDER_R);
	stepcnt[SHOULDER_R]++;
  }else if(stepcnt[SHOULDER_R]>rarm_step_cmd_[SHOULDER_R]){
 	ccwstep(SHOULDER_R);
	stepcnt[SHOULDER_R]--;
  }else{
	  //////motor_release();
  }
  // SHOULDER PITCH
  if(stepcnt[SHOULDER_P]<rarm_step_cmd_[SHOULDER_P]){
  	cwstep(SHOULDER_P);
	stepcnt[SHOULDER_P]++;
  }else if(stepcnt[SHOULDER_P]>rarm_step_cmd_[SHOULDER_P]){
 	ccwstep(SHOULDER_P);
	stepcnt[SHOULDER_P]--;
  }else{
	  //////motor_release();
  }
  // ELBOW ROLL
  if(stepcnt[ELBOW_R]<rarm_step_cmd_[ELBOW_R]){
  	cwstep(ELBOW_R);
	stepcnt[ELBOW_R]++;
  }else if(stepcnt[ELBOW_R]>rarm_step_cmd_[ELBOW_R]){
 	ccwstep(ELBOW_R);
	stepcnt[ELBOW_R]--;
  }else{
	  //////motor_release();
  }
  // ELBOW YAW
  if(stepcnt[ELBOW_Y]<rarm_step_cmd_[ELBOW_Y]){
  	cwstep(ELBOW_Y);
	stepcnt[ELBOW_Y]++;
  }else if(stepcnt[ELBOW_Y]>rarm_step_cmd_[ELBOW_Y]){
 	ccwstep(ELBOW_Y);
	stepcnt[ELBOW_Y]--;
  }else{
	  //////motor_release();
  }
  // WRIST YAW
  if(stepcnt[WRIST_Y]<rarm_step_cmd_[WRIST_Y]){
  	cwstep(WRIST_Y);
	stepcnt[WRIST_Y]++;
  }else if(stepcnt[WRIST_Y]>rarm_step_cmd_[WRIST_Y]){
 	ccwstep(WRIST_Y);
	stepcnt[WRIST_Y]--;
  }else{
	  //////motor_release();
  }

  //printstep(SHOULDER_R);
  //printstep(SHOULDER,YAW);


 // No Feedback exists, dummy Feedback
  rarm_pos_[SHOULDER_R]  = (int) STEP2RAD(stepcnt[SHOULDER_R]  );
  rarm_pos_[SHOULDER_P] = (int) STEP2RAD(stepcnt[SHOULDER_P] );
  rarm_pos_[ELBOW_R]     = (int) STEP2RAD(stepcnt[ELBOW_R]     );
  rarm_pos_[ELBOW_Y]      = (int) STEP2RAD(stepcnt[ELBOW_Y]      );
  rarm_pos_[WRIST_Y]      = (int) STEP2RAD(stepcnt[WRIST_Y]      );

//ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  //ROS_DEBUG("%lf,%lf,%d,%d ",rarm_pos_[SHOULDER_R],rarm_cmd_[SHOULDER_R],stepcnt[SHOULDER_R],rarm_step_cmd_[SHOULDER_R]);
  //ROS_DEBUG("%lf,%lf,%d,%d\n",rarm_pos_[SHOULDER_P],rarm_cmd_[SHOULDER_P],stepcnt[SHOULDER_P],rarm_step_cmd_[SHOULDER_P]);

}
RARMHW::~RARMHW()
  {
#ifndef NO_WIRINGPI
//Debug  	wiringPiI2CWriteReg8(fd_mux[0], 0x0 ,0x04);
//Debug  	wiringPiI2CWriteReg8(fd_mux[1], 0x0 ,0x00);
#endif
	motor_release();
	printf("Motor driver off : RARMHW\n");
  }
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::RARMHW, hardware_interface::RobotHW)
