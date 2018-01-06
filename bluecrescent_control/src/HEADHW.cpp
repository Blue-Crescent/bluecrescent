#include <bluecrescent_control/HEADHW.h>


namespace bluecrescent_control{


HEADHW::HEADHW()
  {
  //debug  fd_mux = wiringPiI2CSetup(0x70);
  //debug  wiringPiI2CWriteReg8(fd_mux, 0x0 ,0x04);
	
	head_cmd_[HEAD][ROLL]= 0;
	head_pos_[HEAD][ROLL]= 0;
	head_vel_[HEAD][ROLL]= 0;
	head_eff_[HEAD][ROLL]= 0;
	head_cmd_[HEAD][YAW]= 0;
	head_pos_[HEAD][YAW]= 0;
	head_vel_[HEAD][YAW]= 0;
	head_eff_[HEAD][YAW]= 0;

	step[HEAD][ROLL].A   = 0xC1;
	step[HEAD][ROLL].nA  = (0x1C << 1);
	step[HEAD][ROLL].B   = 0x70;
	step[HEAD][ROLL].nB  = (0x07 << 1);

	step[HEAD][YAW].A   = 0xC1;
	step[HEAD][YAW].nA  = (0x1C << 1);
	step[HEAD][YAW].B   = 0x70;
	step[HEAD][YAW].nB  = (0x07 << 1);

  	drv8830_addr[HEAD][ROLL][0] = 0x60;
  	drv8830_addr[HEAD][ROLL][1] = 0x61;
  	drv8830_addr[HEAD][YAW][0] = 0x62;
  	drv8830_addr[HEAD][YAW][1] = 0x63;

#ifndef NO_WIRINGPI
   	fd[HEAD][ROLL][0] = wiringPiI2CSetup(drv8830_addr[HEAD][ROLL][0]);
   	fd[HEAD][ROLL][1] = wiringPiI2CSetup(drv8830_addr[HEAD][ROLL][1]);
   	fd[HEAD][YAW][0] = wiringPiI2CSetup(drv8830_addr[HEAD][YAW][0]);
   	fd[HEAD][YAW][1] = wiringPiI2CSetup(drv8830_addr[HEAD][YAW][1]); 
#endif

        stepcnt[HEAD][ROLL] = 0;
        stepcnt[HEAD][YAW] = 0;
}

void HEADHW::motor_release(){
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[HEAD][YAW][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[HEAD][YAW][1],CONTROL,0x18);
#endif
}
void HEADHW::motor_lock(uint8_t arm,uint8_t joint){
	printf("MOTOR LOCKED!\n");
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][1],CONTROL,B(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][YAW][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][YAW][1],CONTROL,B(arm,joint));
#endif
}
void HEADHW::cwstep(uint8_t arm,uint8_t joint){
	lrotate(step[arm][joint].A);
	lrotate(step[arm][joint].nA);
	lrotate(step[arm][joint].B);
	lrotate(step[arm][joint].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][1],CONTROL,B(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][YAW][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][YAW][1],CONTROL,B(arm,joint));
#endif
}
void HEADHW::ccwstep(uint8_t arm,uint8_t joint){
	rrotate(step[arm][joint].A);
	rrotate(step[arm][joint].nA);
	rrotate(step[arm][joint].B);
	rrotate(step[arm][joint].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][ROLL][1],CONTROL,B(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][YAW][0],CONTROL,A(arm,joint));
	wiringPiI2CWriteReg8(fd[HEAD][YAW][1],CONTROL,B(arm,joint));
#endif
}

bool HEADHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
    	printf("This is HEADHW\n");
  
	//chatter_pub = robot_hw_nh.advertise<std_msgs::String>("chatter", 1000);

    	hardware_interface::JointStateHandle state_handle_head_yaw("head_yaw", &head_pos_[HEAD][YAW], &head_vel_[HEAD][YAW], &head_eff_[HEAD][YAW]);
    	hardware_interface::JointStateHandle state_handle_head_roll("head_roll", &head_pos_[HEAD][ROLL], &head_vel_[HEAD][ROLL], &head_eff_[HEAD][ROLL]);
	jnt_state_interface.registerHandle(state_handle_head_yaw);
    	jnt_state_interface.registerHandle(state_handle_head_roll);
    	
	hardware_interface::JointHandle pos_handle_head_roll(jnt_state_interface.getHandle("head_roll"), &head_cmd_[HEAD][ROLL]);
    	hardware_interface::JointHandle pos_handle_head_yaw(jnt_state_interface.getHandle("head_yaw"), &head_cmd_[HEAD][YAW]);
    	jnt_pos_interface.registerHandle(pos_handle_head_roll);
    	jnt_pos_interface.registerHandle(pos_handle_head_yaw);
    	
    	registerInterface(&jnt_state_interface); registerInterface(&jnt_pos_interface);
	return true;
}
void HEADHW::read(const ros::Time& time,const ros::Duration& period)
{
}

void HEADHW::write(const ros::Time& time,const ros::Duration& period)
{
  int head_step_cmd_[1][3];

  head_step_cmd_[HEAD][ROLL] =(int) RAD2STEP(head_cmd_[HEAD][ROLL]);
  head_step_cmd_[HEAD][YAW] =(int) RAD2STEP(head_cmd_[HEAD][YAW]);
  
//  std_msgs::String msg;
//  std::stringstream ss;
//  ss << "hello world " ;
//  msg.data = ss.str();
//  ROS_INFO("%s", msg.data.c_str());
  
  //chatter_pub.publish(msg);

  if(stepcnt[HEAD][ROLL]<head_step_cmd_[HEAD][ROLL]){
  	HEADHW::cwstep(HEAD,ROLL);
	stepcnt[HEAD][ROLL]++;
  }else if(stepcnt[HEAD][ROLL]>head_step_cmd_[HEAD][ROLL]){
 	HEADHW::ccwstep(HEAD,ROLL);
	stepcnt[HEAD][ROLL]--;
  }else{
	  //////motor_release();
  }
  if(stepcnt[HEAD][YAW]<head_step_cmd_[HEAD][YAW]){ 
	  HEADHW::ccwstep(HEAD,YAW); 
	  stepcnt[HEAD][YAW]++;
  }else if(stepcnt[HEAD][YAW]>head_step_cmd_[HEAD][YAW]){
  	HEADHW::cwstep(HEAD,YAW);
	stepcnt[HEAD][YAW]--;
  }else{
	  //motor_release();
  }

  printstep(HEAD,ROLL);
  printstep(HEAD,YAW);

  head_pos_[HEAD][ROLL] =(double)STEP2RAD(stepcnt[HEAD][ROLL]);
  head_pos_[HEAD][YAW]  =(double)STEP2RAD(stepcnt[HEAD][YAW]);

  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  ROS_DEBUG("H:%lf , %lf , %d , %d "  , head_pos_[HEAD][ROLL], RAD2DEG(head_cmd_[HEAD][ROLL]) , stepcnt[HEAD][ROLL] , head_step_cmd_[HEAD][ROLL]);
  ROS_DEBUG("H:%lf , %lf , %d , %d " , head_pos_[HEAD][YAW] , RAD2DEG(head_cmd_[HEAD][YAW] ) , stepcnt[HEAD][YAW]  , head_step_cmd_[HEAD][YAW]);
  
}
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::HEADHW, hardware_interface::RobotHW)
