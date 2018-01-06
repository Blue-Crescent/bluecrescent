#include <bluecrescent_control/HEADHW.h>


namespace bluecrescent_control{


HEADHW::HEADHW()
  {
	
	head_cmd_[ROLL]= 0;
	head_pos_[ROLL]= 0;
	head_vel_[ROLL]= 0;
	head_eff_[ROLL]= 0;
	head_cmd_[YAW]= 0;
	head_pos_[YAW]= 0;
	head_vel_[YAW]= 0;
	head_eff_[YAW]= 0;

	step[ROLL].A   = 0xC1;
	step[ROLL].nA  = (0x1C << 1);
	step[ROLL].B   = 0x70;
	step[ROLL].nB  = (0x07 << 1);

	step[YAW].A   = 0xC1;
	step[YAW].nA  = (0x1C << 1);
	step[YAW].B   = 0x70;
	step[YAW].nB  = (0x07 << 1);

  	drv8830_addr[ROLL][0] = 0x60;
  	drv8830_addr[ROLL][1] = 0x61;
  	drv8830_addr[YAW][0] = 0x62;
  	drv8830_addr[YAW][1] = 0x63;

#ifndef NO_WIRINGPI
  	fd_mux[0] = wiringPiI2CSetup(0x70);
  	fd_mux[1] = wiringPiI2CSetup(0x72);
   	fd[ROLL][0] = wiringPiI2CSetup(drv8830_addr[ROLL][0]);
   	fd[ROLL][1] = wiringPiI2CSetup(drv8830_addr[ROLL][1]);
   	fd[YAW][0] = wiringPiI2CSetup(drv8830_addr[YAW][0]);
   	fd[YAW][1] = wiringPiI2CSetup(drv8830_addr[YAW][1]); 
#endif

        stepcnt[ROLL] = 0;
        stepcnt[YAW] = 0;
}

void HEADHW::motor_release(){
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[ROLL][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[ROLL][1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[YAW][0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[YAW][1],CONTROL,0x18);
#endif
}
void HEADHW::motor_lock(uint8_t joint){
	printf("MOTOR LOCKED!\n");
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[ROLL][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[ROLL][1],CONTROL,B(joint));
	wiringPiI2CWriteReg8(fd[YAW][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[YAW][1],CONTROL,B(joint));
#endif
}
void HEADHW::cwstep(uint8_t joint){
	lrotate(step[joint].A);
	lrotate(step[joint].nA);
	lrotate(step[joint].B);
	lrotate(step[joint].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[ROLL][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[ROLL][1],CONTROL,B(joint));
	wiringPiI2CWriteReg8(fd[YAW][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[YAW][1],CONTROL,B(joint));
#endif
}
void HEADHW::ccwstep(uint8_t joint){
	rrotate(step[joint].A);
	rrotate(step[joint].nA);
	rrotate(step[joint].B);
	rrotate(step[joint].nB);
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[ROLL][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[ROLL][1],CONTROL,B(joint));
	wiringPiI2CWriteReg8(fd[YAW][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[YAW][1],CONTROL,B(joint));
#endif
}

bool HEADHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
    	printf("This is HEADHW\n");
  
	//chatter_pub = robot_hw_nh.advertise<std_msgs::String>("chatter", 1000);

    	hardware_interface::JointStateHandle state_handle_head_yaw("head_yaw", &head_pos_[YAW], &head_vel_[YAW], &head_eff_[YAW]);
    	hardware_interface::JointStateHandle state_handle_head_roll("head_roll", &head_pos_[ROLL], &head_vel_[ROLL], &head_eff_[ROLL]);
	jnt_state_interface.registerHandle(state_handle_head_yaw);
    	jnt_state_interface.registerHandle(state_handle_head_roll);
    	
	hardware_interface::JointHandle pos_handle_head_roll(jnt_state_interface.getHandle("head_roll"), &head_cmd_[ROLL]);
    	hardware_interface::JointHandle pos_handle_head_yaw(jnt_state_interface.getHandle("head_yaw"), &head_cmd_[YAW]);
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
  int head_step_cmd_[3];

  wiringPiI2CWriteReg8(fd_mux[0], 0x0 ,0x04);
  wiringPiI2CWriteReg8(fd_mux[1], 0x0 ,0x00);
  

  head_step_cmd_[ROLL] =(int) RAD2STEP(head_cmd_[ROLL]);
  head_step_cmd_[YAW] =(int) RAD2STEP(head_cmd_[YAW]);
  
//  std_msgs::String msg;
//  std::stringstream ss;
//  ss << "hello world " ;
//  msg.data = ss.str();
//  ROS_INFO("%s", msg.data.c_str());
  
  //chatter_pub.publish(msg);

  if(stepcnt[ROLL]<head_step_cmd_[ROLL]){
  	cwstep(ROLL);
	stepcnt[ROLL]++;
  	printstep(ROLL);
  }else if(stepcnt[ROLL]>head_step_cmd_[ROLL]){
 	ccwstep(ROLL);
	stepcnt[ROLL]--;
  	printstep(ROLL);
  }else{
	  //////motor_release();
  }
  if(stepcnt[YAW]<head_step_cmd_[YAW]){ 
	ccwstep(YAW); 
	stepcnt[YAW]++;
  	printstep(YAW);
  }else if(stepcnt[YAW]>head_step_cmd_[YAW]){
  	cwstep(YAW);
	stepcnt[YAW]--;
  	printstep(YAW);
  }else{
	  //motor_release();
  }

  head_pos_[ROLL] =(double)STEP2RAD(stepcnt[ROLL]);
  head_pos_[YAW]  =(double)STEP2RAD(stepcnt[YAW]);

  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  ROS_DEBUG("H:%lf , %lf , %d , %d "  , head_pos_[ROLL], RAD2DEG(head_cmd_[ROLL]) , stepcnt[ROLL] , head_step_cmd_[ROLL]);
  ROS_DEBUG("H:%lf , %lf , %d , %d " , head_pos_[YAW] , RAD2DEG(head_cmd_[YAW] ) , stepcnt[YAW]  , head_step_cmd_[YAW]);
  
}
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::HEADHW, hardware_interface::RobotHW)
