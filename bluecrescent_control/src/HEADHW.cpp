#include <bluecrescent_control/HEADHW.h>


namespace bluecrescent_control{

HEADHW::HEADHW()
  {
    // connect and register the joint state interface
    //signal(SIGINT, mySigIntHandler);
  //debug  fd_mux = wiringPiI2CSetup(0x70);
  //debug  wiringPiI2CWriteReg8(fd_mux, 0x0 ,0x04);
  //debug

  
  
  
  }

void HEADHW::motor_release(){
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,0x18);
#endif
}
void HEADHW::motor_lock(uint8_t num){
	printf("MOTOR LOCKED!\n");
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
#endif
}
void HEADHW::cwstep(uint8_t num){
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
void HEADHW::ccwstep(uint8_t num){
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

bool HEADHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
    	printf("This is HEADHW\n");
	
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

    	hardware_interface::JointStateHandle state_handle_head_yaw("head_yaw", &head_pos_[YAW], &head_vel_[YAW], &head_eff_[YAW]);
    	hardware_interface::JointStateHandle state_handle_head_roll("head_roll", &head_pos_[ROLL], &head_vel_[ROLL], &head_eff_[ROLL]);
	jnt_state_interface.registerHandle(state_handle_head_yaw);
    	jnt_state_interface.registerHandle(state_handle_head_roll);
    	
	hardware_interface::JointHandle pos_handle_head_roll(jnt_state_interface.getHandle("head_roll"), &head_cmd_[ROLL]);
    	hardware_interface::JointHandle pos_handle_head_yaw(jnt_state_interface.getHandle("head_yaw"), &head_cmd_[YAW]);
    	jnt_pos_interface.registerHandle(pos_handle_head_roll);
    	jnt_pos_interface.registerHandle(pos_handle_head_yaw);
    	
    	registerInterface(&jnt_state_interface);
    	registerInterface(&jnt_pos_interface);
	return true;
}
void HEADHW::read(const ros::Time& time,const ros::Duration& period)
{
}

void HEADHW::write(const ros::Time& time,const ros::Duration& period)
{
  int head_step_cmd_[4];

  head_step_cmd_[ROLL] =(int) (PI_step * head_cmd_[ROLL]/PI);
  head_step_cmd_[YAW] =(int) (PI_step * head_cmd_[YAW]/PI);

  if(stepcnt[ROLL]<head_step_cmd_[ROLL]){
  	HEADHW::cwstep(ROLL);
	stepcnt[ROLL]++;
  }else if(stepcnt[ROLL]>head_step_cmd_[ROLL]){
 	HEADHW::ccwstep(ROLL);
	stepcnt[ROLL]--;
  }else{
	  //////motor_release();
  }
  if(stepcnt[YAW]<head_step_cmd_[YAW]){
 	HEADHW::ccwstep(YAW);
	stepcnt[YAW]++;
  }else if(stepcnt[YAW]>head_step_cmd_[YAW]){
  	HEADHW::cwstep(YAW);
	stepcnt[YAW]--;
  }else{
	  //motor_release();
  }

  //printstep(ROLL);
  //printstep(YAW);

  head_pos_[ROLL] =(int) stepcnt[ROLL] * PI / PI_step;
  head_pos_[YAW] =(int) stepcnt[YAW] * PI / PI_step;

//ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  //printf("%lf,%lf,%d,%d ",head_pos_[ROLL],head_cmd_[ROLL],stepcnt[ROLL],head_step_cmd_[ROLL]);
 // printf("%lf,%lf,%d,%d\n",head_pos_[YAW],head_cmd_[YAW],stepcnt[YAW],head_step_cmd_[YAW]);
  
  //head_pos_[ROLL] = head_cmd_[ROLL];// + 0.01*(head_cmd_[ROLL] - head_pos_[ROLL]);
  //head_pos_[YAW] = head_cmd_[YAW];// + 0.01*(head_cmd_[YAW] - head_pos_[YAW]);
}
}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::HEADHW, hardware_interface::RobotHW)
