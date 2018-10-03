#include <bluecrescent_control/HEADHW.h>


namespace bluecrescent_control{


HEADHW::HEADHW()
  {
	head_cmd_[HEAD_R]= 0;
	head_pos_[HEAD_R]= 0;
	head_vel_[HEAD_R]= 0;
	head_eff_[HEAD_R]= 0;
	head_cmd_[HEAD_Y]= 0;
	head_pos_[HEAD_Y]= 0;
	head_vel_[HEAD_Y]= 0;
	head_eff_[HEAD_Y]= 0;

	// HALFSTEP
	step[HEAD_R].A   = 0xC1;
	step[HEAD_R].nA  = (0x1C << 1);
	step[HEAD_R].B   = 0x70;
	step[HEAD_R].nB  = (0x07 << 1);
	step[HEAD_Y].A   = 0xC1;
	step[HEAD_Y].nA  = (0x1C << 1);
	step[HEAD_Y].B   = 0x70;
	step[HEAD_Y].nB  = (0x07 << 1);
	// FULL STEP
	//step[HEAD_R].A   = 0xCC;
	//step[HEAD_R].nA  = (0x33 << 1);
	//step[HEAD_R].B   = 0x66;
	//step[HEAD_R].nB  = 0x33;
	//step[HEAD_Y].A   = 0xCC;
	//step[HEAD_Y].nA  = (0x33 << 1);
	//step[HEAD_Y].B   = 0x66;
	//step[HEAD_Y].nB  = 0x33;

  	drv8830_addr[HEAD_Y][0] = 0x60;
  	drv8830_addr[HEAD_Y][1] = 0x61;
  	drv8830_addr[HEAD_R][0] = 0x62;
  	drv8830_addr[HEAD_R][1] = 0x63;

#ifndef NO_WIRINGPI
  	fd_mux[0] = wiringPiI2CSetup(0x70);
  	fd_mux[1] = wiringPiI2CSetup(0x72);
   	fd[HEAD_R][0] = wiringPiI2CSetup(drv8830_addr[HEAD_R][0]);
   	fd[HEAD_R][1] = wiringPiI2CSetup(drv8830_addr[HEAD_R][1]);
   	fd[HEAD_Y][0] = wiringPiI2CSetup(drv8830_addr[HEAD_Y][0]);
   	fd[HEAD_Y][1] = wiringPiI2CSetup(drv8830_addr[HEAD_Y][1]);
#endif

        stepcnt[HEAD_R] = 0;
        stepcnt[HEAD_Y] = 0;
        drv[HEAD_R] = 0;
        drv[HEAD_Y] = 0;
}


void HEADHW::motor_select(uint8_t drv_status){
  ROS_DEBUG("HEAD driver selected.\n");
  #ifndef NO_WIRINGPI
  if(drv_status == 0){
    wiringPiI2CWriteReg8(fd_mux[0], 0x0 ,0x04);
    wiringPiI2CWriteReg8(fd_mux[1], 0x0 ,0x00);
  }
  #endif
}
void HEADHW::motor_release(uint8_t joint){
#ifndef NO_WIRINGPI
  switch(joint){
    case HEAD_R:
     ROS_DEBUG("HEAD_R driver off\n");
	   wiringPiI2CWriteReg8(fd[HEAD_R][0],CONTROL,0x18);
	   wiringPiI2CWriteReg8(fd[HEAD_R][1],CONTROL,0x18);
     break;
    case HEAD_Y:
     ROS_DEBUG("HEAD_Y driver off\n");
	   wiringPiI2CWriteReg8(fd[HEAD_Y][0],CONTROL,0x18);
	   wiringPiI2CWriteReg8(fd[HEAD_Y][1],CONTROL,0x18);
     break;
    default:
     ROS_INFO("ALL HEAD drivers off\n");
	   wiringPiI2CWriteReg8(fd[HEAD_R][0],CONTROL,0x18);
	   wiringPiI2CWriteReg8(fd[HEAD_R][1],CONTROL,0x18);
	   wiringPiI2CWriteReg8(fd[HEAD_Y][0],CONTROL,0x18);
	   wiringPiI2CWriteReg8(fd[HEAD_Y][1],CONTROL,0x18);
   }
#endif
}
void HEADHW::motor_lock(uint8_t joint){
	ROS_DEBUG("HEAD driver lock\n");
#ifndef NO_WIRINGPI
	wiringPiI2CWriteReg8(fd[HEAD_R][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[HEAD_R][1],CONTROL,B(joint));
	wiringPiI2CWriteReg8(fd[HEAD_Y][0],CONTROL,A(joint));
	wiringPiI2CWriteReg8(fd[HEAD_Y][1],CONTROL,B(joint));
#endif
}
void HEADHW::cwstep(uint8_t joint){
	lrotate(step[joint].A);
	lrotate(step[joint].nA);
	lrotate(step[joint].B);
	lrotate(step[joint].nB);
#ifndef NO_WIRINGPI
  switch(joint){
    case HEAD_R:
	   wiringPiI2CWriteReg8(fd[HEAD_R][0],CONTROL,A(joint));
     wiringPiI2CWriteReg8(fd[HEAD_R][1],CONTROL,B(joint));
     break;
    case HEAD_Y:
     wiringPiI2CWriteReg8(fd[HEAD_Y][0],CONTROL,A(joint));
     wiringPiI2CWriteReg8(fd[HEAD_Y][1],CONTROL,B(joint));
     break;
   }
 #endif
}
void HEADHW::ccwstep(uint8_t joint){
  rrotate(step[joint].A);
  rrotate(step[joint].nA);
  rrotate(step[joint].B);
  rrotate(step[joint].nB);
  #ifndef NO_WIRINGPI
  switch(joint){
    case HEAD_R:
	   wiringPiI2CWriteReg8(fd[HEAD_R][0],CONTROL,A(joint));
     wiringPiI2CWriteReg8(fd[HEAD_R][1],CONTROL,B(joint));
     break;
    case HEAD_Y:
     wiringPiI2CWriteReg8(fd[HEAD_Y][0],CONTROL,A(joint));
     wiringPiI2CWriteReg8(fd[HEAD_Y][1],CONTROL,B(joint));
     break;
   }
  #endif
}

bool HEADHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  using namespace hardware_interface;
  ROS_INFO("This is HEADHW\n");

  //chatter_pub = robot_hw_nh.advertise<std_msgs::String>("chatter", 1000);

  hardware_interface::JointStateHandle state_handle_head_yaw("head_yaw", &head_pos_[HEAD_Y], &head_vel_[HEAD_Y], &head_eff_[HEAD_Y]);
  hardware_interface::JointStateHandle state_handle_head_roll("head_roll", &head_pos_[HEAD_R], &head_vel_[HEAD_R], &head_eff_[HEAD_R]);
  jnt_state_interface.registerHandle(state_handle_head_yaw);
  jnt_state_interface.registerHandle(state_handle_head_roll);

  hardware_interface::JointHandle pos_handle_head_roll(jnt_state_interface.getHandle("head_roll"), &head_cmd_[HEAD_R]);
  hardware_interface::JointHandle pos_handle_head_yaw(jnt_state_interface.getHandle("head_yaw"), &head_cmd_[HEAD_Y]);
  jnt_pos_interface.registerHandle(pos_handle_head_roll);
  jnt_pos_interface.registerHandle(pos_handle_head_yaw);

  registerInterface(&jnt_state_interface); registerInterface(&jnt_pos_interface);
  return true;
}
void HEADHW::read(const ros::Time& time,const ros::Duration& period)
{
  head_pos_[HEAD_R] =(double)STEP2RAD(stepcnt[HEAD_R]);
  head_pos_[HEAD_Y]  =(double)STEP2RAD(stepcnt[HEAD_Y]);
}

void HEADHW::write(const ros::Time& time,const ros::Duration& period)
{
  int head_step_cmd_[3]; // Destination angle [rad]

  head_step_cmd_[HEAD_R] =(int) RAD2STEP(head_cmd_[HEAD_R]);
  head_step_cmd_[HEAD_Y] =(int) RAD2STEP(head_cmd_[HEAD_Y]);

  motor_select(drv[HEAD_R]+drv[HEAD_Y]);

  if(reset_HOME){
     stepcnt[HEAD_R] = 0;
     stepcnt[HEAD_Y] = 0;
     reset_HOME = 0;
  }
  // HEAD_R
    if(stepcnt[HEAD_R]<head_step_cmd_[HEAD_R]){
      drv[HEAD_R] = 1;
      ccwstep(HEAD_R);
      stepcnt[HEAD_R]++;
    }else if(stepcnt[HEAD_R]>head_step_cmd_[HEAD_R]){
      drv[HEAD_R] = 1;
      cwstep(HEAD_R);
      stepcnt[HEAD_R]--;
    }else{
      if(drv[HEAD_R]==1) motor_release(HEAD_R);
      drv[HEAD_R] = 0;
    }
    ROS_DEBUG("HEAD_R:%d %.3lf %d\n",drv[HEAD_R],head_pos_[HEAD_R],stepcnt[HEAD_R]);

  // HEAD_Y
    if(stepcnt[HEAD_Y]<head_step_cmd_[HEAD_Y]){
      drv[HEAD_Y] = 1;
      ccwstep(HEAD_Y);
      stepcnt[HEAD_Y]++;
    }else if(stepcnt[HEAD_Y]>head_step_cmd_[HEAD_Y]){
      drv[HEAD_Y] = 1;
      cwstep(HEAD_Y);
      stepcnt[HEAD_Y]--;
    }else{
      if(drv[HEAD_Y]==1) motor_release(HEAD_Y);
      drv[HEAD_Y] = 0;
    }
    ROS_DEBUG("HEAD_Y:%d %.3lf %d\n",drv[HEAD_Y],head_pos_[HEAD_Y],stepcnt[HEAD_Y]);

  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  //ROS_DEBUG("H:%lf , %lf , %d , %d "  , RAD2DEG(head_pos_[HEAD_R]), RAD2DEG(head_cmd_[HEAD_R]) , stepcnt[HEAD_R] , head_step_cmd_[HEAD_R]);
  //ROS_DEBUG("H:%lf , %lf , %d , %d " , RAD2DEG(head_pos_[HEAD_Y]) , RAD2DEG(head_cmd_[HEAD_Y] ) , stepcnt[HEAD_Y]  , head_step_cmd_[HEAD_Y]);
}
HEADHW::~HEADHW()
{
  motor_select(1);
  motor_release(-1);
}

}
PLUGINLIB_EXPORT_CLASS( bluecrescent_control::HEADHW, hardware_interface::RobotHW)
