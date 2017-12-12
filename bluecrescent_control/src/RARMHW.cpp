#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>
#include <angles/angles.h>
#include <bluecrescent_control/RARMHW.h>
#include <iostream> // for debug
#include <math.h>

#define QUIT_CHAR 0x03 // ctrl+c

#define lrotate(x) (x=(((0x80 & x) >> 0x7) | (x << 1)))
#define rrotate(x) (x=(((0x01 & x) << 0x7) | (x >> 1)))
#define A(num) (VSET | (step[num].nA & 0x2) | (step[num].A & 0x1)) 
#define B(num) (VSET | (step[num].nB & 0x2) | (step[num].B & 0x1))
#define printstep(num) printf("%x %x %x %x\r\n",0x1 & step[num].A,(0x2&step[num].nA)>>1,0x1&step[num].B,(0x2&step[num].nB)>>1)


#define PI 3.141592653589
//FULL
//#define PI_step 500
//HALF
#define PI_step 1000

// 8830 Register
//#define VSET (0x26 << 2)
#define VSET (0x30<< 2)
#define CONTROL 0x0
#define FAULT 0x1

namespace bluecrescent_control{

struct termios CookedTermIos;
struct termios RawTermIos;
struct timespec ts;

// DRV8830 drvreg Register for phase A,B
typedef struct 
{
 unsigned char CONTROL_A;
 unsigned char CONTROL_B;
 unsigned char FAULT_A;
 unsigned char FAULT_B;
}drv8830reg;

// motorstep step
typedef struct
{
 unsigned char A;
 unsigned char nA;
 unsigned char B;
 unsigned char nB;
}motorstep;

drv8830reg drvreg[4];

//FULL STEP
//motorstep step[4] = {
//	//M0
//	{0xcc,(0x33 << 1),0x66,0x33},
//	//M1
//	{0xcc,(0x33 << 1),0x66,0x33},
//	//M2
//	{0xcc,(0x33 << 1),0x66,0x33},
//	//M3
//	{0xCC,(0x33 << 1),0x66,0x33}
//};
//HALF STEP
motorstep step[4] = {
	//M0
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
	//M1
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
	//M2
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
	//M3
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)}
};

int stepcnt[4] = {0,0,0,0};
unsigned const char drv8830_addr_M0[2] = {0x60,0x61};
unsigned const char drv8830_addr_M1[2] = {0x62,0x63};
int fd_mux,fd_M0[2],fd_M1[2];


void RARMHW::motor_release(){
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,0x18);
}
void RARMHW::motor_lock(uint8_t num){
	printf("MOTOR LOCKED!\n");
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}
void RARMHW::cwstep(uint8_t num){
	lrotate(step[num].A);
	lrotate(step[num].nA);
	lrotate(step[num].B);
	lrotate(step[num].nB);
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}
void RARMHW::ccwstep(uint8_t num){
	rrotate(step[num].A);
	rrotate(step[num].nA);
	rrotate(step[num].B);
	rrotate(step[num].nB);
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}
// Replacement SIGINT handler
//void mySigIntHandler(int sig)
//{
//  motor_release();
//  printf("Motor Stopped!\n");
//  ROS_WARN("Shutdown request received. Motor Stopping. Driver Released!");
//}


bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_roll("arm_shoulder_right_roll", &bluecrescent_control::rarm_pos_[SHOULDER][ROLL], &bluecrescent_control::rarm_vel_[SHOULDER][ROLL], &bluecrescent_control::rarm_eff_[SHOULDER][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_roll(jnt_state_interface.getHandle("arm_shoulder_right_roll"), &bluecrescent_control::rarm_cmd_[SHOULDER][ROLL]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_roll);
  
  hardware_interface::JointStateHandle state_handle_arm_shoulder_right_pitch("arm_shoulder_right_pitch", &bluecrescent_control::rarm_pos_[SHOULDER][PITCH], &bluecrescent_control::rarm_vel_[SHOULDER][PITCH], &bluecrescent_control::rarm_eff_[SHOULDER][PITCH]);
  jnt_state_interface.registerHandle(state_handle_arm_shoulder_right_pitch);

  hardware_interface::JointHandle rarm_pos_handle_arm_shoulder_right_pitch(jnt_state_interface.getHandle("arm_shoulder_right_pitch"), &bluecrescent_control::rarm_cmd_[SHOULDER][PITCH]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_shoulder_right_pitch);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_yaw("arm_elbow_right_yaw", &bluecrescent_control::rarm_pos_[ELBOW][YAW], &bluecrescent_control::rarm_vel_[ELBOW][YAW], &bluecrescent_control::rarm_eff_[ELBOW][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_yaw(jnt_state_interface.getHandle("arm_elbow_right_yaw"), &bluecrescent_control::rarm_cmd_[ELBOW][YAW]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_yaw);
  
  hardware_interface::JointStateHandle state_handle_arm_elbow_right_roll("arm_elbow_right_roll", &bluecrescent_control::rarm_pos_[ELBOW][ROLL], &bluecrescent_control::rarm_vel_[ELBOW][ROLL], &bluecrescent_control::rarm_eff_[ELBOW][ROLL]);
  jnt_state_interface.registerHandle(state_handle_arm_elbow_right_roll);

  hardware_interface::JointHandle rarm_pos_handle_arm_elbow_right_roll(jnt_state_interface.getHandle("arm_elbow_right_roll"), &bluecrescent_control::rarm_cmd_[ELBOW][ROLL]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_elbow_right_roll);

  hardware_interface::JointStateHandle state_handle_arm_wrist_right_yaw("arm_wrist_right_yaw", &bluecrescent_control::rarm_pos_[WRIST][YAW], &bluecrescent_control::rarm_vel_[WRIST][YAW], &bluecrescent_control::rarm_eff_[WRIST][YAW]);
  jnt_state_interface.registerHandle(state_handle_arm_wrist_right_yaw);

  hardware_interface::JointHandle rarm_pos_handle_arm_wrist_right_yaw(jnt_state_interface.getHandle("arm_wrist_right_yaw"), &bluecrescent_control::rarm_cmd_[WRIST][YAW]);
  jnt_pos_interface.registerHandle(rarm_pos_handle_arm_wrist_right_yaw);
  
  
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);

  return true;
}
  
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
  
void RARMHW::read(ros::Time time, ros::Duration period)
{
}

void RARMHW::write(ros::Time time, ros::Duration period)
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
