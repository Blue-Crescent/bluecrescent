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
#include <bluecrescent_control/LARMHW.h>
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
////HALF STEP
//motorstep step[4] = {
//	//M0
//	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
//	//M1
//	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
//	//M2
//	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
//	//M3
//	{0xC1,(0x1C << 1),0x70,(0x07 << 1)}
//};
motorstep step[4];



void LARMHW::motor_release(){
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,0x18);
}
void LARMHW::motor_lock(uint8_t num){
	printf("MOTOR LOCKED!\n");
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}
void LARMHW::cwstep(uint8_t num){
	lrotate(step[num].A);
	lrotate(step[num].nA);
	lrotate(step[num].B);
	lrotate(step[num].nB);
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}
void LARMHW::ccwstep(uint8_t num){
	rrotate(step[num].A);
	rrotate(step[num].nA);
	rrotate(step[num].B);
	rrotate(step[num].nB);
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}

LARMHW::LARMHW()
  {
  printf("This is bluecrescnet_hw_larm\n");
}
bool LARMHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	using namespace hardware_interface;
	
	stepcnt[0]=0; stepcnt[1]=0; stepcnt[2]=0; stepcnt[3]=0;
  	drv8830_addr_M0[0] = 0x60;
  	drv8830_addr_M0[1] = 0x61;
  	drv8830_addr_M1[0] = 0x62;
  	drv8830_addr_M1[1] = 0x63;

   	fd_M0[0] = wiringPiI2CSetup(drv8830_addr_M0[0]);
   	fd_M0[1] = wiringPiI2CSetup(drv8830_addr_M0[1]);
   	fd_M1[0] = wiringPiI2CSetup(drv8830_addr_M1[0]);
   	fd_M1[1] = wiringPiI2CSetup(drv8830_addr_M1[1]); 
	
	step[0].A={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[0].nA={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[0].B={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[0].nB={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	
	step[1].A={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[1].nA={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[1].B={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[1].nB={0xC1,(0x1C << 1),0x70,(0x07 << 1)};

	step[2].A={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[2].nA={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[2].B={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[2].nB={0xC1,(0x1C << 1),0x70,(0x07 << 1)};

	step[3].A={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[3].nA={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[3].B={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
	step[3].nB={0xC1,(0x1C << 1),0x70,(0x07 << 1)};
  
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

void LARMHW::read(ros::Time time, ros::Duration period)
{
}

void LARMHW::write(ros::Time time, ros::Duration period)
{
}
}
