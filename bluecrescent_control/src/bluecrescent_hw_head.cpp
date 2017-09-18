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
#include <bluecrescent_control/bluecrescent_hw_head.h>
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

PLUGINLIB_EXPORT_CLASS( bluecrescent_control::BlueCrescent_head, hardware_interface::RobotHW)

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


void BlueCrescent_head::motor_release(){
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,0x18);
}
void BlueCrescent_head::motor_lock(uint8_t num){
	printf("MOTOR LOCKED!\n");
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}
void BlueCrescent_head::cwstep(uint8_t num){
	lrotate(step[num].A);
	lrotate(step[num].nA);
	lrotate(step[num].B);
	lrotate(step[num].nB);
	wiringPiI2CWriteReg8(fd_M0[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M0[1],CONTROL,B(num));
	wiringPiI2CWriteReg8(fd_M1[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd_M1[1],CONTROL,B(num));
}
void BlueCrescent_head::ccwstep(uint8_t num){
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


void BlueCrescent_head::read(ros::Time time, ros::Duration period)
{
}

void BlueCrescent_head::write(ros::Time time, ros::Duration period)
{
  // Real Robot functionality coding here...
  // below code is simulating real robot delay.	

  int head_step_cmd_[4];

  head_step_cmd_[ROLL] =(int) (PI_step * bluecrescent_control::head_cmd_[ROLL]/PI);
  head_step_cmd_[YAW] =(int) (PI_step * bluecrescent_control::head_cmd_[YAW]/PI);

  if(bluecrescent_control::stepcnt[ROLL]<head_step_cmd_[ROLL]){
  	cwstep(ROLL);
	bluecrescent_control::stepcnt[ROLL]++;
  }else if(bluecrescent_control::stepcnt[ROLL]>head_step_cmd_[ROLL]){
 	ccwstep(ROLL);
	bluecrescent_control::stepcnt[ROLL]--;
  }else{
	  //////motor_release();
  }
  if(bluecrescent_control::stepcnt[YAW]<head_step_cmd_[YAW]){
 	ccwstep(YAW);
	bluecrescent_control::stepcnt[YAW]++;
  }else if(bluecrescent_control::stepcnt[YAW]>head_step_cmd_[YAW]){
  	cwstep(YAW);
	bluecrescent_control::stepcnt[YAW]--;
  }else{
	  //motor_release();
  }

  printstep(ROLL);
  printstep(YAW);

  bluecrescent_control::head_pos_[ROLL] =(int) bluecrescent_control::stepcnt[ROLL] * PI / PI_step;
  bluecrescent_control::head_pos_[YAW] =(int) bluecrescent_control::stepcnt[YAW] * PI / PI_step;

//ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  printf("%lf,%lf,%d,%d ",bluecrescent_control::head_pos_[ROLL],bluecrescent_control::head_cmd_[ROLL],bluecrescent_control::stepcnt[ROLL],head_step_cmd_[ROLL]);
  printf("%lf,%lf,%d,%d\n",bluecrescent_control::head_pos_[YAW],bluecrescent_control::head_cmd_[YAW],bluecrescent_control::stepcnt[YAW],head_step_cmd_[YAW]);
  
  //bluecrescent_control::head_pos_[ROLL] = bluecrescent_control::head_cmd_[ROLL];// + 0.01*(bluecrescent_control::head_cmd_[ROLL] - bluecrescent_control::head_pos_[ROLL]);
  //bluecrescent_control::head_pos_[YAW] = bluecrescent_control::head_cmd_[YAW];// + 0.01*(bluecrescent_control::head_cmd_[YAW] - bluecrescent_control::head_pos_[YAW]);
}
}
