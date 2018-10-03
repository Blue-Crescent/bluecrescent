//#define NO_WIRINGPI

#ifndef NO_WIRINGPI
#include <wiringPi.h>
#include <wiringPiI2C.h>
#endif

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/package.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#define DURATION 0.01

// 8830 Register
#define VSET (0x26 << 2)
//#define VSET (0x30<< 2)
#define CONTROL 0x0
#define FAULT 0x1

#define PI 3.141592653589
//FULL
#define PI_step 500
//HALF
//#define PI_step 1000

// Avairable joint
// HEAD ROLL YAW
// SHOULDER ROLL PITCH
// ELBOW ROLL YAW
// WRIST YAW
#define lrotate(x) (x=(((0x80 & x) >> 0x7) | (x << 1)))
#define rrotate(x) (x=(((0x01 & x) << 0x7) | (x >> 1)))
#define A(joint) (VSET | (step[joint].nA & 0x2) | (step[joint].A & 0x1))
#define B(joint) (VSET | (step[joint].nB & 0x2) | (step[joint].B & 0x1))
#define printstep(joint) printf("%x %x %x %x\r\n",0x1 & step[joint].A,(0x2&step[joint].nA)>>1,0x1&step[joint].B,(0x2&step[joint].nB)>>1)

#define RAD2DEG(rad) (180*rad/PI)
#define DEG2RAD(deg) (PI*deg/180)
#define STEP2DEG(step) (180*step/PI_step)
#define DEG2STEP(deg) (PI_step*deg/180)
#define STEP2RAD(step) (PI*step/PI_step)
#define RAD2STEP(rad) (PI_step*rad/PI)

enum NECK {HEAD_Y=0,HEAD_R};
enum ARM {SHOULDER_R,SHOULDER_P,ELBOW_Y,ELBOW_R,WRIST_Y};
//enum ROTATION {YAW=0,ROLL=1,PITCH=2};
//enum ARM {SHOULDER=0,ELBOW,WRIST};
//enum NECK {HEAD=0};
enum SIDE {LEFT=0,RIGHT};

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

static int reset_HOME;

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
