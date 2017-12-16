#define NO_WIRINGPI
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
//#include <map>
//#include <string>
//#include <vector>
//#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
//#include <string.h>
//#include <errno.h>
//#include <signal.h>
//#include <time.h>
//#include <sys/time.h>
//#include <unistd.h>
//#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>
#include <angles/angles.h>
#include <iostream> // for debug
#include <math.h>
#include <pluginlib/class_list_macros.h>

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

enum ROTATION {ROLL=1,PITCH=2,YAW=0};
enum ARM {SHOULDER=0,ELBOW,WRIST};
enum SIDE {LEFT=0,RIGHT};

namespace bluecrescent_control{
class LARMHW : public hardware_interface::RobotHW
{
public:
  LARMHW();
  virtual ~LARMHW(){};
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  virtual void motor_release();
  virtual void motor_lock(uint8_t num);
  virtual void cwstep(uint8_t num);
  virtual void ccwstep(uint8_t num);
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(ros::Time& time, ros::Duration& period);
  virtual void write(ros::Time& time, ros::Duration& period);

protected:

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double larm_cmd_[3][3];
  double larm_pos_[3][3];
  double larm_vel_[3][3];
  double larm_eff_[3][3];
  int stepcnt[4];
  unsigned char drv8830_addr_M0[2];
  unsigned char drv8830_addr_M1[2];
  int fd_M0[2],fd_M1[2];
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

};

}
