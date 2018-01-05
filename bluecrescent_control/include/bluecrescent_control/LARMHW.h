//#define NO_WIRINGPI
//#include <ros/xmlrpc_manager.h>

#include "COMMONHW.h"

// SHOULDER ROLL PITCH
// ELBOW ROLL YAW
// WRIST YAW

namespace bluecrescent_control{
class LARMHW : public hardware_interface::RobotHW
{
public:
  LARMHW();
  virtual ~LARMHW(){};
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(DURATION); }

  virtual void motor_release();
  virtual void motor_lock(uint8_t arm,uint8_t joint);
  virtual void cwstep(uint8_t arm,uint8_t joint);
  virtual void ccwstep(uint8_t arm,uint8_t joint);
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  virtual void write(const ros::Time& time, const ros::Duration& period);

protected:

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double larm_cmd_[3][3];
  double larm_pos_[3][3];
  double larm_vel_[3][3];
  double larm_eff_[3][3];
  int stepcnt[3][3];
  unsigned char drv8830_addr[3][3][2]; // [PLACE][RPY][8830_0,8830_1]
  int fd[3][3][2];
	  
drv8830reg drvreg[3][3];
motorstep step[3][3];


};

}
