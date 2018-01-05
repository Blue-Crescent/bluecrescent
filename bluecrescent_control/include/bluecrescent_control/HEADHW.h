
#include "COMMONHW.h"

namespace bluecrescent_control{

class HEADHW : public hardware_interface::RobotHW
{
public:
  HEADHW();
  virtual ~HEADHW(){};
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(DURATION); }
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time,const ros::Duration& period);
  virtual void write(const ros::Time& time,const ros::Duration& period);

  virtual void motor_release();
  virtual void motor_lock(uint8_t arm,uint8_t joint);
  virtual void cwstep(uint8_t arm,uint8_t joint);
  virtual void ccwstep(uint8_t arm,uint8_t joint);

protected:

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double head_cmd_[1][3];
  double head_pos_[1][3];
  double head_vel_[1][3];
  double head_eff_[1][3];
  int stepcnt[1][3]; 
  unsigned char drv8830_addr[1][3][2]; // [PLACE][RPY][8830_0,8830_1]
  int fd[1][3][2];

drv8830reg drvreg[1][4];
motorstep step[1][4];


};
}
