
#include "COMMONHW.h"

namespace bluecrescent_control{

class HEADHW : public hardware_interface::RobotHW
{
public:
  HEADHW();
  ~HEADHW();
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(DURATION); }
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time,const ros::Duration& period);
  virtual void write(const ros::Time& time,const ros::Duration& period);

  virtual void motor_release(uint8_t joint);
  virtual void motor_lock(uint8_t joint);
  virtual void cwstep(uint8_t joint);
  virtual void ccwstep(uint8_t joint);

protected:

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double head_cmd_[3];
  double head_pos_[3];
  double head_vel_[3];
  double head_eff_[3];
  int stepcnt[3];
  unsigned char drv8830_addr[3][2]; // [PLACE][RPY][8830_0,8830_1]
  int fd[3][2];
  int fd_mux[2];
  int drv[3];
  int switching;

drv8830reg drvreg[4];
motorstep step[4];



};
}
