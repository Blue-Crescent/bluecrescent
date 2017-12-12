#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.h>
	
enum ROTATION {ROLL=1,PITCH=2,YAW=0};
enum ARM {SHOULDER=0,ELBOW,WRIST};
enum SIDE {LEFT=0,RIGHT};

namespace bluecrescent_control{
class HEADHW : public hardware_interface::RobotHW
{
public:
  HEADHW();
  virtual ~HEADHW(){};
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  virtual void motor_release();
  virtual void motor_lock(uint8_t num);
  virtual void cwstep(uint8_t num);
  virtual void ccwstep(uint8_t num);
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  virtual void write(const ros::Time& time, const ros::Duration& period);

protected:

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double head_cmd_[3];
  double head_pos_[3];
  double head_vel_[3];
  double head_eff_[3];
  int stepcnt[4]; 
  unsigned char drv8830_addr_M0[2];
  unsigned char drv8830_addr_M1[2];
  int fd_M0[2],fd_M1[2];


};
}
