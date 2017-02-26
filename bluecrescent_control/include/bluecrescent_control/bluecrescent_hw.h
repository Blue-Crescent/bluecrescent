#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>

enum ROTATION {ROLL=0,PITCH,YAW};
enum ARM {SHOULDER=0,ELBOW,WRIST};
enum SIDE {LEFT=0,RIGHT};


class BlueCrescent : public hardware_interface::RobotHW
{
public:
  BlueCrescent();

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read(ros::Time, ros::Duration);

  void write(ros::Time, ros::Duration);

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  double head_cmd_[3];
  double head_pos_[3];
  double head_vel_[3];
  double head_eff_[3];

  double arm_cmd_[3][2][3];
  double arm_pos_[3][2][3];
  double arm_vel_[3][2][3];
  double arm_eff_[3][2][3];

};

