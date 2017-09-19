#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <bluecrescent_control/bluecrescent_hw_rarm.h>
#include <iostream> // for debug
#include <math.h>


namespace bluecrescent_control{

bool BlueCrescent_rarm::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh){
	
	return true;
}

void BlueCrescent_rarm::read(const ros::Time& time, const ros::Duration& period)
{
  printf("This is bluecrescnet_hw_rarm\n");
}

void BlueCrescent_rarm::write(const ros::Time& time, const ros::Duration& period)
{
  printf("This is bluecrescnet_hw_rarm\n");
 
  // Real Robot functionality coding here...
  // below code is simulating real robot delay.	
  //ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
}
}

PLUGINLIB_EXPORT_CLASS( bluecrescent_control::BlueCrescent_rarm, hardware_interface::RobotHW)
