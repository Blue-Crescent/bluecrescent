#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <bluecrescent_control/bluecrescent_hw_rarm.h>
#include <iostream> // for debug
#include <math.h>


namespace bluecrescent_control{
void BlueCrescent_rarm::read(ros::Time time, ros::Duration period)
{
  printf("This is bluecrescnet_hw_rarm\n");
}

void BlueCrescent_rarm::write(ros::Time time, ros::Duration period)
{
 
  // Real Robot functionality coding here...
  // below code is simulating real robot delay.	
  //ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
}
}

PLUGINLIB_EXPORT_CLASS( bluecrescent_control::BlueCrescent_rarm, hardware_interface::RobotHW)
