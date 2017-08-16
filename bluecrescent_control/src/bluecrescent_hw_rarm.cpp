#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <bluecrescent_control/bluecrescent_hw_rarm.h>
#include <iostream> // for debug
#include <math.h>


namespace combined_robot_hw_tests{
void BlueCrescent_rarm::read(ros::Time time, ros::Duration period)
{
}

void BlueCrescent_rarm::write(ros::Time time, ros::Duration period)
{
 
  // Real Robot functionality coding here...
  // below code is simulating real robot delay.	
  //ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
}
}
