#include <ros/ros.h>
#include <ros/time.h>
#include <bluecrescent_ros_control/bluecrescent_hw.h>

#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bluecrescent");

  ros::NodeHandle nh;
  combined_robot_hw::CombinedRobotHW hw;
  controller_manager::ControllerManager cm(&hw, nh);

  ros::AsyncSpinner spinner(1);// Hz
  spinner.start();

  ros::Duration period(50.0); //Hz / hw.getPeriod().toSec());
  while(ros::ok())
  {
    hw.read(ros::Time::now(), period);
    cm.update(ros::Time::now(), period);

    hw.write(ros::Time::now(), period);
    period.sleep();
  }
  spinner.stop();

  return 0;
}

