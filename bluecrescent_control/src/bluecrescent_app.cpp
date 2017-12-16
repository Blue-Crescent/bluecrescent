#include <ros/ros.h>
#include <ros/time.h>
#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>

//#include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/joint_state_interface.h>
//#include <hardware_interface/robot_hw.h>
//#include <map>
//#include <string>
//#include <vector>
//#include <pluginlib/class_list_macros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bluecrescent_control");

  //BlueCrescent bluecrescent;

  ros::AsyncSpinner spinner(10);//Hz
  spinner.start();

  ros::NodeHandle nh;
  combined_robot_hw::CombinedRobotHW hw;
  bool init_success = hw.init(nh, nh);
  printf("Initialize: %d \n",init_success);
  
  controller_manager::ControllerManager cm(&hw, nh);

  ros::Duration period(0.1);
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

