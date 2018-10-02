//#define NO_WIRINGPI

#ifndef NO_WIRINGPI
#include <wiringPi.h>
#include <wiringPiI2C.h>
#endif

#include <ros/ros.h>
#include <ros/time.h>
#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>


int main(int argc, char *argv[])
{
  int fd;

  ros::init(argc, argv, "bluecrescent_control");

  // enabling ROS_DEBUG
  //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  //   ros::console::notifyLoggerLevelsChanged();
  //}

#ifndef NO_WIRINGPI
  fd = wiringPiI2CSetup(0x70);
  wiringPiI2CWriteReg8(fd,0x00,0x04);
#endif

  ros::AsyncSpinner spinner(100);//Hz
  spinner.start();

  ros::NodeHandle nh;
  combined_robot_hw::CombinedRobotHW hw;
  bool init_success = hw.init(nh, nh);
  printf("Initialize Status: %d \n",init_success);

  controller_manager::ControllerManager cm(&hw, nh);

  //ros::Duration period(0.02);
  ros::Duration period(0.01);
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
