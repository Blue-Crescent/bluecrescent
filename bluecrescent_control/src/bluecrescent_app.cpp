#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>


int main(int argc, char *argv[])
{
  int fd,ID;

  ros::init(argc, argv, "bluecrescent_control");

  // enabling ROS_DEBUG
  //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  //   ros::console::notifyLoggerLevelsChanged();
  //}

  ID = 0x70;
  /* WHO AM I */
  fd = wiringPiI2CSetup(ID);
  printf("setup return : %d\n",fd);
  					        
  /* start senser */
  if((wiringPiI2CWriteReg8(fd,0x00,0x04))<0){
          printf("I2C write error");
  }

  ros::AsyncSpinner spinner(100);//Hz
  spinner.start();

  ros::NodeHandle nh;
  combined_robot_hw::CombinedRobotHW hw;
  bool init_success = hw.init(nh, nh);
  printf("Initialize: %d \n",init_success);
  
  controller_manager::ControllerManager cm(&hw, nh);

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

