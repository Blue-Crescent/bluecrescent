#include <ros/ros.h>
#include <ros/time.h>
#include <controller_manager/controller_manager.h>
#include <bluecrescent_control/bluecrescent_hw.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bluecrescent");
  ros::NodeHandle nh;

  BlueCrescent bluecrescent;
  controller_manager::ControllerManager cm(&bluecrescent, nh);

  ros::Rate rate(200.0 / bluecrescent.getPeriod().toSec());
  ros::AsyncSpinner spinner(10);
  spinner.start();

  while(ros::ok())
  {
    ros::Time now = bluecrescent.getTime();
    ros::Duration dt = bluecrescent.getPeriod();

    bluecrescent.read(now, dt);
    cm.update(now, dt);

    bluecrescent.write(now, dt);
    rate.sleep();
  }
  spinner.stop();

  return 0;
}

