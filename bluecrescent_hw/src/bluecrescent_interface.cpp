#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager/controller_manager.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>


class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_a);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_b);

   registerInterface(&jnt_pos_interface);
  }

  void read(){
	  ROS_INFO_STREAM("Commands for joints: " << cmd[0] << ", " << cmd[1])
		  ;};

  void write(){}

  ros::Time get_time() const { return ros::Time::now(); }
  ros::Duration get_period() const { return ros::Duration(0.01); }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bluecrescent_hw_node");
  ros::NodeHandle nh;
  
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot,nh);

  ros::Rate rate(1.0 / robot.get_period().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
     robot.read();
     cm.update(robot.get_time(), robot.get_period());
     robot.write();

     rate.sleep();
  }
    spinner.stop();
    return 0;
}
