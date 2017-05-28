#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <bluecrescent_control/bluecrescent_hw.h>
#include <iostream> // for debug
#include <math.h>

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

char buf[10];
int com_serial;
int failcount;
 

//////////
// Init I2Cdevice
//////////
int i2c_init(char filename[40], int addr)
    {
        int file;
 
        if ((file = open(filename,O_RDWR)) < 0)
        {
            printf("Failed to open the bus.");
            /* ERROR HANDLING; you can check errno to see what went wrong */
        com_serial=0;
            exit(1);
            }
 
    if (ioctl(file,I2C_SLAVE,addr) < 0)
        {
            printf("Failed to acquire bus access and/or talk to slave.\n");
            /* ERROR HANDLING; you can check errno to see what went wrong */
        com_serial=0;
            exit(1);
            }
    return file;
    }
 
 
//////////
// Set pointer address
//////////
void i2c_set_pointer(int address,int value,int file)
    {
    /*
    //printf("end: 0x%x%x\n", address,value);
    if (i2c_smbus_write_byte_data(file, address, value)<0)
        {
        fprintf(stderr, "Warning - write failed\n");
        } 
    */
    char buf[10];
    buf[0]=address;
    buf[1]=value;
 
    if (write(file, buf, 2) != 2)
        {
        fprintf(stderr, "Error setting pointer\n");
        com_serial=0;
        failcount++;
        } 
    else
        {
        //printf("w_0x%0*x\n", 2, buf[0]);
        //printf("w_0x%0*x\n", 2, buf[1]);
        com_serial=1;
        failcount=0;
        }
 
    }
 
//////////
// Read n bytes
//////////
 
char * i2c_read(int add1, int add2, int nbytes,int file)
    {
    int n;
 
    i2c_set_pointer(add1,add2,file);
 
    if (read(file, buf, nbytes) != nbytes)
        {
        fprintf(stderr, "Error reading %i bytes\n",nbytes);
        com_serial=0;
        failcount++;
        } 
    else
        {
        for (n=0;n<nbytes;n++)
            {
            //printf("r_0x%0*x\n", 2, buf[n]);
            }
        com_serial=1;
        failcount=0;
        return buf;
        }
    }
 
//////////
// Write n bytes
//////////
void i2c_write(int add1,int add2,int nbytes,char value[10],int file)
    {
    int n;
    unsigned char buf[10];
    buf[0] = add1;
    buf[1] = add2;
    if (nbytes>=1) buf[2] = value[0];
    if (nbytes>=2) buf[3] = value[1];
    if (nbytes>=3) buf[4] = value[2];
    if (nbytes>=4) buf[5] = value[3];
 
    if (write(file, buf, nbytes) != nbytes)
        {
        fprintf(stderr, "Error writing %i bytes\n",nbytes);
        com_serial=0;
        failcount++;
        } 
    else
        {
        for (n=0;n<(nbytes+2);n++)
            {
            //printf("w_0x%0*x\n", 2, buf[n]);
            }
        com_serial=1;
        failcount=0;
        }
    }

BlueCrescent::BlueCrescent()
{
  // connect and register the joint state interface

  // Head

  hardware_interface::JointStateHandle state_handle_head_roll("head_roll", &head_pos_[ROLL], &head_vel_[ROLL], &head_eff_[ROLL]);
  jnt_state_interface.registerHandle(state_handle_head_roll);

  hardware_interface::JointHandle pos_handle_head_roll(jnt_state_interface.getHandle("head_roll"), &head_cmd_[ROLL]);
  jnt_pos_interface.registerHandle(pos_handle_head_roll);
  
  hardware_interface::JointStateHandle state_handle_head_yaw("head_yaw", &head_pos_[YAW], &head_vel_[YAW], &head_eff_[YAW]);
  jnt_state_interface.registerHandle(state_handle_head_yaw);

  hardware_interface::JointHandle pos_handle_head_yaw(jnt_state_interface.getHandle("head_yaw"), &head_cmd_[YAW]);
  jnt_pos_interface.registerHandle(pos_handle_head_yaw);

  
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);

}

void BlueCrescent::read(ros::Time time, ros::Duration period)
{
}

void BlueCrescent::write(ros::Time time, ros::Duration period)
{
  char *buffer;
  char buf[10];
  int file;

  file=i2c_init("/dev/i2c-1",0x38); //dev,slavei2caddr 
  buf[0] = 0x41;
  buf[1] = 0xFF;
  i2c_write(0xE6,0x0E,2,buf,file); //this will write value 0x41FF to the address 0xE60E
	       
  buffer=(char *)i2c_read(0xE6,0x0E,4,file); //reading a 4 bytes value at the address 0xE60E
	         
  close(file);
 
  // Real Robot functionality coding here...
  // below code is simulating real robot delay.	
  //pos_[0] = pos_[0] + 0.01* (cmd_[0] - pos_[0]);
  head_pos_[ROLL] = head_cmd_[ROLL];// + 0.01*(head_cmd_[ROLL] - head_pos_[ROLL]);
  head_pos_[YAW] = head_cmd_[YAW];// + 0.01*(head_cmd_[YAW] - head_pos_[YAW]);
  //ROS_DEBUG_STREAM("Debug:" << pos_[0] << cmd_[0]);
  // Dump cmd_ from MoveIt!, current simulated real robot pos_.
  printf("%lf,%lf\n",head_pos_[ROLL],head_cmd_[ROLL]);
}
