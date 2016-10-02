#include "ros/ros.h"
#include "clothoid_msgs/clothoid_CAN.h"
#include <boost/thread.hpp>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <fstream>
#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*
/* A simple SocketCAN example */
using namespace std;
const char *byte_to_binary(int x)
{
  static char b[9];
  b[0] = '\0';

  int z;
  for (z = 128; z > 0; z >>= 1)
  {
    strcat(b, ((x & z) == z) ? "1" : "0");
  }

  return b;
}

class CANNode{
  ros::NodeHandle nh;
  ros::Publisher can_pub;

  int soc;
  int read_can_port;
  int nbytes;
  uint8_t recv_id;
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  ofstream fout;

public:

  CANNode(ros::NodeHandle _nh):nh(_nh),fout("acc.txt"){
    recv_id =0;
    can_pub = nh.advertise<clothoid_msgs::clothoid_CAN>("vehicle_status", 1000);

    struct ifreq ifr;
    struct sockaddr_can addr;
    read_can_port = 1;
    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0){
      read_can_port = 0;
      return;
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, "can0");

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0){
      read_can_port = 0;
      return;
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0){
      read_can_port = 0;
      return;
    }
    // boost::thread grab_thread = boost::thread(boost::bind(&CANNode::publishCAN, this));
  }
  ~CANNode(){close(soc);}



  void publishCAN()
  {
    clothoid_msgs::clothoid_CAN can_msg;
    struct can_frame frame_rd, frame_wt;
    int recvbytes = 0, wrtbytes = 0;

    while(read_can_port && ros::ok()){
      struct timeval timeout = {1, 0};
      fd_set readSet;
      FD_ZERO(&readSet);
      FD_SET(soc, &readSet);

      if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0){
        if (!read_can_port){
          break;
        }
        if (FD_ISSET(soc, &readSet)){
          recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
          if(recvbytes)
          {
            //printf("ID = %x, dlc = %d, data = %s\n", frame_rd.can_id, frame_rd.can_dlc, byte_to_binary(frame_rd.data[1]));
            switch( frame_rd.can_id ){
            case 0x100:
              can_msg.Gway_Wheel_Velocity_FR = ((frame_rd.data[1]<<8)|frame_rd.data[0])*0.03125;
              can_msg.Gway_Wheel_Velocity_RL = ((frame_rd.data[3]<<8)|frame_rd.data[2])*0.03125;
              can_msg.Gway_Wheel_Velocity_RR = ((frame_rd.data[5]<<8)|frame_rd.data[4])*0.03125;
              can_msg.Gway_Wheel_Velocity_FL = ((frame_rd.data[7]<<8)|frame_rd.data[6])*0.03125;
              can_msg.header.stamp = ros::Time::now();
              recv_id |=0x01;
              if((recv_id & 0x01)==0x01){
                ROS_WARN("CAN_ID 0x100 has already received in this period.");
              }
              break;
            case 0x101:
              can_msg.Gway_Lateral_Accel_Speed = ((frame_rd.data[1]<<8)|frame_rd.data[0])*0.01 - 10.23;
              can_msg.Gway_Parking_Brake_Active = frame_rd.data[2]&0x0F;
              can_msg.Gway_AirConditioner_On = (frame_rd.data[2]&0xF0)>>4;
              can_msg.Gway_Steering_Angle = ((int16_t)(frame_rd.data[4]<<8)|frame_rd.data[3])*0.1;
              can_msg.Gway_Steering_Speed = frame_rd.data[5]*4;
              can_msg.Gway_Steering_Tq = (((frame_rd.data[7]<<8)|frame_rd.data[6])-0x800)*0.01;
              can_msg.header.stamp = ros::Time::now();
              recv_id |=0x02;
              if((recv_id & 0x02)==0x02){
                ROS_WARN("CAN_ID 0x101 has already received in this period.");
              }
              break;
            case 0x102:
              can_msg.Gway_Accel_Pedal_Position = frame_rd.data[0]*0.3906;
              can_msg.Gway_Brake_Active = frame_rd.data[1]&0x0F;
              can_msg.Gway_BrakeMasterCylinder_Pressure = (((frame_rd.data[3]&0x0F)<<12)|(frame_rd.data[2]<<8)|(frame_rd.data[1]&0xF0)>>4)*0.1;
              can_msg.Gway_Engine_Speed = (((frame_rd.data[5]&0x0F)<<12)|(frame_rd.data[4]<<4)|(frame_rd.data[3]&0xF0)>>4)*0.25;
              can_msg.Gway_Gear_Target_Change = ((frame_rd.data[5]&0xF0)>>4);
              can_msg.Gway_GearSelDisp = (frame_rd.data[6]&0x0F);
              can_msg.Gway_Throttle_Position = ((((frame_rd.data[7]&0x0F)<<4)|((frame_rd.data[6]&0xF0)>>4))-0x20)*0.46948357;
              can_msg.header.stamp = ros::Time::now();
              recv_id |=0x04;
              if((recv_id & 0x04)==0x04){
                ROS_WARN("CAN_ID 0x102 has already received in this period.");
              }
              break;
            case 0x103:
              can_msg.Gway_Cluster_Odometer = ((frame_rd.data[2]<<16)|(frame_rd.data[1]<<8)|frame_rd.data[0])*0.1;
              can_msg.Gway_Longitudinal_Accel_Speed = ((frame_rd.data[4]<<8)|frame_rd.data[3])*0.01-10.23;
              can_msg.Gway_Vehicle_Speed_Engine = frame_rd.data[5];
              can_msg.Gway_Yaw_Rate_Sensor = ((frame_rd.data[7]<<8)|frame_rd.data[6])*0.01-40.95;
              can_msg.header.stamp = ros::Time::now();
              recv_id |=0x08;
              if((recv_id & 0x08)==0x08){
                ROS_WARN("CAN_ID 0x103 has already received in this period.");
              }
              break;
            default:
              break;
            }
            if((recv_id & 0x0F)==0x0F){
              fout << setprecision(16)<< can_msg.Gway_Accel_Pedal_Position << " " <<can_msg.Gway_Longitudinal_Accel_Speed <<endl;
              can_pub.publish(can_msg);
              recv_id = 0;

              static float u_steerPos[3] = {0,}, err_steerPos[3] = {0,};
              const float Kp_steerPos = 40, Ki_steerPos = 0, Kd_steerPos = 5;

              static float u_vehicleYaw[3] = {0,}, err_vehicleYaw[3] = {0,};
              const float Kp_vehicleYaw = 20, Ki_vehicleYaw = 0, Kd_vehicleYaw = 0;

              const float A0_vehicleYaw = Kp_vehicleYaw + Ki_vehicleYaw + Kd_vehicleYaw;
              const float A1_vehicleYaw = (-Kp_vehicleYaw ) - (2 * Kd_vehicleYaw );
              const float A2_vehicleYaw = Kd_vehicleYaw;
              double ref_yaw = 0, yaw_bias = 0.2;
              err_vehicleYaw[2] = ref_yaw - (can_msg.Gway_Yaw_Rate_Sensor - yaw_bias);
              u_vehicleYaw[2] = u_vehicleYaw[1] + A0_vehicleYaw * err_vehicleYaw[2] + A1_vehicleYaw * err_vehicleYaw[1] + A2_vehicleYaw * err_vehicleYaw[0];
              u_vehicleYaw[0] = u_vehicleYaw[1]; u_vehicleYaw[1] = u_vehicleYaw[2]; err_vehicleYaw[0] = err_vehicleYaw[1]; err_vehicleYaw[1] = err_vehicleYaw[2];


              const float A0_steerPos = Kp_steerPos + Ki_steerPos + Kd_steerPos;
              const float A1_steerPos = (-Kp_steerPos ) - (2 * Kd_steerPos );
              const float A2_steerPos = Kd_steerPos;

              err_steerPos[2] = u_vehicleYaw[2] - can_msg.Gway_Steering_Angle;
              u_steerPos[2] = u_steerPos[1] + A0_steerPos * err_steerPos[2] + A1_steerPos * err_steerPos[1] + A2_steerPos * err_steerPos[0];
              u_steerPos[0] = u_steerPos[1]; u_steerPos[1] = u_steerPos[2]; err_steerPos[0] = err_steerPos[1]; err_steerPos[1] = err_steerPos[2];

              if(u_steerPos[2] > 0x7FFF)
                u_steerPos[2] = 0x7FFF;
              else if(u_steerPos[2] < -0x7FFE)
                u_steerPos[2] = -0x7FFE;
              uint16_t duty_Steer = abs(u_steerPos[2]);
              if (u_steerPos[2] < 0) {
                duty_Steer &=0x7FFF;
              }
              else {
                duty_Steer |=0x8000;
              }


              double ref_speed = 20;

              static float u_vehicleSpeedAcc[3] = {0,}, u_vehicleSpeedBrk[3] = {0,};
              static float err_vehicleSpeed[3] = {0,};

              static float u_throttle[3] = {0,}, err_throttle[3] = {0,};
              static float u_break[3] = {0,}, err_break[3] = {0,};

              const float Kp_throttle = 800, Ki_throttle = 0.0, Kd_throttle = 1000.0;
              const float Kp_break = 60, Ki_break = 0.0, Kd_break = 15.0;

              const float Kp_vehicleSpeedAcc = 0.6, Ki_vehicleSpeedAcc = 0.0, Kd_vehicleSpeedAcc = 0;
              const float Kp_vehicleSpeedBrk = 20, Ki_vehicleSpeedBrk = 0.0, Kd_vehicleSpeedBrk = 10;

              uint16_t duty_Speed = 0;
              bool isAccelerated=false;
              bool reverse_rotate =false;
              double deadzoneSpeed = 0;

              err_vehicleSpeed[2] = ref_speed - (can_msg.Gway_Wheel_Velocity_RR+can_msg.Gway_Wheel_Velocity_RL)/2;

              if(abs(err_vehicleSpeed[2]) > deadzoneSpeed){
                /* Control Acc Pedal */
                if(err_vehicleSpeed[2] > 0){
                  const float A0_vehicleSpeedAcc = Kp_vehicleSpeedAcc + Ki_vehicleSpeedAcc + Kd_vehicleSpeedAcc;
                  const float A1_vehicleSpeedAcc = (-Kp_vehicleSpeedAcc ) - (2 * Kd_vehicleSpeedAcc );
                  const float A2_vehicleSpeedAcc = Kd_vehicleSpeedAcc;

                  u_vehicleSpeedAcc[2] = u_vehicleSpeedAcc[1] + A0_vehicleSpeedAcc * err_vehicleSpeed[2] + A1_vehicleSpeedAcc * err_vehicleSpeed[1] + A2_vehicleSpeedAcc * err_vehicleSpeed[0];
                  u_vehicleSpeedAcc[0] = u_vehicleSpeedAcc[1]; u_vehicleSpeedAcc[1] = u_vehicleSpeedAcc[2]; err_vehicleSpeed[0] = err_vehicleSpeed[1]; err_vehicleSpeed[1] = err_vehicleSpeed[2];

                  isAccelerated= true;
                  u_break[0] = 0; u_break[1] =0; u_break[2]=0;
                  err_break[0] = 0; err_break[1] =0; err_break[2]=0;

                  const float A0_throttle = Kp_throttle + Ki_throttle + Kd_throttle;
                  const float A1_throttle = (-Kp_throttle ) - (2 * Kd_throttle );
                  const float A2_throttle = Kd_throttle;

                  if(u_vehicleSpeedAcc[2] > 30)
                    u_vehicleSpeedAcc[2] = 30;
                  err_throttle[2] =  u_vehicleSpeedAcc[2]/*60*/ - can_msg.Gway_Accel_Pedal_Position;
                  u_throttle[2] = u_throttle[1] + A0_throttle * err_throttle[2] + A1_throttle * err_throttle[1] + A2_throttle * err_throttle[0];
                  u_throttle[0] = u_throttle[1]; u_throttle[1] = u_throttle[2]; err_throttle[0] = err_throttle[1]; err_throttle[1] = err_throttle[2];

                  if ( u_throttle[2] > 0) {
                    reverse_rotate = false;
                  }else{
                    reverse_rotate = true;
                  }
                  if(u_throttle[2] > 0x7FFF)
                    u_throttle[2] = 0x7FFF;
                  else if(u_throttle[2] < -0x7FFE)
                    u_throttle[2] = -0x7FFE;
                  duty_Speed = abs(u_throttle[2]);
                }
                /* Control Break Pedal */
                else{
                  const float A0_vehicleSpeedBrk = Kp_vehicleSpeedBrk + Ki_vehicleSpeedBrk + Kd_vehicleSpeedBrk;
                  const float A1_vehicleSpeedBrk = (-Kp_vehicleSpeedBrk ) - (2 * Kd_vehicleSpeedBrk );
                  const float A2_vehicleSpeedBrk = Kd_vehicleSpeedBrk;

                  u_vehicleSpeedBrk[2] = u_vehicleSpeedBrk[1] + A0_vehicleSpeedBrk * err_vehicleSpeed[2] + A1_vehicleSpeedBrk * err_vehicleSpeed[1] + A2_vehicleSpeedBrk * err_vehicleSpeed[0];
                  u_vehicleSpeedBrk[0] = u_vehicleSpeedBrk[1]; u_vehicleSpeedBrk[1] = u_vehicleSpeedBrk[2]; err_vehicleSpeed[0] = err_vehicleSpeed[1]; err_vehicleSpeed[1] = err_vehicleSpeed[2];

                  isAccelerated = false;
                  u_throttle[0] = 0; u_throttle[1] = 0; u_throttle[2]=0;
                  err_throttle[0] =0; err_throttle[1] = 0;err_throttle[2]=0;

                  const float A0_break = Kp_break + Ki_break + Kd_break;
                  const float A1_break = (-Kp_break ) - (2 * Kd_break );
                  const float A2_break = Kd_break;

                  //BrakeMasterCylinder_Pressure value
                  //To maintain the vehicle stopped:500(in not-inclined place)
                  //To perform ESTOP: 800
                  err_break[2] =  -u_vehicleSpeedBrk[2] - can_msg.Gway_BrakeMasterCylinder_Pressure;
                  u_break[2] = u_break[1] + A0_break * err_break[2] + A1_break * err_break[1] + A2_break * err_break[0];
                  u_break[0] = u_break[1]; u_break[1] = u_break[2]; err_break[0] = err_break[1]; err_break[1] = err_break[2];
                  if ( u_break[2] > 0) {
                    reverse_rotate = true;
                  }else{
                    reverse_rotate=false;
                  }

                  if(u_break[2] > 0x7FFF)
                    u_break[2] = 0x7FFF;
                  else if(u_break[2] < -0x7FFE)
                    u_break[2] = -0x7FFE;
                  duty_Speed = abs(u_break[2]);
                }
              }
              /* rest pedals when the error of vehicle speed is in deadzone */
              else{
                ;
              }
              //set 32th bit with direction
              if ( !reverse_rotate) {
                duty_Speed &=0x7FFF;
              }
              else {
                duty_Speed |=0x8000;
              }

              ROS_INFO("err_vehicleSpeed: %f, err_throttle:%f, err_break: %f, duty_Speed: %d", err_vehicleSpeed[2], err_throttle[2], err_break[2], duty_Speed&0x7FFF);
              //TODO:GEAR
              uint16_t duty_Gear=0;

              //send pwm command to pcan
              frame_wt.can_id=0x002;
              frame_wt.data[0] = (unsigned char)(duty_Steer & 0xFF);
              frame_wt.data[1] = (unsigned char)((duty_Steer & 0xFF00) >> 8);
              frame_wt.data[2] = (unsigned char)(duty_Speed & 0xFF);
              frame_wt.data[3] = (unsigned char)((duty_Speed & 0xFF00) >> 8);
              frame_wt.data[4] = (unsigned char)(duty_Gear & 0xFF);
              frame_wt.data[5] = (unsigned char)((duty_Gear & 0xFF00) >> 8);

              frame_wt.can_dlc = 8;

              wrtbytes = write(soc, &frame_wt, sizeof(struct can_frame));
            }
          }
        }
      }
    }
  }
};

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "gateway_can_ctrl_node");
  ros::NodeHandle nh;
  CANNode kn(nh);
  while(ros::ok()){
    kn.publishCAN();
    ros::spinOnce();
  }
  return 0;
}

