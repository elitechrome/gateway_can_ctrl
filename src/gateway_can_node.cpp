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

#include <pid.h>

#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*

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
enum DrivingStatus{RUN, PAUSE, MANUAL};
/**
                   * @brief The CANNode class
                   * @details Grabs Gateway CAN data and publishes ROS Message. Also manages the control of all vehicle mechanisms.
                   * @author Jaemin Lee
                   * @date 2016-10-04
                   * @version 0.0.1
                   */
class CANNode{
  ros::NodeHandle nh;
  ros::Publisher can_pub;

  int soc;
  int read_can_port;
  int nbytes;
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  struct can_frame frame_rd, frame_wt;

  clothoid_msgs::clothoid_CAN can_msg;
  uint8_t gway_can_received_ID;
  DrivingStatus driving_status;

  bool isESTOPActivated;

  ofstream fout;
  //  u  =  -Kp * ( err + integral(err)/Ti + deriv(err)*Td )
  const float Kp_throttle = 800, Ti_throttle = FLT_MAX, Td_throttle = FLT_MAX;
  const float Kp_break = 60, Ti_break = FLT_MAX, Td_break = FLT_MAX;

  const float Kp_vehicleSpeedAcc = 0.6, Ti_vehicleSpeedAcc = FLT_MAX, Td_vehicleSpeedAcc = FLT_MAX;
  const float Kp_vehicleSpeedBrk = 20, Ti_vehicleSpeedBrk = FLT_MAX, Td_vehicleSpeedBrk = 10;

  PID pid_vehicleSpeedAcc;
  PID pid_throttle;

  PID pid_vehicleSpeedBrk;
  PID pid_break;


public:
  /**
   * @brief Constructor of CANNode class
   */
  CANNode(ros::NodeHandle _nh):nh(_nh),fout("acc.txt"),
    pid_vehicleSpeedAcc(Kp_vehicleSpeedAcc, Ti_vehicleSpeedAcc, Td_vehicleSpeedAcc, 0.01, 0, 50),//pedal pos : 0~50
    pid_throttle(Kp_throttle, Ti_throttle, Td_throttle, 0.01, -0x7FFF, 0x7FFF),//max dutyratio : 0x7FFF
    pid_vehicleSpeedBrk(Kp_vehicleSpeedBrk, Ti_vehicleSpeedBrk, Td_vehicleSpeedBrk, 0.01, -1000, 0),//cylinder press : 0~1000
    pid_break(Kp_break, Ti_break, Td_break, 0.01, -0x7FFF, 0x7FFF)//max dutyratio : 0x7FFF
  {
    gway_can_received_ID =0;
    isESTOPActivated = false;
    driving_status =MANUAL;
    can_msg.header.stamp = ros::Time::now();
    can_pub = nh.advertise<clothoid_msgs::clothoid_CAN>("vehicle_status", 1000);
    if(!initSocketCAN()){
      ROS_ERROR("CAN Initialization failed.");
      ros::shutdown();
      return;
    }


    //boost::thread grab_thread = boost::thread(boost::bind(&CANNode::grab_publishCAN, this));
  }

  /**
   * @brief Initialize SocketCAN
   */
  bool initSocketCAN(){
    struct ifreq ifr;
    struct sockaddr_can addr;

    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0){
      return false;
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, "can0");

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0){
      return false;
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0){
      return false;
    }
    return true;
  }
  /**
   * @brief Destructor of CANNode class
   */
  ~CANNode(){close(soc);}

  void motorCtrl()
  {
    uint16_t duty_Steer = 0;
    uint16_t duty_Speed = 0;
    uint16_t duty_Gear = 0;
    bool reverse_rotate =false;


    if(driving_status == MANUAL){
      ROS_WARN("Driving Status is in Manual mode.");
      duty_Steer = 0;
      duty_Speed = 0;
      duty_Gear = 0;
    }
    else if( (ros::Time::now() - can_msg.header.stamp) > ros::Duration(0.03) ){
      ROS_ERROR("Timeout occurred. Check CAN Connection.");
      duty_Steer = 0;
      duty_Speed = 0;
      duty_Gear = 0;
    }
    else{

      double ref_speed = 20;
      double deadzoneSpeed = 2;

      double u_throttle=0 ,u_break=0;
      static double last_break=0;
      float err_vehicleSpeed = ref_speed - (can_msg.Gway_Wheel_Velocity_FR+can_msg.Gway_Wheel_Velocity_FL)/2;

      if (driving_status == PAUSE){
        //STOP
        //BrakeMasterCylinder_Pressure value
        //To maintain the vehicle stopped:500(in not-inclined place)
        //To perform ESTOP: 800
        pid_vehicleSpeedAcc.reset();
        pid_throttle.reset();
        u_throttle =0;

        //when the vehicle is almost stopped
        if(err_vehicleSpeed < deadzoneSpeed){
          u_break = pid_break.calc(can_msg.Gway_BrakeMasterCylinder_Pressure, last_break);

        }
        else{
          u_break = pid_break.calc(can_msg.Gway_BrakeMasterCylinder_Pressure,
                                   -1.*pid_vehicleSpeedBrk.calc( (can_msg.Gway_Wheel_Velocity_FR+can_msg.Gway_Wheel_Velocity_FL)/2, ref_speed ));
          last_break = u_break;
        }

        if ( u_break > 0) {
          reverse_rotate = false;
        }else{
          reverse_rotate = true;
        }
        duty_Speed = abs(u_break);
      }
      else if (driving_status == RUN){
        if(abs(err_vehicleSpeed) > deadzoneSpeed){
          /* Control Acc Pedal */
          if(err_vehicleSpeed > 0){
            pid_vehicleSpeedBrk.reset();
            pid_break.reset();
            u_break = 0;

            u_throttle = pid_throttle.calc(can_msg.Gway_Accel_Pedal_Position,
                                           pid_vehicleSpeedAcc.calc( (can_msg.Gway_Wheel_Velocity_FR+can_msg.Gway_Wheel_Velocity_FL)/2, ref_speed ));

            if ( u_throttle > 0) {
              reverse_rotate = true;
            }else{
              reverse_rotate = false;
            }
            duty_Speed = abs(u_throttle);
          }
          /* Control Break Pedal */
          else{
            pid_vehicleSpeedAcc.reset();
            pid_throttle.reset();
            u_throttle =0;

            u_break = pid_break.calc(can_msg.Gway_BrakeMasterCylinder_Pressure,
                                     -1.*pid_vehicleSpeedBrk.calc( (can_msg.Gway_Wheel_Velocity_FR+can_msg.Gway_Wheel_Velocity_FL)/2, ref_speed ));

            if ( u_break > 0) {
              reverse_rotate = false;
            }else{
              reverse_rotate = true;
            }
            duty_Speed = abs(u_break);


          }
        }
        /* rest pedals when the error of vehicle speed is in deadzone */
        else{
          pid_vehicleSpeedAcc.reset();
          pid_throttle.reset();
          u_throttle =0;

          u_break = pid_break.calc(can_msg.Gway_BrakeMasterCylinder_Pressure, 5);

          if ( u_break > 0) {
            reverse_rotate = false;
          }else{
            reverse_rotate = true;
          }
          duty_Speed = abs(u_break);
        }
        //set 16th bit with direction
        if ( !reverse_rotate) {
          duty_Speed &=0x7FFF;
        }
        else {
          duty_Speed |=0x8000;
        }

        ROS_INFO("err_vehicleSpeed: %f, duty_Speed: %d", err_vehicleSpeed, duty_Speed&0x7FFF);
      }
    }

    //send pwm command by pcan
    frame_wt.can_id = 0x002;
    frame_wt.data[0] = (unsigned char)(duty_Steer & 0xFF);
    frame_wt.data[1] = (unsigned char)((duty_Steer & 0xFF00) >> 8);
    frame_wt.data[2] = (unsigned char)(duty_Speed & 0xFF);
    frame_wt.data[3] = (unsigned char)((duty_Speed & 0xFF00) >> 8);
    frame_wt.data[4] = (unsigned char)(duty_Gear & 0xFF);
    frame_wt.data[5] = (unsigned char)((duty_Gear & 0xFF00) >> 8);

    frame_wt.can_dlc = 8;

    int wrtbytes = 0;
    wrtbytes = write(soc, &frame_wt, sizeof(struct can_frame));


  }

  void grab_publishCAN()
  {
    struct can_frame frame_rd;
    int recvbytes = 0;

    struct timeval timeout = {1, 0};
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(soc, &readSet);

    if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0){
      if (FD_ISSET(soc, &readSet)){
        recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
        if(recvbytes)
        {
          switch( frame_rd.can_id ){
          case 0x001:
            if( (frame_rd.data[0] & 0x04)>>2 == 1 || (frame_rd.data[0] & 0x02)>>1 == 0 ){//PAUSE(3rd bit)
              isESTOPActivated = false;
              driving_status = PAUSE;
            }
            if( (frame_rd.data[0] & 0x01) == 1 ){//ESTOP(1st bit)
              isESTOPActivated = true;
              ROS_WARN("ESTOP Trigger is on.");
            }
            if( (frame_rd.data[1] & 0x01) == 0 ){//TIMEOUT(5th bit)
              ROS_WARN("Timeout Trigger is off. Check the remote has turned on.");
              driving_status = MANUAL;
            }
            else if( (frame_rd.data[1] & 0x01) == 1 ){
              if(isESTOPActivated){
                driving_status = PAUSE;
              }else{
                if( (frame_rd.data[0] & 0x02)>>1 == 1 || (frame_rd.data[0] & 0x04)>>2 == 0 ){//RUN(2nd bit)
                  driving_status = RUN;
                }
              }
            }

            if( (frame_rd.data[0] & 0x08)>>3 == 1 ){//DISABLE(4th bit)
              ROS_WARN("DISABLE Trigger is on.");
              driving_status = MANUAL;
            }

            break;
          case 0x100:
            can_msg.Gway_Wheel_Velocity_FR = ((frame_rd.data[1]<<8)|frame_rd.data[0])*0.03125;
            can_msg.Gway_Wheel_Velocity_RL = ((frame_rd.data[3]<<8)|frame_rd.data[2])*0.03125;
            can_msg.Gway_Wheel_Velocity_RR = ((frame_rd.data[5]<<8)|frame_rd.data[4])*0.03125;
            can_msg.Gway_Wheel_Velocity_FL = ((frame_rd.data[7]<<8)|frame_rd.data[6])*0.03125;
            can_msg.header.stamp = ros::Time::now();
            gway_can_received_ID |=0x01;
            if((gway_can_received_ID & 0x01)==0x01){
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
            gway_can_received_ID |=0x02;
            if((gway_can_received_ID & 0x02)==0x02){
              ROS_WARN("CAN_ID 0x101 has already received in this period.");
            }
            break;
          case 0x102:
            can_msg.Gway_Accel_Pedal_Position
                = frame_rd.data[0]*0.3906;
            can_msg.Gway_Brake_Active
                = frame_rd.data[1]&0x0F;
            can_msg.Gway_BrakeMasterCylinder_Pressure
                = (((frame_rd.data[3]&0x0F)<<12)|(frame_rd.data[2]<<8)|(frame_rd.data[1]&0xF0)>>4)*0.1;
            can_msg.Gway_Engine_Speed
                = (((frame_rd.data[5]&0x0F)<<12)|(frame_rd.data[4]<<4)|(frame_rd.data[3]&0xF0)>>4)*0.25;
            can_msg.Gway_Gear_Target_Change
                = ((frame_rd.data[5]&0xF0)>>4);
            can_msg.Gway_GearSelDisp
                = (frame_rd.data[6]&0x0F);
            can_msg.Gway_Throttle_Position
                = ((((frame_rd.data[7]&0x0F)<<4)|((frame_rd.data[6]&0xF0)>>4))-0x20)*0.46948357;
            can_msg.header.stamp
                = ros::Time::now();
            gway_can_received_ID |=0x04;
            if((gway_can_received_ID & 0x04)==0x04){
              ROS_WARN("CAN_ID 0x102 has already received in this period.");
            }
            break;
          case 0x103:
            can_msg.Gway_Cluster_Odometer = ((frame_rd.data[2]<<16)|(frame_rd.data[1]<<8)|frame_rd.data[0])*0.1;
            can_msg.Gway_Longitudinal_Accel_Speed = ((frame_rd.data[4]<<8)|frame_rd.data[3])*0.01-10.23;
            can_msg.Gway_Vehicle_Speed_Engine = frame_rd.data[5];
            can_msg.Gway_Yaw_Rate_Sensor = ((frame_rd.data[7]<<8)|frame_rd.data[6])*0.01-40.95;
            can_msg.header.stamp = ros::Time::now();
            gway_can_received_ID |=0x08;
            if((gway_can_received_ID & 0x08)==0x08){
              ROS_WARN("CAN_ID 0x103 has already received in this period.");
            }
            break;
          default:
            break;
          }

          if((gway_can_received_ID & 0x0F) == 0x0F){
            can_pub.publish(can_msg);
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
  ros::Rate r(400);
  while(ros::ok()){
    kn.grab_publishCAN();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

