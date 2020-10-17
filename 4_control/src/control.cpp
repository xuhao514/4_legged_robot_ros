#include "data_processing.h"
#include <wiringSerial.h>
#include <string>
#include "stdio.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include "walk_leg.h"
#include "walk.h"

std::string port="/dev/ttyUSB0";
int baud_rate =  115200;
bool is_open = false;
int fd;
DataProcess data_process;  

float set_value = 180;
bool dir =false;
void getData();
void sendData();
ros::Publisher get_value_pub;
std_msgs::Float32 dat32;
LegClass leg_fl,leg_fr,leg_bl,leg_br;
WalkLegClass walk_leg_fl,walk_leg_bl;
WalkClass walk;

struct SteerAngle
{
  float c1[4];
  float c4[4];
};
SteerAngle steer_ang;

struct YPR{
  float y,p,r;
};
YPR ypr_data;

struct ArduinoState
{
  float update_dt;
  float rec_msg_dt;
 // bool imu_connected;
};
ArduinoState arduino_state;

bool connected;  //是否连接

ros::Time time_rec,time_update;
ros::Time time_now,time_pre;
double rec_dt,update_dt;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tset1");
    ros::NodeHandle nh;
    nh.param<float>("set_ang", set_value, 2500);  //
    get_value_pub = nh.advertise<std_msgs::Float32>("get_value", 10);
    if((fd = serialOpen(port.c_str(),baud_rate)) < 0)
    {
      printf("can not open port %s\n",port.c_str());
      is_open = false;
      return 0;
    }
    printf("serial start ...\n");
    is_open = true;  
    ros::Rate loop_rate(120);
    leg_fl.legInit(); leg_fr.legInit(); leg_bl.legInit(); leg_br.legInit(); 
    // walk_leg_fl.init(leg_fl,11,85,50,30,1000,1,1500,0,12,1);
    // walk_leg_fl.move2InitPos(1000);
    // walk_leg_bl.init(leg_bl,11,85,50,30,1000,1,1500,1000,12,1);
    // walk_leg_bl.move2InitPos(1000);
    walk.walkInit(leg_fr,leg_fl,leg_br,leg_bl,&ypr_data.y);
    walk.prepWalk();
    time_now =  time_pre = ros::Time::now();
    while (ros::ok())
    {
      time_pre = time_now;
      time_now = ros::Time::now();
      update_dt = (time_now - time_pre).toSec() * 1000;
     // printf("update_dt:%f\n",update_dt);
      if(connected)
      {
        // walk_leg_fl.walkUpdate(update_dt);
        // walk_leg_bl.walkUpdate(update_dt);
        walk.update(update_dt);
      }
      sendData();
      getData();
      loop_rate.sleep();
      ros::spinOnce();
    }
    
    // sudo chmod 777 /dev/ttyUSB0

    return 0;
}

void sendData()
{
  int _len;
  char *arr;
  char _id = 1 ;
  steer_ang.c1[0] = leg_fr.get_c1();steer_ang.c1[1] = leg_fl.get_c1();
  steer_ang.c1[2] = leg_br.get_c1();steer_ang.c1[3] = leg_bl.get_c1();
  steer_ang.c4[0] = leg_fr.get_c4();steer_ang.c4[1] = leg_fl.get_c4();
  steer_ang.c4[2] = leg_br.get_c4();steer_ang.c4[3] = leg_bl.get_c4();
  arr=data_process.dataEncode<SteerAngle>(&steer_ang, _id , &_len);
  for(int i=0;i<_len;i++)
  {
    serialPutchar(fd,arr[i]);
    //printf("%x ",arr[i]);
  }
  // printf("(x,y) %f ,%f\n",leg_fl.get_x(),leg_fl.get_y());
  // printf("(c1,c4) %f ,%f\n",leg_fl.get_c1(),leg_fl.get_c4());
  //std::cout<<walk_leg1.walk_leg_state.to_init_pos << " " <<walk_leg1.walk_leg_state.in_delay <<" " <<walk_leg1.walk_leg_state.walking << "\n";


}
float get_time;
void getData()
{
    time_update = ros::Time::now();
    while(serialDataAvail(fd) >= 1)    //如果串口缓存中有数据
    {
      char  data = serialGetchar(fd);
      // printf("%d \n",data);
      if(data_process.getHeadMsg(data))
      {
        // printf("get_head\n");
        if(data_process.headId() == 11)
        {
          if(data_process.dataDecode<YPR>(data,&ypr_data) )
          {
           // printf("yaw:%f\n",ypr_data.y);
          }
        }
        else if(data_process.headId() == 12)
        {
          time_rec = ros::Time::now();
          if(data_process.dataDecode<ArduinoState>(data,&arduino_state) )
          {
           // printf("arduino_update_dt :  %f\n",arduino_state.update_dt);
            printf("arduino_rec_msg_dt: %f\n",arduino_state.rec_msg_dt);
          }
        }
        else
        {
          data_process.clearFlag();
        }
          
      } 
    }   
    rec_dt = (time_update - time_rec ).toSec();
    //printf("dt:%f\n",rec_dt);
    if(rec_dt > 0.030  )
    {
      connected = false;
    }
    else
      connected = true;
}