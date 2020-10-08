#include "data_processing.h"
#include <wiringSerial.h>
#include <string>
#include "stdio.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include "walk_leg.h"

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

float value;

ros::Time time_now,time_pre;
double dt;

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
    ros::Rate loop_rate(100);
    leg_fl.legInit(); leg_fr.legInit(); leg_bl.legInit(); leg_br.legInit(); 
    walk_leg_fl.init(leg_fl,11,85,50,30,1000,1,1500,0,12,1);
    walk_leg_fl.move2InitPos(1000);
    walk_leg_bl.init(leg_bl,11,85,50,30,1000,1,1500,1000,12,1);
    walk_leg_bl.move2InitPos(1000);
    while (ros::ok())
    {
      walk_leg_fl.walkUpdate(10);
      walk_leg_bl.walkUpdate(10);
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
  printf("(x,y) %f ,%f\n",leg_fl.get_x(),leg_fl.get_y());
  printf("(c1,c4) %f ,%f\n",leg_fl.get_c1(),leg_fl.get_c4());
  //std::cout<<walk_leg1.walk_leg_state.to_init_pos << " " <<walk_leg1.walk_leg_state.in_delay <<" " <<walk_leg1.walk_leg_state.walking << "\n";
}
float get_time;
void getData()
{
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
                  printf("yaw:%f\n",ypr_data.y);
                }
              }
              else if(data_process.headId() == 12)
              {
                if(data_process.dataDecode<float>(data,&get_time) )
                {
                  printf("get_time:%f\n",get_time);
                }
              }
              else
              {
                data_process.clearFlag();
              }
                
            } 
        
        
      }       
}