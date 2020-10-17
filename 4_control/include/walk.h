#ifndef _WALKCLASS
#define _WALKCLASS
#include "leg.h"
#include "walk_leg.h"
#include "pid.hpp"

//行走的具体实现
class WalkClass
{
public:
    void walkInit(LegClass &_legfr,LegClass &_legfl,LegClass &_legbr,LegClass &_legbl,float *_yaw_value);
    void prepWalk();
    void update(float _dt);
    void reset();    //复位
    void startAct(bool _start);  //运动或者暂停
    void setSpeed(float _v,float _w);
    void setTFloor(float _t);

    void set_walk_num(float _num){walk_num = _num;};
    void set_t_floor(float _t_floor){t_floor_set = _t_floor;};
    float get_t_floor(){return t_floor_set;};
    void set_l(float _l){l_set = _l;};
    void set_h(float _h){h_set = _h;};
    float get_l(){return  l_set;};
    float get_h(){return h_set;};
    float get_v(){return v_now;};
    float get_w(){return w_now;};
private:
    WalkLegClass walkFl,walkBl,walkFr,walkBr;
    int walk_num;                            //步数
    float prep_long;                        //准备阶段的时长

    float l_set;            //初始步长
    float t_floor_set;      //着地相时长
    float v_now,w_now;      //当前速度设定值   //v:m/s  w:rad/s(逆时针为正)
    float v_set,w_set;      //设置速度
    float v_rea,w_rea;      //实际值  算出来的
    float vacc,wacc;        //加速度     角加速度
    float V_Max_F,V_Max_B,W_Max;  //最大前进后退速度，最大角速度
    float vl,vr;            //左侧速度  右侧速度   设定值
    float vr_rea,vl_rea;    //实际值
    float te_factor;        //te改变引起速度变化的比例
    float setYaw;           //设定的Yaw值    会随角速度变化
    float h_set;            //步高
    PID pidYaw;
    bool isSetSpeedFinished;
    float yawIncrease;
    float *yaw_value;

    float T_F_Min;  //最小着地相时长
    float T_F_Max;  //最大着地相时长
    float MaxLToCalSpeed;  //用于计算周期的最大步长  m
    float car_wid; //车宽

    void isSpeedChange();
    void speedChange(float _dt);
};


#endif
