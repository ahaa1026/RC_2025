#ifndef __PID_H
#define __PID_H
#include "stdint.h"

typedef struct
{
    float kp;
    float ki;
    float kd;

}Paramaterpid;

typedef struct
{
    float output_kp;
    float output_ki;
    float output_kd;
    float output_total;
    float output_last;
    float outputstep;
}Output;

typedef struct
{
    float target;
    float actural;
    Output output;
}Variables;

typedef struct
{
    float error_now;
    float errpr_last;
    float error_integal;
}Error;

typedef struct
{
    Paramaterpid paramater;
    Variables variables;
    Error error;
    float INTEGAL_MAX;
    float OUTPUT_MAX;
    float ramp_target;
    float ramp_count_time;
    float ramp_target_time;
    float ramp_target_step;
    uint8_t State_Normal_Ramp;

}Pid;


typedef enum
{
    Normal_state = 0,			//PID工作在正常状态
    Ramp_state = 1,				// PID工作在斜坡函数状态

    FollowDefault_Step=12,//坡函数步长
    FollowDefault_Time=1//时间
  } Pid_Ramp_State;


void PidInit(Pid*pid,float kp,float ki,float kd,const float integal_max,const float output_max,const float output_step,int state_normal_ramp);
void setPidTarget(Pid*pid,float target);
void setPidTargetwithRamp(Pid*pid,float target);
void Pid_Update_Gamp(Pid*pid,float actural);
float PidGet(Pid*pid,float actural,float dead_zone,float integal_start_error);

float limit_float(float input1,float max_input1);

#endif
