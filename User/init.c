#include "drv_can.h"
#include "pid.h"
#include "init.h"
//#include "kalman.h"
//#include "sliding.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include"task.h"
#include"debug.h"
#include "drv_can.h"
int cnt_mood=5;
//extern Kalman kalman_pos_follow;
extern float angle;
extern  MotorMsg Motor1,Motor2,Motor3,Motor4;

Pid vec_pid;
Pid pos_pid;

/*
float expect_speed=200;
float expect_speed_division;
float expect_speed_rise;
float expect_speed_dec;
int cnt=1;
*/

void roboinit()
{
//6020串级pid参数

    //CAN_CMD_MOTOR_ENABLE(Able_ID1);
    PidInit(&vec_pid,35,0.015,0,15000,25000,3000,Normal_state);//60.5
    //setPidTargetwithRamp(&pos_pid,CAN_GetMotorAngel(1)/8191.0f*360.0f);
    //angle = 0;
    PidInit(&pos_pid,0.250,0.0000,0.000,1000,400,3000,Normal_state);

    //Motor1.cnt = 0;

/*
    PidInit(&vec_pid,20,0,0,15000,25000,3000,Normal_state);//60.5
    //setPidTarget(&pos_pid,CAN_GetMotorAngel(1)/8191.0f*360.0f);
    PidInit(&pos_pid,0.5,0.0,1,1000,400,3000,Normal_state);

*/
    //PidInit(&vec_pid,70,4,0,20000,25000,3000,Normal_state);//60.5

    //关键debug解决：
    //将此时编码器上电的角度值作为斜坡的起始目标
    //setPidTarget(&pos_pid,CAN_GetMotorAngel(1)/8191.0f*360.0f);


    //PidInit(&pos_pid,0.5,0.0,3,1000,400,3000,Normal_state);//原始还行的数据：0.7；1.0

    //sliding_init(0.9,9.8,9.8,0.5,25000,1);

    //KalmanInit(&kalman_pos_follow,1,9);//卡尔曼滤波器调参数

}
