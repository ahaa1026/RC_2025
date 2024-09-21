#include "debug.h"
#include "pid.h"

#include "drv_can.h"

#include "usart.h"
//本文件是实现串级pid的以指定速度达到目标位置，实现了稳定启动和平稳停下

extern Pid pos_pid;
extern Pid vec_pid;

float angle;
int16_t speed;
extern int start_flag;
int cascade_printf=0;


void pid_task0(void)
{
    //start_flag=1;

    /*
     PidInit(&vec_pid,20,5,0,15000,25000,3000,Normal_state);//60.5
     setPidTarget(&pos_pid,CAN_GetMotorAngel(1)/8191.0f*360.0f);

     PidInit(&pos_pid,0.7,0.0,1,1000,400,3000,Normal_state);
*/

    angle=CAN_GetMotorAngel(1)*360.0f/8191.0f;
    angle=Angle_Consecutive(angle);//角度连续化

    speed=CAN_GetMotorVelocity(1);
    //setPidTarget(&pos_pid,360);//设置外环pid目标，后期可用串口接收

    Pid_Update_Gamp(&pos_pid,angle);
    pos_pid.variables.output.output_total=PidGet(&pos_pid,angle,0.1f,300.0);

    //setPidTarget(&vec_pid,pos_pid.variables.output.output_total);
    setPidTarget(&vec_pid,pos_pid.variables.output.output_total);

    Pid_Update_Gamp(&vec_pid,speed);//实现斜坡函数，单独封装
    vec_pid.variables.output.output_total=PidGet(&vec_pid,speed,0.1f,200);//实现内环速度pid

    CAN_SendCurrent(vec_pid.variables.output.output_total,0,0,0);
    //CAN_SendCurrent(200,0,0,0);

    if(cascade_printf<3)
    {   //pid任务是1ms，但是1msvofa打印一次四个数据会太快了，这里实现了3ms打印一次’
        //原理同串级pid那里的斜坡步长台调整的时间
        cascade_printf++;
    }
    else
    {
        cascade_printf=0;
        //usart_printf("%.2f,%.2f,%d\n",(float)angle,(float)pos_pid.variables.target,speed);
        CAN_SINGLECHIP_SendMessage((int)angle,(int)pos_pid.variables.target,speed);


    }

}