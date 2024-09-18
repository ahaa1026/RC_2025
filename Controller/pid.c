#include "pid.h"
#include <stdio.h>

#include "drv_can.h"

//本个pid代码由我写的考核题目里移植过来，之后会在考核题目那个工程对我写的代码逻辑进行详细的注释
extern Pid pos_pid;
//限制limit函数
float limit_float(float input1,float max_input1)
{
    if((input1>=-max_input1)&&(input1<=max_input1))
    {
        return input1;
    }
    else if(input1>max_input1)
    {
        return max_input1;
    }
    else
    {
        return -max_input1;
    }
}



//初始化pid
void PidInit(Pid*pid,float kp,float ki,float kd,const float integal_max,const float output_max,const float output_step,int state_normal_ramp)
{
    pid->paramater.kp=kp;
    pid->paramater.ki=ki;
    pid->paramater.kd=kd;
    pid->INTEGAL_MAX=integal_max;
    pid->OUTPUT_MAX=output_max;
    pid->variables.output.outputstep=output_step;


    pid->variables.actural=0;
    pid->variables.output.output_kp=0;
    pid->variables.output.output_ki=0;
    pid->variables.output.output_kd=0;
    pid->variables.output.output_total=0;

    pid->error.error_now=0;
    pid->error.error_integal=0;
    pid->error.errpr_last=0;

    pid->ramp_count_time=0.0;
    pid->ramp_target=0.0;
    pid->ramp_target_step=0;
    pid->ramp_target_step=FollowDefault_Step;
    pid->ramp_target_time=FollowDefault_Time;

    pid->State_Normal_Ramp=state_normal_ramp;


}

//设定pid的目标
void setPidTarget(Pid*pid,float target)
{
    pid->variables.target=target;

}

void setPidTargetwithRamp(Pid*pid,float target)
{

    if (pid->ramp_target != target)
    {
        pid->ramp_target=target;
        pid->State_Normal_Ramp=Ramp_state;
    }
}


void Pid_Update_Gamp(Pid*pid,float actural)
{
    if(pid->State_Normal_Ramp==Ramp_state)
    {
        if(pid->ramp_count_time<pid->ramp_target_time)//实现斜坡的步长时间
        {
            ++pid->ramp_count_time;
            //原理说明：ramp_count_time初始化是0，则每（ramp_target+1）个pid任务的周期这么长一个时间会增加pid的目标，结合下面的每次增加目标前先将ramp_count_time置0实现
        }

        else
        {
            pid->ramp_count_time=0;
            if(pid->variables.target<pid->ramp_target)
            {
                pid->variables.target+=pid->ramp_target_step;
                if(pid->variables.target>=pid->ramp_target)
                {
                    pid->variables.target=pid->ramp_target;
                    pid->State_Normal_Ramp=Normal_state;
                }
            }
           else if(pid->variables.target>pid->ramp_target)
            {
                pid->variables.target-=pid->ramp_target_step;
                if(pid->variables.target<=pid->ramp_target)
                {
                    pid->variables.target=pid->ramp_target;
                    pid->State_Normal_Ramp=Normal_state;
                }
            }
            else
            {
                pid->State_Normal_Ramp=Normal_state;//退出斜坡函数的模式
            }
        }
    }


    pid->variables.actural=actural;
    pid->error.errpr_last=pid->error.error_now;//误差传递
    pid->error.error_now=pid->variables.target-pid->variables.actural;
}

//实现pid计算，返回值为pid计算结果
float PidGet(Pid*pid,float actural,float dead_zone,float integal_start_error)
{

   // if(pid->State_Normal_Ramp==Normal_state)
   // {

       /* if ((pid->error.error_now>=-dead_zone)&&(pid->error.error_now<=dead_zone))
            pid->error.error_now=0;

        */

        //计算线性pid输出
        pid->variables.output.output_kp=pid->paramater.kp*pid->error.error_now;


        //判断是否满足积分分离条件，满足时输出为p+i+d
        if((pid->error.error_now>-integal_start_error)&&(pid->error.error_now<integal_start_error))
        {//计算积分pid输出
            pid->error.error_integal+=pid->error.error_now;
            pid->variables.output.output_ki=pid->paramater.ki*pid->error.error_integal;
            pid->variables.output.output_ki=limit_float(pid->variables.output.output_ki,pid->INTEGAL_MAX);//对积分结果进行限幅

            //计算pid微分输出
            pid->variables.output.output_kd=pid->paramater.kd*(pid->error.error_now-pid->error.errpr_last);
            //计算pid输出
            pid->variables.output.output_total=pid->variables.output.output_kp+pid->variables.output.output_ki+pid->variables.output.output_kd;
            pid->variables.output.output_total=limit_float(pid->variables.output.output_total,pid->OUTPUT_MAX);
        }
        //不满足时输出为p+d
        else
        {
            pid->variables.output.output_total=pid->variables.output.output_kp+pid->variables.output.output_kd;
            pid->variables.output.output_total=limit_float(pid->variables.output.output_total,pid->OUTPUT_MAX);
        }

        if(pid->variables.output.output_total-pid->variables.output.output_last>pid->variables.output.outputstep)
        {
            pid->variables.output.output_total=pid->variables.output.output_last+pid->variables.output.outputstep;
        }

        if(pid->variables.output.output_total-pid->variables.output.output_last<-pid->variables.output.outputstep)
        {
            pid->variables.output.output_total=pid->variables.output.output_last-pid->variables.output.outputstep;
        }

        pid->variables.output.output_last=pid->variables.output.output_total;


        //返回pid计算结果

        return pid->variables.output.output_total;
   // }
}
