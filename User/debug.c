#include "debug.h"
#include"stdint.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include "pid.h"
#include "init.h"
#include "string.h"
#include "stdlib.h"
#include "drv_can.h"
#define TX_BUF_SIZE 512
uint8_t send_buf[TX_BUF_SIZE];
//float index_vec=0;
extern Pid vec_pid;
extern Pid pos_pid;
extern int cnt_mood;

char debugRvAll[DEBUG_RV_MXSIZE]={0};//存放串口1直接接收数据
char debugRvData[DEBUG_RV_MXSIZE]={0};//存放要传入的数据
char debugRvData_vec_kp[DEBUG_RV_MXSIZE]={0};//kp
char debugRvData_vec_ki[DEBUG_RV_MXSIZE]={0};//ki
char debugRvData_pos_kp[DEBUG_RV_MXSIZE]={0};//kp
char debugRvData_pos_kd[DEBUG_RV_MXSIZE]={0};//kd


char *pEnd;
int start_flag = 0;//用vofa软件设计电机启动，start_flag为标志位


//串口发送
//重定向串口打印函数，在操作系统里打印的问题
//已解决
void usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);

    length = vsnprintf((char*)send_buf, TX_BUF_SIZE, (const char*)format, args);

    va_end(args);

    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)send_buf, length);
}



//串口接收
void Set_Target_UartInit()
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能串口一的空闲中断,用于串口接收
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)debugRvAll, DEBUG_RV_MXSIZE);//开启串口的DMA接收，debugRvAll存储串口接受的第一手数据
}

void Set_Target_UartIdleCallback(UART_HandleTypeDef *huart);
void Set_Target_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart1.Instance)//判断是否是串口1
    {
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))//判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);//清楚空闲中断标志，防止会一直不断进入中断
            Set_Target_UartIdleCallback(huart);//调用中断处理函数
        }
    }
}



void Set_Target_UartIdleCallback(UART_HandleTypeDef *huart)
{
    float index=0;
    float index_vec_kp=0;
    float index_vec_ki=0;
    float index_pos_kp=0;
    float index_pos_kd=0;
    HAL_UART_DMAStop(huart);//停止本次DMA传输

    //将串口收到的数据进行处理，新的数组存放要传入的数据
    memcpy(debugRvData ,&debugRvAll[0], 5);//debugRvData接收串口传来的指定位置
    memcpy(debugRvData_vec_kp ,&debugRvAll[6],4);//debugRvData_vec接收串口传来的目标速度
    memcpy(debugRvData_vec_ki ,&debugRvAll[11],4);//debugRvData_vec接收串口传来的目标速度
    memcpy(debugRvData_pos_kp ,&debugRvAll[16],4);//debugRvData_vec接收串口传来的目标速度
    memcpy(debugRvData_pos_kd ,&debugRvAll[21],4);//debugRvData_vec接收串口传来的目标速度
    //计算接收到的数据长度，接收到的数据长度等于数组的最大存储长度减去DMA空闲的数据区长度
    uint8_t data_length  = DEBUG_RV_MXSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

/*
    //电机启动，从我之前自己在有刷电机的考核题目那边写的移植过来的，在无刷电机这里不需要
    if(debugRvAll[1] == MAOHAO && debugRvAll[2] == START)
    {
        start_flag = 1;
    }
    else if(debugRvAll[1] == MAOHAO && debugRvAll[2] == STOP)
    {
        start_flag = 0;
    }
*/
    //pid位置环和速度环目标设置
        //if (debugRvAll[0]==PV_Target)
        //{

                //index=strtof(debugRvData,&debugRvAll[8]);
                index=strtof(debugRvData,&pEnd);//字符串转float类型，指定位置转为float类型（解决联合体的问题）
                index_vec_kp=strtof(debugRvData_vec_kp,&pEnd);//期望速度转为float类型（解决联合体的问题）
                index_vec_ki=strtof(debugRvData_vec_ki,&pEnd);//期望速度转为float类型（解决联合体的问题）
                index_pos_kp=strtof(debugRvData_pos_kp,&pEnd);//期望速度转为float类型（解决联合体的问题）
                index_pos_kd=strtof(debugRvData_pos_kd,&pEnd);//期望速度转为float类型（解决联合体的问题）




                //计算公式：ramp_target_step为斜坡函数的步长，ramp_target_step/360为转数，ramp_target_step/360/（ramp_target_time+1）/（pid——task的任务周期）为转速，单位为r/ms
                //{ramp_target_step/360/（ramp_target_time+1）/（pid——task的任务周期）}*1000*60转换单位，为rpm。
             //pos_pid.ramp_target_step=index_vec*360*2*5/(1000*60);
             //vec_pid.ramp_target_step=index_vec*360*2*5/(1000*60);


    vec_pid.paramater.kp=index_vec_kp;
    vec_pid.paramater.ki=index_vec_ki;
    pos_pid.paramater.kp=index_pos_kp;
    pos_pid.paramater.kd=index_pos_kd;
    setPidTargetwithRamp(&pos_pid,index);//设置pid的外环位置环目标
    //CAN_SINGLECHIP_SendVal(index);
        //}
    //测试串级pid部分期望速度的实现
    //usart_printf("%.2f,%.2f",index,index_vec);

    memset(debugRvAll,0,data_length); //清零接收缓冲区
    data_length = 0;
    HAL_UART_Receive_DMA(huart, (uint8_t*)debugRvAll, DEBUG_RV_MXSIZE);//循环中开启串口的DMA接收
}


