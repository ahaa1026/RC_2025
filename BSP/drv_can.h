#ifndef RC_BOTTOM_INC_DRV_CAN_H_
#define RC_BOTTOM_INC_DRV_CAN_H_
#include "stm32f4xx_hal.h"

typedef enum
{
    CAN_REC_ID1 = 0X201,//ID为1
    CAN_REC_ID2 = 0X202,//ID为2
    CAN_REC_ID3 = 0X203,//ID为3
    CAN_REC_ID4 = 0X204,//ID为4
    CAN_SEND_ID = 0X200//发送ID

    //这里设置ID
} CAN_Msg_enum;

typedef struct
{
    uint16_t MotorAngle;
    int16_t MotorSpeed;

    //不要用以下这个类型的
    //float MotorAngle;
    //float MotorSpeed;

    int16_t MotorTorque;
    uint8_t MotorTempture;
    uint8_t Null;
    uint16_t UpDateAngle;
    uint16_t UpDateSpeed;
    uint16_t UpDateTorque;
    uint16_t UpDateTempture;
    int16_t TargetCurrent;//目标速度
    uint16_t Connected;
    float angelAll;          //总角度
} MotorMsg;

typedef struct
{
    int64_t angelAll;          //总角度
    int16_t angel;             //角度
    int16_t speed;             //速度
    int16_t torque;            //扭矩（电流）
    int16_t temperature;       //温度
    int8_t null;              //空值
} Motor_TypeDef;


float Angle_Consecutive(float angle_now);
float Angle_Consecutive1(float angle_now1);

void CAN_All_Init(void);
void CAN_Filter_Init(CAN_HandleTypeDef* hcan);
int16_t CAN_GetMotorVelocity(int8_t Which_x);
uint16_t CAN_GetMotorAngel(int8_t Which_x);
//void CAN_SendCurrent(int16_t current);
void CAN_SendCurrent(int16_t current1,int16_t current2,int16_t current3,int16_t current4);
#endif //RC_BOTTOM_INC_DRV_CAN_H_
