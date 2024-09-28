#ifndef RC_BOTTOM_INC_DRV_CAN_H_
#define RC_BOTTOM_INC_DRV_CAN_H_
#include "stm32f4xx_hal.h"

typedef enum
{
    CAN_REC_ID1 = 0X201,//ID为1
    CAN_REC_ID2 = 0X202,//ID为2
    CAN_REC_ID3 = 0X203,//ID为3
    CAN_REC_ID4 = 0X204,//ID为4
    CAN_SEND_ID = 0X200,//发送ID

    CAN_SINGLECHIP = 0x405,
    CAN_SendMessage_VAL = 0x406

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
    uint16_t offset_angle;
    uint8_t cnt;
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



typedef struct {
    uint32_t Angle;
    uint32_t Speed;
    uint16_t Torque;
    uint8_t Temperature_flag;
    uint8_t Temperature;
}ControlData;

typedef struct {
    float Angle;
    float Speed;
    float Torque;
    uint8_t Temperature_flag;
    uint8_t Temperature;
}FinalData;

#define Disable_Motor_ID 0x020 //1<<5
#define Able_Motor_ID    0x040 //2<<5
#define Control_Motor_ID 0x080 //4<<5

#define Disable_ID1 1 + Disable_Motor_ID
#define Disable_ID2 2 + Disable_Motor_ID
#define Disable_ID3 3 + Disable_Motor_ID
#define Disable_ID4 4 + Disable_Motor_ID
#define Disable_ID5 5 + Disable_Motor_ID
#define Disable_ID6 6 + Disable_Motor_ID

#define Able_ID1    1 + Able_Motor_ID
#define Able_ID2    2 + Able_Motor_ID
#define Able_ID3    3 + Able_Motor_ID
#define Able_ID4    4 + Able_Motor_ID
#define Able_ID5    5 + Able_Motor_ID
#define Able_ID6    6 + Able_Motor_ID

#define Control_ID1    1 + Control_Motor_ID
#define Control_ID2    2 + Control_Motor_ID
#define Control_ID3    3 + Control_Motor_ID
#define Control_ID4    4 + Control_Motor_ID
#define Control_ID5    5 + Control_Motor_ID
#define Control_ID6    6 + Control_Motor_ID

#define Disable_ID1_Receive  1 + Disable_Motor_ID + 0x10
#define Disable_ID2_Receive  2 + Disable_Motor_ID + 0x10
#define Disable_ID3_Receive  3 + Disable_Motor_ID + 0x10
#define Disable_ID4_Receive  4 + Disable_Motor_ID + 0x10
#define Disable_ID5_Receive  5 + Disable_Motor_ID + 0x10
#define Disable_ID6_Receive  6 + Disable_Motor_ID + 0x10

#define Able_ID1_Receive    1 + Able_Motor_ID + 0x10
#define Able_ID2_Receive    2 + Able_Motor_ID + 0x10
#define Able_ID3_Receive    3 + Able_Motor_ID + 0x10
#define Able_ID4_Receive    4 + Able_Motor_ID + 0x10
#define Able_ID5_Receive    5 + Able_Motor_ID + 0x10
#define Able_ID6_Receive    6 + Able_Motor_ID + 0x10

#define Control_ID1_Receive    1 + Control_Motor_ID + 0x10
#define Control_ID2_Receive    2 + Control_Motor_ID + 0x10
#define Control_ID3_Receive    3 + Control_Motor_ID + 0x10
#define Control_ID4_Receive    4 + Control_Motor_ID + 0x10
#define Control_ID5_Receive    5 + Control_Motor_ID + 0x10
#define Control_ID6_Receive    6 + Control_Motor_ID + 0x10

float Angle_Consecutive(float angle_now);
float Angle_Consecutive1(float angle_now1);
void CAN_SINGLECHIP_SendMessage(int16_t angle,int16_t pos_target,int16_t speed);

void CAN_All_Init(void);
void CAN_Filter_Init(CAN_HandleTypeDef* hcan);
int16_t CAN_GetMotorVelocity(int8_t Which_x);
uint16_t CAN_GetMotorAngel(int8_t Which_x);
//void CAN_SendCurrent(int16_t current);
void CAN_SendCurrent(int16_t current1,int16_t current2,int16_t current3,int16_t current4);

void CAN_CMD_MOTOR_ENABLE(uint32_t stdid);
void CAN_CMD_MOTOR_DISABLE(uint32_t stdid);
void CAN_SEND_DATA(uint16_t id, int16_t current);
float CAN_GetDeep_Motor(int8_t Which_x);
void CAN_CMD_MOTOR_CONTROL(float TargetAngle,float TargetSpeed,
                           float Kp,float Kd,float TargetTorque,float stdid);



#endif //RC_BOTTOM_INC_DRV_CAN_H_
