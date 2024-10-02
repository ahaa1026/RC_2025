#include "DeepMotor.h"
#include "main.h"
#include "drv_can.h"
extern float theta;

extern int16_t x;
extern int16_t y;
void motor_task(void)
{

    CAN_SINGLECHIP_SEND(x,y);

    CAN_CMD_MOTOR_CONTROL(theta,0.0f,50.0f,5.0f,0.0f,Control_ID2);




}
