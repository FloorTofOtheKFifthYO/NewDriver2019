#include "can_func.h"
#include "can.h"
#include "gpio.h"
#include "usart.h"
#include "utils.h"
#include "motor.h"
#include <stdlib.h>



void can_pwm_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  //uprintf("ID=%x    ",pRxMsg->StdId);
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
    //uprintf("receive %x ",can_RX_data.ch[i]);
  }  
  //uprintf("\r\n\r\n");  
  //uprintf("receive %lf\r\n\r\n",can_RX_data.df);
  Control_Mode=PWM_Mode;
  if(Motor==BRUSH)
    Brush_PWM_Control(can_RX_data.df);
  else
    Set_Motor_Duty(can_RX_data.df);
}
void can_current_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  } 
  Control_Mode=Current_Mode;
  Target_Current=can_RX_data.df;
}
void can_speed_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  } 
  Control_Mode=Speed_Mode;
  Target_Speed=can_RX_data.df;
}
void can_speed_current_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  } 
  Control_Mode=Speed_Current_Mode;
  Target_Speed=can_RX_data.df;
}
void can_position_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  } 
  Control_Mode=Position_Mode;
  Target_Position=can_RX_data.df;
}
void can_position_current_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  } 
  Control_Mode=Position_Current_Mode;
  Target_Position=can_RX_data.df;
}
void can_position_speed_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  } 
  Control_Mode=Position_Speed_Mode;
  Target_Position=can_RX_data.df;
}
void can_position_speed_current_func(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
  } 
  Control_Mode=Position_Speed_Current_Mode;
  Target_Position=can_RX_data.df;
}