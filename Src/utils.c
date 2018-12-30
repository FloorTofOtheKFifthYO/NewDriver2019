#include "utils.h"
#include "usart.h"
#include <string.h>

#define MAXINTER 16000

int brush_control_flag = 0;

float position_now=0;
float position_last=0;
float speed_now=0;

float target_pwm=0;
float target_speed=0;
float target_position=0;

float wave_arg[4]={0};
int send_wave_flag=0;

int motor_type_flag=BRUSH;

float PID_release(PID_struct *PID,float actual_data,float target_data)
{    
    PID->SetData=target_data;
    PID->ActualData=actual_data;
    PID->err=PID->SetData-PID->ActualData;
    PID->integral=PID->err+PID->integral;
    if(PID->integral>=MAXINTER)
        PID->integral=MAXINTER;
    if(PID->integral<=-MAXINTER)
        PID->integral=-MAXINTER;
    PID->OutData=PID->Kp*PID->err+PID->Ki*PID->integral+PID->Kd*(PID->err-PID->err_last);
    PID->err_last=PID->err;
    return PID->OutData;  
}

char s[22]={'b','y',16,6};
void send_wave(float arg1,float arg2,float arg3,float arg4){
  HAL_UART_Receive_IT(&huart5,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
  s[2]=16;  //length
  s[3]=6;   //type
  s[20]='\r';
  s[21]='\n';
  memcpy(s+4,&arg1,sizeof(arg1));
  memcpy(s+8,&arg2,sizeof(arg1));
  memcpy(s+12,&arg3,sizeof(arg1));
  memcpy(s+16,&arg4,sizeof(arg1));
  HAL_UART_Transmit(&huart5,(uint8_t *)s, 22,1000);   
}