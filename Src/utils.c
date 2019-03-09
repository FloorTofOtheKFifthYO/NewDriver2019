#include "utils.h"
#include "usart.h"
#include <string.h>

Motor_Type Motor=BRUSHLESS;
Control_Mode_Struct Control_Mode=PWM_Mode;

PID_Struct Current_PID={0,0,0,0,0,5000,0,0.005};
PID_Struct Speed_PID={0,0,0,0,0,5000,0,0.005};
PID_Struct Position_PID={0,0,0,0,0,5000,0,0.005};

float Duty =0;
float Target_Speed=0;
float Now_Speed = 0;
float Target_Position=0;
float Now_Position=0;
float Target_Current = 0;
float Now_Current = 0;
float Now_Current_buffer = 0;
float Last_Position = 0;
float Target_PWM=0;

int send_wave_flag=0;

#define Limit(value,max)     if(value>max)value=max;else if(value<-max)value=-max
float PID_Release(PID_Struct *PID,float target,float now)
{    
  float err;
  float err_dt;
  float result;
  
  err=target-now; 
  err_dt=err-PID->last_err; 
  
  err_dt*=0.384f;
  err_dt+=PID->last_d*0.615f;   //µÍÍ¨ÂË²¨ 
  
  PID->last_err=err;
  
  PID->i+=err*PID->I_TIME;
  
  Limit(PID->i,PID->i_max);
  PID->last_d=err_dt;
  
  result = err * PID->KP  +   err_dt * PID->KD   +   PID->i * PID->KI;
  return result;
}


void reset_PID(PID_Struct * s){
  s->i=0;
  s->last_err=0;
  s->last_d=0;
}


void PID_init(){
  reset_PID(&Current_PID);
  reset_PID(&Speed_PID);
  reset_PID(&Position_PID);
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
  //HAL_UART_Transmit(&huart5,(uint8_t *)s, 22,1000);   
  HAL_UART_Transmit_IT(&huart5,(uint8_t *)s, 22);
}