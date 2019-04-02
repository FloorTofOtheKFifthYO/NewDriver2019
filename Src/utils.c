#include "utils.h"
#include "usart.h"
#include <string.h>
#include "flash.h"

int ID = 0;
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
float Last_Position = 0;
float Target_PWM=0;
int Hall_CNT = 0;
float Now_Speed_Hall = 0;
int send_wave_flag=0;

float Motor_Duty_Set=0;

//int setup_once_flag =1;


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


