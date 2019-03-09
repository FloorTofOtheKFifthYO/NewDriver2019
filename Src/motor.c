#include "motor.h"
#include "usart.h"

Brush_Phase Brush_chl_AB={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}};
Brush_Phase Brush_chl_BC={.Brush_Phase_pos={B,C},.Brush_Phase_neg={C,B}};
Brush_Phase Brush_chl_AC={.Brush_Phase_pos={A,C},.Brush_Phase_neg={C,A}};
Brush_Phase Brush_chl={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}};

void Brush_PWM_Control(float pwm){
  Duty=pwm;
  if(pwm>=0)
    Phase_Change(&Brush_chl.Brush_Phase_pos,pwm);
  else
    Phase_Change(&Brush_chl.Brush_Phase_neg,-pwm);
}

void PWM_Control(float duty){
    if(Motor==BRUSH)
        Brush_PWM_Control(duty);
    else
        Set_Motor_Duty(duty);
}

void Control()
{
  float current_out=0;
  float speed_out=0;
  float position_out=0;
  
  if(Control_Mode&Position_Mode)
  {
    position_out=PID_Release(&Position_PID,Target_Position,Now_Position);
    if(Control_Mode&Speed_Mode)
    {
      Target_Speed=position_out;
      speed_out=PID_Release(&Speed_PID,Target_Speed,Now_Speed);
      if(Control_Mode&Current_Mode)
      {//Position_Speed_Current 0x07
        Target_Current=speed_out;
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        PWM_Control(current_out);
      }
      else//Position_Speed 0x06
        PWM_Control(speed_out);
    }
    else
    {
      if(Control_Mode&Current_Mode)
      {//Position_Current 0x05
        Target_Current=position_out;
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        PWM_Control(current_out);
      }
      else//Position 0x04
        PWM_Control(position_out);  
    }
    
  }
  else{
    if(Control_Mode&Speed_Mode)
    {
      speed_out=PID_Release(&Speed_PID,Target_Speed,Now_Speed);
      if(Control_Mode&Current_Mode){//Speed_Current 0x03
        Target_Current=speed_out;
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        PWM_Control(current_out);
      }
      else//Speed 0x02
        PWM_Control(speed_out);
    }
    else
    {
      if(Control_Mode&Current_Mode){//Current 0x01
        current_out=PID_Release(&Current_PID,Target_Current,Now_Current);
        PWM_Control(current_out);
      }
      else//PWM 0x00
        ;
    }
  }
}

