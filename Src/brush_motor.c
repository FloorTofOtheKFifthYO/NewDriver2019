#include "brush_motor.h"
#include "usart.h"

bool Brush_position_with_speed_flag = 0;//是否有速度环的位置环
int Brush_encoder_type=INCREMENT;//编码器种类 （增量式/绝对值式）

int Brush_motor_control_flag = SPEED_MODE;//有刷控制模式

Brush_Phase Brush_chl_AB={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}};
Brush_Phase Brush_chl_BC={.Brush_Phase_pos={B,C},.Brush_Phase_neg={C,B}};
Brush_Phase Brush_chl_AC={.Brush_Phase_pos={A,C},.Brush_Phase_neg={C,A}};
Brush_Phase Brush_chl={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}};//Brush_chl_AB;//有刷通道

PID_struct brush_speed_PID = {.err_last=0, .integral=0.0, .err=0, .Kp=0, .Ki=0, .Kd=0};//有刷速度环PID
PID_struct brush_position_PID = {.err_last=0, .integral=0.0, .err=0, .Kp=0, .Ki=0, .Kd=0};//有刷位置环PID

void Brush_pwm_control(float pwm){
    if(pwm>=0)
        Phase_Change(Brush_chl.Brush_Phase_pos,pwm);
    else
        Phase_Change(Brush_chl.Brush_Phase_neg,-pwm);
}
//有刷速度环
static void Brush_speed_control(float target_speed)//差分时间5ms
{
    if(Brush_encoder_type == INCREMENT)
    {
        speed_now = (float)Brush_encoder_speed_read();
    }
    else
    {
        position_now = Brush_encoder_position_read();
        speed_now = position_now - position_last;//5ms不超过半圈
        while(speed_now > ABSOLUTEMAXPOS/2)
            speed_now-=ABSOLUTEMAXPOS;
        while(speed_now < -ABSOLUTEMAXPOS/2)
            speed_now+=ABSOLUTEMAXPOS;
        position_last = position_now;
    }
    
    float calculate = PID_release(&brush_speed_PID, speed_now,target_speed);
    Brush_pwm_control(calculate);//pwm -95~95
    
    
}
//有刷位置环
static void Brush_position_control(float target_position)//差分时间5ms
{
    if(Brush_encoder_type == INCREMENT)
    {
        speed_now = Brush_encoder_speed_read();
        position_now +=speed_now;
    }
    else
    { 
        position_now = Brush_encoder_position_read();
        //TODO：过零处理  反转
        while (target_position-position_now>ABSOLUTEMAXPOS/2)
            target_position-=ABSOLUTEMAXPOS;
        while (target_position-position_now<-ABSOLUTEMAXPOS/2)
            target_position+=ABSOLUTEMAXPOS;       
    }
    float calculate = PID_release(&brush_position_PID,(float)position_now, target_position);
    if(Brush_position_with_speed_flag)
    {
        Brush_speed_control(calculate);
    }
    else
    {
        Brush_pwm_control(calculate);//pwm -95~95
    }
    
}
//外部调用
void Brush_motor_control()
{
    switch(Brush_motor_control_flag){
    case PWM_MODE:
        {
            Brush_pwm_control(target_pwm);
            break;
        }
    case SPEED_MODE:
        {
            //5ms到了
            if(brush_control_flag){
                Brush_speed_control(target_speed);//编码器种类
                brush_control_flag=0;
            }
            break;
        }
    case POSITION_MODE:
        {
            //5ms到了
            if(brush_control_flag){
                Brush_position_control(target_position);//编码器种类  是否速度环
                brush_control_flag=0;
            }
            break;
        }
    default:
        {
            uprintf("没有这个模式\r\n");
            break;
        }
    }
}