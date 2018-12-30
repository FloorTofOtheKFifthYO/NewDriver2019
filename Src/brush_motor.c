#include "brush_motor.h"
#include "usart.h"

bool Brush_position_with_speed_flag = 0;//�Ƿ����ٶȻ���λ�û�
int Brush_encoder_type=INCREMENT;//���������� ������ʽ/����ֵʽ��

int Brush_motor_control_flag = SPEED_MODE;//��ˢ����ģʽ

Brush_Phase Brush_chl_AB={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}};
Brush_Phase Brush_chl_BC={.Brush_Phase_pos={B,C},.Brush_Phase_neg={C,B}};
Brush_Phase Brush_chl_AC={.Brush_Phase_pos={A,C},.Brush_Phase_neg={C,A}};
Brush_Phase Brush_chl={.Brush_Phase_pos={A,B},.Brush_Phase_neg={B,A}};//Brush_chl_AB;//��ˢͨ��

PID_struct brush_speed_PID = {.err_last=0, .integral=0.0, .err=0, .Kp=0, .Ki=0, .Kd=0};//��ˢ�ٶȻ�PID
PID_struct brush_position_PID = {.err_last=0, .integral=0.0, .err=0, .Kp=0, .Ki=0, .Kd=0};//��ˢλ�û�PID

void Brush_pwm_control(float pwm){
    if(pwm>=0)
        Phase_Change(Brush_chl.Brush_Phase_pos,pwm);
    else
        Phase_Change(Brush_chl.Brush_Phase_neg,-pwm);
}
//��ˢ�ٶȻ�
static void Brush_speed_control(float target_speed)//���ʱ��5ms
{
    if(Brush_encoder_type == INCREMENT)
    {
        speed_now = (float)Brush_encoder_speed_read();
    }
    else
    {
        position_now = Brush_encoder_position_read();
        speed_now = position_now - position_last;//5ms��������Ȧ
        while(speed_now > ABSOLUTEMAXPOS/2)
            speed_now-=ABSOLUTEMAXPOS;
        while(speed_now < -ABSOLUTEMAXPOS/2)
            speed_now+=ABSOLUTEMAXPOS;
        position_last = position_now;
    }
    
    float calculate = PID_release(&brush_speed_PID, speed_now,target_speed);
    Brush_pwm_control(calculate);//pwm -95~95
    
    
}
//��ˢλ�û�
static void Brush_position_control(float target_position)//���ʱ��5ms
{
    if(Brush_encoder_type == INCREMENT)
    {
        speed_now = Brush_encoder_speed_read();
        position_now +=speed_now;
    }
    else
    { 
        position_now = Brush_encoder_position_read();
        //TODO�����㴦��  ��ת
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
//�ⲿ����
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
            //5ms����
            if(brush_control_flag){
                Brush_speed_control(target_speed);//����������
                brush_control_flag=0;
            }
            break;
        }
    case POSITION_MODE:
        {
            //5ms����
            if(brush_control_flag){
                Brush_position_control(target_position);//����������  �Ƿ��ٶȻ�
                brush_control_flag=0;
            }
            break;
        }
    default:
        {
            uprintf("û�����ģʽ\r\n");
            break;
        }
    }
}