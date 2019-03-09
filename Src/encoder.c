#include "encoder.h"
#include "as5047p.h"
#include "utils.h"

Encoder_Type Encoder=INCREMENT;//�����޸ľ���ֵ��������������ʽ������


//����ʽ������
void Encoder_Get()
{
    int TNT=TIM4->CNT;
    int Now_Speed=TNT-32767;
    TIM4->CNT=32767;
    Now_Position+=Now_Speed*0.005;
}

//����ֵ������
//���㴦�������޸�
void AS5047p_Get()
{ 
    float Now_Position_i = as5047p_Get_Position();
    if(Motor != BRUSHLESS_NONSENSOR)
        Now_Position = Now_Position_i;
    Now_Speed = Now_Position_i - Last_Position;
    
    while(Now_Speed > AS5047PMAXPOS/2)
        Now_Speed-=AS5047PMAXPOS;
    while(Now_Speed < -AS5047PMAXPOS/2)
        Now_Speed+=AS5047PMAXPOS;
    /*
    while (Target_Position-Now_Position_i>AS5047PMAXPOS/2)
    Target_Position-=AS5047PMAXPOS;
    while (Target_Position-Now_Position_i<-AS5047PMAXPOS/2)
    Target_Position+=AS5047PMAXPOS;
    */
    Last_Position = Now_Position_i;
}

//��ȡ�ٶ�&λ��
void Get_SP()
{
    switch(Encoder)
    {
    case INCREMENT:
        Encoder_Get();
        break;
    case ABSOLUTE:
        AS5047p_Get();
        break;
    } 
}