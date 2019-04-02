#include "encoder.h"
#include "as5047p.h"
#include "utils.h"

Encoder_Type Encoder=INCREMENT;//�����޸ľ���ֵ��������������ʽ������


//����ʽ������
void Encoder_Get()
{
  int TNT=TIM4->CNT;
  Now_Speed=TNT-32767;
  TIM4->CNT=32767;
  Now_Position+=Now_Speed*0.005;
}

//����ֵ������
//���㴦�������޸�
void AS5047p_Get()
{ 
  
  if(Motor != BRUSHLESS_NONSENSOR)
    Now_Position = as5047p_Get_Position();
  Now_Speed = Now_Position - Last_Position;
  
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
  Last_Position = Now_Position;
}

void Hall_Get(){
  Now_Speed_Hall = Hall_CNT;
  Now_Speed = Now_Speed_Hall;
  if(Duty<0)
    Now_Speed = -Now_Speed;
  Hall_CNT = 0;
  Now_Position+=Now_Speed;
}
void Hall_AS5047p_Get(){
  Now_Speed_Hall = Hall_CNT;
  Now_Speed = Now_Speed_Hall;
  if(Duty<0)
    Now_Speed = -Now_Speed;
  Hall_CNT = 0;
  if(Motor != BRUSHLESS_NONSENSOR)
    Now_Position = as5047p_Get_Position();
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
  case HALLINC:
    Hall_Get();
    break;
  case HALLABS:
    Hall_AS5047p_Get();
    break;
  } 
}