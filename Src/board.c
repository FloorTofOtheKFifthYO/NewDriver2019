#include "board.h"
#include "math.h"
#include "usart.h"
#include "adc.h"
#include "as5047p.h"
#include "utils.h"


#define LOW_CLOSE       IO_Low
#define LOW_OPEN        (IO_State)!LOW_CLOSE

typedef enum{
  FORWARD=0x01,
  BACKWARD=-0x01
}MAG_Direction;


Phase_State AB={A,B,0};
Phase_State AC={A,C,1};
Phase_State BC={B,C,2};
Phase_State BA={B,A,3};
Phase_State CA={C,A,4};
Phase_State CB={C,B,5};

MAG_Direction Direction=BACKWARD;
Mode Board_Mode=NORMAL;
Phase_State Now_Phase={A,B,0};
int Now_Phase_Index=0;

uint8_t phase_loop_cnt=0;
char  Phase_String[3]={'A','B','C'};

//有感
Phase_State * Phase_Table[7]={0};  // 第一个元素是空，因为霍尔没有000的状态。下标是霍尔状态。
Phase_State * Phase_Table_Reverse[7]={0}; // 反向换向表
Phase_State ** Phase_Table_Using_Sensor=0; //当前使用的换向表

//无感
//这两个，选择一个使用，具体使用哪一个，与ABC三条线的接法有关
//当对Phase_Const中，下标从0-5转一圈，如果位置增大，那么就使用Phase_Const
//否则就应该使用Phase_Const_Reverse
Phase_State * Phase_Const[6]={&AB,&AC,&BC,&BA,&CA,&CB};      //有感无感共用
Phase_State * Phase_Const_Reverse[6]={&CA,&BA,&BC,&AC,&AB,&CB};  //无感反向
Phase_State * const Phase_Const_Reverse_sensor[6]={&BA,&CA,&CB,&AB,&AC,&BC};  //有感反向
Phase_State ** Phase_Table_Using_Nonsensor=Phase_Const; //当前使用的换向表


uint8_t Phase_Test_Table[6]={5,1,3,2,6,4};//{2,6,4,5,1,3}; // 测试换向表时记录的霍尔状态
//{5,4,6,2,3,1} //反转时的霍尔状态
//{&AB,&CB,&CA,&BA,&BC,&AC};

//将其与正转时候的霍尔状态统一，方便程序里写
//{2,6,4,5,1,3}
//{&BA,&CA,&CB,&AB,&AC,&BC}

int Test_Table_Cnt=0;

uint8_t First_Time_Check;


float Motor_Duty=0;       //电机占空比

int16_t Start_Position=13406;
uint16_t Mag_Position=0;
int Phase_Change_Cnt=0;//换向计数，仅用于磁编码器的无刷电机

int Phase_Open_Cnt=0;// 相开启时间计数，防止某一相导通太长时间导致电流过大
// 1ms 增加一次，在systick中断中增加
uint8_t Hall_Position=0;
int dir_flag=0;

//分有感和无感模式
void Set_Motor_Duty(float duty){ //设置电机占空比
  Duty = duty;
  if(Motor==BRUSHLESS)//有感无刷
  {
    if(duty<0)
      Phase_Table_Using_Sensor=Phase_Table_Reverse;
    else
      Phase_Table_Using_Sensor=Phase_Table;
  }
  else
  {
    if(duty<0)
      Direction=FORWARD;
    else
      Direction=BACKWARD;
  }  
  Motor_Duty_Set=fabs(duty);
}

//有感
void Phase_Table_Init(){
  
  uint8_t hall_state=0;
  for(int i=0;i<6;++i){
    hall_state=Phase_Test_Table[i];
    Phase_Table[hall_state]=Phase_Const[i];
    Phase_Table_Reverse[hall_state]=Phase_Const_Reverse_sensor[i];
  }
  Phase_Table_Using_Sensor=Phase_Table;
  
  /*
  //逆时针换相表
  Phase_Table[2]=&AB;
  Phase_Table[6]=&AC;
  Phase_Table[4]=&BC;
  Phase_Table[5]=&BA;
  Phase_Table[1]=&CA;
  Phase_Table[3]=&CB;
  */
}


//无感最后还有一个条件
void Phase_Change(Phase_State *target,float speed){
  Close_Phases();
  Phase_Open_Cnt=0;
  Set_Phase_Low_State(target->Low,LOW_OPEN);
  Set_Phase_High_Speed(target->High,speed);     //开启目标高低桥  
  if(Motor==BRUSHLESS_NONSENSOR)
    Now_Phase_Index=target->index;
}

//有感无感一样
void Set_Phase_Low_State(Phase phase,IO_State state){
  switch(phase){
  case A:
    Set_AL_State(state);
    break;
  case B:
    Set_BL_State(state);
    break;
  case C:
    Set_CL_State(state);
    break;
  }
}

//有感无感一样
void Set_Phase_High_Speed(Phase phase,float speed){
  speed=speed<0?-speed:speed;
  speed=speed>95?95:speed;
  switch(phase){
  case A:
    Set_AH_Speed(speed);
    break;
  case B:
    Set_BH_Speed(speed);
    break;
  case C:
    Set_CH_Speed(speed);
    break;
  }
}

//有感无感一样
void Close_Phases(){
  Set_Phase_Low_State(A,IO_Low);
  Set_Phase_Low_State(B,IO_Low);
  Set_Phase_Low_State(C,IO_Low);
  Set_Phase_High_Speed(A,0);
  Set_Phase_High_Speed(B,0);
  Set_Phase_High_Speed(C,0);
  
  //uprintf("ok,close all phases\r\n");
}

//有感
uint16_t Read_Mag(){
  uint8_t data[2];
  uint16_t position;
  HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET);
  HAL_SPI_Receive(&hspi2,data,2,10);
  HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_SET);
  position=(uint16_t)(data[0])<<8|data[1];
  position>>=6;
  //uprintf("data = %x %x \r\n",data[0],data[1]);
  //uprintf("position=%d\r\n",position);
  return position;
}

//有感
uint8_t Get_Hall_Position(){
  uint8_t temp;
  temp=GPIOA->IDR&(GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
  return temp>>3;
}

//有感
void Set_To_Statble_Positon(){  //将转子定位到固定的位置
  int i=0;
  uint8_t position,last_position;
  int position_same_cnt=0;         //本次位置与上次位置相同的情况 计数值
  for(i=0;i<10;++i){
    Phase_Change(&CB,TEST_TABLE_SPEED); //将转子转到CB处，这样就从AB开始换相
    HAL_Delay(10);
    Close_Phases();
    
    position=Get_Hall_Position();
    if(position==last_position){
      position_same_cnt++;
    }
    last_position=position;
    if(position_same_cnt>=3){
      uprintf("already in stable position!i=%d the same cnt=%d\r\n",i,position_same_cnt);
      break;
    }
  }
}

/**************************无感************************************/
void Set_To_CB_Positon(){  
  //将转子定位CB位置，为什么是CB?因为他是换向表最后一个位置，如果换向表不同
  //则要定位到其他位置（也是换向表的最后一个）
  for(int i=0;i<=10;++i){
    Phase_Change(Phase_Const[5],TEST_TABLE_SPEED);
    HAL_Delay(5);
    Close_Phases();
    HAL_Delay(5);
  }
}

void Get_Start_Position(){
  Set_To_CB_Positon();
  HAL_Delay(1000);
  uprintf("start_position=%d\r\n",as5047p_Get_Position());
}
void Rotate_Test(){
  //测试板子，以选择正确的换向表
  //换向表错误的话，板子会来回振动，无法正常运转
  uint16_t last_position=0;
  uint16_t position=0;
  int larger_cnt=0;
  
  Set_To_CB_Positon();
  
  HAL_Delay(1000);
  last_position=as5047p_Get_Position();
  for(int j=0;j<3;++j){
    for(int i=0;i<6;++i){
      Phase_Change(Phase_Const[i],TEST_TABLE_SPEED);
      HAL_Delay(5);
      Close_Phases();
      position=as5047p_Get_Position();
      if(position-last_position>0){
        larger_cnt++;
      }else{
        larger_cnt--;
      }
      last_position=position;
    }
  }
  
  if(larger_cnt>0){
    Phase_Table_Using_Nonsensor=Phase_Const;
  }else{
    Phase_Table_Using_Nonsensor=Phase_Const_Reverse;
  }
  
  uprintf("larger_cnt:%d\r\n",larger_cnt);
}

void Mag_Brushless_Mointor(uint16_t mag_position){
  uint32_t fixed_start_position=0;
  uint32_t position=(uint32_t)mag_position;
  
  int temp=0;
  fixed_start_position=Start_Position+Direction*HALF_MIN_ANGLE;
  if(position<fixed_start_position){
    position+=MAG_ENCODER_LINES;
  }
  if(Direction==FORWARD){
    temp=(int)((float)(position-(fixed_start_position))/MIN_ANGLE)+1;
  }else{
    temp=(int)((float)(position-(fixed_start_position))/MIN_ANGLE)-3;
  }
  // 假设电机运转到换向点，实际上有两个方向，往前一格，和往后一格
  // 比如运行顺序为 AB AC BC
  // 假设电机运行到AC，此时它可以往AB，也可以往BC，两种选择对应正转和反转
  if(temp<0){
    temp+=6;
  }
  temp%=6;
  
  if(Board_Mode!=NORMAL){
    return ;
  }
  
  if(temp!=Phase_Change_Cnt){
    Phase_Change_Cnt=temp;   //判断是否到达下一相，如果是，换相
    if(Motor_Duty_Set!=Motor_Duty)
      Motor_Duty = Motor_Duty_Set;
    Phase_Change(Phase_Table_Using_Nonsensor[Phase_Change_Cnt],Motor_Duty);
    //uprintf("%d\r\n",Phase_Change_Cnt);
  }
}
