#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include "spi.h"

typedef enum{
  HALL,         //霍尔传感器
  MAG_Encoder,  //磁编码器
}Sensor_Type;

#define SENSOR_HALL
//#define SENSOR_MAG

/**********原本无感的**************
// 以下定义为使用磁编码器时才会用到
#define MOTOR_N         12
#define MOTOR_P         14
#define MOTOR_NP        84  // 最小公倍数，也就是一圈要换向多少次
#define MAG_ENCODER_LINES       1024 //磁编码器线数
#define MIN_ANGLE       24.4  // 最小换向角度
************************************/

// 以下定义为使用磁编码器时才会用到
#define MOTOR_N         12
#define MOTOR_P         14
#define MOTOR_NP        42  // 最小公倍数，也就是一圈要换向多少次
#define MAG_ENCODER_LINES       16384 //磁编码器线数
#define MIN_ANGLE       390  // 最小换向角度 (MAG_ENCODER_LINES/MOTOR_NP)
#define HALF_MIN_ANGLE  MIN_ANGLE/2

typedef enum{
  IO_Low=0x00,
  IO_High=0x01
}IO_State;

typedef enum{
  A,
  B,
  C
}Phase;

extern char  Phase_String[3];

typedef struct{
  Phase High;      //高端MOS
  Phase Low;       //低端MOS
  int index;
}Phase_State;

typedef enum{
    TEST,
    NORMAL,
    TEST_TALBE   //换向表尝试
}Mode;

extern Mode Board_Mode;
extern Phase_State AB;
extern Phase_State AC;
extern Phase_State BC;
extern Phase_State BA;
extern Phase_State CA;
extern Phase_State CB;

extern uint8_t First_Time_Check;

#define Set_AL_State(x)        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,(GPIO_PinState)x)
#define Set_BL_State(x)        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,(GPIO_PinState)x)
#define Set_CL_State(x)        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,(GPIO_PinState)x)


#define Set_AH_Speed(x)        TIM2->CCR3=(uint16_t)(x*TIM2->ARR/100.0f)
#define Set_BH_Speed(x)        TIM2->CCR2=(uint16_t)(x*TIM2->ARR/100.0f)
#define Set_CH_Speed(x)        TIM2->CCR1=(uint16_t)(x*TIM2->ARR/100.0f)

#define TEST_TABLE_SPEED        10              //测试换向表时候的占空比

#define OPEN_TIME_MAX           15              //一相导通时间最大值，超过就关闭


void Set_Phase_High_Speed(Phase phase,float speed);
void Set_Phase_Low_State(Phase phase,IO_State state);
void Phase_Change(Phase_State *target,float speed); //换相,speed: 0~95
void Close_Phases();
void Phase_Table_Init();
uint16_t Read_Mag();

//extern Phase_State * Phase_Table[7];
extern Phase_State ** Phase_Table_Using_Sensor;
extern int Test_Table_Cnt;
extern uint8_t Phase_Test_Table[6];
extern Phase_State* Phase_Const[6];
extern Phase_State* Phase_Const_Reverse[6];
extern Phase_State * const Phase_Const_Reverse_sensor[6];
extern uint8_t Hall_Position;
uint8_t Get_Hall_Position();
void Set_To_Statble_Positon();
void Set_Motor_Duty(float duty);
extern float Motor_Duty;    

extern int dir_flag;

extern int16_t Start_Position;
extern int Phase_Change_Cnt;
extern int Phase_Open_Cnt;

extern uint16_t Mag_Position;
void Mag_Brushless_Mointor(uint16_t mag_position);
void Rotate_Test();
void Get_Start_Position();
#endif