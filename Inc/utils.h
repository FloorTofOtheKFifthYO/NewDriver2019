#ifndef __utils_H
#define __utils_H
#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    BRUSH = 0,
    BRUSHLESS,
    BRUSHLESS_NONSENSOR
}Motor_Type;
    
typedef struct{
	float KP;
	float KD;
	float KI;
	float i;
	float last_err;
	float i_max;
	float last_d;
    float I_TIME;    //2018年7月9日 修改，增加积分时间，
                    //以前是作为宏定义，但不同PID的积分应该是不一样的                 
}PID_Struct;

typedef enum{
  PWM_Mode=0x00,
  Current_Mode=0x01,
  Speed_Mode=0x02,
  Speed_Current_Mode=0x03,
  Position_Mode=0x04,
  Position_Current_Mode=0x05,
  Position_Speed_Mode=0x06,
  Position_Speed_Current_Mode=0x07
}Control_Mode_Struct;

extern PID_Struct Current_PID;
extern PID_Struct Speed_PID;
extern PID_Struct Position_PID;

extern float Duty;
extern float Target_Speed;
extern float Now_Speed;
extern float Target_Position;
extern float Now_Position;
extern float Target_Current;
extern float Now_Current;
extern float Now_Current_buffer;
extern float Last_Position;
extern float Target_PWM;

extern Control_Mode_Struct Control_Mode;
extern Motor_Type Motor;

extern int send_wave_flag;


float PID_Release(PID_Struct *PID,float target,float now);
void PID_init();
void send_wave(float arg1,float arg2,float arg3,float arg4);
    
#ifdef __cplusplus
}
#endif
#endif /*__utils_H */