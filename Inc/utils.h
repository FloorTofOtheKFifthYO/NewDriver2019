#ifndef __utils_H
#define __utils_H
#ifdef __cplusplus
extern "C" {
#endif
 
extern int brush_control_flag;   
extern int move_flag;
    
extern float speed_now;
extern float position_now;
extern float position_last;

extern float target_pwm;
extern float target_speed;
extern float target_position;
    
extern float wave_arg[4];
extern int send_wave_flag;

extern int motor_type_flag; 

typedef enum{
    BRUSH = 0,
    BRUSHLESS
}Motor_type;
    
typedef struct{
    float SetData;             //�����趨ֵ
    float ActualData;          //����ʵ��ֵ
    float integral;             //�������ֵ
    float err;          //����ƫ��ֵ
    float err_last;             //������һ��ƫ��ֵ
    float Kp;
    float Ki;
    float Kd;             //������������֡�΢��ϵ��
}PID_struct;

float PID_release(PID_struct *PID,float target_data, float actual_data);
void send_wave(float arg1,float arg2,float arg3,float arg4);
    
#ifdef __cplusplus
}
#endif
#endif /*__utils_H */