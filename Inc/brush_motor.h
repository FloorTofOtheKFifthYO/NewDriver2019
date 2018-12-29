#ifndef __brush_motor_H
#define __brush_motor_H
#ifdef __cplusplus
extern "C" {
#endif
#include "board.h"
#include "utils.h"
#include "encoder.h"
#include <stdbool.h>
 
typedef enum{
    PWM_MODE=0,
    SPEED_MODE,
    POSITION_MODE
}Brush_control_mode;
    
typedef struct{
    Phase_State Brush_Phase_pos;
    Phase_State Brush_Phase_neg;
}Brush_Phase;


#define Brush_encoder_speed_read() Encoder_read_speed()
#define Brush_encoder_position_read() as5047p_read_pos()
#define ABSOLUTEMAXPOS AS5047PMAXPOS

extern Brush_Phase Brush_chl_AB;
extern Brush_Phase Brush_chl_BC;
extern Brush_Phase Brush_chl_AC;
extern Brush_Phase Brush_chl;

extern bool Brush_position_with_speed_flag;//是否有速度环的位置环
extern int Brush_encoder_type;//编码器种类 （增量式/绝对值式）

extern int Brush_motor_control_flag;//有刷控制模式  pwm/speed/position

extern PID_struct brush_speed_PID;
extern PID_struct brush_position_PID;
    
void Brush_pwm_control(float pwm);
void Brush_motor_control();

#ifdef __cplusplus
}
#endif
#endif /*__brush_motor_H */