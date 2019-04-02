#ifndef __motor_H
#define __motor_H
#ifdef __cplusplus
extern "C" {
#endif
#include "board.h"
#include "utils.h"
#include "encoder.h"
#include <stdbool.h>
 
typedef struct{
    Phase_State Brush_Phase_pos;
    Phase_State Brush_Phase_neg;
}Brush_Phase;

extern Brush_Phase Brush_chl_AB;
extern Brush_Phase Brush_chl_BC;
extern Brush_Phase Brush_chl_AC;
extern Brush_Phase Brush_chl;

extern double MAXSPEED;

void Brush_PWM_Control(float pwm);  
void PWM_Control(float duty);
void Control();

#ifdef __cplusplus
}
#endif
#endif /*__motor_H */