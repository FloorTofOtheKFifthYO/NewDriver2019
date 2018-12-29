#include "encoder.h"

int Encoder_read_speed()
{
    int TNT=TIM4->CNT;
    int now_speed=TNT-32767;
    TIM4->CNT=32767;
    return  now_speed;
}