#ifndef __can_func_H
#define __can_func_H
#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f1xx_hal.h"
#include "main.h"

void can_pwm_func(CanRxMsgTypeDef* pRxMsg);
void can_current_func(CanRxMsgTypeDef* pRxMsg);
void can_speed_func(CanRxMsgTypeDef* pRxMsg);
void can_speed_current_func(CanRxMsgTypeDef* pRxMsg);
void can_position_func(CanRxMsgTypeDef* pRxMsg);
void can_position_current_func(CanRxMsgTypeDef* pRxMsg);
void can_position_speed_func(CanRxMsgTypeDef* pRxMsg);
void can_position_speed_current_func(CanRxMsgTypeDef* pRxMsg);


   
 #ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */