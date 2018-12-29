#ifndef __as5047p_H
#define __as5047p_H
#ifdef __cplusplus
 extern "C" {
#endif
     
#include "spi.h"

#define AS5047PMAXPOS 16384
     
#define as5047p_Enable() 		HAL_GPIO_WritePin(CSN_MAG_GPIO_Port, CSN_MAG_Pin, GPIO_PIN_RESET)
#define as5047p_Disable() 		HAL_GPIO_WritePin(CSN_MAG_GPIO_Port, CSN_MAG_Pin, GPIO_PIN_SET)

     
extern uint16_t as5047p_read_pos();
#ifdef __cplusplus
}
#endif
#endif /*__ as5047p_H */