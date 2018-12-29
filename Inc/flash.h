#ifndef __flash_H
#define __flash_H
#ifdef __cplusplus
 extern "C" {
#endif

   
#include "stm32f1xx_hal.h"
#include "main.h"
  
   
extern float flash_data[9];
void write_prams();
void load_prams();
   
#ifdef __cplusplus
}
#endif
#endif /*__ speed_H */