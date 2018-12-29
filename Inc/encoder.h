#ifndef __encoder_H
#define __encoder_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "tim.h"
#include "as5047p.h"
     
typedef enum{
    INCREMENT = 0,
    ABSOLUTE
}Encoder_type;
     
int Encoder_read_speed();
#ifdef __cplusplus
}
#endif
#endif /*__encoder_H*/