#ifndef __encoder_H
#define __encoder_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "tim.h"
#include "as5047p.h"
     
typedef enum{
    INCREMENT = 0,
    ABSOLUTE,
    HALLINC,
    HALLABS
}Encoder_Type;

extern Encoder_Type Encoder;

void Get_SP();

#ifdef __cplusplus
}
#endif
#endif /*__encoder_H*/