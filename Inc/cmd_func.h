#ifndef __cmd_func_H
#define __cmd_func_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "main.h"
#include "usart.h"
#include "cmd.h"
#include "stdlib.h"
#include "math.h"
#define PI 3.1415926535
     
void cmd_hello_func(int argc,char *argv[]);
void cmd_info_func(int argc,char *argv[]);
void cmd_driver_info_func(int argc,char *argv[]);
 
void cmd_stop_func(int argc,char *argv[]);
void cmd_pwm_func(int argc,char *argv[]);

void cmd_speed_func(int argc,char *argv[]);
void cmd_speed_pid_func(int argc,char *argv[]);

void cmd_position_func(int argc,char *argv[]);
void cmd_position_pid_func(int argc,char *argv[]);

void cmd_write_flash_func(int argc,char *argv[]);
void cmd_read_flash_func(int argc,char *argv[]);

void cmd_read_pwm_func(int argc,char *argv[]);

   
#ifdef __cplusplus
}
#endif
#endif /*__ cmd_func_H */
