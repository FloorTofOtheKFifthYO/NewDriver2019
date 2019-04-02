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
   
typedef struct{
  char * var_name;
  void * value_ptr;
}Var_Edit_Struct;

     
void cmd_driver_info_func(int argc,char *argv[]);

void cmd_write_flash_func(int argc,char *argv[]);
void cmd_read_flash_func(int argc,char *argv[]);

void cmd_motor_mode_func(int argc,char *argv[]);
void cmd_encoder_mode_func(int argc,char *argv[]);
void cmd_control_mode_func(int argc,char *argv[]);

void cmd_set_duty_func(int argc,char *argv[]);
void cmd_current_func(int argc,char *argv[]);
void cmd_speed_func(int argc,char *argv[]);
void cmd_position_func(int argc,char *argv[]);

void cmd_current_pid_func(int argc,char *argv[]);
void cmd_speed_pid_func(int argc,char *argv[]);
void cmd_position_pid_func(int argc,char *argv[]);

void cmd_send_wave_func(int argc,char *argv[]);
void cmd_brush_change_func(int argc,char *argv[]);
   
void cmd_state_func(int argc,char *argv[]);

void cmd_set_val_func(int argc,char *argv[]);
void cmd_get_hall_func(int argc,char *argv[]);
void cmd_set_fd6288_func(int argc,char *argv[]);
void cmd_set_mode_func(int argc,char *argv[]);
void cmd_set_mode_s_func(int argc,char *argv[]);
void cmd_set_phase_func(int argc,char *argv[]);
void cmd_phase_change_func(int argc,char *argv[]);


void cmd_rotate_test_func(int argc,char *argv[]);
void cmd_get_start_position_func(int argc,char *argv[]);
void cmd_read_mag_func(int argc,char *argv[]);
void cmd_write_as5047p_position_func(int argc,char *argv[]);

void cmd_max_speed_func(int argc,char *argv[]);
void cmd_brushlessAbsolute_fonc(int argc,char *argv[]);

#ifdef __cplusplus
}
#endif
#endif /*__ cmd_func_H */
