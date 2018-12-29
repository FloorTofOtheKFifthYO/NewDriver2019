#include "cmd_func.h"
#include "board.h"
#include "flash.h"
#include "can.h"
#include "gpio.h"
#include "utils.h"
#include "brush_motor.h"

void cmd_hello_func(int argc,char *argv[])
{
    uprintf("hello world");
}

void cmd_info_func(int argc,char *argv[])
{
    uprintf("version 1.0.0\r\n");
    uprintf("update time 2018.12.30\r\n");
}

void cmd_driver_info_func(int argc,char *argv[])
{
    switch(motor_type_flag){
    case BRUSH:
        {
            uprintf("brush motor\r\n");
            if(Brush_chl.Brush_Phase_pos.High==A&&Brush_chl.Brush_Phase_pos.Low==B)
                uprintf("channel:AB\r\n");
            else if(Brush_chl.Brush_Phase_pos.High==B&&Brush_chl.Brush_Phase_pos.Low==C)
                uprintf("channel:BC\r\n");
            else if(Brush_chl.Brush_Phase_pos.High==A&&Brush_chl.Brush_Phase_pos.Low==C)
                uprintf("channel:AC\r\n");
            else
                uprintf("没有这种通道\r\n");
            if(Brush_encoder_type==INCREMENT)
                uprintf("encoder:incerment\r\n");
            else if(Brush_encoder_type==ABSOLUTE)
                uprintf("encoder:absolute\r\n");
            else
                uprintf("没这种编码器\r\n");
            
            uprintf("brush_control_flag=%d\r\n", brush_control_flag);
            break;
        }
    case BRUSHLESS:
        {
            uprintf("brushless motor\r\n");
            break;
        }
    default:
        {
            uprintf("没这种电机\r\n");
            break;
        }
    }  
}
void cmd_stop_func(int argc,char *argv[])
{
    move_flag=0;
    brush_control_flag=0;
    Brush_pwm_control(0.0);
}
//pwm 1
void cmd_pwm_func(int argc,char *argv[])
{
    uprintf("set pwm = %f", atof(argv[1]));
    Brush_pwm_control(atof(argv[1]));
}

//speed 1
void cmd_speed_func(int argc,char *argv[])
{
    move_flag=1;
    brush_control_flag=1;
    target_speed=atof(argv[1]);
}

//speed_pid 1 2 3
void cmd_speed_pid_func(int argc,char *argv[])
{
    brush_speed_PID.Kp=atof(argv[1]);
    brush_speed_PID.Ki=atof(argv[2]);
    brush_speed_PID.Kd=atof(argv[3]);
    flash_data[1]=brush_speed_PID.Kp;
    flash_data[2]=brush_speed_PID.Ki;
    flash_data[3]=brush_speed_PID.Kd;
    write_prams();
}

//position 1
void cmd_position_func(int argc,char *argv[])
{
    move_flag=1;
    brush_control_flag=1;
    target_position=atof(argv[1]);
}

//position_pid 1 2 3
void cmd_position_pid_func(int argc,char *argv[])
{
    brush_position_PID.Kp=atof(argv[1]);
    brush_position_PID.Ki=atof(argv[2]);
    brush_position_PID.Kd=atof(argv[3]);
    flash_data[4]=brush_position_PID.Kp;
    flash_data[5]=brush_position_PID.Ki;
    flash_data[6]=brush_position_PID.Kd;
    write_prams();
}

//write_flash LF
void cmd_write_flash_func(int argc,char *argv[])
{
    flash_data[0]=atoi(argv[1]);
    write_prams();
    can_init();
}

//read_flash
void cmd_read_flash_func(int argc,char *argv[])
{
    load_prams();
}

void cmd_read_pwm_func(int argc,char *argv[])
{
    uprintf("TIM2->CCR1=%d\r\n",TIM2->CCR1);
    uprintf("TIM2->CCR2=%d\r\n",TIM2->CCR2);
    uprintf("TIM2->CCR3=%d\r\n",TIM2->CCR3);
}