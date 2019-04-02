#include "cmd_func.h"
#include "board.h"
#include "flash.h"
#include "can.h"
#include "gpio.h"
#include "utils.h"
#include "motor.h"
#include "encoder.h"
#include "as5047p.h"
#include "cmd.h"

void cmd_driver_info_func(int argc,char *argv[])
{
    switch(Motor){
    case BRUSH:
        {
            uprintf("brush motor\r\n");
            
            if(Brush_chl.Brush_Phase_pos.High==A&&Brush_chl.Brush_Phase_pos.Low==B)
                uprintf("channel : AB\r\n");
            else if(Brush_chl.Brush_Phase_pos.High==B&&Brush_chl.Brush_Phase_pos.Low==C)
                uprintf("channel : BC\r\n");
            else
                uprintf("channel : AC\r\n");
            break;
        }
    case BRUSHLESS:
        {
            uprintf("brushless motor\r\n");
            if(Board_Mode==NORMAL)
                uprintf("Board_Mode NORMAL\r\n");  
            else if(Board_Mode==TEST)
                uprintf("Board_Mode TEST\r\n");  
            else if(Board_Mode==TEST_TALBE)
                uprintf("Board_Mode TEST_TALBE\r\n");  
            break;
        }
    case BRUSHLESS_NONSENSOR:
        {
            uprintf("brushless motor\r\n");
            if(Board_Mode==NORMAL)
                uprintf("Board_Mode NORMAL\r\n");  
            else if(Board_Mode==TEST)
                uprintf("Board_Mode TEST\r\n");  
            else if(Board_Mode==TEST_TALBE)
                uprintf("Board_Mode TEST_TALBE\r\n");  
            break;
        }
    default:
        break;
    }
    
    if(Encoder==INCREMENT)
        uprintf("encoder : incerment\r\n");
    else if(Encoder==ABSOLUTE)
        uprintf("encoder : absolute\r\n");
    else if(Encoder==HALLINC)
        uprintf("encoder : hall\r\n");
    else if(Encoder==HALLABS)
        uprintf("encoder : hall absolute\r\n");
    
    if(Control_Mode==0)
        uprintf("control mode : PWM\r\n");
    else if(Control_Mode==1)
        uprintf("control mode : current\r\n");
    else if(Control_Mode==2)
        uprintf("control mode : speed\r\n");
    else if(Control_Mode==3)
        uprintf("control mode : speed-current\r\n");
    else if(Control_Mode==4)
        uprintf("control mode : position\r\n");
    else if(Control_Mode==5)
        uprintf("control mode : position-current\r\n");
    else if(Control_Mode==6)
        uprintf("control mode : position-speed\r\n");
    else
        uprintf("control mode : position-speed-current\r\n");
}

//write_flash 0
void cmd_write_flash_func(int argc,char *argv[])
{
    uprintf("set can id = %d\r\n", atoi(argv[1]));
    ID = atoi(argv[1]);
    flash_data[0]=ID;
    write_prams();
}

//read_flash
void cmd_read_flash_func(int argc,char *argv[])
{
    load_prams();
}


//set_motor_mode 0(有刷）/1（无刷）
void cmd_motor_mode_func(int argc,char *argv[])
{
    if(atoi(argv[1])==0)
        uprintf("set motor mode as brush\r\n");
    else if(atoi(argv[1])==1)
        uprintf("set motor mode as brushless\r\n");
    else if(atoi(argv[1])==2)
        uprintf("set motor mode as brushless-nonsensor\r\n");
    else
        ;
    Motor=atoi(argv[1]);
    flash_data[1]=Motor;
    write_prams();
}

//set_encoder_mode 0(增量式）/1（绝对值）
void cmd_encoder_mode_func(int argc,char *argv[])
{
    if(atoi(argv[1])==0)
        uprintf("set encoder mode as increment\r\n");
    else if(atoi(argv[1])==1)
        uprintf("set encoder mode as absolute\r\n");
    else if(atoi(argv[1])==2)
        uprintf("set encoder mode as hall\r\n");
    else if(atoi(argv[1])==3)
        uprintf("set encoder mode as hall absolute\r\n");
    Encoder=atoi(argv[1]);
    flash_data[2]=Encoder;
    write_prams();
}

//set_control_mode 0(PWM)/1(电流环）/2（速度环）/3（速度电流环）/4（位置环）/5（位置电流环）/6（位置速度环）/7（位置速度电流环）
void cmd_control_mode_func(int argc,char *argv[])
{
    if(atoi(argv[1])==0)
        uprintf("set control mode as PWM\r\n");
    else if(atoi(argv[1])==1)
        uprintf("set control mode as current\r\n");
    else if(atoi(argv[1])==2)
        uprintf("set control mode as speed\r\n");
    else if(atoi(argv[1])==3)
        uprintf("set control mode as speed-current\r\n");
    else if(atoi(argv[1])==4)
        uprintf("set control mode as position\r\n");
    else if(atoi(argv[1])==5)
        uprintf("set control mode as position-current\r\n");
    else if(atoi(argv[1])==6)
        uprintf("set control mode as position-speed\r\n");
    else
        uprintf("set control mode as position-speed-current\r\n");
    Control_Mode=atoi(argv[1]);
    flash_data[3]=Control_Mode;
    write_prams();
}

//duty 1
void cmd_set_duty_func(int argc,char *argv[])
{
    if(Motor==BRUSH)
        Brush_PWM_Control(atof(argv[1]));
    else
        Set_Motor_Duty(atof(argv[1]));
    uprintf("ok,set motor duty=%f\r\n",atof(argv[1]));
}

//current 1
void cmd_current_func(int argc,char *argv[])
{
    uprintf("set current = %f\r\n", atof(argv[1]));
    Target_Current=atof(argv[1]);
}

//speed 1
void cmd_speed_func(int argc,char *argv[])
{
    uprintf("set speed = %f\r\n", atof(argv[1]));
    Target_Speed=atof(argv[1]);
}

//position 1
void cmd_position_func(int argc,char *argv[])
{
    reset_PID(&Position_PID);
    uprintf("set position = %f\r\n", atof(argv[1]));
    Target_Position=atof(argv[1]);
    //setup_once_flag = 1;
}

//current_pid 1 2 3
void cmd_current_pid_func(int argc,char *argv[])
{
    uprintf("set current pid = %f %f %f\r\n", atof(argv[1]),atof(argv[2]),atof(argv[3]));
    Current_PID.KP=atof(argv[1]);
    Current_PID.KI=atof(argv[2]);
    Current_PID.KD=atof(argv[3]);
    flash_data[4]=Current_PID.KP;
    flash_data[5]=Current_PID.KI;
    flash_data[6]=Current_PID.KD;
    write_prams();
}

//speed_pid 1 2 3
void cmd_speed_pid_func(int argc,char *argv[])
{
    uprintf("set speed pid = %f %f %f\r\n", atof(argv[1]),atof(argv[2]),atof(argv[3]));
    Speed_PID.KP=atof(argv[1]);
    Speed_PID.KI=atof(argv[2]);
    Speed_PID.KD=atof(argv[3]);
    flash_data[7]=Speed_PID.KP;
    flash_data[8]=Speed_PID.KI;
    flash_data[9]=Speed_PID.KD;
    write_prams();
}


//position_pid 1 2 3
void cmd_position_pid_func(int argc,char *argv[])
{
    uprintf("set position pid = %f %f %f\r\n", atof(argv[1]),atof(argv[2]),atof(argv[3]));
    Position_PID.KP=atof(argv[1]);
    Position_PID.KI=atof(argv[2]);
    Position_PID.KD=atof(argv[3]);
    flash_data[10]=Position_PID.KP;
    flash_data[11]=Position_PID.KI;
    flash_data[12]=Position_PID.KD;
    write_prams();
}

void cmd_send_wave_func(int argc,char *argv[])
{
    send_wave_flag = atoi(argv[1]);
}

void cmd_brush_change_func(int argc,char *argv[])
{
  if(compare_string(argv[1],"ab"))
    Brush_chl = Brush_chl_AB;
  else if(compare_string(argv[1],"bc"))
    Brush_chl = Brush_chl_BC;
  else if(compare_string(argv[1],"ac"))
    Brush_chl = Brush_chl_AC;
}

void cmd_state_func(int argc,char *argv[])
{
    switch(Motor){
    case BRUSH:
        {
            uprintf("brush motor\r\n");
            
            if(Brush_chl.Brush_Phase_pos.High==A&&Brush_chl.Brush_Phase_pos.Low==B)
                uprintf("channel : AB\r\n");
            else if(Brush_chl.Brush_Phase_pos.High==B&&Brush_chl.Brush_Phase_pos.Low==C)
                uprintf("channel : BC\r\n");
            else
                uprintf("channel : AC\r\n");
            break;
        }
    case BRUSHLESS:
        {
            uprintf("brushless motor\r\n");
            if(Board_Mode==NORMAL)
                uprintf("Board_Mode NORMAL\r\n");  
            else if(Board_Mode==TEST)
                uprintf("Board_Mode TEST\r\n");  
            else if(Board_Mode==TEST_TALBE)
                uprintf("Board_Mode TEST_TALBE\r\n");  
            break;
        }
    case BRUSHLESS_NONSENSOR:
        {
            uprintf("brushless Non-sensor motor\r\n");
            if(Board_Mode==NORMAL)
                uprintf("Board_Mode NORMAL\r\n");  
            else if(Board_Mode==TEST)
                uprintf("Board_Mode TEST\r\n");  
            else if(Board_Mode==TEST_TALBE)
                uprintf("Board_Mode TEST_TALBE\r\n");  
            break;
        }
    default:
        break;
    }
    
    if(Encoder==INCREMENT)
        uprintf("encoder : incerment\r\n");
    else if(Encoder==ABSOLUTE)
        uprintf("encoder : absolute\r\n");
    else if(Encoder==HALLINC)
        uprintf("encoder : hall\r\n");
    else if(Encoder==HALLABS)
        uprintf("encoder : hall absolute\r\n");
    
    if(Control_Mode==0)
        uprintf("control mode : PWM\r\n");
    else if(Control_Mode==1)
        uprintf("control mode : current\r\n");
    else if(Control_Mode==2)
        uprintf("control mode : speed\r\n");
    else if(Control_Mode==3)
        uprintf("control mode : speed-current\r\n");
    else if(Control_Mode==4)
        uprintf("control mode : position\r\n");
    else if(Control_Mode==5)
        uprintf("control mode : position-current\r\n");
    else if(Control_Mode==6)
        uprintf("control mode : position-speed\r\n");
    else
        uprintf("control mode : position-speed-current\r\n");
    
    uprintf("ID=%d\r\n", ID);
    uprintf("Motor_Mode is %x\r\n", Motor);
    uprintf("Control Mode is %x\r\n", Control_Mode);
    uprintf("Encoder is %x\r\n", Encoder);
    uprintf("target Current:%f\r\n", Target_Current);
    uprintf("Now_Current:%f\r\n", Now_Current);
    uprintf("tartget speed is :%f\r\n", Target_Speed);
    uprintf("now speed is :%f\r\n", Now_Speed);
    uprintf("tartget position is :%f\r\n", Target_Position);
    uprintf("now position is :%f\r\n", Now_Position);
    uprintf("TIM2->CCR1=%d\r\n",(int)TIM2->CCR1);
    uprintf("TIM2->CCR2=%d\r\n",(int)TIM2->CCR2);
    uprintf("TIM2->CCR3=%d\r\n",(int)TIM2->CCR3);
    uprintf("TIM4->CNT=%d\r\n", (int)TIM4->CNT);
    uprintf("Motor_Duty=%f\r\n",Motor_Duty);
    uprintf("Motor_Duty_Set=%f\r\n",Motor_Duty_Set);
    uprintf("Duty=%f\r\n",Duty);
    uprintf("max speed = %f\r\n", MAXSPEED);
    uprintf("Now_Speed_Hall = %f\r\n", Now_Speed_Hall);
}

/***有感无刷**/
Var_Edit_Struct Var_List[10]={
    {"first",&First_Time_Check}
};


void cmd_set_val_func(int argc,char *argv[])
{
    void * edit_value;
    
    for(int i=0;i<sizeof(Var_List)/sizeof(Var_Edit_Struct);++i){
        if(compare_string(Var_List[i].var_name,argv[1])){
            edit_value=Var_List[i].value_ptr;
            break;
        }
    }
    
    if(compare_string(argv[2],"u8")){
        *(uint8_t *)edit_value=(uint8_t)atoi(argv[3]);
        uprintf("ok set %s = %d\r\n",argv[1],*(uint8_t *)edit_value);  
    }else if(compare_string(argv[2],"int")){
        *(int16_t *)edit_value=(int16_t)atoi(argv[3]);
        uprintf("ok set %s = %d\r\n",argv[1],*(int16_t *)edit_value);
    }else if(compare_string(argv[2],"f")){
        *(float *)edit_value=atof(argv[3]);
        uprintf("ok set %s = %f\r\n",argv[1],*(float *)edit_value);
    }
}

void cmd_get_hall_func(int argc,char *argv[])
{
    uprintf("gpio-A5:%d\r\n\
            gpio-A4:%d\r\n\
                gpio-A3:%d\r\n",\
                    HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5),
                    HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4),
                    HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3));
}

void cmd_set_fd6288_func(int argc,char *argv[])
{
    int io_state=0;
    // set_fd a h 1
    io_state=atoi(argv[3]);//l
    
    if(compare_string(argv[2],"h")){
        switch(argv[1][0]){
        case 'a':
            Set_Phase_High_Speed(A,50*io_state);
            break;
        case 'b':
            Set_Phase_High_Speed(B,50*io_state);
            break;
        case 'c':
            Set_Phase_High_Speed(C,50*io_state);
            break;      
        default:
            uprintf("error phase!\r\n");
            return ;
        }
        
    }else{
        switch(argv[1][0]){
        case 'a':
            Set_Phase_Low_State(A,io_state);
            break;
        case 'b':
            Set_Phase_Low_State(B,io_state);
            break;
        case 'c':
            Set_Phase_Low_State(C,io_state);
            break;
        default:
            uprintf("error phase!\r\n");
            return ;      
        }
    }
    uprintf("ok,set %s-%s ->  %d,\r\n",argv[1],argv[2],io_state);
    
}



void cmd_set_mode_func(int argc,char *argv[])
{
    if(argv[1][0]=='n'){
        Board_Mode=NORMAL;
        uprintf("ok set mode =nn\r\n");
    }else if(compare_string(argv[1],"test")){
        Board_Mode=TEST;
    }else if(compare_string(argv[1],"table")){
        Set_To_Statble_Positon();
        Board_Mode=TEST_TALBE;
        uprintf("use your hand to rotate motor!\r\n");
    }
    uprintf("ok set mode =%c\r\n",argv[1][0]);
}


void cmd_set_mode_s_func(int argc,char *argv[])
{
    Board_Mode=NORMAL;
    Start_Position=atoi(argv[1]);
    uprintf("ok,set start=%d\r\n",Start_Position);
}

void cmd_set_phase_func(int argc,char *argv[])
{
    char * high[4]={0,0,"h","1"};
    char * low[4]={0,0,"l","1"};  
    
    high[1]=argv[1];
    low[1]=argv[2];
    Close_Phases();
    
    cmd_set_fd6288_func(4,high);
    cmd_set_fd6288_func(4,low);
    
    HAL_Delay(atoi(argv[3]));
    Close_Phases();
    uprintf("Delay %f ms !\r\n",atof(argv[3]));
}

void cmd_phase_change_func(int argc,char *argv[])
{
    char high,low;
    high=argv[1][0];
    low=argv[2][0];
    if(high=='a'){
        if(low=='b')
            Phase_Change(&AB,50);
        if(low=='c')
            Phase_Change(&AC,50);
    }else if(high=='b'){
        if(low=='a')
            Phase_Change(&BA,50);
        if(low=='c')
            Phase_Change(&BC,50);
    }else if(high=='c'){
        if(low=='a')
            Phase_Change(&CA,50);
        if(low=='b')
            Phase_Change(&CB,50);
    }
    HAL_Delay(atoi(argv[3]));
    
    Close_Phases();
}


void cmd_rotate_test_func(int argc,char *argv[])
{
    Rotate_Test();
}

void cmd_get_start_position_func(int argc,char *argv[])
{
    Get_Start_Position();
}

void cmd_read_mag_func(int argc,char *argv[])
{
    uprintf("position=%d\r\n",as5047p_Get_Position());
}

void cmd_write_as5047p_position_func(int argc,char *argv[])
{
    uprintf("write position:%d\r\n", atoi(argv[1]));
    uint16_t return_p=as5047p_Write_Position((uint16_t)atoi(argv[1]));
    uprintf("return position:%d\r\n", return_p);
}
//max speed 
void cmd_max_speed_func(int argc,char *argv[])
{
    
    MAXSPEED=atof(argv[1]);
    uprintf("max speed = %f\r\n", MAXSPEED);
    flash_data[13]=MAXSPEED;
    write_prams();
}

void cmd_brushlessAbsolute_fonc(int argc,char *argv[])
{
  flash_data[1] = 1;
  flash_data[2] = 1;
  flash_data[3] = 0;
    flash_data[4]=0;
    flash_data[5]=0;
    flash_data[6]=0;
    flash_data[7]=0;
    flash_data[8]=0;
    flash_data[9]=0;
    flash_data[10]=0;
    flash_data[11]=0;
    flash_data[12]=0;
    flash_data[13]=100000;
    write_prams();
}  