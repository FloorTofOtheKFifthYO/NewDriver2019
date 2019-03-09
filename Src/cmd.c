#include "cmd.h"


/*
  *�������Ľṹ��
  * ������������Ҫ�ڴ˼��ϣ�
  * CMD_ADD("������","����ʹ�÷�������Ϊ�ո�,�����ܲ���˫���ţ�",��Ӧ�����ִ�к�����)
  * ע�����һ������Ҫ���ţ�ǰ��Ķ���Ҫ����
  */
static cmd_struct cmd_tbl[] = {
  CMD_ADD("help","help",cmd_help_func),
  CMD_ADD("driver_info","about",cmd_driver_info_func),
  
  CMD_ADD("write_flash","write the can id to flash",cmd_write_flash_func),
  CMD_ADD("read_flash","read flash",cmd_read_flash_func),
  
  CMD_ADD("motor_mode","brush or brushless",cmd_motor_mode_func),
  CMD_ADD("encoder_mode","increment or absolute",cmd_encoder_mode_func),
  CMD_ADD("control_mode","current or speed or position",cmd_control_mode_func),

  CMD_ADD("current","set current",cmd_current_func),
  CMD_ADD("pwm","set pwm",cmd_pwm_func),
  CMD_ADD("speed","set target speed",cmd_speed_func),
  CMD_ADD("position","set target position",cmd_position_func),
  
  CMD_ADD("current_pid","set current pid",cmd_current_pid_func),  
  CMD_ADD("speed_pid","set speed pid",cmd_speed_pid_func), 
  CMD_ADD("position_pid","set position pid",cmd_position_pid_func),
  
  CMD_ADD("state","read the absoulte encoder",cmd_state_func),     
  CMD_ADD("wave", "send wave 1 2 3 4", cmd_send_wave_func),
  
  CMD_ADD("set_mode", "set brushless mode", cmd_set_mode_func),
  CMD_ADD("set_mode_s", "set sensor brushless mode", cmd_set_mode_s_func),
  CMD_ADD("set_phase", "brushless set_phase", cmd_set_phase_func),
  CMD_ADD("phase_change", "brushless phase_change", cmd_phase_change_func),  
  CMD_ADD("set_duty", "brushless set_duty", cmd_set_duty_func),
  
  CMD_ADD("set_val", "brushless set_val", cmd_set_val_func),
  CMD_ADD("get_hall", "brushless get_hall", cmd_get_hall_func),
  CMD_ADD("set_fd6288", "brushless set_fd6288", cmd_set_fd6288_func),
  
  CMD_ADD("rotate_test", "brushless nonsensor rotate_test", cmd_rotate_test_func),
  CMD_ADD("get_start_position", "brushless nonsensor get_start_position", cmd_get_start_position_func),
  CMD_ADD("read_mag","brushless nonsensor read_as5047p_position",cmd_read_mag_func),
  CMD_ADD("write_as5047p_pos","write_as5047p_position",cmd_write_as5047p_position_func)
 
};

char cmd_line[MAX_CMD_LINE_LENGTH + 1];
char *cmd_argv[MAX_ARGC]; 

void cmd_init()
{
  for(int i = 0;i < MAX_ARGC;i++){
    cmd_argv[i] = (char *)malloc(MAX_CMD_ARG_LENGTH + 1);//��ȷ���������ݵ��ڴ�ռ䣬���Է���һ��
  }
}
/*
*���������
*/
int cmd_parse(char *cmd_line,int *argc,char *argv[]){
  char c_temp;
  int i = 0,arg_index = 0;
  int arg_cnt = 0;
  c_temp = cmd_line[i++];  
  while(c_temp != '\r'){
    if(c_temp == ' '){
      if(arg_index == 0){   //���������߲����ַ�����һ���ǿո������   
        c_temp = cmd_line[i++];
        continue;
      }
      //�ո�Ϊ������������ķָ���
      if(arg_cnt == MAX_ARGC){   //���������������,�򷵻�
        return -1;
      }
      argv[arg_cnt][arg_index] = 0;
      arg_cnt++;
      arg_index = 0;
      c_temp = cmd_line[i++];
      continue;
    }
    if(arg_index == MAX_CMD_ARG_LENGTH){   //����������ȹ������򱨴���
      return -2;
    }
    argv[arg_cnt][arg_index++] = c_temp;
    c_temp = cmd_line[i++];
  }
  if(arg_cnt == 0 && arg_index == 0){  //���������߲����ǿյģ��򷵻�
    return -3;
  }
  //���һ�������Ľ���û���������whileѭ���н�����
  argv[arg_cnt++][arg_index] = 0;
  *argc = arg_cnt;//������
  return 0;
}

int cmd_exec(int argc,char *argv[]){
  int cmd_index = 0;
  uint32_t cmd_num;
  
  cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
  
  if(argc == 0){  //��������ǿյģ��򷵻�
    return -1;
  }
  for(cmd_index = 0;cmd_index < cmd_num;cmd_index++){   //��������
    if(strcmp((char *)(cmd_tbl[cmd_index].cmd_name),(char *)argv[0]) == 0){  //����ҵ��������ִ���������Ӧ�ĺ���
      uprintf("*******************************************************************************\r\n");
      cmd_tbl[cmd_index].cmd_func(argc,argv);
      memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
      return 0;
    }
  }
  return -2;
}

void cmd_help_func(int argc,char *argv[]){
  int i;
  uint32_t cmd_num;
  cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
  if(argc > 1){
    uprintf("msg:\n help�����������\r\n\r\n");      
    return;         
  }
  for(i = 0;i < cmd_num;i++){
    uprintf("cmd:%s\r\n",cmd_tbl[i].cmd_name);
    uprintf("usage:%s\r\n\r\n",cmd_tbl[i].cmd_usage);
  }
}

uint8_t compare_string(const char *s1,char * s2)
{
  int i=0;
  while(s1[i]==s2[i]&&s1[i]&&s2[i]){
    i++;
  }
  if(!s1[i]&&!s2[i]){
    return 1;
  }else{
    return 0;
  }  
}

