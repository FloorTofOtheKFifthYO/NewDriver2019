#ifndef __cmd_H
#define __cmd_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "main.h"
#include "cmd_func.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
/*
*定义添加指令的宏函数
*/
#define CMD_ADD(cmd_name,cmd_usage,cmd_func) \
    { \
        cmd_name, \
        cmd_usage, \
        cmd_func \
    } \
   
#define MAX_CMD_ARG_LENGTH 16
#define MAX_CMD_INFO_LENGTH 64
#define MAX_CMD_LINE_LENGTH 128
#define MAX_ARGC 12   //参数数量

extern char cmd_line[MAX_CMD_LINE_LENGTH + 1];
extern char *cmd_argv[MAX_ARGC];
typedef struct {
    char cmd_name[MAX_CMD_ARG_LENGTH];   //命令的名字
    char cmd_usage[MAX_CMD_INFO_LENGTH];   //命令的信息
    void (*cmd_func)(int acgc,char *argv[]); //命令执行函数
}cmd_struct;
   
void cmd_help_func(int argc,char *argv[]);   
int cmd_parse(char *cmd_line,int *argc,char *argv[]);
int cmd_exec(int argc,char *argv[]);
void cmd_init();
#ifdef __cplusplus
}
#endif
#endif /*__ cmd_H */
