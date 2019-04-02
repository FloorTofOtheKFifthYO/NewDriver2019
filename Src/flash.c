#include "flash.h"
#include "usart.h"
#include "motor.h"
#include "encoder.h"
#include "utils.h"


//如果不成功，就有可能是FLASH的大小和地址设置问题。

#define FLASH_Start 0x08020000   

//#define FLASH_Start 0x08000000 //  注意每次都要查一下手册的flash起始地址


//对FLASH写入数据：
//1.解锁FLASH
//2.擦除FLASH
//3.写入数据到FLASH
//4.锁住FLASH
//FLASH读取数据：直接读取相应的FLASH地址即可

float flash_data[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};//烧进flash中的13个数据


//将参数写进flash中
void write_prams()
{  
  uint32_t SectorError;
  uint32_t temp;
  int i;
  FLASH_EraseInitTypeDef EraseInitStruct;
  
  HAL_FLASH_Unlock();//解锁flash
  
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;//选择页擦除还是块擦除，这里是页擦除
  EraseInitStruct.PageAddress = FLASH_Start;//擦除的起始地址
  EraseInitStruct.NbPages = 1; //擦除的页数
  
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)//调用擦除函数
  { 
    uprintf("erase flash fail!\r\n");
    HAL_FLASH_Lock();
    return ;
  }
  int pram_num=sizeof(flash_data)/sizeof(flash_data[0]);
  
  for(i=0;i<pram_num;++i){
    temp=*((uint32_t *)(flash_data+i));//将flash_data[i]对应的float类型的4字节数据以无符号整型读出。
    //flash_data是flash_data[0]地址,flash_data+i是flash_data[i]地址,然后将flash_data[i]对应的float型转化为无符号整型,再取这个指针指向的地址里的值
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_Start+i*4,temp);  //对flash烧写
    uprintf("write Pram[%d]Ok!\r\n",i);
  }
  HAL_FLASH_Lock();//锁住flash
  uprintf("Write OK!\r\n");
}

void load_prams()
{
  int i;
  int pram_num=sizeof(flash_data)/sizeof(flash_data[0]);
  
  for(i=0;i<pram_num;++i){
    flash_data[i]=*((float *)(FLASH_Start+i*4));
    uprintf("flash_data[%d]=%lf\r\n",i,flash_data[i]);
  }	
  uprintf("\r\n");
  ID = (int)flash_data[0];
  Motor=(int)flash_data[1];
  Encoder=(int)flash_data[2];
  //Control_Mode=flash_data[3];
  Current_PID.KP=flash_data[4];
  Current_PID.KI=flash_data[5];
  Current_PID.KD=flash_data[6];
  Speed_PID.KP=flash_data[7];
  Speed_PID.KI=flash_data[8];
  Speed_PID.KD=flash_data[9];
  Position_PID.KP=flash_data[10];
  Position_PID.KI=flash_data[11];
  Position_PID.KD=flash_data[12];
  MAXSPEED=flash_data[13];
}


