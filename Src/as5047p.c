#include "as5047p.h"
#include "spi.h"
#include "usart.h"

#define SPI_USE hspi2
#define Set_CSN(x)      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,(GPIO_PinState)x);
//extern void Delay_Us(uint32_t nus);

static uint16_t Count_PARC(uint16_t data);
static uint16_t Command(uint16_t address,uint8_t read);
static uint16_t Write_frame(uint16_t data);

uint16_t Read_Reg(uint16_t reg){
  uint16_t command=0;
  uint16_t result=0;
  uint16_t err;
  //uint8_t result[2]={0};
  //uint8_t command[2]={0x7F,0xFE};
  
  command=Command(0x01,1);
  Set_CSN(0);
  HAL_SPI_Transmit(&SPI_USE,(uint8_t *)&command,1,100);
  Set_CSN(1);
  
  command=Command(reg,1);
  Set_CSN(0);
  HAL_SPI_TransmitReceive(&SPI_USE,(uint8_t *)&command,(uint8_t *)&err,1,100);
  Set_CSN(1);
  
  Set_CSN(0);
  HAL_SPI_Receive(&SPI_USE,(uint8_t *)&result,1,100);
  Set_CSN(1);
  return result;
}

uint16_t Write_Reg(uint16_t reg,uint16_t data){
  uint16_t command=0;
  uint16_t err;
  uint16_t frame;
  
  command=Command(0x01,1);//´íÎó¼Ä´æÆ÷
  Set_CSN(0);
  HAL_SPI_Transmit(&SPI_USE,(uint8_t *)&command,1,100);
  Set_CSN(1);
  
  command=Command(reg,0);//Ð´µÄµØÖ·
  Set_CSN(0);
  HAL_SPI_TransmitReceive(&SPI_USE,(uint8_t *)&command,(uint8_t *)&err,1,100);
  Set_CSN(1);
  
  Set_CSN(0);//Ð´µÄÄÚÈÝ
  frame=Write_frame(data);//Ð´µÄÄÚÈÝ
  HAL_SPI_Transmit(&SPI_USE,(uint8_t *)&frame,1,100);
  Set_CSN(1);
  return err;
}

uint16_t as5047p_Get_Position(){
  uint16_t raw_data=0;
  uint16_t position=0;
  raw_data=Read_Reg(0x3FFF);
  if(raw_data&(1<<14)){
    return 0xFFFF;
  }
  position=raw_data&0x3FFF;
  return position;
}

uint16_t as5047p_Write_Position(uint16_t data){
  uint16_t raw_data=0;
  uint16_t position=0;
  raw_data=Write_Reg(0x0003,data);
  if(raw_data&(1<<14)){
    return 0xFFFF;
  }
  position=raw_data&0x3fff;
  return position;
}


static uint16_t Count_PARC(uint16_t data){
  // ?§µ?
  int cnt=0;
  uint16_t temp=0;
  for(int i=0;i<15;++i){
    temp=(1<<i);
    if(data&temp){
      cnt++;
    }
  }
  if(cnt%2==0){
    return 0;
  }else{
    return temp<<1;
  }
}

static uint16_t Command(uint16_t address,uint8_t read){
  uint16_t command=0;
  command|=address;
  command|=(read<<14);
  command|=Count_PARC(command);
  return command;
}

static uint16_t Write_frame(uint16_t data){
  uint16_t frame=0;
  frame|=data;
  frame|=(0<<14);
  frame|=Count_PARC(frame);
  return frame;
}
