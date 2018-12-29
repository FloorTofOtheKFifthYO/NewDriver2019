#include "can_func.h"
#include "can.h"
#include "gpio.h"
//#include "speed.h"
#include "usart.h"
#include <stdlib.h>

can_change_msg can_RX_data;
can_change_msg can_TX_data;

void callback(CanRxMsgTypeDef* pRxMsg)
{
  int i;
  for(i=0;i<8;i++)
  {
    can_RX_data.ch[i]=pRxMsg->Data[i];
    uprintf("receive %x ",can_RX_data.ch[i]);
  }
  
  //uprintf("\r\n\r\n");
  uprintf("ID=%x    ",pRxMsg->StdId);
  uprintf("receive %lf\r\n\r\n",can_RX_data.in);
  /*
  target_angle=(float)can_RX_data.in;
  uprintf("target_angle=%f\r\n",target_angle);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);*/
}


