#include "as5047p.h"
#include "spi.h"
#include "usart.h"

uint8_t sendBuffer[2]={0xff,0xff};

static void as5047p_send_message(uint8_t *sendBuffer)
{
  as5047p_Enable();
  if(HAL_SPI_Transmit(&hspi2, sendBuffer, 2, SPI2_TIMEOUT_VALUE)!=HAL_OK)
  {
    uprintf("spi_transmit_error!\r\n");
  }
  as5047p_Disable();
}
           
static void as5047p_receive_message(uint8_t *receiveBuffer)
{
  as5047p_Enable();
  HAL_SPI_Receive(&hspi2, receiveBuffer, 2, SPI2_TIMEOUT_VALUE);
  as5047p_Disable();
}           


uint16_t as5047p_read_pos()
{
    uint8_t receiveBuffer[2]={1,1};  
    as5047p_send_message(sendBuffer);
    as5047p_receive_message(receiveBuffer); 
    uint16_t pos=(uint16_t)((receiveBuffer[0]&0x3f)*256+receiveBuffer[1]);
    return pos;
}