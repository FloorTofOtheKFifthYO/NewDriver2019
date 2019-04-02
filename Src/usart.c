/**
******************************************************************************
* File Name          : USART.c
* Description        : This file provides code for the configuration
*                      of the USART instances.
******************************************************************************
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether 
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
*
* COPYRIGHT(c) 2019 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include <stdarg.h>
#include "cmd.h"
#include <string.h>

uint8_t c_recv; 
int DMA_RxOK_Flag=0;
uint8_t aRxBuffer[RXBUFFERSIZE];           //hal库使用串口接收缓冲
uint8_t DMAaRxBuffer[99];
char DMAUSART_RX_BUF[99];
char USART_RX_BUF[USART_REC_LEN];       //自定义接收存放的数组
uint16_t USART_RX_STA=0;                   //接收状态标志，接收到0x0d，0x0a结束	
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{
  
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */
    
    /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
    
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);
    
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    
    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);
    
    /* USER CODE BEGIN USART1_MspInit 1 */
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  
  if(uartHandle->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */
    
    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
    
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
    
    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    
    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspDeInit 1 */
    
    /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/***************串口打印函数***************/
char uart_buffer[100 + 1];
int uprintfFlag =0;
void uprintf(char *fmt, ...)
{
  uprintfFlag = 1;
  int size;
  
  va_list arg_ptr;
  
  va_start(arg_ptr, fmt);  
  
  size=vsnprintf(uart_buffer, 100 + 1, fmt, arg_ptr);
  va_end(arg_ptr);
  HAL_UART_Transmit(&huart1,(uint8_t *)uart_buffer,size,1000);
  //HAL_UART_Transmit_DMA(&huart1,(uint8_t *)uart_buffer,size);
  uprintfFlag =0;
}

/*********************串口中断回调函数*******************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
  if(huart->Instance==USART1){ 
    //USART_RX_STA的bit15为1，即2^15=32768,十六进制表示为0x8000，位与为0说明第十五位不为1，接收未完成
    if((USART_RX_STA&0x8000)==0)
    {
      //USART_RX_STA的bit14为1，即2^14=16384,十六进制表示为0X4000，位与不为0说明第十四位为1，接收到0x0d
      if((USART_RX_STA&0x4000)!=0)
      {
        if(aRxBuffer[0]!=0x0a)  USART_RX_STA=0;//接收到了0x0d但是缓冲区不是0x0a，则不符合我们的协议，接受错误重新开始
        else USART_RX_STA|=0x8000;//如果接收成功把USART_RX_STA按位或0x8000，把他的第15位置1表示接受完成
      }
      else// 未接收到0x0d
      {
        if(aRxBuffer[0]==0x0d)//缓冲区已经是0x0d的话
        {
          USART_RX_STA|=0x4000;//把USART_RX_STA的14位置1表示接收到0x0d
          USART_RX_BUF[USART_RX_STA&0x3fff]=aRxBuffer[0];
        }
        else//如果缓冲区还不是0x0d
        {
          USART_RX_BUF[USART_RX_STA&0x3fff]=aRxBuffer[0];//存储数组的索引为USART_RX_STA的0~13位，所以取出他们就位与001111111111，然后把缓冲区的值赋值给存储数组
          USART_RX_STA++;//存储一次标志加1，用于后面给出接收字符长度
          if(USART_RX_STA>(USART_REC_LEN-1))//宏定义接收最大长度USART_REC_LEN若接收的字符长度过长，则报错重新开始接收
            USART_RX_STA=0;//重新开始接收
        }       
      }                         
    }
  }
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);  
}
*/

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart){
  if(huart->Instance==USART1){
    uint8_t temp;
    
    __HAL_UART_CLEAR_IDLEFLAG(huart);   //清除函数空闲标志
    temp= huart->Instance->SR;
    temp= huart->Instance->DR;//读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
    temp = hdma_usart1_rx.Instance->CNDTR; 
    //huart->hdmarx->XferCpltCallback(huart->hdmarx); //调用DMA接受完毕后的回调函数，最主要的目的是要将串口的状态设置为Ready，否则无法开启下一次DMA
    HAL_UART_DMAStop(&huart1);      //停止本次DMA
    
    strcpy((char *)DMAUSART_RX_BUF,(char *)DMAaRxBuffer);
    if(DMAUSART_RX_BUF[0]!='\0')
      DMA_RxOK_Flag=1;
    memset(DMAaRxBuffer,0,98);
    HAL_UART_Receive_DMA(&huart1,(uint8_t *)&DMAaRxBuffer, 99);
  }
  
  
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  
  uint32_t isrflags   = READ_REG(huart->Instance->SR);//手册上有讲，清错误都要先读SR
  if((__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))!=RESET)
  {
    READ_REG(huart->Instance->DR);//PE清标志，第二步读DR
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);//清标志
  }
  if((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))!=RESET)
  {
    READ_REG(huart->Instance->DR);//FE清标志，第二步读DR
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
  }
  
  if((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))!=RESET)
  {
    READ_REG(huart->Instance->DR);//NE清标志，第二步读DR
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
  }        
  
  if((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))!=RESET)
  {
    READ_REG(huart->Instance->CR1);//ORE清标志，第二步读CR
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
  }
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
  HAL_UART_Receive_DMA(&huart1,(uint8_t *)&DMAaRxBuffer, 99);
}

void usart_init(){
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);  
}
void usart_DMA_init(){ 
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)&DMAaRxBuffer, 99);
}

void usart_exc()
{
  int cmd_argc,len;
  int erro_n;
  if(USART_RX_STA&0x8000){//检测是否接受完成  //接受完一次指令
    len=USART_RX_STA&0x3fff;//取出接收的长度
    if(len == 0){
      //HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
      USART_RX_STA=0;
      return;
    }
    erro_n = cmd_parse(USART_RX_BUF,&cmd_argc,cmd_argv);  //解析命令
    erro_n = cmd_exec(cmd_argc,cmd_argv);   //执行命令
    if(erro_n < 0){
      //打印函数执行错误信息
      uprintf("No such command%s\r\n",cmd_argv[0]);
      len = 0;
      memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
      //HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
      USART_RX_STA=0;
      return;
    }
    len = 0;
    memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
    USART_RX_STA=0;
  }
}
void usart_exc_DMA()
{
  int cmd_argc;
  int erro_n;
  if(DMA_RxOK_Flag){
    erro_n = cmd_parse((char *)DMAUSART_RX_BUF,&cmd_argc,cmd_argv);  //解析命令
    erro_n = cmd_exec(cmd_argc,cmd_argv);   //执行命令
    if(erro_n < 0){
      //打印函数执行错误信息
      uprintf("No such command%s\r\n",cmd_argv[0]);
    }
    memset(DMAUSART_RX_BUF,0,98);
    
    DMA_RxOK_Flag=0;
    
  }
}
char s[22]={'b','y',16,6};
void send_wave(float arg1,float arg2,float arg3,float arg4){
  if(uprintfFlag == 1)
    return;
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&aRxBuffer,RXBUFFERSIZE);
  //HAL_UART_Receive_DMA(&huart1,(uint8_t *)&DMAaRxBuffer, 99);
  s[2]=16;  //length
  s[3]=6;   //type
  s[20]='\r';
  s[21]='\n';
  memcpy(s+4,&arg1,sizeof(arg1));
  memcpy(s+8,&arg2,sizeof(arg1));
  memcpy(s+12,&arg3,sizeof(arg1));
  memcpy(s+16,&arg4,sizeof(arg1));
  //HAL_UART_Transmit(&huart1,(uint8_t *)s, 22,1000);   
  //HAL_UART_Transmit_IT(&huart1,(uint8_t *)s, 22);
  if(HAL_UART_Transmit_DMA(&huart1,(uint8_t *)s,22)!=HAL_OK)
  {
    uprintf("can't send\r\n");
      //Error_Handler();;
  }
}
/* USER CODE END 1 */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
