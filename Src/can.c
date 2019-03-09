/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include <assert.h>
#include "flash.h"
#include "can_func.h"
static int canlistnum = 0;
CanList canList[50];//最多能加50个can链接，可以改
CAN_FilterConfTypeDef  sFilterConfig;
static CanTxMsgTypeDef TxMessage;
static CanRxMsgTypeDef RxMessage;
int can_ID;
void Configure_Filter(void);
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_9TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */
    
  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */
    
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */
    
  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */
    
  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/****
*@brief can初始化
****/
void can_init()
{
  hcan.pTxMsg = &TxMessage;
  hcan.pRxMsg = &RxMessage;
  Configure_Filter();
  if(HAL_CAN_Receive_IT(&hcan,CAN_FIFO0)!=HAL_OK)
  {
    __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_FOV0 | CAN_IT_FMP0);
  }
  uprintf("can ready!!!\r\n\r\n");
  can_add_func();
}

//过滤器初始化
void Configure_Filter(void)
{  
  sFilterConfig.FilterNumber = 0;                   //过滤器组0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //工作在标识符屏蔽位模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//滤波器位宽为单个32位
  
  
  //sFilterConfig.FilterIdHigh =(int)flash_data[0]<<5;
  sFilterConfig.FilterIdHigh =0x0000<<5;//0x0122<<5;//(((unsigned int)0x1314<<3)&0xFFFF0000)>>16; //要过滤的ＩＤ高位
  sFilterConfig.FilterIdLow = 0x0000;//(((unsigned int)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;//要过滤的ID低位
  //sFilterConfig.FilterIdLow = (uint32_t)flash_data[0];
  sFilterConfig.FilterMaskIdHigh = 0x7ff<<5;// 0xffff;
  //sFilterConfig.FilterMaskIdHigh = 0x0000<<5;// 0xffff;
  sFilterConfig.FilterMaskIdLow = 0x0000;//0xffff;
  sFilterConfig.FilterFIFOAssignment =CAN_FILTER_FIFO0;//过滤器被关联到FIFO0；
  sFilterConfig.FilterActivation = ENABLE;//使能过滤器
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
}



/****
*@brief 用can总线发送数据
*@param ID : 发送数据的ID
*@param data : 发送的数据，长度为8
*@retval : 发送失败返回0，正常返回1
*/
int can_send_msg(uint32_t ID, uint8_t* data, uint32_t len)
{
  hcan.pTxMsg->StdId = ID;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  hcan.pTxMsg->IDE = CAN_ID_STD;
  hcan.pTxMsg->DLC = len;
  for(int i = 0; i < 8; i++)
  {
    hcan.pTxMsg->Data[i] = data[i];
  }
  
  if(HAL_CAN_Transmit(&hcan, 100) != HAL_OK) 
  {
    return 0;
  }
  return 1;
}

/****
*@brief 在can总线接收表中添加项
*@param ID : 接收数据的ID
*@param void (*func)(uint8_t data[8]) : 接收数据链接的处理函数
*@retval 返回1表示ID越界，返回0表示正常
****/
int can_add_callback(uint32_t ID, void (*func)(CanRxMsgTypeDef* pRxMsg)) 
{
  assert(ID <= 2048);
  assert(canlistnum < 50);
  canList[canlistnum].func = func;
  canList[canlistnum].ID = ID;
  canlistnum++;
  return 0;
}


/****
*@brief 将接收到的数据与can总线接收表匹配
*@param ID : 接收到数据的ID
*@param uint8_t data[8] : 接收的数据
*@retval 返回1表示匹配失败，返回0表示正常
*/
int CAN_LIST_MATCH(uint32_t ID, CanRxMsgTypeDef* pRxMsg)
{
  for(int i = 0; i < canlistnum; i++)
  {
    if(ID == canList[i].ID )
    {
      (*canList[i].func)(pRxMsg);//执行相应的函数
      return 1;
    }
  }
  return 0;
}


/****
*@brief can接收回调函数
****/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)	
{
  
  CAN_LIST_MATCH(hcan->pRxMsg->StdId, hcan->pRxMsg);
  if(HAL_CAN_Receive_IT(hcan,CAN_FIFO0)!=HAL_OK)
  {
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FOV0 | CAN_IT_FMP0);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
  hcan->Instance->MSR=0;
  if(HAL_CAN_Receive_IT(hcan,CAN_FIFO0)!=HAL_OK)
  {
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FOV0 | CAN_IT_FMP0);
  }
}



//消息ID+处理函数
void can_add_func(void)
{
  can_add_callback(0X00,callback);
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
