
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "cmd.h"
#include "flash.h"
#include "can_func.h"
#include "motor.h"
#include "utils.h"
#include "board.h"
#include "as5047p.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int control_flag=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  //usart_init();
  usart_DMA_init();
  TIM_Mode_init();
  cmd_init();
  Phase_Table_Init();
  load_prams();//必须在can_init之前
  can_init();
  PID_init();
  Current_init();
  uprintf("START!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   
    usart_exc_DMA();
    
    if((control_flag))
    {
      Control();
    }
    
    if(Motor==BRUSHLESS_NONSENSOR)
    {
      Mag_Position=as5047p_Get_Position();
      Now_Position=Mag_Position;
      Mag_Brushless_Mointor(Mag_Position);
    }
    
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

/* USER CODE BEGIN 4 */
int current_flag=0;
void HAL_SYSTICK_Callback(void){  
}
int clog_cnt = 1;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
  if (htim->Instance == htim6.Instance)//1000Hz
  {
    static int time_1ms_cnt;
    time_1ms_cnt++;
    if(time_1ms_cnt%5 == 0){
      Get_Current(100);//current
      Get_SP(); //speed&position
      if(control_flag==0)
      {        
        control_flag=1;
      }       
    }
    
    if(time_1ms_cnt%5 == 0){    
      if(Now_Current>4)
        current_flag++;
      else 
        current_flag=0;
      if(current_flag>4){
        Close_Phases();
        Motor_Duty = 0;
        uprintf("high current!\r\n");
      }
      if (send_wave_flag == 1)
      {
        send_wave((float)Now_Current,(float)Target_Current,(float)Current_PID.i,(float)Motor_Duty);
      }
      else if(send_wave_flag == 2)
      {
        send_wave((float)Now_Speed,(float)Target_Speed,(float)Now_Current,(float)Target_Current);
      }
      else if (send_wave_flag == 3)
      {
        send_wave((float)Now_Position,(float)Target_Position,(float)Now_Speed,(float)Target_Speed);
      }
    }
    
    
    if(time_1ms_cnt >= 65533){      
      time_1ms_cnt = 0;
    }
    
    //无刷防止卡住模块
    if(Motor==BRUSH||Board_Mode!=NORMAL)
      return;
    
    if(Motor==BRUSHLESS){
      if(Motor_Duty_Set<7)     
      {
        Motor_Duty_Set=0;
        Motor_Duty=0;
        return ;
      }
    }
    
    if(Motor_Duty_Set==0)
      return;
    
    Phase_Open_Cnt++;
    if(Phase_Open_Cnt>OPEN_TIME_MAX)
    {
      //if(setup_once_flag==0)
      //  return;
      Close_Phases();
      Phase_Open_Cnt=0;
      uprintf("close phases because too long!\r\n");
      if(Motor==BRUSHLESS)
      {
        uprintf("now position is %d\r\n",Hall_Position);
        uprintf("now duty is %f\r\n",Motor_Duty);
        uprintf("now duty set is %f\r\n",Motor_Duty_Set);
        uprintf("Now_Position is %f\r\n",Now_Position);
        uprintf("clog_cnt is %d\r\n", clog_cnt);
        if(Motor_Duty_Set!=Motor_Duty)
          Motor_Duty = Motor_Duty_Set;
        Phase_Change(Phase_Table_Using_Sensor[(Hall_Position+clog_cnt)%6],Motor_Duty);   //防止卡住，并且进行自启动
      }
      else if(Motor==BRUSHLESS_NONSENSOR)
      {
        if(Motor_Duty_Set!=Motor_Duty)
          Motor_Duty = Motor_Duty_Set;
        Phase_Change(Phase_Table_Using_Nonsensor[(Phase_Change_Cnt+clog_cnt)%6],Motor_Duty);
      }
      clog_cnt++;
      if(clog_cnt>6)
        clog_cnt =1;
      //setup_once_flag = 0;      
    }    
    
  }  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  uprintf("ERROR HANDLER!!!!\r\n");
  while(1)
  {
    
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
