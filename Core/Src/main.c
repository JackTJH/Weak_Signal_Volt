/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "amp.h"
#include <stdio.h>
#include "beep.h"
#include "ads1256.h"
#include "atk_md0350.h"
#include "my_timer.h"
#include "pannelkey.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

AMP_Parameters_TypeDef AMP_Parameters = 
{
    .amp_lpf_mode = AMP_LPF_Mode_0Hz,  
    .amp_second_magnification = AMP2_Times_X1, 
    .dg408_in_channel = OUT, 
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */

  AMP_Setup(&AMP_Parameters); 
  if(ADS1256_Init())
  {
    // BEEP_Short();
  }

  lcd_init();

  HAL_TIM_Base_Start_IT(&htim6);
  printf("All Initial is OK\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    switch (Global_Key_Val) 
    {
      case Key_Val_K13:
        AMP_Parameters.dg408_in_channel = OUT; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_       OUT");
      break;
      case Key_Val_K14:
        AMP_Parameters.dg408_in_channel = LNA_OUT; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_   LNA_OUT");
      break;
      case Key_Val_K15:
        AMP_Parameters.dg408_in_channel = VREF; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_      VREF");
      break;
      case Key_Val_K16:
        AMP_Parameters.dg408_in_channel = VREF_700mV; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_VREF_700mV");
      break;
      case Key_Val_K9:
        AMP_Parameters.dg408_in_channel = VREF_70mV; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_ VREF_70mV");
      break;
      case Key_Val_K10:
        AMP_Parameters.dg408_in_channel = VREF_9mV; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_  VREF_9mV");
      break;
      case Key_Val_K11:
        AMP_Parameters.dg408_in_channel = AGND; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_      AGND");
      break;
      case Key_Val_K12:
        AMP_Parameters.dg408_in_channel = Ele_Input; 
        lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_ Ele_Input");
      break;
      case Key_Val_K5:
        AMP_Parameters.amp_second_magnification = AMP2_Times_X1; 
        lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"AMP_Second_Magnification_X1  ");
      break;
      case Key_Val_K6:
        AMP_Parameters.amp_second_magnification = AMP2_Times_X10; 
        lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"AMP_Second_Magnification_X10 ");
      break;
      case Key_Val_K7:
        AMP_Parameters.amp_second_magnification = AMP2_Times_X100; 
        lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"AMP_Second_Magnification_X100");
      break;
      case Key_Val_K1:
        AMP_Parameters.amp_lpf_mode = AMP_LPF_Mode_0Hz; 
        lcd_printf(0,32*3,Word_Size_32,BLUE,WHITE,"AMP_LPF_Mode_  0Hz");
      break;
      case Key_Val_K2:
        AMP_Parameters.amp_lpf_mode = AMP_LPF_Mode_35Hz; 
        lcd_printf(0,32*3,Word_Size_32,BLUE,WHITE,"AMP_LPF_Mode_ 35Hz");
      break;
      case Key_Val_K3:
        AMP_Parameters.amp_lpf_mode = AMP_LPF_Mode_100Hz; 
        lcd_printf(0,32*3,Word_Size_32,BLUE,WHITE,"AMP_LPF_Mode_100Hz");
      default: break;
    }
    Set_DG408_IN(AMP_Parameters.dg408_in_channel);  
    Set_AMP_Second_Magnification(AMP_Parameters.amp_second_magnification);
    Set_AMP_LPF(AMP_Parameters.amp_lpf_mode);  
    ADS1256_Read_Data_ISR();


    if(TimeBaseGetFlag(BASE_10MS))
    {
      Key_Detect();
    }
    if(TimeBaseGetFlag(BASE_200MS))
    {
      TimeBaseClearFlag(BASE_200MS);
      lcd_printf(0,0,Word_Size_32,BLUE,WHITE,"ADS1256_Voltage:%.5lfmV",ADS1256_DATA.Voltage*1000);
    }
    if(TimeBaseGetFlag(BASE_1000MS))
    {
      TimeBaseClearFlag(BASE_1000MS);  
      HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);  
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6)
  {
    TimeBaseUpdata();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
