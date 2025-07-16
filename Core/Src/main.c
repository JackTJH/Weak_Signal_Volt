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
#include <stdint.h>
#include <stdio.h>
#include "beep.h"
#include "ads1256.h"
#include "atk_md0350.h"
#include "my_timer.h"
#include "pannelkey.h"
#include "arm_math.h"
#include "00_j_vofa_uart.h"
#include <stdbool.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

AMP_Parameters_TypeDef AMP_Parameters = 
{
    .amp_lpf_mode = AMP_LPF_Mode_0Hz,  
    .amp_second_magnification = AMP2_Times_X1, 
    .dg408_in_channel = LNA_OUT,
};

extern ADS125X_t ads;

#define SAMPLE_RATE  15000

static uint8_t freq_calculated_flag = 0;
uint8_t need_recalculate = 0;  // 按键触发重新计算标志
uint8_t Detect_DC_Or_AC = 0; // 0: DC, 1: AC
static volatile uint8_t adc_ready_flag = 0;



float fft_output[FFT_LEN];
float fft_mag[FFT_LEN >> 1];
arm_rfft_fast_instance_f32 fft_instance;
uint16_t fft_sample_index = 0;
uint16_t fft_sample_index_control = 0;

float remove_dc(float *data, const uint16_t length)
{
  float sum = 0.0f;
  for(uint16_t i = 0; i < length; i++)
  {
    sum += data[i];
  }
  const float avg = sum / (float)length;
  for(uint16_t i = 0; i < length; i++)
  {
    data[i] -= avg;
  }
  return avg; // 返回直流分量
}

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

  HAL_UART_Transmit_IT(&huart1,"Weak_Sig_Volt\r\n",sizeof("Weak_Sig_Volt\r\n"));

  ADS1256_Init();
  AMP_Setup(&AMP_Parameters);

  arm_rfft_fast_init_f32(&fft_instance, FFT_LEN);


  lcd_init();
  MultTimer_Init();

  printf("All Initial is OK\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (adc_ready_flag) {
      adc_ready_flag = 0;
      ADS1256_Read_Data_ISR();
    }



    if(ADS1256_DATA.ReadOver)
    {
      ADS1256_DATA.ReadOver = 0;
      ADS1256_DATA.adc[0] = ADS1256_GetAdc(0);
      ADS1256_DATA.volt[0] = (int32_t)(((int64_t)ADS1256_DATA.adc[0] * 2532400) / 4194303);
      float voltage = (float)ADS1256_DATA.volt[0] / 1000.0f;
      Vofa_JustFloat_Send(&huart1, &voltage, 1);

      if (fft_sample_index_control < FFT_LEN) {
        ADS1256_DATA.volt_buf_control[fft_sample_index_control] = (float)ADS1256_DATA.volt[0] / 1000.0f;
        fft_sample_index_control++;
      }
    }

    if (fft_sample_index_control >= FFT_LEN) {

      if (!freq_calculated_flag) {
        // 找到第一个和最后一个最大值，截取完整区间
        float global_max = ADS1256_DATA.volt_buf_control[0];

        // 先找到全局最大值
        for (int i = 0; i < FFT_LEN; i++) {
          if (ADS1256_DATA.volt_buf_control[i] > global_max) {
            global_max = ADS1256_DATA.volt_buf_control[i];
          }
        }

        float threshold = global_max * 0.9f;  // 设置阈值为最大值的90%

        // 找到第一个局部最大值
        int first_max_idx = 0;
        for (int i = 1; i < FFT_LEN - 1; i++) {
          if (ADS1256_DATA.volt_buf_control[i] > ADS1256_DATA.volt_buf_control[i-1] &&
              ADS1256_DATA.volt_buf_control[i] > ADS1256_DATA.volt_buf_control[i+1] &&
              ADS1256_DATA.volt_buf_control[i] >= threshold) {
            first_max_idx = i;
            break;
          }
        }

        // 从后往前找最后一个局部最大值
        int last_max_idx = FFT_LEN - 1;
        for (int i = FFT_LEN - 2; i > first_max_idx + 10; i--) {
          if (ADS1256_DATA.volt_buf_control[i] > ADS1256_DATA.volt_buf_control[i-1] &&
              ADS1256_DATA.volt_buf_control[i] > ADS1256_DATA.volt_buf_control[i+1] &&
              ADS1256_DATA.volt_buf_control[i] >= threshold) {
            last_max_idx = i;
            break;
          }
        }

        // 截取从第一个最大值到最后一个最大值的区间
        int valid_length = last_max_idx - first_max_idx + 1;

        // 将截取的数据复制到临时数组进行FFT处理
        float temp_buffer[FFT_LEN];
        memcpy(temp_buffer, &ADS1256_DATA.volt_buf_control[first_max_idx], valid_length * sizeof(float));

        // 如果截取长度小于FFT_LEN，用零填充
        if (valid_length < FFT_LEN) {
          memset(&temp_buffer[valid_length], 0, (FFT_LEN - valid_length) * sizeof(float));
        }

        float dc_component = remove_dc(temp_buffer, valid_length);

        /************************FFT计算波形频率************************/
        arm_rfft_fast_f32(&fft_instance, temp_buffer, fft_output, 0);
        arm_cmplx_mag_f32(fft_output, fft_mag, FFT_LEN >> 1);
        float max_value;
        uint32_t max_index;
        arm_max_f32(fft_mag, FFT_LEN >> 1, &max_value, &max_index);
        float peak_frequency = (float)max_index * ((float)SAMPLE_RATE / FFT_LEN);
        lcd_printf(0,32*4,Word_Size_32,BLUE,WHITE,"Fre:%.2fHz", peak_frequency/2.0f);
        /************************FFT计算波形频率************************/



        /************************计算有效值和信噪比************************/
        if (Detect_DC_Or_AC == 0) {
          if (AMP_Parameters.dg408_in_channel == LNA_OUT || AMP_Parameters.dg408_in_channel == OUT) {
            lcd_printf(0,32*1,Word_Size_32,BLUE,WHITE,"DC->S_RMS:%.fuV", dc_component);
          }else if (AMP_Parameters.dg408_in_channel == Ele_Input) {
            lcd_printf(0,32*1,Word_Size_32,BLUE,WHITE,"DC->S_RMS:%.fpA", dc_component);
          }
        } else if (Detect_DC_Or_AC == 1) {
          float peak_amplitude = fft_mag[max_index] * 2.0f / FFT_LEN;
          float rms_value = peak_amplitude / 1.414213562f;
          if (AMP_Parameters.dg408_in_channel == LNA_OUT || AMP_Parameters.dg408_in_channel == OUT) {
            lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "AC->S_RMS:%.2fuV,Vp:%.2fuV", rms_value,rms_value*1.61);
          }else if (AMP_Parameters.dg408_in_channel == Ele_Input) {
            lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "AC->S_RMS:%.2fpA", rms_value);
          }
        } else {
          /************************计算噪声的RMS************************/
          // 使用更严格的噪声计算方法
          float noise_power = 0.0f;
          int noise_bins = 0;

          // 只计算远离主信号频率的频率bins作为噪声
          for (uint16_t i = 1; i < (FFT_LEN >> 1); i++) {
            // 排除主信号及其周围的频率bins（减少泄漏影响）
            if (abs((int)i - (int)max_index) > 3) {  // 跳过主信号周围3个bins
              noise_power += fft_mag[i] * fft_mag[i];
              noise_bins++;
            }
          }

          // 如果噪声bins太少，设置最小值
          if (noise_bins < 10) {
            noise_power = 1e-10f;
          } else {
            noise_power = noise_power / noise_bins;  // 平均噪声功率
          }

          // 计算信号RMS值
          float peak_amplitude = fft_mag[max_index] * 2.0f / FFT_LEN;
          float signal_rms = peak_amplitude / 1.414213562f;
          float noise_rms = sqrtf(noise_power) * 2.0f / FFT_LEN / 1.414213562f;


          if (AMP_Parameters.dg408_in_channel == LNA_OUT || AMP_Parameters.dg408_in_channel == OUT) {
            lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"Noise->S_RMS:%.2fuV", noise_rms);
          }else if (AMP_Parameters.dg408_in_channel == Ele_Input) {
            lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"Noise->S_RMS:%.2fpA", noise_rms);
          }
          /************************计算噪声的RMS************************/

          /************************SNR************************/
          float snr_linear = signal_rms / noise_rms;
          float snr_db = 20.0f * log10f(snr_linear);  // 使用20而不是10，因为这是幅度比
          lcd_printf(0,32*3,Word_Size_32,BLUE,WHITE,"SNR:%.1fdB", snr_db);
          /************************SNR************************/
        }
        /************************计算有效值和信噪比************************/

        freq_calculated_flag = 1;
      }

      if (need_recalculate) {
        fft_sample_index_control = 0;
        freq_calculated_flag = 0;
        need_recalculate = 0;
        memset(ADS1256_DATA.volt_buf_control, 0, sizeof(ADS1256_DATA.volt_buf_control));
      }
    }
    MultiTimer_TaskHandler();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
    MultiTimer_Update();  
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == ADS1256_DRDY_Pin)
  {
    adc_ready_flag = 1;
    // ADS1256_Read_Data_ISR_Fast();
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
