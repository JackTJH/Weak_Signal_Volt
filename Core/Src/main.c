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
#include "data_process.h"


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



static uint8_t freq_calculated_flag = 0;
uint8_t need_recalculate = 0;  // 按键触发重新计算标志
static volatile uint8_t adc_ready_flag = 0;

uint16_t fft_sample_index = 0;
uint16_t fft_sample_index_control = 0;

//FFT计算变量
static FFT_Processor_TypeDef fft_processor;

//选择检测模式变量
Detect_Mode_TypeDef Detect_DC_Or_AC = DETECT_DC; // 初始化为直流模式

DisplayMode_e current_display_mode = MODE_PC;

//rms计算变量
static RMS_Average_TypeDef rms_calculator;

/*******************************直流校准全局变量*******************************/
static uint8_t calibration_done = 0;  // 校准完成标志
static float calibration_offset = 0.0f;  // 校准偏置值
static uint8_t I_calibration_done = 0;  // 校准完成标志
static float I_calibration_offset = 0.0f;  // 校准偏置值
static const float target_value = 20.0f;  // 目标校准值20

//直流电压自交准系数
static DC_Calibration_TypeDef voltage_dc_cal;
static DC_Calibration_TypeDef current_dc_cal;

//交流电压自交准系数
static Frequency_Calibration_TypeDef freq_40hz_cal_V;
static Frequency_Calibration_TypeDef freq_80hz_cal_V;

//交流电流自交准系数
static Frequency_Calibration_TypeDef freq_40hz_cal_I;
static Frequency_Calibration_TypeDef freq_80hz_cal_I;

// 噪声计算变量
static Noise_Calculator_TypeDef noise_calculator;

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

  FFT_Processor_Init(&fft_processor, SAMPLE_RATE);
  RMS_Average_Init(&rms_calculator);
  DC_Calibration_Init(&voltage_dc_cal, 20.0f, 1.0f);          // 电压通道，系数为1
  DC_Calibration_Init(&current_dc_cal, 20.0f, DC_I_COEFFICIENT); // 电流通道，使用DC_I_COEFFICIENT
  Frequency_Calibration_Init(&freq_40hz_cal_V, RMS_SELF_CAL);
  Frequency_Calibration_Init(&freq_80hz_cal_V, RMS_SELF_CAL);
  Frequency_Calibration_Init(&freq_40hz_cal_I, RMS_SELF_CAL);
  Frequency_Calibration_Init(&freq_80hz_cal_I, RMS_SELF_CAL);
  Noise_Calculator_Init(&noise_calculator);


  lcd_init();

  if(current_display_mode == MODE_ARM)
    lcd_printf(0, 32 * 5, Word_Size_32, BLUE, WHITE, "MODE->ARM");
  else if(current_display_mode == MODE_PC)
    lcd_printf(0, 32 * 5, Word_Size_32, BLUE, WHITE, "MODE->PC ");


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
      ADS1256_DATA.volt[0] = (int32_t)(((int64_t)ADS1256_DATA.adc[0] * 2546800) / 4194303);
      float voltage = (float)ADS1256_DATA.volt[0] / 1000.0f;
      if (current_display_mode == MODE_PC)
        Vofa_JustFloat_Send(&huart1, &voltage, 1);

      if (fft_sample_index_control < FFT_LEN) {
        ADS1256_DATA.volt_buf_control[fft_sample_index_control] = (float)ADS1256_DATA.volt[0] / 1000.0f;
        fft_sample_index_control++;
      }
    }

    if (fft_sample_index_control >= FFT_LEN) {

      if (!freq_calculated_flag) {
        /*****************************截取一段完整的周期*****************************/
        float temp_buffer_fft[FFT_LEN];
        float temp_buffer_rms[FFT_LEN];  // 用于后续处理的副本
        float temp_buffer_noise[FFT_LEN];  // 用于后续处理的副本
        float dc_component = signal_preprocess(ADS1256_DATA.volt_buf_control, FFT_LEN, temp_buffer_fft, FFT_LEN);
        // 复制数据用于后续处理
        copy_float_array(temp_buffer_fft, temp_buffer_rms, FFT_LEN);
        copy_float_array(temp_buffer_fft, temp_buffer_noise, FFT_LEN);
        /*****************************截取一段完整的周期*****************************/

        /*****************************调试时使用打印出处理的波形数据*****************************/
        if (current_display_mode == MODE_ARM) {
          for (int i = 0; i < FFT_LEN; i++) {
            Vofa_JustFloat_Send(&huart1, &temp_buffer_fft[i], 1);
          }
        }

        /*****************************调试时使用打印出处理的波形数据*****************************/

        /************************FFT计算波形频率************************/
        float peak_frequency = FFT_Calculate_Peak_Frequency(&fft_processor, temp_buffer_fft);
        lcd_printf(0,32*4,Word_Size_32,BLUE,WHITE,"Fre:%.2fHz", peak_frequency/2.0f);
        /************************FFT计算波形频率************************/



        /************************直流有效值计算************************/
        if (Detect_DC_Or_AC == DETECT_DC) {
          if (AMP_Parameters.dg408_in_channel == LNA_OUT) {
              float calibrated_dc = Process_DC_Calibration(&voltage_dc_cal, dc_component);
              lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "DC->S_RMS:%.3fuV", calibrated_dc);
              /***************************噪声计算***************************/
              // 对于直流信号，噪声主要来自电路的本底噪声和干扰
              // 由于没有特定频率需要去除，直接计算原始信号的波动作为噪声
              Calculate_DC_Noise(&noise_calculator, temp_buffer_noise, FFT_LEN, dc_component);
              lcd_printf(0, 32 * 2, Word_Size_32, BLUE, WHITE, "DC->Noise_RMS:%.3fuV", noise_calculator.averaged_noise);
              /***************************噪声计算***************************/
              /***************************计算信号和噪声的比值***************************/
              float snr_ratio = calibrated_dc / noise_calculator.averaged_noise;
              lcd_printf(0, 32 * 3, Word_Size_32, BLUE, WHITE, "DC->SNR:%.3f", snr_ratio);
              /***************************计算信号和噪声的比值***************************/


          }
          else if (AMP_Parameters.dg408_in_channel == Ele_Input) {
              float calibrated_dc = Process_DC_Calibration(&current_dc_cal, dc_component);
              lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "DC->S_RMS:%.3fpA", calibrated_dc);
              /***************************噪声计算***************************/
              // 对于直流信号，噪声主要来自电路的本底噪声和干扰
              // 由于没有特定频率需要去除，直接计算原始信号的波动作为噪声
              Calculate_DC_Noise(&noise_calculator, temp_buffer_noise, FFT_LEN, dc_component);
              lcd_printf(0, 32 * 2, Word_Size_32, BLUE, WHITE, "DC->Noise_RMS:%.3fpA", noise_calculator.averaged_noise);
              /***************************噪声计算***************************/
              /***************************计算信号和噪声的比值***************************/
              float snr_ratio = calibrated_dc / noise_calculator.averaged_noise;
              lcd_printf(0, 32 * 3, Word_Size_32, BLUE, WHITE, "DC->SNR:%.3f", snr_ratio);
              /***************************计算信号和噪声的比值***************************/
          }
        }
        /************************直流有效值计算************************/
        /************************交流有效值计算************************/
        else if (Detect_DC_Or_AC == DETECT_AC) {
          /************************平均交流有效值计算************************/
          float rms_value = RMS_Average_Update(&rms_calculator, temp_buffer_rms, FFT_LEN);
          /************************平均交流有效值计算************************/

          if (AMP_Parameters.dg408_in_channel == LNA_OUT) {
            float signal_rms = 0.0f;  // 在此处定义 signal_rms
            /***************************处理40Hz频率校准***************************/
            float calibrated_40hz = Process_Frequency_Calibration(&freq_40hz_cal_V, &rms_calculator,
                                                                 peak_frequency/2.0f, 40.0f,
                                                                 rms_value, 1);
            if (calibrated_40hz > 0.0f) {
              signal_rms = calibrated_40hz / 1.141f;  // 计算真实信号RMS
              lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "AC->S_RMS:%.3fuV,Vp:%.3fuV",
                        rms_value, calibrated_40hz);
            }
            /***************************处理40Hz频率校准***************************/

            /***************************处理80Hz频率校准***************************/
            float calibrated_80hz = Process_Frequency_Calibration(&freq_80hz_cal_V, &rms_calculator,
                                                                 peak_frequency/2.0f, 80.0f,
                                                                 rms_value, 2);

            if (calibrated_80hz > 0.0f) {
              signal_rms = calibrated_80hz / 1.141f;  // 计算真实信号RMS
              lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "AC->S_RMS:%.3fuV,Vp:%.3fuV",
                        rms_value, calibrated_80hz);
            }
            /***************************处理80Hz频率校准***************************/

            /***************************噪声计算***************************/
            Calculate_Noise_After_Signal_Removal(&noise_calculator,
                                                                    temp_buffer_noise,
                                                                    peak_frequency/2.0f,
                                                                    FFT_LEN);
            lcd_printf(0, 32 * 2, Word_Size_32, BLUE, WHITE, "AC->Noise_RMS:%.3fuV", noise_calculator.averaged_noise);
            lcd_printf(0, 32 * 0, Word_Size_32, BLUE, WHITE, "AC->Real_Sig_RMS:%.3fuV", signal_rms);
            /***************************噪声计算***************************/

            /***************************计算信号和噪声的比值***************************/
            float snr_ratio = 0.0f;
            if (noise_calculator.averaged_noise > 0.0f && signal_rms > 0.0f) {
                // 将信号和噪声值截断为小数点后两位
                float truncated_signal = floorf(signal_rms * 100.0f) / 100.0f;
                float truncated_noise = floorf(noise_calculator.averaged_noise * 100.0f) / 100.0f;

                if (truncated_noise > 0.0f) {
                    snr_ratio = truncated_signal / truncated_noise;
                } else {
                    snr_ratio = 0.0f;
                }
            } else {
                snr_ratio = 0.0f;  // 避免除零错误
            }
            lcd_printf(0, 32 * 3, Word_Size_32, BLUE, WHITE, "AC->SNR:%.3f", snr_ratio);
            /***************************计算信号和噪声的比值***************************/




          }
          else if (AMP_Parameters.dg408_in_channel == Ele_Input) {

            float signal_rms = 0.0f;  // 在此处定义 signal_rms

            /***************************处理40Hz频率校准***************************/
            float calibrated_40hz = Process_Frequency_Calibration(&freq_40hz_cal_I, &rms_calculator,
                                                                 peak_frequency/2.0f, 40.0f,
                                                                 rms_value, 1);
            if (calibrated_40hz > 0.0f) {
              signal_rms = calibrated_40hz / 1.141f;  // 计算真实信号RMS
              lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "AC->S_RMS:%.3fpA,Vp:%.3fpA",
                        rms_value, calibrated_40hz);
            }
            /***************************处理40Hz频率校准***************************/

            /***************************处理80Hz频率校准***************************/
            float calibrated_80hz = Process_Frequency_Calibration(&freq_80hz_cal_I, &rms_calculator,
                                                                 peak_frequency/2.0f, 80.0f,
                                                                 rms_value, 2);

            if (calibrated_80hz > 0.0f) {
              signal_rms = calibrated_80hz / 1.141f;  // 计算真实信号RMS
              lcd_printf(0, 32 * 1, Word_Size_32, BLUE, WHITE, "AC->S_RMS:%.3fpA,Vp:%.3fpA",
                        rms_value, calibrated_80hz);
            }
            /***************************处理80Hz频率校准***************************/

            /***************************噪声计算***************************/
            Calculate_Noise_After_Signal_Removal(&noise_calculator,
                                                                    temp_buffer_noise,
                                                                    peak_frequency/2.0f,
                                                                    FFT_LEN);

            lcd_printf(0, 32 * 2, Word_Size_32, BLUE, WHITE, "AC->Noise_RMS:%.3fpA", noise_calculator.averaged_noise);
            lcd_printf(0, 32 * 0, Word_Size_32, BLUE, WHITE, "AC->Real_Sig_RMS:%.3fpA", signal_rms);
            /***************************噪声计算***************************/


           /***************************计算信号和噪声的比值***************************/
            float snr_ratio = 0.0f;
            if (noise_calculator.averaged_noise > 0.0f && signal_rms > 0.0f) {
                // 将信号和噪声值截断为小数点后两位
                float truncated_signal = floorf(signal_rms * 100.0f) / 100.0f;
                float truncated_noise = floorf(noise_calculator.averaged_noise * 100.0f) / 100.0f;

                if (truncated_noise > 0.0f) {
                    snr_ratio = truncated_signal / truncated_noise;
                } else {
                    snr_ratio = 0.0f;
                }
            } else {
                snr_ratio = 0.0f;  // 避免除零错误
            }
            lcd_printf(0, 32 * 3, Word_Size_32, BLUE, WHITE, "AC->SNR:%.3f", snr_ratio);
            /***************************计算信号和噪声的比值***************************/


          }
        }
        /************************交流有效值计算************************/



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
