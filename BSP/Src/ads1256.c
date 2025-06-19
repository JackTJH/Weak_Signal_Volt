#include "ads1256.h"
#include "main.h"
#include "delay.h"
#include <stdio.h>
#include <time.h>


ADS125X_t ads = 
{
  .csPort    = ADS1256_CS_GPIO_Port,
  .csPin     = ADS1256_CS_Pin,
  .drdyPort  = ADS1256_DRDY_GPIO_Port,
  .drdyPin   = ADS1256_DRDY_Pin,
  .resetPort = ADS1256_RESET_GPIO_Port,
  .resetPin  = ADS1256_RESET_Pin,
  .hspix     = &hspi1
};

ADS1256_InitParams_t ADS1256_InitParams = 
{
  .DataOutputBitOrder                   = MSB_FRIST,                      // 数据输出位序，MSB方式输出数据
  .AutoCalibration                      = ACAL_ON,                        // 自校准功能打开
  .AnalogInputBufferEnable              = BUFEN_ON,                       // 输入缓冲器打开    
  .InputMultiplexerControl              = POSITIVE_AIN0|NEGTIVE_AIN1,     // 输入通道选择，AINCOM作为负输入端，单端输入模式      
  .ClockOutRateSetting                  = CLKOUT_OFF,                     // 输出时钟关闭       
  .SensorDetectCurrentSources           = DETECT_OFF,                     // 外部电路检测关闭  
  .ProgrammableGainAmplifierSetting     = PGA_2,                          // 可编程外部增益放大器设置为1倍增益
  .DataRateSetting                      = DRATE_1KHz,                     // 设置数据输出速率为100Hz
  // .VREF                                 = 2.5360                          // 参考电压设置为2.5360V实际电压表测出来的值（这个参考电压算出来值与实际贴切有差距）
  .VREF                                 = 2.5166                          // 参考电压设置为2.5166V（这个参考电压算出来值与实际贴切）      
};

void CS_1(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->csPort,ads->csPin ,GPIO_PIN_SET);
}
void CS_0(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->csPort,ads->csPin ,GPIO_PIN_RESET);
}

void RST_1(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->resetPort,ads->resetPin,GPIO_PIN_SET);
}

void RST_0(ADS125X_t *ads)
{
  HAL_GPIO_WritePin(ads->resetPort,ads->resetPin,GPIO_PIN_RESET);
}

void waitDRDY(ADS125X_t *ads)
{
  while(HAL_GPIO_ReadPin(ads->drdyPort, ads->drdyPin) == GPIO_PIN_SET);
}

void hardRESET(ADS125X_t *ads)
{
  RST_0(ads);
  delay_us(10); 
  RST_1(ads); 
  delay_us(10); 
  waitDRDY(ads); 
}

void ADS1256_Write_Bit(uint8_t data,ADS125X_t *ads)
{
    HAL_SPI_Transmit(ads->hspix, &data, 1, HAL_MAX_DELAY);
}

void ADS1256_Write_Reg(uint8_t reg_addr,uint8_t *Write_Buf,uint8_t len,ADS125X_t *ads)
{
    ADS1256_Write_CS_L;
    ADS1256_Write_Bit(CMD_WREG|reg_addr,ads);
    ADS1256_Write_Bit(len-1,ads);
    for(uint8_t i = 0;i<len;i++)
    {
        ADS1256_Write_Bit(Write_Buf[i],ads);
    }
    ADS1256_Write_CS_H;
}

void ADS1256_Read_Bit(uint8_t *data,ADS125X_t *ads)
{
    HAL_SPI_Receive(ads->hspix, data, 1, HAL_MAX_DELAY);
}

void ADS1256_Read_Reg(uint8_t reg_addr,uint8_t *Read_Buf,uint8_t len,ADS125X_t *ads)
{
    ADS1256_Write_CS_L;

    ADS1256_Write_Bit(CMD_RREG|reg_addr,ads);
    ADS1256_Write_Bit(len-1,ads);
    delay_us(10);

    // HAL_SPI_Receive(&hspi1,Read_Buf,len,HAL_MAX_DELAY);
    for(uint8_t i = 0;i<len;i++)
    {
        // ADS1256_Read_Bit(&Read_Buf[i]);
        ADS1256_Read_Bit(Read_Buf++,ads);
    }
    ADS1256_Write_CS_H;
}

void writeCMD(uint8_t command,ADS125X_t *ads)
{
	CS_0(ads);
	ADS1256_Write_Bit(command,ads);
	CS_1(ads);
}

void softReset(ADS125X_t *ads)
{
  writeCMD(CMD_RESET,ads);
  delay_us(10); 
  waitDRDY(ads); // 等待数据准备好
}

void setDIFFChannel(uint8_t positiveCh, uint8_t negativeCh,ADS125X_t *ads)
{
  uint8_t muxValue = (positiveCh << 4) | negativeCh;
  ADS1256_Write_Reg(REG_MUX,&muxValue,1,ads);
}

int32_t ADS1256_ReadAdcData_Original(uint8_t positiveCh, uint8_t negativeCh,ADS125X_t *ads)
{
    int32_t sum;
    uint8_t data[3];

    setDIFFChannel(positiveCh, negativeCh,ads);
    writeCMD(CMD_RDATAC, ads); // Start read data continuous.
    delay_us(7); 
    
    CS_0(ads);
    HAL_SPI_Receive(ads->hspix, data, 3, 10);
    sum = (unsigned int)(data[0]<<16)|(data[1]<<8)|data[2];
	  sum |= (sum & 0x800000) ? 0xFF000000 : 0;
    CS_1(ads);

    return sum; // 返回原始数据
}

//这个使用不了莫名其妙会死机
int32_t ADS1256_ReadAdcData_Original_Sig(uint8_t positiveCh, uint8_t negativeCh,ADS125X_t *ads)
{
    int32_t sum;
    uint8_t data[3];

    setDIFFChannel(positiveCh, negativeCh,ads);
    writeCMD(CMD_SYNC,ads);
    while(ADS1256_Read_DRDY != GPIO_PIN_SET);
    writeCMD(CMD_WAKEUP,ads);
    writeCMD(CMD_RDATA, ads); 
    delay_us(10);
    CS_0(ads);
    HAL_SPI_Receive(ads->hspix, data, 3, 1000);
    CS_1(ads);

    sum = (unsigned int)(data[0]<<16)|(data[1]<<8)|data[2];
	  sum |= (sum & 0x800000) ? 0xFF000000 : 0;

    return sum; // 返回原始数据
}

double ADS1256_ADCDataConvert(int32_t Data, double Vref, uint8_t Gain)
{
    double ReadVoltage;
    
    // ADS1256电压转换公式
    // Vin = (2 * Vref / Gain) * (ADC_Code / 2^23)
    // 其中 2^23 = 8388608 (24位ADC的满量程)
    ReadVoltage = (2.0 * Vref / Gain) * ((double)Data / 8388608.0);
    
    return ReadVoltage;
}

uint8_t ADS1256_Init(void) 
{
  uint8_t write_Reg[4] = {0};
  uint8_t read_Reg[4] = {0};

  // STATUS寄存器设置(0x36)
  write_Reg[REG_STATUS] =   ADS1256_InitParams.DataOutputBitOrder | 
                            ADS1256_InitParams.AutoCalibration | 
                            ADS1256_InitParams.AnalogInputBufferEnable;          
  // MUX寄存器设置(0x08)
  write_Reg[REG_MUX]    =   ADS1256_InitParams.InputMultiplexerControl;          
  // ADCON寄存器设置(0x00)
  write_Reg[REG_ADCON]  =   ADS1256_InitParams.ClockOutRateSetting | 
                            ADS1256_InitParams.SensorDetectCurrentSources | 
                            ADS1256_InitParams.ProgrammableGainAmplifierSetting;  
  // DRATE寄存器设置(0xE0)
  write_Reg[REG_DRATE]  =   ADS1256_InitParams.DataRateSetting;                   

  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  // hardRESET(&ads); // 硬件复位ADS1256
  softReset(&ads); // 软复位ADS1256
  ADS1256_Read_Reg(REG_STATUS,read_Reg,sizeof(read_Reg),&ads);
  printf("Before_Wreg: %X %X %X %X\r\n", read_Reg[0], read_Reg[1], read_Reg[2], read_Reg[3]);


  ADS1256_Write_Reg(REG_STATUS,write_Reg,sizeof(write_Reg),&ads);
  waitDRDY(&ads); // 等待数据准备好

  ADS1256_Read_Reg(REG_STATUS,read_Reg,sizeof(read_Reg),&ads);
  printf("After_Wreg: %X %X %X %X\r\n", read_Reg[0], read_Reg[1], read_Reg[2], read_Reg[3]);

  if(read_Reg[0] == 0x36)
  {
      printf("ADS1256 Init OK\r\n");
      return 1;
  }
  else
  {
      printf("ADS1256 Init Failed\r\n");
      return 0; // 初始化失败
  }
  
  waitDRDY(&ads); // 等待数据准备好
  writeCMD(CMD_SELFCAL, &ads); // 发送复位命令
  waitDRDY(&ads); // 等待数据准备好

  // HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

// 滤波相关变量
#define FILTER_WINDOW_SIZE 5
// 中值滤波函数
uint32_t ADS1256_MedianFilter(uint32_t new_data)
{
  static uint32_t filter_buffer[FILTER_WINDOW_SIZE] = {0};
  static uint8_t filter_index = 0;
  static uint8_t filter_filled = 0;

  // 将新数据存入缓冲区
  filter_buffer[filter_index] = new_data;
  filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE;
  
  if(!filter_filled && filter_index == 0)
  {
      filter_filled = 1;
  }
  
  // 如果缓冲区未满，直接返回新数据
  if(!filter_filled)
  {
      return new_data;
  }
  
  // 复制缓冲区数据进行排序
  uint32_t temp_array[FILTER_WINDOW_SIZE];
  for(int i = 0; i < FILTER_WINDOW_SIZE; i++)
  {
      temp_array[i] = filter_buffer[i];
  }
  
  // 简单冒泡排序
  for(int i = 0; i < FILTER_WINDOW_SIZE - 1; i++)
  {
      for(int j = 0; j < FILTER_WINDOW_SIZE - 1 - i; j++)
      {
          if(temp_array[j] > temp_array[j + 1])
          {
              uint32_t temp = temp_array[j];
              temp_array[j] = temp_array[j + 1];
              temp_array[j + 1] = temp;
          }
      }
  }
  
  // 返回中值
  return temp_array[FILTER_WINDOW_SIZE / 2];
}


ADS1256_DATA_t ADS1256_DATA = {0};

void ADS1256_Read_Data_ISR(void)
{
  if(ADS1256_Read_DRDY == GPIO_PIN_RESET) // 检查DRDY引脚状态
  {
      uint32_t raw_data = ADS1256_ReadAdcData_Original(AIN0,AIN1,&ads); // 读取原始数据
      
      // 应用中值滤波
      ADS1256_DATA.OriginalData = ADS1256_MedianFilter(raw_data);
      
      ADS1256_DATA.Voltage = ADS1256_ADCDataConvert(
              ADS1256_DATA.OriginalData,
              ADS1256_InitParams.VREF,  // 参考电压 
              1<<ADS1256_InitParams.ProgrammableGainAmplifierSetting// PGA增益倍数 (PGA_1 = 1倍增益)
      );
      printf("ADS1256_Ori_data,ADS1256_Voltage:%d,%lf\r\n", ADS1256_DATA.OriginalData,ADS1256_DATA.Voltage);
  }

  //不滤波的方式

  // if(ADS1256_Read_DRDY == GPIO_PIN_RESET) // 检查DRDY引脚状态
  // {
  //     ADS1256_DATA.OriginalData = ADS1256_ReadAdcData_Original(AIN0,AINCOM,&ads); // 读取原始数据
  //     ADS1256_DATA.Voltage = ADS1256_ADCDataConvert(
  //             ADS1256_DATA.OriginalData,
  //             ADS1256_InitParams.VREF,  // 参考电压 
  //             1<<ADS1256_InitParams.ProgrammableGainAmplifierSetting// PGA增益倍数 (PGA_1 = 1倍增益)
  //     );
  //     printf("ADS1256_Ori_data,ADS1256_Voltage:%d,%lf\r\n", ADS1256_DATA.OriginalData,ADS1256_DATA.Voltage);
  // }
}




 
