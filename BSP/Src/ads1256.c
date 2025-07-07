#include "ads1256.h"
#include "main.h"
#include "delay.h"
#include <stdio.h>
#include <time.h>
#include "beep.h"

ADS1256_DATA_t ADS1256_DATA = {0};

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
  .AnalogInputBufferEnable              = BUFEN_OFF,                       // 输入缓冲器打开
  .InputMultiplexerControl              = POSITIVE_AIN0|NEGTIVE_AIN1,     // 输入通道选择，AINCOM作为负输入端，单端输入模式
  .ClockOutRateSetting                  = CLKOUT_OFF,                     // 输出时钟关闭       
  .SensorDetectCurrentSources           = DETECT_OFF,                     // 外部电路检测关闭  
  .ProgrammableGainAmplifierSetting     = PGA_1,                          // 可编程外部增益放大器设置为1倍增益
  .DataRateSetting                      = DRATE_15KHz,                     // 设置数据输出速率为100Hz
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

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WaitDRDY
*	功能说明: 等待内部操作完成。 自校准时间较长，需要等待。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
    uint32_t i;

    for (i = 0; i < 40000000; i++)
    {
        if (ADS1256_Read_DRDY == GPIO_PIN_RESET)
        {
            break;
        }
    }
    if (i >= 40000000)
    {
        printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* 调试语句. 用语排错 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadChipID
*	功能说明: 读芯片ID, 读状态寄存器中的高4bit
*	形    参: 无
*	返 回 值: 8bit状态寄存器值的高4位
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void)
{
    uint8_t id;

    ADS1256_WaitDRDY();
    ADS1256_Read_Reg(REG_STATUS, &id, 1, &ads); // 读取状态寄存器
    return (id >> 4);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_StopScan
*	功能说明: 停止 DRDY 中断
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_StopScan(void)
{
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
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

/*
*********************************************************************************************************
*	函 数 名: ADS1256_StartScan
*	功能说明: 将 DRDY引脚 （PC3 ）配置成外部中断触发方式， 中断服务程序中扫描8个通道的数据。
*	形    参: _ucDiffMode : 0 表示单端模式（扫描8路）； 1表示差分模式，扫描4路
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_StartScan(const uint8_t _ucScanMode)
{
    ADS1256_DATA.ReadOver = 0;
    ADS1256_DATA.ScanMode = _ucScanMode;
    /* 开始扫描前, 清零结果缓冲区 */
    {
        ADS1256_DATA.Channel = 0;

        for (uint8_t i = 0; i < 8; i++)
        {
            ADS1256_DATA.AdcNow[i] = 0;
        }
    }
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void ADS1256_Init(void)
{
    /*问题：发现使用stlink下载程序的时候一直显示Error, ASD1256 Chip ID = 0xF*/
    /*原因：stlink在下载程序的时候MCU的复位时间比ADS1256快速，而ADS1256上电和内部时钟未定需要一段时间，
     *如果ADS1256还没准备好就开始读数据就会ERROR*/
    /*解决办法加入writeCMD(CMD_RESET,&ads);即可解决*/
    writeCMD(CMD_RESET,&ads); // 软件复位ADS1256
    delay_us(1);//根据数据手册，复位之后需要延时最小0.5us
    {
        const uint8_t id = ADS1256_ReadChipID();
        HAL_Delay(10);
        if (id != 3)
        {
            printf("Error, ASD1256 Chip ID = 0x%X\r\n", id);
        }
        else
        {
            printf("Ok, ASD1256 Chip ID = 0x%X\r\n", id);
            BEEP_Short();
        }
    }
    ADS1256_StopScan(); // 停止 DRDY 中断
    uint8_t write_Reg[4] = {0};
    uint8_t read_Reg[4] = {0};

    // STATUS寄存器设置(0x34)
    write_Reg[REG_STATUS] =   ADS1256_InitParams.DataOutputBitOrder |
                            ADS1256_InitParams.AutoCalibration |
                            ADS1256_InitParams.AnalogInputBufferEnable;
    // MUX寄存器设置(0x01)
    write_Reg[REG_MUX]    =   ADS1256_InitParams.InputMultiplexerControl;
    // ADCON寄存器设置(0x00)
    write_Reg[REG_ADCON]  =   ADS1256_InitParams.ClockOutRateSetting |
                            ADS1256_InitParams.SensorDetectCurrentSources |
                            ADS1256_InitParams.ProgrammableGainAmplifierSetting;
    // DRATE寄存器设置(0x33) 0x33->15HZ
    write_Reg[REG_DRATE]  =   ADS1256_InitParams.DataRateSetting;



    ADS1256_Write_Reg(REG_STATUS,write_Reg,sizeof(write_Reg),&ads);
    //实测发现要想正确读到数据必须加延时和DRDY
    delay_us(10);
    ADS1256_WaitDRDY();
    ADS1256_Read_Reg(REG_STATUS,read_Reg,sizeof(read_Reg),&ads);
    printf("After_Write_Reg: %X %X %X %X\r\n", read_Reg[0], read_Reg[1], read_Reg[2], read_Reg[3]);

    //配置为差分输入模式,由于控制板AINCOM没有接入GND这里只能选择差分输入
    ADS1256_StartScan(3);


  // waitDRDY(&ads); // 等待数据准备好
  // writeCMD(CMD_SELFCAL, &ads); // 发送复位命令
  // waitDRDY(&ads); // 等待数据准备好


}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_SetChannal
*	功能说明: 配置通道号。多路复用。AIN- 固定接地（ACOM).
*	形    参: _ch : 通道号， 0-7
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
    /*
    Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
        0000 = AIN0 (default)
        0001 = AIN1
        0010 = AIN2 (ADS1256 only)
        0011 = AIN3 (ADS1256 only)
        0100 = AIN4 (ADS1256 only)
        0101 = AIN5 (ADS1256 only)
        0110 = AIN6 (ADS1256 only)
        0111 = AIN7 (ADS1256 only)
        1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

        NOTE: When using an ADS1255 make sure to only select the available inputs.

    Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
        0000 = AIN0
        0001 = AIN1 (default)
        0010 = AIN2 (ADS1256 only)
        0011 = AIN3 (ADS1256 only)
        0100 = AIN4 (ADS1256 only)
        0101 = AIN5 (ADS1256 only)
        0110 = AIN6 (ADS1256 only)
        0111 = AIN7 (ADS1256 only)
        1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
    */
    if (_ch > 7)
    {
        return;
    }
    uint8_t muxValue = (_ch << 4) | (1 << 3);
    ADS1256_Write_Reg(REG_MUX,&muxValue,1,&ads);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_SetDiffChannal
*	功能说明: 配置差分通道号。多路复用。
*	形    参: _ch : 通道号,0-3；共4对
*	返 回 值: 8bit状态寄存器值的高4位
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
    /*
    Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
        0000 = AIN0 (default)
        0001 = AIN1
        0010 = AIN2 (ADS1256 only)
        0011 = AIN3 (ADS1256 only)
        0100 = AIN4 (ADS1256 only)
        0101 = AIN5 (ADS1256 only)
        0110 = AIN6 (ADS1256 only)
        0111 = AIN7 (ADS1256 only)
        1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

        NOTE: When using an ADS1255 make sure to only select the available inputs.

    Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
        0000 = AIN0
        0001 = AIN1 (default)
        0010 = AIN2 (ADS1256 only)
        0011 = AIN3 (ADS1256 only)
        0100 = AIN4 (ADS1256 only)
        0101 = AIN5 (ADS1256 only)
        0110 = AIN6 (ADS1256 only)
        0111 = AIN7 (ADS1256 only)
        1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
    */
    uint8_t muxValue = 0;
    if (_ch == 0)
    {
        muxValue = (0 << 4) | 1;	/* 差分输入 AIN0， AIN1 */
        ADS1256_Write_Reg(REG_MUX, &muxValue,1,&ads);	/* 差分输入 AIN0， AIN1 */
    }
    else if (_ch == 1)
    {
        muxValue = (2 << 4) | 3;	/* 差分输入 AIN2， AIN3 */
        ADS1256_Write_Reg(REG_MUX, &muxValue,1,&ads);	/* 差分输入 AIN2， AIN3 */
    }
    else if (_ch == 2)
    {
        muxValue = (4 << 4) | 5;	/* 差分输入 AIN4， AIN5 */
        ADS1256_Write_Reg(REG_MUX, &muxValue,1,&ads);	/* 差分输入 AIN4， AIN5 */
    }
    else if (_ch == 3)
    {
        muxValue = (6 << 4) | 7;	/* 差分输入 AIN6， AIN7 */
        ADS1256_Write_Reg(REG_MUX, &muxValue,1,&ads);	/* 差分输入 AIN6， AIN7 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_DelayDATA
*	功能说明: 读取DOUT之前的延迟
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
    /*
        Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
        最小 50 个tCLK = 50 * 0.13uS = 6.5uS
    */
    delay_us(10);	/* 最小延迟 6.5uS, 此处取10us */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadData
*	功能说明: 读ADC数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
    uint32_t read = 0;
    const uint8_t cmd = CMD_RDATA;
    uint8_t rx_buf[3] = {0};

    CS_0(&ads);	/* SPI片选 = 0 */
    // 发送读数据命令
    HAL_SPI_Transmit(ads.hspix, &cmd, 1, HAL_MAX_DELAY);

    // 必须延迟才能读取芯片返回数据
    ADS1256_DelayDATA();

    // 读取3字节ADC数据
    HAL_SPI_Receive(ads.hspix, rx_buf, 3, HAL_MAX_DELAY);

    /*printf("Temp: %02X %02X %02X\r\n", rx_buf[0], rx_buf[1], rx_buf[2]);*/

    read = (rx_buf[0] << 16) | (rx_buf[1] << 8) | rx_buf[2];

    CS_1(&ads);	/* SPI片选 = 1 */

    /* 负数进行扩展。24位有符号数扩展为32位有符号数 */
    if (read & 0x800000)
    {
        read += 0xFF000000;
    }

    return (int32_t)read;
}


void ADS1256_Read_Data_ISR(void)
{
    if (ADS1256_DATA.ScanMode == 0)	/* 0 表示单端8路扫描，1表示差分4路扫描 */
    {
        /* 读取采集结构，保存在全局变量 */
        ADS1256_SetChannal(ADS1256_DATA.Channel);	/* 切换模拟通道 */
        delay_us(5);

        writeCMD(CMD_SYNC,&ads);
        delay_us(5);

        writeCMD(CMD_WAKEUP,&ads);
        delay_us(25);

        if (ADS1256_DATA.Channel == 0)
        {
            ADS1256_DATA.AdcNow[7] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
        }
        else
        {
            ADS1256_DATA.AdcNow[ADS1256_DATA.Channel-1] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
        }

        if (++ADS1256_DATA.Channel >= 8)
        {
            ADS1256_DATA.Channel = 0;
            ADS1256_DATA.ReadOver = 1;
        }
    }
    else if (ADS1256_DATA.ScanMode == 3) //固定读取某个通道-根据原理固定读取AIN0:信号输入，AIN1:GND
    {
        // ADS1256_SetDiffChannal(ADS1256_DATA.Channel);	/* 切换模拟通道 */
        // delay_us(5);

        // writeCMD(CMD_SYNC,&ads);
        // delay_us(5);
        //
        // writeCMD(CMD_WAKEUP,&ads);
        // delay_us(25);

        ADS1256_DATA.AdcNow[0] = ADS1256_ReadData();
        ADS1256_DATA.ReadOver = 1;
    }
    else	/* 差分4路扫描 */
    {
        /* 读取采集结构，保存在全局变量 */
        ADS1256_SetDiffChannal(ADS1256_DATA.Channel);	/* 切换模拟通道 */
        delay_us(5);

        writeCMD(CMD_SYNC,&ads);
        delay_us(5);

        writeCMD(CMD_WAKEUP,&ads);
        delay_us(25);

        if (ADS1256_DATA.Channel == 0)
        {
            ADS1256_DATA.AdcNow[3] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
            // printf("ADC1256_DATA.AdcNow[3] = %d\r\n", ADS1256_DATA.AdcNow[3]);
        }
        else
        {
            ADS1256_DATA.AdcNow[ADS1256_DATA.Channel-1] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
        }

        if (++ADS1256_DATA.Channel >= 4)
        {
            ADS1256_DATA.Channel = 0;
            ADS1256_DATA.ReadOver = 1;
        }
    }
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_GetAdc
*	功能说明: 从缓冲区读取ADC采样结果。采样结构是由中断服务程序填充的。
*	形    参: _ch 通道号 (0 - 7)
*	返 回 值: ADC采集结果（有符号数）
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch)
{
    if (_ch > 7)
    {
        return 0;
    }

    __set_PRIMASK(1);	/* 禁止中断 */

    int32_t iTemp = ADS1256_DATA.AdcNow[_ch];

    __set_PRIMASK(0);	/* 使能中断 */

    return iTemp;
}

 
