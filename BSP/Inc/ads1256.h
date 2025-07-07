#ifndef __ADS1256_H
#define __ADS1256_H

#include "main.h"

#include "spi.h"


#define CH_NUM 4//差分是4路，单端是8路


#define ADS1256_Write_CS_L  HAL_GPIO_WritePin(ADS1256_CS_GPIO_Port,ADS1256_CS_Pin,GPIO_PIN_RESET)
#define ADS1256_Write_CS_H  HAL_GPIO_WritePin(ADS1256_CS_GPIO_Port,ADS1256_CS_Pin,GPIO_PIN_SET)
#define ADS1256_Read_DRDY   HAL_GPIO_ReadPin(ADS1256_DRDY_GPIO_Port,ADS1256_DRDY_Pin)

// ADS1256 寄存器地址定义
#define REG_STATUS                0x00    // 状态寄存器
#define REG_MUX                   0x01    // 输入通道选择寄存器
#define REG_ADCON                 0x02    // A/D控制寄存器
#define REG_DRATE                 0x03    // 数据输出速率寄存器
#define REG_IO                    0x04    // GPIO控制寄存器
#define REG_OFC0                  0x05    // 偏移校准系数0
#define REG_OFC1                  0x06    // 偏移校准系数1
#define REG_OFC2                  0x07    // 偏移校准系数2
#define REG_FSC0                  0x08    // 满量程校准系数0
#define REG_FSC1                  0x09    // 满量程校准系数1
#define REG_FSC2                  0x0A    // 满量程校准系数2

/*STATUS REGISTER*/
#define MSB_FRIST                (0x00<<3)
#define LSB_FRIST                (0x01<<3)
#define ACAL_OFF                 (0x00<<2)
#define ACAL_ON                  (0x01<<2)
#define BUFEN_OFF                (0x00<<1)
#define BUFEN_ON                 (0x01<<1)

/*Input Multiplexer Control Register*/
#define POSITIVE_AIN0            (0X00<<4)
#define POSITIVE_AIN1            (0X01<<4)
#define POSITIVE_AIN2            (0X02<<4)
#define POSITIVE_AIN3            (0X03<<4)
#define POSITIVE_AIN4            (0X04<<4)
#define POSITIVE_AIN5            (0X05<<4)
#define POSITIVE_AIN6            (0X06<<4)
#define POSITIVE_AIN7            (0X07<<4)
#define POSITIVE_AINCOM          (0X08<<4)        

#define NEGTIVE_AIN0              0X00
#define NEGTIVE_AIN1              0X01
#define NEGTIVE_AIN2              0X02
#define NEGTIVE_AIN3              0X03
#define NEGTIVE_AIN4              0X04
#define NEGTIVE_AIN5              0X05
#define NEGTIVE_AIN6              0X06
#define NEGTIVE_AIN7              0X07
#define NEGTIVE_AINCOM            0X08

/*ADCON REGISTER*/
#define CLKOUT_OFF                (0x00<<5)
#define CLKOUT_DIV1               (0x01<<5)
#define CLKOUT_DIV2               (0x02<<5)
#define CLKOUT_DIV4               (0x03<<5)

#define DETECT_OFF                (0x00<<3)
#define DETECT_CURRENT_0_5UA      (0x01<<3)
#define DETECT_CURRENT_2UA        (0x02<<3)
#define DETECT_CURRENT_10UA       (0x03<<3)

#define PGA_1                      0x00  
#define PGA_2                      0x01  
#define PGA_4                      0x02  
#define PGA_8                      0x03  
#define PGA_16                     0x04  
#define PGA_32                     0x05  
#define PGA_64                     0x06  

/*A/D Data Rate REGESTER*/
#define DRATE_30KHz                0xF0
#define DRATE_15KHz                0xE0
#define DRATE_7_5KHz               0xD0
#define DRATE_3_75KHz              0xC0
#define DRATE_2KHz                 0xB0
#define DRATE_1KHz                 0xA1
#define DRATE_500Hz                0x92
#define DRATE_100Hz                0x82
#define DRATE_60Hz                 0x72
#define DRATE_50Hz                 0x63
#define DRATE_30Hz                 0x53
#define DRATE_25Hz                 0x43
#define DRATE_15Hz                 0x33
#define DRATE_10Hz                 0x23
#define DRATE_5Hz                  0x13
#define DRATE_2_5Hz                0x03

/*GPIO Control Register*/
#define D3_IO_PIN_OUTPUT          (0x00<<7)
#define D3_IO_PIN_INPUT           (0x01<<7)
#define D2_IO_PIN_OUTPUT          (0x00<<6)
#define D2_IO_PIN_INPUT           (0x01<<6)
#define D1_IO_PIN_OUTPUT          (0x00<<5)
#define D1_IO_PIN_INPUT           (0x01<<5)
#define D0_IO_PIN_OUTPUT          (0x00<<4)
#define D0_IO_PIN_INPUT           (0x01<<4)

//  This commands control the operation of the ADS1256. 
//	All of the commands are stand-alone except for the register reads and writes 
//	(RREG, WREG) which require a second command byte plus data.
//	CS must stay low (CS_0()) during the entire command sequence.
enum
{
	CMD_WAKEUP   = 0x00, // Completes SYNC and Exits Standby Mode
	CMD_RDATA    = 0x01, // Read Data
	CMD_RDATAC   = 0x03, // Read Data Continuously
	CMD_SDATAC   = 0x0F, // Stop Read Data Continuously
	CMD_RREG     = 0x10, // Read from REG - 1st command byte: 0001rrrr 
						 //					2nd command byte: 0000nnnn
	CMD_WREG     = 0x50, // Write to REG  - 1st command byte: 0001rrrr
						 //					2nd command byte: 0000nnnn
						 // r = starting reg address, n = number of reg addresses
	CMD_SELFCAL  = 0xF0, // Offset and Gain Self-Calibration
	CMD_SELFOCAL = 0xF1, // Offset Self-Calibration
	CMD_SELFGCAL = 0xF2, // Gain Self-Calibration
	CMD_SYSOCAL  = 0xF3, // System Offset Calibration
	CMD_SYSGCAL  = 0xF4, // System Gain Calibration
	CMD_SYNC     = 0xFC, // Synchronize the A/D Conversion
	CMD_STANDBY  = 0xFD, // Begin Standby Mode
	CMD_RESET    = 0xFE, // Reset to Power-Up Values
};

enum
{
	AIN0   = 0, //Binary value: 0000 0000
	AIN1   = 1, //Binary value: 0000 0001
	AIN2   = 2, //Binary value: 0000 0010
	AIN3   = 3, //Binary value: 0000 0011
	AIN4   = 4, //Binary value: 0000 0100
	AIN5   = 5, //Binary value: 0000 0101
	AIN6   = 6, //Binary value: 0000 0110
	AIN7   = 7, //Binary value: 0000 0111
	AINCOM = 8  //Binary value: 0000 1000
};

typedef struct 
{
    SPI_HandleTypeDef *hspix;

    GPIO_TypeDef *csPort;
    uint16_t     csPin;

    GPIO_TypeDef *drdyPort;
    uint16_t     drdyPin;
	
    GPIO_TypeDef *resetPort;
    uint16_t     resetPin;
} ADS125X_t;


typedef struct 
{
    uint8_t DataOutputBitOrder;                 // 数据输出位序
    uint8_t AutoCalibration;                    // 自校准
    uint8_t AnalogInputBufferEnable;            // 输入缓冲
    uint8_t InputMultiplexerControl;            // 通道选择
    uint8_t ClockOutRateSetting;                // 输出时钟
    uint8_t SensorDetectCurrentSources;         // 外部电路检测
    uint8_t ProgrammableGainAmplifierSetting;   // 可编程外部增益放大器
    uint8_t DataRateSetting;                    // 数据输出速率
    double  VREF;                               // 参考电压
}ADS1256_InitParams_t;

typedef struct 
{
	int32_t AdcNow[8];			/* 8路ADC采集结果（实时）有符号数 */
	uint8_t Channel;			/* 当前通道 */
	uint8_t ScanMode;			/* 扫描模式，0表示单端8路， 1表示差分4路 */
	uint8_t ReadOver;
	int32_t adc[8];
	int32_t volt[8];
	double Voltage;			/* 电压值，单位V */

}ADS1256_DATA_t;


extern ADS1256_DATA_t ADS1256_DATA;

void ADS1256_Init(void);
void ADS1256_Read_Data_ISR(void);
int32_t ADS1256_GetAdc(uint8_t _ch);

#endif
