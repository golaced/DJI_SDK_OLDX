#include "stm32f4xx.h"
#include "include.h"

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
#define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)

int32_t calculateTemperature(void);
void readTemperature();
void readPressure();
void requestTemperature();
void requestPressure();
void calculatePressureAltitude(int32_t *pressure, int32_t *temperature);
u8 ms5611DetectSpi(void);
void MS5611_ThreadNew_SPI(void) ;

extern int32_t ms5611Press,ms5611Temp;
extern float   ms5611Alt;
				    
// SPI总线速度设置 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7
						  	    													  
void SPI2_Init1(void);			 //初始化SPI1口
u8 Spi_RW(u8 TxData);//SPI1总线读写一个字节

#define MPU9250 0
#define NRF2401 1
#define MS5611  2
void SPI_CS(u8 sel,u8 set);

