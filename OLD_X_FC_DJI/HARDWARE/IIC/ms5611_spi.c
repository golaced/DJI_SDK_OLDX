#include "ms5611_spi.h"
#include "time.h"
#define ENABLE_MS5611       SPI_CS(MS5611,0)
#define DISABLE_MS5611      SPI_CS(MS5611,1)

typedef union {
    uint16_t value;
    uint8_t bytes[2];
} uint16andUint8_t;

typedef union {
    uint32_t value;
    uint8_t bytes[4];
} uint32andUint8_t;

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
#define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)


uint16andUint8_t c1, c2, c3, c4, c5, c6;

uint32andUint8_t d1, d2;

int32_t dT;

int32_t ms5611Temperature;
int32_t ms5611Press,ms5611Temp;
float   ms5611Alt;
///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

int32_t calculateTemperature(void)
{
    dT = (int32_t)d2.value - ((int32_t)c5.value << 8);
    ms5611Temperature = 2000 + (int32_t)(((int64_t)dT * c6.value) >> 23);
    return ms5611Temperature;
}

///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperature()
{
    ENABLE_MS5611;
    Spi_RW(0x00);
    d2.bytes[2] = Spi_RW(0x00);
    d2.bytes[1] = Spi_RW(0x00);
    d2.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;
    calculateTemperature();
}

void readPressure()
{
    ENABLE_MS5611;
    Spi_RW(0x00);
    d1.bytes[2] = Spi_RW(0x00);
    d1.bytes[1] = Spi_RW(0x00);
    d1.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;
}

void requestTemperature()
{
    ENABLE_MS5611;                      // Request temperature conversion
#if   (OSR ==  256)
    Spi_RW(0x50);
#elif (OSR ==  512)
    Spi_RW(0x52);
#elif (OSR == 1024)
    Spi_RW(0x54);
#elif (OSR == 2048)
    Spi_RW(0x56);
#elif (OSR == 4096)
    Spi_RW(0x58);
#endif
    DISABLE_MS5611;
}

void requestPressure()
{
    ENABLE_MS5611;                      // Request pressure conversion
#if   (OSR ==  256)
    Spi_RW(0x40);
#elif (OSR ==  512)
    Spi_RW(0x42);
#elif (OSR == 1024)
    Spi_RW(0x44);
#elif (OSR == 2048)
    Spi_RW(0x46);
#elif (OSR == 4096)
    Spi_RW(0x48);
#endif
    DISABLE_MS5611;
}


///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////
static float Alt_offset_Pa=0; 
void calculatePressureAltitude(int32_t *pressure, int32_t *temperature)
{
    int64_t offset;
    int64_t offset2 = 0;
    int64_t sensitivity;
    int64_t sensitivity2 = 0;
    int64_t f;
    int32_t p;

    int32_t ms5611Temp2 = 0;

    offset = ((int64_t)c2.value << 16) + (((int64_t)c4.value * dT) >> 7);
    sensitivity = ((int64_t)c1.value << 15) + (((int64_t)c3.value * dT) >> 8);

    if (ms5611Temperature < 2000) {
        ms5611Temp2 = (dT * dT) >> 31;

        f = ms5611Temperature - 2000;
        f = f * f;
        offset2 = 5 * f >> 1;
        sensitivity2 = 5 * f >> 2;

        if (ms5611Temperature < -1500) {
            f = (ms5611Temperature + 1500);
            f = f * f;
            offset2 += 7 * f;
            sensitivity2 += 11 * f >> 1;
        }

        ms5611Temperature -= ms5611Temp2;

        offset -= offset2;
        sensitivity -= sensitivity2;
    }

    p = (((d1.value * sensitivity) >> 21) - offset) >> 15;
    if (pressure)
        *pressure = p;
    if (temperature)
        *temperature = ms5611Temperature;
		
  static u16 paInitCnt;	
  static long paOffsetNum;		
	// �Ƿ��ʼ����0����ѹֵ��
	if(Alt_offset_Pa == 0)
	{ 
		if(paInitCnt > 500)
		{
			Alt_offset_Pa = paOffsetNum / paInitCnt;
			//paOffsetInited=1;
		}
		else
			paOffsetNum += p;
		
		paInitCnt++;
		
		ms5611Alt = 0; //�߶� Ϊ 0
		
	}else
   ms5611Alt= (44330.0f * (1.0f - pow((float)p / Alt_offset_Pa, 1.0f / 5.255f)));
	// ms5611Alt = 4433000.0 * (1 - pow((p / Alt_offset_Pa), 0.1903))*0.01f;  
   
	 
}

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

u8 ms5611DetectSpi(void)
{
    

    ENABLE_MS5611;   // Reset Device
    Spi_RW(0x1E);
    Delay_ms(3);
    DISABLE_MS5611;

    Delay_us(150);

    ENABLE_MS5611;   // Read Calibration Data C1
    Spi_RW(0xA2);
    c1.bytes[1] = Spi_RW(0x00);
    c1.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C2
    Spi_RW(0xA4);
    c2.bytes[1] = Spi_RW(0x00);
    c2.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C3
    Spi_RW(0xA6);
    c3.bytes[1] = Spi_RW(0x00);
    c3.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C4
    Spi_RW(0xA8);
    c4.bytes[1] = Spi_RW(0x00);
    c4.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C5
    Spi_RW(0xAA);
    c5.bytes[1] = Spi_RW(0x00);
    c5.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

    Delay_ms(10);

    ENABLE_MS5611;   // Read Calibration Data C6
    Spi_RW(0xAC);
    c6.bytes[1] = Spi_RW(0x00);
    c6.bytes[0] = Spi_RW(0x00);
    DISABLE_MS5611;

 
    return 1;
}

#include "bmp.h"
static uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
	//20000, // 16.44ms
};
//#define MS5611Press_OSR  MS561101BA_OSR_2048  //��ѹ��������
//#define MS5611Temp_OSR   MS561101BA_OSR_2048 //�¶Ȳ�������
#define MS5611Press_OSR  MS561101BA_OSR_4096  //��ѹ��������
#define MS5611Temp_OSR   MS561101BA_OSR_4096 //�¶Ȳ�������
// ��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ �¶�ת��
#define CTemperatureing  0x02  //����ת���¶�
#define SCPressure  0x03	  //��ʼת�� ��ѹ
#define SCPressureing  0x04	  //����ת����ѹֵ
uint8_t  Now_doing = SCTemperature;	//��ǰת��״̬
static uint32_t Current_delay=0;	    //ת����ʱʱ�� us 
static uint32_t Start_Convert_Time; //����ת��ʱ�� ʱ�� us 
static uint8_t Baro_ALT_Updated = 0; //��ѹ�Ƹ߶ȸ�����ɱ�־��
void MS5611_ThreadNew_SPI(void) 
{
	switch(Now_doing)
	{ //��ѯ״̬ ������������ ����Щʲô��
 		case SCTemperature:  //�����¶�ת��
			//�����¶�ת��		  
        requestTemperature(); 
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time =GetSysTime_us();// micros(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
 		break;
		
		case CTemperatureing:  //����ת���� 
			if((GetSysTime_us()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				readTemperature(); 
				//������ѹת��
				requestPressure(); 
				Current_delay = MS5611_Delay_us[MS5611Press_OSR];//ת��ʱ��
				Start_Convert_Time = GetSysTime_us();//��ʱ��ʼ
				Now_doing = SCPressureing;//��һ��״̬
			}
			break;
 
		case SCPressureing:	 //����ת����ѹֵ
			if((GetSysTime_us()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				readPressure();
				calculatePressureAltitude(&ms5611Press,&ms5611Temp);
				Baro_ALT_Updated = 0xff; 	//�߶ȸ��� ��ɡ�
				//�����¶�ת��
				requestTemperature(); 
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time = GetSysTime_us(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
			}
			break;
		default: 
			Now_doing = CTemperatureing;
			break;
	}
}


//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI1�ĳ�ʼ��
void SPI2_Init1(void)
{	GPIO_InitTypeDef GPIO_InitStructure;
		SPI_InitTypeDef  SPI_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOFʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOFʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOFʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//ʹ��GPIOFʱ��

	 //
	/*?? SPI_NRF_SPI? SCK,MISO,MOSI??,GPIOA^5,GPIOA^6,GPIOA^7 */ //pa5  pa6  pa7
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13|GPIO_Pin_15|GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	/*??SPI_NRF_SPI?CE??,?SPI_NRF_SPI? CSN ??:*/   //ce pc4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //ce
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	#if USE_VER_4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //csn  pa4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	/*??SPI_NRF_SPI?IRQ??,*/  //pc5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //csn  pa4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //csn  pa4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //csn  pa4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	/*??SPI_NRF_SPI?IRQ??,*/  //pc5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //csn  pa4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //csn  pa4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	#endif

	SPI_CS(NRF2401,1);
	SPI_CS(MPU9250,1);
	SPI_CS(MS5611,1);
	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB3����Ϊ SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); //PB4����Ϊ SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); //PB5����Ϊ SPI1
 
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//��λSPI1
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//ֹͣ��λSPI1 
       
  	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //????? 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //??? 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //????8? 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //????,????? 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //?1?????,???????? 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS??????? 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8??,9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //???? 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI2, &SPI_InitStructure); 
	/* Enable SPI1 */ 
	SPI_Cmd(SPI2, ENABLE);
}

u8 Spi_RW(u8 dat) 
{ 
	/* ? SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
	/* ?? SPI2??????? */ 
	SPI_I2S_SendData(SPI2, dat); 
	/* ?SPI?????????? */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI2); 
}


void SPI_CS(u8 sel,u8 set)
{
	#if USE_VER_4
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	GPIO_SetBits(GPIOC, GPIO_Pin_4);//9250
	GPIO_SetBits(GPIOC, GPIO_Pin_5);//ms5611
switch(sel)
{
	case MPU9250:
  if(set)	
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	else
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	break;
	case NRF2401:
	if(set)	
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	else
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	break;
	case MS5611:
	if(set)	
	GPIO_SetBits(GPIOC, GPIO_Pin_5);
	else
	GPIO_ResetBits(GPIOC, GPIO_Pin_5);
	break;
}
	#else
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);//9250
	GPIO_SetBits(GPIOB, GPIO_Pin_9);//ms5611
switch(sel)
{
	case MPU9250:
  if(set)	
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	else
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	break;
	case NRF2401:
	if(set)	
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	else
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	break;
	case MS5611:
	if(set)	
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
	else
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	break;
}
 #endif
}	