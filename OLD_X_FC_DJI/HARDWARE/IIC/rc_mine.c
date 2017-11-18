
#include "rc.h"
#include "led_fc.h"
#include "ms5611.h"
#include "height_ctrl.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "circle.h"

#define RX_DR			6		//????
#define TX_DS			5
#define MAX_RT		4

vs16 QH,ZY,XZ;
u8 is_lock=1;
u8 EN_FIX_GPSF=0;
u8 EN_FIX_LOCKWF=0;
u8 EN_CONTROL_IMUF=0;
u8 EN_FIX_INSF=0;
u8 EN_FIX_HIGHF=0;
u8 tx_lock=1;
u8 EN_FIX_GPS=0;
u8 EN_FIX_LOCKW=0;
u8 EN_CONTROL_IMU=0;
u8 EN_FIX_INS=0;
u8 EN_FIX_HIGH=0;
u8 EN_TX_GX=1;
u8 EN_TX_AX=1;
u8 EN_TX_HM=1;
u8 EN_TX_YRP=1;
u8 EN_TX_GPS=1;
u8 EN_TX_HIGH=1;
u8 up_load_set=0;
u8 up_load_pid=0;
u8 key_rc[6]={1,1,1,1,1,1};
u16 Yaw_sb_rc=0;

u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;
u16 data_rate;
#define MID_RC_KEY 15
 u8 key_rc_reg[7][MID_RC_KEY];
#define MID_RC_GET 4
float RC_GET[4][MID_RC_GET];
float control_scale=1;
float ypr_sb[3];


void CAL_CHECK(void)
{static u8 state_mpu,state_hml;
static u16 cnt_mpu,cnt_hml;
static u8 check_num_mpu,check_num_hml;
	if(!Mag_CALIBRATED&&!fly_ready)
	switch (state_mpu)
	{
		case 0:
			 if(Rc_Get.YAW-1500>200&&!mode.cal_sel)
				  state_mpu=1;
			 break;
		case 1:
			 if(Rc_Get.YAW-1500<-200&&!mode.cal_sel)
				  state_mpu=2;
			 else if(cnt_mpu++>2000)
			 {cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			 else if(check_num_mpu>6)
			 {  state_mpu=3;
	LEDRGB_COLOR(YELLOW);Delay_ms(100);
	LEDRGB_COLOR(BLACK);Delay_ms(100);
	LEDRGB_COLOR(YELLOW);Delay_ms(100);
	LEDRGB_COLOR(BLACK);Delay_ms(100);
	LEDRGB_COLOR(YELLOW);Delay_ms(100);
	LEDRGB_COLOR(BLACK);Delay_ms(100);
	LEDRGB_COLOR(YELLOW);Delay_ms(500);
				 
				 mpu6050.Gyro_CALIBRATE=1;
			  mpu6050.Acc_CALIBRATE=1;}
			  break;
		case 2:
			if(Rc_Get.YAW-1500>200&&!mode.cal_sel)
			{  state_mpu=1;check_num_mpu++;}
			 else if(cnt_mpu++>2000)
			 {cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			  break;
		case 3:	 
			 if(!mpu6050.Gyro_CALIBRATE)
			 {  
			 cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			break; 
				
	}
	else
	{cnt_mpu=0;state_mpu=0;check_num_mpu=0;}

if( !mpu6050.Gyro_CALIBRATE&&!fly_ready)
switch (state_hml)
	{
		case 0:
			 if(Rc_Get.YAW-1500>200&&mode.cal_sel)
				  state_hml=1;
			 break;
		case 1:
			 if(Rc_Get.YAW-1500<-200&&mode.cal_sel)
				  state_hml=2;
			 else if(cnt_hml++>2000)
			 {state_hml=0;cnt_hml=0;check_num_hml=0;}
			 else if(check_num_hml>6)
			 {  state_hml=3;
			LEDRGB_COLOR(BLUE);Delay_ms(100);
			LEDRGB_COLOR(BLACK);Delay_ms(100);
			LEDRGB_COLOR(BLUE);Delay_ms(100);
			LEDRGB_COLOR(BLACK);Delay_ms(100);
			LEDRGB_COLOR(BLUE);Delay_ms(100);
			LEDRGB_COLOR(BLACK);Delay_ms(100);
			LEDRGB_COLOR(BLUE);Delay_ms(500);Mag_CALIBRATED=1;				 
			 }
			  break;
		case 2:
			if(Rc_Get.YAW-1500>200&&mode.cal_sel)
			{  state_hml=1;check_num_hml++;}
			 else if(cnt_hml++>2000)
			 {state_hml=0;cnt_hml=0;check_num_hml=0;}
			  break;
		case 3:	 
			 if(!Mag_CALIBRATED)
			 {  //flag.calibratingM=0;
			   
			 cnt_hml=0;state_hml=0;check_num_hml=0;}
			break; 
				
	}
	else
	{
	cnt_hml=0;state_hml=0;check_num_hml=0;
	}

}

