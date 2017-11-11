#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"
#define MAXMOTORS 		(4)		//电机数量
u8 PWM_Out_Init(uint16_t hz);
void  Set_DJ(float ang1,float ang2,float ang3,float ang4);
void SetPwm(u32 pwm[8],int off[8],u32 min,u32 max);
void SEL_PWM(u8 sel);
void SEL_Init();
u8 PWM_Out_Init_FOR_CAL(uint16_t hz,uint16_t min,uint16_t max);
void SHOOT_Init(void);
void EN_SHOOT(u8 on);
void EN_SHOOT_D(u8 on);
typedef struct 
{ u16 pwm_tem[3];
	int flag[3];
	u16 init[3];
	u16 max[3];
	u16 min[3];
	float att[3],att_ctrl[3],att_off[3];
	float pwm_per_dig;
	float ero[3],ero_reg[3];
}AUX_S;

extern AUX_S aux;
u8 PWM_AUX_Out_Init(uint16_t hz);//50Hz
void SetPwm_AUX(float pit,float rol,float yaw);
#endif

