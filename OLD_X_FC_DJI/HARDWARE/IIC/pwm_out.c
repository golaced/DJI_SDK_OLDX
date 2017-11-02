
#include "pwm_in.h"
#include "pwm_out.h"
#include "include.h"
#include "my_math.h"

//21·ÖÆµµ½ 84000000/21 = 4M   0.25us

#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0

//u16 shoot_cnt[3]={10,6,0};
u16 shoot_cnt[3]={10,3,0};
void EN_SHOOT(u8 on)
{
static u16 cnt;	
static u8 state;
	
switch (state)
{
	case 0:
		if(on)
		{cnt=0;state=1;GPIO_SetBits(GPIOB,GPIO_Pin_8);}
	break;
	 case 1:
     if(cnt++>shoot_cnt[0])
		 {cnt=0;state=2;GPIO_ResetBits(GPIOB,GPIO_Pin_8);}	
		break;		 
	 
	case 2:
     if(cnt++>shoot_cnt[1])
		 {cnt=0;state=0;if(en_shoot)en_shoot=0;cnt_shoot++;}
		
	break;		 

}
}



void SHOOT_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	EN_SHOOT(0);


}

u8 sel=0;
s32 pwm_tem_view[8];
#include "filter.h"
void SetPwm(u32 pwm[8],int off[8],u32 min,u32 max)
{
	u8 i;
	s32 pwm_tem[8];
	u32 pwmr[8];
		for(i=0;i<8;i++)
	{  
			pwmr[i] = pwm[i]+off[i];
	}
	
	

	for(i=0;i<8;i++)
	{   
			pwm_tem[i] = Moving_Median(i+2,3,pwmr[i]);
		 if(i!=4||i!=5)
			pwm_tem_view[i]=pwm_tem[i] = LIMIT(pwm_tem[i],min,max);
	}
	

	TIM1->CCR4 = ( pwm_tem[0] ) ;//+ INIT_DUTY;				//1	
	TIM1->CCR3 = ( pwm_tem[1] ) ;//+ INIT_DUTY;				//2
	TIM1->CCR2 = ( pwm_tem[2] ) ;//+ INIT_DUTY;				//3	
	TIM1->CCR1 = ( pwm_tem[3] ) ;//+ INIT_DUTY;				//4

 	TIM4->CCR2 = ( pwm_tem[4] ) ;//+ INIT_DUTY;				//5	
 	TIM4->CCR1 = ( pwm_tem[5] ) ;//+ INIT_DUTY;				//6	
	TIM4->CCR3 = ( pwm_tem[4] ) ;//+ INIT_DUTY;				//5	
 	//TIM4->CCR4 = ( pwm_tem[5] ) ;//+ INIT_DUTY;				//6	
	
 // TIM8->CCR4 = ( pwm_tem[6] ) ;//+ INIT_DUTY;				//7	
 // TIM8->CCR3 = ( pwm_tem[7] ) ;//+ INIT_DUTY;				//8	
}


u8 PWM_AUX_Out_Init(uint16_t hz)//50Hz
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz*2;

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);

		hz_set = LIMIT (hz_set,1,84000000);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
		//////////////////////////////////////TIM8///////////////////////////////////////////
	  hz_set = ACCURACY*hz;
		hz_set = LIMIT (hz_set,1,84000000);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOC, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC2Init(TIM8, &TIM_OCInitStructure);


		TIM_CtrlPWMOutputs(TIM8, ENABLE);
		TIM_ARRPreloadConfig(TIM8, ENABLE);
		TIM_Cmd(TIM8, ENABLE);	
//---- TIM4

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);

		TIM_CtrlPWMOutputs(TIM4, ENABLE);
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_Cmd(TIM4, ENABLE);	
		
		
    SetPwm_AUX(0,0,0);
		
	
}

AUX_S aux;
float kp_aux[2]={8,20};
float yaw_test=0;
void SetPwm_AUX_DJ(float pit,float rol,float yaw)
{ static u8 init;
	u8 i;
	if(!init&&Pit_yun!=0&&Rol_yun!=0)
	{
	init=1;
	aux.init[0]=1525;
	aux.init[1]=1525;
	aux.min[0]=500+150;
	aux.min[1]=500+150;
  aux.max[0]=2500-150;
	aux.max[1]=2500-150;
		
	aux.init[2]=1200;
  aux.min[2]=850;
	aux.max[2]=1500;		
	aux.flag[0]=-1;	
  aux.flag[1]=-1;		
	aux.flag[2]=1;		
	aux.pwm_per_dig=10;
	}	
	
	aux.pwm_tem[0]=aux.init[0]+kp_aux[0]*(pit-Pit_yun);//aux.pwm_per_dig*pit*aux.flag[0];
	aux.pwm_tem[1]=aux.init[1]+kp_aux[1]*(rol-Rol_yun);//aux.pwm_per_dig*rol*aux.flag[1];
	aux.pwm_tem[2]=aux.init[2]+aux.pwm_per_dig*yaw*aux.flag[2];
	for(i=0;i<3;i++)
	{
			aux.pwm_tem[i] = LIMIT(aux.pwm_tem[i],aux.min[i],aux.max[i]);
	}
	if(!init)
	{
	aux.pwm_tem[0]=aux.init[0];
	aux.pwm_tem[1]=aux.init[1];
	aux.pwm_tem[2]=aux.init[2];
	}
	TIM8->CCR1 = (aux.pwm_tem[0] ) ;				//pit	
	TIM8->CCR2 = (aux.pwm_tem[1] ) ;				//rol
	TIM4->CCR3 = (aux.pwm_tem[2] )/2  ;          //yaw
}	

void SetPwm_AUX(float pit,float rol,float yaw)
{ static u8 init;
	u8 i;
	if(!init)
	{
	init=1;
	aux.init[0]=1500;
	aux.init[1]=1500;
	aux.min[0]=500+150;
	aux.min[1]=500+150;
  aux.max[0]=2500-150;
	aux.max[1]=2500-150;
		
	aux.init[2]=1200;
  aux.min[2]=850;
	aux.max[2]=1500;		
	aux.flag[0]=-1;	
  aux.flag[1]=-1;		
	aux.flag[2]=1;		
	aux.pwm_per_dig=10;
	}	
	
	aux.pwm_tem[0]=pit;//aux.init[0]+kp_aux[0]*(pit-Pit_yun);//aux.pwm_per_dig*pit*aux.flag[0];
	aux.pwm_tem[1]=rol;//aux.init[1]+kp_aux[1]*(rol-Rol_yun);//aux.pwm_per_dig*rol*aux.flag[1];
	aux.pwm_tem[2]=aux.init[2]+aux.pwm_per_dig*yaw*aux.flag[2];
	for(i=0;i<3;i++)
	{
			aux.pwm_tem[i] = LIMIT(aux.pwm_tem[i],aux.min[i],aux.max[i]);
	}
	if(!init)
	{
	aux.pwm_tem[0]=aux.init[0];
	aux.pwm_tem[1]=aux.init[1];
	aux.pwm_tem[2]=aux.init[2];
	}
	TIM8->CCR1 = (aux.pwm_tem[0] ) ;				//pit	
	TIM8->CCR2 = (aux.pwm_tem[1] ) ;				//rol
	TIM4->CCR3 = (aux.pwm_tem[2] )/2  ;          //yaw
}	


void SEL_PWM(u8 sel)
{
//GPIO_SetBits(GPIOC,GPIO_Pin_3);//out5678 force up
//if(sel){
////GPIO_SetBits(GPIOC,GPIO_Pin_3);
//GPIO_SetBits(GPIOD,GPIO_Pin_12);GPIO_SetBits(GPIOD,GPIO_Pin_15);}
//else{
////GPIO_ResetBits(GPIOC,GPIO_Pin_3);
//GPIO_ResetBits(GPIOD,GPIO_Pin_12);GPIO_ResetBits(GPIOD,GPIO_Pin_15);}
}
void SEL_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;   
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_12|GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;   
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	SEL_PWM(0);
}




u8 PWM_Out_Init_FOR_CAL(uint16_t hz,uint16_t min,uint16_t max)//400hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz*2;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	hz_set = LIMIT (hz_set,1,84000000);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE, ENABLE);

////////////////////////////////PWM5-8//////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	hz_set = ACCURACY*hz;
	//------------------
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
///////////////////////////////////PWM1-4///////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	hz_set = ACCURACY*hz;
  hz_set = LIMIT (hz_set,1,84000000);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = hz;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_Cmd(TIM1, ENABLE);	
	//////////////////////////////////AUX 1-2///////////////////////////////////////////

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	
	/* Compute the prescaler value */
  //PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = hz;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = min;
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
  TIM_ARRPreloadConfig(TIM8, ENABLE);
  TIM_Cmd(TIM8, ENABLE);


	if( hz_set > 84000000 )
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
