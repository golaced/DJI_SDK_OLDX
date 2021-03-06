#include   "beep.h"
#include   "time.h"
#include   "delay.h"

u8 start_music[]={
 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 
 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 17,21 , 21,21,
// 11,21, 12,21, 13,21, 14,21, 15,21, 16,21, 17,21, 21,21,
};

u8 mems_right_music[]={
 05,34, 06,34, 07,34, 
};

u8 mems_error_music[]={
 07,36, 00,32, 07,36, 00,32, 
};

u8 bat_error_music[]={
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32, 	
};

u8 rc_error_music[]={
 27,11, 00,11,   27,11, 00,11,  27,11, 00,11, 00,39 , 00,39
};

u8 beep_one[]={
 27,11, 00,11,   00,11, 00,11,  00,11, 00,11, 
};

u8 beep_two[]={
 27,11, 00,11,   27,11, 00,11,  00,11, 00,11, 
};

u8 beep_three[]={
 27,11, 00,11,   27,11, 00,11,  27,11, 00,11, 
};

u8 fc_state_beep[3];
u8 beep_state[]={
 00,11, 00,21,   00,11, 00,21,  00,11, 00,21, 00,11, 00,11,
};

u8 fc_save_gps_beep;
u8 beep_gps_save[]={
 00,11, 00,21,   00,11, 00,21,  
};

u8 start_music_pix[]={
 04,32, 24,22, 05,32, 
 04,32, 24,22, 05,32,
 04,32, 24,22, 05,32,
};
u8 test_to24;
u16 Beat_delay[7]={0,62,94,125,125,187,250};
void Play_Music(u8 *music, u16 st,u16 ed)
{
	u16 i;
  for(i=st;i<ed-st;i++)
   {
		 u8 level=music[i*2]/10;
		 u8 tone=music[i*2]%10;
		 Tone(level,tone);
		 u8 loop=music[i*2+1]%10;
		 u8 beat=music[i*2+1]/10;
     delay_ms(Beat_delay[beat]*loop);	 
	 }	
   Tone(0,0);		 
}

u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,u8 en,float dt)
{ static u8 state=0;
	static u16 i,cnt;
	
	switch(state)
	{
		case 0:
	     if(en)
	     {state=1;i=st;cnt=0;}
	  break;
	  case 1:
			 if(en) 
			 {
				u8 level=music[i*2]/10;
				u8 tone=music[i*2]%10;
				u8 loop=music[i*2+1]%10;
				u8 beat=music[i*2+1]/10;
				 
				 if(cnt++>Beat_delay[beat]*loop/1000./dt){Tone(level,tone);i++;cnt=0;
				 				} 
			 }
	     else
			 {Tone(0,0);state=0;  }
			 if(i>ed)
			 {state=0;Tone(0,0);	}
		break;
	} 
}

void Play_Music_Direct(u8 sel)
{
  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music(mems_right_music,0,sizeof(mems_right_music)/2);	
	  break;
		case START_BEEP:
		Play_Music(start_music,0,sizeof(start_music)/2);	
		break;
		case MEMS_ERROR_BEEP:
		Play_Music(mems_error_music,0,sizeof(mems_error_music)/2);		
		break;
	}
}

void Play_Music_Task(u8 sel,float dt)//<-----------------
{
  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music_In_Task(mems_right_music,0,sizeof(mems_right_music)/2,1,dt);	
	  break;
		case START_BEEP:
		Play_Music_In_Task(start_music,0,sizeof(start_music)/2,1,dt);	
		break;
		case MEMS_ERROR_BEEP:
		Play_Music_In_Task(mems_error_music,0,sizeof(mems_error_music)/2,1,dt);		
		break;
		case BAT_ERO_BEEP:
		Play_Music_In_Task(bat_error_music,0,sizeof(bat_error_music)/2,1,dt);		
		break;
		case RC_ERO_BEEP:
		Play_Music_In_Task(rc_error_music,0,sizeof(rc_error_music)/2,1,dt);		
		break;
		case BEEP_ONE:
		Play_Music_In_Task(beep_one,0,sizeof(beep_one)/2,1,dt);		
		break;
		case BEEP_TWO:
		Play_Music_In_Task(beep_two,0,sizeof(beep_two)/2,1,dt);		
		break;
		case BEEP_THREE:
		Play_Music_In_Task(beep_three,0,sizeof(beep_three)/2,1,dt);		
		break;
		case BEEP_STATE:
		beep_state[0]=fc_state_beep[0];	
		beep_state[2]=fc_state_beep[1];	
		beep_state[4]=fc_state_beep[2];	
		Play_Music_In_Task(beep_state,0,sizeof(beep_state)/2,1,dt);		
		break;
		case BEEP_GPS_SAVE:
		beep_gps_save[0]=beep_gps_save[2]=fc_save_gps_beep;	
		Play_Music_In_Task(beep_gps_save,0,sizeof(beep_gps_save)/2,1,dt);		
		break;
		default:
			Tone(0,0);
		break;
	}
}

void Beep_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); //GPIOF9复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM14
  Tone(0,0);
		
		
	Play_Music_Direct(START_BEEP);
//  u8 i,j;
//	for(i=0;i<3;i++)
//	 for(j=1;j<8;j++)
//	 {Tone(i,j);Delay_ms(111);}
	 Tone(0,0);
}  

u16 tone_table[3][8]={
     {0,261,293,329,349,391,440,493},
     {0,523,587,659,698,783,880,987},
     {0,1046,1174,1318,1396,1567,1760,1975}};
void Tone(u8 level, u8 tone)
{ u32 psc=84-1;
  u32 arr=1000000/tone_table[level][tone]-1;
  if(tone==0)
		arr=1000000/1-1;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM14
//	TIM8->PSC=psc;
//	TIM8->ARR=arr;
//	TIM8->CCR4=arr/2;

}
