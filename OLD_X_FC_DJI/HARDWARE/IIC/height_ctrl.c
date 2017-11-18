#include "height_ctrl.h"
#include "ultrasonic.h"
#include "my_math.h"
#include "filter.h"
#include "att.h"
#include "ms5611.h"
#include "rc.h"
#include "alt_kf.h"
#include  "pwm_in.h"
#include  "circle.h"
#if PLANE_IS_BIG
#define HOLD_THR 600 
#define ALT_HOLD_THR_RANGE_SCALE 2
#else  ///use now
#define HOLD_THR 420 // 5030 M=600g/310g*4=0.48 *1.1=520 | 6030 M=600g/410*4=0.34 *1.1= 0.37  PA 570
#define ALT_HOLD_THR_RANGE_SCALE LIMIT((1000-HOLD_THR)/250,1,2.5)//PA 250
#endif

#define EXP_Z_SPEED  ( 0.4f *my_deathzoom( (thr-500), 50 ) )
float exp_spd_zv,thr_use,thr_in_view;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid_v wz_speed_pid_v_safe;
_st_height_pid_v wz_speed_pid_v_eso;

_st_height_pid_v ultra_ctrl,ultra_ctrl_safe,ultra_ctrl_head;

_st_height_pid wz_speed_pid,wz_speed_pid_safe;
_st_height_pid ultra_pid_safe,ultra_pid,ultra_pid_head;
#define high_data_sel 0//高度数据选择0-> bmp from m100
#define OFF_HEIGHT_ALL 3500
#define MAX_HEIGH_ERO 800  //check
float exp_height_speed;
float exp_height=1200+OFF_HEIGHT_ALL,exp_height_check=1366+OFF_HEIGHT_ALL,//1466,//
											exp_height_front=1266+OFF_HEIGHT_ALL,//1250,
											 exp_height_back=965+OFF_HEIGHT_ALL,//1100,//1050,
                       exp_height_home=4000,
	exp_height_shoot_off=0;

#define OFF_HEIGHT_ALL_FRONE 0
#define DEAD_NAV_RC 80
#if USE_M100
	float exp_height_head=1800+300+OFF_HEIGHT_ALL_FRONE,exp_height_head_scan=2150+400+OFF_HEIGHT_ALL_FRONE;
	#if RISK_MODE
	float exp_height_head_shoot=1800+OFF_HEIGHT_ALL_FRONE;
	#else
	float exp_height_head_shoot=2000+OFF_HEIGHT_ALL_FRONE;
	#endif
	float	exp_height_head_check=4666+OFF_HEIGHT_ALL_FRONE,exp_height_map=2500+OFF_HEIGHT_ALL_FRONE,exp_height_map_to=2222+OFF_HEIGHT_ALL_FRONE;
#else
	float exp_height_head=1800+OFF_HEIGHT_ALL_FRONE,exp_height_head_scan=2150+OFF_HEIGHT_ALL_FRONE,exp_height_head_shoot=1800+OFF_HEIGHT_ALL_FRONE;
#endif

float exp_height_speed_safe,exp_height_safe;
float ultra_speed,ultra_speed_safe;
float ultra_dis_lpf,ultra_dis_lpf_safe;
float ultra_ctrl_out_safe, ultra_ctrl_out,ultra_ctrl_out_head;
float baro_speed;
float height_ctrl_out,height_ctrl_out_head;
float wz_acc;
float adrc_out;
void Ultra_PID_Init()
{//use 
	#if USE_M100
	ultra_pid.kp = 1.25;
	ultra_pid.ki = 0.25;//1;//101;//add
	ultra_pid.kd = 6.666/2;//4.0;//0;
	#else
	ultra_pid.kp = 0.8;//0.45;//0.45;//1.8;//1.65;//1.5;   WT
	ultra_pid.ki = 0.25;//1;//101;//add
	ultra_pid.kd = 9;//4.0;//0;
	#endif
	#if USE_M100
	ultra_pid_head.kp = 0.1;//0.03;//1.8;//1.65;//1.5;   WT
	ultra_pid_head.ki = 0.05;//0.450;//1;//101;//add
	ultra_pid_head.kd = 6;//2.5;//0;
	#else
	ultra_pid_head.kp = 0.06;//0.03;//1.8;//1.65;//1.5;   WT
	ultra_pid_head.ki = 0.02;//0.450;//1;//101;//add
	ultra_pid_head.kd = 8;//2.5;//0;
	#endif
}

#define BARO_SPEED_NUM 10
float baro_speed_arr[BARO_SPEED_NUM + 1];
u16 baro_cnt;
u8 switch_r;
int wz_acc_ukf;
void height_mode_switch(void)
{
static u8 state;
static u16 cnt,cnt1;	

 if(height_ctrl_mode==2){
	 
	 switch(state)
			{ 
			case 0:
			if(ultra_ok==1){
			if(ALT_POS_SONAR*1000>ULTRA_MAX_HEIGHT)
			{state=1;height_ctrl_mode_use=1;cnt=0;}
			else
				height_ctrl_mode_use=2;
			}
			else
			height_ctrl_mode_use=1;
			break;
			case 1:
			if(ALT_POS_SONAR<ULTRA_MAX_HEIGHT-100)
			cnt++;
			else
			cnt=0;

			if(cnt>800)
			{state=0;height_ctrl_mode_use=2;cnt=0;}

			break;
			}

}
 else
 {	height_ctrl_mode_use=height_ctrl_mode;state=0;}
 height_ctrl_mode_use=height_ctrl_mode;
}
u8 cnt_height=19;
float out_timer_high,in_timer_high;
void Height_Ctrl(float T,float thr)
{			static u8 hs_ctrl_cnt;
	static float wz_speed_t;
	static u8 height_ctrl_start_f,height_mode_reg;
	static u16 hc_start_delay;
	float t_sonar_out;
	float temp;
	
	switch( height_ctrl_start_f )
	{		
		case 0:
		temp=(reference_vr[2] *mpu6050.Acc.z + reference_vr[0] *mpu6050.Acc.x + reference_vr[1 ] *mpu6050.Acc.y - 4096  );
		wz_acc_ukf += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc_ukf),25 );
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc),100);//100 );
		
	if(mode.en_h_mode_switch)
				height_mode_switch();
	height_ctrl_mode_use=2;
		 if( height_ctrl_mode_use!=0)
		{ 
	
			hs_ctrl_cnt++;
		  hs_ctrl_cnt = hs_ctrl_cnt%10;
			if(hs_ctrl_cnt == 0)
			{  //----------------------mode switch----------------------
				in_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_I);
//				//heigh thr test
				static u8 state_thr,cnt_down;
				static float thr_reg;
				thr_in_view=thr;
				if(mode.thr_fix_test)//WT
				{  
					switch(state_thr)
					{
						case 0:
							if(thr>525)
							{state_thr=1;thr_reg=500;cnt_down=0;}
							else
							{thr_use=thr;}	
						break;
						case 1:
							if(thr>525){
								if(thr<thr_reg*0.85)
									cnt_down++;
								else
								{thr_use=thr;cnt_down=0;}
								if(cnt_down>3)state_thr=2;
								if(thr>=thr_reg)
								 thr_reg=thr;
							}
							else
								state_thr=0;
							
							
							break;
						case 2:
							thr_use=500;
							if(thr<550)
								state_thr=0;
						break;
					}
				}
				else
				{thr_use=thr;state_thr=0;thr_reg=500;}

				 exp_spd_zv=EXP_Z_SPEED;
			}//---end of speed control
			static u8 cnt_100ms;
			if( cnt_100ms++>=cnt_height )//wt
			{
	      out_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_O);
				cnt_100ms=0;

				Ultra_Ctrl(out_timer_high,thr_use);//超声波周期100ms
				Ultra_Ctrl_Front(out_timer_high,thr_use);
				//Ultra_Ctrl_Safe(0.1f,thr);//超声波周期100ms
				ultra_start_f = -1;
			}
		}

			if(height_ctrl_mode_use)//定高模式
		{	
			if(mode.height_safe)
			height_ctrl_out =  wz_speed_pid_v_safe.pid_out;//LIMIT(thr-50*LIMIT(Moving_Median(9,5,wz_acc_ukf)/4096,-0.2,0.2)*9.8,0,500);	
			else
			height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else//手动模式
		{		
		  	height_ctrl_out = thr-50*LIMIT(wz_acc_ukf/4096,-0.2,0.2)*9.8;   
		}
		
		break; //case 1
		
		default: break;
		
	} //switch
}


///test 
float p1=0.4,p2=0.1;//0.35;//0.3;//WT
u8 speed_ctrl_sel=0;
float wz_speed,wz_speed_old,wz_acc_mms2;
float height_thrv;

u8 baro_ctrl_start;
float baro_height,baro_height_old;

float ultra_sp_test[2];

#define EXP_Z_SPEED_RC  ( 0.4f *my_deathzoom( (Rc_Pwm_Inr_mine[RC_THR]-OFF_RC_THR), 50 ) )
int RC_THR_HIGH;
#include "eso.h"
#include "circle.h"
u16 Pitch_Follow_Dead=40;
void Ultra_Ctrl(float T,float thr)
{
float ultra_sp_tmp,ultra_dis_tmp;	
	#define MID_THR 500 //摇杆中位PWM
if((state_v==SD_HOLD2||state_v==SD_SHOOT)&&!mode.dj_by_hand&&mode.en_hold2_h_off_fix)
	{
	if(circle.connect&&circle.check)
	exp_height_shoot_off+=-circle.control_k*LIMIT(my_deathzoom(PWM_DJ[0]-(PWM_DJ0+SHOOT_PWM_OFF0),Pitch_Follow_Dead),-80,80)*1;	
	exp_height_shoot_off=LIMIT(exp_height_shoot_off,-400-168*0,400+100*1);	
  }
else
  exp_height_shoot_off=0;	
	static float off_m100;

	//模式转换初始化
	if(mode_change){mode_change=0;
	//exp_height=ALT_POS_SONAR2*1000;
	Flow_reset_pos();
	}	
	
  static float sonar_ground;
	if(state_v==SG_LOW_CHECK)
	{off_m100=m100.H;
	sonar_ground=ALT_POS_SONAR2;
	}
	
	if(high_data_sel||m100.refresh==0)
	ultra_dis_tmp=  ALT_POS_SONAR2*1000;
	else
	ultra_dis_tmp=  LIMIT(m100.H-off_m100+sonar_ground,0,20)*1000;	
		

	ultra_dis_lpf=  ultra_dis_tmp;
		
	if(ultra_pid.ki==0||((Rc_Pwm_Inr_mine[RC_THR]<200+1000))||Rc_Pwm_Inr_mine[RC_THR]<450+1000||Rc_Pwm_Inr_mine[RC_THR]>550+1000||pwmin.sel==0)ultra_ctrl.err_i=0;

	ultra_ctrl.err = ( ultra_pid.kp*LIMIT(my_deathzoom(exp_height - ultra_dis_lpf,5),-400,400) );

	ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;

	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-1 *ULTRA_INT,1 *ULTRA_INT);

	ultra_ctrl.err_d = ultra_pid.kd *( 0.0f *(-(ALT_VEL_BMP_EKF*1000)*T) + 1.0f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	
	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;

	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-1000,1000);
		
	ultra_ctrl_out = ultra_ctrl.pid_out;
	
	ultra_ctrl.err_old = ultra_ctrl.err;
}

void Ultra_Ctrl_Front(float T,float thr)
{
float ultra_sp_tmp,ultra_dis_tmp;	
u16 exp_height_head_use;


  if(state_v==SD_HOLD||state_v==SD_HOLD_BACK)  
		exp_height_head=exp_height_head_scan;
	else if(state_v==SU_CHECK_TAR||state_v==SU_TO_CHECK_POS)
	  exp_height_head=exp_height_head_check;
	else if(state_v==SU_MAP1||state_v==SU_MAP2||state_v==SU_MAP3)  
		exp_height_head=exp_height_map;
	else if(state_v==SU_MAP_TO)  
		exp_height_head=exp_height_map_to;
	else if(state_v==SU_TO_START_POS)
		exp_height_head=exp_height_head_scan+250;
	else if(state_v==SD_SHOOT)
		exp_height_head=exp_height_head_shoot;
	else
		exp_height_head=exp_height_head_scan;
	
	
	
	if(mode.test2)
	ultra_dis_tmp=ALT_POS_SONAR_HEAD_LASER_SCANER*1000;
	else
	ultra_dis_tmp=ALT_POS_SONAR_HEAD*1000;//ultra_distance;
			
	if(ultra_pid_head.ki==0
			||ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)>DEAD_NAV_RC
			||ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)>DEAD_NAV_RC
			||(mode.test3==0&&mode.test2==0))
	ultra_ctrl_head.err_i=0;
 
	ultra_ctrl_head.err = ( ultra_pid_head.kp*LIMIT(exp_height_head - ultra_dis_tmp,-500,500) );	
	ultra_ctrl_head.err1 = ( LIMIT(exp_height_head - ultra_dis_tmp,-500,500) );	// state change use
					
	#if USE_M100		
	if(fabs(exp_height_head - ultra_dis_tmp)>200)		
	ultra_ctrl_head.err_i=0;
	#endif			

	ultra_ctrl_head.err_i += ultra_pid_head.ki *ultra_ctrl_head.err *T;

	ultra_ctrl_head.err_i = LIMIT(ultra_ctrl_head.err_i,-1 *ULTRA_INT,1 *ULTRA_INT);

	ultra_ctrl_head.err_d = ultra_pid_head.kd *((ultra_ctrl_head.err - ultra_ctrl_head.err_old) );
	
	ultra_ctrl_head.pid_out = ultra_ctrl_head.err + ultra_ctrl_head.err_i + ultra_ctrl_head.err_d;

	ultra_ctrl_head.pid_out = LIMIT(ultra_ctrl_head.pid_out,-1000,1000);
		
	if(ultra_dis_tmp<6666&&S_head>20)		
	ultra_ctrl_out_head = -LIMIT(ultra_ctrl_head.pid_out*0.4,-120,120);
	else
	ultra_ctrl_out_head=0;	
	ultra_ctrl_head.err_old = ultra_ctrl_head.err;
}
