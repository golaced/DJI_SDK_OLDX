
#include "att.h"
#include "height_ctrl.h"
#include "ultrasonic.h"
#include "flash.h"
#include "pwm_out.h"
#include "pwm_in.h"
#include "alt_kf.h"
#include "circle.h"
ctrl_t ctrl_1;
ctrl_t ctrl_2;
ctrl_t ctrl_2_fuzzy;
// Calculate nav_lat and nav_lon from the x and y error and the speed
#define NAV_ANGLE_MAX 0.8*MAX_CTRL_ANGLE
float dj_angle,dj_angle_offset[3]={-2,3,9},dj_angle_set;
#define MAX_FIX_ANGLE_DJ 13
void AUTO_LAND_FLYUP(float T);
float nav[2];
float GPS_angle[2];
float  target_position[2],target_position_task_s[2],target_position_task_e[2],target_position_task_b[2];
float  now_position[2];
float actual_speed[2];
float tar_speed[2],tar_speed_avoidc[2];
float d_flow_watch[2];
float  integrator[2];
float flow_control_out,flow_k=30;

void Flow_reset_pos(void)
{
	integrator[0]=integrator[1]=0;
	target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
	target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west;
}
#define  TEST_MOVE_RANGE_FLOW 5
void Flow_save_tar_s(void)
{
	target_position_task_s[LON]=now_position[LON];//imu_nav.flow.position.east;
	target_position_task_s[LAT]=now_position[LAT];//imu_nav.flow.position.west;
	
	target_position_task_e[LON]=target_position_task_s[LON]-TEST_MOVE_RANGE_FLOW;//imu_nav.flow.position.east;
	target_position_task_e[LAT]=target_position_task_s[LAT]-TEST_MOVE_RANGE_FLOW;//imu_nav.flow.position.west;
}

void Flow_save_tar_b(void)
{
	target_position_task_b[LON]=now_position[LON];//imu_nav.flow.position.east;
	target_position_task_b[LAT]=now_position[LAT];//imu_nav.flow.position.west;
	
}

void Flow_set_tar(float set)
{
	target_position[1]=set;//imu_nav.flow.position.east;
}

void Ctrl_Para_Init()		//设置默认参数
{
//====================================
	ctrl_1.PID[PIDROLL].kdamp  = 1;
	ctrl_1.PID[PIDPITCH].kdamp = 1;
	ctrl_1.PID[PIDYAW].kdamp 	 = 1;
	
	ctrl_1.FB = 0.20;   //内环  0<fb<1
}

xyz_f_t except_A = {0,0,0};
xyz_f_t except_AR = {0,0,0};
xyz_f_t ctrl_angle_offset = {0,0,0};

xyz_f_t compensation;
#define YAW_ERO_MAX 25
float YAW_NO_HEAD;	
float except_A_SB[2],except_A_SB_lft[2],nav_angle[2];
float scale_lf_sb=5.5;//感度
float scale_lf_nav=5;
float px_v,ix_v;
float yaw_ctrl_out,Yaw_set_dji=0;//178;
void CTRL_2(float T)//角度环
{ 
}

xyz_f_t except_AS;

float g_old[7];
 
void CTRL_1(float T)  //x roll,y pitch,z yaw 角速度  内环  2ms
{
	Thr_Ctrl(T);// 油门控制
#if	 USE_MY_PWM_OUT
	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z);
#endif
}
#include "pwm_in.h"
int baro_to_ground,baro_ground_off;
float thr_value;
u8 Thr_Low,force_Thr_low=0;
float Thr_Weight;
float thr_test;
void Thr_Ctrl(float T)
{	float delta_thr;
	static float thr;
	static float Thr_tmp;
		static u8 cnt_thr_add,fly_ready_r;
	if(!fly_ready)
	   thr=0;		 
	else
    thr = 500 + CH_filter[THRr]; //油门值 0 ~ 1000
	thr=Rc_Pwm_Out_mine[RC_THR]-1000;
//----------Drop protector-----------------
	if(!fly_ready&&500 + CH_filter[THRr]<100)
	force_Thr_low=0;
	if((ABS(Pitch)>45||ABS(Roll)>45)&&fly_ready)
		force_Thr_low=1;
//protect flag init	
	if(fly_ready_r==0&&fly_ready==1&&500 + CH_filter[THRr]>100)
		force_Thr_low=1;
		fly_ready_r=fly_ready;
	
	if(force_Thr_low)
		thr=0;
	
	
	Thr_tmp += 10 *3.14f *T *(thr/400.0f - Thr_tmp); //低通滤波
	Thr_Weight = LIMIT(Thr_tmp,0,1);    							//后边多处分离数据会用到这个值
	
	if( thr < 100 )
	{
		Thr_Low = 1;
	
	}
	else
	{ 
		Thr_Low = 0;
	}
	if( 500 + CH_filter[THRr]<100)	baro_ground_off=ALT_POS_BMP*1000;
	baro_to_ground=LIMIT(ALT_POS_BMP*1000-baro_ground_off,10,8000);
	
	Height_Ctrl(T,thr);
	
	//AUTO_LAND_FLYUP
	AUTO_LAND_FLYUP(T);
	
	
	thr_value = Thr_Weight *height_ctrl_out;   //实际使用值	
	thr_test=thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);
}
/*      HEAD
         Y + (0~240)  1
      1  | 2
    _____|______ X+(0~360) 0
      3  | 4
				 |
*/
#include "rng.h"
u8 init_circle_search;
#define MISS_CIRCLE_CONTROL 25
float nav_land_miss[2];
u8 Rate_Max_num=1;
void circle_search(void)
{
	static float Rate[5];
	int flag[2]={0};
	u8 Rate_locate;
	int Y_c,X_c;
	float out[2];
	static u8 Re_map[5]={0,1,2,3};
	Y_c=circle.y_flp;
	X_c=360-circle.x_flp;
if(init_circle_search)
{
	if(Y_c>120)
		 if(X_c>180)
	   Rate_Max_num=3;
		 else
		 Rate_Max_num=4;	 
	else
	   if(X_c>180)
	   Rate_Max_num=1;
		 else
		 Rate_Max_num=2	; 
	
		 
 switch(Rate_Max_num){
	case 1: Re_map[1]=1;Re_map[2]=2;Re_map[3]=3;break;
	case 2: Re_map[1]=2;Re_map[2]=4;Re_map[3]=1;break;
	case 3: Re_map[1]=3;Re_map[2]=1;Re_map[3]=4;break;
	case 4: Re_map[1]=4;Re_map[2]=3;Re_map[3]=2;break;
 }	 
  
init_circle_search=0;
}

u8 random,temp=0;
random=RNG_Get_RandomRange(0,100);
#define RATE_BIG 60
if(random>RATE_BIG)
	Rate_locate=Re_map[1];
else if(random>(100-RATE_BIG)/2)
	Rate_locate=Re_map[2];
else 
  Rate_locate=Re_map[3];




switch(Rate_locate){
	case 1:  flag[0]=-1;flag[1]=1;break;
	case 2:  flag[0]=1; flag[1]=1;break;
	case 3:  flag[0]=-1;flag[1]=-1;break;
	case 4:  flag[0]=1; flag[1]=-1;break;
 }	
	nav_land_miss[0]=nav_land[ROLr]=out[0]=flag[0]*MISS_CIRCLE_CONTROL*circle.control_k_miss;
  nav_land_miss[1]=nav_land[PITr]=out[1]=flag[1]*MISS_CIRCLE_CONTROL*circle.control_k_miss;
 
 
}


#include "pwm_in.h"
u8 mode_change;
u8 state_pass=0;
u16 AUTO_UP_CUARVE[]={1580,1660,1660,1655,1650,1650,1650,1650,1650};
u16 AUTO_DOWN_CUARVE[]={1500,1500-50,1500-150,1500-150,1500-200,1500-200};
u16 AUTO_DOWN_CUARVE1[]={1500-150,1500-150,1500-100,1500-100,1500-80,1500-80};
#if USE_M100
float SONAR_SET_HIGHT =0.19599;
#else
float SONAR_SET_HIGHT =0.14;
#endif
float AUTO_FLY_HEIGHT =2.5;
float SONAR_CHECK_DEAD =0.1;

float AUTO_LAND_HEIGHT_1= 2.5;// 3.5 //bmp check
float AUTO_LAND_HEIGHT_2= 1.6;//1.8
float AUTO_LAND_HEIGHT_3= 0.0925;

float MINE_LAND_HIGH= 0.3;
float AUTO_LAND_SPEED_DEAD =0.2;
u8 state_v;
u32 cnt[10]={0};
float baro_ground_high;
float nav_land[2];
#define DEAD_NAV_RC 80
#define AVOID_RC 95
#if USE_M100
float AVOID[2]={1.35+0.4,1.35+0.4};
#else
float AVOID[2]={1.35,1.35};
#endif
u8 cnt_shoot=0;
u8 force_check,force_check_pass;
u8 tar_need_to_check_odroid[3]={0,0,66};
u8 tar_buf[20];
u8 tar_cnt;
u8 over_time;
float time_fly;
u8 m100_gps_in=0;
#if USE_M100
#define GPS_ERO 300
#if NAV_ERO_USE_LINE
#define GPS_ERO_S 300
#else
#define GPS_ERO_S 888
#endif
#else
#define GPS_ERO 450
#define GPS_ERO_S 450
#endif
#define NAV_USE_AVOID 1//使用前向避障
#define SHOOT_DIRECT 1//连续射击   
#define CHECK_NUM_DIS_WITH_LASER 1//识别数字时也使用激光定墙
#define USE_OVER_TIME_FOR_STATE 0//使用MAP状态超时
#define CHECK_SCREEN_WHILE_FLYING 0//在巡航状态也检测屏幕
#define USE_MISS_SCAN 1 //使用丢失左右侧飞
#define MAX_TARGET 5//最多射击数字
float dead_pix_check=0.365;//图像对准中心射击触发
float CHECK_OVER_TIME=60;//最长数字识别时间(s)
float MAX_TRACK_TIME=26*2;//对准状态最长时间 (s)
float MAX_SHOOT_TIME=13*2;//8.888;//射击状态最长时间(s)
float SHOOTING_TIME=6;//射击时间(s)
float SHOOTING_TIME_CHECK=17.5;//射击状态图像触发次数(num)
float T_SHOOT_CHECK=0.4;//射击检查时间(s)
float STATE_OVER_TIME_MAX[10]={30,30};//状态超时时间
float FORCE_LAND_TIME=8;//强制着陆时间(min)
float FORCE_HOME_TIME=0.5;//6;//强制返航时间(min)
#define FAKE_TAR_NUM 9
u8 fake_target[FAKE_TAR_NUM]={8,7,6,5,4,3,2,1,0};
u8 fake_target_force=0;
u8 fake_target_flag=0;
u8 shoot_finish_cnt=0;
void AUTO_LAND_FLYUP(float T)
{static u8 state,init,cnt_retry;
 static float sonar_r,bmp_r,bmp_thr;
	static u8 cnt_circle_check=0;
	static u8 state_reg,state_last;
	static u16 thr_sel[3],cnt_miss_track;//跟踪失败
	static u8 flow_head_flag=0;
  static float cnt_back_home,cnt_map_home,cnt_check_over_time,cnt_shoot_over_time;
	static u16 fly_cover_cnt;
	static u8 cnt_first_avoid;
	static int pan_reg[2];
	static u16 cnt_state_over_time,cnt_state_over_time_map;
	static u8 miss_state;
	u8 gimbal_stink_check=1;
	u16 i,j;
	time_fly=cnt_back_home;
	u8 temp_pass=!force_check_pass;
	if(!init&&ALT_POS_BMP!=0){init=1;
		baro_ground_high=ALT_POS_BMP;
	}
	if(Rc_Pwm_Inr_mine[RC_THR]<200+1000)
	{sonar_r=SONAR_SET_HIGHT+0.05;bmp_r=ALT_POS_BMP;}
//state_change 
	switch(state)
	{//-------------------------------------------------起飞
		case SG_LOW_CHECK://low thr check
			cnt_shoot_over_time=cnt_check_over_time=shoot_finish_cnt=cnt_state_over_time_map=miss_state=cnt_state_over_time=fake_target_flag=fly_cover_cnt=cnt_shoot=over_time=cnt_back_home=cnt_map_home=0;tar_need_to_check_odroid[2]=66;
		  qr_pos_off[1]=qr_pos_off[0]=get_qr_pos=mode.use_qr_as_gps_tar=tar_cnt=0;
		  for(i=0;i<19;i++)
		    tar_buf[i]=66;
			mode.auto_land=0;	cnt_retry=0;
			if(mode.auto_fly_up==1&&pwmin.sel_in==1&&ALT_POS_SONAR2<SONAR_SET_HIGHT+SONAR_CHECK_DEAD&&ABS(Pitch)<10&&ABS(Roll)<10){
					if(Rc_Pwm_Inr_mine[RC_THR]<200+1000)
					{state=SG_MID_CHECK;cnt[0]=0;mode_change=1;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;}//to auto fly
				}
			else if(mode.auto_fly_up==0&&pwmin.sel_in==1&&ALT_POS_SONAR2>MINE_LAND_HIGH&&ABS(Pitch)<15&&ABS(Roll)<15){
					if((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000))
					{
						#if defined(DEBUG_TRACK)  
						state=SD_HOLD2;
						#elif defined(DEBUG_HOLD_HEIGHT) 
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_HOLD_WALL) 
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_TARGET_AVOID)
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_MAPPING)
						state=SU_MAP2; Clear_map();
						#elif defined(DEBUG_QR_LAND)
						state=SU_TO_CHECK_POS;
						#else
						state=SU_HOLD;
						#endif
						cnt[0]=0;mode_change=1;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;
					if(gps_data.latitude!=0&&gps_data.longitude!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){
					home_point[0]=gps_data.latitude;
					home_point[1]=gps_data.longitude;}
					}//to hold || land			
				}

					if(state_pass)
					{ state_pass=0;
						#if defined(DEBUG_TRACK)  
						state=SD_HOLD2;
						#elif defined(DEBUG_HOLD_HEIGHT) 
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_HOLD_WALL) 
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_TARGET_AVOID)
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_MAPPING)
						state=SU_MAP2; Clear_map();
						#elif defined(DEBUG_QR_LAND)
						state=SU_TO_CHECK_POS;
						#else
						state=SU_TO_CHECK_POS;
						#endif
			  	}
			break;
		//----------------------------------------------自动起飞
		case SG_MID_CHECK://middle thr check
			if(mode.auto_fly_up==1&&ALT_POS_SONAR2<SONAR_SET_HIGHT+SONAR_CHECK_DEAD&&ABS(Pitch)<15&&ABS(Roll)<15){
					if((Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<600+1000))
						cnt[0]++;
					else cnt[0]=0;
					
					if(cnt[0]>2/T)
					{state=SU_UP1;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;mode_change=1;
					
					if(gps_data.latitude!=0&&gps_data.longitude!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){
					home_point[0]=gps_data.latitude;
					home_point[1]=gps_data.longitude;}
						
					
					}
				}//  
			else if(Rc_Pwm_Inr_mine[RC_THR]<200+1000||mode.auto_fly_up==0)
			{state=SG_LOW_CHECK;mode_change=1;}
			
			if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;mode_change=1;}}
			break;
		case SU_UP1://take off
		if(mode.auto_fly_up==1&&(Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<600+1000)){
				 if(cnt[0]++>2/T||ALT_POS_SONAR2>exp_height_front*0.85)
				 {mode_change=1;
				 #if RISK_MODE
						#if USE_MAP
						state=SU_MAP1;
						Clear_map();
						#else								 
						state=SU_TO_CHECK_POS;
						#endif	 
				 #else 
				 state=SU_HOLD;
				 #endif
				 thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;}	 
	   }
		 else
		 {mode_change=1;state=SU_HOLD;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;} 
		  
		 
		 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		 break;
		case SU_HOLD://keep high
				if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				
							 if(cnt[1]++>4.666*0.618/T)
							 {
							 #if USE_MAP
							 state=SU_MAP1;
							 Clear_map();
               #else								 
							 state=SU_TO_CHECK_POS;
							 #endif	 
							 cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					}
			   
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		break;
    //--------------------------------------MAP--------------------------------------------
	  case SU_MAP1://start pos
			tar_need_to_check_odroid[2]=66;
			if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
								if(ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>0.6/T)
							 {state=SU_MAP2;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}

							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>0.4/T)
						{state=SU_MAP2;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}}
					}
			
			    if(cnt_state_over_time_map++>STATE_OVER_TIME_MAX[0]/T)
						{state=SU_MAP2;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time_map=0;mode_change=1;}
					
					
			    #if USE_OVER_TIME_FOR_STATE
				  //state_over_time
					 if(cnt_state_over_time++>STATE_OVER_TIME_MAX[0]/T)
						{state=SU_MAP2;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time=0;mode_change=1;}
					#endif
					if(state_pass)
					{state_pass=0;state=SU_MAP2;}
					if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 
		break;		
	  case SU_MAP2://end pos
			tar_need_to_check_odroid[2]=66;
			if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
						#if NAV_ERO_USE_LINE
						if(ABS(nav_Data.dis_ero)<GPS_ERO&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
            #else
								if(ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO_S&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*1.618&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
            #endif
								{
							 if(cnt[1]++>0.6/T)
							 {state=SU_TO_CHECK_POS;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}

							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>0.4/T)
						{state=SU_MAP3;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}}
					}
			     #if USE_OVER_TIME_FOR_STATE
					  //state_over_time
					 if(cnt_state_over_time++>STATE_OVER_TIME_MAX[0]/T)
						{state=SU_MAP3;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time=0;mode_change=1;}
					 #endif
					if(state_pass)
					{state_pass=0;state=SU_MAP3;}
					if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 
		break;	
    case SU_MAP3://start pos
			tar_need_to_check_odroid[2]=66;
			if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
						#if NAV_ERO_USE_LINE
						if(ABS(nav_Data.dis_ero)<GPS_ERO&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
						#else
						if(ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO_S&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
						#endif
								{
							 if(cnt[1]++>0.6/T)
							 { 
							 state=SD_TO_HOME;
							 cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}

							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>0.4/T)
						{state=SU_TO_CHECK_POS;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}}
					}
			      #if USE_OVER_TIME_FOR_STATE
					  //state_over_time
					 if(cnt_state_over_time++>STATE_OVER_TIME_MAX[0]/T)
						{state=SU_TO_CHECK_POS;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time=0;mode_change=1;}
						#endif
					if(state_pass)
					{state_pass=0;state=SU_TO_CHECK_POS;}
					if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 
		break;			

    case SU_MAP_TO:
    cnt_retry=0;
		     if((circle.check&&circle.connect)||force_check)
					 cnt_circle_check++;
				 else
					 cnt_circle_check=0;
    	if(cnt_circle_check>3&&
				ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
					if(cnt[4]++>0.4/T)
				 {
					 if(mode.en_flow_break)
					 {state=SD_HOLD_BREAK;Flow_save_tar_b();} 	 
					 else
					 state=SD_HOLD2;
					 cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[0]=0; 
				 
				 flow_head_flag=1;
					
				 #if USE_M100   //warning need test
				 if(thr_sel[2]>5/T)
					 m100_gps_in=1;
				 else if(ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*0.15)
					 thr_sel[2]++;
			
				 #else
				 m100_gps_in=0;
				 #endif
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					 #if NAV_ERO_USE_LINE
					  if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((ABS(nav_Data.dis_ero)<GPS_ERO)||m100_gps_in)))
            #else
					 if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO_S&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S)||m100_gps_in)))
					 #endif
				  {
					  
					 if(cnt[1]++>0.4/T){
					 state=SD_HOLD;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				
					}	else
						cnt[1]=0;
			    }
				 
					 if(thr_sel[1]++>30/T){
					 state=SD_HOLD_BACK;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
					 }
					
					if(state_pass)
					{state_pass=0;state=SD_HOLD;m100_gps_in=fly_cover_cnt=0;}
					 #if USE_OVER_TIME_FOR_STATE
					if(cnt_state_over_time++>STATE_OVER_TIME_MAX[0]/T)
					{state=SD_HOLD;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time=0;mode_change=1;}
						#endif
						
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
         #if !DEBUG_IN_ROOM						
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 #endif
					break; 			
  	//----------------------------------------巡航--------------------------------------------------	 
		
		case SU_TO_CHECK_POS://循航到检测目标位置
					 tar_need_to_check_odroid[2]=66;
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
								if(ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>0.6/T)
							 {
//							 #if defined(DEBUG_QR_LAND)
//						   state=SD_TO_HOME;
//							 #else
							 state=SU_CHECK_TAR;
							 //#endif
							 cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}

							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>0.4/T)
						{state=SU_CHECK_TAR;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}}
					}
         #if CHECK_SCREEN_WHILE_FLYING
         if(((tar_need_to_check_odroid[0]&&tar_need_to_check_odroid[1]!=66&&circle.connect)||force_check_pass)&&
				ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
					   	if(cnt[4]++>0.4/T)
 				         {  
									 state=SU_CHECK_TAR;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;						 
								 }
           }//Add new WT check num while flying
				 }
				 #endif 
					if(state_pass)
					{state_pass=0;
					#if defined(DEBUG_QR_LAND)
					state=SD_TO_HOME;	
					#else
					state=SU_CHECK_TAR;
					#endif
					}
					#if defined(DEBUG_TARGET_AVOID)
					state=SU_CHECK_TAR;	
					#endif
					#if defined(DEBUG_TARGET_AVOID)
					#else
					if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					#endif
				 #if !DEBUG_IN_ROOM
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 #endif
		break; 
		case SU_CHECK_TAR://检测目标
					#if defined(DEBUG_HOLD_WALL) 
					force_check_pass=0;
					#elif defined(DEBUG_HOLD_HEIGHT) 
					force_check_pass=0;
					#elif defined(DEBUG_TARGET_AVOID)
					force_check_pass=0;
					#endif
		  
    	if(((tar_need_to_check_odroid[0]&&tar_need_to_check_odroid[1]!=66&&circle.connect)||force_check_pass)&&
				ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
					if(cnt[4]++>0.4/T)
 				 {  
					  #if !MISSION_USE_FAKE_TARGET
					  tar_need_to_check_odroid[2]=tar_need_to_check_odroid[1];
					  #endif
					  #if defined(DEBUG_TARGET_AVOID)
					  tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
					  state=SU_TO_CHECK_POS;
					  #else				  
					  for(i=0;i<19;i++){
					  if(tar_need_to_check_odroid[2]==tar_buf[i])
						{state=SU_TO_CHECK_POS;break;}	
					  else
					  #if USE_MAP
						if(target_map[tar_need_to_check_odroid[2]][0]!=0&&target_map[tar_need_to_check_odroid[2]][1]!=0)
						state=SU_MAP_TO;
            else		
            state=SU_TO_START_POS;							
						#else	
					  state=SU_TO_START_POS;
						#endif
						}
					  #endif
					 cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
					 miss_state=1;
					 }} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[4]=0; 
		    //---------------------------无屏幕流程测试
				 #if MISSION_USE_FAKE_TARGET
				 if(ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
						if(fake_target_force!=0)
						tar_need_to_check_odroid[2]=fake_target_force;
            else						
				    tar_need_to_check_odroid[2]=fake_target[fake_target_flag++];
						
					  #if defined(DEBUG_TARGET_AVOID)
					  tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
					  state=SU_TO_CHECK_POS;
					  #else				  
					  for(i=0;i<19;i++){
					  if(tar_need_to_check_odroid[2]==tar_buf[i])
						{state=SU_TO_CHECK_POS;break;}	
					  else
					  #if USE_MAP
						if(target_map[tar_need_to_check_odroid[2]][0]!=0&&target_map[tar_need_to_check_odroid[2]][1]!=0)
						state=SU_MAP_TO;		
						else
						state=SU_TO_START_POS;
						#else	
					  state=SU_TO_START_POS;
						#endif
						miss_state=1;
						}
					  #endif
					  cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
						
						if(fake_target_flag>FAKE_TAR_NUM)	
						{
						state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
						}
					  } 
				  	}

				 #endif
						
					if(cnt_check_over_time>CHECK_OVER_TIME){cnt_check_over_time=0;
          shoot_finish_cnt++;
				  }
				  #if defined(DEBUG_HOLD_WALL) 
					#elif defined(DEBUG_HOLD_HEIGHT) 
				  #elif defined(DEBUG_TARGET_AVOID)
					#else
				 	if(cnt[1]++>666.6666/T)
						{state=SU_TO_START_POS;
							cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
							//--- out_timer_random
							   tar_need_to_check_odroid[2]=2;
								 for(j=0;j<500;j++){		
									 int tar_temp=	RNG_Get_RandomRange(0,9);
									 u8 flag1=0;
											for(i=0;i<19;i++){
											if(tar_temp==tar_buf[i]){
												flag1=1;break;}
											}	
											
											if(!flag1)
											{tar_need_to_check_odroid[2]=tar_temp;break;}
											else
											flag1=0;
								 }
									//
								
						}
					#endif
					#if defined(DEBUG_TARGET_AVOID)
					#else
				 if(over_time==1||shoot_finish_cnt>=MAX_TARGET)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					#endif
				 #if !DEBUG_IN_ROOM
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 #endif
					break; 		 
				 
		case SU_TO_START_POS://巡航到作业起点位置8
					 
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
								if(ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*1.618&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>0.4/T)
							 {state=SD_HOLD;m100_gps_in=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1; gps_target_change=1;
							 tar_now_gps[0]=tar_point_globle[0];
							 tar_now_gps[1]=tar_point_globle[1];
							 Flow_save_tar_s();}

							}	else
								cnt[1]=0;
							} 
						else//not gps
						{
						if(cnt[1]++>2.5/T)//默认测飞	
						{state=SD_HOLD;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;Flow_save_tar_s();}}
					}
			      #if USE_OVER_TIME_FOR_STATE
					 if(cnt_state_over_time++>STATE_OVER_TIME_MAX[0]/T)
					 {state=SD_HOLD;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time=0;mode_change=1;}
						#endif
					
					 if(thr_sel[1]++>14.6/T){
					 state=SU_TO_CHECK_POS;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
					 }
					 
						if(state_pass)
					{state_pass=0;state=SD_HOLD;m100_gps_in=fly_cover_cnt=0;}
					
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
         #if !DEBUG_IN_ROOM						
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 #endif
		break; 		 
				 
				 
		//-----------------------------------------对目标---------------------------------------------
    case SD_HOLD://正飞直到看到目标
			
		cnt_retry=0;
		     if((circle.check&&circle.connect)||force_check)
					 cnt_circle_check++;
				 else
					 cnt_circle_check=0;
    	if(cnt_circle_check>3&&
				ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
					if(cnt[4]++>0.4/T)
				 {
					 if(mode.en_flow_break)
					 {state=SD_HOLD_BREAK;Flow_save_tar_b();} 	 
					 else
					 state=SD_HOLD2;
					 cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[0]=0; 
				 
				 flow_head_flag=1;
					
				 #if USE_M100   //warning need test
				 if(thr_sel[2]>5/T)
					 m100_gps_in=1;
				 else if(ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*0.15)
					 thr_sel[2]++;
			
				 #else
				 m100_gps_in=0;
				 #endif
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					#if NAV_ERO_USE_LINE
					  if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((ABS(nav_Data.dis_ero)<GPS_ERO)||m100_gps_in)))
            #else
					 if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO_S&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*1.618)||m100_gps_in)))
				  #endif
					 {
					  
					 if(cnt[1]++>0.4/T){//&&(tar_point_globler[0]!=tar_point_globle[0])&&(tar_point_globler[1]!=tar_point_globle[1])){
					 state=SD_HOLD_BACK;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				   if(miss_state==1)miss_state=2;
					}	else
						cnt[1]=0;
			    }
				    #if USE_OVER_TIME_FOR_STATE
					 if(cnt_state_over_time++>STATE_OVER_TIME_MAX[1]/T)
					 {state=SD_HOLD_BACK;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time=0;mode_change=1;
					   if(miss_state==1)miss_state=2;
					 }
						#endif
					 if(thr_sel[1]++>30/T){
					 state=SD_HOLD_BACK;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
						  if(miss_state==1)miss_state=2;
					 }
					
					if(state_pass)
					{state_pass=0;state=SD_HOLD_BACK;m100_gps_in=fly_cover_cnt=0;
					 if(miss_state==1)miss_state=2;
					}
					
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 
		case SD_HOLD_BACK://倒飞直到看到目标
			
		cnt_retry=0;
		     if((circle.check&&circle.connect)||force_check)
					 cnt_circle_check++;
				 else
					 cnt_circle_check=0;
    	if(cnt_circle_check>3&&
				ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
							if(cnt[4]++>0.4/T)
							{
									if(mode.en_flow_break)//&&ABS(PWM_DJ[1]-PWM_DJ1)<80)//倒着飞云台中位附近才光流制动
									{state=SD_HOLD_BREAK;Flow_save_tar_b();} 	 
									else
									state=SD_HOLD2;
									cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
							}
					 } 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[0]=0; 
				 
				 flow_head_flag=0;
				 
				 #if USE_M100
				 if(thr_sel[2]>5/T)
					 m100_gps_in=1;
				 else if(ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*0.15)
					 thr_sel[2]++;
			
				 #else
				 m100_gps_in=0;
				 #endif
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
						#if NAV_ERO_USE_LINE
					  if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((ABS(nav_Data.dis_ero)<GPS_ERO)||m100_gps_in)))
            #else
					 if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO_S&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S)||m100_gps_in)))
				    #endif
					 {
					  if(cnt[1]++>0.34/T)
						{state=SU_TO_CHECK_POS;m100_gps_in=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					   
					 } else
							cnt[1]=0;
			    }
				 	if(state_pass)
					{state_pass=0;state=SU_TO_CHECK_POS;}
					  #if USE_OVER_TIME_FOR_STATE
						if(cnt_state_over_time++>STATE_OVER_TIME_MAX[1]/T)
					 {state=SU_TO_CHECK_POS;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt_state_over_time=0;mode_change=1;}
						#endif
					 if(thr_sel[1]++>30/T){
					 state=SU_TO_CHECK_POS;m100_gps_in=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
					 }
					
         if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}					 
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 		 
		case SD_HOLD_BREAK://光流刹车
			
    	if(ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&(ABS(Pitch)<5&&ABS(Roll)<5))
					 {
					if(cnt[4]++>1.5/T)
				 {state=SD_HOLD2;cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[4]=0; 
				 
			   if(cnt[1]++>2.5/0.005)
					{state=SD_HOLD2;cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 	
				 
    case SD_HOLD2://云台跟踪目标控制 <---------------------------------------------------------
		
		cnt_retry=0;
    	if(mode.en_shoot&&ABS(PWM_DJ[1]-PWM_DJ1)<SHOOT_PWM_DEAD1&&ABS(PWM_DJ[0]-(PWM_DJ0+SHOOT_PWM_OFF0))<SHOOT_PWM_DEAD0&&
			   #if !DEBUG_IN_ROOM
			    (ABS(ultra_ctrl_head.err1)<168)&&
			   #endif
					((circle.check &&circle.connect)||force_check)&&
						ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC)//跟踪到位
				{
				 if(1){
					 if(cnt[4]++>T_SHOOT_CHECK/T)
				 {state=SD_SHOOT;cnt_shoot_over_time=cnt_shoot=cnt_miss_track=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   	}
			 else
					cnt[4]=0; 
			//WT Track Overtime
			#if !DEBUG_IN_ROOM
			if(ABS(ultra_ctrl_head.err1)<168)
      #endif				
			   cnt_shoot_over_time+=T;
			if(cnt_shoot_over_time>MAX_TRACK_TIME)
				EN_SHOOT_D(1);
			if(cnt_shoot_over_time>MAX_TRACK_TIME+SHOOTING_TIME*0.6) 
			{
			EN_SHOOT_D(0);
			state=SU_TO_CHECK_POS;
			shoot_finish_cnt++;	
			tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
			cnt_shoot_over_time=cnt_shoot=cnt_miss_track=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
			}
			
			if(ABS(PWM_DJ[1]-1500)>1.618*DEAD_NAV_RC) 
			pan_reg[1]=PWM_DJ[1]-1500;
			 
			if(mode.en_track_forward)
			{    if(cnt_miss_track++>2/0.005)
							{
							#if defined(DEBUG_TRACK)  
							#else
							u8 flag1,flag_use;
              if(pan_reg[1]>0)
							flag1=1;
              else
							flag1=0;	
								//<---------------丢失策略
							#if USE_MISS_SCAN
							if(miss_state==1)//check to right
							{flag_use=flag1;miss_state=3;}
							else
							flag_use=flow_head_flag;
							#else
							flag_use=flow_head_flag;
							#endif
							if(flag_use)
							state=SD_HOLD;
							else 
							state=SD_HOLD_BACK;	
							#endif
							#if SHOOT_DIRECT 
							EN_SHOOT_D(0);
							#endif
							cnt_shoot_over_time=fly_cover_cnt=cnt_miss_track=cnt_shoot=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			}
			else
				cnt_miss_track=0;
			
			if((circle.check&&circle.connect)||force_check)
				cnt_miss_track=0;

			
			if(over_time==1)
			{			
			#if SHOOT_DIRECT 
			EN_SHOOT_D(0);
			#endif
			state=SD_TO_HOME;
			cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			else if(over_time==2)
			{
			#if SHOOT_DIRECT 
			EN_SHOOT_D(0);
			#endif
			state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			#if !DEBUG_IN_ROOM
			if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){			
			#if SHOOT_DIRECT 
			EN_SHOOT_D(0);
			#endif
			mode_change=1;state=SD_SAFE;cnt[3]=0;}}
			#endif
		break;
		case SD_SHOOT://射击模式
		#if SHOOT_USE_YUN
		gimbal_stink_check=1;
		#else
		  if(gimbal_stink){
				if(ABS(my_deathzoom_2(circle.x-160,8))<160*dead_pix_check)
				gimbal_stink_check=1;	
				else
				gimbal_stink_check=0;
			}
			else
				gimbal_stink_check=1;
		#endif
		if(mode.en_shoot&&ABS(PWM_DJ[1]-PWM_DJ1)<SHOOT_PWM_DEAD1&&ABS(PWM_DJ[0]-(PWM_DJ0+SHOOT_PWM_OFF0))<SHOOT_PWM_DEAD0&&
			(ABS(Pitch)<16&&ABS(Roll)<16)&&
		  gimbal_stink_check&&
		  ((circle.check &&circle.connect)||force_check)&&
						ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC)//跟踪到位
				{
					if(cnt[4]++>(T_SHOOT_CHECK)/T)
					{
						en_shoot=1;cnt[4]=cnt[1]=0;thr_sel[1]++;
						#if SHOOT_DIRECT 					 
						EN_SHOOT_D(1);
						#endif
					}

							if(thr_sel[1]>SHOOTING_TIME_CHECK)
							{
								#if defined(DEBUG_TRACK)  
								#else 	
									state=SU_TO_CHECK_POS;
								  shoot_finish_cnt++;		
								#endif
								cnt_shoot=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;								
								tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
								#if SHOOT_DIRECT 
									EN_SHOOT_D(0);
								#endif
							}			
				}//end if check 
				else
			  {cnt[0]=0;cnt[4]=0;}
			 
			 if(ABS(PWM_DJ[1]-1500)>1.618*DEAD_NAV_RC) 
			  pan_reg[1]=PWM_DJ[1]-1500;
			 
			 if(mode.en_track_forward)
			{    if(cnt_miss_track++>2/0.005)//loss target
							{state=SD_HOLD2;fly_cover_cnt=cnt_shoot=cnt_miss_track=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
							#if SHOOT_DIRECT 
							 EN_SHOOT_D(0);
							#endif
							}
			}
			else
				cnt_miss_track=0;
			//shoot over time
			thr_sel[2]++;
			if(thr_sel[2]>MAX_SHOOT_TIME/0.005)
				EN_SHOOT_D(1);
			if(thr_sel[2]>(MAX_SHOOT_TIME+SHOOTING_TIME*0.6)/0.005)
				{
				#if defined(DEBUG_TRACK)  
			  #else	
				state=SU_TO_CHECK_POS;
				shoot_finish_cnt++;	
				#endif		
				cnt_shoot=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
				tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
				#if SHOOT_DIRECT 
				EN_SHOOT_D(0);
				#endif
				}
			//loss target	
			if((circle.check&&circle.connect)||force_check)
				cnt_miss_track=0;
			if(mode.dj_by_hand){state=SD_HOLD2;cnt_shoot=en_shoot=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
			#if SHOOT_DIRECT 
			EN_SHOOT_D(0);
			#endif
			} 
			
			if(over_time==1||shoot_finish_cnt>=MAX_TARGET)
			{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
			#if SHOOT_DIRECT 
			EN_SHOOT_D(0);
			#endif
			}
			else if(over_time==2)
			{state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
			#if SHOOT_DIRECT 
			EN_SHOOT_D(0);
			#endif
			}
			#if !DEBUG_IN_ROOM
			if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt_shoot=en_shoot=cnt[3]=0;
			#if SHOOT_DIRECT 
			EN_SHOOT_D(0);
			#endif
			}}
			#endif
			
		break;	
		//-------------------------------------自动降落
			case SD_TO_HOME://巡航到作业起点位置
					 
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
								if(ABS(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&ABS(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>1.2/T)
							 {
								 
							 if(get_qr_pos&&ABS(qr_local_pos[2]<100)&&qr.connect&&mode.en_qr_land)
							 state=SU_TO_QR_FIRST;
							 else
							 state=SD_CIRCLE_MID_DOWN;
				 
							 cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}     
							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>2.5/T)//
						{state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}}
					}

				    if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	 
				 #if !DEBUG_IN_ROOM		
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
				 #endif
		break; 		 
	  case SU_TO_QR_FIRST://巡航到第一次看到qr的GPS位置8
					 if(mode.qr_cal_by_px)
						 mode.use_qr_as_gps_tar=1;
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&ABS((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&ABS((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){//use now
								if(((qr.check&&qr.connect)||//过程中看见
									(fabs(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO))&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>0.45*2/T)//开始下降
							 {state=SD_CIRCLE_MID_DOWN;m100_gps_in=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1; gps_target_change=1;
							 tar_now_gps[0]=tar_point_globle[0];
							 tar_now_gps[1]=tar_point_globle[1];
							 mode.use_qr_as_gps_tar=1;//使能QR作为目标
							 }

							}	else
								cnt[1]=0;
							} 	
					}
			
					if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	 
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		break; 	

				 
		case SD_CIRCLE_MID_DOWN://   在圆死区内中速下降
			if(((int)Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&((int)Rc_Pwm_Inr_mine[RC_THR]<600+1000)){
				 if(ALT_POS_SONAR2<SONAR_SET_HIGHT+0.25)//&&ABS(ALT_POS_BMP-bmp_r)<0.866)//Sonar check 0.5m
				 {if(cnt[4]++>1/T)
					{state=SD_CHECK_G;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt[2]=0;mode_change=1;}
				 }
				 else
					 cnt[4]=0;
				}
       
			 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}//restart until land
				break;
		case SD_CHECK_G://shut motor  电机停转检测
			if(ABS(ALT_VEL_SONAR)<AUTO_LAND_SPEED_DEAD&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.25)//
				cnt[2]++;
			else
				cnt[2]=0;
			if(cnt[2]>1.25/T)
			{cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[2]=cnt[1]=0;mode_change=1;state=SD_SHUT_DOWN;}
			
			if(pwmin.sel_in==0){mode_change=1;state=SD_SAFE;}
		break;
		case SD_SHUT_DOWN://reset
    if((Rc_Pwm_Inr_mine[RC_THR]<200+1000)&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.3)
		{state=SG_LOW_CHECK;cnt_retry=0;}
		break;
		//------------------------------------SAFE------------------------------------------------
		case SD_SAFE://safe out
			cnt_retry=0;mode_change=1;flow_head_flag=0;
		if(mode.auto_fly_up==0&&(Rc_Pwm_Inr_mine[RC_THR]<200+1000)&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.25)
		state=SG_LOW_CHECK;	
		break;
		default:{mode_change=1;state=SD_SAFE;}break;
	}

	if(state_reg!=state)state_last=state;
	state_reg=state;
	
#if defined(DEBUG_HOLD_WALL) 
mode.en_rth_mine=0;
#elif defined(DEBUG_TRACK) 
mode.en_rth_mine=0;
#elif defined(DEBUG_HOLD_HEIGHT) 	
mode.en_rth_mine=0;
#elif defined(DEBUG_TARGET_AVOID) 	
mode.en_rth_mine=0;	
#endif	
//超时处理	
	#if SHOOT_DIRECT 
	if(state==SD_SHOOT||state==SG_LOW_CHECK||state==SD_HOLD2)
			;
	else
	    EN_SHOOT_D(0);
	#endif
	if(state==SU_CHECK_TAR)
	cnt_check_over_time+=T;
	else
	cnt_check_over_time=0;
	if(mode.en_rth_mine){
	if(state!=SG_LOW_CHECK)
	{cnt_back_home+=T;cnt_map_home+=T;}
	#if USE_MAP
	if(cnt_map_home>1*60)//超时降落	
	{
	  if(state==SU_MAP1||state==SU_MAP2||state==SU_MAP3)
		{state=SU_TO_CHECK_POS;cnt_map_home=0;}	
	}
	#endif
	if(cnt_back_home>FORCE_LAND_TIME*60)//超时降落	
	{over_time=2;}
	else if(cnt_back_home>FORCE_HOME_TIME*60)//超时间返航
	{over_time=1;}
	}
	
	#if USE_M100
  k_m100[4]=k_m100_laser_avoid;
	#else
	k_m100[4]=1;
	#endif	  
//-----------------------NAV_OutPut--------------------
		switch(state){
			case SU_TO_CHECK_POS://导航到检查点
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(ABS(ctrl_2.err.z)<2.5&&(ABS(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
					
						tar_point_globle[0]=check_way_point[0]; tar_point_globle[1]=check_way_point[1];
						nav_land[PITr]=LIMIT(nav_gps[PITr],-120,100);
						nav_land[ROLr]=nav_gps[ROLr];
					#if NAV_USE_AVOID 
					// 	if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					//else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					//if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					#endif	
				 }
			break;
			
			//----------------------------MAP
      case SU_MAP1://MAP导航到起始点
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(ABS(ctrl_2.err.z)<2.5&&(ABS(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
					
						tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];
						nav_land[PITr]=LIMIT(nav_gps[PITr],-120*0.816,80*0.816);
						nav_land[ROLr]=nav_gps[ROLr];
					#if	NAV_USE_AVOID
					 if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				  #endif
				 }
			break;	 				 
			case SU_MAP2://正飞
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(ABS(ctrl_2.err.z)<2.5&&(ABS(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else{
					if(mode.en_gps1)	
					{nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
						
					}
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else  if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
				  tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];
					 
					if(fly_cover_cnt++>1.5/T){ fly_cover_cnt=5/T;
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
					}
					else
					{
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-100,0);
						else	
						nav_land[PITr]=0;
					}
						
					if((ABS(ultra_ctrl_head.err1)<1000))		
			    nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward*0.816,track.forward*0.816);
					 
					 	if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
			break;
			case SU_MAP3://倒着飞 for track	
				if(!mode.en_gps){
			    if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))
					{
					Flow_set_tar(target_position_task_s[1]*2);		
					if(mode.en_dji_yaw&&(ABS(ultra_ctrl_head.err1)<120))
					{if(ABS(ctrl_2.err.z)<2.5)	
							nav_land[ROLr]=-track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					{	
					if(mode.en_gps1)	
					{nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];}
					else	
					nav_land[ROLr]=-track.forward;//flow_control_out;
					}
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				}
				else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				{
				 tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];
					if(fly_cover_cnt++>1.5/T){ fly_cover_cnt=5/T;
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
					}
					else
					{
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-100,0);
						else	
						nav_land[PITr]=0;
					}
					
					if((ABS(ultra_ctrl_head.err1)<1000))	
			    nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward*0.816,track.forward*0.816);
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
				}
			break;	 
			case SU_MAP_TO:
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(ABS(ctrl_2.err.z)<2.5&&(ABS(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else{
					if(mode.en_gps1)	
					{nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
						
					}
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else  if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
					if(target_map[tar_need_to_check_odroid[2]][0]!=0&&target_map[tar_need_to_check_odroid[2]][1]!=0)
					{
				  tar_point_globle[0]=target_map[tar_need_to_check_odroid[2]][0]; 
					tar_point_globle[1]=target_map[tar_need_to_check_odroid[2]][1];//set
					}
					else{
				  tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];//set
					}
					
					if(fly_cover_cnt++>1.5/T){ fly_cover_cnt=5/T;
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
					}
					else
					{
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-100,0);
						else	
						nav_land[PITr]=0;
					}
					
          //nav_land[PITr]=LIMIT(nav_gps[PITr],-120,100);//new-----for map WT use GPS nav not laser head control	
	        if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;					
					if((ABS(ultra_ctrl_head.err1)<1000))		
			    nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);
					 
					 	if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }	
			break;
				 
				 
			//-----------------------------------------MISSION	 
			case SU_TO_START_POS://导航到起始点
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(ABS(ctrl_2.err.z)<2.5&&(ABS(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
					
						tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];
						nav_land[PITr]=LIMIT(nav_gps[PITr],-120,100);
						nav_land[ROLr]=nav_gps[ROLr];
					#if	NAV_USE_AVOID
					 if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				  #endif
				 }
			break;	 
				 
				case SU_CHECK_TAR://检查数字
				  if(!mode.en_gps){
					
						if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
						nav_land[ROLr]=0;
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
						#if defined(DEBUG_HOLD_WALL)
						if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
						nav_land[ROLr]=0;					 
						#elif defined(DEBUG_HOLD_HEIGHT) 
						if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
						nav_land[ROLr]=0;
						#else
						
						tar_point_globle[0]=check_way_point[0]; tar_point_globle[1]=check_way_point[1];
						
						//if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&ultra_ctrl_out_head!=0&&ALT_POS_SONAR_HEAD>1.88)//前向壁障
						//nav_land[PITr]=ultra_ctrl_out_head;
						//else	
						//nav_land[PITr]=nav_gps[PITr];
						#if CHECK_NUM_DIS_WITH_LASER
							if(S_head<20)
							nav_land[PITr]=LIMIT(nav_gps[PITr],-120,100);	
							else
							nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-120,100);
						#else
							nav_land[PITr]=LIMIT(nav_gps[PITr],-120,100);
						#endif
					  //---------------------------wait for test  检查点使用前向壁障
						#if !NAV_USE_AVOID	
							nav_land[PITr]=nav_gps[PITr];
						#endif
						nav_land[ROLr]=nav_gps[ROLr];
					#endif	
						
					#if NAV_USE_AVOID	
					// if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					//else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					//if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				  #endif
				 }
			break;	 	 
				 
				 
			//
			case SD_HOLD://正飞
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(ABS(ctrl_2.err.z)<2.5&&(ABS(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else{
					if(mode.en_gps1)	
					{nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
						
					}
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else  if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
				  tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];
					 
					if(fly_cover_cnt++>1.5/T){ fly_cover_cnt=5/T;
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
					}
					else
					{
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-100,0);
						else	
						nav_land[PITr]=0;
					}
						
					if((ABS(ultra_ctrl_head.err1)<1000))		
			    nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);
					 
					 	if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
			break;
			case SD_HOLD_BACK://倒着飞 for track	
				if(!mode.en_gps){
			    if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))
					{
					Flow_set_tar(target_position_task_s[1]*2);		
					if(mode.en_dji_yaw&&(ABS(ultra_ctrl_head.err1)<120))
					{if(ABS(ctrl_2.err.z)<2.5)	
							nav_land[ROLr]=-track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					{	
					if(mode.en_gps1)	
					{nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];}
					else	
					nav_land[ROLr]=-track.forward;//flow_control_out;
					}
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				}
				else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				{
				 tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];
					if(fly_cover_cnt++>1.5/T){ fly_cover_cnt=5/T;
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
					}
					else
					{
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-100,0);
						else	
						nav_land[PITr]=0;
					}
					
					if((ABS(ultra_ctrl_head.err1)<1000))	
			    nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
				}
			break;
      case SD_HOLD_BREAK://刹车wa  效果不理想
			    if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
					if((!mode.dj_by_hand&&mode.hold_use_flow))
					{Flow_set_tar(target_position_task_b[1]);	
					 nav_land[ROLr]=LIMIT(flow_control_out,-track.forward*1.5,track.forward*1.5);	}
					else
					{nav_land[ROLr]=0;}
					
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
			break;					
			case SD_HOLD2:
					
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
					if(!mode.dj_by_hand&&circle.connect&&circle.check&&mode.en_track_forward)
					{ 
						nav_land[ROLr]=PWM_DJ[2];//云台
					}
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))//光流
					nav_land[ROLr]=flow_control_out;
					else 
					{nav_land[ROLr]=0;}
					
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
			break;
			case SD_SHOOT://发射模式
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
					if(!mode.dj_by_hand&&circle.connect&&circle.check&&mode.en_track_forward)
					{ 
						nav_land[ROLr]=PWM_DJ[2];//云台
					}
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))//光流
					nav_land[ROLr]=flow_control_out;
					else 
					{nav_land[ROLr]=0;}
					
				
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
			break;	


	     case SD_TO_HOME://导航到home
				  if(!mode.en_gps){
					
					nav_land[PITr]=nav_land[ROLr]=0;
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)
				 {
					
						tar_point_globle[0]=home_point[0]; tar_point_globle[1]=home_point[1];
						nav_land[PITr]=nav_gps[PITr];
						nav_land[ROLr]=nav_gps[ROLr];
						
					 //if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					//else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					//if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
			break;	 	

	    case SU_TO_QR_FIRST://导航到第一次看到QR处
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(fabs(ctrl_2.err.z)<2.5&&(fabs(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
					
						tar_point_globle[0]=qr_gps_pos[0]; tar_point_globle[1]=qr_gps_pos[1];
						nav_land[PITr]=nav_gps[PITr];
						nav_land[ROLr]=nav_gps[ROLr];
					#if	NAV_USE_AVOID
					 if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				  #endif
				 }
			break;	
//circe track
	     case SD_CIRCLE_MID_DOWN://
				  if(!mode.en_gps){
					
					nav_land[PITr]=nav_land[ROLr]=0;
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)
				 {				
					  tar_point_globle[0]=qr_gps_pos[0]; tar_point_globle[1]=qr_gps_pos[1];
						nav_land[PITr]=nav_gps[PITr];
						nav_land[ROLr]=nav_gps[ROLr];
				 }
			break;	 							 
			default:	
					{nav_land[0]=	nav_land[1]=0;}
			break;
		}

state_v=state;
	#if USE_MAP		
	if(state>SG_LOW_CHECK&&(state==SU_MAP1||state==SU_MAP2||state==SU_MAP3))
		map_builder();
	#endif
	#if USE_M100
  k_m100[2]=k_m100_gps[2];
	#else
	k_m100[2]=1;
	#endif
u16 Heigh_thr=LIMIT(ultra_ctrl_out*0.4,-100,100)*k_m100[2]+OFF_RC_THR;
//-----------------state_out thr-----------------------
	switch(state)
	{
		case SG_LOW_CHECK:Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];break;
		case SG_MID_CHECK:Rc_Pwm_Out_mine[RC_THR]=LIMIT(Rc_Pwm_Inr_mine[RC_THR],0,1450);break;		
		case SU_UP1://load curve
			   exp_height=exp_height_check;
			if(cnt[1]++>1/T)
			{thr_sel[0]++;cnt[1]=0;}
			Rc_Pwm_Out_mine[RC_THR]=AUTO_UP_CUARVE[thr_sel[0]];
		  break;
		case SU_HOLD://keep height
				exp_height=exp_height_check;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;
		//MAP	
			case SU_MAP1://keep height
				exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;	
			case SU_MAP2://keep height
				exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;
			case SU_MAP3://keep height
				exp_height=exp_height_back;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;			
      case SU_MAP_TO://keep height
				exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;				
		//	
			case SU_TO_CHECK_POS://keep height
				exp_height=exp_height_check;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;	
			case SU_CHECK_TAR://keep height
				exp_height=exp_height_check;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;	
			case SU_TO_START_POS://keep height
				exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;	
			
		//--------------------------------------目标对准---------------------------------------------
    case SD_HOLD://keep height
			exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_HOLD_BACK://keep height
			exp_height=exp_height_back;
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_HOLD_BREAK://keep height
//		if(flow_head_flag)exp_height=exp_height_back;//重复之前的方向
//					 else exp_height=exp_height_front;
		
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;			
		case SD_HOLD2://keep height
		if(!flow_head_flag)exp_height=LIMIT(exp_height_back+exp_height_shoot_off,500,2222);//重复之前的方向
					 else exp_height=LIMIT(exp_height_front+exp_height_shoot_off,500,2222);	
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_SHOOT://keep height
		if(!flow_head_flag)exp_height=LIMIT(exp_height_back+exp_height_shoot_off,500,2222);//重复之前的方向
					 else exp_height=LIMIT(exp_height_front+exp_height_shoot_off,500,2222);	
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		//------------------------自动下降
		case SD_TO_HOME://keep height
				exp_height=exp_height_home;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SU_TO_QR_FIRST://keep height
				exp_height=exp_height_home;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;	
		case SD_CIRCLE_MID_DOWN://land check
			#if USE_M100
		  Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-100*0.8;
		  #else
			Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-60;
		  #endif
		break;
		case SD_CHECK_G://shut motor
				#if USE_M100
		  Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-100;
		  #else
			Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-60;
		  #endif
		break;
		case SD_SHUT_DOWN://reset
      Rc_Pwm_Out_mine[RC_THR]=0+1000;
		break;
		//----------------------------
		case SD_SAFE://safe out
//			if(mode.en_dji_h)//&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
//			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
//			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;
		default:Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];break;
	}
Rc_Pwm_Out_mine[RC_THR]=LIMIT(Rc_Pwm_Out_mine[RC_THR],pwmin.min,pwmin.max);


static u8 pwmin_selr,en_pid_r;	
	if(pwmin.sel_in!=pwmin_selr)
	mode_change=1;
	
	if(mode.en_dji_h!=en_pid_r)
	mode_change=1;
	
 en_pid_r=mode.en_dji_h;	
 pwmin_selr= pwmin.sel_in;	
}






float motor[MAXMOTORS];
float posture_value[MAXMOTORS];
float curve[MAXMOTORS];
#define MAX_THR_FIX_ANGLE MAX_CTRL_ANGLE
void All_Out(float out_roll,float out_pitch,float out_yaw)
{
	s16 motor_out[MAXMOTORS];
	u8 i;
	float posture_value[MAXMOTORS];
  float curve[MAXMOTORS];
	static float motor_last[MAXMOTORS];

	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%
	
	posture_value[0] = - out_roll + out_pitch + out_yaw ;
	posture_value[1] = + out_roll + out_pitch - out_yaw ;
	posture_value[2] = + out_roll - out_pitch + out_yaw ;
	posture_value[3] = - out_roll - out_pitch - out_yaw ;
	
	for(i=0;i<4;i++)
	{
		posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
	}
	
	curve[0] = (0.55f + 0.45f *ABS(posture_value[0])/1000.0f) *posture_value[0] ;
	curve[1] = (0.55f + 0.45f *ABS(posture_value[1])/1000.0f) *posture_value[1] ;
	curve[2] = (0.55f + 0.45f *ABS(posture_value[2])/1000.0f) *posture_value[2] ;
	curve[3] = (0.55f + 0.45f *ABS(posture_value[3])/1000.0f) *posture_value[3] ;
	
	int date_throttle	= (thr_value)/cos(LIMIT(Pitch,-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57.324841	)/cos(LIMIT(Roll,-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57.324841	);//add  12.9
  motor[0] = date_throttle + Thr_Weight *curve[0] ;
	motor[1] = date_throttle + Thr_Weight *curve[1] ;
	motor[2] = date_throttle + Thr_Weight *curve[2] ;
	motor[3] = date_throttle + Thr_Weight *curve[3] ;
	mode.en_moto_smooth=1;
	  if(mode.en_moto_smooth){
     for(i=0;i<MAXMOTORS;i++){
        if(motor[i] > motor_last[i]) 
					motor[i] = (1 * (int16_t) motor_last[i] + motor[i]) / 2;  //mean of old and new
        else                                         
					motor[i] = motor[i] - (motor_last[i] - motor[i]) * 1; // 2 * new - old
			}
			 for(i=0;i<MAXMOTORS;i++)
					motor_last[i] = motor[i];  //mean of old and new
     
	    }
			
	/* 是否解锁 */
	if(fly_ready)
	{
		if( !Thr_Low )  //油门拉起
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
			}
		}
		else						//油门低
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );
			}
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			motor[i] = 0;
		}
	}
	/* xxx */
	#if NEW_FLY_BOARD
	motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	#else
  motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	#endif
	//SetPwm(motor_out,0,1000); //
}
//



