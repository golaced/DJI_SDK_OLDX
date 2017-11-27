#include "include.h"
#include "circle.h"
#include "alt_kf.h"

CIRCLE circle,track,mouse;
M100 m100;
float nav_circle[2],nav_circle_last[2];
/*
        y add (0~256)  1
        |
        |


        -------- x decrease (0~320)  0


*/

//---------------------------MAP----------------
float target_map[MAP_NUM][5];//lat lon h init 
void Clear_map(void){
	u8 i;
	for(i=0;i<MAP_NUM;i++)
	target_map[i][0]=target_map[i][1]=target_map[i][2]=target_map[i][3]=target_map[i][4]=0;
}



int pix_ero=MID_X*0.25;
float map_correct_step=0.69;
u16 map_dead_cnt=8;
void map_builder(void){
	u8 i;
	
	for(i=0;i<6;i++){		
	if(circle.map[i][0]!='N'&&ABS(circle.map[i][1]-MID_X)<pix_ero&&gps_data.latitude!=0&&gps_data.longitude!=0){
    target_map[i][4]++; 
		if(target_map[i][4]>map_dead_cnt){
		if(target_map[circle.map[i][0]][3]==0){
		target_map[circle.map[i][0]][3]=circle.map[i][0];
		target_map[circle.map[i][0]][0]=gps_data.latitude;
		target_map[circle.map[i][0]][1]=gps_data.longitude;
		target_map[circle.map[i][0]][2]=ALT_POS_SONAR2;
		}
		else if(S_head>20&&ALT_POS_SONAR_HEAD<3.666)
		{
		float flt_use;
		if(state_v==SU_MAP1||state_v==SU_MAP2||state_v==SU_MAP3)
			flt_use=map_correct_step;
		else
			flt_use=map_correct_step/2;
		target_map[circle.map[i][0]][0]=target_map[circle.map[i][0]][0]*flt_use+(1-flt_use)*gps_data.latitude;
		target_map[circle.map[i][0]][1]=target_map[circle.map[i][0]][1]*flt_use+(1-flt_use)*gps_data.longitude;
		target_map[circle.map[i][0]][2]=target_map[circle.map[i][0]][2]*flt_use+(1-flt_use)*ALT_POS_SONAR2;
		}
	 }
	}
  }
}


_st_height_pid_v qr_ctrl[2];
_st_height_pid qr_pid;
double qr_gps_pos[2]={30.8498172, 119.6145019};//检查点
u8 get_qr_pos=0;
float qr_local_pos[3];
float qr_local_pos1[3];
float qr_pos_off[2];
 void Estimate_land_marker_local_position(float qr_posx,float qr_posy,float qr_yaw,float drone_pos_n_local,float drone_pos_e_local,float drone_yaw,float T)
{
float qr_yaw_in_global;
qr_yaw_in_global=To_180_degrees(drone_yaw-qr_yaw);
float cy=cos(qr_yaw_in_global*0.0173);	
float sy=sin(qr_yaw_in_global*0.0173);		
float x_temp= qr_posx*cy + qr_posy*sy;
float y_temp=	-qr_posx*sy+ qr_posy*cy;
qr_local_pos[2]=sqrt(y_temp*y_temp+x_temp*x_temp);
qr_local_pos[North]=drone_pos_n_local-y_temp;
qr_local_pos[East]=drone_pos_e_local-x_temp;	
}

void Estimate_land_marker_local_position1(float qr_posx,float qr_posy,float qr_yaw,float drone_pos_n_local,float drone_pos_e_local,float drone_yaw,float T)
{	
float cy=cos(drone_yaw*0.0173);	
float sy=sin(drone_yaw*0.0173);	
float x_temp= qr_posx*cy + qr_posy*sy;
float y_temp=	-qr_posx*sy+ qr_posy*cy;
qr_local_pos1[2]=sqrt(y_temp*y_temp+x_temp*x_temp);
qr_local_pos1[North]=drone_pos_n_local-y_temp;
qr_local_pos1[East]=drone_pos_e_local-x_temp;	
}


float drone_local_pos[2];
void Estimate_drone_local_position(double lat_drone,double lon_drone,double lat_cor,double lon_cor,float T)
{
    drone_local_pos[North] = (lat_drone - lat_cor) * navUkfData.r1;
    drone_local_pos[East] =  (lon_drone - lon_cor) * navUkfData.r2;
}
float tar_drone_local_pos[2];
void Estimate_drone_target_local_position(double tar_lat_drone,double tar_lon_drone,double lat_cor,double lon_cor,float T)
{
    tar_drone_local_pos[North] = (tar_lat_drone - lat_cor) * navUkfData.r1;
    tar_drone_local_pos[East] =  (tar_lon_drone - lon_cor) * navUkfData.r2;
}


//----------------------------------GPS------------------------
#include "pwm_in.h"
#include "gps.h"
#include "filter.h"
float nav_gps[2];
navUkfStruct_t navUkfData;
navigation_gps nav_Data;

static void navUkfCalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    navUkfData.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    navUkfData.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

void navUkfCalcGlobalDistance(double lat, double lon, float *posNorth, float *posEast) {
    *posNorth = (lat - navUkfData.holdLat) * navUkfData.r1;
    *posEast = (lon - navUkfData.holdLon) * navUkfData.r2;
}

// input lat/lon in degrees, returns distance in meters
float navCalcDistance(double lat1, double lon1, double lat2, double lon2) {
    float n = (lat1 - lat2) * navUkfData.r1;
    float e = (lon1 - lon2) * navUkfData.r2;
    return __sqrtf(n*n + e*e);
}

// input lat/lon in degrees, returns bearing in radians
float navCalcBearing(double lat1, double lon1, double lat2, double lon2) {
    float n = (float)((lat2 - lat1) * (double)DEG_TO_RAD * navUkfData.r1);
    float e = (float)((lon2 - lon1) * (double)DEG_TO_RAD * navUkfData.r2);
    float ret = atan2f(e, n);

    if (!isfinite(ret))
        ret = 0.0f;

    return To_180_degrees(ret);
}


static void navUkfResetPosition(float deltaN, float deltaE, float deltaD) {
    int i;

    for (i = 0; i < UKF_HIST; i++) {
	navUkfData.posN[i] += deltaN;
	navUkfData.posE[i] += deltaE;
	navUkfData.posD[i] += deltaD;
    }
}

void navUkfSetGlobalPositionTarget(double lat, double lon) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;

    navUkfCalcGlobalDistance(lat, lon, &oldPosN, &oldPosE);

    navUkfData.holdLat = lat;
    navUkfData.holdLon = lon;

    navUkfCalcGlobalDistance(lat, lon, &newPosN, &newPosE);

    navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

static void navUkfCalcLocalDistance(double localPosN, double localPosE, double *posN, double *posE) {
    *posN = localPosN - (float)navUkfData.holdLat;
    *posE = localPosE - (float)navUkfData.holdLon;
}

static void navUkfSetLocalPositionTarget(double posN, double posE) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;


    navUkfData.holdLat = posN;
    navUkfData.holdLon = posE;
}

void navUkfSetHereAsPositionTarget(void) {
	navUkfSetGlobalPositionTarget(gps_data.latitude, gps_data.longitude);
}

void CalcGlobalLocation(float posNorth,float posEast,float local_Lat,float local_Lon,double *GPS_W_F,double *GPS_J_F){ 
    *GPS_W_F=(float)posNorth/(float)(navUkfData.r1+0.1)+local_Lat;
    *GPS_J_F=(float)posEast/(float)(navUkfData.r2+0.1)+local_Lon;
}

//矢量垂线方程
void line_function90_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=-1/tan(tyaw*ANGLE_TO_RADIAN);
  *b=y-k_temp*x;
}	


//两直线交点
u8 cross_point_of_lines(float k1,float b1,float k2,float b2,float *x,float *y)
{ 
	if(ABS(k1-k2)<0.001){
		*x=*y=0;
		return 0;}
	float x_temp;
	*x=x_temp=(b1-b2)/(k2-k1+0.00001);
//	if(fabs(k1)>10000&&fabs(b1)>10000)
//  *y=0;
//  else	
	*y=k1*x_temp+b1;
	
	return 1;
}	

//计算两点距离
float cal_dis_of_points(float x1,float y1,float x2,float y2)
{
return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}	

//矢量求直线方程
void line_function_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=tan(tyaw*ANGLE_TO_RADIAN);
  *b=y-k_temp*x;
}	

_st_height_pid_v gps_ctrl[2];
_st_height_pid gps_pid;
double lon,lat;
double tar_pos_gps[2];//测试
float flt_gps=0.618;
u16 MAX_GPS=100*2;//100*1.5
u8 gps_target_change;
double home_point[2]={30.8498039, 119.614204};//起飞点
double check_way_point[2]={30.8498172, 119.6145019};//检查点
double way_point[2][2]={
											  30.8498344, 119.6145401,
											  30.8498344, 119.6146316
};
double tar_point_globle[2]={39.9626688, 116.3039488};//全局输出
double tar_now_gps[2];
double tar_point_globler[2]={39.9626688, 116.3039488};//全局输出
double gps_local_cor_zero[2]={39.9626688, 116.3039488};//局部GPS坐标系原点
u8 state_set_point;
u8 set_point1;
void  GPS_hold(nmea_msg *gpsx_in,float T)
{ static u8 init,state;
	float out_temp[2];
	u8 set_gps_point;
	_st_height_pid gps_pid_use;
	float gain_out=0.4;
	if(!init){init=1;
		gps_pid.kp=0.125;//0.2;
		gps_pid.ki=0.00;//01;
		gps_pid.kd=1.888;
		
		qr_pid.kp=0.08;//0.2;
		qr_pid.ki=0.00;//01;
		qr_pid.kd=1.888/2;
	}
		if(state==SU_MAP1||state==SU_MAP2||state==SU_MAP3){
		gps_pid_use.kp=gps_pid.kp*1.618;
		gps_pid_use.ki=gps_pid.ki;
		gps_pid_use.kd=gps_pid.kd;
		gain_out=0.8;
		}else{
		gps_pid_use.kp=gps_pid.kp;
		gps_pid_use.ki=gps_pid.ki;
		gps_pid_use.kd=gps_pid.kd;
		}
		

	gps_local_cor_zero[0]=check_way_point[0];//使用第一个检测点作为qrland局部位置
	gps_local_cor_zero[1]=check_way_point[1];
		
	#if USE_M100
	 #if USE_PX4
	 if(Rc_Pwm_Inr_mine[RC_PITCH]>1800&&Rc_Pwm_Inr_mine[RC_ROLL]>1800&&Rc_Pwm_Inr_mine[RC_YAW]<1200&&!dji_rst_protect&&state_v==SG_LOW_CHECK&&!en_vrc)
		 set_point1=1;
	 else if(ABS((float)Rc_Pwm_Inr_mine[RC_PITCH]-1500)<50&&ABS((float)Rc_Pwm_Inr_mine[RC_ROLL]-1500)<50&&ABS((float)Rc_Pwm_Inr_mine[RC_YAW]-1500)<50)
		 set_point1=0;
	 #else
	 if(m100.Rc_pit>9000&m100.Rc_rol>9000&&m100.Rc_yaw<-9000&&!dji_rst_protect&&state_v==SG_LOW_CHECK&&!en_vrc)
		 set_point1=1;
	 else if(ABS(m100.Rc_pit)<1500&&ABS(m100.Rc_rol)<1500&&ABS(m100.Rc_yaw)<1500)
		 set_point1=0;
	 #endif
	 lon = m100.Lon;
	 lat = m100.Lat;
	 if(m100.STATUS>0&&m100.Lat!=0&&m100.Lon!=0&&!dji_rst_protect)
	 gpsx.gpssta=1;
	 else
	 gpsx.gpssta=0;	 
	 gpsx.rmc_mode='A';
	#else
	 lon = gpsx.longitude;
	 lat = gpsx.latitude;
  #endif	
	//if(1){
	if(lat!=0&&lon!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){//有效数据
	gps_data.latitude=lat;
	gps_data.longitude=lon;
	gps_data.angle=gpsx.angle;	
	
	set_gps_point=mode.set_point1;
  set_gps_point=set_point1;		
	static u16 cnt_delay;	
	switch(state_set_point)	
	{
		case 0:
		if(set_gps_point)	
		{
		state_set_point=1;cnt_delay=0;
		
		}
		
		break;
		case 1:
		check_way_point[0]=gps_data.latitude;
		check_way_point[1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=2;
		}
		break;
		case 2:
		if(set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		state_set_point=3;
	}
		}
		break;
		case 3:
		way_point[0][0]=gps_data.latitude;
		way_point[0][1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=4;
		}
		break;
		case 4:
		if(set_gps_point)//&&way_point[0][0]!=gps_data.latitude&&way_point[0][1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;		
		state_set_point=5;
		
		}
		}
		break;
	  case 5:
		way_point[1][0]=gps_data.latitude;
		way_point[1][1]=gps_data.longitude;	
		if(!set_gps_point)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		Yaw_set_dji=To_180_degrees(m100.Yaw);	
		state_set_point=0;
		//if(KEY[3])
		WRITE_PARM();
		}
		}
		
		break;
	
	}
		

	u8 flag;
	if(Rc_Pwm_Inr_mine[RC_PITCH]<OFF_RC_PIT-80||Rc_Pwm_Inr_mine[RC_PITCH]>OFF_RC_PIT+80||
	Rc_Pwm_Inr_mine[RC_ROLL]<OFF_RC_ROL-80||Rc_Pwm_Inr_mine[RC_ROLL]>OFF_RC_ROL+80)
	flag=1;
	else 
	flag=0;
	
	navUkfCalcEarthRadius(lat);
  if(flag)//悬停测试	
  navUkfSetHereAsPositionTarget();	
	float y[3];

	#if !defined(DEBUG_HOVER_GPS)
	if(mode.en_gps)//返航测试
	navUkfSetLocalPositionTarget( tar_point_globle[0], tar_point_globle[1]);//设置期望GPS位置	
	#endif
	if(tar_pos_gps[0]!=0||tar_pos_gps[1]!=0) 
	navUkfSetHereAsPositionTarget();
	
	navUkfCalcGlobalDistance(lat, lon, &y[0], &y[1]);//0->north 1->east 目标和当前经纬度 转换m的误差
	
	if(ABS(y[0]>1*1000)||ABS(y[1]>1*1000))//>1km 复位
	{
	 navUkfSetHereAsPositionTarget();
	 navUkfCalcGlobalDistance(lat, lon, &y[0], &y[1]);
	 Estimate_drone_local_position( lat, lon, gps_local_cor_zero[North], gps_local_cor_zero[East], T);
	 Estimate_drone_target_local_position( navUkfData.holdLat, navUkfData.holdLon, gps_local_cor_zero[North], gps_local_cor_zero[East], T);	
	}	

	//转换GPS到局部坐标系
	Estimate_drone_local_position( lat, lon, gps_local_cor_zero[North], gps_local_cor_zero[East], T);
	Estimate_drone_target_local_position( navUkfData.holdLat, navUkfData.holdLon, gps_local_cor_zero[North], gps_local_cor_zero[East], T);
	
  //qr land & map
	float	yaw_use_gimbal_v=To_180_degrees(m100.Yaw-0*(float)(PWM_DJ[1]-1500)/250.*65);//飞行器当前航向
	if(qr.check&&qr.connect&&(state_v==SD_TO_HOME)&&get_qr_pos==0)	
	{
		get_qr_pos=1;
//		qr_gps_pos[0]=gps_data.latitude;
//		qr_gps_pos[1]=gps_data.longitude;
		
		qr_pos_off[0]=0;
		qr_pos_off[1]=0;
	}
	
	if(qr.check&&qr.connect)//QR map
	{		
	float qr_posx=(float)qr.x/100.;
	float qr_posy=(float)qr.y/100.;
	float qr_posz=(float)qr.z/100.;
	float qr_yaw=qr.yaw;
	#if DEBUG_IN_ROOM
  drone_local_pos[East]=drone_local_pos[North]=0;
  #endif		
	Estimate_land_marker_local_position( qr_posx, qr_posy, qr_yaw, drone_local_pos[North], drone_local_pos[East], yaw_use_gimbal_v, T);
	//没有3d信息的qr位置
	Estimate_land_marker_local_position1((float)qr.center_x/100., (float)qr.center_y/100., qr_yaw, drone_local_pos[North], drone_local_pos[East], yaw_use_gimbal_v, T);
  CalcGlobalLocation(qr_local_pos[North],qr_local_pos[East],gps_local_cor_zero[North], gps_local_cor_zero[East],&qr_gps_pos[0],&qr_gps_pos[1]);
	}
	
	if(mode.use_qr_as_gps_tar&&mode.en_qr_land)//目标为qr 局部坐标
	{
		if(mode.qr_cal_by_px){
		tar_drone_local_pos[North]=qr_local_pos1[North];
		tar_drone_local_pos[East]= qr_local_pos1[East];}
		else{
		tar_drone_local_pos[North]=qr_local_pos[North];
		tar_drone_local_pos[East]= qr_local_pos[East];
		}
	}	
	
	
// 导航控制误差计算
//  y[North]=drone_local_pos[North]-tar_drone_local_pos[North];
//  y[East]= drone_local_pos[East] -tar_drone_local_pos[East];
	
  float yaw_use;
  #if USE_M100
  yaw_use=Moving_Median(23,3,m100.Yaw);
  #else
  yaw_use=Yaw;		
  #endif	
	navUkfData.yawCos=cos(0.0173*yaw_use);
	navUkfData.yawSin=sin(0.0173*yaw_use);
	
	//0-->wei  1 -->jing
	//filter
	nav_Data.gps_ero_dis_lpf[0]=flt_gps*y[0]*1000+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[0];
	nav_Data.gps_ero_dis_lpf[1]=flt_gps*y[1]*1000+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[1];
	//cal_pos_dis_for_line
	float k[3],b[3];
	float jiao[2];
	line_function_from_arrow(0,0,m100.Yaw,&k[0],&b[0]);
	line_function90_from_arrow(y[1],y[0],m100.Yaw,&k[1],&b[1]);
	//两直线交点
  cross_point_of_lines(k[0],b[0],k[1],b[1],&jiao[0],&jiao[1]);
  nav_Data.dis_ero=cal_dis_of_points(jiao[0],jiao[1],y[1],y[0]);
	
	int max_ero0,max_ero1;
	if(fabs(nav_Data.gps_ero_dis_lpf[0])>800*4)
		max_ero0=600*2;
	else
		max_ero0=600;
	
	if(fabs(nav_Data.gps_ero_dis_lpf[1])>800*4)
		max_ero1=600*2;
	else
		max_ero1=600;
	//PID
  //N
	if(gps_pid.ki==0||flag)gps_ctrl[0].err_i=0;
	gps_ctrl[0].err = ( gps_pid_use.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[0],25),-max_ero0,max_ero0) );//mm
	gps_ctrl[0].err_i += gps_pid_use.ki *gps_ctrl[0].err *T;
	gps_ctrl[0].err_i = LIMIT(gps_ctrl[0].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[0].err_d = gps_pid_use.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[0].err - gps_ctrl[0].err_old) );
	gps_ctrl[0].pid_out = gps_ctrl[0].err + gps_ctrl[0].err_i + gps_ctrl[0].err_d;
	gps_ctrl[0].pid_out = LIMIT(gps_ctrl[0].pid_out,-1000,1000);
	gps_ctrl[0].err_old = gps_ctrl[0].err;
	//E
	if(gps_pid.ki==0||flag)gps_ctrl[1].err_i=0;
	gps_ctrl[1].err = ( gps_pid_use.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[1],25),-max_ero1,max_ero1) );//mm
	gps_ctrl[1].err_i += gps_pid_use.ki *gps_ctrl[1].err *T;
	gps_ctrl[1].err_i = LIMIT(gps_ctrl[1].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[1].err_d = gps_pid_use.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[1].err - gps_ctrl[1].err_old) );
	gps_ctrl[1].pid_out = gps_ctrl[1].err + gps_ctrl[1].err_i + gps_ctrl[1].err_d;
	gps_ctrl[1].pid_out = LIMIT(gps_ctrl[1].pid_out,-1000,1000);
	gps_ctrl[1].err_old = gps_ctrl[1].err;
	
	
	nav_Data.holdSpeedN = -gps_ctrl[0].pid_out*gain_out ;
	nav_Data.holdSpeedE = -gps_ctrl[1].pid_out*gain_out ;

	//x-> pitch  y->roll  --->ero
	nav_Data.velX = 	nav_Data.holdSpeedN * navUkfData.yawCos + 	nav_Data.holdSpeedE * navUkfData.yawSin;
	nav_Data.velY = 	nav_Data.holdSpeedE * navUkfData.yawCos - 	nav_Data.holdSpeedN * navUkfData.yawSin;

	out_temp[PITr]=nav_Data.velX;
	out_temp[ROLr]=nav_Data.velY;
	
	tar_point_globler[0]=tar_point_globler[0];
	tar_point_globler[1]=tar_point_globler[1];
				if(gps_target_change){
					if(tar_point_globle[0]!=tar_now_gps[0]||tar_point_globle[1]!=tar_now_gps[1])
						gps_target_change=0;
					}
	}	
	// pix control land for qr code
	float pix_dead=0;
	if(mode.qr_cal_by_px)
		pix_dead=0.3;
	else
		pix_dead=0.8;
	
	float ero_qr_pix[2];
	static float ero_qr_pixr[2];
		if((mode.land_by_pix&&mode.en_qr_land&&
			((ABS(circle.x-160)<160*pix_dead&&ABS(circle.y-120)<120*pix_dead))
		   &&state_v==SD_CIRCLE_MID_DOWN)||0)//使用图像对准
	{
		if(qr.check&&qr.connect){
		ero_qr_pix[ROLr]=my_deathzoom(circle.x-160,0)*((float)qr.z/100.);
		ero_qr_pix[PITr]=-my_deathzoom(circle.y-120,0)*((float)qr.z/100.);	
		out_temp[ROLr]=ero_qr_pix[ROLr]*qr_pid.kp+(ero_qr_pix[ROLr] - ero_qr_pixr[ROLr])*qr_pid.kd;
	  out_temp[PITr]=ero_qr_pix[PITr]*qr_pid.kp+(ero_qr_pix[PITr] - ero_qr_pixr[PITr])*qr_pid.kd;
		ero_qr_pixr[ROLr]=ero_qr_pix[ROLr];	
		ero_qr_pixr[PITr]=ero_qr_pix[PITr];	
		out_temp[ROLr]=LIMIT(out_temp[ROLr],-MAX_GPS*2,MAX_GPS*2);
	  out_temp[PITr]=LIMIT(out_temp[PITr],-MAX_GPS*2,MAX_GPS*2);
		}
		else
		{
		ero_qr_pixr[ROLr]=0;	
		ero_qr_pixr[PITr]=0;		
		}		
	}
	
	//final output  0  ROL  1 PIT
	if((gpsx.gpssta>=1&&gpsx.rmc_mode=='A')||0){//定位有效
	nav_gps[ROLr]=LIMIT(out_temp[ROLr],-MAX_GPS,MAX_GPS);
	nav_gps[PITr]=LIMIT(out_temp[PITr],-MAX_GPS,MAX_GPS);
  }
	else
	{	
	nav_gps[ROLr]=0;
	nav_gps[PITr]=0;
	}
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	   
head  |    1 PIT y-  RC0
			| 
		   _____  0 ROL x+   RC 1
	
		*/

		
}

void  GPS_hold1(nmea_msg *gpsx_in,float T)
{ static u8 init,state;
	float out_temp[2];
	u8 set_gps_point;
	_st_height_pid gps_pid_use;
	float gain_out=0.4;
	if(!init){init=1;
		gps_pid.kp=0.125;//0.2;
		gps_pid.ki=0.00;//01;
		gps_pid.kd=1.888;
		
		qr_pid.kp=0.88;//0.2;
		qr_pid.ki=0.00;//01;
		qr_pid.kd=1.888;
	}
		if(state==SU_MAP1||state==SU_MAP2||state==SU_MAP3){
		gps_pid_use.kp=gps_pid.kp*1.618;
		gps_pid_use.ki=gps_pid.ki;
		gps_pid_use.kd=gps_pid.kd;
		gain_out=0.8;
		}else{
		gps_pid_use.kp=gps_pid.kp;
		gps_pid_use.ki=gps_pid.ki;
		gps_pid_use.kd=gps_pid.kd;
		}
	#if USE_M100
	 
	 if(m100.Rc_pit>9000&m100.Rc_rol>9000&&m100.Rc_yaw<-9000&&!dji_rst_protect&&state_v==SG_LOW_CHECK&&!en_vrc)
		 set_point1=1;
	 else if(ABS(m100.Rc_pit)<1500&&ABS(m100.Rc_rol)<1500&&ABS(m100.Rc_yaw)<1500)
		 set_point1=0;
	 lon = m100.Lon;
	 lat = m100.Lat;
	 if(m100.STATUS>0&&m100.Lat!=0&&m100.Lon!=0&&!dji_rst_protect)
	 gpsx.gpssta=1;
	 else
	 gpsx.gpssta=0;	 
	 gpsx.rmc_mode='A';
	#else
	 lon = gpsx.longitude;
	 lat = gpsx.latitude;
  #endif	
	//if(1){
	if(lat!=0&&lon!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){//????
	gps_data.latitude=lat;
	gps_data.longitude=lon;
	gps_data.angle=gpsx.angle;	
	
	set_gps_point=mode.set_point1;
  set_gps_point=set_point1;		
	static u16 cnt_delay;	
	switch(state_set_point)	
	{
		case 0:
		if(set_gps_point)	
		{
		state_set_point=1;cnt_delay=0;
		
		}
		
		break;
		case 1:
		check_way_point[0]=gps_data.latitude;
		check_way_point[1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=2;
		}
		break;
		case 2:
		if(set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		state_set_point=3;
	}
		}
		break;
		case 3:
		way_point[0][0]=gps_data.latitude;
		way_point[0][1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=4;
		}
		break;
		case 4:
		if(set_gps_point)//&&way_point[0][0]!=gps_data.latitude&&way_point[0][1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;		
		state_set_point=5;
		
		}
		}
		break;
	  case 5:
		way_point[1][0]=gps_data.latitude;
		way_point[1][1]=gps_data.longitude;	
		if(!set_gps_point)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		Yaw_set_dji=To_180_degrees(m100.Yaw);	
		state_set_point=0;
		//if(KEY[3])
		WRITE_PARM();
		}
		}
		
		break;
	
	}
		

	u8 flag;
	if(Rc_Pwm_Inr_mine[RC_PITCH]<OFF_RC_PIT-80||Rc_Pwm_Inr_mine[RC_PITCH]>OFF_RC_PIT+80||
	Rc_Pwm_Inr_mine[RC_ROLL]<OFF_RC_ROL-80||Rc_Pwm_Inr_mine[RC_ROLL]>OFF_RC_ROL+80)
	flag=1;
	else 
	flag=0;
	
	navUkfCalcEarthRadius(lat);
  if(flag)//????	
  navUkfSetHereAsPositionTarget();	
	float y[3];

	
	if(mode.en_gps)//????
	navUkfSetLocalPositionTarget( tar_point_globle[0], tar_point_globle[1]);	
	if(tar_pos_gps[0]!=0||tar_pos_gps[1]!=0) 
	navUkfSetLocalPositionTarget( 39.9626144, 116.3038848);
	
	navUkfCalcGlobalDistance(lat, lon, &y[0], &y[1]);//0->north 1->east
	
	if(ABS(y[0]>1000)||ABS(y[1]>1000))
	{
	 navUkfSetHereAsPositionTarget();
	 navUkfCalcGlobalDistance(lat, lon, &y[0], &y[1]);
	}	

  
  float yaw_use;
  #if USE_M100
  yaw_use=Moving_Median(23,3,m100.Yaw);
  #else
  yaw_use=Yaw;		
  #endif	
	navUkfData.yawCos=cos(0.0173*yaw_use);
	navUkfData.yawSin=sin(0.0173*yaw_use);
	
	//0-->wei  1 -->jing
	//filter
	
	
	nav_Data.gps_ero_dis_lpf[0]=flt_gps*Moving_Median(28,3,y[0]*1000)+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[0];
	nav_Data.gps_ero_dis_lpf[1]=flt_gps*Moving_Median(29,3,y[1]*1000)+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[1];
	//cal_pos_dis_for_line
	float k[3],b[3];
	float jiao[2];
	line_function_from_arrow(0,0,m100.Yaw,&k[0],&b[0]);
	line_function90_from_arrow(y[1],y[0],m100.Yaw,&k[1],&b[1]);
	//?????
  cross_point_of_lines(k[0],b[0],k[1],b[1],&jiao[0],&jiao[1]);
  nav_Data.dis_ero=cal_dis_of_points(jiao[0],jiao[1],y[1],y[0]);
	
//	15075434@qq.com  yang-->15877918559
	int max_ero0,max_ero1;
	if(fabs(nav_Data.gps_ero_dis_lpf[0])>800*4)
		max_ero0=600*2;
	else
		max_ero0=600;
	
	if(fabs(nav_Data.gps_ero_dis_lpf[1])>800*4)
		max_ero1=600*2;
	else
		max_ero1=600;
	//PID
  //N
	if(gps_pid.ki==0||flag)gps_ctrl[0].err_i=0;
	gps_ctrl[0].err = ( gps_pid_use.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[0],25),-max_ero0,max_ero0) );//mm
	gps_ctrl[0].err_i += gps_pid_use.ki *gps_ctrl[0].err *T;
	gps_ctrl[0].err_i = LIMIT(gps_ctrl[0].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[0].err_d = gps_pid_use.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[0].err - gps_ctrl[0].err_old) );
	gps_ctrl[0].pid_out = gps_ctrl[0].err + gps_ctrl[0].err_i + gps_ctrl[0].err_d;
	gps_ctrl[0].pid_out = LIMIT(gps_ctrl[0].pid_out,-1000,1000);
	gps_ctrl[0].err_old = gps_ctrl[0].err;
	//E
	if(gps_pid.ki==0||flag)gps_ctrl[1].err_i=0;
	gps_ctrl[1].err = ( gps_pid_use.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[1],25),-max_ero1,max_ero1) );//mm
	gps_ctrl[1].err_i += gps_pid_use.ki *gps_ctrl[1].err *T;
	gps_ctrl[1].err_i = LIMIT(gps_ctrl[1].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[1].err_d = gps_pid_use.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[1].err - gps_ctrl[1].err_old) );
	gps_ctrl[1].pid_out = gps_ctrl[1].err + gps_ctrl[1].err_i + gps_ctrl[1].err_d;
	gps_ctrl[1].pid_out = LIMIT(gps_ctrl[1].pid_out,-1000,1000);
	gps_ctrl[1].err_old = gps_ctrl[1].err;
	
	
	nav_Data.holdSpeedN = -gps_ctrl[0].pid_out*gain_out ;
	nav_Data.holdSpeedE = -gps_ctrl[1].pid_out*gain_out ;

	//x-> pitch  y->roll  --->ero
	nav_Data.velX = 	nav_Data.holdSpeedN * navUkfData.yawCos + 	nav_Data.holdSpeedE * navUkfData.yawSin;
	nav_Data.velY = 	nav_Data.holdSpeedE * navUkfData.yawCos - 	nav_Data.holdSpeedN * navUkfData.yawSin;

	out_temp[PITr]=nav_Data.velX;
	out_temp[ROLr]=nav_Data.velY;
	
	tar_point_globler[0]=tar_point_globler[0];
	tar_point_globler[1]=tar_point_globler[1];
				if(gps_target_change){
					if(tar_point_globle[0]!=tar_now_gps[0]||tar_point_globle[1]!=tar_now_gps[1])
						gps_target_change=0;
					}
	}	
	
		// pix control land for qr code
	float pix_dead=0;
	if(mode.qr_cal_by_px)
		pix_dead=0.3;
	else
		pix_dead=0.8;
	
	pix_dead=1;
	float ero_qr_pix[2];
	static float ero_qr_pixr[2];
		if(mode.land_by_pix&&mode.en_qr_land&&state_v==SD_CIRCLE_MID_DOWN){
			  if((ABS(circle.x-160)<160*pix_dead&&ABS(circle.y-120)<120*pix_dead))
		    {
						if(qr.check&&qr.connect){
						ero_qr_pix[ROLr]=my_deathzoom(circle.x-160,0)*((float)qr.z/100.);
						ero_qr_pix[PITr]=-my_deathzoom(circle.y-120,0)*((float)qr.z/100.);	
						out_temp[ROLr]=ero_qr_pix[ROLr]*qr_pid.kp+(ero_qr_pix[ROLr] - ero_qr_pixr[ROLr])*qr_pid.kd;
						out_temp[PITr]=ero_qr_pix[PITr]*qr_pid.kp+(ero_qr_pix[PITr] - ero_qr_pixr[PITr])*qr_pid.kd;
						ero_qr_pixr[ROLr]=ero_qr_pix[ROLr];	
						ero_qr_pixr[PITr]=ero_qr_pix[PITr];	
						out_temp[ROLr]=LIMIT(out_temp[ROLr],-MAX_GPS*2,MAX_GPS*2);
						out_temp[PITr]=LIMIT(out_temp[PITr],-MAX_GPS*2,MAX_GPS*2);
						}
						else
						{
						out_temp[ROLr]=out_temp[PITr]=0;	
						ero_qr_pixr[ROLr]=ero_qr_pixr[PITr]=0;				
						}			
				}	
		}
	
	
	//0  ROL  1 PIT
	if(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){//????
	nav_gps[ROLr]=LIMIT(out_temp[ROLr],-MAX_GPS,MAX_GPS);
	nav_gps[PITr]=LIMIT(out_temp[PITr],-MAX_GPS,MAX_GPS);
  }
	else
	{	
	nav_gps[ROLr]=0;
	nav_gps[PITr]=0;
	}
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	   
head  |    1 PIT y-  RC0
			| 
		   _____  0 ROL x+   RC 1
	
		*/
		
}