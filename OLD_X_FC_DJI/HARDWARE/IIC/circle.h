#ifndef __CIRCLE_H
#define __CIRCLE_H	 
#include "stm32f4xx.h" 
#include "include.h" 
#include "gps.h"
 typedef struct
{
 int x,y,z;
 float pit,rol , yaw ;
	int center_x,center_y;
 int r;
 int x_flp,y_flp;
 u8 check;
 u8 connect,lose_cnt;
 int control[2];
 float control_k;
 float control_k_miss; 
	float control_yaw,control_yaw_pix;
 float forward;
 float forward_end_dj_pwm;
  int map[6][4];
	u8 dj_fly_line;
}CIRCLE;
extern CIRCLE circle,track,mouse,qr;
extern float nav_circle[2],nav_land[2];
void circle_control(float T);
#define MID_Y 125
#define MID_X 140

#define East 1
#define North 0
extern float circle_use[2];
extern float  integrator[2];
void  GPS_hold(nmea_msg *gpsx_in,float T);

 typedef struct
{
 float Pit,Rol,Yaw;
 float Lat,Lon;
 float H,H_Spd;	
 u8 GPS_STATUS;	//>3  5->Best
/*
Flight status val	status name
1	standby
2	take_off
3	in_air
4	landing
5	finish_landing
*/
 u8 STATUS;
 float Bat;	
 int Rc_pit,Rc_rol,Rc_yaw,Rc_thr,Rc_mode,Rc_gear;
 u8 m100_connect;
 float spd[3];
 float q[4];
 u8 refresh;
 u8 mems_board_connect;
 u16 mems_loss_cnt;
}M100;
extern M100 m100;
#define MAP_NUM 20
extern float target_map[MAP_NUM][5];
void Clear_map(void);
void map_builder(void);
void navUkfCalcGlobalDistance(double lat, double lon, float *posNorth, float *posEast);

extern double qr_gps_pos[2];//检查点
extern double gps_local_cor_zero[2];//局部GPS坐标系原点
extern float qr_local_pos[3],drone_local_pos[2], qr_local_pos1[3],qr_pos_off[2];
extern u8 get_qr_pos;
extern float tar_drone_local_pos[2];
extern _st_height_pid_v qr_ctrl[2];
extern _st_height_pid qr_pid;
void GPS_Qr_Control(nmea_msg *gpsx_in,float T);
#endif











