
#include "imu.h"
#include "include.h"
#include "hml5833l.h"
#include "my_math.h"

#define Kp 0.6f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.1f                	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

float q_nav[4],q_nav_r[4];
xyz_f_t reference_v;
ref_t 	ref;
float reference_vr[3];
//xyz_f_t Gravity_Vec;  				//½âËãµÄÖØÁ¦ÏòÁ¿
	
float Roll,Pitch,Yaw;    				//×ËÌ¬½Ç

float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;
xyz_f_t mag_sim_3d;
extern u8 fly_ready;

int test_flag[3]={-1,-1,1};
u8 imu_sel=6;


#define Kp1 3.2f					// ±ÈÀýÔöÒæÖ§ÅäÊÕÁ²ÂÊaccelerometer/magnetometer
#define Ki1 0.002f				// »ý·ÖÔöÒæÖ´ÕþËÙÂÊÍÓÂÝÒÇµÄÏÎ½Ógyroscopeases
#define halfT1 0.001f			// ²ÉÑùÖÜÆÚµÄÒ»°ë

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// ËÄÔªÊýµÄÔªËØ£¬´ú±í¹À¼Æ·½Ïò
float exInt = 0, eyInt = 0, ezInt = 0;// °´±ÈÀýËõÐ¡»ý·ÖÎó²î


#define Gyro_Gr1		0.0010653f
float Yaw1,Pitch1,Roll1;
void AHRSupdate(float T,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;

	// ¸¨Öú±äÁ¿£¬ÒÔ¼õÉÙÖØ¸´²Ù×÷Êý
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;          
	
    if(ax*ay*az==0)
    return;
//    gx *= Gyro_Gr;
//    gy *= Gyro_Gr;
//    gz *= Gyro_Gr;	
	// ²âÁ¿Õý³£»¯
	norm = sqrt(ax*ax + ay*ay + az*az);       
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
//	norm = sqrt(mx*mx + my*my + mz*mz);          
//	mx = mx / norm;
//	my = my / norm;
//	mz = mz / norm;         
	
	// ¼ÆËã²Î¿¼´ÅÍ¨·½Ïò
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;        
	
	//¹À¼Æ·½ÏòµÄÖØÁ¦ºÍ´ÅÍ¨£¨VºÍW£©
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);  
	
	// ´íÎóÊÇ¿ç²úÆ·µÄ×ÜºÍÖ®¼äµÄ²Î¿¼·½ÏòµÄÁìÓòºÍ·½Ïò²âÁ¿´«¸ÐÆ÷
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	
	// »ý·ÖÎó²î±ÈÀý»ý·ÖÔöÒæ
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// µ÷ÕûºóµÄÍÓÂÝÒÇ²âÁ¿
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// ÕûºÏËÄÔªÊýÂÊºÍÕý³£»¯
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*T;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*T;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*T;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*T;  
	
	// Õý³£»¯ËÄÔª
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
  //×ª»»ÎªÅ·À­½Ç
  
 Pitch1  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  Roll1 =  atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  Yaw1 = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw  
}

#define Kp_Yaw 0.3f
float accConfidenceDecay 			  =	5.2f;
float accConfidence      = 1.0f; 
#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)
#define accelOneG 10
void calculateAccConfidence(float accMag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;

	float accMag =accMag_in/ accelOneG;  // HJI Added to convert MPS^2 to G's

	accMagP  = HardFilter(accMagP, accMag );


	accConfidence=((accConfidenceDecay * sqrt(fabs(accMagP - 1.0f))));
  if(accConfidence>1)
		accConfidence=1;
	if(accConfidence<0)
		accConfidence=0;
}
//use
float mag_norm ,mag_norm_xyz ;
float yaw_mag,Kp_use;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{	static u8 init;
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	
	if(!init)
	{
	init=1;
	ref.err_tmp.x=ref.err_tmp.y=ref.err_tmp.z=0;
	ref.err.x=ref.err.y=ref.err.z=0;
	ref.err_lpf.x=ref.err_lpf.y=ref.err_lpf.z=0;
	ref.err_Int.x=ref.err_Int.y=ref.err_Int.z=0;
	ref.g.x=ref.g.y=ref.g.z=0;
	}
	mag_norm_tmp = 20 *(6.28f *half_T);	
	
	mag_norm_xyz = my_sqrt(ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y + ak8975.Mag_Val.z * ak8975.Mag_Val.z);
	if(mag_norm_xyz==0)
		mag_norm_xyz=0.0001;
		if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( (float)ak8975.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)ak8975.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)ak8975.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
	}

	/*
	void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out)
	
	???????????,????????????,??????,?????????????
	???????????????,?????????,??????????
	??:??????????????????,?????????,?????,????????????
	*/
	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d); 
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	if(mag_norm==0)
		mag_norm=0.0001;
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;
		
	}
//---------acc norm---------
	float norm;
	norm = sqrt(ax*(ax) + ay*(ay) + az*az)/4096.*9.8;
	calculateAccConfidence(norm);
	
	Kp_use =	Kp* accConfidence ;
	//=============================================================================
	// ????????
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	
	//?????????«??????»???????????
	//?????????????,??????????,???????,?????????
	//?????vx\y\z,??????????(????)?????????,????????????       
	//=============================================================================

//	if(acc_ng_cali)
//	{
//		if(acc_ng_cali==2)
//		{
//			acc_ng_offset.x = 0;
//			acc_ng_offset.y = 0;
//			acc_ng_offset.z = 0;
//		}
//			
//		acc_ng_offset.x += 10 *TO_M_S2 *(ax - 4096*reference_v.x) *0.0125f ;
//		acc_ng_offset.y += 10 *TO_M_S2 *(ay - 4096*reference_v.y) *0.0125f ;
//		acc_ng_offset.z += 10 *TO_M_S2 *(az - 4096*reference_v.z) *0.0125f ;	
//		
//		acc_ng_cali ++;
//		if(acc_ng_cali>=82) //start on 2
//		{
//			acc_ng_cali = 0;
//		}
//	}
//	
//	acc_ng.x = 10 *TO_M_S2 *(ax - 4096*reference_v.x) - acc_ng_offset.x;
//	acc_ng.y = 10 *TO_M_S2 *(ay - 4096*reference_v.y) - acc_ng_offset.y;
//	acc_ng.z = 10 *TO_M_S2 *(az - 4096*reference_v.z) - acc_ng_offset.z;
//	
//	acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;
	

	// ?????????
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
  if(norm_acc==0)
		norm_acc=1;
  
	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//???????????????
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* ?????? */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* ???? */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* ???? */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* ???? */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	
	if( reference_v.z > 0.0f )
	{
		if( fly_ready  )
		{
			yaw_correct = Kp_Yaw *0.2f *To_180_degrees(yaw_mag - Yaw);
			//????,????????
		}
		else
		{
			yaw_correct = Kp_Yaw *1.5f *To_180_degrees(yaw_mag - Yaw);
			//????,??????,????
		}
// 		if( yaw_correct>360 || yaw_correct < -360  )
// 		{
// 			yaw_correct = 0;
// 			//??????+-360,??+-180?????
// 		}
	}
	else
	{
		yaw_correct = 0; //????,????,???????????
	}

	
	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp_use*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp_use*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* ???????PI?????? */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* ?????? normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	if(norm_q==0)
		norm_q=1;
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	
  reference_vr[0]=reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1]=reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2]=reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;

	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
	//*yaw = yaw_mag;

}
