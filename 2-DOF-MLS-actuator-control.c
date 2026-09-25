/*
	C Souce File Template for Exp3C6713A

	The code implements the position control for the 2-DOF Magnetic Lead Scew Actuator

	From the program Pe-View 9, start the program, then to initialize the the sensors:
	1) wait until the variable "mode" becomes 1. 
	2) rotate rotors and mover till "Initial_Setup" == 1.
	First upload the position of the mover ("angle_ref_M " and "linear_encoder_ref_M") within the possible shaft range, then "mode" to 2,
	wait about one second and then set to one the variable "Upload_Angle_Reference".
	From now the procedure is: upload the mover references to the desired position, wait about 1 second and set "Upload_Angle_Reference" to 1.
*/


#include <mwio3.h>
#include <math.h>


#define BDN (0)
#define BDN1 (1)
#define GRP0 (0)
#define GRP1 (1)
#define CH0 (0)
#define CH1 (1)
#define VADR (5.0)
#define FS (10000) 
#define TS (1.0 / FS)
#define DEADT (4000.0)
#define PI (3.141592653589793)
#define resolution_rotary (2 * PI / (3200 * 4))
#define AD_VAC (500.0 / 5.0)
#define AD_VDC (500.0 / 5.0)
#define AD_IAC (31.25 / 5.0)
#define AD_IDC (31.25 / 5.0)
#define SQRT3 (1.732050807568877)


/* Reference variables */
int Upload_Angle_Reference = 0;
float Initial_Setup = 0;


/* Controllers */
float w_current = 3000;
float Kp_speed = 0.09;
float Kp_ang = 5.5;


/* Filter */
float Forg_Fact = 0.395, current_dob_R_raw = 0, current_dob_L_raw = 0;
float Forg_Fact_DOB = 0.113, f_DOB_filter = 1200; //rad/s
float w_speed_cutoff = 1200;					  
float f_Current_filter = 12566; 
float Forg_Fact_current = 0;
float w_c_speed = 1200.0;

/* Program time check*/
float Check_Execution_Time=0, Execution_Time=0;
float start = 0, end = 0;


/* Variables */


/* Plant parameters and safety limits*/
float lamda_mg = 0.045;
float Gr = 6.36619;
float Ktn = 0.227; 
float L = 0.0246;			//Inductance (identified)
float R = 6.225;			//Resistance (identified)
float J = 1.5e-4;			//Inertia (identified)
int pp = 4;					//pole-pair number
float B = 0.0006;			//Friction

//float J_M = 0.0, M_M = 0.0; //Mover inertia & mass (Need identification)
// float Kf = 14.14e3; // Hypotetical (Need identification)

float w_motor_limit = 40;
float currentLimit = 3;

float Vdc_set = 48.0;


/*		DOB		*/
float current_dob_R = 0, prev_Iq_ref_R = 0, current_iq_R = 0, prev_w_R = 0, prev_dob_R = 0;
float current_dob_L = 0, prev_Iq_ref_L = 0, current_iq_L = 0, prev_w_L = 0, prev_dob_L = 0;
float gdr = 250, gdl = 250;



/* Switches & Flags */
int start = 0;
int mode = 0;


int z_flag_R = 0, z_flag_L = 0, Zflag_M = 0;
int SAT_d_R = 0, SAT_q_R = 0;
int SIGN_d_R = 0, SIGN_q_R = 0;
int SAT_d_L = 0, SAT_q_L = 0;
int SIGN_d_L = 0, SIGN_q_L = 0;
int Sign_q_L = 0, Sign_d_L = 0;
int Sign_q_R = 0, Sign_d_R = 0;


/* Counters */
int count = 0;
float rid = 0;
/* Refereces */
float Vu_ref_R = 0.0, Vv_ref_R = 0.0, Vw_ref_R = 0.0;
float Vu_ref_L = 0.0, Vv_ref_L = 0.0, Vw_ref_L = 0.0;
float Va_ref_R = 0.0, Vb_ref_R = 0.0;
float Va_ref_L = 0.0, Vb_ref_L = 0.0;
float Vd_ref_R = 0.0, Vq_ref_R = 0.0;
float Id_ref_R = 0.0, Iq_ref_R = 0.0;
float Vd_ref_L = 0.0, Vq_ref_L = 0.0;
float Id_ref_L = 0.0, Iq_ref_L = 0.0;
float Iq_cmd = 0.0;
float U_pwm_R = 0.0, V_pwm_R = 0.0, W_pwm_R = 0.0;
float U_pwm_L = 0.0, V_pwm_L = 0.0, W_pwm_L = 0.0;
float w_ref_R = 0.0, w_ref_L = 0.0;
float angle_ref_R = 0.0, angle_ref_L = 0.0, angle_ref_M = 0, linear_encoder_ref_M =0;



/* Encoders */
float encoder_R = 0.0, encoder_L = 0.0, encoder_M = 0;
float angle_R = 0.0, angle_L = 0.0;
float angle_M = 0.0, pzt_M = 0.0;
float eAngle_R = 0.0, eAngle_L = 0.0;
float offset_R = 133.93, offset_L = 133.98; //offset_L = 112.2759; // They are in grad and electric angle
int Z_R = 1, Z_M = 1, Z_L = 1;

int angle_mem_R = 0, angle_mem_L = 0;
int turn_num_R = 0, turn_num_L = 0, turn_num_M = 0;

/* Speed Calculation*/
float angle_R_0 = 0.0, angle_L_0 = 0.0;
float w_R = 0.0, w_L = 0.0;
float w_R_0 = 0.0, w_L_0 = 0.0;


/* Variables */
float Vdc_R = 0.0, Idc_R = 0.0;
float Iu_R = 0.0, Iv_R = 0.0, Iw_R = 0.0;
float Ia_R = 0.0, Ib_R = 0.0;
float Iq_R = 0.0, Id_R = 0.0;

float Vdc_L = 0.0, Idc_L = 0.0;
float Iu_L = 0.0, Iv_L = 0.0, Iw_L = 0.0;
float Ia_L = 0.0, Ib_L = 0.0;
float Iq_L = 0.0, Id_L = 0.0;

float Iq_ref_R_speed_D = 0, Iq_ref_R_speed_D_prev = 0;
float Iq_ref_R_prev = 0, Iq_ref_L_prev = 0;
float Iq_ref_L_DOB = 0, Iq_ref_L_DOB_prev = 0;
float Iq_ref_R_DOB = 0, Iq_ref_R_DOB_prev = 0;
float Iu_L_prev = 0, Iw_L_prev = 0;
float Iu_R_prev = 0, Iw_R_prev = 0;
float Iq_ref_L_speed = 0, Iq_ref_R_speed = 0;
float prev_Id_err_L = 0, prev_Iq_err_L = 0;
float Id_err_R = 0, Iq_err_R = 0;
float Id_err_L = 0, Iq_err_L = 0;
float prev_Id_err_R = 0, prev_Iq_err_R = 0;

float w_err_R_prev =0, w_err_R_D =0;
float w_R_1 = 0, angle_R_1 =0;
float w_L_1 = 0, angle_L_1 =0;
float w_err_R = 0, w_err_L = 0;


float x_M = 0;
float theta_R = 0, theta_L = 0;
float theta_R_init =0, theta_L_init =0;
float angle_ref_L_raw = 0,angle_ref_R_raw = 0;
float angle_err_R = 0, angle_err_L = 0;

float Vd_ref_FF_L = 0, Vd_ref_I_L = 0, Sat_d_L = 0, Vd_ref_P_L = 0;
float Vq_ref_FF_L = 0, Vq_ref_I_L = 0, Sat_q_L = 0, Vq_ref_P_L = 0;
float Vd_ref_PI_L = 0, Vq_ref_PI_L = 0;

float Vd_ref_FF_R = 0, Vd_ref_I_R = 0, Sat_d_R = 0, Vd_ref_P_R = 0;
float Vq_ref_FF_R = 0, Vq_ref_I_R = 0, Sat_q_R = 0, Vq_ref_P_R = 0;
float Vd_ref_PI_R = 0, Vq_ref_PI_R = 0;

float Vdq_mag_R = 0, Vdq_mag_L = 0;
float a_R = 0, a_L = 0;
float r_R = 0, r_L = 0 ;




/* AD board(MWPE3_PEV) setting  */
float offset0 = 0.0, offset1 = 0.0, offset2 = 0.0, offset3 = 0.0; //offsets for sensors
float offset4 = 0.0, offset5 = 0.0, offset6 = 0.0, offset7 = 0.0;
float c0 = 0.0, c1 = 0.0, c2 = 0.0, c3 = 0.0; //sensor reading results
float c4 = 0.0, c5 = 0.0, c6 = 0.0, c7 = 0.0;

/* Calibration */
int FLAG_AD_CAL = 0;
float acc0 = 0.0, acc1 = 0.0, acc2 = 0.0, acc3 = 0.0; //accumulators
float acc4 = 0.0, acc5 = 0.0, acc6 = 0.0, acc7 = 0.0;









interrupt void carrier_interrupt(void)
{

// To find the execution time do: (end-start)*17.77777ns
	
	if (Check_Execution_Time == 1){
		
		start = timer0_read();}
	
	wdt_clear();



	/* Position Sensing */


	encoder_R = pios_abz_read(BDN, CH0);
	theta_R = pios_abz_read(BDN, CH0)* resolution_rotary -theta_R_init ;
	
	encoder_L = pios_abz_read(BDN, CH1);
	theta_L = pios_abz_read(BDN, CH1)* resolution_rotary  -theta_L_init ;
	encoder_M = pios_abz_read(BDN1, CH0);
	
	x_M =pios_abz_read(BDN1, CH1) * 0.0001;



	angle_R = encoder_R * resolution_rotary - angle_mem_R;
	angle_L = encoder_L * resolution_rotary - angle_mem_L;
	angle_M = encoder_M * resolution_rotary;


	eAngle_R = pp * angle_R - offset_R * (2 * PI / 360);
	eAngle_L = pp * angle_L - offset_L * (2 * PI / 360);

	eAngle_R = fmod(eAngle_R, 2 * PI);
	if (eAngle_R > PI)
	{
		eAngle_R -= 2 * PI;
	}
	else if (eAngle_R < -PI)
	{
		eAngle_R += 2 * PI;
	} /* =>(-pi,pi) */
	eAngle_L = fmod(eAngle_L, 2 * PI);
	if (eAngle_L > PI)
	{
		eAngle_L -= 2 * PI;
	}
	else if (eAngle_L < -PI)
	{
		eAngle_L += 2 * PI;
	} /* =>(-pi,pi) */

	/* Sensing of Currents & Voltages */
	pev_ad_start(BDN, GRP0);
	while (pev_ad_in_st(BDN, GRP0) != 0)
		;
	pev_ad_in_grp(BDN, GRP0, &c0, &c1, &c2, &c3);
	pev_ad_start(BDN, GRP1);
	while (pev_ad_in_st(BDN, GRP1) != 0)
		;
	pev_ad_in_grp(BDN, GRP1, &c4, &c5, &c6, &c7);

	Iu_R = AD_IAC * (c0 - offset0);

	Iu_R = (1 - Forg_Fact_current) * Iu_R_prev + Forg_Fact_current * Iu_R;

	Iw_R = AD_IAC * (c1 - offset1);

	Iw_R = (1 - Forg_Fact_current) * Iw_R_prev + Forg_Fact_current * Iw_R;

	Iv_R = -(Iu_R + Iw_R);

	Vdc_R = AD_VDC * (c2 - offset2) + Vdc_set;
	Idc_R = AD_IDC * (c3 - offset3);


	Iu_L = AD_IAC * (c4 - offset4);

	Iu_L = (1 - Forg_Fact_current) * Iu_L_prev + Forg_Fact_current * Iu_L;

	Iw_L = AD_IAC * (c5 - offset5);

	Iw_L = (1 - Forg_Fact_current) * Iw_L_prev + Forg_Fact_current * Iw_L;

	Iv_L = -(Iu_L + Iw_L);

	Vdc_L = AD_VDC * (c6 - offset6) + Vdc_set;
	//Idc_L = AD_IDC * (c7 - offset7);

	uvw2ab(Iu_R, Iv_R, Iw_R, &Ia_R, &Ib_R);

	ab2dq(Ia_R, Ib_R, eAngle_R, &Id_R, &Iq_R);

	uvw2ab(Iu_L, Iv_L, Iw_L, &Ia_L, &Ib_L);

	ab2dq(Ia_L, Ib_L, eAngle_L, &Id_L, &Iq_L);

 /* Actual speed computation: position derivate */
	
	w_R = (-w_R_0*(-8+2*w_c_speed*w_c_speed*TS*TS) - w_R_1*(-1.4142*w_c_speed*TS*2 + w_c_speed*w_c_speed*TS*TS + 4) + 2*w_c_speed*w_c_speed*TS*(angle_R - angle_R_1)) / (4+1.4142*w_c_speed*TS*2 + w_c_speed*w_c_speed*TS*TS);
	
	w_L = (-w_L_0*(-8+2*w_c_speed*w_c_speed*TS*TS) - w_L_1*(-1.4142*w_c_speed*TS*2 + w_c_speed*w_c_speed*TS*TS + 4) + 2*w_c_speed*w_c_speed*TS*(angle_L - angle_L_1)) / (4+1.4142*w_c_speed*TS*2 + w_c_speed*w_c_speed*TS*TS);
	

	switch (mode)
	{
	default:

		break;

	case 0: // Sensors calibration
		if (count < 10 * FS)
		{
			acc0 += c0;
			acc1 += c1;
			acc2 += c2;
			acc3 += c3;
			acc4 += c4;
			acc5 += c5;
			acc6 += c6;
			acc7 += c7;
			count += 1;
		}
		else
		{
			offset0 = acc0 / count;
			offset1 = acc1 / count;
			offset2 = acc2 / count;
			offset3 = acc3 / count;
			offset4 = acc4 / count;
			offset5 = acc5 / count;
			offset6 = acc6 / count;
			offset7 = acc7 / count;
			FLAG_AD_CAL = 1;
			count = 0.0;
			mode = 1;
		}
		break;

	case 1: // Zero-positioning

		a_R = 0;
		r_R = 0;

		a_L = 0;
		r_L = 0;

		count = 0;
		
		
		
		//angle_ref_L =  angle_ref_M  - (1/Gr)* linear_encoder_ref_M;
		//angle_ref_R =  -angle_ref_M  - (1/Gr)* linear_encoder_ref_M;
		
		
		break;



	case 2: // Position control
	
		
	
	//%%%	When input form 'Inspector', the input are delayed from each other 		%%%//
			
		//	Uploading of the angle references at the same time
			
			
   		angle_ref_L_raw =  angle_ref_M  - (1/Gr)* linear_encoder_ref_M;
		angle_ref_R_raw =  -angle_ref_M  - (1/Gr)* linear_encoder_ref_M;
	
		if(Upload_Angle_Reference == 1)   {
			
			
		angle_ref_L =  angle_ref_L_raw;
		angle_ref_R =  angle_ref_R_raw;
			
			Upload_Angle_Reference = 0;
		}  
 

//For circuit

//1 Count = TS >> 100us

// Cycle for Trise = 0.2s
/*  		if(Upload_Angle_Reference == 1)   {
			
			count++;
			
					if((count>=0) && (count<9999))
					{	
		angle_ref_L =  0  - (1/Gr)* 0;
		angle_ref_R =  0  - (1/Gr)* 0;
					}	

					if((count>=9999) && (count<19999))
					{	
		angle_ref_L =  0  - (1/Gr)* 20;
		angle_ref_R =  0  - (1/Gr)* 20;
					}

					if((count>=19999) && (count<29999))
					{	
		angle_ref_L =   (2*PI)  - (1/Gr)* 20;
		angle_ref_R =  -(2*PI)  - (1/Gr)* 20;
					}
		
					if((count>=29999) && (count<39999))
					{	
		angle_ref_L =   (PI)  - (1/Gr)* (-30);
		angle_ref_R =  -(PI)  - (1/Gr)* (-30);
					}
					
					if((count>=39999) && (count<49999))
					{	
		angle_ref_L =   (-2*PI)  - (1/Gr)* (-30);
		angle_ref_R =  -(-2*PI)  - (1/Gr)* (-30);
					}
		
					if(count>=49999)
					{	
		angle_ref_L =   0;
		angle_ref_R =  	0;
		Upload_Angle_Reference = 0;
		count =0;
					}
		
		

			
			
		}    */



// Cycle for Trise = 0.4s
/*  		if(Upload_Angle_Reference == 1)   {
			
			count++;
			
					if((count>=0) && (count<9999))
					{	
		angle_ref_L =  0  - (1/Gr)* 0;
		angle_ref_R =  0  - (1/Gr)* 0;
					}	

					if((count>=9999) && (count<19999))
					{	
		angle_ref_L =  0  - (1/Gr)* -30;
		angle_ref_R =  0  - (1/Gr)* -30;
					}

					if((count>=19999) && (count<29999))
					{	
		angle_ref_L =   (-2*PI)  - (1/Gr)* -30;
		angle_ref_R =  -(-2*PI)  - (1/Gr)* -30;
					}
		
					if((count>=29999) && (count<39999))
					{	
		angle_ref_L =   (0)  - (1/Gr)* (-15);
		angle_ref_R =  -(0)  - (1/Gr)* (-15);
					}
					
					if((count>=39999) && (count<49999))
					{	
		angle_ref_L =   (2*PI)  - (1/Gr)* (0);
		angle_ref_R =  -(2*PI)  - (1/Gr)* (0);
					}
		
					if(count>=49999)
					{	
		angle_ref_L =   0;
		angle_ref_R =  	0;
		Upload_Angle_Reference = 0;
		count =0;
					}

			
		}   */ 




		///***	LEFT MOTOR CONTROL LOOPS	***///





			/* Position loop */


			angle_err_L = angle_ref_L - theta_L;

			w_ref_L = Kp_ang * angle_err_L;


			w_ref_L = mwlimit(w_ref_L, w_motor_limit);

		
		
				/* Speed loop and DOB compensation */

				w_err_L = w_ref_L - w_L;

				Iq_ref_L_speed = Kp_speed * w_err_L;

		

				Iq_ref_L_DOB = (1-Forg_Fact_DOB)*Iq_ref_L_DOB_prev + Forg_Fact_DOB*Iq_ref_L ;

		
				current_dob_L = ((gdr / Ktn) * (Ktn * (0.5 * TS) * (Iq_ref_L_DOB + Iq_ref_L_DOB_prev) - J * (w_L - w_L_0)-B * (0.5 * TS) * (w_L + w_L_0)) + (1 - gdr * (0.5 * TS)) * prev_dob_L) / (1 + gdr * (0.5 * TS));

				Iq_ref_L_DOB_prev = Iq_ref_L_DOB;

				prev_dob_L = current_dob_L;

	
				Iq_ref_L = Iq_ref_L_speed + current_dob_L ;



		/*  Current loop and Field Oriented Control  */

		Id_ref_L = 0.0;

		Id_ref_L = mwlimit(Id_ref_L, currentLimit);
		Iq_ref_L = mwlimit(Iq_ref_L, currentLimit);

		Id_err_L = Id_ref_L - Id_L;
		Iq_err_L = Iq_ref_L - Iq_L;

		Vd_ref_P_L = Kp_current * Id_err_L;
		Vd_ref_FF_L = -w_L * pp * L * Iq_L;

		Vd_ref_I_L = Vd_ref_I_L + Ki_current * (0.5 * TS) * (Id_err_L + prev_Id_err_L);

		Vd_ref_L = Vd_ref_P_L + Vd_ref_FF_L + Vd_ref_I_L;




			/*  Space Vector Modulation  */


			if (abs(Vd_ref_L) > Vdc_set * 0.707)
			{

				Sat_d_L = 1;

				Vd_ref_L = mwlimit(Vd_ref_L, (Vdc_set * 0.707));
			}
			else
			{
				Sat_d_L = 0;
			}

			if (abs(Vd_ref_L) > Vdc_set * 0.707)
			{

				Sat_d_L = 1;

				Vd_ref_L = mwlimit(Vd_ref_L, (Vdc_set * 0.707));
			}
			else
			{
				Sat_d_L = 0;
			}	

			if (((Vd_ref_L > 0) && ((Id_ref_L - Id_L) > 0)) || ((Vd_ref_L < 0) && ((Id_ref_L - Id_L) < 0)))
			{
				Sign_d_L = 1;
			}
			else
			{
				Sign_d_L = 0;
			}

			if (Sign_d_L && Sat_d_L == 1)
			{ //Id_ref_L is 0

				Vd_ref_I_L = Vd_ref_I_L - Ki_current * (0.5 * TS) * (Id_err_L + prev_Id_err_L);
			}

			Vd_ref_L = mwlimit(Vd_ref_L, (Vdc_set * 0.707));

			Vq_ref_P_L = Kp_current * Iq_err_L;
			Vq_ref_FF_L = +w_L * pp * L * Id_L + w_L * pp * lamda_mg;

			Vq_ref_I_L = Vq_ref_I_L + Ki_current * (0.5 * TS) * (Iq_err_L + prev_Iq_err_L);

			Vq_ref_L = Vq_ref_P_L + Vq_ref_FF_L + Vq_ref_I_L;

			if (abs(Vq_ref_L) > Vdc_set * 0.707)
			{

				Sat_q_L = 1;

				Vq_ref_L = mwlimit(Vq_ref_L, (Vdc_set * 0.707));
			}
			else
			{
				Sat_q_L = 0;
			}

			if (((Vq_ref_L > 0) && ((Iq_ref_L - Iq_L) > 0)) || ((Vq_ref_L < 0) && ((Iq_ref_L - Iq_L) < 0)))
			{
				Sign_q_L = 1;
			}
			else
			{
				Sign_q_L = 0;
			}

			if ((Sign_q_L == 1) && (Sat_q_L == 1))
			{ //Iq_ref_L can be 0

				Vq_ref_I_L = Vq_ref_I_L - Ki_current * (0.5 * TS) * (Iq_err_L + prev_Iq_err_L);
				rid = rid + 1;
			}

			Vq_ref_L = mwlimit(Vq_ref_L, (Vdc_set * 0.707));

			dq2ab(Vd_ref_L, Vq_ref_L, eAngle_L, &Va_ref_L, &Vb_ref_L);
			ab2uvw(Va_ref_L, Vb_ref_L, &Vu_ref_L, &Vv_ref_L, &Vw_ref_L);

			Vdq_mag_L = Va_ref_L * Va_ref_L + Vb_ref_L * Vb_ref_L;
			Vdq_mag_L = mwsqrt(Vdq_mag_L);

			a_L = mwarctan2(Vb_ref_L, Va_ref_L);

			r_L = Vdq_mag_L / (Vdc_set * 0.8164965809); //Maxinum voltage delivered in SVM
			r_L = mwlimit(r_L, 0.866);			 // Maxinum voltage without disortion --> on the phase I have maxinum




////////////////////////////

	
	///***	RIGHT MOTOR CONTROL LOOPS	***///

	angle_err_R = angle_ref_R - theta_R;

	w_ref_R = Kp_ang * angle_err_R;

		
		w_ref_R = mwlimit(w_ref_R, w_motor_limit);

		w_err_R = w_ref_R - w_R;

		Iq_ref_R_speed = Kp_speed * w_err_R;

		
		Iq_ref_R_DOB = (1-Forg_Fact_DOB)*Iq_ref_R_DOB_prev + Forg_Fact_DOB*Iq_ref_R ;


		current_dob_R = ((gdr / Ktn) * (Ktn * (0.5 * TS) * (Iq_ref_R_DOB + Iq_ref_R_DOB_prev) - J * (w_R - w_R_0)-B * (0.5 * TS) * (w_R + w_R_0)) + (1 - gdr * (0.5 * TS)) * prev_dob_R) / (1 + gdr * (0.5 * TS));

		Iq_ref_R_DOB_prev = Iq_ref_R_DOB;

		prev_dob_R = current_dob_R;


		Iq_ref_R = Iq_ref_R_speed + current_dob_R ;

	

			Id_ref_R = 0.0;

			Id_ref_R = mwlimit(Id_ref_R, currentLimit);
			Iq_ref_R = mwlimit(Iq_ref_R, currentLimit);

			Id_err_R = Id_ref_R - Id_R;
			Iq_err_R = Iq_ref_R - Iq_R;

			Vd_ref_P_R = Kp_current * Id_err_R;
			Vd_ref_FF_R = -w_R * pp * L * Iq_R;

			Vd_ref_I_R = Vd_ref_I_R + Ki_current * (0.5 * TS) * (Id_err_R + prev_Id_err_R);

			Vd_ref_R = Vd_ref_P_R + Vd_ref_FF_R + Vd_ref_I_R;

			if (abs(Vd_ref_R) > Vdc_set * 0.707)
			{

				Sat_d_R = 1;

				Vd_ref_R = mwlimit(Vd_ref_R, (Vdc_set * 0.707));
			}
			else
			{
				Sat_d_R = 0;
			}

			if (abs(Vd_ref_R) > Vdc_set * 0.707)
			{

				Sat_d_R = 1;

				Vd_ref_R = mwlimit(Vd_ref_R, (Vdc_set * 0.707));
			}
			else
			{
				Sat_d_R = 0;
			}

			if (((Vd_ref_R > 0) && ((Id_ref_R - Id_R) > 0)) || ((Vd_ref_R < 0) && ((Id_ref_R - Id_R) < 0)))
			{
				Sign_d_R = 1;
			}
			else
			{
				Sign_d_R = 0;
			}

			if (Sign_d_R && Sat_d_R == 1)
			{ //Id_ref_R is 0

				Vd_ref_I_R = Vd_ref_I_R - Ki_current * (0.5 * TS) * (Id_err_R + prev_Id_err_R);
			}

			Vd_ref_R = mwlimit(Vd_ref_R, (Vdc_set * 0.707));

			Vq_ref_P_R = Kp_current * Iq_err_R;
			Vq_ref_FF_R = +w_R * pp * L * Id_R + w_R * pp * lamda_mg;

			Vq_ref_I_R = Vq_ref_I_R + Ki_current * (0.5 * TS) * (Iq_err_R + prev_Iq_err_R);

			Vq_ref_R = Vq_ref_P_R + Vq_ref_FF_R + Vq_ref_I_R;

			if (abs(Vq_ref_R) > Vdc_set * 0.707)
			{

				Sat_q_R = 1;

				Vq_ref_R = mwlimit(Vq_ref_R, (Vdc_set * 0.707));
			}
			else
			{
				Sat_q_R = 0;
			}

			if (((Vq_ref_R > 0) && ((Iq_ref_R - Iq_R) > 0)) || ((Vq_ref_R < 0) && ((Iq_ref_R - Iq_R) < 0)))
			{
				Sign_q_R = 1;
			}
			else
			{
				Sign_q_R = 0;
			}

			if ((Sign_q_R == 1) && (Sat_q_R == 1))
			{ //Iq_ref_R can be 0

				Vq_ref_I_R = Vq_ref_I_R - Ki_current * (0.5 * TS) * (Iq_err_R + prev_Iq_err_R);
				rid = rid + 1;
			}

			Vq_ref_R = mwlimit(Vq_ref_R, (Vdc_set * 0.707));

			dq2ab(Vd_ref_R, Vq_ref_R, eAngle_R, &Va_ref_R, &Vb_ref_R);
			ab2uvw(Va_ref_R, Vb_ref_R, &Vu_ref_R, &Vv_ref_R, &Vw_ref_R);

			Vdq_mag_R = Va_ref_R * Va_ref_R + Vb_ref_R * Vb_ref_R;
			Vdq_mag_R = mwsqrt(Vdq_mag_R);

			a_R = mwarctan2(Vb_ref_R, Va_ref_R);

			r_R = Vdq_mag_R / (Vdc_set * 0.8164965809); 
			r_R = mwlimit(r_R, 0.866);			 

		/////////////////////////////////////////////////////////

	



		/////////////////////////////////////////////////////////
	

		break;
	}

	pev_inverter_set_ra(BDN, CH0, r_R, a_R); // When you have  r_R =1 --> Vuv_RMS = 0.707*Vdc
	pev_inverter_set_ra(BDN, CH1, r_L, a_L);


/*	Variables upload	*/

	prev_Id_err_R = Id_err_R;
	prev_Iq_err_R = Iq_err_R;
	
	
	angle_R_1 =angle_R_0;
	angle_R_0 = angle_R;
	
	w_R_1 = w_R_0;
	w_R_0 = w_R;
	
	Iu_R_prev = Iu_R;
	Iw_R_prev = Iw_R;

	
	
	
	prev_Id_err_L = Id_err_L;
	prev_Iq_err_L = Iq_err_L;
		
	angle_L_1 =angle_L_0;
	angle_L_0 = angle_L;
	
	w_L_1 = w_L_0;
	w_L_0 = w_L;
	
	Iu_L_prev = Iu_L;
	Iw_L_prev = Iw_L;


	if (Check_Execution_Time == 1){
		
		end = timer0_read();
		Check_Execution_Time =0;
		
		Execution_Time = (end - start)*0.0177777;  //(us)		
	
	}
}


///***	Interrupt for the Z-signal of the encoders	***///

interrupt void z_int_R(void)
{

	Z_R = pios_abz_in_pin(BDN, CH0);
	Z_R = Z_R >> 2;
	Z_R = Z_R & 1;

	Z_L = pios_abz_in_pin(BDN, CH1);
	Z_L = Z_L >> 2;
	Z_L = Z_L & 1;



	if (Z_R == 0)
	{
		Z_R = 1;
		if (z_flag_R == 0)
		{

			pios_abz_clear(BDN, CH0); // reset the Z-signal of the encoder
			//angle_mem_R = pios_abz_read(BDN, CH0)*resolution_rotary;
			z_flag_R = 1;
		}
		else if (z_flag_R == 1)
		{
			if ((encoder_R > 1600) || ((encoder_R < 0) && (encoder_R > -1600)))
			{
				turn_num_R = turn_num_R + 1;
			}
			else
			{
				turn_num_R = turn_num_R - 1;
			}
		}
	}

	if (Z_L == 0)
	{
		Z_L = 1;
		if (z_flag_L == 0)
		{

			pios_abz_clear(BDN, CH1);
			//angle_mem_L = pios_abz_read(BDN, CH1)*resolution_rotary;
			z_flag_L = 1;
		}
		else if (z_flag_L == 1)
		{
			if ((encoder_L > 1600) || ((encoder_L < 0) && (encoder_L > -1600)))
			{
				turn_num_L = turn_num_L + 1;
			}
			else
			{
				turn_num_L = turn_num_L - 1;
			}
		}
	}

	if (z_flag_R == 1 && z_flag_L == 1 && Initial_Setup == 0)
	{

		pios_abz_clear(BDN1, CH0);
		pios_abz_clear(BDN1, CH1);
		theta_R_init = pios_abz_read(BDN, CH0) * resolution_rotary;
		theta_L_init = pios_abz_read(BDN, CH1) * resolution_rotary;

		Initial_Setup = 1;
	}
}








void main(void)
{

	///***	Pe-Experts3 setup	***///
	
	while (start == 0)
	{
		wait(2e6 / FS);
	}

	timer0_init(1000000);
	timer0_start();
	pev_ad_set_range(BDN, GRP0, VADR, VADR, VADR, VADR);
	pev_ad_set_range(BDN, GRP1, VADR, VADR, VADR, VADR);

	int_disable();
	watch_init();
	pev_init(BDN);
	pev_inverter_init(BDN, CH0, FS, DEADT);
	pev_inverter_init(BDN, CH1, FS, DEADT);
	pev_inverter_set_syncint(BDN, 5.0);

	pios_abz_clear(BDN, CH0);
	pios_abz_clear(BDN, CH1);
	pios_abz_clear(BDN1, CH0);
	pios_abz_clear(BDN1, CH1);
	
	pev_inverter_enable_up_int5(BDN);
	int5_init_vector(carrier_interrupt);

	pios_abz_enable_int6(BDN, CH0); //enable z interupt for R-encoder
	pios_abz_enable_int6(BDN, CH1); //enable z interupt for L-encoder
									//	pios_abz_enable_int6(BDN1, CH0); //enable z interupt for M-encoder

	int6_init_vector(z_int_R);
	int6_enable_int();

	//pios_abz_disable_int6(BDN, CH0); //disable z interupt for R-encoder
	//pios_abz_disable_int6(BDN, CH1); //disable z interupt for L-encoder

	int5_enable_int();
	int_enable(); //enable all interupts

	pios_abz_disable_clear(BDN, CH0);
	pios_abz_disable_clear(BDN, CH1);
	pios_abz_disable_clear(BDN1, CH0);
	pios_abz_disable_clear(BDN1, CH1);

	wait(2e6 / FS);

	pev_inverter_set_ra(BDN, CH0, 0, 0);
	pev_inverter_set_ra(BDN, CH1, 0, 0);

	pev_inverter_start_pwm(BDN, CH0);
	pev_inverter_start_pwm(BDN, CH1);
	/* */
	wdt_init((TS * 10.0) * 1e6);

	/*	Current loop gains and low-pass filters setup	*/

	Kp_current = L * w_current;
	Ki_current = R * w_current;
	
	Forg_Fact_current = 1 - exp(-f_Current_filter * TS);
	Forg_Fact_DOB = 1 - exp(-f_DOB_filter * TS);
	
	while (1)
	{

		watch_data_8ch();

	}
}



