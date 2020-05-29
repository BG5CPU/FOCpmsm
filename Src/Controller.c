/* controller.c
 *
 *  Created on: 2018Äê5ÔÂ2ÈÕ
 *      Author: BG5CPU
 */

#include "main.h"
#include "controller.h"
#include "pid.h"
#include <math.h>

/* test fixture */
//#define FIX_POSITION_OUT

#define SPEED_FIL_CNT			1

MOTOR pmsm;
OPEN_LOOP_MOTOR fake_pmsm;
PIDREG pi_id,pi_iq,pi_spd;
uint8_t cntl_level = CNTL_LEVEL_OPENLOOP;

static uint16_t fake_a = 0,fake_b=1365,fake_c=2730;

extern const float sine_table[4096];
extern const float cosine_table[4096];

void update_curr(uint16_t currentA,uint16_t currentB,uint16_t currentC)
{
		pmsm.currA = -((int16_t)currentA - 3103)*7.5f/0.625f*3.3f/4096;
		pmsm.currB = -((int16_t)currentB - 3103)*7.5f/0.625f*3.3f/4096;	
		pmsm.currC = -((int16_t)currentC - 3103)*7.5f/0.625f*3.3f/4096;	
}

/*
 * fake_3Phase_Voltage: to generate 3Phase_Voltage
 * parameter: 2.44 rps
 */
void fake_3Phase_Voltage(void)
{
	fake_pmsm.fake_volA = FAKE_GAIN*sine_table[fake_a];
	fake_pmsm.fake_volB = FAKE_GAIN*sine_table[fake_b];
	fake_pmsm.fake_volC = FAKE_GAIN*sine_table[fake_c];
	
	fake_pmsm.fake_theta = fake_a*0.00024414;
	
	fake_a += 1;
	fake_b += 1;
	fake_c += 1;
	if(fake_a > 4095){fake_a = 0;}
	if(fake_b > 4095){fake_b = 0;}
	if(fake_c > 4095){fake_c = 0;}
}


/*
 * read_angle: read the AD2S1210 angle data.
 * XXX: not complete. to be modified.
 */
extern SPI_HandleTypeDef hspi6;
void read_angle(void)
{
	uint16_t ad2s_theta;
	
	HAL_GPIO_WritePin(SPI6_NSS_GPIO_Port, SPI6_NSS_Pin, GPIO_PIN_RESET);
	extern SPI_HandleTypeDef hspi6;
	HAL_SPI_Receive(&hspi6,(uint8_t*)&ad2s_theta,1,1000);
	HAL_GPIO_WritePin(SPI6_NSS_GPIO_Port, SPI6_NSS_Pin, GPIO_PIN_SET);
	ad2s_theta >>= 4;
	pmsm.mechanic_angle = ad2s_theta/4096.0f;
	pmsm.theta = (pmsm.mechanic_angle-pmsm.corr_theta)*MOTOR_PP;
	while(pmsm.theta>1.0f) pmsm.theta -= 1.0f;
	while(pmsm.theta<0.0f) pmsm.theta += 1.0f;
	
}


/*
 * abc2alphabeta: to perform clarke transform.
 */
void abc2alphabeta(void)
{
	pmsm.curr_alpha = 0.66667f * (pmsm.currA - 0.5f*pmsm.currB - 0.5f*pmsm.currC);
	pmsm.curr_beta = 0.66667f * 0.866f * (pmsm.currB - pmsm.currC);
}


/*
 * alphabeta2dq: to perform park transform.
 */
void alphabeta2dq(void)
{
	float cos_theta,sin_theta;
	uint16_t theta;
	theta = (uint16_t)(pmsm.theta*4095);
	cos_theta = cosine_table[theta];
	sin_theta = sine_table[theta];
	pmsm.currD = pmsm.curr_alpha*cos_theta + pmsm.curr_beta*sin_theta;
	pmsm.currQ = pmsm.curr_alpha*(-sin_theta) + pmsm.curr_beta*cos_theta;
}


/*
 * dq2alphabeta: to perform inverse park transform
 * note: input and output is voltage
 */
void dq2alphabeta(void)
{
	float cos_theta,sin_theta;
	uint16_t theta;
	theta = (uint16_t)(pmsm.theta*4095);
	cos_theta = cosine_table[theta];
	sin_theta = sine_table[theta];
	pmsm.vol_alpha = pmsm.volD*cos_theta + pmsm.volQ*(-sin_theta);
	pmsm.vol_beta  = pmsm.volD*sin_theta + pmsm.volQ*cos_theta;
}

/*
 * calc_speed: update the real speed(rpm) in pmsm model.
 */
void calc_speed(void)
{

	float delta_theta;
	static float theta_l = 0.0;
	float K1,K2;
	
	delta_theta = pmsm.theta - theta_l;
	if(pmsm.theta<0.1f && theta_l>0.9f)
		delta_theta += 1.0f;
	else if(pmsm.theta>0.9f && theta_l<0.1f)
		delta_theta -= 1.0f;

	K1 = 1/(1+CONTROL_FREQ/(2*PI*SPD_FIL_FREQ));
	K2 = 1.0f-K1;
	
	pmsm.speed = pmsm.speed*K2 + delta_theta*CONTROL_FREQ*60*K1;

	theta_l = pmsm.theta;
	
/*	
	float delta_theta;
	static float theta_l = 0.0;
	
	delta_theta = pmsm.theta - theta_l;
	
	if(pmsm.theta<0.1f && theta_l>0.9f)
		delta_theta += 1.0f;
	else if(pmsm.theta>0.9f && theta_l<0.1f)
		delta_theta -= 1.0f;
	
	pmsm.speed = delta_theta*CONTROL_FREQ*60;

	theta_l = pmsm.theta;
*/
}

void svpwm(void)
{
	float va,vb,vc,t1,t2;
	uint16_t sec;
	
	sec = 0;
	va = pmsm.vol_beta;
	vb = pmsm.vol_alpha*0.866f + pmsm.vol_beta*(-0.5f);
	vc = pmsm.vol_alpha*(-0.866f) + pmsm.vol_beta*(-0.5f);
	if(va>0) sec = 1;
	if(vb>0) sec += 2;
	if(vc>0) sec += 4;

	vb = pmsm.vol_alpha*0.866f + pmsm.vol_beta*0.5f;
	vc = pmsm.vol_alpha*(-0.866f) + pmsm.vol_beta*0.5f;

	switch(sec)
	{
	case 0:
		pmsm.volA = 0.5f;
		pmsm.volB = 0.5f;
		pmsm.volC = 0.5f;
		break;
	case 1:
		t1 = vc;
		t2 = vb;
		pmsm.volB = 0.5f*(1.0f-t1-t2);
		pmsm.volA = pmsm.volB + t1;
		pmsm.volC = pmsm.volA + t2;
		break;
	case 2:
		t1 = vb;
		t2 = -va;
		pmsm.volA = 0.5f*(1.0f-t1-t2);
		pmsm.volC = pmsm.volA + t1;
		pmsm.volB = pmsm.volC + t2;
		break;
	case 3:
		t1 = -vc;
		t2 = va;
		pmsm.volA = 0.5f*(1.0f-t1-t2);
		pmsm.volB = pmsm.volA + t1;
		pmsm.volC = pmsm.volB + t2;
		break;
	case 4:
		t1 = -va;
		t2 = vc;
		pmsm.volC = 0.5f*(1.0f-t1-t2);
		pmsm.volB = pmsm.volC + t1;
		pmsm.volA = pmsm.volB + t2;
		break;
	case 5:
		t1 = va;
		t2 = -vb;
		pmsm.volB = 0.5f*(1.0f-t1-t2);
		pmsm.volC = pmsm.volB + t1;
		pmsm.volA = pmsm.volC + t2;
		break;
	case 6:
		t1 = -vb;
		t2 = -vc;
		pmsm.volC = 0.5f*(1.0f-t1-t2);
		pmsm.volA = pmsm.volC + t1;
		pmsm.volB = pmsm.volA + t2;
		break;
	default:
		break;
	}

	pmsm.volA = 2*(pmsm.volA - 0.5f);
	pmsm.volB = 2*(pmsm.volB - 0.5f);
	pmsm.volC = 2*(pmsm.volC - 0.5f);

	if(pmsm.volA > 0.95f) pmsm.volA = 0.95f;
	else if(pmsm.volA < -0.95f) pmsm.volA = -0.95f;

	if(pmsm.volB > 0.95f) pmsm.volB = 0.95f;
	else if(pmsm.volB < -0.95f) pmsm.volB = -0.95f;

	if(pmsm.volC > 0.95f) pmsm.volC = 0.95f;
	else if(pmsm.volC < -0.95f) pmsm.volC = -0.95f;
}


void a_monitor(uint8_t channel,float variable)
{
	uint16_t dac_val;

	if(variable>1.0f) variable = 1.0f;
	else if(variable<-1.0f) variable = -1.0f;
	dac_val = (uint16_t)(variable*2047+2048);
	dac_val |= channel<<12;
	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	extern SPI_HandleTypeDef hspi3;
	HAL_SPI_Transmit(&hspi3,(uint8_t*)&dac_val,1,1000);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_RESET);
	return;
}

void init_control(void)
{
	pi_id.Kc = 0.2f;
	pi_id.Kd = 0.0;
	pi_id.Ki = 6e-4f;//1e-3;
	pi_id.Kp = 0.04f;//
	pi_id.OutMax = 0.4f;
	pi_id.OutMin = -0.4f;
	pi_id.Ud = 0.0;
	pi_id.UiMax = 0.4f;
	pi_id.UiMin = -0.4f;
	pi_id.Ui = 0.0;

	pi_iq.Kc = 0.2f;
	pi_iq.Kd = 0.0;
	pi_iq.Ki = 6e-4f;//1e-3;
	pi_iq.Kp = 0.04f;//
	pi_iq.OutMax = 0.4f;
	pi_iq.OutMin = -0.4f;
	pi_iq.Ud = 0.0;
	pi_iq.UiMax = 0.4f;
	pi_iq.UiMin = -0.4f;
	pi_iq.Ui = 0.0;

	pi_spd.Kc = 0.2f;
	pi_spd.Kd = 0.0001;//-0.01
	pi_spd.Ki = 0.005f;//1e-4;
	pi_spd.Kp = 0.6f;//1.6;
	pi_spd.OutMax = 3.2f; //3.2
	pi_spd.OutMin = -3.2f; //-3.2
	pi_spd.Ud = 0.0;
	pi_spd.UiMax = 4.0f;
	pi_spd.UiMin = -4.0f;
	pi_spd.Ui = 0.0;
	
	pmsm.corr_theta = 0.0700683594f;
	
	pi_iq.Ref = 0.1f;
	pi_spd.Ref = 120.0f*MOTOR_PP; //rpm
}

extern TIM_HandleTypeDef htim1;
void controltask()
{
	
//	if(HAL_GPIO_ReadPin(PWM_FEN_GPIO_Port,PWM_FEN_Pin) == GPIO_PIN_SET)
//	{
//		htim1.Instance->BDTR &= ~(TIM_BDTR_MOE);
//		pi_id.Ui = 0.0;
//		pi_iq.Ui = 0.0;
//	}
//	else
//	{
//		__HAL_TIM_MOE_ENABLE(&htim1);
//	}
	
	if(CNTL_LEVEL_OPENLOOP == cntl_level)
	{
		fake_3Phase_Voltage();
		pmsm.currA = fake_pmsm.fake_volA;
		pmsm.currB = fake_pmsm.fake_volB;
		pmsm.currC = fake_pmsm.fake_volC;
		abc2alphabeta();
		pmsm.vol_alpha = pmsm.curr_alpha;
		pmsm.vol_beta = pmsm.curr_beta;
		svpwm();
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)(2700+2700*pmsm.volA));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)(2700+2700*pmsm.volB));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)(2700+2700*pmsm.volC));	
		a_monitor(0,pmsm.volA);
	}
	else if(CNTL_LEVEL_CURR == cntl_level)
	{
		read_angle();
				
		abc2alphabeta();
		alphabeta2dq();
		
		pi_id.Ref = 0.0;
		pi_id.Fdb = pmsm.currD;
		
		pi_iq.Fdb = pmsm.currQ;
		
		pid_calc(&pi_id);
		pid_calc(&pi_iq);		
		
		pmsm.volD = pi_id.Out;
		pmsm.volQ = pi_iq.Out;
			
		dq2alphabeta();
		
#ifdef FIX_POSITION_OUT
		pmsm.vol_alpha = 0.5f;
		pmsm.vol_beta = 0;		
#endif	

		svpwm();
		
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)(2700+2700*pmsm.volA));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)(2700+2700*pmsm.volB));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)(2700+2700*pmsm.volC));	

	}
	else if(CNTL_LEVEL_SPEED == cntl_level)
	{
		read_angle();
		
		calc_speed();
		pi_spd.Fdb = pmsm.speed;
		pid_calc(&pi_spd);
		

		abc2alphabeta();
		alphabeta2dq();
		pi_id.Ref = 0.0f;
		pi_id.Fdb = pmsm.currD;
		pi_iq.Ref = pi_spd.Out;
		pi_iq.Fdb = pmsm.currQ;
		pid_calc(&pi_id);
		pid_calc(&pi_iq);	
		pmsm.volD = pi_id.Out;
		pmsm.volQ = pi_iq.Out;
		dq2alphabeta();
		svpwm();

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)(2700+2700*pmsm.volA));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)(2700+2700*pmsm.volB));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)(2700+2700*pmsm.volC));

//		a_monitor(0,delta_theta*100.0f);
		a_monitor(0,(pmsm.theta-0.5f)*2.0f);
//		a_monitor(0,(pmsm.speed-1680.0f)/100.0f);
	}
}




