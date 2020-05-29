/* controller.h
 *
 *  Created on: 2018Äê5ÔÂ2ÈÕ
 *      Author: BG5CPU
 */

#include "stm32f7xx_hal.h"

#define CNTL_LEVEL_OPENLOOP		0
#define CNTL_LEVEL_CURR			1
#define CNTL_LEVEL_SPEED		2
#define CNTL_LEVEL_POS			3

#define CONTROL_FREQ	20000
#define PI				3.141592653L
#define SPD_FIL_FREQ	70.0

#define MOTOR_PP	7

#define FAKE_GAIN 1.0f

typedef struct
{
	float currA,currB,currC;
	float curr_alpha,curr_beta;
	float currD,currQ;
	float vol_alpha,vol_beta;
	float volD,volQ;
	float volA,volB,volC;
	float theta;
	float mechanic_angle;
	float corr_theta;
	float speed;
	uint8_t angle_locked;
	uint8_t fault_detect;
}MOTOR;


typedef struct
{
	float fake_volA,fake_volB,fake_volC;
	float fake_theta;
	
}OPEN_LOOP_MOTOR;


void update_curr(uint16_t currentA,uint16_t currentB,uint16_t currentC);
void init_control(void);
void controltask(void);


