/*
 * pid.h: pid regulator.
 * adapted from Texas Instruments.
 */
#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float Ref;   			// Input: Reference input
	float Fdb;   			// Input: Feedback input
	float Err;				// Variable: Error
	float Kp;				// Parameter: Proportional gain
	float Up;				// Variable: Proportional output
	float Ui;				// Variable: Integral output
	float Ud;				// Variable: Derivative output
	float OutPreSat; 		// Variable: Pre-saturated output
	float OutMax;		    // Parameter: Maximum output
	float OutMin;	    	// Parameter: Minimum output
	float Out;   			// Output: PID output
	float SatErr;			// Variable: Saturated difference
	float Ki;			    // Parameter: Integral gain
	float Kc;		     	// Parameter: Integral correction gain
	float Kd; 		        // Parameter: Derivative gain
	float Up1;		   	    // History: Previous proportional output
	float UiPreSat;
	float UiMax;
	float UiMin;
} PIDREG;

/* Default initalizer for the PIDREG object. */ 
#define PIDREG_DEFAULTS \
{\
  0,   /* Ref */\
  0,   /* Fdb */\
  0,   /* Err */\
  0,   /* Kp */\
  0,   /* Up */\
  0,   /* Ui */\
  0,   /* Ud */\
  0,   /* OutPreSat */\
  0.5,   /* OutMax */\
 -0.5,   /* OutMin */\
  0,   /* Out */\
  0,   /* SatErr */\
  0.001,   /* Ki */\
  0.2,   /* Kc */\
  0,  /* Kd */\
  0,  /* Up1 */\
  0,  /* UiPreSat */\
  0.1,   /* UiMax */\
 -0.1,   /* UiMin */\
}

void pid_calc(PIDREG *v);

#endif 

