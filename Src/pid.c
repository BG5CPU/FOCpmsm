/*
 * pid.c: pid regulator
 * adapted from ti.
 */

#include "pid.h"

void pid_calc(PIDREG *v)
{	
    // Compute the error
    v->Err = v->Ref - v->Fdb;
    // Compute the proportional output
    v->Up = v->Kp * v->Err;
    // Compute the integral output
    v->UiPreSat = v->Ui + v->Ki * v->Up + v->Kc * v->SatErr;
   
   // Saturate the Ui
    if (v->UiPreSat > v->UiMax)
      v->Ui =  v->UiMax;
    else if (v->UiPreSat < v->UiMin)
      v->Ui =  v->UiMin;
    else
      v->Ui = v->UiPreSat;
    // Compute the derivative output
    v->Ud = v->Kd * (v->Up - v->Up1);

    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;

    // Saturate the output
    if (v->OutPreSat > v->OutMax)
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;
    else
      v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;
    // Update the previous proportional output
    v->Up1 = v->Up; 
	
	return;
}


