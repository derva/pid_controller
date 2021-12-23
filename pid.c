/* INTEGRATION AND DERIVATIVE 
 * can be found here:
 *
 * https://github.com/pms67/PID/blob/master/PID.c
 * */
#include "pid.h"
#include <stdio.h>

/* void PIDInit(PIDController* pid, float kp, float ki, float kd) { */
void PIDInit(PIDController* pid) {
  /* pid->Kp = kp; */
  /* pid->Ki = ki; */
  /* pid->Kd = kd; */

  pid->integrator = 0.0f;
  pid->prevError= 0.0f;

  pid->differentiator= 0.0f;
  pid->prevMeasurement= 0.0f;
  
  pid->out= 0.0f;


}

void printPID(PIDController* pid) {
  printf("%f %f %f\n", pid->Kp, pid->Ki, pid->Kd);
}

double PIDUpdate(PIDController* pid, double setpoint, double measurement) {
  float error = setpoint - measurement;

  pid->proportional = pid->Kp * error;

  /* INTEGRATOR */

  pid->integrator =
      pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

  if (pid->integrator > pid->limMaxInt) {
    pid->integrator = pid->limMaxInt;
  } else if (pid->integrator < pid->limMinInt) {
    pid->integrator = pid->limMinInt;
  }

  /* DERIVATIVE */
  pid->differentiator =
      -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) +
        (2.0f * pid->tau - pid->T) * pid->differentiator) /
      (2.0f * pid->tau + pid->T);

  pid->out = pid->proportional + pid->integrator + pid->differentiator;
  if (pid->out > pid->limMax) {
    pid->out = pid->limMax;
  } else if (pid->out < pid->limMin) {
    pid->out = pid->limMin;
  }

  pid->prevError = error;
  pid->prevMeasurement = measurement;
  return pid->out;
}

