#pragma once

typedef struct{
  /* PID Controller parameters */
  float Kp;
  float Ki;
  float Kd;

  float tau;

  float limMin;
  float limMax;

  float limMinInt;
  float limMaxInt;

  float T;

  float integrator;
  float prevError;
  float differentiator;
  float prevMeasurement;

  // float error;
  // float setpoint;


  float proportional;
  float out;

} PIDController;
 
// void PIDInit(PIDController* pid, float kp, float ki, float kd);
void PIDInit(PIDController* pid);
void printPID(PIDController* pid);
double PIDUpdate(PIDController* pid, double setpoint, double measurement);
