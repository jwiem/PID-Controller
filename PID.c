#include "pid.h"

#include <math.h>
#include <stdlib.h>

void pid_init(pid_ctrl_t *pid) {
  pid_set_gains(pid, 1., 0., 0.);

  pid->integrator = 0.0f;
  pid->prevError = 0.0f;
  pid->differentiator = 0.0f;
  pid->prevMeasurement = 0.0f;

  pid->out = 0.0f;
}

void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void pid_get_gains(const pid_ctrl_t *pid, float *kp, float *ki, float *kd) {
  *kp = pid->kp;
  *ki = pid->ki;
  *kd = pid->kd;
}

float pid_get_integral_limit(const pid_ctrl_t *pid) {
  return pid->limitMaxInt;
}

float pid_get_integral(const pid_ctrl_t *pid) {
  return pid->integrator;
}

void pid_set_integral_limit(pid_ctrl_t *pid, float max) {
  pid->limitMaxInt = max;
}

void pid_reset_integral(pid_ctrl_t *pid) {
  pid->integrator = 0.0f;
}

float pid_update(pid_ctrl_t *pid, float setpoint, float measurement) {
  float error = setpoint - measurement;

  float proportional = pid->kp * error;

  pid->integrator = pid->integrator + 0.5f * pid->ki * pid->ki * pid->T * (error + pid->prevError);

  if (pid->integrator > pid->limitMaxInt) {
    pid->integrator = pid->limitMaxInt;
  } else if (pid->integrator < pid->limitMinInt) {
    pid->integrator = pid->limitMinInt;
  }

  pid->differentiator = -(2.0f * pid->kd * (measurement - pid->prevMeasurement) 
                        + (2.0f * pid->tau - pid->T) * pid->differentiator) 
                        / (2.0f * pid->tau + pid->T);

  /*
	* Compute output and apply limits
	*/
  pid->out = proportional + pid->integrator + pid->differentiator;

  if (pid->out > pid->limMax) {
    pid->out = pid->limMax;
  } else if (pid->out < pid->limMin) {
    pid->out = pid->limMin;
  }

  /* Store error and measurement for later use */
  pid->prevError = error;
  pid->prevMeasurement = measurement;

  /* Return controller output */
  return pid->out;
}
