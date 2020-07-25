#ifndef PID_H_
#define PID_H_

typedef struct {
  /* Controller gains */
  float kp;
  float ki;
  float kd;

  /* Output limits */
  float limMin;
  float limMax;

  float tau;

  /* Integrator limits */
  float limitMinInt;
  float limitMaxInt;

  /* Sample time (in seconds) */
  float T;

  /* Controller "memory" */
  float integrator;
  float prevError;
  float differentiator;
  float prevMeasurement;

  /* Controller output */
  float out;
} pid_ctrl_t;

/* Initializes a PID controller */
void pid_init(pid_ctrl_t *pid);

/* Sets the gains of the given PID */
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd);

/* Returns the proportional gains of the controller. */
void pid_get_gains(const pid_ctrl_t *pid, float *kp, float *ki, float *kd);

/* Returns the limit of the PID integrator. */
float pid_get_integral_limit(const pid_ctrl_t *pid);

/* Returns the value of the PID integrator. */
float pid_get_integral(const pid_ctrl_t *pid);

/* Sets a maximum value for the PID integrator. */
void pid_set_integral_limit(pid_ctrl_t *pid, float max);

/* Resets the PID integrator to zero. */
void pid_reset_integral(pid_ctrl_t *pid);

/* Process one step of the PID algorithm. */
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement);

#endif

