#include "pid.h"

void pid_init(pid_data *pid) {

	/* Clear controller variables */
  pid->proportional = 0.0f;
  pid->integral = 0.0f;
	pid->error_prev  = 0.0f;
	pid->derivative  = 0.0f;
	pid->measurement_prev = 0.0f;
	pid->pid_out = 0.0f;
}

float pid_update(pid_data *pid, float set_point, float measurement) {

  float error = set_point - measurement;

  pid->proportional = pid->Kp * error;

  pid->integral = pid->integral + 0.5f * pid->Ki * pid->interval * (error + pid->error_prev);
  if (pid->integral > pid->integral_max) {
  pid->integral = pid->integral_max;
  } else if (pid->integral < pid->integral_min) {
  pid->integral = pid->integral_min;
  }

  pid->derivative = - (pid->Kd * (measurement - pid->measurement_prev) / pid->interval);

  pid->pid_out = pid->proportional + pid->integral + pid->derivative;
  if (pid->pid_out > pid->output_max){
  pid->pid_out = pid->output_max;
  }
  if (pid->pid_out < pid->output_min){
  pid->pid_out = pid->output_min;
  }

  pid->error_prev = error;
  pid->measurement_prev = measurement;
  
  return pid->pid_out;
}