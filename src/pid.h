#ifdef __cplusplus
extern "C" {
#endif

#ifndef PID_H
#define PID_H

typedef struct {

	// Controller gains
	float Kp;
	float Ki;
	float Kd;

	// Output limits
    float output_max;
    float output_min;
	
	// Integrator limits
	float integral_max;
	float integral_min;

	// Sample time [ms]
	unsigned long interval;

	// Controller "memory"
	float proportional;
    float integral;
	float error_prev;			// Required for integrator
	float derivative;
	float measurement_prev;		// Required for differentiator

	// Controller output
	float pid_out;

} pid_data;

void  pid_init(pid_data *pid);
float pid_update(pid_data *pid, float set_point, float measurement);

#endif

#ifdef __cplusplus
}
#endif