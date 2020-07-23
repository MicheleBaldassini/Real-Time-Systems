//-----------------------------------------------------------------------------
// CONTROL MOTOR
// In this file there are functions implementation for controlling the control
// loop. Code for motor, filter, PID and closed-loop function
//-----------------------------------------------------------------------------

#include "control_motor.h"

// initialize buffer input and buffer output of the motor
void init_motor(Motor_t *pan_motor, Motor_t *tilt_motor) {
	int	i;
	for (i=0; i<MOTOR_BUFF; i++){
		pan_motor->input[i] = tilt_motor->input[i] = 0;
		pan_motor->output[i] = tilt_motor->output[i] = 0;
	}
	pan_motor->index = tilt_motor->index = 0;
}


// Motor code (simulation of dc motor)
float update_motor(Motor_t *motor, float control) {

	int	k, k1, k2;
	float	K, TAU, P, A, B;

	k = motor->index;
	// k1 is the previous value
	k1 = (k >= 1) ? (k - 1) : (MOTOR_BUFF - 1);
	// k2 is the second previous value
	k2 = (k >= 2) ? (k - 2) : (k1 == 0) ? (MOTOR_BUFF - 1) : (k1 - 1);

	motor->input[k] = control;

	K = KT / ((R * b) + (KT * KB));
	TAU = (R * J) / ((R * b) + (KT * KB));
	// motor parameter
	P = exp(-TS / TAU);
	A = K * (TS - TAU + (P * TAU));
	B = K * (TAU - (P * TS) - (P * TAU));
	// calculate the k output
	motor->output[k] = ((A * motor->input[k1]) + (B * motor->input[k2]) + 
						((1 + P) * motor->output[k1]) - 
						(P * motor->output[k2]));
	motor->index = (k + 1) % MOTOR_BUFF;

	return motor->output[k];
} 


// initialize PID CONTROLLER
void init_PID(Pid_t *pan_pid, Pid_t *tilt_pid) {
	// set prev and integrated error to zero
	pan_pid->prev_error = tilt_pid->prev_error = 0;
	pan_pid->int_error = tilt_pid->int_error = 0;
	pan_pid->proportional_gain = tilt_pid->proportional_gain = KP;
	pan_pid->integral_gain = tilt_pid->integral_gain = KI;
	pan_pid->derivative_gain = tilt_pid->derivative_gain = KD;
	pan_pid->windup_guard = tilt_pid->windup_guard = PI/VOLT;
}


// control the plant input throught the PID
// the PID output will be the input for the motor
float update_PID(Pid_t *pid, float curr_error, float dt) {
	float	diff;		// difference term
	float	p_term;		// proportional term
	float	i_term;		// integrative term
	float	d_term;		// derivative term

	// integration with windup guarding
	pid->int_error += (curr_error * dt);
	if (pid->int_error < -(pid->windup_guard))
		pid->int_error = -(pid->windup_guard);
	else if (pid->int_error > pid->windup_guard)
		pid->int_error = pid->windup_guard;

	// differentiation
	diff = ((curr_error - pid->prev_error) / dt);
	// scaling
	p_term = (pid->proportional_gain * curr_error);
	i_term = (pid->integral_gain * pid->int_error);
	d_term = (pid->derivative_gain * diff);
	// summation of terms
	pid->control = p_term + i_term + d_term;
	// save current error as previous error for next iteration
	pid->prev_error = curr_error;
	return pid->control;
}

// initialize filter in the closed loop
void init_filter(Filter_t *pan_filter, Filter_t *tilt_filter) {
	int	i;

	for (i=0; i<FILTER_BUFF; i++){
		pan_filter->input[i] = tilt_filter->input[i] = 0;
		pan_filter->output[i] = tilt_filter->output[i] = 0;
	}
	pan_filter->index = tilt_filter->index = 0;
}

// filter is situated in the closed-loop
// the input of the filter is the value returned from camera 
// (the rotatotional angle that indicates the actual direction of the camera)
float update_filter(Filter_t* filter, float control) {
	int	k, k1;
	float	TAU, P;

	k = filter->index;
	// k1 is the previous value
	k1 = (k >= 1) ? (k - 1) : (FILTER_BUFF - 1);
	filter->input[k] = control;

	// filter constant
	TAU = (R * J) / ((R * b) + (KT * KB));
	P = exp(-TS / TAU);
	// calculate the k output
	filter->output[k] = ((P * filter->output[k1]) + 
						((1 - P) * filter->input[k1]));
	filter->index = (k + 1) % FILTER_BUFF;
	return filter->output[k];
}


// Control motor input (closed-loop)
float control_motor(Motor_t *motor, Pid_t *pid, Filter_t *filter,
					float des_pos, float pos, float dt, motor_type type) {

	float	output;					//the plant output
	float	err;					// error
	float	control;				// PID output
	// negative feedback 
	if(type == PAN)			
		// pan filter (connected to the pan motor)
		err = des_pos - update_filter(filter, cos(pos) * PI);
	else 
		// tilt filter (connected to the tilf motor) 
		err = des_pos - update_filter(filter, sin(pos) * PI);

		control = update_PID(pid, err, dt);			// compute control output
	output = update_motor(motor, (control / VOLT));	// compute plant output
	return output;
}

//-----------------------------------------------------------------------------