//-----------------------------------------------------------------------------
// CONTROL MOTOR header file for the control loop implementation
//-----------------------------------------------------------------------------

#ifndef CONTROL_MOTOR_H
#define CONTROL_MOTOR_H

#include <math.h>

#define	KP	0.1			// proportional gain
#define	KD	0.02		// derivative gain
#define	KI	0.03		// integral gain

#define	PI	3.14		// pi greco

#define	VOLT	1.256	// volt value for scaling (angle(2*PI) = 5 * VOLT) 

#define	MOTOR_BUFF	3	// number of value needed for calculate motor output
#define	FILTER_BUFF	2	// number of value needed for calculate filter output

typedef enum {PAN, TILT}	motor_type;

// motor constant
extern float	KT;
extern float	KB;
extern float	J;
extern float	b;
extern int	R;

// sampling time
extern float	TS;


typedef struct Motor {
	int	index;
	float	input[MOTOR_BUFF];
	float	output[MOTOR_BUFF];
} Motor_t;


typedef struct Filter {
	int	index;
	float	input[FILTER_BUFF];
	float	output[FILTER_BUFF];
} Filter_t;


typedef struct Pid {
	float	windup_guard;
	float	proportional_gain;
	float	integral_gain;
	float	derivative_gain;
	float	prev_error;
	float	int_error;
	float	control;
} Pid_t;

extern void init_filter(Filter_t*, Filter_t*);

extern float update_filter(Filter_t*, float);

extern void init_motor(Motor_t*, Motor_t*);

extern float update_motor(Motor_t*, float);

extern void init_PID(Pid_t*, Pid_t*);

extern float update_PID(Pid_t*, float, float);

extern float control_motor(Motor_t*, Pid_t*, Filter_t*, 
							float, float, float, motor_type);


#endif

//-----------------------------------------------------------------------------