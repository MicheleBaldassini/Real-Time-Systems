//-----------------------------------------------------------------------------
//				MAIN ---> Simulating visual tracking camera 
//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <allegro.h>
#include <time.h>
//#include "sched_deadline.h"	// Uncomment if you use SCHED_DEADLINE
#include "ptask.h"
#include "control_motor.h"


#define	BGC	0				// background color
#define	GREEN	2			// ball color
#define	BLUE	3			// camera color
#define	RED	4			// moving area border color
#define	PINK	5			// error color
#define	YELLOW	14			// text color
#define	ORANGE	6			// rect of options color
#define	GREY	8			// rect of graphic error color

#define	XWIN	800			// 640
#define	YWIN	600			// 480

#define	XMIN	40			// min position X of the ball 
#define	XMAX	760			// max position X of the ball 

#define	YMIN	40			// min position Y of the ball 
#define	YMAX	440			// max position Y of the ball 

// Uncomment if you use SCHED_DEADLINE	
//#define	PER	32			// task period in ms 
//#define	DREL	32		// realtive deadline in ms
//#define	RUNT	10		// task runtime


// Comment if you use SCHED_DEADLINE
#define	PER	20				// task period in ms 
#define	DREL	20			// realtive deadline in ms
#define	PRIO	80			// task priority 


#define	VEL	10				// velocity of ball and camera
#define	MAX_VEL	50			// max velocity of ball and camera
 
#define	L	4				// dimension of a ball 

#define	WIDTH	60			// x dimension of a rect  
#define	HEIGHT	60			// y dimension of a rect  
#define	MAX_WIDTH	360		// max x dimension of a rect  
#define	MAX_HEIGHT	360		// max y dimension of a rect   

#define	DIM	2				// position buffer dimension
#define	INCREMENT	10		// dimension of camera variation
#define	RANGE	3.14 / 6	// range per la pseudo-casualità dello spostamento
#define	SCALE	8			// error dimension

// constant used to positioning object on the graphic interface
#define	ONE	1
#define	TWO	2
#define	THREE	3
#define	FOUR	4
#define	FIVE	5
#define	SIX	6
#define	EIGHT	8
#define	TEN	10

// distance between two object in the graphic interface
// the real distance between two object is given by a constant 
// multiplying the offset
#define	OFFSET	20

#define	HUNTH	0.001		// incremento centesimale
#define	THOTH	0.0001		// incremento millesimale

#define	CTRL_K	11		// ctrl + k
#define	CTRL_R	18		// ctrl + r
#define	CTRL_J	10		// ctrl + j
#define	CTRL_B	2		// ctrl + b
#define	CTRL_T	20		// ctrl + t

#define	EXP	2.0			// used to calculate the error distance camera-ball

// Motor constant
float	KT = FIVE * HUNTH;
float	KB = FIVE * HUNTH;
float	J = FIVE * THOTH;
float	b = THOTH;
int	R = 1;

// Sampling time
float	TS = TWO * HUNTH;


static int	pausa = 0;
static int	mode = 0;


// ball struct to store info about ball
struct Ball {
	int	color;			// ball color
	float	radius;		// ball dimension
	float	x;			// x position of ball
	float	y;			// y position of ball
	float	v;			// velocity of ball
	float	a;			// angle of ball (identify moving direction)
};


// camera struct to store info about camera
struct Camera {
	int	color;			// camera color
	int	width;			// dimension of camera w.r.t. x-axis
	int	height;			// dimension of camera w.r.t. y-axis
	float	x;			// x position of centre of camera
	float	y;			// y position of centre of camera
	float	v;			// velocity of camera
	float	a;			// angle of camera (identify moving direction)
};

// structure used to store the centroid calculation from camera
struct Position {
	int	index;
	float	buffer[DIM];	// buffer used to store the centroid value
};


struct Ball	ball;

struct Camera	camera;

struct Position	position_x;
struct Position	position_y;

// mutual esclusion semaphores  
pthread_mutex_t	mxa;
pthread_mutex_t	mxb;
pthread_mutex_t	mxc;

// global image buffer
int	image[MAX_WIDTH][MAX_HEIGHT];


void init() {

	allegro_init();
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0);
	clear_to_color(screen, BGC);

	install_keyboard();
	install_mouse();
	srand(time(NULL));

	rect(screen, XMIN - 1, YMIN - 1, XMAX + 1, YMAX + 1, RED);
	textout_centre_ex(screen, font, "SPACE init", 
						XMAX/2, YMAX/2, GREEN, 0);
	textout_centre_ex(screen, font, "ESC exit", 
						XMAX/2, YMAX/2 + OFFSET + TEN, BLUE, 0);
	textout_centre_ex(screen, font, "Change motor value:", 
						XMAX/2, YMAX/2 + (THREE * OFFSET), YELLOW, 0);
	textout_ex(screen, font, "+ : ctrl & letter", 
						XMAX/2 - (THREE * OFFSET) - TEN, 
						YMAX/2 + (FOUR * OFFSET) + TEN, PINK, 0);
	textout_ex(screen, font, "- : alt & letter", 
						XMAX/2 - (THREE * OFFSET) - TEN, 
						YMAX/2 + (SIX * OFFSET), PINK, 0);

	// PRIO_CEILING semaphores
	//pmux_create_pc(&mxa, PRIO);
	//pmux_create_pc(&mxb, PRIO);
	//pmux_create_pc(&mxc, PRIO);

	// PRIO_INHERITANCE semaphores
	pmux_create_pi(&mxa);
	pmux_create_pi(&mxb);
	pmux_create_pi(&mxc);

	// Comment if you use SCHED_DEADLINE
	ptask_init(SCHED_FIFO, PRIO_INHERITANCE);	// PRIO_CEILING

	// Uncomment if you use SCHED_DEADLINE
	//ptask_init(PRIO_INHERITANCE);

}


//--------------------------------------------------------------
//			Utility function
//--------------------------------------------------------------

// random float number generation
float frand(float x, float y) {
	float	r;
	r = rand() / (float)RAND_MAX;
	return x + (y - x) * r;
}

// initialize the struct in which are stored positions of centroid
void init_position(struct Position position) {
	int	i;

	position.index = 0;
	for (i=0; i<DIM; i++)
		position.buffer[i] = 0;
}


//--------------------------------------------------------------
//				Function used by Camera Process
//--------------------------------------------------------------

// Reads an area of the screen centered in (x0,y0) 
// and stores it into image[][] 
void get_image(int image[][MAX_HEIGHT], int x0, int y0) { 
	int	i, j; 								// image indexes 
	int	x, y; 								// video coordinates

	for (i=0; i<camera.width; i++) { 
		for (j=0; j<camera.height; j++) { 
			x = x0 + i;
			y = y0 + j;
			image[i][j] = getpixel(screen, x, y); 
		} 
	}
}


// Discard black pixels and save in img_x, img_y 
// the position of the green pixel
void threshold(int image[][MAX_HEIGHT], int x0, int y0, 
				int *img_x, int *img_y) {
	int	i, j;
	*img_x = -1;
	*img_y = -1;

	for (i=0; i<camera.width; i++) {
		for (j=0; j<camera.height; j++) {
			if (image[i][j] == ball.color ) {
				*img_x = x0 + i;
				*img_y = y0 + j;
			}
			image[i][j] = BGC;
		}
	}
}


// Compute centroid for the x-axis of the ball
float compute_centroid_x(float p) {
	int	k, k1;
	float	centroid;

	k = position_x.index;
	position_x.buffer[k] = p;
	k1 = (k >= 1) ? (k - 1) : (DIM - 1);	// k1 is the previous value 

	centroid = ((position_x.buffer[k] * 2) + (position_x.buffer[k1]));
	position_x.index = (k + 1) % DIM;

	return centroid;
}


// Compute centroid for the y-axis of the ball
float compute_centroid_y(float p) {
	int	k, k1;
	float	centroid;

	k = position_y.index;
	position_y.buffer[k] = p;
	k1 = (k >= 1) ? (k - 1) : (DIM - 1);	// k1 is the previous value 

	centroid = ((position_y.buffer[k] * 2) + (position_y.buffer[k1]));
	position_y.index = (k + 1) % DIM;

	return centroid;
}


void draw_camera(int x, int y, int w, int h, int c) {
	rect(screen, x, y, (x + w), (y + h), c);
}


// initialize the camera struct
void init_camera() {
	camera.color = BLUE;
	camera.height = HEIGHT;
	camera.width = WIDTH;
	camera.x = XMAX/2;
	camera.y = YMAX/2;
	camera.v = VEL;
	camera.a = frand(-PI, PI);
}


// handle bounce of the camera with graphic board of the scenario
void handle_bounce_camera() {
	int	outl, outr, outt, outb;

	outl = (camera.x - camera.width/2 <= XMIN);
	outr = (camera.x + camera.width/2 >= XMAX);
	outt = (camera.y + camera.height/2 >= YMAX);
	outb = (camera.y - camera.height/2 <= YMIN);

	if (outl) {
		pthread_mutex_lock(&mxc);
		camera.x = XMIN + camera.width/2;
		pthread_mutex_unlock(&mxc);
	}
	if (outr) {
		pthread_mutex_lock(&mxc);
		camera.x = XMAX - camera.width/2; 
		pthread_mutex_unlock(&mxc);
	}
	if (outl || outr) {
		pthread_mutex_lock(&mxc);
		camera.a = camera.a - PI;
		pthread_mutex_unlock(&mxc);
	}
	if (outt) {
		pthread_mutex_lock(&mxc);
		camera.y = YMAX - camera.height/2;
		pthread_mutex_unlock(&mxc);
	}
	if (outb) {
		pthread_mutex_lock(&mxc);
		camera.y = YMIN + camera.height/2 ;
		pthread_mutex_unlock(&mxc);
	}
	if (outt || outb) {
		pthread_mutex_lock(&mxc);
		camera.a = - camera.a;
		pthread_mutex_unlock(&mxc);
	}
}


// enlarge camera dimension
void camera_enlarge() {
	if (camera.width < MAX_WIDTH && camera.height < MAX_HEIGHT) {
		pthread_mutex_lock(&mxc);
		camera.width += INCREMENT;
		camera.height += INCREMENT;
		pthread_mutex_unlock(&mxc);
	} 
	else return;
} 


// restrict camera dimension
void camera_restrict() {
	if (camera.width > WIDTH && camera.height > HEIGHT) {
		pthread_mutex_lock(&mxc); 
		camera.width 	-= INCREMENT;
		camera.height 	-= INCREMENT;
		pthread_mutex_unlock(&mxc);
	}
	else return;
}


// update the position of the camera
void update_camera(int x_motor, int y_motor, int img_x, int img_y, float dt) {
	float	vx, vy;
	float	alpha, delta_x, delta_y;
	float	vdx, vdy;
	// difference between the input motor and 
	// the actual position of the camera w.r.t. x-axis and y-axis
	delta_x = (x_motor - camera.x);
	delta_y = (y_motor - camera.y);

	if (img_x != -1 && img_y != -1) {
		// the ball have to be in the centre of the camera
		vdx = (img_x - camera.x) / dt;
		vdy = (img_y - camera.y) / dt;
	} else {
		vdx = 0;
		vdy = 0;
	}
	// new camera direction
	alpha = atan2(delta_y, delta_x);

	pthread_mutex_lock(&mxc);

	camera.a += alpha;

	vx = camera.v * cos(camera.a);
	vy = camera.v * sin(camera.a);

	camera.x = camera.x + (vx + vdx) * dt;
	camera.y = camera.y + (vy + vdy) * dt;

	pthread_mutex_unlock(&mxc);
}


//--------------------------------------------------------------
//						CAMERA process
//--------------------------------------------------------------

void* taskcamera(void *arg) {

	int	i;								// indice del task  
	int	ox, oy;							// vecchia posizione camera 
	int	ow, oh;							// vecchia width and height camera 
	int	img_x, img_y;					// coordinate rilevamento palla
	float	dt;							// incremento temporale    
	float	da;							// variazione dell' angolo di direzione
	float	controller_x, controller_y;	// control variable
	// motor output without control
	float	x_cord, y_cord;				// the ball is outside the camera
	float	pred_x, pred_y;				// prediction of new centroid position

	//int ret; 							// Uncomment if you use SCHED_DEADLINE

	//struct sched_attr attr;			// Uncomment if you use SCHED_DEADLINE

	struct task_par	*pdes = (struct task_par *)arg;
	i = pdes->index;

	/* 
	// Uncomment if you use SCHED_DEADLINE
	attr.size = sizeof(attr);
	attr.sched_flags = 0;
	attr.sched_nice = 0;
	attr.sched_priority = 0;
	// This creates a RUNT(ms)/PER(ms) reservation
	// PER and DREL have to be equal
	attr.sched_policy = SCHED_DEADLINE;
	attr.sched_runtime = RUNT * 1000 * 1000;
	attr.sched_period = PER * 1000 * 1000;
	attr.sched_deadline = DREL * 1000* 1000;

	ret = sched_setattr(getgid(), &attr, 0);
	if (ret < 0) {
		perror("sched_setattr");
		exit(-1);
	}
	*/

	ptask_set_period(i);
	dt = ptask_get_period(i, MILLI) / 100.;

	init_camera();
	init_position(position_x);
	init_position(position_y);

	Pid_t	pid_x, pid_y;
	Motor_t	motor_x, motor_y;
	Filter_t	filter_x, filter_y;

	init_PID(&pid_x, &pid_y);
	init_motor(&motor_x, &motor_y);
	init_filter(&filter_x, &filter_y);

	while (1) {
		
		if (pausa != 1) {
			ox = camera.x;
			oy = camera.y;
			ow = camera.width;
			oh = camera.height;

			get_image(image, camera.x-camera.width/2, camera.y-camera.height/2);
			threshold(image, camera.x-camera.width/2, camera.y-camera.height/2, 
				  &img_x, &img_y); 

			if (img_x == -1 && img_y == -1) {
				camera_enlarge();
				// pseudo-random direction generation
				da = frand(camera.a - RANGE, camera.a + RANGE);
				// update motor value with pseudo-random input
				// se a 2PI voglio 5 VOLT:  angle(2PI) = 5V * constant
				x_cord = update_motor(&motor_x, (cos(da) * PI) / VOLT);
				y_cord = update_motor(&motor_y, (sin(da) * PI) / VOLT);

				update_camera(x_cord, y_cord, img_x, img_y, dt); 
			} else {
				camera_restrict();
				// the ball id in (img_x, img_y), compute next position taking 
				// actual position into account
				pred_x = compute_centroid_x(img_x);
				pred_y = compute_centroid_y(img_y);
				// closed-loop
				// (PAN motor used to move camera on the x-axis)
				// (TILT motor used to move camera on the y-axis)
				controller_x = control_motor(&motor_x, &pid_x, &filter_x, 
											pred_x, camera.a, dt, PAN);

				controller_y = control_motor(&motor_y, &pid_y, &filter_y, 
											pred_y, camera.a, dt, TILT);

				update_camera(controller_x, controller_y, img_x, img_y, dt);

			}

			handle_bounce_camera();
			pthread_mutex_lock(&mxa);
			draw_camera(ox-ow/2, oy-oh/2, ow, oh, BGC);
			draw_camera(camera.x-camera.width/2, camera.y-camera.height/2, 
					camera.width, camera.height, camera.color);
			pthread_mutex_unlock(&mxa);
		}
		// check for deadline miss 
		if (ptask_deadline_miss(i)) {
			pthread_mutex_lock(&mxa);
			printf("Deadline miss camera!\n");
			pthread_mutex_unlock(&mxa);
		}
		ptask_wait_for_period(i);
	}
	release_tp(i);
	return 0;
}

	
//--------------------------------------------------------------
//				Function used by Ball Process
//--------------------------------------------------------------

void draw_ball(int x, int y, int c) {
	circlefill(screen, x, y, L, c);
}


void init_ball() {
	ball.color = GREEN;
	ball.radius = L;
	ball.x = XMAX/2;
	ball.y = YMAX/2;
	ball.v = VEL;
	ball.a = frand(-PI, PI);
}


// update ball position with respect to the mode selected by user
// 1: mouse mode
// 2: sine mode
// 0: random mode
void update_ball(int ox, int oy, float dt) {
	float	vx, vy;
	float	da, alpha;
	static int	range;
	static int	y;

	if (mode == 1) {
		pthread_mutex_lock(&mxb);
		ball.x = mouse_x;
		ball.y = mouse_y;
		ball.a = atan2((ball.y - oy), (ball.x - ox));
		pthread_mutex_unlock(&mxb);
	} else if (mode == 2) {
		pthread_mutex_lock(&mxb);

		if (ball.x == XMAX - L)
			range = 1;
		if (ball.x == XMIN + L)
			range = 0;

		if (range == 0)
			ball.x = ball.x + 1;
		else
			ball.x = ball.x - 1;

		// sine wave with x-axis in YMAX/2
		ball.y = YMAX/2  + (ball.v * sin(2*PI * 1/dt * ball.x));

		ball.a = atan2((ball.y - oy), (ball.x - ox));

		pthread_mutex_unlock(&mxb);
	} else {
		// pseudo-random direction generation
		da = frand(ball.a - RANGE, ball.a + RANGE);

		alpha = ((ball.a + da) > PI) ? (ball.a - da) : 
				((ball.a - da) < -PI) ? (ball.a + da) : da;

		pthread_mutex_lock(&mxb);

		ball.a = alpha;

		vx = ball.v * cos(ball.a);
		vy = ball.v * sin(ball.a);

		ball.x = ball.x + vx * dt;
		ball.y = ball.y + vy * dt;

		pthread_mutex_unlock(&mxb);
	}
}


void handle_bounce_ball() {
	int outl, outr, outt, outb;

	outl = (ball.x <= XMIN + L);
	outr = (ball.x >= XMAX - L);
	outt = (ball.y >= YMAX - L);
	outb = (ball.y <= YMIN + L);

	if (outl) {
		pthread_mutex_lock(&mxb);
		ball.x = XMIN + L;
		pthread_mutex_unlock(&mxb);
	}
	if (outr) {
		pthread_mutex_lock(&mxb);
		ball.x = XMAX - L;
		pthread_mutex_unlock(&mxb);
	}
	if (outl || outr) {
		pthread_mutex_lock(&mxb);
		ball.a = ball.a - PI;
		pthread_mutex_unlock(&mxb);
	}
	if (outt) {
		pthread_mutex_lock(&mxb);
		ball.y = YMAX - L;
		pthread_mutex_unlock(&mxb);
	}
	if (outb) {
		pthread_mutex_lock(&mxb);
		ball.y = YMIN + L;
		pthread_mutex_unlock(&mxb);
	}
	if (outt || outb) {
		pthread_mutex_lock(&mxb); 
		ball.a = - ball.a;
		pthread_mutex_unlock(&mxb);
	}
}


//--------------------------------------------------------------
//						BALL process
//--------------------------------------------------------------

void* taskball(void *arg) {
	int	i;				// indice del task 
	int	ox, oy;			// vecchia posizione pallina 
	float 	dt;
	
	//int ret; 							// Uncomment if you use SCHED_DEADLINE
	
	//struct sched_attr attr;			// Uncomment if you use SCHED_DEADLINE
	
	struct task_par *pdes = (struct task_par *)arg;
	i = pdes->index;
	/* 
	// Uncomment if you use SCHED_DEADLINE
	attr.size = sizeof(attr);
	attr.sched_flags = 0;
	attr.sched_nice = 0;
	attr.sched_priority = 0;
	// This creates a RUNT(ms)/PER(ms) reservation
	// PER and DREL have to be equal
	attr.sched_policy = SCHED_DEADLINE;
	attr.sched_runtime = RUNT * 1000 * 1000;
	attr.sched_period = PER * 1000 * 1000;
	attr.sched_deadline = DREL * 1000* 1000;

	ret = sched_setattr(getgid(), &attr, 0);
	if (ret < 0) {
		perror("sched_setattr");
		exit(-1);
	}
	*/

	ptask_set_period(i);
	dt = ptask_get_period(i, MILLI) / 100.;

	init_ball();

	while (1) {

		if ( pausa != 1) {
			ox = ball.x;
			oy = ball.y;

			update_ball(ox, oy, dt);
			handle_bounce_ball();

			// draw ball
			pthread_mutex_lock(&mxa);
			draw_ball(ox, oy, BGC);
			draw_ball(ball.x, ball.y, ball.color);
			pthread_mutex_unlock(&mxa);
		}
		// check for deadline miss 
		if (ptask_deadline_miss(i)) {
			pthread_mutex_lock(&mxa);
			printf("Deadline miss ball!\n");
			pthread_mutex_unlock(&mxa);
		}
		ptask_wait_for_period(i);
	}
	release_tp(i);
	return 0;
}


//--------------------------------------------------------------
//						ERROR process
//--------------------------------------------------------------

void* taskerror(void *arg) {

	int	i;
	int	oe, e;
	int	ox, x;
	int	mod_x, mod_y;
	float	dt;

	//int ret; 						// Uncomment if you use SCHED_DEADLINE
	
	//struct sched_attr attr;		// Uncomment if you use SCHED_DEADLINE

	struct task_par *pdes = (struct task_par *)arg;
	i = pdes->index;
	/* 
	// Uncomment if you use SCHED_DEADLINE
	attr.size = sizeof(attr);
	attr.sched_flags = 0;
	attr.sched_nice = 0;
	attr.sched_priority = 0;
	// This creates a RUNT(ms)/PER(ms) reservation
	// PER and DREL have to be equal
	attr.sched_policy = SCHED_DEADLINE;
	attr.sched_runtime = RUNT * 1000 * 1000;
	attr.sched_period = PER * 1000 * 1000;
	attr.sched_deadline = DREL * 1000* 1000;

	ret = sched_setattr(getgid(), &attr, 0);
	if (ret < 0) {
		perror("sched_setattr");
		exit(-1);
	}
	*/

	ptask_set_period(i);
	dt = ptask_get_period(i, MILLI) / 100.;

	e = 0;
	x = XMAX/2;

	while (1) {

		if (pausa != 1) {
			oe = e;
			// distance between the centre of camera and ball
			mod_x = pow(ball.x - camera.x, EXP); 
			mod_y = pow(ball.y - camera.y, EXP); 
			e = sqrt(mod_x + mod_y) / SCALE;			//scala 1 : 8

			if (x < XMAX){
				ox = x;
				x += 1;
			} else {
				ox = XMAX/2 - 1;
				x = XMAX/2;
			}

			// draw error
			if (x == XMIN) {
				pthread_mutex_lock(&mxa);
				rectfill(screen, ox, YMAX + OFFSET + FIVE, x, 
							YMAX + (SIX * OFFSET), BGC);
				pthread_mutex_unlock(&mxa);
			} else {
				pthread_mutex_lock(&mxa);
				rectfill(screen, x, YMAX + OFFSET + FIVE, x + 1, 
							YMAX + (SIX * OFFSET), BGC);
				pthread_mutex_unlock(&mxa);
			}
			pthread_mutex_lock(&mxa);
			line(screen, ox, YMAX + (SIX * OFFSET) - oe, x, 
							YMAX + (SIX * OFFSET) - e, PINK); 
			pthread_mutex_unlock(&mxa);
		}
		if (ptask_deadline_miss(i)) {
			pthread_mutex_lock(&mxa);
			printf("Deadline miss error!\n");
			pthread_mutex_unlock(&mxa);
		}
		ptask_wait_for_period(i);
	}
	release_tp(i);
	return 0;
}


//--------------------------------------------------------------
//						MAIN process
//--------------------------------------------------------------

int main(void) {
	int		i;								// number of tasks created 
	int		c, k, k1;						// character from keyboard 
	int	ntasks = 0;							// total number of created tasks 

	int vel = VEL;
	init();

	i = 0;

	do {
		c = 0;
		k = 0;
		k1 = 0;
		if (keypressed()) {
				c = readkey();
				k = (c >> 8);
				k1 = (c & 0xff); 	// ctrl + letter
		}

		if ((ntasks == 0) && (k == KEY_SPACE)) {
			// draw scenario 
			clear_to_color(screen, BGC);
			rect(screen, XMIN - 1, YMIN - 1, XMAX + 1, YMAX + 1, RED);

			rect(screen, XMAX/2 - 1, YMAX + OFFSET, XMAX + L + 1, 
					YMAX + (SIX * OFFSET) + FIVE, GREY);

			rect(screen, XMIN - 1, YMAX + OFFSET, XMAX/2 - TEN, 
					YMAX + (SIX * OFFSET) + FIVE, ORANGE);

			textprintf_ex(screen, font, XMAX/2, 
							OFFSET - FIVE, GREEN, -1, "VEL: %d", vel);

			textout_ex(screen, font, "Up    : Vel ++", XMIN, 
						YMIN - OFFSET - TEN, YELLOW, 0);
			textout_ex(screen, font, "Down  : Vel --", XMIN, 
						YMIN - OFFSET, YELLOW, 0);

			textout_ex(screen, font, "P : Pause", XMIN + FIVE, 
						YMAX + (TWO * OFFSET) - TEN, YELLOW, 0);
			textout_ex(screen, font, "A : Start", XMIN + FIVE, 
						YMAX + (TWO * OFFSET) + TEN, YELLOW, 0);

			textout_ex(screen, font, "M : Mouse", XMIN + FIVE, 
						YMAX + (THREE * OFFSET) + TEN, YELLOW, 0);
			textout_ex(screen, font, "C : Random", XMIN + FIVE, 
						YMAX + (FOUR * OFFSET) + TEN, YELLOW, 0);
			textout_ex(screen, font, "S : Sine", XMIN + FIVE, 
						YMAX + (FIVE * OFFSET) + TEN, YELLOW, 0);

			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + OFFSET + TEN, YELLOW, -1, "R: %d", R);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (TWO * OFFSET) + TEN, 
							YELLOW, -1, "K: %.3f", KT);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (THREE * OFFSET) + TEN, 
							YELLOW, -1, "J: %.4f", J);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (FOUR * OFFSET) + TEN, 
							YELLOW, -1, "b: %.4f", b);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (FIVE * OFFSET) + TEN, 
							YELLOW, -1, "TS: %.3f", TS);

			// create task camera
			tpars params_cam = TASK_SPEC_DFL;
			params_cam.period = tspec_from(PER, MILLI);
			params_cam.rdline = tspec_from(DREL, MILLI);

			// Uncomment if you use SCHED_DEADLINE
			//params_cam.runtime = tspec_from(RUNT, MILLI);

			// Comment if you use SCHED_DEADLINE
			params_cam.priority = PRIO - i;

			i = ptask_create(taskcamera, &params_cam);
			if (i != -1) {
				printf("Task %d created and activated\n", i);
				ntasks += 1;
			} else {
				allegro_exit();
				printf("Error in creating task!\n");
				exit(-1);
			}
			// create task ball
			tpars params_ball = TASK_SPEC_DFL;
			params_ball.period = tspec_from(PER, MILLI);
			params_ball.rdline = tspec_from(DREL, MILLI);

			// Uncomment if you use SCHED_DEADLINE
			//params_ball.runtime = tspec_from(RUNT, MILLI);
			
			// Comment if you use SCHED_DEADLINE
			params_ball.priority = PRIO - i;
			
			i = ptask_create(taskball, &params_ball);
			if (i != -1) {
				printf("Task %d created and activated\n", i);
				ntasks += 1;
			} else {
				allegro_exit();
				printf("Error in creating task!\n");
				exit(-1);
			}
			// create task error
			tpars params_err = TASK_SPEC_DFL;
			params_err.period = tspec_from(PER, MILLI);
			params_err.rdline = tspec_from(DREL, MILLI);
			
			// Uncomment if you use SCHED_DEADLINE
			//params_err.runtime = tspec_from(RUNT, MILLI);
			
			// Comment if you use SCHED_DEADLINE
			params_err.priority = PRIO - i;
			
			i = ptask_create(taskerror, &params_err);
			if (i != -1) {
				printf("Task %d created and activated\n", i);
				ntasks += 1;
			} else {
				allegro_exit();
				printf("Error in creating task!\n");
				exit(-1);
			}
		}
		// enter pause (task body is empty if pausa = 1)
		if (k == KEY_P)
			pausa = 1;
		// exit pause
		if (k == KEY_A)
			pausa = 0;
		// random motion
		if (k == KEY_C && mode != 0)
			mode = 0;
		// mouse motion 
		if (k == KEY_M && mode != 1)
			mode = 1;
		// sine motion
		if (k == KEY_S && mode != 2)
			mode = 2;
		// velocity increment
		if (k == KEY_UP && vel < MAX_VEL) {
			vel++;
			pthread_mutex_lock(&mxc);
			camera.v = vel;
			pthread_mutex_unlock(&mxc);
			pthread_mutex_lock(&mxb);
			ball.v = vel;
			pthread_mutex_unlock(&mxb);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMAX/2 - (TWO * OFFSET) - TEN, 0, 
						XMAX/2 + (FIVE * OFFSET), OFFSET + FIVE, BGC);
			textprintf_ex(screen, font, XMAX/2, 
							OFFSET - FIVE, GREEN, -1, "VEL: %d", vel);
			pthread_mutex_unlock(&mxa);
		}
		// velocity decrement
		if (k == KEY_DOWN && vel > 0) {
			vel--;
			pthread_mutex_lock(&mxc);
			camera.v = vel;
			pthread_mutex_unlock(&mxc);
			pthread_mutex_lock(&mxb);
			ball.v = vel;
			pthread_mutex_unlock(&mxb);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMAX/2 - (TWO * OFFSET) - TEN, 0, 
						XMAX/2 + (FIVE * OFFSET), OFFSET + FIVE, BGC);
			textprintf_ex(screen, font, XMAX/2, 
							OFFSET - FIVE, GREEN, -1, "VEL: %d", vel);
			pthread_mutex_unlock(&mxa);
		}
		
		// Change R value of motor
		if (k1 == CTRL_R) {
			R++;
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), YMAX + OFFSET + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (TWO * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + OFFSET + TEN, YELLOW, -1, "R: %d", R);
			pthread_mutex_unlock(&mxa);
		}
		if (c == (KEY_R << 8)) {
			R--;
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), YMAX + OFFSET + TEN, 
						XMAX/2 - (TWO * OFFSET) - TEN, 
						YMAX + (TWO * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + OFFSET + TEN, YELLOW, -1, "R: %d", R);
			pthread_mutex_unlock(&mxa);
		}
		
		// Change KT AND KB values of motor
		if (k1 == CTRL_K) {
			KT += nextafterf(HUNTH, 0);
			KB += nextafterf(HUNTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
						YMAX + (TWO * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (THREE * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (TWO * OFFSET) + TEN, 
							YELLOW, -1, "K: %.3f", KT);
			pthread_mutex_unlock(&mxa);
		}
		if (c == (KEY_K << 8)) {
			KT -= nexttowardf(HUNTH, 0);
			KB -= nexttowardf(HUNTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
				YMAX + (TWO * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (THREE * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (TWO * OFFSET) + TEN, 
							YELLOW, -1, "K: %.3f", KT);
			pthread_mutex_unlock(&mxa);	
		}
		
		// Change J value of motor
		if (k1 == CTRL_J) {
			J += nextafterf(THOTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
				YMAX + (THREE * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (FOUR * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (THREE * OFFSET) + TEN, 
							YELLOW, -1, "J: %.4f", J);
			pthread_mutex_unlock(&mxa);
		}
		if (c == (KEY_J << 8)) {
			J -= nexttowardf(THOTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
						YMAX + (THREE * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (FOUR * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (THREE * OFFSET) + TEN, 
							YELLOW, -1, "J: %.4f", J);
			pthread_mutex_unlock(&mxa);	
		}
		
		// Change b value of motor
		if (k1 == CTRL_B) {
			b += nextafterf(THOTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
						YMAX + (FOUR * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (FIVE * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (FOUR * OFFSET) + TEN, 
							YELLOW, -1, "b: %.4f", b);
			pthread_mutex_unlock(&mxa);
		}
		if (c == (KEY_B << 8)) {
			b -= nexttowardf(THOTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
						YMAX + (FOUR * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (FIVE * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (FOUR * OFFSET) + TEN, 
							YELLOW, -1, "b: %.4f", b);
			pthread_mutex_unlock(&mxa);	
		}
		
		// change TS value of motor
		if (k1 == CTRL_T) {
			TS += nextafterf(HUNTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
						YMAX + (FIVE * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (SIX * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (FIVE * OFFSET) + TEN, 
							YELLOW, -1, "TS: %.3f", TS);
			pthread_mutex_unlock(&mxa);
		}
		if (c == (KEY_T << 8)) {
			TS -= nexttowardf(HUNTH, 0);
			pthread_mutex_lock(&mxa);
			rectfill(screen, XMIN + (FIVE * OFFSET), 
						YMAX + (FIVE * OFFSET) + TEN, 
						XMAX/2 - (TWO * OFFSET) + TEN, 
						YMAX + (SIX * OFFSET), BGC);
			textprintf_ex(screen, font, XMIN + (EIGHT * OFFSET) - TEN, 
							YMAX + (FIVE * OFFSET) + TEN, 
							YELLOW, -1, "TS: %.3f", TS);
			pthread_mutex_unlock(&mxa);	
		}
		
	} while (k != KEY_ESC);

	printf("End\n");
	allegro_exit();
	return 0;
}

//-----------------------------------------------------------------------------
