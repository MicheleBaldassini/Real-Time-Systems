//-----------------------------------------------------------------------------
// PTASK LIBRARY - HEADER FILE 
//-----------------------------------------------------------------------------

#ifndef PTASK_H
#define PTASK_H

#include <pthread.h>
#include <semaphore.h>
#include "ptime.h"

#define	MAX_TASKS	3

#define	MAX_PRIO	80

// semaphore protocol
typedef enum {PRIO_INHERITANCE, PRIO_CEILING, NO_PROTOCOL}	sem_protocol;

// task possible state
typedef enum {TASK_ACTIVE, TASK_WFP}	ptask_state;

typedef void ptask;

// This structure is used to simplify 
// the creation of a task by setting standard arguments 
typedef struct {
	tspec	period;
	tspec	rdline;		// relative deadline
	//tspec	runtime; 	// Uncomment if you use SCHED_DEADLINE
	int	priority;		// from 0 to 99 ---> Comment if you use SCHED_DEADLINE
	void*	arg;		// pointer to a task argument
} tpars;


struct task_par {
	void*	arg;				// task argument
	int	index;					// task index
	tspec	period;				// task period
	tspec	deadline;			// relative deadline
	//tspec	runtime;			// Uncomment if you use SCHED_DEADLINE
	int	priority;				// task priority in [0,99] ---> Comment if you use SCHED_DEADLINE
	int	dmiss;					// number of deadline misses
	tspec	at;					// next activation time
	tspec	dl;					// current absolute deadline
	int	free;					// >=0 if this descr is avail.
	ptask_state	state;			// ACTIVE, SUSPENDED, WFP
	pthread_mutex_t	mux;		// mutex for this data struct
};

extern const tpars TASK_SPEC_DFL;

// create mutex with priority inheritance protocol
int pmux_create_pi(pthread_mutex_t *m);

// create mutex with priority inheritance ceiling protocol
int pmux_create_pc(pthread_mutex_t *m, int ceiling);

int pmux_destroy(pthread_mutex_t *m);


// GLOBAL FUNCTIONS/

//-----------------------------------------------------------------------------
// Frees a descriptor. It inserts the free descriptor at the head of 
// the queue. It uses the tp_mutex to protect the critical section
//-----------------------------------------------------------------------------
void release_tp(int);


//-----------------------------------------------------------------------------
// This function returns a free descriptor, or -1 if there are no more
// free descriptors. tp is organised as a linked list, first_free is
// the head of the list. This extracts from the head. It uses the
// tp_mutex to protect the critical section
//-----------------------------------------------------------------------------
int allocate_tp();


// initialize some PTASK variables
void ptask_init(int, sem_protocol);		//Comment if you use SCHED_DEADLINE
//void ptask_init(sem_protocol);		//Uncomment if you use SCHED_DEADLINE


//----------------------------------------------------------------------------- 
// Creates a task with a set of parameters. If tp is NULL, by default 
// the period and the relative deadline will be equal to 1 second and the 
// priority is 1 (lowest). 
//-----------------------------------------------------------------------------
int ptask_create(void* (*task)(void*), tpars*);


// set period
void ptask_set_period(int);


// waits for next period or activation
void ptask_wait_for_period(int); 


//-----------------------------------------------------------------------------
// DEADLINE_MISS: if a deadline is missed increments dmiss
// and returns 1, otherwise returns 0
//-----------------------------------------------------------------------------
int ptask_deadline_miss(int);


// return current period 
int ptask_get_period(int, int);


#endif

//-----------------------------------------------------------------------------
