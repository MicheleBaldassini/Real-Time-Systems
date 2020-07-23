//-----------------------------------------------------------------------------
// PTASK
// This file contains the implementations of functions used to create and
// manage periodic tasks
//-----------------------------------------------------------------------------


#define _GNU_SOURCE

#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>

#include "ptask.h"

const tpars TASK_SPEC_DFL = {
	.period = {1, 0},  
	.rdline = {1, 0},
	.priority = 1, 					// Comment if you use SCHED_DEADLINE
	//.runtime = {1, 0},			// Uncomment if you use SCHED_DEADLINE
	.arg = NULL
};



int	ptask_policy;					// common scheduling policy   
sem_protocol	ptask_protocol;		// semaphore protocol         
pthread_t	tid[MAX_TASKS];			// thread id
struct task_par	tp[MAX_TASKS];		// descriptor of periodic task
static int	first_free;				// first descriptor free

// this is used to protect the tp data structure 
// from concurrent accesses from the main and the threads
static pthread_mutex_t	tp_mutex;


//mutex
// create PRIO_INHERITANCE mutex
int pmux_create_pi(pthread_mutex_t *m) {
	int	ret;
	pthread_mutexattr_t	mta;
	pthread_mutexattr_init(&mta);
	pthread_mutexattr_setprotocol(&mta, PTHREAD_PRIO_INHERIT);

	ret = pthread_mutex_init(m, &mta);

	pthread_mutexattr_destroy(&mta);
	return ret;
}

//create PRIO_CEILING mutex
int pmux_create_pc(pthread_mutex_t *m, int ceiling) {
	int	ret;
	pthread_mutexattr_t	mta;

	pthread_mutexattr_init(&mta);
	pthread_mutexattr_setprotocol(&mta, PTHREAD_PRIO_PROTECT);
	pthread_mutexattr_setprioceiling(&mta, ceiling);

	ret = pthread_mutex_init(m, &mta);

	pthread_mutexattr_destroy(&mta);
	return ret;
}


int pmux_destroy(pthread_mutex_t *m) {
	return pthread_mutex_destroy(m);
}


// set deadline and period of task with index passing as argument
void ptask_set_period(int i) {
	tspec	t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	// compute the absolute deadline 
	tp[i].dl = tspec_add(&t, &tp[i].deadline);
	// compute the next activation time 
	tp[i].at = tspec_add(&t, &tp[i].period);
}


// allocate a descriptor for the task 
// (initialize the mutex for guarantees mutual exclusion for descriptor)
int allocate_tp(){

	int	x = first_free;
	pthread_mutex_lock(&tp_mutex);
	first_free = tp[x].free;

	if (ptask_protocol == PRIO_INHERITANCE) 
		pmux_create_pi(&tp[x].mux);
	else if (ptask_protocol == PRIO_CEILING) 
		pmux_create_pc(&tp[x].mux, MAX_PRIO);
	else 
		pthread_mutex_init(&tp[x].mux, 0);

	pthread_mutex_unlock(&tp_mutex);
	return x;
}


//free a descriptor (is called when task ends)
void release_tp(int i) {
	pthread_mutex_lock(&tp_mutex);
	tp[i].free = first_free;
	pmux_destroy(&tp[i].mux);
	first_free = i;
	pthread_mutex_unlock(&tp_mutex);
}


// initialize ptask
// Uncomment if you use SCHED_DEADLINE
//void ptask_init(sem_protocol protocol) {
// Comment if you use SCHED_DEADLINE
void ptask_init(int policy, sem_protocol protocol) {

	int	i;
	ptask_policy = policy;				// Comment if you use SCHED_DEADLINE
	ptask_protocol = protocol;

	for (i=0; i<MAX_TASKS; i++)
		tp[i].free = i+1;

	first_free = 0;
	if (ptask_protocol == PRIO_INHERITANCE) 
		pmux_create_pi(&tp_mutex);
	else if (ptask_protocol == PRIO_CEILING) 
		pmux_create_pc(&tp_mutex, MAX_PRIO);
	else if (ptask_protocol == NO_PROTOCOL) 
		pthread_mutex_init(&tp_mutex, 0);
	// initialize time
	tspec_init();
}


int ptask_create(void* (*task)(void*), tpars *tp_arg) {
	int	tret;
	tspec	t;
	pthread_attr_t	myatt;				// Comment if you use SCHED_DEADLINE
	struct sched_param	mypar;			// Comment if you use SCHED_DEADLINE

	int i = allocate_tp();

	tp[i].index = i;
	tp[i].dmiss = 0;
	tp[i].state = TASK_ACTIVE; 

	if (tp == NULL) {
		tp[i].period = tspec_from(1, SEC);
		tp[i].deadline = tspec_from(1, SEC);
		// Comment if you use SCHED_DEADLINE
		tp[i].priority = 1;
		// Uncomment if you use SCHED_DEADLINE
		//tp[i].runtime = tspec_from(1, SEC);
		tp[i].arg = 0;
	} else {
		tp[i].period = tp_arg->period;
		tp[i].deadline = tp_arg->rdline;
		// Comment if you use SCHED_DEADLINE
		tp[i].priority = tp_arg->priority;
		// Uncomment if you use SCHED_DEADLINE
		//tp[i].runtime = tp_arg->runtime;
		tp[i].arg = tp->arg;
	}

	// Comment if you use SCHED_DEADLINE
	//-------------------------------------------------------------------------
	pthread_attr_init(&myatt);
	if (ptask_policy != SCHED_OTHER)
		pthread_attr_setinheritsched(&myatt, PTHREAD_EXPLICIT_SCHED);

	pthread_attr_setschedpolicy(&myatt, ptask_policy);
	mypar.sched_priority = tp[i].priority;
	pthread_attr_setschedparam(&myatt, &mypar);

	tret = pthread_create(&tid[i], &myatt, task, (void*)(&tp[i]));

	pthread_attr_destroy(&myatt);
	// ------------------------------------------------------------------------

	// Uncomment if you use SCHED_DEADLINE
	//tret = pthread_create(&tid[i], NULL, task, (void*)(&tp[i]));	

	if (tret == 0) {
		return i;
	} else {
		release_tp(i);
		return -1;
	}
}


int ptask_get_period(int i, int unit) {
	int	p;
	pthread_mutex_lock(&tp[i].mux);
	p = tspec_to(&tp[i].period, unit); 
	pthread_mutex_unlock(&tp[i].mux);
	return p;
}


void ptask_wait_for_period(int i) {
	pthread_mutex_lock(&tp[i].mux);

	tp[i].state = TASK_WFP;
	pthread_mutex_unlock(&tp[i].mux);
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp[i].at), NULL);
	pthread_mutex_lock(&tp[i].mux);
	tp[i].state = TASK_ACTIVE;
	// update absolute deadline 
	tp[i].dl = tspec_add(&(tp[i].at), &tp[i].deadline);
	// when awaken, update next activation time 
	tp[i].at = tspec_add(&(tp[i].at), &tp[i].period);

	pthread_mutex_unlock(&tp[i].mux);

	return;
}


int ptask_deadline_miss(int i) {
	struct timespec	now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	if (tspec_cmp(&now, &tp[i].dl) > 0) 
		return 1;
	else 
		return 0;
}

//-----------------------------------------------------------------------------