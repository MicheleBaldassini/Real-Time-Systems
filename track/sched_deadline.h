//-----------------------------------------------------------------------------
// SCHED DEADLINE header file implementing EDF + CBS scheduling algorithm
//-----------------------------------------------------------------------------

#ifndef SCHED_DEADLINE_H
#define SCHED_DEADLINE_H

#include <linux/types.h>
#include <sys/syscall.h>
#include <pthread.h>

#define gettid() syscall(__NR_gettid)


#define SCHED_DEADLINE	6

struct sched_attr {
	__u32 size;

	__u32 sched_policy;
	__u64 sched_flags;

	// SCHED_NORMAL, SCHED_BATCH 
	__s32 sched_nice;

	// SCHED_FIFO, SCHED_RR 
	__u32 sched_priority;

	// SCHED_DEADLINE (nsec)
	__u64 sched_runtime;
	__u64 sched_deadline;
	__u64 sched_period;
};

int sched_setattr(pid_t pid, const struct sched_attr *attr,
				  unsigned int flags) {
	return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid, struct sched_attr *attr, 
				  unsigned int size, unsigned int flags) {
	return syscall(__NR_sched_getattr, pid, attr, size, flags);
}


#endif

//-----------------------------------------------------------------------------