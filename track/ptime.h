//-----------------------------------------------------------------------------
// PTIME header file for time management
//-----------------------------------------------------------------------------

#ifndef PTIME_H
#define PTIME_H

#include <time.h>

// Shortcut for struct timespec 
// (represents absolute time, as returned by the clock_gettime())
typedef struct timespec tspec;

// This represents a time interval(from SEC to NANO)
typedef long ptime;

// time units
#define	SEC	0
#define	MILLI	1
#define	MICRO	2
#define	NANO	3


// Initializes this library   
void tspec_init();

// Converts to a long expressed in unit
ptime tspec_to(const tspec *t, int unit);

// From a long integer, expressed as unit, into a timespec
tspec tspec_from(ptime tu, int unit);

// Compute s = a + b
tspec tspec_add(const tspec *a, const tspec *b);

// Compares two timespecs
int tspec_cmp(const tspec *a, const tspec *b);


#endif 

//-----------------------------------------------------------------------------