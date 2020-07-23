//-----------------------------------------------------------------------------
// PTIME
// In this file are implemented the function for the correct use of time
// To convert timespec into long and viceversa, to sum two timespec 
// and to compare them
//-----------------------------------------------------------------------------

#include "ptime.h"

struct unit_conv {
	long	mul;
	long	div;
};

const struct unit_conv conv_table[] = {
	{1, 1000000000},	// SEC
	{1000, 1000000},	// MILLI
	{1000000, 1000},	// MICRO
	{1000000000, 1}		// NANO
};

static tspec	tspec_t0;

// init time
void tspec_init() {
	clock_gettime(CLOCK_MONOTONIC, &tspec_t0);
}

// Given a timespec, converts to a long according to unit.
long tspec_to(const tspec *t, int unit) {
	long	tu;
	tu = (t->tv_sec) * conv_table[unit].mul;
	tu += (t->tv_nsec) / conv_table[unit].div;

	return tu;
}

//Given a long integer, expressed as unit, converts it into a timespec
tspec tspec_from(long tu, int unit) {
	tspec	t;
	long	mm; 

	mm = tu % conv_table[unit].mul;

	t.tv_sec = tu / conv_table[unit].mul;
	t.tv_nsec = mm * conv_table[unit].div;

	return t;
}


// Given two timespec, return a timespec which is the sum 
tspec tspec_add(const tspec *a, const tspec *b) {
	tspec	s;
	s.tv_nsec = a->tv_nsec + b->tv_nsec;
	s.tv_sec = a->tv_sec + b->tv_sec;
	while (s.tv_nsec >= 1000000000) {
		s.tv_nsec = s.tv_nsec - 1000000000;
		s.tv_sec += 1;
	}
	return s;
}

// Compares two timespecs
int tspec_cmp(const tspec *a, const tspec *b) {
	if (a->tv_sec > b->tv_sec) 
		return 1;
	else if (a->tv_sec < b->tv_sec)
		return -1;
	else if (a->tv_sec == b->tv_sec) {
		if (a->tv_nsec > b->tv_nsec)
			return 1;
		else if (a->tv_nsec == b->tv_nsec)
			return 0;
		else return -1;
	}
	return 0;
}

//-----------------------------------------------------------------------------