#ifndef TURN_OMEGA_HDR
#define TURN_OMEGA_HDR

#include "functions.h"
/* The above to get 'pointtype' data structure ... */

/* Prototype for function to perform omega turn. */
int TURN_omega(const pointtype dmp_i,
	       const pointtype dmp_ip1,
	       const pointtype dmp_ip2);
/* The omega turn is performed based on these two points.
 * If some error is encountered, then it returns non-zero,
 * otherwise zero. */

/* ??Change prototype to use pointers instead? */

#endif /* TURN_OMEGA_HDR */
