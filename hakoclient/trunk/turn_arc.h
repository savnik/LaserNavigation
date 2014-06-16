#ifndef TURN_ARC_HDR
#define TURN_ARC_HDR

/* Prototype for function making "arc" turns ... (i.e. circling). */

#include "functions.h"
/* In order to get 'pointtype' type - this should be moved to some kind of "*_types.h" file. */

int TURN_arc(const pointtype dmp_i,
	     const double turn_angle,
	     const double turn_radius,
	     const pointtype dmp_ip1,
	     const pointtype dmp_ip2);
/* Should be called when 'dmp_i' is reached.
 * This function will construct a SMR-CL string based on 'turn_angle' and 'turn_radius'.
 * It ends by doing a drive according to 'dmp_ip1' and 'dmp_ip2', but no checking is made
 * wrt. the consistency of the parameters.
 * If zero is specified for 'turn_angle' or 'turn_radius', then default values are chosen.
 */

#endif /* TURN_ARC_HDR */
