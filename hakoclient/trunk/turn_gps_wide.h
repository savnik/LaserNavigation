#ifndef TURN_GPS_WIDE_HDR
#define TURN_GPS_WIDE_HDR

/* Prototype for function making "gps_wide" turns (arcs approximated by line segments). */

#include "functions.h"
/* In order to get 'pointtype' type - this should be moved to some kind of "*_types.h" file. */

int TURN_gps_wide(const pointtype dmp_i,
		  const double turn_angle,
		  const double turn_radius,
		  const double speed,
		  const double targetdist,
		  const double arc_step_length,
		  const pointtype dmp_ip1);
/* Should be called when 'dmp_i' is reached.
 * Will generate an appropriate number of line-follow commands, ended with a "driveto" towards 'dmp_ip1'.
 * The arc to approximate is specified by 'turn_angle' and 'turn_radius'.
 * The approximation is done by straight line segments.
 * The parameters 'targetdist' and 'arc_step_length' determines how long the line segments will be
 * and when to provide a new reference to the low level controller.
 * Note that a high number of SMR-CL commands might be generated - the current buffer size is 400 messages (??)
 *  - this function should be implemented using a 'wait_m5' call to limit the number of messages present
 * in the queue, but not let the queue get empty ...
 * It is probably not implemented like this, but will generate all messages right away ...
 *
 * If zero is specified for 'turn_angle', 'turn_radius' or 'arc_step_length' then default values are chosen.
 */

#endif /* TURN_GPS_WIDE_HDR */
