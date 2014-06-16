#ifndef TURN_GPS_FISHTAIL_HDR
#define TURN_GPS_FISHTAIL_HDR

#include "functions.h"
/* In order to get 'pointtype' - the interface should be changed to use 'Vector'
 * and angles in radians instead. */

void TURN_gps_fishtail(const pointtype dmp_i,
		       const pointtype dmp_ip1,
		       const pointtype dmp_ip2,
		       const double turn_radius,
		       const double speed,
		       const double acc,
		       const double alt_speed);
/* Will perform a GPS assisted fishtail turn according to the
 *   1. Easting, Northing and angle of 'dmp_i'.
 *   2. Easting and Northing of 'dmp_ip1'.
 *   3. angle of 'dmp_ip2'.
 *   4. 'turn_radius' (if not specified (i.e. zero) default value will be used).
 *   5. 'speed' (speed when moving forwards).
 *   6. 'acc' (passed on to the low level controller. ??Interpretation?).
 *   7. 'alt_speed' (speed when reversing, if not specified (i.e. zero) then
 *         default value is used).
 */

/* If unable to handle the parameters, then it will call 'stop_tractor'. */

/* If there is enough room between the rows (lanes), then the turning might
 * be done similarly to 'TURN_gps_link_lanes' (i.e. no reversing). */

/* Currently the plan is to do the turnings using the SMR-CL "turnr" command,
 * and the straight part of the fishtail using GPS,
 * but other options might be used (for instance GPS assisted turning). */

#endif /* TURN_GPS_FISHTAIL_HDR */
