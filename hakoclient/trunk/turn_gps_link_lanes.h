#ifndef TURN_GPS_LINK_LANES_HDR
#define TURN_GPS_LINK_LANES_HDR

#include "functions.h"
/* In order to get 'pointtype' - interface should be changed to use 'Vector' and
 * angles in radians. */

void TURN_gps_link_lanes(const pointtype dmp_i,
			 const pointtype dmp_ip1,
			 const pointtype dmp_ip2,
			 const double turn_radius,
			 const double speed,
			 const double acc);
/* Will perform GPS assisted linking of lanes (rows) according to the
 *   1. Easting, Northing and angle of 'dmp_i'.
 *   2. Easting and Northing of 'dmp_ip1'.
 *   3. angle of 'dmp_ip2'.
 *   4. 'turn_radius' (if not specified (i.e. zero) default value will be used).
 *   5. 'speed' (speed when moving forwards).
 *   6. 'acc' (passed on to the low level controller. ??Interpretation?).
 */

/* If unable to handle the parameters (for instance if lanes are located
 * too close to each other), then it will call 'stop_tractor'. */

#endif /* TURN_GPS_LINK_LANES_HDR */
