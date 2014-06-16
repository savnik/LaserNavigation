#ifndef DRIVETOWARDS_HDR
#define DRIVETOWARDS_HDR

#include "vector_type.h"

void drivetowards(const Vector position,
		  const double angle_rad,
		  const double targetdist,
		  const double speed,
		  const double acceleration);
/* Similar to the 'driveto' function, but uses 'Vector' type and supports 'targetdist'. */

#endif /* DRIVETOWARDS_HDR */
