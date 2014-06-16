#include <math.h>
/* To get 'M_PI' ?? */

#include "drivetowards.h"

#include "vector_lib.h"

#include "functions.h"
/* In order to get 'cl_cmdwait' prototype. */

void drivetowards(const Vector position,
		  const double angle_rad,
		  const double targetdist,
		  const double speed,
		  const double acceleration)
{
  char str[2000] = {0};

  /* Shouldn't this include command to enable Kalman odometry? */

  sprintf(str,
	  "drive %f %f %f @v%f @a%f : ($targetdist < %f)\n",
	  VL_get_x(position),
	  VL_get_y(position),
	  angle_rad * 180.0 / M_PI,
	  speed,
	  acceleration,
	  targetdist);
  cl_cmdwait(str); /* This will cause it to wait for completion.
		    * One way to make sure the low level controller has a new command
		    * when the tractor is physically located at the endpoint would be to
		    * specify an appropriate 'targetdist'.
		    * However, the timing should have an overhaul. */
}

