#include <math.h>
#include <stdlib.h>

#include "turn_gps_link_lanes.h"

#include "turn_support_lib.h"
#include "stop_tractor.h"
#include "vector_lib.h"

#include "functions.h"
/* In order to get the macro MAXTURNRAD, used as default turning radius. */

void TURN_gps_link_lanes(const pointtype dmp_i,
			 const pointtype dmp_ip1,
			 const pointtype dmp_ip2,
			 const double turn_radius,
			 const double speed,
			 const double acc)
{
  const Vector pos_entry = VL_set_xy(dmp_i.e, dmp_i.n);
  const double alfa_entry = dmp_i.ang / 180.0 * M_PI;
  const Vector pos_exit = VL_set_xy(dmp_ip1.e, dmp_ip1.n);
  const double alfa_exit = dmp_ip2.ang / 180.0 * M_PI;
  const double adj_turn_radius = (turn_radius > 0.05) ? turn_radius : MAXTURNRAD;
 
  const TURN_SL_link_test_type s = TURN_SL_link_test(pos_entry,
						     alfa_entry,
						     pos_exit,
						     alfa_exit,
						     adj_turn_radius);

  if (s.valid) {
    /* A valid path was found - now follow it! */
    const double beta_entry_degrees = s.beta_entry * 180.0 / M_PI;
    const double beta_exit_degrees = s.beta_exit * 180.0 / M_PI;
    const double beta_tangent_degrees = s.beta_tangent * 180.0 / M_PI;

    char str[2000] = {0};

    /* Do arc (entry) */
    cl_send("set \"usekalmanodo\" 0\n");
    sprintf (str,
	     "turnr %f %f @v%f @a%f\n",
	     adj_turn_radius,
	     beta_entry_degrees,
	     speed,
	     acc);
    printf("GPS_LINK_LANES: Sending : %s \n", str);
    cl_send (str);
    /* Consider whether to use "shorter" arc to pass control to
     * the GPS assisted lane tracker sooner. */
    
    /* Do straight line segment (with GPS feedback). */
    cl_send("set \"usekalmanodo\" 1\n");
    /* It is expected that the two following commands describe the same
     * line, but with different stop-conditions.
     * Used in order to have synchronization (should probably be done using return codes ...). */
    sprintf(str,
	    "drive %f %f %f @v%f @a%f : ($targetdist < %f)\n",
	    VL_get_x(s.pos_t_entry),
	    VL_get_y(s.pos_t_entry),
	    beta_tangent_degrees,
	    speed,
	    acc,
	    0.0);
    printf("GPS_LINK_LANES: Sending : %s \n", str);
    cl_cmdwait(str); /* !!! */
    sprintf(str,
	    "drive %f %f %f @v%f @a%f : ($targetdist < %f)\n",
	    VL_get_x(s.pos_t_exit),
	    VL_get_y(s.pos_t_exit),
	    beta_tangent_degrees,
	    speed,
	    acc,
	    0.0);
    printf("GPS_LINK_LANES: Sending : %s \n", str);
    cl_send(str);
    
    /* Do arc (exit) */
    cl_send("set \"usekalmanodo\" 0\n");
    sprintf (str,
	     "turnr %f %f @v%f @a%f\n",
	     adj_turn_radius,
	     beta_exit_degrees,
	     speed,
	     acc);
    printf("GPS_LINK_LANES: Sending : %s \n", str);
    cl_send (str);
    /* Consider whether to use "shorter" arc to pass control to
     * the next row (lane) tracker sooner. */

    /* ??Do ID-based synchronization? */
  } else {
    /* No valid path found - stop! */
    stop_tractor();
  }
}
