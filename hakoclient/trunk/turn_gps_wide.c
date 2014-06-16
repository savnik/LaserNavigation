#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "turn_gps_wide.h"

#include "vector_type.h"
#include "vector_lib.h"

static void core_turn_gps_wide(const Vector s_entry /* The entry point of the "arc" */,
			       const double ang_rad /* The entry angle (expected to be angle from previous point). */,
			       const double turn_angle_rad,
			       const double turn_radius,
			       const double speed,
			       const double targetdist,
			       const double arc_step_length,
			       const Vector s_exit /* The point to aim for after arc. */);
/* This function will issue the SMR-CL commands (without checking parameters - it expects them to be checked by the
 * "wrapper" function. */
 
int TURN_gps_wide(const pointtype dmp_i,
		  const double turn_angle,
		  const double turn_radius,
		  const double speed,
		  const double targetdist,
		  const double arc_step_length,
		  const pointtype dmp_ip1)
{
  printf("TURN_gps_wide called with (%f, %f, ang=%f), angle=%f, radius=%f, speed=%f, targetdist=%f, arc_step_length=%f, (%f, %f).\n",
	 dmp_i.e,
	 dmp_i.n,
	 dmp_i.ang,
	 turn_angle,
	 turn_radius,
	 speed,
	 targetdist,
	 arc_step_length,
	 dmp_ip1.e,
	 dmp_ip1.n);
  {
    const Vector s_entry = VL_set_xy(dmp_i.e, dmp_i.n);
    const double ang_rad = dmp_i.ang / 180.0 * M_PI;
    const Vector s_exit = VL_set_xy(dmp_ip1.e, dmp_ip1.n);
    double adj_turn_angle_rad = 0.0;
    double adj_turn_radius = 0.0;
    double adj_arc_step_length = 0.0;
    double adj_speed = 0.0;

    if (turn_angle) {
      adj_turn_angle_rad = turn_angle / 180.0 * M_PI;
    } else {
      adj_turn_angle_rad = M_PI / 2.0; /* Equivalent to 90 degrees. */
    }

    if (turn_radius > 0) {
      adj_turn_radius = turn_radius;
    } else {
      adj_turn_radius = 3.0; /* Choice of default value. */
    }

    if (arc_step_length > 0.05) {
      adj_arc_step_length = arc_step_length;
    } else {
      adj_arc_step_length = 1.0;
    }

    if (speed) {
      adj_speed = speed;
    } else {
      adj_speed = DFLTSPEED;
    }

    core_turn_gps_wide(s_entry,
		       ang_rad,
		       adj_turn_angle_rad,
		       adj_turn_radius,
		       adj_speed,
		       targetdist,
		       adj_arc_step_length,
		       s_exit);
  }

  return 0;
}

static Vector get_center(const Vector s_entry,
			 const double ang_rad,
			 const double turn_angle_rad,
			 const double turn_radius);
/* Returns the center of the turn. */

static Vector get_center(const Vector s_entry,
			 const double ang_rad,
			 const double turn_angle_rad,
			 const double turn_radius)
{
  double radial_angle_rad = 0.0;
  Vector rv = {0.0};
  
  if (turn_angle_rad > 0) {
    /* Turn angle positive (turn to the left). */
    radial_angle_rad = ang_rad - M_PI / 2.0;
  } else {
    radial_angle_rad = ang_rad + M_PI / 2.0;
  }
  
  {
    const Vector radial_vector =
      VL_set_xy(turn_radius * cos(radial_angle_rad),
		turn_radius * sin(radial_angle_rad));
    
    rv = VL_subtract(s_entry, radial_vector);
  }

  return rv;					     
}

/* Candidate for replacement of 'driveto' */
static void local_drivetowards(const Vector position,
			       const double angle_rad,
			       const double targetdist,
			       const double speed,
			       const double acceleration)
{
  char str[2000] = {0};

  sprintf(str,
	  "drive %f %f %f @v%f @a%f : ($targetdist < %f)\n",
	  VL_get_x(position),
	  VL_get_y(position),
	  angle_rad * 180.0 / M_PI,
	  speed,
	  acceleration,
	  targetdist);
  cl_send(str); /* This should be changed to some function which makes sure the queue is
		 * neither empty nor too full.
		 * Pause will probably not work with this implementation. */
}

static void core_turn_gps_wide(const Vector s_entry /* The entry point of the "arc" */,
			       const double ang_rad /* The entry angle (expected to be angle from previous point). */,
			       const double turn_angle_rad,
			       const double turn_radius,
			       const double speed,
			       const double targetdist,
			       const double arc_step_length,
			       const Vector s_exit /* The point to aim for after arc. */)
{
  const Vector s_center = get_center(s_entry, ang_rad, turn_angle_rad, turn_radius);
  const double end_angle_rad = ang_rad + turn_angle_rad;
  const double arc_step_rad = fabs(arc_step_length / turn_radius);
  double current_angle_rad = ang_rad;
  Vector s_current = s_center; /* Not correct, but "benign". */

  if (turn_angle_rad > 0) {
    /* Turn angle positive, a turn to the left. */
    while (current_angle_rad + (1.5 * arc_step_rad) < end_angle_rad) {
      /* Still not done with arc, do another iteration (this could easily end up as an infinite loop!). */
      const double new_angle_rad = current_angle_rad + arc_step_rad;
      const double new_radial_angle_rad = new_angle_rad - M_PI / 2.0;
      const Vector new_radial_vector =
	VL_set_xy(turn_radius * cos(new_radial_angle_rad),
		  turn_radius * sin(new_radial_angle_rad));
      const Vector s_new = VL_add(s_center, new_radial_vector);
      const double line_angle_rad = (new_angle_rad + current_angle_rad) / 2.0;

      /* Send SMR-CL command ... this should be wrapped in queue handler somehow ... */
      local_drivetowards(s_new, line_angle_rad, targetdist, speed, TURNACC);

      /* Prepare for next iteration ... */
      current_angle_rad = new_angle_rad;

      /* Prepare for the event that this was the last iteration ... */
      s_current = s_new;
    }
  } else {
    /* Turn angle not positive, taken to be negative and thus a turn to the right. */
    while (current_angle_rad - (1.5 * arc_step_rad) > end_angle_rad) {
      /* Still not done with arc, do another iteration. */
      const double new_angle_rad = current_angle_rad - arc_step_rad;
      const double new_radial_angle_rad = new_angle_rad + M_PI / 2.0;
      const Vector new_radial_vector =
	VL_set_xy(turn_radius * cos(new_radial_angle_rad),
		  turn_radius * sin(new_radial_angle_rad));
      const Vector s_new = VL_add(s_center, new_radial_vector);
      const double line_angle_rad = (new_angle_rad + current_angle_rad) / 2.0; /* Average of new and old angles. */

      /* Send SMR-CL command ... */
      local_drivetowards(s_new, line_angle_rad, targetdist, speed, TURNACC);

      /* Prepare for next iteration ... */
      current_angle_rad = new_angle_rad;

      /* Prepare for the event that this was the last iteration ... */
      s_current = s_new;
    }
  }

  /* Drive towards next waypoint */
  {
    const Vector delta = VL_subtract(s_exit, s_current);
    const double angle_rad = atan2(VL_get_y(delta), VL_get_x(delta));

    local_drivetowards(s_exit, angle_rad, targetdist, TURNSPEED, TURNACC);
  }
}
