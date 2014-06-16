#include <math.h>
#include <stdio.h>

#include "turn_wide.h"
#include "functions.h"

#include "vector_type.h"
#include "vector_lib.h"

static void mk_turn(const Vector r1, const Vector r2, const double radius);
static void mk_turn(const Vector r1, const Vector r2, const double radius)
{
  const double sin_v = VL_det(r1, r2); /* By assumption that they are unit vectors ... */
  const double cos_v = VL_dot(r1, r2); /* By assumption that they are unit vectors ... */
  const double v_rad = atan2(sin_v, cos_v); /* NOT ADJUSTED PROPERLY!!! */
  const double v_deg = v_rad * 180.0 / M_PI;
  
  {
    char str[2000] = {0};
    
    printf("  mk_turn called (r1=(%f, %f), r2=(%f, %f), radius=%f .\n!", r1.x, r1.y, r2.x, r2.y, radius);
    cl_send("set \"usekalmanodo\" 0\n");
    sprintf (str,
	     "turnr %f %f @v%f @a%f\n",
	     radius,
	     v_deg,
	     TURNSPEED,
	     TURNACC);
    cl_send (str);
    printf("  SMR-CL string : %s \n", str);
  }
}

int TURN_wide(const pointtype dmp_i, const pointtype dmp_ip1, const pointtype dmp_ip2)
{
  printf("TURN_wide called ...\n");

  /* Calculate angles in radians (from degrees). */
  const double entry_angle_deg = dmp_i.ang;
  const double entry_angle_rad = dmp_i.ang / 180.0 * M_PI;
  const double exit_angle_rad = dmp_ip2.ang / 180.0 * M_PI;

  /* Calculate vectors describing "boundary condition" of turn. */
  const Vector p1 = VL_set_xy(dmp_i.e, dmp_i.n);
  printf("  TURN_wide: entry_angle_rad=%f \n", entry_angle_rad);
  const Vector r1 = VL_rotate90(VL_set_xy(cos(entry_angle_rad), sin(entry_angle_rad)));
  const Vector p2 = VL_set_xy(dmp_ip1.e, dmp_ip1.n);
  const Vector r2 = VL_rotate90(VL_set_xy(cos(exit_angle_rad), sin(exit_angle_rad)));
  printf("  TURN_wide: r1=(%f, %f), r2=(%f, %f).\n", r1.x, r1.y, r2.x, r2.y);

  /* The following approach assumes fairly benign point settings - i.e. might not
   * recover nicely from "unexpected" arguments. */
  /* Calculate the intersection point */
  const Vector c1 = VL_line_intersect(p1, r1, p2, r2);
  printf("  TURN_wide: intersection (%f, %f).\n", c1.x, c1.y);

  /* Calculate distances ... */
  const double d1 = VL_dist(p1, c1);
  const double d2 = VL_dist(p2, c1);
  const double diff_d = d2 - d1;

  if (diff_d > -0.1) {
    /* The difference in distances is below 10 cm or it is AFTER the turn, so it is not considered 
     * necessary to adjust by moving forward in straight line before entering the turn. */
    printf("  Not adjusting ... \n");

    mk_turn(r1, r2, d1);
  } else {
    /* Difference bigger than acceptable (before turn) (or range error) */
    /* Drive forward for 'diff_d' meters, then do the turn. */
    printf("  Adjusting ... \n");
    const Vector delta_s = VL_set_length(r1, -diff_d);
    const Vector adjusted_target_pos = VL_add(VL_set_xy(dmp_i.e, dmp_i.n), delta_s);
    
    driveto(VL_get_y(adjusted_target_pos),
	    VL_get_x(adjusted_target_pos),
	    entry_angle_deg,
	    TURNSPEED,
	    TURNACC);

    /* Now do the turning */
    mk_turn(r1, r2, d1);
  }

  cl_send("set \"usekalmanodo\" 1\n");
  driveto(dmp_ip1.n, dmp_ip1.e, dmp_ip2.ang, TURNSPEED,TURNACC);

  return 0;
}
