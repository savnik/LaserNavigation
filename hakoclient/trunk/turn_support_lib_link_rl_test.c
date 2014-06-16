#include <math.h>

#if 0
#define TEST 1
#endif

#ifdef TEST
#include <stdio.h>
#endif

#include "turn_support_lib.h"
#include "vector_lib.h"

TURN_SL_link_test_type TURN_SL_link_rl_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius)
{
#ifdef TEST
  int dummy = printf("RL entry\n");
#endif

  TURN_SL_link_test_type rs = {0};

  /* Entry */
  const double phi_entry = alfa_entry - M_PI / 2.0;
  const Vector pen_cen = VL_set_xy(turn_radius * cos(phi_entry), turn_radius * sin(phi_entry));
  const Vector entry_center = VL_add(pos_entry, pen_cen);

  /* Exit */
  const double phi_exit = alfa_exit + M_PI / 2.0;
  const Vector pex_cex = VL_set_xy(turn_radius * cos(phi_exit), turn_radius * sin(phi_exit));
  const Vector exit_center = VL_add(pos_exit, pex_cex);

  const double dist_frac =
    (VL_dist(entry_center, exit_center) / 2.0 - turn_radius) / turn_radius;

  if (dist_frac > TURN_SL_FRAC_LIMIT) {
    /* Numerical conditions OK ... */

    /* Vector from entry_center (C1) to exit_center (C2). */
    const Vector c1_c2 = VL_subtract(exit_center, entry_center);
    const double phi_center_line = atan2(VL_get_y(c1_c2), VL_get_x(c1_c2));

    /* Note that the tangent intersects the line between C1 and C2 in the mid-point
     * between them. This point is called Q. */
    const double dist_c1_c2 = VL_length(c1_c2);
    const double dist_c1_q = dist_c1_c2 / 2.0;
    
    /* There is a triangle with 90 degrees vertex.
     *   a = turn_radius
     *   b = ??           (distance from T1 to Q)
     *   c = dist_c1_q
     */
    const double dist_t1_q = sqrt(pow(dist_c1_q, 2.0) - pow(turn_radius, 2.0)); /* Pythagoras */

    /* Thus the angle between the center line and the tangent can thus be calculated using 'atan2' */
    const double theta = atan2(turn_radius, dist_t1_q);
#ifdef TEST
    int dummy = printf("RL: theta = %f \n", theta);
#endif
    /* Now the angle of the tangent line can be calculated! */
    const double beta_tangent = TURN_SL_norm_mpi_pi(phi_center_line - theta);

    /* Determination of T1 */
    const double phi_radius_rturn = beta_tangent + M_PI / 2.0;
    const Vector rturn_tangent_radius_vector =
      VL_set_xy(turn_radius * cos(phi_radius_rturn),
		turn_radius * sin(phi_radius_rturn));
    const Vector pos_t_entry = VL_add(entry_center, rturn_tangent_radius_vector);

    /* Determination of T2 */
    const double phi_radius_lturn = beta_tangent - M_PI / 2.0;
    const Vector lturn_tangent_radius_vector =
      VL_set_xy(turn_radius * cos(phi_radius_lturn),
		turn_radius * sin(phi_radius_lturn));
    const Vector pos_t_exit = VL_add(exit_center, lturn_tangent_radius_vector);

    /* Arc lengths (radians) */
    const double beta_entry = TURN_SL_norm_m2pi_0(beta_tangent - alfa_entry);
    const double beta_exit = TURN_SL_norm_0_2pi(alfa_exit - beta_tangent);

    const double total_curve_length =
      turn_radius * fabs(beta_entry) + 
      VL_dist(pos_t_entry, pos_t_exit) +
      turn_radius * fabs(beta_exit);

    /* Assign values to return structure ... */
    rs.beta_entry = beta_entry;
    rs.beta_exit = beta_exit;
    rs.center_entry = entry_center;
    rs.center_exit = exit_center;
    rs.pos_t_entry = pos_t_entry;
    rs.pos_t_exit = pos_t_exit;
    rs.beta_tangent = beta_tangent;
    rs.total_curve_length = total_curve_length;
    
    if ((fabs(beta_entry) > M_PI) || (fabs(beta_exit) > M_PI)) {
      /* Wide angles (turning around). */
      rs.w = 1;
    } else {
      rs.w = 0;
    }

    rs.valid = 1;
  } else {
    /* Circles too close (or even overlapping) ... */
  }

  return rs;
}
