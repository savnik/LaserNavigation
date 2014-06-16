#include <math.h>

#include "turn_support_lib.h"

TURN_SL_link_test_type TURN_SL_link_ll_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius)
{
  TURN_SL_link_test_type rs = {0};

  /* Obtain result by reversing the direction of traversal and call other function. */
  const TURN_SL_link_test_type rr_result = TURN_SL_link_rr_test(pos_exit,
								alfa_exit - M_PI,
								pos_entry,
								alfa_entry - M_PI,
								turn_radius);

  if (rr_result.valid) {
    /* Valid result, now rearrange results for original call arguments ... */
    rs.beta_entry = - rr_result.beta_exit; /* Turn left instead of right. */
    rs.beta_exit = - rr_result.beta_entry; /* Turn left instead of right. */

    rs.center_entry = rr_result.center_exit;
    rs.center_exit = rr_result.center_entry;

    rs.pos_t_entry = rr_result.pos_t_exit;
    rs.pos_t_exit = rr_result.pos_t_entry;

    rs.beta_tangent = TURN_SL_norm_mpi_pi(rr_result.beta_tangent - M_PI);

    rs.total_curve_length = rr_result.total_curve_length;
    rs.w = rr_result.w;
    rs.valid = rr_result.valid;
  } else {
    /* Not valid */
  }

  return rs;
}
