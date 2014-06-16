#include <math.h>

#include "turn_support_lib.h"

#include "vector_lib.h"

TURN_SL_link_test_type TURN_SL_link_rr_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius)
{
  TURN_SL_link_test_type rs = {0};

  /* Entry */
  const double phi_entry = alfa_entry - M_PI / 2.0;
  const Vector pen_cen = VL_set_xy(turn_radius * cos(phi_entry), turn_radius * sin(phi_entry));
  const Vector entry_center = VL_add(pos_entry, pen_cen);

  /* Exit */
  const double phi_exit = alfa_exit - M_PI / 2.0;
  const Vector pex_cex = VL_set_xy(turn_radius * cos(phi_exit), turn_radius * sin(phi_exit));
  const Vector exit_center = VL_add(pos_exit, pex_cex);

  if (VL_dist(entry_center, exit_center) > (turn_radius * TURN_SL_FRAC_LIMIT)) {
    /* Numerical conditions OK ... */

    /* Vector from entry_center (C1) to exit_center (C2). */
    const Vector c1_c2 = VL_subtract(exit_center, entry_center);
    const double beta_tangent = atan2(VL_get_y(c1_c2), VL_get_x(c1_c2));
    
    /* Tangent point */
    const double phi_radius_tangent = beta_tangent + M_PI / 2.0;
    const Vector tangent_radius_vector = VL_set_xy(turn_radius * cos(phi_radius_tangent),
						   turn_radius * sin(phi_radius_tangent));
    const Vector pos_t_entry = VL_add(entry_center, tangent_radius_vector);
    const Vector pos_t_exit = VL_add(exit_center, tangent_radius_vector);
    
    const double w_beta_entry = beta_tangent - alfa_entry;
    /* The above angle should be normalized for the range -2*pi to 0 ... */
    const double beta_entry = TURN_SL_norm_m2pi_0(w_beta_entry);

    const double w_beta_exit = alfa_exit - beta_tangent;
    /* The above angle should be normalized for the range -2*pi to 0 ... */
    const double beta_exit = TURN_SL_norm_m2pi_0(w_beta_exit);

    /* Total curve length */
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
    /* Centers too close ... */
  }

  return rs;
}
