#include <math.h>

#include "turn_support_lib.h"
#include "vector_lib.h"

static Vector mirror_vector(const Vector a)
     /* In y axis */
{
  const double x = VL_get_x(a);
  const double y = VL_get_y(a);
  const Vector s = VL_set_xy(-x, y);

  return s;
}

static double mirror_angle(const double alfa)
     /* In y axis */
{
#if 0
  const double beta = TURN_SL_norm_mpi_pi(alfa);
  double phi = 0.0;

  if (beta >= 0) {
    phi = M_PI - beta;
  } else {
    phi = - M_PI - beta;
  }

  return phi;
#else
  const Vector v = VL_set_xy(cos(alfa), sin(alfa));
  const Vector v_mirror = mirror_vector(v);
  const double xm = VL_get_x(v_mirror);
  const double ym = VL_get_y(v_mirror);
  const double beta = atan2(ym, xm);

  return beta;
#endif
}

TURN_SL_link_test_type TURN_SL_link_lr_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius)
{
  /* By inverting one coordinate, the 'TURN_SL_link_rl_test' function can be
   * used by this function. */
  TURN_SL_link_test_type rs = {0};

  const Vector x_pos_entry = mirror_vector(pos_entry);
  const double x_alfa_entry = mirror_angle(alfa_entry);

  const Vector x_pos_exit = mirror_vector(pos_exit);
  const double x_alfa_exit = mirror_angle(alfa_exit);

  const TURN_SL_link_test_type rl_result = TURN_SL_link_rl_test(x_pos_entry,
								x_alfa_entry,
								x_pos_exit,
								x_alfa_exit,
								turn_radius);

  if (rl_result.valid) {
    const double beta_entry = - rl_result.beta_entry;
    const double beta_exit = - rl_result.beta_exit;
    const Vector center_entry = mirror_vector(rl_result.center_entry);
    const Vector center_exit = mirror_vector(rl_result.center_exit);
    const Vector pos_t_entry = mirror_vector(rl_result.pos_t_entry);
    const Vector pos_t_exit = mirror_vector(rl_result.pos_t_exit);
    const double beta_tangent = mirror_angle(rl_result.beta_tangent);
    
    rs.beta_entry = beta_entry;
    rs.beta_exit = beta_exit;
    rs.center_entry = center_entry;
    rs.center_exit = center_exit;
    rs.pos_t_entry = pos_t_entry;
    rs.pos_t_exit = pos_t_exit;
    rs.beta_tangent = beta_tangent;
    
    rs.total_curve_length = rl_result.total_curve_length;
    rs.w = rl_result.w;

    rs.valid = 1;
  } else {
    /* Some problem occurred. */
    rs.valid = 0;
  }

  return rs;
}
