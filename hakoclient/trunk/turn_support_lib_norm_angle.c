#include <math.h>

#include "turn_support_lib.h"

double TURN_SL_norm_mpi_pi(double alfa)
{
  const double x = cos(alfa);
  const double y = sin(alfa);
  const double beta = atan2(y, x); /* Probably very inefficient way to normalize angle ... */

  return beta;
}

double TURN_SL_norm_m2pi_0(double alfa)
{
  const double phi = TURN_SL_norm_mpi_pi(alfa);
  double beta = phi;

  if (beta > 0) {
    beta = beta - 2.0 * M_PI;
  }

  return beta;
}

double TURN_SL_norm_0_2pi(double alfa)
{
  const double phi = TURN_SL_norm_mpi_pi(alfa);
  double beta = phi;

  if (beta < 0) {
    beta = beta + 2.0 * M_PI;
  }

  return beta;
}
