#ifndef TURN_SUPPORT_LIB_HDR
#define TURN_SUPPORT_LIB_HDR

/* Support functions for TURN_* functions. */

#include "vector_type.h"

#define TURN_SL_FRAC_LIMIT 0.001
/* Fraction used when determining whether numerical problems present ... */

double TURN_SL_norm_mpi_pi(double alfa);
/* Normalizes angle in the range:  -pi to pi  */

double TURN_SL_norm_m2pi_0(double alfa);
/* Normalizes angle in the range -2*pi to 0  */

double TURN_SL_norm_0_2pi(double alfa);
/* Normalizes angle in the range 0 to 2*pi   */



/* Evaluation of various turn scenarios (link). */
typedef struct {
  int valid; /* Whether a solution was found (i.e. the rest of the fields are valid). */
  double beta_entry; /* The arc length to travel on entry (positive is "turn left",
		      * negative is "turn right" - similar to SMR-CL turnr (only in radians)). */
  double beta_exit; /* The arc length to travel when leaving "link" (similar to 'beta_entry'). */
  Vector center_entry; /* Center of circle movement on entry. */
  Vector center_exit; /* Center of circle movement on exit. */
  Vector pos_t_entry; /* Location of tangent point (entry side). */
  Vector pos_t_exit; /* Location of vector point (exit side). */
  double beta_tangent; /* Angle (in radians) of the tangent connecting circles. */
  double total_curve_length; /* Total curve length (arc, tangent, arc). */
  int w; /* Flag stating whether any of the angles will bring it "back" into row area ... 
	  * i.e. whether any angle is bigger than pi (abs.value). */
} TURN_SL_link_test_type;

TURN_SL_link_test_type TURN_SL_link_rr_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius);
/* This function takes the entry and exit information (location and angle) and evaluates
 * how this could be obtained using "right turns" both at entry and exit (connected with
 * tangent line - see sketches ...). */
/* There will always exist a solution for this situation (ignoring numerical problems),
 * but it might include a "near full turn" which will make it unsuitable. */

TURN_SL_link_test_type TURN_SL_link_ll_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius);
/* This function is similar to the 'TURN_SL_link_rr_test' function, only here the situation
 * is evaluated for "left turns" both at entry and exit. */

TURN_SL_link_test_type TURN_SL_link_rl_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius);
/* This function evaluates whether a right turn, followed by straight line, followed by
 * a left turn can be used to go from entry to exit.
 * There might not exist a solution under these conditions. */

TURN_SL_link_test_type TURN_SL_link_lr_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius);
/* Similar to 'TURN_SL_rl_test', only now it is a left turn, then a straight line,
 * and then a right turn. */

TURN_SL_link_test_type TURN_SL_link_test(Vector pos_entry,
					    double alfa_entry,
					    Vector pos_exit,
					    double alfa_exit,
					    double turn_radius);
/* This function will select the optimum solution of the above 4. */
/* For numerical reasons it might not return a valid solution ... */

#endif /* TURN_SUPPORT_LIB_HDR */
