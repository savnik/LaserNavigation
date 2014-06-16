#include <math.h>
#include <stdio.h>

#include "turn_support_lib.h"

static TURN_SL_link_test_type choose_best(const TURN_SL_link_test_type s1,
					  const TURN_SL_link_test_type s2)
     /* Selects the optimum path (of the two presented). */
{
  TURN_SL_link_test_type rs = {0};

  if (s1.valid) {
    if (s2.valid) {
      /* Both are valid - now compare total curve length. */
      if (s1.total_curve_length > s2.total_curve_length) {
	/* Curve 2 is shorter ... */
	rs = s2;
      } else {
	rs = s1; /* Keep 's1' */
      }
    } else {
      /* 's2' not valid! */
      rs = s1;
    }
  } else {
    /* Current structure 's1' not valid. */
    if (s2.valid) {
      rs = s2;
    } else {
      /* Neither 's1' nor 's2' valid. */
    }
  }

  return rs;
}


TURN_SL_link_test_type TURN_SL_link_test(Vector pos_entry,
					 double alfa_entry,
					 Vector pos_exit,
					 double alfa_exit,
					 double turn_radius)
{
  const TURN_SL_link_test_type s_rr =
    TURN_SL_link_rr_test(pos_entry, alfa_entry, pos_exit, alfa_exit, turn_radius);

  const TURN_SL_link_test_type s_ll =
    TURN_SL_link_ll_test(pos_entry, alfa_entry, pos_exit, alfa_exit, turn_radius);

  const TURN_SL_link_test_type s_rl =
    TURN_SL_link_rl_test(pos_entry, alfa_entry, pos_exit, alfa_exit, turn_radius);

  const TURN_SL_link_test_type s_lr =
    TURN_SL_link_lr_test(pos_entry, alfa_entry, pos_exit, alfa_exit, turn_radius);

  TURN_SL_link_test_type s_current = {0};

  if ((s_rr.valid && !s_rr.w) || (s_rl.valid && !s_rl.w) ||
      (s_ll.valid && !s_ll.w) || (s_lr.valid && !s_lr.w)) {
    /* There is at least one solution without excessive angles. */
    printf("Link solution exists!\n");
  };

  s_current = choose_best(s_rr, s_ll);
  s_current = choose_best(s_current, s_rl);
  s_current = choose_best(s_current, s_lr);
  
  if (s_current.valid) {
    /* Valid path found! */
  } else {
    /* No valid path available. */
  }

  return s_current;
}
