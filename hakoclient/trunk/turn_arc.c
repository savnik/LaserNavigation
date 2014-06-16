#include "turn_arc.h"

int TURN_arc(const pointtype dmp_i,
	     const double turn_angle,
	     const double turn_radius,
	     const pointtype dmp_ip1,
	     const pointtype dmp_ip2)
{
  double adj_turn_angle = 0.0;
  double adj_turn_radius = 0.0;

  if (0.0 == turn_angle) {
    /* The argument turn_angle was zero (i.e. not set in route plan),
     * thus use default value. */
    adj_turn_angle = 90.0;
  } else {
    /* 'turn_angle' different from zero - thus use it. */
    adj_turn_angle = turn_angle;
  }

  if (0.0 == turn_radius) {
    /* The argument 'turn_radius' was zero (i.e. not set in route plan),
     * thus use default value. */
    adj_turn_radius = 3.0;
  } else {
    /* 'turn_radius' different from zero - thus use it. */
    adj_turn_radius = turn_radius;
  }

  {
    char str[2000] = {0};
    
    printf("  TURN_arc called (angle=%f, radius=%f).\n!", turn_angle, turn_radius);
    cl_send("set \"usekalmanodo\" 0\n");
    sprintf (str,
	     "turnr %f %f @v%f @a%f\n",
	     adj_turn_radius,
	     adj_turn_angle,
	     TURNSPEED,
	     TURNACC);
    cl_send (str);
    printf("  SMR-CL string : %s \n", str);
  }


  cl_send("set \"usekalmanodo\" 1\n");
  driveto(dmp_ip1.n, dmp_ip1.e, dmp_ip2.ang, TURNSPEED,TURNACC);

  return 0;
}
