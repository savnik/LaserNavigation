#include <math.h>

#if 1
#include <stdio.h>
#include <stdlib.h>
#endif

#include "vector_type.h"
#include "vector_lib.h"

/* Implement the vector functions ... */

Vector VL_add(const Vector a, const Vector b)
{
	Vector u = {0.0, 0.0};

	u.x = a.x + b.x;
	u.y = a.y + b.y;

	return u;
}

Vector VL_add3(const Vector a, const Vector b, const Vector c)
{
	Vector u = {0.0, 0.0};

	u.x = a.x + b.x + c.x;
	u.y = a.y + b.y + c.y;

	return u;
}

Vector VL_subtract(const Vector a, const Vector b)
{
	Vector u = {0.0, 0.0};

	u.x = a.x - b.x;
	u.y = a.y - b.y;

	return u;
}

Vector VL_scalmult(const double k, const Vector a)
{
	Vector u = {0.0, 0.0};

	u.x = k * a.x;
	u.y = k * a.y;

	return u;
}

double VL_length(const Vector a)
{
	const double d2 = a.x * a.x + a.y * a.y;
    double d = sqrt(d2);

	return d;
}

double VL_get_x(const Vector a)
{
	return a.x;
}

double VL_get_y(const Vector a)
{
	return a.y;
}

Vector VL_set_xy(const double x, const double y)
{
	Vector u = {0.0, 0.0};

	u.x = x;
	u.y = y;

	return u;
}

Vector VL_rotate90(const Vector a)
{
	Vector u = {0.0, 0.0};

	u.x = -a.y;
	u.y = a.x;

	return u;
}

Vector VL_rotate270(const Vector a)
{
	Vector u = {0.0, 0.0};

	u.x = a.y;
	u.y = -a.x;

	return u;
}

Vector VL_mk_unit(const Vector a)
{
	/* Do check for (0, 0) vector ?? */
	const double s = VL_length(a);
	const Vector u = VL_scalmult(1/s, a);

	return u;
}

Vector VL_set_length(const Vector a, const double k)
{
	const Vector u = VL_mk_unit(a);
	const Vector v = VL_scalmult(k, u);

	return v;
}

double VL_dot(const Vector a, const Vector b)
{
	const double dot = a.x * b.x + a.y * b.y;

	return dot;
}

double VL_det(const Vector a, const Vector b)
{
	const double det = a.x * b.y - a.y * b.x;

	return det;
}

double VL_dist(const Vector a, const Vector b)
{
	const Vector delta = VL_subtract(a, b);
	const double dist = VL_length(delta);

	return dist;
}

Vector VL_mid(const Vector a, const Vector b)
{
	const double x = (a.x + b.x) / 2;
	const double y = (a.y + b.y) / 2;
	Vector r = {0};

	r.x = x;
	r.y = y;

	return r;
}

double VL_line_dist_core(const Vector line_p,
						 const Vector line_r,
						 const Vector point)
{
	/* "Normal vector" components - not necessarily unit vector! */
	const double a = - line_r.y; /* Direction vector rotated 90 degrees CCW. */
	const double b =   line_r.x; /*  - do - */

	/* Calculation of 'c' in line equation: a*x + b*y = c */
	const double c = a * line_p.x + b * line_p.y;

	/* Numerator in distance expression */
	const double n_dist = fabs(a * point.x + b * point.y + c);

	/* Denominator in distance expression */
	const double d_dist = sqrt(pow(a, 2) + pow(b, 2));

	/* N/D of distance expression (i.e. the final result). */
	const double dist_to_point = n_dist / d_dist;

	return dist_to_point;
}

double VL_line_dist(const Vector line_p,
					const Vector line_r,
					const Vector point)
{
	/* Subtracting 'line_p' before call of 'VL_line_dist_core' */
	const Vector point_dot = VL_subtract(point, line_p);
	const Vector line_p_dot = VL_set_xy(0.0, 0.0);

	/* Calculate the distance - using the 'dot' coordinate system. */
	const double dist = VL_line_dist_core(line_p_dot, line_r, point_dot);

	return dist;
}

Vector VL_line_intersect_core(const Vector line_p1,
							  const Vector line_r1,
						      const Vector line_p2,
							  const Vector line_r2)
{
	/* In this implementation it is assumed that NaN's are non-signalling(??)
	* (i.e. division by zero will not cause the program to stop, but rather
	* return some NaN code). */
	/* It is NOT required that the r-vectors be unit vectors (other lengths than
	* 1.0 are acceptable). */

	/* The coefficients in equation for line 1:  a1*x + b1*y = c1 */
	const double a1 = - line_r1.y; /* Direction vector rotated 90 degrees CCW. */
	const double b1 =   line_r1.x; /*  - do - */
	const double c1 = a1 * line_p1.x + b1 * line_p1.y;

	/* The coefficients in equation for line 2:  a2*x + b2*y = c2 */
	const double a2 = - line_r2.y; /* Direction vector rotated 90 degrees CCW. */
	const double b2 =   line_r2.x; /*  - do - */
	const double c2 = a2 * line_p2.x + b2 * line_p2.y;

	/* Setting up argument vectors for determinant calculations ... */
	const Vector va = VL_set_xy(a1, a2);
	const Vector vb = VL_set_xy(b1, b2);
	const Vector vc = VL_set_xy(c1, c2);

	/* Do the determinant calculations and thus coordinates of intersection ... */
	const double x0 = VL_det(vc, vb) / VL_det(va, vb);
	const double y0 = VL_det(va, vc) / VL_det(va, vb);

	/* Construct the result vector */
	const Vector intersection_point = VL_set_xy(x0, y0);

	return intersection_point;
}

#define SINE_R1R2_LOW_LIMIT (1e-7)
Vector VL_line_intersect(const Vector line_p1, const Vector line_r1,
						 const Vector line_p2, const Vector line_r2)
{
	/* JoN: 2007jun11 */
	/* First, check whether the lines are parallel */
	const double det_r1r2 = VL_det(line_r1, line_r2);
	const double sine_r1r2 = det_r1r2 / VL_length(line_r1) / VL_length(line_r2);
	Vector ret_vector = VL_set_xy(0.0, 0.0);

	if (fabs(sine_r1r2) >= SINE_R1R2_LOW_LIMIT) {
		/* sin(v), where 'v' is the angle between the lines is big enough
		* for the lines to be considered "not parallel". */
		/* Convert to local coordinate system with 'line_p1' as Origo (for 
		* numerical reasons). The mean between the two points would also be
		* a good Origo. */
		const Vector line_p1_dot = VL_set_xy(0.0, 0.0);
		const Vector line_p2_dot = VL_subtract(line_p2, line_p1);

		/* Calculate intersection coordinates - in local coordinate system. */
		const Vector intersection_dot =
			VL_line_intersect_core(line_p1_dot, line_r1, line_p2_dot, line_r2);

		/* Convert back to "global" coordinate system. */
		const Vector intersection = VL_add(intersection_dot, line_p1);
	
		/* Should the return value be tested - and if so, what should it be
		* replaced with if it is out of bound? */
		if ((-HUGE_VAL < VL_get_x(intersection)) &&
			(VL_get_x(intersection) < HUGE_VAL) &&
			(-HUGE_VAL < VL_get_y(intersection)) &&
			(VL_get_y(intersection) < HUGE_VAL)) {
				/* No range error (??) */
				ret_vector = intersection;
			} else {
				/* Some kind of range error ... */
				/* For now, return the location as is anyway.
				* The whole vector should probably be NaN'ed. */
				ret_vector = intersection;
			}
	} else {
		/* The angle between lines is not big enough for them to be considered
		* "not parallel" - this implies that they are "parallel" or that some
		* range error has occurred in the calculations (i.e. result is NaN). */
		ret_vector = VL_set_xy(0.0, 0.0); /* Expressions should evaluate to NaN. */
		fprintf(stderr, "vector_lib: VL_line_intersect: handling of parallel lines to be implemented ... \n");
		exit(-1);
	}

	return ret_vector;
}
