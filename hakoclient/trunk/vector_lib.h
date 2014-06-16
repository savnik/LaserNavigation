#ifndef VECTOR_LIB_HDR
#define VECTOR_LIB_HDR

#include "vector_type.h"

/* This library is NOT intended to follow JAUS conventions.
* Instead Easting follows x and 
* Northing follows y. */

Vector VL_add(const Vector a, const Vector b);
Vector VL_add3(const Vector a, const Vector b, const Vector c);
/* Adds vectors. */

Vector VL_subtract(const Vector a, const Vector b);
/* a - b */

Vector VL_scalmult(const double k, const Vector a);
/* Multiply vector by scalar k. */

double VL_length(const Vector a);
/* Length of vector a. */

double VL_get_x(const Vector a);
double VL_get_y(const Vector a);

Vector VL_set_xy(const double x, const double y);
/* Add more functions as the need arises ... */

Vector VL_rotate90(const Vector a);
/* Rotate vector 90 degrees (Counter Clock Wise). */
/* With NavCom on Hako conventions (i.e. non-JAUS):
 *   Easting axis is rotated to Northing axis. */

Vector VL_rotate270(const Vector a);
/* Rotate vector 270 degrees (Counter Clock Wise, which is equivalent to
* a 90 degrees Clock Wise rotation). */
/* With NavCom on Hako conventions (i.e. non-JAUS):
 *   Northing axis is rotated to Easting axis. */

Vector VL_mk_unit(const Vector a);
/* Return unit vector with same direction as vector 'a'. */

Vector VL_set_length(const Vector a, const double k);
/* Return vector with same direction as vector 'a', but with length 'k'.
* Note: if k is negative, then the direction will be opposite. */

double VL_dot(const Vector a, const Vector b);
/* Returns the dot-product of the vectors, that is:
*     a.x * b.x + a.y * b.y
* Which should be identical to:
*   cos(angle(a,b)) * length(a) * length(b)
*/

double VL_det(const Vector a, const Vector b);
/* Returns the determinant of the vectors, that is:
*     a.x * b.y - a.y * b.x
* Which should be identical to:
*   sin(angle(a,b)) * length(a) * length(b)
*/

double VL_dist(const Vector a, const Vector b);
/* Returns the distance between the two points described by vectors. */

Vector VL_mid(const Vector a, const Vector b);
/* Returns the midpoint between the two points described
* by the vectors 'a' and 'b'. */

double VL_line_dist_core(const Vector line_p, const Vector line_r, const Vector point);
/* Returns the distance from the line described by a point 'line_p' and a direction
* vector 'line_r' to a point 'point'.
* This is the core functionality - without considerations for numerical properties. */

double VL_line_dist(const Vector line_p, const Vector line_r, const Vector point);
/* As 'VL_line_dist_core', but with local coordinate system to improve numerical
* properties. */

Vector VL_line_intersect_core(const Vector line_p1, const Vector line_r1,
						      const Vector line_p2, const Vector line_r2);
/* Returns the intersection point between the two lines,
* line 1:
*   Point on line 'line_p1' and direction vector 'line_r1'.
* and line 2:
*   Point on line 'line_p2' and direction vector 'line_r2'. 
* If the lines are parallel, then the behaviour is undefined (probably NaNs for
* coordinates, possibly +Inf or -Inf). */

Vector VL_line_intersect(const Vector line_p1, const Vector line_r1,
						 const Vector line_p2, const Vector line_r2);
/* As 'VL_line_intersect_core' above, but with local coordinate system to improve
* numerical properties if used with coordinate system with large offsets (i.e. GPS). */
/* Handling of parallel lines to be decided upon and implemented ... */

#endif /* VECTOR_LIB_HDR */
