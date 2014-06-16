/*-------------------------------------------------------------------------------------------------------------------------------------------------------
 * Fault-tolerant Control of Field Robot                                                                                                              F08
 *                                                                                                                                  S�ren Hansen, s021751
 * linreg.h:  Header file for the linear regression function. Defines the data type used.                                           Peter Tjell,  s032041
 *-------------------------------------------------------------------------------------------------------------------------------------------------------
 */
 
# ifndef LINREG_DOT_H
# define LINREG_DOT_H

# include <math.h>

// Defines functions as external C code. For use in C++ programs.
#ifdef __cplusplus
 extern "C" {
#endif

typedef struct
{
  // User input:
  double *x_array;  // Array containing x values
  double *y_array;  // Array containing y values
  int N;  // Number of datasets
  
  // Internal variables:
  long double sum_xy;  // Sum of x � y
  long double sum_x;   // Sum of x
  long double sum_y;   // Sum of y
  long double sum_xx;  // Sum of x^2
  long double sum_yy;  // Sum of y^2

  // Error value:
  int error;  // 1 if an error occurs during regression (the output is invalid).

  // Regression output:
  long double a, b, c;   // Output. Constants to form the line ax + by + c = 0.
  double normr;     // Norm of the residuals
  // extend along major axis
  double tMin, tMax;
  // extend across minor axis
  double dMin, dMax;
  // center of gravity for point cloud
  double cogX, cogY;
} regtype;

void linear_regression(regtype *reg);

#ifdef __cplusplus
 }
#endif

# endif // LINREG_DOT_H
