/*-------------------------------------------------------------------------------------------------------------------------------------------------------
 * Fault-tolerant Control of Field Robot                                                                                                              F08
 *                                                                                                                                  Søren Hansen, s021751
 * linreg.c:  Calculates linear regression by using the least squares method                                                        Peter Tjell,  s032041
 *-------------------------------------------------------------------------------------------------------------------------------------------------------
 */
 
# include "linreg.h"
#include <stdio.h>

void clear_variables(regtype *reg);


/*
  Makes linear regression with reg->N number of points from 
  reg->x_array and reg->y_array. The points are treated as 
  points in a normal two-dimensional Cartesian coordinate-
  system.
*/
void linear_regression(regtype *reg)
{
  int i;
  double sum_residuals = 0;
  double x_val, y_val;
  double sum_xl = 0, sum_yl = 0;

  double x, y, d, t;

  clear_variables(reg);

  for(i = 0; i < reg->N; i++)
    {
      x_val = *(reg->x_array + i) - *(reg->x_array);
      y_val = *(reg->y_array + i) - *(reg->y_array);
      reg->sum_xy += x_val * y_val;
      reg->sum_x += x_val;
      reg->sum_y += y_val;
      reg->sum_xx += x_val * x_val; 
      reg->sum_yy += y_val * y_val;
      sum_xl += *(reg->x_array + i);
      sum_yl += *(reg->y_array + i);
      //printf("%.10f %.10f\n", *(reg->x_array + i), *(reg->y_array + i));
    }
    
  // If the given points are can't be calculated the error bit is set.
  if((reg->N * reg->sum_xx - reg->sum_x * reg->sum_x == 0.0) && (reg->N * reg->sum_yy - reg->sum_y * reg->sum_y == 0.0))
    {
      reg->error = 1;
      return;
    }
  
  if(fabs(reg->N * reg->sum_xx - reg->sum_x * reg->sum_x) > fabs(reg->N * reg->sum_yy - reg->sum_y * reg->sum_y))
    {
      reg->a = (reg->N * reg->sum_xy - reg->sum_x * reg->sum_y) / (reg->N * reg->sum_xx - reg->sum_x * reg->sum_x);
      reg->b = - 1.0;
      reg->c = (sum_yl - reg->a * sum_xl) / reg->N;  
    }
  else
    {
      reg->a = - 1.0;
      reg->b = (reg->N * reg->sum_xy - reg->sum_x * reg->sum_y) / (reg->N * reg->sum_yy - reg->sum_y * reg->sum_y );
      reg->c =  (sum_xl - reg->b * sum_yl) / reg->N;
    }
  // normalize equation, so that a²+b²=1;
  d = hypot(reg->a, reg->b);
  reg->a /= d;
  reg->b /= d;
  reg->c /= d;
  // Calculate the norm of the residuals:
  for(i = 0; i < reg->N; i++)
    sum_residuals += pow(*(reg->y_array + i) + reg->a/reg->b * *(reg->x_array + i) + reg->c/reg->b, 2);
  reg->normr = sqrt(sum_residuals);

  reg->cogX = reg->sum_x/(double)reg->N;
  reg->cogY = reg->sum_y/(double)reg->N;
  //printf("COG x=%f, y=%f\n", x, y);
  // COG
  
  for(i = 0; i < reg->N; i++)
  {
    x = *(reg->x_array + i);
    y = *(reg->y_array + i);
    t = reg->b * x + reg->a * -y;
    d = x * reg->a + y*reg->b + reg->c; // assuming line is normalized
    if (i == 0)
    {
      reg->tMin = t;
      reg->tMax = t;
      reg->dMin = d;
      reg->dMax = d;
    }
    else
    {
      if (t < reg->tMin)
        reg->tMin = t;
      if (t > reg->tMax)
        reg->tMax = t;
      if (d < reg->dMin)
        reg->dMin = d;
      if (d > reg->dMax)
        reg->dMax = d;
    }
  }
  //printf("REG: A = %.10Lf, B = %.10Lf, C = %.10Lf\n", reg->a, reg->b, reg->c);
}


/*
  Initializes all the internal values and resets output.
*/
void clear_variables(regtype *reg)
{
  //reg->N = 0;
  reg->error = 0;
  reg->sum_xy = 0;
  reg->sum_x = 0;
  reg->sum_y = 0;
  reg->sum_xx = 0;
  reg->sum_yy = 0;
  reg->a = 0;
  reg->b = 0;
  reg->c = 0;
  reg->normr = 0;
}
