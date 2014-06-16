# include "blockdetect.h"

/**
   This function filters a signal by this formula:
     y[n] = 1/f * (x[n] + x[n-1] + ... + x[n-(f-1)])

   The input x is given in the array values, where values[0] = x[n].
 */
double filter(double *values)
{
  double res = 0;
  int i;
  
  for(i = 0; i < FILTER_SIZE; i++)
    res += values[i];
  res /= FILTER_SIZE;
  return(res);
}

/**
   This function detects if the robots way is blocked.
   This is done by looking for a sudden rise in the motor voltage
   combined with a sudden drop in velocity.
 */
void detect_blocking(blocktype *bt)
{
  double delta_pwmr, delta_pwml, delta_vel;
  int i;
  
  bt->cnt++;  // Increment number of calls to this function
  
  // Shift arrays
  for(i = FILTER_SIZE-1; i > 0; i--)
    {
      bt->vel_array[i] = bt->vel_array[i-1];
      bt->pwmr_array[i] = bt->pwmr_array[i-1];
      bt->pwml_array[i] = bt->pwml_array[i-1];
    }
  
  // Load present values to arrays
  bt->vel_array[0] = bt->velocity;
  bt->pwmr_array[0] = bt->pwmr;
  bt->pwml_array[0] = bt->pwml;
  
  // Shift arrays
  for(i = BUFFER_SIZE-1; i > 0; i--)
    {
      bt->filter_vel[i] = bt->filter_vel[i-1];
      bt->filter_pwmr[i] = bt->filter_pwmr[i-1];
      bt->filter_pwml[i] = bt->filter_pwml[i-1];
    }
  
  // The signals are filtered to supress the noise  
  bt->filter_vel[0] = filter(bt->vel_array);
  bt->filter_pwmr[0] = filter(bt->pwmr_array);
  bt->filter_pwml[0] = filter(bt->pwml_array);
  
  // The filtered signal is now checked for sudden changes if the arrays are filled (cnt == 10)
  if(bt->cnt == 10)
    {
      delta_pwmr = (bt->filter_pwmr[0] - bt->filter_pwmr[BUFFER_SIZE-1]) / 10;
      delta_pwml = (bt->filter_pwml[0] - bt->filter_pwml[BUFFER_SIZE-1]) / 10;
      delta_vel = (bt->filter_vel[0] - bt->filter_vel[BUFFER_SIZE-1]) * 20;
    
      if((delta_vel < -0.5) && (delta_pwmr > 0.5) && (delta_pwml > 0.5))
	bt->blocked = 1;
      else
	bt->blocked = 0;
      
      bt->cnt--;
    }
}

/**
   This function initializes the internal variables used in the
   block detection.
   Should be called before any use of detect_blocking !!
 */
void init_blocking(blocktype *bt)
{
  int i;  
  
  bt->cnt = 0;
  bt->blocked = 0;
  
  for(i = 0; i < FILTER_SIZE; i++)
    {
      bt->vel_array[i] = 0;
      bt->pwmr_array[i] = 0;
      bt->pwml_array[i] = 0;
    }
  
  for(i = 0; i < BUFFER_SIZE; i++)
    {
      bt->filter_vel[i] = 0;
      bt->filter_pwmr[i] = 0;
      bt->filter_pwml[i] = 0;
    }
}
