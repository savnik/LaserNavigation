# ifndef BLOCKDETECT_DOT_H
# define BLOCKDETECT_DOT_H


# define FILTER_SIZE 5     // Number of previous values used in filter
# define BUFFER_SIZE 10    // Number of previous values used to determine sudden changes

typedef struct
{
  // User input:
  double velocity;    // Robots velocity
  double pwmr;        // Right motor pwm
  double pwml;        // Left motor pwm
  
  // Internal variables:
  double vel_array[FILTER_SIZE];
  double pwmr_array[FILTER_SIZE];
  double pwml_array[FILTER_SIZE];
  double filter_vel[BUFFER_SIZE];
  double filter_pwmr[BUFFER_SIZE];
  double filter_pwml[BUFFER_SIZE];
  int cnt;   //  Internal counter, used to avoid giving any results before the internal arrays are filled
  
  // Output:
  int blocked;   // Output signal. 1 if blocked else 0.
  
} blocktype;

/**
   This function detects if the robots way is blocked.
   This is done by looking for a sudden rise in the motor voltage
   combined with a sudden drop in velocity.
 */
void detect_blocking(blocktype *bt);

/**
   This function initializes the internal variables used in the
   block detection.
   Should be called before any use of detect_blocking !!
 */
void init_blocking(blocktype *bt);

# endif // BLOCKDETECT_DOT_H
