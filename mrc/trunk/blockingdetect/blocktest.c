/**
   This program is a testapplication to the blockdetection routines in blockdetect.c / blockdetect.h

   It must be executed with a filename as first argument. The file must be a logfile with $odovelocity in column 3,
   $motorpwml in column 5 and $motorpwmr in column 6.
*/

# include <stdio.h>
# include <stdlib.h>

/* !!!! */
# include "blockdetect.h"
/* !!!! */

int main(int argc, char **argv)
{
  /* !!!! */
  blocktype bt;
  /* !!!! */ 

  char *filename;
  char ch;
  FILE *datafile;
  int datapoints = 0;
  int blocked;
  int i;
  double *vel_array = NULL;
  double *pwmr_array = NULL;
  double *pwml_array = NULL;
  double tmpa, tmpb, tmpc;
  
  /* !!!! */
  init_blocking(&bt);   // Initialize internal variables in block detection
  /* !!!! */

  if(argc == 2)
    filename = argv[1];
  else
    {
      printf("Specify logfile as in:  %s file.log\n", argv[0]);
      exit(1);
    }
  
  
  if((datafile = fopen(filename, "r")) == NULL)    // Opens file (if possible)
    {
      printf("Unable to open file %s\n", filename);    
      exit(2);
    }
  
  while(1)               // Read data from file
    {
      datapoints++;
      
      // Reallocate arrays to fit the size of datapoints
      vel_array = (double *) realloc(vel_array, sizeof(double) * datapoints);
      pwmr_array = (double *) realloc(pwmr_array, sizeof(double) * datapoints);
      pwml_array = (double *) realloc(pwml_array, sizeof(double) * datapoints);
      
      ch = (char) fscanf(datafile, "%lf %lf %lf %lf %lf %lf", &tmpa, &tmpb, vel_array + datapoints-1, 
      			 &tmpc, pwml_array + datapoints-1, pwmr_array + datapoints-1);
      if(ch != 6)        // If the previous line doesn't read 6 values we have reached EOF
	{
	  datapoints--;
	  break;
	}
      while(fgetc(datafile) != '\n');             // Push filepointer to next line
    }
  
  fclose(datafile);
  
  for(i = 0; i < datapoints; i++)
    {

      /* !!!! */  // Must be placed inside main loop   !!!!!!

      // Read in velocity and pwm values to blocktype:
      bt.velocity = *(vel_array + i);
      bt.pwmr = *(pwmr_array + i);
      bt.pwml = *(pwml_array + i);    
      
      // Run block detecting:
      detect_blocking(&bt);
      
      // Interpret output:
      blocked = bt.blocked;
      /* !!!! */

      if(blocked)
	printf("Blocking detected at measurement: %d\n", i);
    }

  exit(0);
}
