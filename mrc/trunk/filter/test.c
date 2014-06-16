/** \file test.c
 * \brief Test file for sensor fusion purposes.
 * 
 * Simulation of an Ackerman steered mobile robot equipped
 * with GPS and odometry.
 * 
 * The file illustrates the use of the filter and can be 
 * used for debugging purposes if no robot simulator
 * is available.
 * 
 * Uses iau_mat for calculations.
 * 
 * \author Lars Valdemar Mogensen
 * \date 22/05-2006
 */

#include "filter.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/** \brief Offset for the GPS measurement. 
 * 
 * This is to test the filter when using very large numbers.
 */ 
#define GPS_OFFSET 1000.0
/// Steering angle definition for the ackerman vehicle.
#define STEERING_ANGLE (10.0 * M_PI/180.0)

#if(1)
/** \brief Driven distance in one sample. 
 * This translates into 0.5 m/s at a sample rate of 10Hz
 */
#define DIST 0.05
#endif

#if(0)
/** \brief Driven distance in one sample. 
 * This translates into 0.5m/s at a sample rate of 100Hz
 */ 
#define DIST 0.005
#endif

/** \brief Length of the Ackerman vehicle from rear axel to front axle.
 */
#define ROBOT_LENGTH 1.22

/** \brief Test the odomtric filters running on an Ackerman
 * vehicle.
 * 
 * The noise of the GPS and model states can be changed in the 
 * code below.
 */
int main(int argc, char** argv)
{
   int i=0;
   double x=0,y=0,th=0.0,dth,dv;
   IAU_kalman_odotype odo_closed;
   IAU_kalman_odotype odo_open;

   FILE *fp;
  dv=DIST;

   // Setup of the external variables for the kalman filter
   odo_closed.measurement_noise_std_x = 0.1;
   odo_closed.measurement_noise_std_y = 0.1;
   odo_open.measurement_noise_std_x = 0.1;
   odo_open.measurement_noise_std_y = 0.1;

//       odo.process_noise_std_steering_angle = 0.5 * M_PI/180.0;
//       odo.process_noise_std_dist = 0.05;
  
   odo_closed.process_noise_std_steering_angle = 0.01;//0.5 * M_PI/180.0;
   odo_closed.process_noise_std_dist = 0.02;//0.05;
   odo_open.process_noise_std_steering_angle = 0.02;//0.5 * M_PI/180.0;
   odo_open.process_noise_std_dist = 0.02;//0.05;
      
   odo_closed.error_std_x = 1000.0;
   odo_closed.error_std_y = 1000.0;
   odo_closed.error_std_th = 1;
   odo_open.error_std_x = 1000.0;
   odo_open.error_std_y = 1000.0;
   odo_open.error_std_th = 1;

   // Setup of the simulation specific variables
   odo_closed.range = 10.0; //the range of the gps, for filtering purpuse 
   odo_closed.offset_x = 1000.0;
   odo_closed.offset_y = 1000.0;
   odo_open.range = 10.0; //the range of the gps, for filtering purpuse 
   odo_open.offset_x = 1000.0;
   odo_open.offset_y = 1000.0;
      
   // Init two filters
   IAU_kalman_odometry_init(&odo_closed);
   IAU_kalman_odometry_init(&odo_open);
      
   // Setup of the trajectory the vehicle is to travel in the simulation (circle)
   odo_closed.dist = dv;
   odo_closed.steering_angle = STEERING_ANGLE;
   odo_closed.robot_length = ROBOT_LENGTH;
   odo_open.dist = dv;
   odo_open.steering_angle = STEERING_ANGLE+0.05;
   odo_open.robot_length = ROBOT_LENGTH;
      
   printf("\nStarting Loop \tClosed\t\tOpen\n");
   printf("dist       \t%lf\t\t%lf\n",odo_closed.dist,odo_open.dist);
   printf("STEERING ANGLE \t%lf\t\t%lf\n",odo_closed.steering_angle,odo_open.steering_angle);
   printf("robot_length \t%lf\t\t%lf\n",odo_closed.robot_length,odo_open.robot_length);
      
   fp=fopen("test.log","w");
   
   if(fp == NULL) {
      
      printf("Error opening file\n");
      return(1);
   }
      
   // Test loop for the IAU_ekf
   for(i=0;i<400;i++) {
      fprintf(stderr,"i> %i",i);
      if(i>200) {
         dv=0;     
      }
      if (i>300)
        dv=-0.05;
      dth=tan(STEERING_ANGLE)/ROBOT_LENGTH*dv;
      x+=dv*cos(th+dth/2);
      y+=dv*sin(th+dth/2);
      th+=dth;
      
     
      // Generate random measurement noise into the measurement matrix
       // Copy the measurement into the open form solution
      vput(odo_open.measurement,0,x+((rand()/(double)RAND_MAX)-0.5)*0.4+700000);
      vput(odo_open.measurement,1,y+((rand()/(double)RAND_MAX)-0.5)*0.4+6000000);
      odo_open.dist = dv;
       // Run the open form [Larsen, 1998] nonlinear update type kalman filter
      IAU_kalman_open_ackerman_predict(&odo_open);
      IAU_kalman_open_correct(&odo_open);
      #define PRINT_DEBUG    
      #ifdef PRINT_DEBUG
      printf("\n %4i> %16lf   %16lf    %16lf   True position\n",
             i,x,y,th);
      printf("%4i> %16lf   %16lf                       Closed loop measurement\n",
             i,vget(odo_open.measurement,0),vget(odo_open.measurement,1));
      printf("%4i> %16lf   %16lf    %16lf   Open Loop estimator\n",
             i,vget(odo_open.kalman->Xpost,0),vget(odo_open.kalman->Xpost,1),vget(odo_open.kalman->Xpost,2));
     #endif
      fprintf(fp,"%4i %16lf   %16lf    %16lf  %16lf %16lf %16lf \n",
             i,x,y,th,vget(odo_open.kalman->Xpost,0)-700000,vget(odo_open.kalman->Xpost,1)-6000000,vget(odo_open.kalman->Xpost,2));
 
   }
      
   printf("Q - Closed\n");
   mprint(odo_closed.kalman->Q);
   printf("Ppost - Open\n");
   mprint(odo_open.kalman->Ppost);
      
   fclose(fp);
      
   IAU_kalman_odometry_free(&odo_closed);
   IAU_kalman_odometry_free(&odo_open);

   return 0;
}
