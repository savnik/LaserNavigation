/** \mainpage
 * 
 * This library is a sensor fusion expansion to the Robot Control Program (SMRdemo)
 * 
 * The library is based on extended Kalman filtering, and inplemented using
 * the real time safe IAUmat matrix library developed at Ørsted*DTU, Automation.
 * 
 * The library contains both textbook linear kalman routines and customized
 * algorithms used for the actual implementation of the sensor fusion routines.
 * 
 * An example of how to use the filter is explained in test.c
 *
 * \author Lars Valdemar Mogensen
 * \date 30-07-2006
 */

/** \file filter.h
 * \brief Header file for IAUs library for sensor fusion purposes.
 * 
 * Definition of the structs used and the functions for the kalman 
 * filters.
 * Uses iau_mat for calculations.
 * 
 * \author Lars Valdemar Mogensen
 * \date 05/22-2006
 */

#ifndef IAU_FILTER_H
#define IAU_FILTER_H

#include <iau_mat.h>

/** \struct IAU_ekf
 * \brief Struct to hold the matrices needed for the Extended kalman filter.
 * 
 * The system is in discrete time.
 * \author Lars Valdemar Mogesen
 */
typedef struct{
   
   int flag; ///< Flag for internal use
   int NA;   ///< Number of states in the model
   int NX;   ///< Number of states in the model
   int NU;   ///< Number of control signals in the model
   int NY;   ///< Number of outputs from the model
   
   matrix *M1ay,*M2ay,*M1ya,*M2ya,*M1yy,*M2yy,*M1aa,*M2aa,*M3aa,*M1a1,*M2a1,*M3a1,*M1y1;
   
   matrix *F;  ///< Linearized System matrix
   matrix *G;  ///< Linearized Input matrix 
   matrix *C;  ///< Linearized Output matrix 
   matrix *D;  ///< Linearized Feed forward matrix
   matrix *Gv; ///< Noise input matrix
   matrix *v1; ///< State noise vector
   matrix *v2; ///< Measurement noise vector
   matrix *Y;  ///< Measurement vector
   matrix *U;  ///< Control signal vector
   matrix *Q;  ///< Error covariance matrix
   matrix *Qm; ///< Process noise covariance matrix
   matrix *X;  ///< State vector
   matrix *L;  ///< Kalman gain
   
   // For the open form kalman filter
   matrix *Qk;    ///< Error covariance matrix
   
   matrix *Xpre;  ///< State vector before measurement update
   matrix *Xpost; ///< State vector after measurement update
   
   matrix *Ppre;  ///< State covariance before measurement update
   matrix *Ppost; ///< State covariance after measurement update
   
   matrix *tempXU; ///< Temporary variable
   
} IAU_ekf;


/** \struct IAU_kalman_odotype
 * \brief Struct to hold the matrices needed the sensor fusion of odometry
 * and GPS measurement.
 * 
 * The system is in discrete time.
 * \author Lars Valdemar Mogesen
 */
typedef struct {
   
   IAU_ekf* kalman;              ///< Struct to hold the information for the extended kalman filter
   matrix* state;                ///< State matrix (x, y, theta)
   matrix* noise_input_matrix;   ///< Noise input matrix, G
   matrix* noise_input_matrix_T; ///< Transpose of the noise input matrix, G'
   matrix* noise_input_cov;      ///< Noise input covariance Qm
   matrix* process_noise;        ///< Process noise matrix Q
   matrix* measurement;          ///< Measurement noise matrix
   matrix* Fk;                   ///< Linearized system model df/dx
   matrix* FkT;                  ///< Transpose of linearized system model
   matrix* temp3x2;              ///< Temporary matrix variable
   matrix* temp2x1;              ///< Temporary matrix variable
   
   double steering_angle;  ///< Variable to hold the steer angle for the Ackerman steered vehicle
   double dist;            ///< Variable to hold the driven distance in one sample
   double robot_length;    ///< Variable to hold the length of the vehicle for the Ackerman steered vehicle
   double dth;             ///< Variable to hold the angle change for one sample for the differentially steered vehicle
   
   double offset_x; ///< Offset of GPS easting, to avoid numerical problems. Set as close to starting point as possible.
   double offset_y; ///< Offset of GPS northing, to avoid numerical problems. Set as close to starting point as possible.
   
   double range; ///< Internal variable to hold range from estimate to measurement.
   
   double measurement_noise_std_x; ///< Measurement noise standard deviation for the x direction
   double measurement_noise_std_y; ///< Measurement noise standard deviation for the y direction

   double process_noise_std_steering_angle; ///< Process noise standard deviation for the steer angle
   double process_noise_std_dist; ///< Process noise standard deviation for the driven distance

   double error_std_x; ///< Initial value for error covariance matrix.
   double error_std_y; ///< Initial value for error covariance matrix.
   double error_std_th; ///< Initial value for error covariance matrix.

   int startup_flag; ///< Flag used to indicate startup of the filter. Used to settle the filter.
   int correct_flag; ///< Flag used to indicate if the filter should measurement update.
   
   char type[20];    ///< Array to hold the name of the filter. Not used yet.
	
   int size_input;         ///< Number of inputs to the model
   int size_state;         ///< Number of states in the model
   int size_noise_process; ///< Number of uncorrelated process noise inputs
   int size_output;        ///< Number of outputs from the model
   int size_noise_output;  ///< Number of uncorrelated measurement noise inputs

   int config;   ///< Flag to indicate if the module is configured
   int status;   ///< Flag to indicate the status of the module
   int run;      ///< Flag to indicate if the module is to be updated
   int use;      ///< Flag to indicate if the date returned from the module is to be used in the following calculations.
} IAU_kalman_odotype;

// Prototypes for handling the filters
void IAU_kalman_odometry_init( IAU_kalman_odotype* odo );
void IAU_kalman_odometry_free( IAU_kalman_odotype* odo );

// Prototypes for the open form kalman filter
void IAU_kalman_open_ackerman_predict( IAU_kalman_odotype *odo );
void IAU_kalman_open_differential_predict( IAU_kalman_odotype *odo );
void IAU_kalman_open_correct( IAU_kalman_odotype *odo );

// Prototypes for the closed for kalman filter
void IAU_kalman_closed_form( IAU_kalman_odotype *odo );

#endif

