/** \file filter.c
 * \brief Filter implementation for sensor fusion purposes.
 * 
 * Uses iau_mat for calculations.
 * 
 * References made are to books listed in the thesis:
 * Sensor Fusion for Mobile Robots, 2006, Elektro, Automation.
 * 
 * \author Lars Valdemar Mogensen
 *
 * \date 22/05-2006
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "filter.h"

void kalman_predict( IAU_kalman_odotype *odo );

/// Definition of inline function square of a double value.
inline double sqr(double value){ return value*value; }

/** \brief Correct the angle to be between [-pi;pi]
 * 
 * \param[in] angle Value to be corrected
 * \return Corrected value
 */
double correctAngle(double angle)
{
   if(angle > M_PI)
   {
//       fprintf(stderr,"correcting pos angle - %lf\n",angle);
      while(angle > M_PI)
      angle -= 2.0 * M_PI;
      
   }
   else if(angle < -M_PI)
   {
//       fprintf(stderr,"correcting neg angle - %lf\n",angle);
      while(angle < -M_PI)
      angle += 2.0 * M_PI;
   }

  return angle;
}

/** \brief Create the kalman filter struct
 * 
 * \param[in] Dstates Number of states in the filter
 * \param[in] Doutput Number of output from the model
 * \param[in] Dcontrol Number of control signals to the model
 * \return Returns a pointer to the initialized struct.
 */ 
IAU_ekf* IAU_create_ekf(int Dstates, int Doutput, int Dcontrol)
{
   IAU_ekf *tmp;
  
   tmp = (IAU_ekf *)malloc(sizeof(IAU_ekf));
  
   int NA = tmp->NA = Dstates; //getrows(A)
   int NY = tmp->NY = Doutput; //getrows(Y)
   int NU = tmp->NU = Dcontrol; //getrows(U)

   // Allocate memory for temporary variables
   tmp->M1ay = mmake(NA,NY);
   tmp->M2ay = mmake(NA,NY);
   tmp->M1ya = mmake(NY,NA);
   tmp->M2ya = mmake(NY,NA);
   tmp->M1yy = mmake(NY,NY);
   tmp->M2yy = mmake(NY,NY);
   tmp->M1aa = mmake(NA,NA);
   tmp->M2aa = mmake(NA,NA);
   tmp->M3aa = mmake(NA,NA);

   tmp->M1a1 = mmake(NA,1);
   tmp->M2a1 = mmake(NA,1);
   tmp->M3a1 = mmake(NA,1);
   tmp->M1y1 = mmake(NY,1);
  
   // Allocate memory for variables
   tmp->F  = mmake(NA,NA);
   tmp->C  = mmake(NY,NA);
  
   if(NU>0)
   {
      tmp->G  = mmake(NA,NU);
      tmp->D  = mmake(NY,NU);
      tmp->U  = mmake(NU,1);
   }
  
   tmp->Gv = mmake(NA,NA);
   tmp->v1 = mmake(NA,NA);
   tmp->v2 = mmake(NY,NY);
   tmp->Y  = mmake(NY,1);
   tmp->Q  = mmake(NA,NA);
   tmp->X  = mmake(NA,1);
   tmp->L  = mmake(NA,NY); 
  
   tmp->Qm = mmake(NU,NU);
   tmp->Qk = mmake(NA,NA);
  
   tmp->Xpre  = mmake(NA,1);
   tmp->Xpost  = mmake(NA,1);
  
   tmp->Ppre  = mmake(NA,NA);
   tmp->Ppost  = mmake(NA,NA);
    
   return tmp;
}

/** \brief Finalize and remove kalman filter struct
 * 
 * \param[in] *ekf Pointer to the kalman filter struct to remove.
 */ 
void IAU_free_ekf(IAU_ekf *ekf)
{
   // Freeing temporary variables
   mfree(ekf->M1ay);
   mfree(ekf->M2ay);
   mfree(ekf->M1ya);
   mfree(ekf->M2ya);
   mfree(ekf->M1yy);
   mfree(ekf->M2yy);
   mfree(ekf->M1aa);
   mfree(ekf->M2aa);
   mfree(ekf->M3aa);
  
   mfree(ekf->M1a1);
   mfree(ekf->M2a1);
   mfree(ekf->M3a1);
   mfree(ekf->M1y1);
  
   // Freeing system variables
   mfree(ekf->F);
   mfree(ekf->C);
  
   if(ekf->NU>1)
   {
      mfree(ekf->G);
      mfree(ekf->D);
      mfree(ekf->U);
   }
  
   mfree(ekf->Gv);
   mfree(ekf->v1);
   mfree(ekf->v2);
   mfree(ekf->Y);
   mfree(ekf->Q);
   mfree(ekf->X);
   mfree(ekf->L); 
  
   mfree(ekf->Qm);
   mfree(ekf->Qk);
  
   // Open form kalman filter
  
   mfree(ekf->Xpre);
   mfree(ekf->Xpost);
  
   mfree(ekf->Ppre);
   mfree(ekf->Ppost);
  
   // Freeing memory for the struct
   free(ekf);
}

/** \brief Create and initialize the kalman filter struct
 * 
 * \param[in] *odo Pointer to an odometry filter struct.
 */ 
void IAU_kalman_odometry_init( IAU_kalman_odotype* odo )
{
   // This implementation needs to be implemented in iau_mat
   
   // Setting up kalman filter to run at double precision
   // Automatically clearing the kalman variables
   odo->kalman = IAU_create_ekf( 3, 2, 2 );

   // Initialization of internal matrices, and set to zero
   odo->state = mmake(3,1);
   odo->noise_input_matrix = mmake(3,2); 
   odo->noise_input_matrix_T = mmake(2,3);
   odo->noise_input_cov =  mmake(2,2); 
   odo->process_noise =  mmake(3,3); 
   odo->measurement = mmake(2,1);

   // Linearized system
   odo->Fk = mmake(3,3);
   odo->FkT = mmake(3,3);

   odo->temp3x2 = mmake(3,2);
   odo->temp2x1 = mmake(2,1);

   double Fk_base[] = {1, 0, 0, 
                       0, 1, 0, 
                       0, 0, 1};
   
   double C[] = {1, 0, 0, 
                 0, 1, 0};
   
   // The D matrix is not initialized beacuse no direct path is there.
   double mes_noise_cov[] = { sqr(odo->measurement_noise_std_x), 0,
                              0, sqr(odo->measurement_noise_std_y)};

   // Process noise covariance
   double pro_noise_cov[] = {sqr(odo->process_noise_std_steering_angle), 0, 
                             0, sqr(odo->process_noise_std_dist) };

   double error_cov[] = {sqr(odo->error_std_x), 0.0, 0.0,
                         0.0, sqr(odo->error_std_y), 0.0,
                         0.0, 0.0, sqr(odo->error_std_th)};

   // Initialize state for debug function
   minit(odo->state);
       
   // Initialize system matrices
   array2mat(odo->kalman->C,C,getrows(odo->kalman->C)*getcols(odo->kalman->C));
   array2mat(odo->kalman->F,Fk_base,getrows(odo->kalman->F)*getcols(odo->kalman->F));
   
   array2mat(odo->Fk,Fk_base,getrows(odo->Fk)*getcols(odo->Fk));

   // Initialize the error covariance for the two different filters
   array2mat(odo->kalman->Q , error_cov, 9);
   array2mat(odo->kalman->Ppre , error_cov, 9);
   array2mat(odo->kalman->Ppost , error_cov, 9);
   
   // Initialize the measurement noise covariance
   array2mat(odo->kalman->v2, mes_noise_cov, 4);
   
   // Initialize the noise input covariance
   array2mat(odo->noise_input_cov, pro_noise_cov,4);

   // Initialize the measurement noise covariance
   array2mat(odo->kalman->Qm, pro_noise_cov,4);   

   // Initialize the open filter to start away from origo
   if(1){
//       fprintf(stderr,"Filter estimate initialized -------------------------------------------------------------------\n");
      double ang = 0.0/180.0*M_PI;
      double eta = 0.001;
      vput(odo->kalman->Xpre,0,eta);
      vput(odo->kalman->Xpre,1,eta);
      vput(odo->kalman->Xpre,2,ang);
      
      vput(odo->kalman->Xpost,0,eta);
      vput(odo->kalman->Xpost,1,eta);
      vput(odo->kalman->Xpost,2,ang);
   }
   else
      ;
//       fprintf(stderr,"Filter estimate initialized to ZERO-------------------------------------------------------------------\n");
   // Initialize the closed filter to start away from origo, for testing pruposes
   //    vput(odo->kalman->X,0,-1);
   //    vput(odo->kalman->X,1,1);

   odo->steering_angle = 0.0;
   odo->dist = 0.0;
   odo->startup_flag = 10;
   odo->correct_flag = 0;
   odo->status = 0;
}

/** \brief Finalize and remove odometry filter struct
 * 
 * \param[in] *odo Pointer to the odometry filter struct to remove.
 */ 
void IAU_kalman_odometry_free( IAU_kalman_odotype* odo )
{
   IAU_free_ekf(odo->kalman);

   mfree(odo->state);
   mfree(odo->noise_input_matrix);
   mfree(odo->noise_input_matrix_T);
   mfree(odo->noise_input_cov);
   mfree(odo->process_noise);
   mfree(odo->measurement);
   mfree(odo->Fk);
   mfree(odo->FkT);
   mfree(odo->temp3x2);
   mfree(odo->temp2x1);
}   

// This is for the open form representation.
// Finish after getting the closed form prototype to work

// For the open form kalman filter  
/** \brief Text book kalman filter prediction.
 * 
 * \note This function uses the linearized model model.
 * \param[in] *ekf Pointer to the kalman filter to predict.
 */ 
void ekf_predict(IAU_ekf *ekf)
{
   // Calculate the state update, Xpre(k) [Hendriks et al, 2003, p. 500]
   /// Calculate Xpre(k) = F(k-1)*Xpost(k-1) + G(k-1)*U(k-1)
    
   mmul(ekf->M1a1,ekf->G,ekf->U);               // G(k-1)*U(k-1)
   mmul(ekf->M2a1,ekf->F,ekf->Xpost);           // F(k-1)*Xpost(k-1)
   madd(ekf->Xpre,ekf->M2a1,ekf->M1a1);         // Xpre(k)

   // Calculate the error covariance, Ppre(k) [Hendriks et al, 2003, p. 500]
   /// Calculate Ppre(k) = Fk(k-1)*Ppost(k-1)*Fk(k-1)' + Gk(k-1)*V1*Gk(k-1)'
   
   mtrans(ekf->M3aa,ekf->F);                    // Fk(k-1)'
   mmul(ekf->M1aa,ekf->F,ekf->Ppost);           // Fk(k-1)*Ppost(k-1)
   mmul(ekf->M2aa,ekf->M1aa,ekf->M3aa);         // Fk(k-1)*Ppost(k-1)*Fk(k-1)'
   
   mtrans(ekf->M1ya,ekf->G);                    // Gk(k-1)'
   mmul(ekf->M2ya,ekf->Qm,ekf->M1ya);           // V1*Gk(k-1)' NOTE Qm is process noise covariance
   mmul(ekf->M3aa,ekf->G,ekf->M2ya);            // Gk(k-1)*V1*Gk(k-1)'   

   madd(ekf->Ppre,ekf->M2aa,ekf->M3aa);         // Ppre(k)

}


// For the open form kalman update
/** \brief Text book kalman filter correction.
 * 
 * \note This function uses the linearized model model.
 * \param[in] *ekf Pointer to the kalman filter to correct.
 * \param[in] *meas Pointer to the measurement vector.
 */ 
void ekf_correct(IAU_ekf *ekf, matrix *meas)
{   
    // Calculate Kk, Larsen 1998, p. 45, eqn. 3.10
    /// Calculate L(k)=P(k)*C(k)'*inv(C((k)*P(k)*C(k)' + Q(k));
    
   mtrans(ekf->M1ay,ekf->C);                 //  C'
   mmul(ekf->M1ya,ekf->C,ekf->Ppre);         //  C*Ppre
   mmul(ekf->M1yy,ekf->M1ya,ekf->M1ay);      //  C*Ppre*C'
   madd(ekf->M1yy,ekf->M1yy,ekf->v2);        //  C*Ppre*C' + v2

   minvgauss(ekf->M2yy,ekf->M1yy);           //  inv(C*Ppre*C'+v2)
 
   mmul(ekf->M2ay,ekf->Ppre,ekf->M1ay);      //  Ppre*C'
   mmul(ekf->L,ekf->M2ay,ekf->M2yy);         //  L
   
   // Calculate Xpost, Larsen 1998, p. 45, eqn. 3.11
   /// Calculate Xpost(k) = Xpre(k) + L(k)*(Y(k) - C(k)*Xpre(k))
   
   mmul(ekf->M1y1,ekf->C,ekf->Xpre);         // C*X
   msub(ekf->M1y1,ekf->Y,ekf->M1y1);         // Y-C*X

   mmul(ekf->M3a1,ekf->L,ekf->M1y1);         // L*(Y-C*X)
   
   madd(ekf->Xpost,ekf->Xpre,ekf->M3a1);     // Xpost
   
   // Calculate Ppost, Larsen 1998, p. 45, eqn. 3.12
   /// Calculate Ppost(k) = (I- L(k)*C(k))*Ppre(k)
   
   mmul(ekf->M1aa,ekf->L,ekf->C);            // L(k)*C(K)
   munit(ekf->M2aa);                         // I
   msub(ekf->M3aa,ekf->M2aa,ekf->M1aa);      // I - L(k)*C(K)
   mmul(ekf->Ppost,ekf->M3aa,ekf->Ppre);     // Ppost(k)
   
}

/** \brief Closed form kalman filter update.
 * 
 * Calculate kalman filter update.
 * 
 * Discrete time system description:\n
   \verbatim
   X(k+1) = Fk*X(k) + Gk*U(k) + Gm(k)*v1(k)
   Y(k+1) = C*X(k) + I*v2(k) \endverbatim
 *
 * \param *ekf Pointer to the filter struct to update.
 * \param[in] *meas Pointer to the measurement vector.
 */
void ekf_update(IAU_ekf *ekf, matrix *meas)
{
   int i=0,j=0;
   
   // Calculate the kalman gain
   /// Calculate L(k) = F(k)*Q(k)*C(k)'*inv(C(k)*Q(k)*C(k)'+v2)

   mtrans(ekf->M1ay,ekf->C);               // C'
   mmul(ekf->M1ya,ekf->C,ekf->Q);          // C*Q
   mmul(ekf->M1yy,ekf->M1ya,ekf->M1ay);    // C*Q*C'
   madd(ekf->M1yy,ekf->M1yy,ekf->v2);      // C*Q*C' + v2
   
   minvgauss(ekf->M2yy,ekf->M1yy);         // inv(C*Q*C'+v2)
 
   mmul(ekf->M1aa,ekf->F,ekf->Q);          // F*Q
   mmul(ekf->M2ay,ekf->M1aa,ekf->M1ay);    // F*Q*C'
   mmul(ekf->L,ekf->M2ay,ekf->M2yy);       // L
   
   // Calculate the state update
   /// Calculate X(k+1) = F(k)*X(k) + G*U(k) + L(k)*(Y(k)-C*X(k))
   mmul(ekf->M1y1,ekf->C,ekf->X);          // C*X
   msub(ekf->M1y1,ekf->Y,ekf->M1y1);       // Y-C*X

   mmul(ekf->M1a1,ekf->F,ekf->X);          // F*X 
   mmul(ekf->M3a1,ekf->L,ekf->M1y1);       // L*(Y-C*X)

   if(ekf->NU != 0)
   {
      mmul(ekf->M2a1,ekf->G,ekf->U);       // G*U
      madd(ekf->M1a1,ekf->M1a1,ekf->M2a1); // F*X + G*U
   }
   
   madd(ekf->X,ekf->M1a1,ekf->M3a1);       // F*X + G*U + L*(Y-C*X)

   // Calculate the error covarinace update
   /// Calculate Q(k+1) = (F(k)-L(k)*C(k))*Q(k)*F' + V1

   mmul(ekf->M1aa,ekf->L,ekf->C);          // L*C
   msub(ekf->M1aa,ekf->F,ekf->M1aa);       // F-L*C
   mmul(ekf->M2aa,ekf->M1aa,ekf->Q);       // (F-L*C)*Q
   mtrans(ekf->M3aa,ekf->F);               // F'
   mmul(ekf->M1aa,ekf->M2aa,ekf->M3aa);    // (F-L*C)*Q*F'
   madd(ekf->Q,ekf->M1aa,ekf->v1);         // (F-L*C)*Q*F' + v1

   // make  Q symmetric
   for (i=0; i<ekf->NA; i++)
   {
      for (j=i+1; j<ekf->NA; j++)
         ekf->Q->mat[i][j] = ekf->Q->mat[j][i];
   }
  
}

/** \brief Closed form update for an Ackerman steered vehicle
 * odometric vehicle.
 * 
 * \param[in] *odo Pointer to odometric filter struct.
 */ 
void IAU_kalman_closed_ackerman_update( IAU_kalman_odotype *odo )
{
   double mes_temp[2] = {0};
   double dth=0.0;
   double dv=0.0;

   dv = odo->dist;
   dth = dv/odo->robot_length * tan( odo->steering_angle ) ;
   
   // Correction of the real world update

   vput(odo->state, 0, vget(odo->state,0) + dv*cos(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 1, vget(odo->state,1) + dv*sin(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 2, correctAngle(vget(odo->state,2)+dth));
   
   // Update of the time varying matrices in the extended kalman filter
   // The reason for doing it here is the testing of the new filter
   // This is for the closed form update
   
   // Calculation of temporary variables for:
   // * Kalman filter pre update
   // * Time update of linearized system matrix F(k)
   
   mes_temp[0] = dv * cos(vget(odo->kalman->X,2)+dth/2);
   mes_temp[1] = dv * sin(vget(odo->kalman->X,2)+dth/2);

   // Calculation of the linearized system F(k), the time dependent part
   mput(odo->Fk,0,2, -mes_temp[1]);
   mput(odo->Fk,1,2,  mes_temp[0]);
   mtrans(odo->FkT, odo->Fk);

   // Calculate the linearized input matrix U(k), the time dependent part
   // Calculate the noise input matrix G(k) from kinematic model, like eqn. 3.25
   
   // This is the simplified version of the calculation, G(k)
   /*   minit(odo->noise_input_matrix);
   mput(odo->noise_input_matrix,0,0, cos( vget(odo->kalman->X,2 )));
   mput(odo->noise_input_matrix,1,0, sin( vget(odo->kalman->X,2 )));
   mput(odo->noise_input_matrix,2,0, tan( odo->steering_angle )/ odo->robot_length);
   mput(odo->noise_input_matrix,2,1, dv/( odo->robot_length * \
   (1 + cos(odo->steering_angle)*cos(odo->steering_angle))));
   mtrans( odo->noise_input_matrix_T, odo->noise_input_matrix );*/
   
   // This is the complete version of the calculation G(k)
   {
      double th=vget(odo->kalman->X,2 ), fi=odo->steering_angle, L=odo->robot_length;
   
      // Calculate the noise input matrix G(k) from kinematic model, like eqn. 3.25
      minit(odo->noise_input_matrix);
   
      mput(odo->noise_input_matrix,0,0, cos( th + dth/2 ));
      mput(odo->noise_input_matrix,1,0, sin( th + dth/2 ));
      mput(odo->noise_input_matrix,2,0, tan( fi )/ L);
   
      mput(odo->noise_input_matrix,0,1, -sqr(dv)*sin(th+dth/2)/(2*sqr(cos(fi))*L));
      mput(odo->noise_input_matrix,1,1,  sqr(dv)*cos(th+dth/2)/(2*sqr(cos(fi))*L));
      mput(odo->noise_input_matrix,2,1,  dv/(L * (1 + sqr(cos(fi)))));
   
      mtrans( odo->noise_input_matrix_T, odo->noise_input_matrix );
   
   }
   
   //Calculate the covariance matrix Q(k), eqn. 3.24
   mmul(odo->temp3x2, odo->noise_input_matrix, odo->noise_input_cov);	
   mmul(odo->process_noise, odo->temp3x2, odo->noise_input_matrix_T);

   // Update the efk struct with gps measurement Y(k) = gps measurement
   mset(odo->kalman->Y,odo->measurement);

   // Update the internal F(k)
   mset(odo->kalman->F,odo->Fk);
   
   // Update the ekf struct with the time updated process noise covariance matrix
   mset(odo->kalman->v1,odo->process_noise);
   
   // G(k) is calculated in the same way as the noise input matrix df(x,u,w) / du
   mset(odo->kalman->G,odo->noise_input_matrix);
   
   // U(k) is updated (constant, so not needed)
   vput(odo->kalman->U,0,dv);
   vput(odo->kalman->U,1,odo->steering_angle);
   
   // Calculate the kalman-correction
   if(odo->correct_flag == 1)
   {
      ekf_update(odo->kalman,odo->measurement);
      vput(odo->kalman->X,2, correctAngle( vget( odo->kalman->X,2 ) ) );
      odo->status = 0;
      if(odo->startup_flag)
         odo->startup_flag--;
   }
   else 
   {
      // This is for getting the kalman filter to converge to 
      // the wanted solution in the first sample
      vput(odo->kalman->X,0, vget(odo->state,0));// + odo->offset_x);
      vput(odo->kalman->X,1, vget(odo->state,1));// + odo->offset_y);
      vput(odo->kalman->X,2, correctAngle( vget(odo->state,2)));
   }
   
   odo->correct_flag = 1;
}

/** \brief Open form filter prediction for an Ackerman steered
 * vehicle.
 *  
 * Prediction of the filter. This is the first step in the two
 * stage open from kalman filter update.
 * \note This is a prediction based on a nonlinear model.
 * \param[in] *odo Pointer to the odometric filter to be predicted.
 */ 
void IAU_kalman_open_ackerman_predict(IAU_kalman_odotype* odo)
{
   double mes_temp[2] = {0};
   double dth=0.0;
   double dv=0.0;

   // Ackerman dependent odometry
   dv = odo->dist;
   dth = dv/odo->robot_length * tan( odo->steering_angle ) ;
   dth=odo->dth; // NAA 19-8-2009
   
   // Correction of the real world update
   vput(odo->state, 0, vget(odo->state,0) + dv*cos(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 1, vget(odo->state,1) + dv*sin(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 2, correctAngle(vget(odo->state,2)+dth));
   
   // Calculation of the kalman prediction, for the open form kalman filter
   
   // Update of the time varying matrices in the extended kalman filter
   // The reason for doing it here is the testing of the new filter
   // This is for the open form update
   
   // Calculation of temporary variables for:
   // * Kalman filter pre update
   // * Time update of linearized system matrix F(k)
   
   mes_temp[0] = dv * cos(vget(odo->kalman->Xpost,2)+dth/2.0);
   mes_temp[1] = dv * sin(vget(odo->kalman->Xpost,2)+dth/2.0);


   // Kalman filter pre correction update, for the open form kalman filter (non-linear)
   vput(odo->kalman->Xpre,0, vget(odo->kalman->Xpost,0) + mes_temp[0]);
   vput(odo->kalman->Xpre,1, vget(odo->kalman->Xpost,1) + mes_temp[1]);
   vput(odo->kalman->Xpre,2, vget(odo->kalman->Xpost,2) + dth); 

   //Calculate Error covariance P, eqn. 3.9, Open form kalman filter  
   // P(k) = Fk(k-1)*P(k-1)*Fk(k-1)' + Q(k-1)
   mmul(odo->kalman->Ppre,odo->kalman->Ppost,odo->FkT);
   mmul(odo->kalman->M1aa,odo->Fk,odo->kalman->Ppre);
   madd(odo->kalman->Ppre,odo->kalman->M1aa,odo->process_noise);
   
   // Calculation of the linearized system F(k), the time dependent part
   mput(odo->Fk,0,2, -mes_temp[1]);
   mput(odo->Fk,1,2,  mes_temp[0]);
   mtrans(odo->FkT, odo->Fk);

   // Calculate the linearized input matrix U(k), the time dependent part
   // df(x,u,v1)/du, not used since nonlinear update is deployed
   
   // Calculate the noise input matrix G(k) from kinematic model, like eqn. 3.25
   minit(odo->noise_input_matrix);
   mput(odo->noise_input_matrix,0,0, cos( vget(odo->kalman->Xpre,2 )));
   mput(odo->noise_input_matrix,1,0, sin( vget(odo->kalman->Xpre,2 )));
   mput(odo->noise_input_matrix,2,0, tan( odo->steering_angle )/ odo->robot_length);
   mput(odo->noise_input_matrix,2,1, dv/( odo->robot_length *  
         (1 + cos(odo->steering_angle)*cos(odo->steering_angle))));
   mtrans( odo->noise_input_matrix_T, odo->noise_input_matrix );


   //Calculate the covariance matrix Q(k), eqn. 3.24  Gv(k)*Q(k)*Gv'
   mmul(odo->temp3x2, odo->noise_input_matrix, odo->noise_input_cov);	
   mmul(odo->process_noise, odo->temp3x2, odo->noise_input_matrix_T);

   // Update the efk struct with gps measurement Y(k) = gps measurement
   mset(odo->kalman->Y,odo->measurement);
   
   // Update the ekf struct with the time updated process noise covariance matrix
   mset(odo->kalman->Qk,odo->process_noise);
}

/** \brief Open form filter prediction for a differential steered
 * vehicle.
 *  
 * Prediction of the filter. This is the first step in the two
 * stage open from kalman filter update.
 * \param[in] *odo Pointer to the odometric filter to be predicted.
 */ 
void IAU_kalman_open_differential_predict(IAU_kalman_odotype* odo)
{
   double mes_temp[2] = {0};
   double dth=0.0;
   double dv=0.0;

   // Differential odometry dependent
   dv = odo->dist;
   dth = odo->dth;

   
   // Correction of the real world update

   vput(odo->state, 0, vget(odo->state,0) + dv*cos(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 1, vget(odo->state,1) + dv*sin(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 2, correctAngle(vget(odo->state,2)+dth));
   
   // Calculation of the kalman prediction, for the open form kalman filter
   
   // Update of the time varying matrices in the extended kalman filter
   // The reason for doing it here is the testing of the new filter
   // This is for the open form update
   
   // Calculation of temporary variables for:
   // * Kalman filter pre update
   // * Time update of linearized system matrix F(k)
   
   mes_temp[0] = dv * cos(vget(odo->kalman->Xpost,2)+dth/2.0);
   mes_temp[1] = dv * sin(vget(odo->kalman->Xpost,2)+dth/2.0);


   // Kalman filter pre correction update, for the open form kalman filter (non-linear)
   vput(odo->kalman->Xpre,0, vget(odo->kalman->Xpost,0) + mes_temp[0]);
   vput(odo->kalman->Xpre,1, vget(odo->kalman->Xpost,1) + mes_temp[1]);
   vput(odo->kalman->Xpre,2, correctAngle( vget(odo->kalman->Xpost,2) + dth )); 

   // Calculate Error covariance P, eqn. 3.9, Open form kalman filter  
   // P(k) = Fk(k-1)*P(k-1)*Fk(k-1)' + Q(k-1)
   mmul(odo->kalman->Ppre,odo->kalman->Ppost,odo->FkT);
   mmul(odo->kalman->M1aa,odo->Fk,odo->kalman->Ppre);
   madd(odo->kalman->Ppre,odo->kalman->M1aa,odo->process_noise);
   
   // Calculation of the linearized system F(k), the time dependent part
   mput(odo->Fk,0,2, -mes_temp[1]);
   mput(odo->Fk,1,2,  mes_temp[0]);
   mtrans(odo->FkT, odo->Fk);

   // Calculate the linearized input matrix U(k), the time dependent part
   // df(x,u,v1)/du, not used since nonlinear update is deployed
   
   // Calculate the noise input matrix G(k) from kinematic model, like eqn. 3.25
   minit(odo->noise_input_matrix);
   mput(odo->noise_input_matrix,0,0, cos( vget(odo->kalman->Xpre,2 )));
   mput(odo->noise_input_matrix,1,0, sin( vget(odo->kalman->Xpre,2 )));
   mput(odo->noise_input_matrix,2,1, 1);
   mtrans( odo->noise_input_matrix_T, odo->noise_input_matrix );


   //Calculate the covariance matrix Q(k), eqn. 3.24  Gv(k)*Q(k)*Gv'
   mmul(odo->temp3x2, odo->noise_input_matrix, odo->noise_input_cov);	
   mmul(odo->process_noise, odo->temp3x2, odo->noise_input_matrix_T);

   // Update the efk struct with gps measurement Y(k) = gps measurement
   mset(odo->kalman->Y,odo->measurement);
   
   // Update the ekf struct with the time updated process noise covariance matrix
   mset(odo->kalman->Qk,odo->process_noise);
}

/** \brief Open form filter correction for an odometric filter.
 *  
 * Correction of the filter. This is the last step in the two
 * stage open from kalman filter update.
 * \param[in] *odo Pointer to the odometric filter to be corrected.
 */ 
void IAU_kalman_open_correct(IAU_kalman_odotype* odo)
{   
   // Calculate the kalman-correction
   if(odo->correct_flag == 1)
   {
      ekf_correct(odo->kalman,odo->measurement);
      vput(odo->kalman->Xpost,2,correctAngle( vget(odo->kalman->Xpost,2 )));
      odo->status = 0;
      if(odo->startup_flag)
         odo->startup_flag--;
   }
   else 
   {
      // This is for getting the kalman filter to converge 
      // to the wanted solution in the first sample
      vput(odo->kalman->Xpost,0, vget(odo->kalman->Xpre,0));
      vput(odo->kalman->Xpost,1, vget(odo->kalman->Xpre,1));
      vput(odo->kalman->Xpost,2, correctAngle(vget(odo->kalman->Xpre,2)));
      
      fprintf(stderr,"Corrected kalman filter odo->correct_flag = %i\n",odo->correct_flag);
      fprintf(stderr,"Xpost(0) %lf Xpost(1) %lf Xpost(2) %lf\n", \
            vget(odo->kalman->Xpost,0),vget(odo->kalman->Xpost,1),vget(odo->kalman->Xpost,2));
   }
   
   odo->correct_flag = 1;
}

/** \brief Open form filter prediction for an Ackerman steered
 * vehicle.
 *  
 * Prediction of the filter. This is the first step in the two
 * stage open from kalman filter update.
 * \note This is a prediction based on a linear model.
 * \param[in] *odo Pointer to the odometric filter to be predicted.
 */ 
void kalman_open_lin(IAU_kalman_odotype* odo)
{
   double mes_temp[2] = {0};
   double dth=0.0;
   double dv=0.0;

   dv = odo->dist;
   dth = dv/odo->robot_length * tan( odo->steering_angle ) ;
   
   // Correction of the real world update
   vput(odo->state, 0, vget(odo->state,0) + dv*cos(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 1, vget(odo->state,1) + dv*sin(vget(odo->state,2)+dth/2.0));
   vput(odo->state, 2, correctAngle(vget(odo->state,2)+dth));
   
   // Calculation of the kalman prediction, for the open form kalman filter
   
   // Update of the time varying matrices in the extended kalman filter
   // The reason for doing it here is the testing of the new filter
   // This is for the open form update
   
   // Calculation of temporary variables for:
   // * Kalman filter pre update
   // * Time update of linearized system matrix Fk, Gk
   
   mes_temp[0] = dv * cos(vget(odo->kalman->Xpost,2)+dth/2.0);
   mes_temp[1] = dv * sin(vget(odo->kalman->Xpost,2)+dth/2.0);

   // Kalman filter prediction, Linear case
   ekf_predict(odo->kalman);
   vput(odo->kalman->Xpre,2,correctAngle( vget(odo->kalman->Xpre,2 )));
   
   // Update U(k)
   vput(odo->kalman->U,0,dv);
   vput(odo->kalman->U,1,odo->steering_angle);
   
   // Calculation of the linearized system F(k), the time dependent part
   mput(odo->Fk,0,2, -mes_temp[1]);
   mput(odo->Fk,1,2,  mes_temp[0]);
   mset(odo->kalman->F,odo->Fk);

   // Calculate the noise input matrix G(k) from kinematic model, like eqn. 3.25
   // df(x,u,v1)/du, not used since nonlinear update is deployed
   minit(odo->noise_input_matrix);
   mput(odo->noise_input_matrix,0,0, cos( vget(odo->kalman->Xpre,2 )));
   mput(odo->noise_input_matrix,1,0, sin( vget(odo->kalman->Xpre,2 )));
   mput(odo->noise_input_matrix,2,0, tan( odo->steering_angle )/ odo->robot_length);
   mput(odo->noise_input_matrix,2,1, dv/( odo->robot_length * \
         (1 + cos(odo->steering_angle)*cos(odo->steering_angle))));
   mset(odo->kalman->G,odo->noise_input_matrix);
   
   // Update the efk struct with gps measurement Y(k) = gps measurement
   mset(odo->kalman->Y,odo->measurement);
   
   // Calculate the kalman-correction
   if(odo->correct_flag == 1)
   {
      ekf_correct(odo->kalman,odo->measurement);
      vput(odo->kalman->Xpost,2,correctAngle( vget(odo->kalman->Xpost,2 )));
   }
   else 
   {
      // This is for getting the kalman filter to converge to the wanted solution in the first sample
      vput(odo->kalman->Xpost,0, vget(odo->kalman->Xpre,0));
      vput(odo->kalman->Xpost,1, vget(odo->kalman->Xpre,1));
      vput(odo->kalman->Xpost,2, vget(odo->kalman->Xpre,2));
   }
   
   odo->correct_flag = 1;
}

/** \brief Call rutine for the closed form kalman filter for
 * the Ackerman steered HAKO vehicle.
 *  
 * This function sets boundaries to when the filter correction
 * is to be carried out.
 * 
 * \param[in] *odo Pointer to the odometric filter to be updated.
 */ 
void IAU_kalman_closed_form(IAU_kalman_odotype* odo)
{
   int code=0;
   
   double dist2offset = sqrt(sqr(vget(odo->measurement,0) - odo->offset_x) +  
            sqr(vget(odo->measurement,1) - odo->offset_y));
//    printf("dist2offset: %lf\n",dist2offset);
    
   double dist2kalman = sqrt( sqr(vget(odo->measurement,0) - 
            vget(odo->kalman->Xpost,0)) + sqr(vget(odo->measurement,1) - 
            vget(odo->kalman->Xpost,1)) );
//     printf("dist2kalman: %lf\n",dist2kalman);

   if( odo->correct_flag == 0 ) //correct is off from klient side
   {
      odo->correct_flag = 0;
      code = 1;
   }
   else if( dist2offset > odo->range )//offset is too far away
   {
      odo->correct_flag = 0;
      code = 2;
   }
   //measurement is to far away from predicted position
   else if( dist2kalman > 10.0* odo->measurement_noise_std_x && odo->startup_flag == 0)
   {
      odo->correct_flag = 0;
      code = 3;
   }
   if(odo->correct_flag == 0)
   {
      fprintf(stderr, "Kalman_odometry: Absolute measurement discarded, error code = %i\n", code);
      odo->correct_flag = 1;
   }
   
   // Closed form linear Ackerman update
   IAU_kalman_closed_ackerman_update(odo);
}
