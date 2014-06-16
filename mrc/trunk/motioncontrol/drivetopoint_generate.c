#define _USE_MATH_DEFINES
#include <math.h>
#include "drivetopoint.h"
int dtp_sign(double x);
double modangle(double alpha);
double get2picomplementaryangle(double alpha);
void displacecoordinatesystem(double * x, double * y, double dx, double dy);
void rotatecoordinatesystem(double * x, double * y, double dtheta);
void drivetopoint_generate_rel(double xf, double yf, double theta, double r, dtp_path arrpath[]);

/**************************************************************
 *
 *	@file		drivetopoint_generate.c
 *
 *	@version	1.0
 *
 * 	@author		Johan Musaeus Bruun, s052655
 * 	@author		Olivier Corradi, s052516
 *
 *	@updated	June 20th 2007
 *
 *	@project:	course 01666: Fagprojekt
 *
 * 	@description:
 *				Implementation of the drivetopoint algorithm
 *
 ***************************************************************/

void drivetopoint_generate(double xa, double ya, double thetaa, double xb, double yb, double thetab, double r, dtp_path arrpath[]) {

	// Declare variables
	double xdiff, ydiff, thetadiff;
	double xf, yf, theta;

	// Assign end values
	xf = xb;
	yf = yb;

	// Compute displacement
	xdiff = xa;
	ydiff = ya;
    
	// Compute rotation
	thetadiff = thetaa;

	// Calculate changed coordinates
	displacecoordinatesystem(&xf,&yf,xdiff,ydiff);
	rotatecoordinatesystem(&xf,&yf,thetadiff);

	// Calculate changed orientation
	theta = thetab - thetaa;

	// Call the relative DriveToPoint method using the changed coordinates
	drivetopoint_generate_rel(xf, yf, theta, r, arrpath);
}

void drivetopoint_generate_rel(double xf, double yf, double theta, double r, dtp_path arrpath[]) {

	// Constants
	const double tol = 0.000001; // 0.001 mm or 0.00006 degrees

	// Variables
	double C[4], L[4];
	double beta1, l, beta3;
	int dir1, dir2, dir3, rot1, rot3, relcirclepos1, relcirclepos3;
	double omega1short, omega1long, omega3short, omega3long, omega1, omega3, phi, rho, psi, eta, d;
	int i, j, k, n, m, solutionindex;

	// Set every solution in the array to be invalid
	for (i=0; i<DTP_MAX_NUM_SOLUTIONS; i++)
		arrpath[i].D = -1; // Mark all paths as invalid (D == -1)

	// Check for r < tol
	if (r < tol) {

		beta1 = atan2(yf,xf);
		dir1 = 1;
		l = sqrt(pow(xf,2)+pow(yf,2));
		dir2 = 1;
		beta3 = theta - beta1;
		dir3 = 1;

		arrpath[0].beta1 = beta1;
		arrpath[0].dir1 = dir1;
		arrpath[0].l = l;
		arrpath[0].dir2 = dir2;
		arrpath[0].beta3 = beta3;
		arrpath[0].dir3 = dir3;
		arrpath[0].D = 0;

	} else {
		solutionindex = 0;

		// Loop through startcicles
		for (i=0; i<2; i++) {
            
			// Compute circle i's centre
			C[0] = 0; // C1x
			C[1] = r*pow(-1,i); // C1y

			// Loop through endcicles
			for (j=0; j<2; j++) {
                
				// Compute circle j's centre
				C[2] = xf - pow(-1,j) * r*sin(theta); // C2x
				C[3] = yf + pow(-1,j) * r*cos(theta); // C2y

				// d: distance between chosen circle centres
				d = sqrt( pow(C[2]-C[0],2) + pow(C[3]-C[1],2) );

				// phi: angle between chosen circle centres
				phi = atan2( C[3]-C[1], C[2]-C[0] );

				// Loop through tangents
				for (k=0; k<4; k++) {
					// Compute tangent k
					if (k < 2) {
						// Non-crossing tangent
                        
                        if (i != j){
                            continue; // Discard invalid path
                        }
                        
						if (d > tol) {
							// Non-crossing tangent with distincts circles
							L[0] = C[0]-pow(-1,k)*r*sin(phi); // L1x
							L[1] = C[1]+pow(-1,k)*r*cos(phi); // L1y
							L[2] = C[2]-pow(-1,k)*r*sin(phi); // L2x
							L[3] = C[3]+pow(-1,k)*r*cos(phi); // L2y
						} else if (k == 0) {
							// Non-crossing tangent with egocentric circles (First tangent)
							L[0] = 0;
							L[1] = 0;
							L[2] = 0;
							L[3] = 0;
						} else {
							// Non-crossing tangent with egocentric circles (Second tangent)
							// We dicard the sencond tangent for egocentric circles
							// as it is the same as the first one
							continue;
						}
					} else {
						// Crossing tangent
                        
                        if (i == j){
                            continue; // Discard invalid path
                        }
                        
						if (d >= 2*r + tol) {
							// Crossing tangent with distincts circles
							rho = acos(2*r / d); // rho: angle correction for crossing tangents
							L[0] = C[0]+r*cos(phi+pow(-1,k)*rho); // L1x
							L[1] = C[1]+r*sin(phi+pow(-1,k)*rho); // L1y
							L[2] = C[2]-r*cos(phi+pow(-1,k)*rho); // L2x
							L[3] = C[3]-r*sin(phi+pow(-1,k)*rho); // L2y
						} else if (fabs(d - 2*r) < tol && k == 2) {
							// Crossing tangent with tangent circles (First tangent)
							L[0] = (C[0]+C[2])/2; // L1x
							L[1] = (C[1]+C[3])/2; // L1y
							L[2] = (C[0]+C[2])/2; // L2x
							L[3] = (C[1]+C[3])/2; // L2y
						} else {
							// No crossing tangent if d <= 2*r - tol
							// We also dicard the sencond tangent for tangent circles
							// as it is the same as the first one
							continue;
						}
					}

					// Compute omega
					omega1 = atan2( L[1]-C[1], L[0]-C[0] ) - atan2( 0  - C[1], 0  - C[0] );
					omega3 = atan2( yf - C[3], xf - C[2] ) - atan2( L[3]-C[3], L[2]-C[2] );

					// Angles close to tol are set to 0
					if (fabs(omega1) < tol)
						omega1 = 0.0;
					if (fabs(omega3) < tol)
						omega3 = 0.0;

					// Modify omega in order to get the quickest way around circle
					omega1short = modangle(omega1);
					omega3short = modangle(omega3);

					// Modify omega in order to get the longest way around the circle
					omega1long = get2picomplementaryangle(omega1short);
					omega3long = get2picomplementaryangle(omega3short);

					// Loop through possible omega1
					for (n=0; n<2; n++) {
						// Define which omega1 to use while avoiding a
						// complementary angle close to 2pi
						if (n == 0) {
							omega1 = omega1short;
						} else if (n == 1 && fabs(omega1short) >= tol) {
							omega1 = omega1long;
						} else {
							continue;
						}

						// Loop through possible omega3
						for (m=0; m<2; m++) { // Loop through possible omega3
							// Define which omega3 to use while avoiding a
							// complementary angle close to 2pi
							if (m == 0) {
								omega3 = omega3short;
							} else if (m == 1 && fabs(omega3short) >= tol) {
								omega3 = omega3long;
							} else {
								continue;
							}

							// Define rot1 and rot3
							rot1 = dtp_sign(omega1);
							rot3 = dtp_sign(omega3);

							// Compute relcirclepos1
							relcirclepos1 = (int)pow(-1,i);

							// Compute dir1 (Fwd or Back at startpoint?)
							dir1 = rot1 * relcirclepos1;

							// Compute beta1
							beta1 = relcirclepos1 * fabs(omega1);

							// l: length of the straight line
							l = sqrt( pow(L[2]-L[0],2) + pow(L[3]-L[1],2) );

							// Ignore line length smaller than tol
							if (l < tol)
								l = 0;

							// Check if we need to determine straight line direction
							if (l != 0) {
								// Define psi
								psi = atan2( L[3]-L[1], L[2]-L[0] );

								// Fwd or Back along the straight line?
								if ( fabs(L[0]) > tol && fabs(psi) > tol )
									dir2 = relcirclepos1 * dtp_sign(psi) * dtp_sign(L[0]);
								else
									dir2 = relcirclepos1 * dtp_sign(cos(psi)) * dtp_sign(C[1]-L[1]);
								
								// relcirclepos3 : is the circle to the right (-1) or to the left (+1)
								eta = modangle(atan2(C[3] - L[1], C[2] - L[0]) - psi);
								relcirclepos3 = dir2 * dtp_sign(eta);
							} else {
								if (fabs(d - 2*r) < tol) {
									// Case where the two circles touch, and a
									// crossing tagent has been chosen

									// Set dir2 to zero
									dir2 = 0;

									// Compute dir3
									dir3 = rot1 * dir1 * rot3 * (-1);

									// Compute relcirclepos3
									relcirclepos3 = (-1) * relcirclepos1;

								} else  {
									// Egocentric circles
									dir2 = 0;
									relcirclepos3 = relcirclepos1; // We do not change circle
								}
							}

							// Compute dir3
							dir3 = rot3 * relcirclepos3;

							// Compute beta3
							beta3 = relcirclepos3 * fabs(omega3);

                            // Save the found path in the path array
							arrpath[solutionindex].beta1 = beta1;
							arrpath[solutionindex].dir1 = dir1;
							arrpath[solutionindex].l = l;
							arrpath[solutionindex].dir2 = dir2;
							arrpath[solutionindex].beta3 = beta3;
							arrpath[solutionindex].dir3 = dir3;
							arrpath[solutionindex].D = 0; // Mark path as valid (D != -1)

							solutionindex++;
						}
					}
				}
			}
		}
	}
}

int dtp_sign(double x) {
	if (x > 0) {
		return 1;
	} else if (x < 0) {
		return -1;
	} else {
		return 0;
	}
}

double modangle(double alpha) {
	if (alpha > M_PI) {
		alpha = alpha - 2 * M_PI;
	} else if (alpha <= -M_PI){
		alpha = alpha + 2 * M_PI;
	}

	return alpha;
}

double get2picomplementaryangle(double alpha) {
	if (alpha <= 0) {
		alpha = alpha + 2 * M_PI;
	} else if (alpha > 0) {
		alpha = alpha - 2 * M_PI;
	}

	return alpha;
}
void displacecoordinatesystem(double * x, double * y, double dx, double dy) {
	// Apply displacement
	*x = *x - dx;
	*y = *y - dy;
}
void rotatecoordinatesystem(double * x, double * y, double dtheta) {
	// Temp variables
	double xx, yy;

	// Apply rotation
	xx = *x * cos(dtheta) + *y * sin(dtheta);
	yy = *y * cos(dtheta) - *x * sin(dtheta);

	// Return
	*x = xx;
	*y = yy;
}
