#include <math.h>
#include "drivetopoint.h"

/**************************************************************
 *
 *	@file		drivetopoint_evaluate.c
 *
 *	@version	1.0
 *
 * 	@author		Johan Musaeus Bruun, s052655
 * 	@author		Olivier Corradi, s052516
 *
 *	@updated	June 19th 2007
 *
 *	@project:	course 01666: Fagprojekt
 *
 * 	@description:
 *				Regroups all the evaluation algorithms in order
 *				to obtain the optimal solution.
 *				Remember that the invalid solutions in the array
 *				are distinguished by arrpath[i].D == -1
 *
 ***************************************************************/

void dtp_eval_shortest(dtp_path arrpath[], dtp_path * optpath, double r) {

	/***************************************************************
	* dtp_eval_shortest
	* 
	* Chooses the solution with the shortest travelled distance
	****************************************************************/

	// Define vars
	int i;
	int isfirstsolution = 1;
	double tol = 0.001;

	// Loop through all solutions
	for (i=0; i<DTP_MAX_NUM_SOLUTIONS; i++) {

		// Unvalid solutions have a cost of -1
		// We must then throw out every solution with a cost of -1
		if (arrpath[i].D == -1)
			continue;

		// Filter commands that are under tol
		if (fabs(arrpath[i].beta1) < tol)
			arrpath[i].beta1 = 0;
		if (arrpath[i].l < tol)
			arrpath[i].l = 0;
		if (fabs(arrpath[i].beta3) < tol)
			arrpath[i].beta3 = 0;

		// Compute cost
		arrpath[i].D = r*(fabs(arrpath[i].beta1)
			+ fabs(arrpath[i].beta3))
			+ arrpath[i].l;

		// If cost is less, then this solution is better
		if (isfirstsolution == 1 || arrpath[i].D + tol < optpath->D) {
			*optpath = arrpath[i];
			isfirstsolution = 0;
		}
	}

}

void dtp_eval_shortest_noback(dtp_path arrpath[], dtp_path * optpath, double r) {

	/***************************************************************
	* dtp_eval_shortest_noback
	* 
	* Chooses the solution with the shortest travelled distance
	* while avoiding backward movements
	****************************************************************/

	// Define vars
	int i;
	int isfirstsolution = 1;
	double tol = 0.001;

	// Loop through all solutions
	for (i=0; i<DTP_MAX_NUM_SOLUTIONS; i++) {

		// Unvalid solutions have a cost of -1
		// We must then throw out every solution with a cost of -1
		// Discard solutions that are using backward movements
		if (arrpath[i].D == -1 || arrpath[i].dir1 < 0 ||
			arrpath[i].dir2 < 0 || arrpath[i].dir3 < 0)
			continue;

		// Filter commands that are under tol
		if (fabs(arrpath[i].beta1) < tol)
			arrpath[i].beta1 = 0;
		if (arrpath[i].l < tol)
			arrpath[i].l = 0;
		if (fabs(arrpath[i].beta3) < tol)
			arrpath[i].beta3 = 0;

		// Compute cost
		arrpath[i].D = r*(fabs(arrpath[i].beta1)
			+ fabs(arrpath[i].beta3))
			+ arrpath[i].l;

		// If cost is less, then this solution is better
		if (isfirstsolution == 1 || arrpath[i].D + tol < optpath->D) {
			*optpath = arrpath[i];
			isfirstsolution = 0;
		}
	}

}
