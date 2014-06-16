
/**************************************************************  
 *
 *	@file		drivetopoint.h							  
 * 										   					    
 *	@version	1.0										        
 *															    
 * 	@author		Johan Musaeus Bruun, s052655				    
 * 	@author		Olivier Corradi, s052516				    	
 * 															    
 *	@updated	June 14th 2007							    
 *															    
 *	@project:	course 01666: Fagprojekt		    
 *															  
 * 	@description:											   
 *				Header to drivetopoint_evaluate.c
 *				and drivetopoint_generate.c
 * 																
 ***************************************************************/

#define DTP_MAX_NUM_SOLUTIONS 32 /* Maximum number of dtp_paths returned */

// Structure
typedef struct{
	double beta1, l, beta3, D;
	int dir1, dir2, dir3;
} dtp_path;

// Function prototypes
void drivetopoint_generate(double xa, double ya, double thetaa, double xb, double yb, double thetab, double r, dtp_path arrpath[]);
void dtp_eval_shortest(dtp_path arrpath[], dtp_path * optpath, double r);
void dtp_eval_shortest_noback(dtp_path arrpath[], dtp_path * optpath, double r);
