#ifndef PLANNERS_H
#define PLANNERS_H

/*
 * For C++ compilers, use extern "C"
 */
 
#ifdef __cplusplus
extern "C" {
#endif
#define 	NMAXCON	10

typedef struct {
         double x,y,th;
	}posetype;

typedef struct{
		double x,y;
		int conlist[NMAXCON];
		double costlist[NMAXCON];
		double nodecost;
		int  fromnode;
		int inlist,initialized;
		int  slistprenode,slistnextnode;
		}graphnode;
		
typedef struct {
	 	int slisthead;		
		int Nnodes;
		graphnode *nodes;
		} graphtype;
		
double findroute(int startnode,int targetnode,graphtype *G);

void addp(graphtype * G,int id, double x, double y);      				
void addcon(graphtype *G, int id, int conid,double cost);
void removep(graphtype * G,int id);
void removecon(graphtype *G, int id, int conid);
void calculatecost(graphtype * G);
double findarc(graphtype *G,double xr, double yr, double *xa, double *ya,int *p,int *con); 
double findnearestpoint(graphtype *G,double xr, double yr, double *xa, double *ya,int *p);
double findroutexy(graphtype *G, double startx, double starty, double goalx, double goaly,posetype *route, int * routestart);
/*
 * end block for C++
 */

#ifdef __cplusplus
}
#endif


#endif  /* PLANNER_H */

