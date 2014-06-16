#include "administration.h"
#include "mobilerobot.h"
#include "regulators.h"
#include "drivetopoint.h"
#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H


#define HALF_ROBOT_WIDTH1 0.13 /*lateral distance from the odometry center to the sensor of the robot */
#define ROBOT_LENGTH1 0.21 /*longitudinal distance from the odometry center to the sensor of the robot */
//#define SMR2IS 0.01039 /* 1 smr speed unit is equal to 0.01039 m/s */
#define WDSSPAR2IS 0.01039 /* 1 line in the wdssparametres file is equal to a speed increment of 0.01039 m/s */
#define MINIMUN_SPEED 0.1 /* minimun speed for running in m/s units */

#define MaxSteerHist 100 /* length of buffer for steering control in delay time for command */

enum{MC_FWDCMD=1,MC_STOPCMD,MC_TURNCMD,MC_TURNRCMD,MC_TURNCCMD,MC_FOLLOW_LINECMD,
     MC_FOLLOW_WALLCMD,MC_DRIVECMD,MC_DRIVEONCMD, MC_DRIVECMDW,MC_DRIVEONCMDW,  MC_IDLECMD,MC_FOLLOW_WALLCMD2,MC_ALIGNCMD,
     MC_DIRECTMOTORCMDS,MC_DRIVETOPOINTCMD,MC_MOVETOXYZCMD,MC_FUNCGEN};
enum states{$MC_FWD,$MC_STOP,$MC_TURN,$MC_TURNR,$MC_TURNC,$MC_FOLLOW_LINE,$MC_FOLLOW_WALL,
     $MC_DRIVE,$MC_IDLE,$MC_FOLLOW_WALL2,$MC_ALIGN,$MC_DIRECTMOTORCMDS,$MC_DRIVETOPOINT,$MC_MOVETOXYZ,$MC_FUNCGEN};
enum{MC_OK,MC_OBSTACLE};
enum{MC_LEFT,MC_RIGHT};
enum{MC_DIRECT,MC_REGRESSION};
enum dtp_states{DTP_INIT,DTP_STARTCIRCLE,DTP_STRAIGHTLINE,DTP_ENDCIRCLE,DTP_FINISH};
enum dtp_coordtypes{DTP_COORD_REL,DTP_COORD_ABS};
enum dtp_evalmethods{DTP_EVAL_SHORTEST,DTP_EVAL_SHORTEST_NOBACK};
enum {DIFFERENTIAL=0,ACKERMAN,VELOMEGA,GRIPPER,HEXACOPTER};

typedef
struct {// parameters ---------------------------------------------------------
	int	type;
	double	ts;	// sampletime (s)
	int tick;       // sample tick
        double  robotlength;
  	double  gain;
	double  kp;
  	double  alfa;
	double  tau;
	double  lim;
	double	w;	//distance between wheels
	double  stopdist;
	double  alarmdist;
        double  nolinedist;
	double  kangle_drive;
	double  kdist_drive;
        // inserted to handle Hako steering
        double  steerDelay;
        double  steerVel;
        //
	double  *refx_drive;
        double  *refy_drive;
	double  *refth_drive;
	double	line_gain;  // gain for line following controller
  	double  line_alfa;
  	double  line_tau;
	double	wall_gain;  // gain for wall following controller
	double  wall_alfa;  // PD-controller parameter
  	double  wall_tau;   // PD-controller parameter
	double  wall_anglegain;
  	double irlen, irwidth;  // length and width from the odometry center of the robot to the ir sensor
	//double smr2is;
  	double wdsspar2is; /* 1 line in the wdssparametres file is equal to a speed increment of 0.01039 m/s */
	float sswd[3][100]; //parametres for the state space wall driving controller
        struct {double b0,b1,a1,ffwd,dlim;}xyzcon1;

	// input --------------------------------------------------------------
	int cmd;	// motion command
  	int ignore_obstacles; //
	double velcmd;	// velocity command
	double acccmd;	// acceleration command
        double dist;	// distance
	double turnangle; // turn angle radians
	double R;	// turning radius
	double velcmdl,velcmdr; // direct motor velocity commands
	posetype refpose; // reference pose
	pose3dtype refpose3d;

        double walldistref;  // reference distance to wall
	double velmin;
	int side; // side to follow the wall
        int method; // method to follow the wall ( directly, regression )
	double walldistregr; //calculated distance from the robot to the regression line
	double wallangregr; //calculated orientation of the robot relative to the regression line.
        int wallreliant;
	posetype w2odo;	    // Transformation from world coordinates to odometry coordinates
	int usew2odo;
        //sensor values
	posetype pose;	// current pose of vehicle
	pose3dtype pose3d;
	double vel;	// current vehicle velocity
        double curvell,curvelr;
	double motoroutl,motoroutr;
        double d_obstac_front;// distance to nearest obstacle in front of vehicle
	double d_obstac_back;
	double linepos;  // line position middle=0 positive if line is to the
	                 //right
        int	noline; // 1 if no black line is found
	//double wallpos; // it was used for wall_driving 1 changed by jose. Now irdistl
	double *irdistl, *irdistr;// meassured distance to the wall (left and right sides)
        double *wallanglel,*wallangler;
	double     wallinputmode;
        double steeringangleNow; // steering angle from robot (especially Hako)
	double funcgenp[10];
	double runlength;
	// output -------------------------------------------------------------
	double rvel;	// velocity command right wheel
	double lvel;	// velocity command left wheel
	double steeringangle; // commanded steering angle
	double curvature;
	double omega;
	double xyzout1,xyzout2,xyzout3,xyzout4;
	int status;	// status of motion controller
  	int curcmd;	// current cmd
	double target_dist;
	posetype poseerr;
	double funcgenout;

	// internal states, dont change ----------------------------------------
  	int ignore_obstacles_flag;
	int state;
	int time;
	posetype tgt;
	posetype tgt_o;
	double velref;
	double velref_l;
  	double velref_r;
  	double vr_turn;
	double vl_turn;
	double tgtnormal;
  	double tcx;
  	double tcy;
	double thturned;
	double thold;
  	pd_type wall_pd;
  	pd_type pd;
  	pd_type line_pd;
	posetype lastlinepose;
	double lineposold;
	double duold; //used in followline
        struct {
	       double vnx,vny,c;
	       }l_drive;
        // internal variables used for steering control historty
        double steerHist[MaxSteerHist];
        int steerHistCnt;
        int steerTick;
//
//  	int smr_actual_speed;
  	int wdsspar_pos;
	double sswallang; // wall driving orientation
	double sswalldist;   // wall driving distance

	/*for turnc*/
	double drivendist;
	double angacc;
	double halfdist;
	double turncgain;

	// for drivetopoint
	struct {
		int state;
		int oldstate;
		int statetime;
		int coordtype; // Abs or Rel
		int evalmethod; // Evaluation method used to choose the optimal path
		double xa, ya, thetaa;
		double xb, yb, thetab;
		double r; // Used turning radius
		double TOL;
		dtp_path arrpath[DTP_MAX_NUM_SOLUTIONS]; // Array with all valid paths
		dtp_path optpath; // Optimal path
	}dtp;
        struct { double d;}movetoxyzstates;
	}motiontype;

double calcdist(posetype p1, posetype p2);
void motion_init(motiontype *p);
void motion_update(motiontype *p);
void read_wdssparam(motiontype *p);









int addpar(parameterlisttype *list,char * name,int partype,int vartype,void *parptr);


#endif //MOTIONCONTROL_H
