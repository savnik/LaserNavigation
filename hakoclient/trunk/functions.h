#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include "server.h"
#include "client.h"

#define UPDATESAMPLETIME 	1.0
#define TURNSPEED 				0.5
#define TURNACC 					1.0
#define MAXTURNRAD 				3.0
#define DFLTSPEED 				1.0
/** acceleration when changing speed (m/s2) - used in fishtail turn */
#define STOPACC               0.7
/** reaction time when changing speed (seconds) - used in fishtail turn */
#define REACTIME              0.75
/**/
#define DFLTACC 					1.0	
#define DFLTKDIST 				5.0	
#define MISSIONPOINTS 		500
#define FISHTURN 					1
#define OMEGATURN					2
#define ARCTURN  					3
#define WIDETURN					4
#define GPS_WIDE_TURN                                   5
#define GPS_FISHTAIL_TURN                               6
#define GPS_LINK_LANES_TURN                             7
#define NOTURN  					0
#define DRIVE							1
#define NOCHANGE					0
#define OFF								1
#define ON								2
#define DFLTTOOLPOS				80
#define CURRENTANGLE	 		-90
#define NOANGLE				1000
#define GOSTR 						"Go"
#define EXITSTR 					"exit"
#define PAUSESTR 					"pause"
#define HELLOSTR 					"hello"
#define SLOGSTR 					"savelog"
#define STARTSTRING 			"mission"
#define KALMANINITSTRING 	"kalmaninit"
#define TYPESTRING 				"type"
#define DISTSTRING 				"dist"
#define ROUTESTRING 			"route"
#define POINTSTRING 			"point"
#define UTMNSTRING 				"utmn"
#define UTMESTRING 				"utme"
#define ANGLESTRING 			"angle"
#define TURNSTRING 				"turn"
#define DRIVESTRING 			"drive"
#define SPEEDSTRING 			"speed"
#define ACCSTRING 				"acc"
#define PAUSESTRING				"pause"
#define FISHTURNSTRING 		"fishtail"
#define OMEGATURNSTRING 	"omega"
#define ARCTURNSTRING     	"arc"
#define WIDETURNSTRING     	"wide"
#define GPS_WIDE_TURN_STRING    "gps_wide"
#define LOGSTRING 				"log"
#define GPSSTRING 				"gps"
#define KALMANSTRING 			"kalman"
#define IMUSTRING					"imu"
#define ODOSTRING 				"odometry"
#define TRUESTRING 				"true"
#define FALSESTRING 			"false"
#define TOOLSTRING				"tool"
#define POWERSTRING				"power"
#define ONSTRING					"on"
#define OFFSTRING					"off"
/* Added 2007jun13, JoN */
#define ENGINE_RPM_STRING                "enginerpm"
#define HIGH_RPM_STRING                  "high"
#define LOW_RPM_STRING                   "low"
#define ENGINE_HIGH_RPM_CMD              1
#define ENGINE_LOW_RPM_CMD               2
/* Added 2007jun14, JoN */
#define TURN_ANGLE_STRING         "turn_angle"
#define TURN_RADIUS_STRING        "turn_radius"
/* These attributes are only relevant in connection with a an "arc" or "gps_wide" turn command. */
#define TARGETDIST_STRING         "targetdist"
/* The above attribute is for straight line segments (no turn) and "gps_wide" turn. */
#define ARC_STEP_LENGTH_STRING    "arc_step_length"
/* The above attribute is for "gps_wide" turn command. */
#define SMRCL_CMD_STRING          "smrcl_cmd"
/* The above attribute is for the "escape sequence", enabling embedded SMR-CL commands in XML route plan file. */

/* Added 2007jun16, JoN */
#define GPS_FISHTAIL_STRING       "gps_fishtail"
#define GPS_LINK_LANES_STRING     "gps_link_lanes"
#define ALT_SPEED_STRING          "alt_speed"


typedef struct
{
	int startok;
	int kalmaninit;
	int route;
	int depth;
	int speeddef;
	int accdef;
	int angledef;
	int cnt;
}xmlflagtype;

typedef struct
{
	int logflag;
	int gpsflag;
	int kalmanflag;
	int imuflag;
	int odoflag;
}logflagtype;

typedef struct
{
	double enginespeed;
	unsigned char navmode;
	double odovel;
	double xkalman;
	double ykalman;
	double thkalman;
	double gpsn;
	double gpse;
	double steeringangle;
	unsigned char liftingGearPos;
	unsigned char powerTakeoffSpeed;
	unsigned char gpsquality;
	unsigned char satellites;
	double dop;
	double targetdist;
	double kalmanstatus;
} sensortype;

typedef struct
{
	double e;
	double n;
	double ang;
	
}pointtype;

typedef struct
{
	int type;
	double dist;
	double speed;
	double acc;
	
}kalmaninittype;

typedef struct
{
	int toolpos;
	int power;
}toolstype;

typedef struct
{
  pointtype point[MISSIONPOINTS];
  toolstype tools[MISSIONPOINTS];
  double speed[MISSIONPOINTS];
  double alt_speed[MISSIONPOINTS]; /* Added 2007jun16.
				    * Extra speed field (could be used when reversing in fishtail?).
				    * Note: this is NOT sticky, i.e. it must be specified for each
				    * point using it (otherwise default value will be used). */
  double acc[MISSIONPOINTS];
  int turntype[MISSIONPOINTS];
  int pause[MISSIONPOINTS];
  int engine_rpm_cmd[MISSIONPOINTS]; /* 0 is "no action" */
  double turn_angle[MISSIONPOINTS]; /* Added 2007jun14. Used by "arc" and "gps_wide_turn". */
  double turn_radius[MISSIONPOINTS]; /* Added 2007jun14. Used by "arc" and "gps_wide_turn". */
  double targetdist[MISSIONPOINTS]; /* Added 2007jun14. Used for straight lines and by "gps_wide_turn". */
  double arc_step_length[MISSIONPOINTS]; /* Added 2007jun14. Used by "gps_wide_turn". */
  const char * smrcl_cmd_str[MISSIONPOINTS]; /* Pointer to SMR-CL command string ("escape sequence"). Added 2007jun14 */
} missiontype;

typedef struct
{
	xmlflagtype xml;
	logflagtype log;
	kalmaninittype kinit; 
	missiontype mission;
	
}datatype;

void init(void);
void reset(void);
void update_sensors(void);
void read_sensors(void);
int driveto(double n, double e, double th, double sp, double acc);
int fishtale(double dist,double anglefrom, double angleto);
int fishtail2(double dist,/* double stopacc, double reactime,*/ double drivevel);

clienttype ct;
datatype data;
sensortype sensors;
servertype serv;
int socket_busy;
int server_mode;
int go_flag;
int exit_flag;
int pause_flag;
#endif
