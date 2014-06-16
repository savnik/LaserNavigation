#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "client.h"
#include "functions.h"

void init()
{
	reset();
	sensors.enginespeed = 0;
	sensors.navmode = 0;
	sensors.odovel = 0;
	sensors.xkalman = 0;
	sensors.ykalman = 0;
	sensors.thkalman = 0;
	sensors.liftingGearPos = 0;
	sensors.powerTakeoffSpeed = 0;
	sensors.gpsquality = 0;
	sensors.satellites = 0;
	sensors.dop = 0;
	sensors.gpsn = 0;
	sensors.gpse = 0;
	sensors.steeringangle = 0;
	sensors.targetdist = 0;
	sensors.kalmanstatus = 1;
	socket_busy = 0;
	server_mode = 0;
	go_flag = 1;
	exit_flag = 0;
	pause_flag = 0;
}

void reset()
{
  const missiontype null_mission = {{{0}}};

	int i;
	data.xml.startok = 0;
	data.xml.kalmaninit = 0;
	data.xml.route = 0;
	data.xml.depth = 0;
	data.xml.speeddef = 0;
	data.xml.accdef = 0;
	data.xml.angledef = 0;
	data.xml.cnt = 0;
	
	data.kinit.type = 0;
	data.kinit.dist = DFLTKDIST;
	data.kinit.speed = DFLTSPEED;
	data.kinit.acc = DFLTACC;
	
	data.log.logflag = 0;
	data.log.gpsflag = 0;
	data.log.kalmanflag = 0;
	data.log.imuflag = 0;
	data.log.odoflag = 0;
	
	data.mission = null_mission;
	/* Get a "zeroed" copy of a mission - thus fields which
	 * are inactive when zero need not be initialized further.
	 * I.e. as long as new fields are inactive when zero, they
	 * need not be included in the loop below. */
	/* Extend to whole of 'data' ? */

	for(i=0; i<MISSIONPOINTS; i++)
	 {
	 	data.mission.point[i].e = 0;
	 	data.mission.point[i].n = 0;
	 	data.mission.point[i].ang = 0;
	 	data.mission.speed[i] = DFLTSPEED;
		data.mission.acc[i] = DFLTACC;
		data.mission.turntype[i] = NOTURN;
		data.mission.tools[i].toolpos = NOCHANGE;
		data.mission.tools[i].power = NOCHANGE;
		data.mission.pause[i] = 0;
	 }
}

void update_sensors()
{
	
	
}

void read_sensors(void)
{
  double tmp[5] = {0};
  double smrtime;
  sscanf(&ct.answer[6],"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&smrtime,
				&sensors.enginespeed,
				&tmp[0],
				&sensors.odovel,
				&sensors.xkalman,
				&sensors.ykalman,
				&sensors.thkalman,
				&sensors.gpsn,
				&sensors.gpse,
				&sensors.steeringangle,
				&sensors.targetdist,
				&tmp[1],
				&tmp[2],
				&sensors.dop,
				&sensors.kalmanstatus,
				&tmp[3],
				&tmp[4]);
	sensors.navmode = (unsigned char) tmp[0];
	sensors.gpsquality = (unsigned char) tmp[1];
	sensors.satellites = (unsigned char) tmp[2];
	sensors.liftingGearPos = (unsigned char) tmp[3];
	sensors.powerTakeoffSpeed = (unsigned char) (tmp[4]/10);
  /* fix kalman pre-init position, so that display position is OK */
  if (fabs(sensors.ykalman) < 1.0)
  { /* too close to equator - kalman filter not started */
    sensors.xkalman = sensors.gpse;
    sensors.ykalman = sensors.gpsn;
  }
}

int driveto(double n, double e, double th, double sp, double acc)
{
	char str[128];
	
	sprintf(str,"drive %f %f %f @v%f @a%f : ($targetdist < 0)\n",e,n,th,sp,acc);
	cl_cmdwait(str);

	return 0;
}

/**
Generate fishtail turn.
The turn is to the a line parallel to the current path at the distance
specified. If this can be acheived without reversing, then this is what is planned.
The fishtail turn turns towards the next row, then reversing while turning opposite,
stopping and is now ready to aim for next row, which should be reachable
from about the position where the turn started, using a driveon command.
@author Christian Andersen at July 13, 2010
\param dist is perpendicular distance to next row - positive is left.
\param stopacc is a robot constant defining maximum speed change (about 0.75 m/s2)
\param reactime is the speed change reaction time (approx 0.7 sec)
\param drivevel is speed into fishtail turn - to be maintained if possible
\returns 0 */
int fishtail2(double dist, /*double stopacc, double reactime,*/ double drivevel)
{
  double stopacc = STOPACC;  /* acceleration during speed change - after reaction time*/
  double reacTime = REACTIME; /* expected delay for speed changes */
  double tonextrow;
  double turnrad = MAXTURNRAD; /* used turn radius in turn */
  double stopang; /* angle required to stop the robot */
  const int MSL = 128;
  char str[MSL];
  double angfac; /* calculations are for a left turn, and angfac is -1 if right turn. */
  double fwdturn;
  double revturnhalf;
  double ang1, ang2;
  /**/
  if (dist > 0.0)
  {
    tonextrow = dist;
    angfac = 1.0;
    printf("Fishturn next-row dist %.3fm (left) speed is %.1fm/s\n", dist, drivevel);
  }
  else
  {
    tonextrow = -dist;
    angfac = -1;
    printf("Fishturn next-row dist %.3fm (right) speed is %.1fm/s\n", dist, drivevel);
  }
  /* angle required to stop robot */
  stopang = drivevel * drivevel / (2.0 * turnrad * stopacc) + drivevel * reacTime / turnrad;
  printf("fishturn stopangle %.3frad (%.2fm)\n", stopang, stopang * turnrad);
  /* Angle required for first part of tail */
  if (tonextrow < 2.0 * turnrad) /* "noReverseNeeded" */
  { /* angle of first turn forward */
    /**/
    fwdturn = acos((turnrad - tonextrow / 2) / (2 * turnrad));
    /* angle of reverse turn */
    revturnhalf = M_PI / 2.0 - fwdturn;
    ang1 = 0.0;
    if (stopang < fwdturn) /* "stopnow" */
    { /* turn first part at full speed */
      ang1 = fwdturn - stopang;
      /* first part at full speed */
      /** ang = ang1 * angfac
          turnr turnrad ang "rad" @v drivevel @a stopaccCtl */
      snprintf(str, MSL, "turnr %f %f \"rad\" @v%f\n",turnrad, ang1 * angfac, drivevel);
      cl_send(str);
    }
    /* label "stopnow" */
    /* turn while stopping */
    ang2 = fwdturn - ang1; 
    /** ang = ang2 * angfac
        turnr turnrad ang "rad" @v 0.01 @a stopacc : ($odovelocity < 0.012) */
    snprintf(str, MSL, "turnr %f %f \"rad\" @v 0.0 : ($odovelocity < 0.1)\n",turnrad, ang2 * angfac);
    cl_send(str);
    /* % reverse turn */
    ang1 = revturnhalf;
    if (stopang < revturnhalf) /* "noFullRevSpeed" */
    { /* can reach full speed */
      ang1 = revturnhalf * 2.0 - stopang;
    }
    /* label "noFullRevSpeed" */
    /* % make angle negative and to the correct side */
    ang2 = -(revturnhalf * 2.0 - ang1);
    ang1 = -ang1;
    /* set reverse target speed */
    /** revvel = -drivevel
    % drive until time to stop
    ang = ang1 * angfac
    turnr turnrad ang "rad" @v revvel @a stopaccCtl */
    snprintf(str, MSL, "turnr %f %f \"rad\" @v%f\n",turnrad, ang1 * angfac, -drivevel);
    cl_send(str);
    /* % complete reverse turn and stop */
    /** ang = ang2 * angfac
    turnr turnrad ang "rad" @v0 @a stopacc : (abs($odovelocity) < 0.012)*/
    snprintf(str, MSL, "turnr %f %f \"rad\" @v 0.0 ($odovelocity > -0.1)\n",turnrad, ang2 * angfac);
    cl_send(str);
    /* goto "finishedFishtail" */
  }
  else
  { // label "noReverseNeeded"
    // turn with no reversing - at full speed
    fwdturn = M_PI / 2.0;
    ang1 = fwdturn;
    /** ang = ang1 * angfac
        turnr turnrad ang "rad" @v drivevel @a stopaccCtl */
    snprintf(str, MSL, "turnr %f %f \"rad\" @v%f\n",turnrad, ang1 * angfac, drivevel);
    cl_send(str);
  }
  printf("Fishturn finished - aiming for next row\n");
  // ready to aim for next row
  // label "finishedFishtail"
  return 0;
}

/****************************************/

int fishtale(double dist, double anglefrom, double angleto)
{
	char str[128];
	double angle = fmod(fmod(angleto-anglefrom+180,360)+360,360)-180;
	if(fmod(angleto-anglefrom,180) == 0 && anglefrom-angleto > 0)
		angle = 180;
	sprintf(str,"angle = (%lf) + (%lf) - $thkalman*180.0/%lf\n",angle,anglefrom,M_PI);
	cl_send(str);
	sprintf(str,"angle = normalizeangledeg(angle)\n");
	cl_send(str);
	sprintf(str,"turnr %lf angle @v%lf @a%lf\n",MAXTURNRAD,TURNSPEED,TURNACC);
	cl_send(str);
	sprintf(str,"fwd 0.7 @v%lf @a%lf\n",TURNSPEED,TURNACC);
	cl_send(str);
	/* reverse to allow drive to line */
	/* sprintf(str,"fwd %lf @v%lf @a%lf\n",dist-MAXTURNRAD-MAXTURNRAD-1,TURNSPEED,TURNACC); */
	sprintf(str,"fwd %lf @v%lf @a%lf\n",dist-MAXTURNRAD-MAXTURNRAD-0.2,TURNSPEED,TURNACC);
	cl_send(str);
	/* then forward a bit */
	sprintf(str,"drive @v%lf @a%lf : ($drivendist > 0.2)\n",TURNSPEED,TURNACC);
	cl_send(str);
	/* no need for the last turn, as it ends in a drive x,y,th command */
	/*
	sprintf(str,"angle = (%lf) - $thkalman*180/%lf\n",anglefrom-180,M_PI);
	cl_send(str);
	sprintf(str,"angle = normalizeangledeg(angle)\n");
	cl_send(str);
	sprintf(str,"angle = angle/2.0\n");
	cl_send(str);
	sprintf(str,"turnr %lf angle @v%lf @a%lf\n",MAXTURNRAD,TURNSPEED,TURNACC);
	cl_cmdwait(str);
	*/
	return 0;
}
