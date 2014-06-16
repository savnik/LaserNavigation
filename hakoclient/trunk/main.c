/**\file main.c
 * The main program file for the top level missioncontroller
 * This is the main source file for the top level mission controller. 
 * The purpose of this program is to parse the mission XML and decide 
 * exactly how the mission should be executed.
 * These tasks include taking orders from the GUI communication using UDP packages 
 * in unconnected mode to improve stability of this program. 
 * Communication to smrdemo is done using a normal TCP socket connection.
 * \author Asbj�rn Mejnertsen
 * \author Anders Reske-Nielsen
 *
 * \date 03/31-06
 */
#include <stdlib.h>
#include <stdio.h>
#include <expat.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <semaphore.h>

#include "client.h"
#include "server.h"
#include "functions.h"
#include "turn.h"
#include "logtime.h"
#include "drivetowards.h"
#include "command_store.h"
//#include "/vhome/naa/robot2005/src/server/stream2line.h"
#include "stream2line.h"

#define sqr(x) (x*x)
#define abs(x) (x<0.0?-1.0*x:x)

void *udprxtask(void *);
void *updatetask(void *);
void *recievetask(void *);
pthread_t udprxthread, updatethread, recievethread;
pthread_attr_t attr;
sem_t cmdsem,updatesem;
stream2line_type s2line;

///XML start element function.
///This function is used when parsing XML file. Everytime the parser reaches a start tag
///this function is called and the data is read from the file and copied to relevant locations
///like the mission struct.
static void XMLCALL
startElement(void *userData, const char *name, const char **atts)
{
  int i = 0;
  datatype *data = (datatype *) userData;
  
  //If the XML file has the correct start tag parsing can continue
  if(data->xml.depth == 0 && strcasecmp(name,STARTSTRING) == 0) {
    /* XML start tag in accepted */
    data->xml.startok = 1;
    data->xml.kalmaninit = 0;
    data->xml.route = 0;
    data->xml.depth = 0;
    data->xml.cnt = 0;
  } else
    if (data->xml.depth == 1 && data->xml.startok) {
      /* Log tag found */
      if (strcasecmp(name,LOGSTRING) == 0) {
	data->log.logflag = 1;
	for (i = 0; atts[i]; i += 2) {
	  // Data that should be logged
	  if(strcasecmp(atts[i],GPSSTRING) == 0) {
	    if(strcasecmp(atts[i+1],TRUESTRING) == 0)
	      data->log.gpsflag = 1;
	  } else
	    if(strcasecmp(atts[i],KALMANSTRING) == 0) {
	      if(strcasecmp(atts[i+1],TRUESTRING) == 0)
		data->log.kalmanflag = 1;
	    } else
	      if(strcasecmp(atts[i],IMUSTRING) == 0) {
		if(strcasecmp(atts[i+1],TRUESTRING) == 0) {
		  data->log.imuflag = 1;
		}
	      } else
		if (strcasecmp(atts[i],ODOSTRING) == 0) {
		  if(strcasecmp(atts[i+1],TRUESTRING) == 0)
		    data->log.odoflag = 1;
		}
	}
      }
      // Tag defining how the Kalman filter should be initialized  	
      if(strcasecmp(name,KALMANINITSTRING) == 0) {
	data->xml.route = 0;
	data->xml.kalmaninit = 1;
	for (i = 0; atts[i]; i += 2) {
	  if(strcasecmp(atts[i],TYPESTRING) == 0) {
       	    if(strcasecmp(atts[i+1],DRIVESTRING) == 0)
	      data->kinit.type = DRIVE;
	  }
	  else if (strcasecmp(atts[i],DISTSTRING) == 0)
	      data->kinit.dist = atof(atts[i + 1]); 	
	  else if(strcasecmp(atts[i],SPEEDSTRING) == 0)
		data->kinit.speed = atof(atts[i + 1]);
	  else if(strcasecmp(atts[i],ACCSTRING) == 0)
		  data->kinit.acc = atof(atts[i + 1]);
	}
      }
      // route tag has been found
      else
	if(strcasecmp(name,ROUTESTRING) == 0) {
	  printf("Routetag found \n");
	  data->xml.route = 1;
	  data->xml.cnt = 0;
  	}
    } else
      if(data->xml.depth == 2 && data->xml.startok && data->xml.route) {
	printf("_");
	// Point in route has been found
	if(strcasecmp(name,POINTSTRING) == 0) {
	   printf("pointtag found\n");
	  for (i = 0; atts[i]; i += 2) {
	    // Individual point data is being copied to mission struct and missing values is calculated or assigned default values
	    if (strcasecmp(atts[i],UTMNSTRING) == 0)
	      data->mission.point[data->xml.cnt].n = atof(atts[i + 1]);
	    else
	      if (strcasecmp(atts[i],UTMESTRING) == 0)
		data->mission.point[data->xml.cnt].e = atof(atts[i + 1]);
	      else
		if (strcasecmp(atts[i],ANGLESTRING) == 0) {
		  data->xml.angledef = 1;
		  data->mission.point[data->xml.cnt].ang = atof(atts[i + 1]);
		} else
		  if (strcasecmp(atts[i],SPEEDSTRING) == 0) {
		    data->xml.speeddef = 1;
		    data->mission.speed[data->xml.cnt] = atof(atts[i + 1]);
		  } else
		    if (strcasecmp(atts[i],ACCSTRING) == 0) {
		      data->xml.accdef = 1;
		      data->mission.acc[data->xml.cnt] = atof(atts[i + 1]);
		    } else
		      /* Not ACCSTRING */
		      if (strcasecmp(atts[i],TURNSTRING) == 0) {
			/* Two turn types added 2007jun10 */
			if (strcasecmp(atts[i + 1],FISHTURNSTRING) == 0) {
			  data->mission.turntype[data->xml.cnt] = FISHTURN;
			} else {
			  if (strcasecmp(atts[i + 1],OMEGATURNSTRING) == 0) {
			    data->mission.turntype[data->xml.cnt] = OMEGATURN;
			  } else {
			    if (strcasecmp(atts[i + 1], ARCTURNSTRING) == 0) {
			      data->mission.turntype[data->xml.cnt] = ARCTURN;
			    } else {
			      if (strcasecmp(atts[i + 1], WIDETURNSTRING) == 0) {
				data->mission.turntype[data->xml.cnt] = WIDETURN;
			      } else {
				/* Turn type added 2007jun14. */
				if (strcasecmp(atts[i + 1], GPS_WIDE_TURN_STRING) == 0) {
				  data->mission.turntype[data->xml.cnt] = GPS_WIDE_TURN;
				  printf("gps_wide_turn attribute value parsed.\n");
				} else {
				  /* Turn types added 2007jun16. */
				  if (strcasecmp(atts[i + 1], GPS_FISHTAIL_STRING) == 0) {
				    data->mission.turntype[data->xml.cnt] = GPS_FISHTAIL_TURN;
				    printf("gps_fishtail turn attribute value parsed.\n");
				  } else {
				    if (strcasecmp(atts[i + 1], GPS_LINK_LANES_STRING) == 0) {
				      data->mission.turntype[data->xml.cnt] = GPS_LINK_LANES_TURN;
				      printf("gps_link_lanes turn attribute value parsed.\n");
				    } else {
				      data->mission.turntype[data->xml.cnt] = NOTURN;
				    }
				  }
				}
			      }
			    }
			  }
			}
		      } else
			/* Not TURNSTRING */
			if (strcasecmp(atts[i],TOOLSTRING) == 0) {
			  data->mission.tools[data->xml.cnt].toolpos = atoi(atts[i + 1]);
			} else
			  /* Not TOOLSTRING */
			  if (strcasecmp(atts[i],POWERSTRING) == 0) {
			    if(strcasecmp(atts[i + 1],ONSTRING) == 0)
			      data->mission.tools[data->xml.cnt].power = ON;
			    else
			      if (strcasecmp(atts[i + 1],OFFSTRING) == 0)
				data->mission.tools[data->xml.cnt].power = OFF;
			      else
				data->mission.tools[data->xml.cnt].power = NOCHANGE;
			  } else 
			    /* Not POWERSTRING */
			    if (strcasecmp(atts[i],PAUSESTRING) == 0) {
			      data->mission.pause[data->xml.cnt] = atof(atts[i + 1]);
			    } else {
			      /* Not PAUSESTRING, test for ENGINE_RPM_STRING. */
			      printf("*");
			      if (strcasecmp(atts[i],ENGINE_RPM_STRING) == 0) {
				/* A command to set engine rpm was encountered, next determine
				 * whether it is high or low engine rpm. */
				if (strcasecmp(atts[i+1], HIGH_RPM_STRING) == 0) {
				  /* It was a "high engine rpm" command ... */
				  data->mission.engine_rpm_cmd[data->xml.cnt] = ENGINE_HIGH_RPM_CMD;
				  printf("'engine_high_rpm' command parsed.\n");
				} else {
				  if (strcasecmp(atts[i+1], LOW_RPM_STRING) == 0) {
				    /* It was a "low engine rpm" command ... */
				    data->mission.engine_rpm_cmd[data->xml.cnt] = ENGINE_LOW_RPM_CMD;
				    printf("'engine_low_rpm' command parsed.\n");
				  } else {
				    /* Unrecognized argument to engine rpm command - ignore */
				    /* ??Has this array been cleared? */
				    printf("Unrecognized engine rpm argument.\n");
				  }
				}
			      } else {
				/* Not ENGINE_RPM_STRING, test for TURN_ANGLE_STRING. */
				if (strcasecmp(atts[i], TURN_ANGLE_STRING) == 0) {
				  /* TURN_ANGLE_STRING found. */
				  data->mission.turn_angle[data->xml.cnt] = atof(atts[i+1]);
				  printf("turn_angle parsed\n");
				} else {
				  /* Not TURN_ANGLE_STRING, test for TURN_RADIUS_STRING. */
				  if (strcasecmp(atts[i], TURN_RADIUS_STRING) == 0) {
				    /* TURN_RADIUS_STRING found. */
				    data->mission.turn_radius[data->xml.cnt] = atof(atts[i+1]);
				    printf("turn_radius parsed\n");
				  } else {
				    /* Not TURN_RADIUS_STRING, test for TARGETDIST_STRING. */
				    if (strcasecmp(atts[i], TARGETDIST_STRING) == 0) {
				      /* TARGETDIST_STRING found. */
				      data->mission.targetdist[data->xml.cnt] = atof(atts[i+1]);
				      printf("targetdist parsed\n");
				    } else {
				      /* Not TARGETDIST_STRING, test for ARC_STEP_LENGTH_STRING. */
				      if (strcasecmp(atts[i], ARC_STEP_LENGTH_STRING) == 0) {
					/* ARC_STEP_LENGTH_STRING found. */
					data->mission.arc_step_length[data->xml.cnt] = atof(atts[i+1]);
					printf("arc_step_length parsed\n");
				      } else {
					/* Not ARC_STEP_LENGTH_STRING, test for SMRCL_CMD_STRING. */
					if (strcasecmp(atts[i], SMRCL_CMD_STRING) == 0) {
					  /* SMRCL_CMD_STRING found. */
					  printf("SMR-CL command parsed.\n");
					  printf("  The command is: %s \n", atts[i+1]);
					  data->mission.smrcl_cmd_str[data->xml.cnt] = CS_store(atts[i+1]);
					  /* This implementation is only intended for a single string, but perhaps
					   * a pointer to pointers should be stored (like 'argv') to allow a sequence
					   * of commands ... ?? */
					} else {
					  /* Not SMRCL_CMD_STRING, test for ALT_SPEED_STRING. */
					  if (strcasecmp(atts[i], ALT_SPEED_STRING) == 0) {
					    /* ALT_SPEED_STRING found. */
					    data->mission.alt_speed[data->xml.cnt] = atof(atts[i+1]);
					  } else {
					    /* Unrecognized token. */
					    printf("X");
					  }
					}
				      }
				    }
				  }
				}
			      }
			    }
	  }
	  if (!data->xml.accdef && data->xml.cnt)
	    data->mission.acc[data->xml.cnt] = data->mission.acc[data->xml.cnt-1];
	  else
	    if (!data->xml.accdef)
	      data->mission.acc[data->xml.cnt] = DFLTACC;
	  
	  if (!data->xml.speeddef && data->xml.cnt)
	    data->mission.speed[data->xml.cnt] = data->mission.speed[data->xml.cnt-1];
	  else
	    if (!data->xml.speeddef)
	      data->mission.speed[data->xml.cnt] = DFLTSPEED;
	  
	  if (!data->xml.angledef /* && data->mission.turntype[data->xml.cnt] == NOTURN */) {
	    if (data->xml.cnt == 0)
	      data->mission.point[data->xml.cnt].ang = NOANGLE;
	    else {
	      double x = (data->mission.point[data->xml.cnt].e - data->mission.point[data->xml.cnt-1].e);
	      double y = (data->mission.point[data->xml.cnt].n - data->mission.point[data->xml.cnt-1].n);
	      if (y>0) 						
		data->mission.point[data->xml.cnt].ang =   90 - atan(x/y)*180.0/M_PI;
	      else
		if (y<0)
		  data->mission.point[data->xml.cnt].ang =  -90 - atan(x/y)*180.0/M_PI;
		else
		  if (y==0 && x>=0.0)
		    data->mission.point[data->xml.cnt].ang = 0;
		  else
		    if(y==0 && x<0.0)
		      data->mission.point[data->xml.cnt].ang = 180;
	      //nomalized angle between former point and this
	    }
	  }
	  data->xml.angledef = 0;
	  data->xml.speeddef = 0;
	  data->xml.accdef = 0;
	  data->xml.cnt ++;
	  printf("xmlcnt %d\n",  data->xml.cnt );
	}
      } else {
	printf("Unknown element or wrong placement <%s>. Attribute: %s \n",name, atts[i]);
      }
  
  data->xml.depth += 1;
}

///XML end element function
///This function is called everytime an endtag in the XML file has been found.
static void XMLCALL
endElement(void *userData, const char *name)
{
  datatype *data = (datatype *)userData;
  if(data->xml.depth == 2 && data->xml.route)
    data->xml.route = 0;
  data->xml.depth -= 1;
}

///Main function start
///Main function initializes everything and enters idle mode, waiting for gui to connect and start the mission
int main(const int argc,
#if 1
	 char * const * const argv
	 /* Without const'ing the "char" in order to avoid problem
	  * compiling the program - used in call to 'getopt', which
	  * has not "const'ed" the chars. */
#else
	 const char * const * const argv
#endif
)
{
  FILE *fp = NULL;
  int i=0, opt=0, e;
  char buf[BUFSIZ] = {0};
  char * missionfile = "";
  XML_Parser parser = XML_ParserCreate(NULL);
  int done = 0;
  
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  sem_init(&cmdsem,0,0);
  sem_init(&updatesem,0,0);

  //Catch broken pipe
  if (signal(SIGPIPE, sig_pipe) == SIG_ERR) {
    fprintf(stderr, "Unable to set up signal handler\n");
  }
  
  ct.port = 31001; 
  serv.port =  htons(2000);
  server_mode=0;
  stream2lineinit(&s2line);
  while (EOF != (opt = getopt(argc, argv, "t:l:s:"))) {
    switch (opt) {
    case 't':
      ct.port = 31001;
      if (optarg)
	ct.port = 31000 + atoi(optarg);
      break;
      
    case 's':
      server_mode = 1;
      go_flag = 0;
      serv.port = htons(2000);
      if (optarg)
	serv.port = htons(2000 + atoi(optarg));
      break;	 
      
    default:
      ;
    }
  }
  if (optind < argc) {
    missionfile=argv[optind];
  }
  init();
  if(argc == 1)
    {
      server_mode = 1;
      go_flag = 0;
    }
  //Start server for GUI to connect to
  if(server_mode)
    start_server();
  
  ct.hostname = "localhost";
  
  
  //Connect to smrdemo
  cl_connect();
  if(ct.error != 0)
    {
      printf("Socket connection error!\n");
      exit(1);
    }
  else 
    printf("Connected to MRC on %s!\n", ct.hostname);
  
  if (pthread_create(&recievethread, &attr, recievetask, 0))
  {
	  perror("can't start recieve thread");
	  exit(-1);
  }
 
  
  if(server_mode)
    {
      //Get latest sensor values to avoid sending zeros to GUI. Spawn threads to communication with the GUI.
      update_sensors();
      if (pthread_create(&udprxthread, &attr, udprxtask, 0))
	{
	  perror("can't start server thread");
	  exit(-1);
	}
      if (pthread_create(&updatethread, &attr, updatetask, 0))
	{
	  perror("can't start update thread");
	  exit(-1);
	}
      //Remove old files
      e = system("rm -f ../var/mission.xml");
      e = system("rm -f ../log/log");
    }
  else
    {
      go_flag = 1;
    }
  //Initialize the hako
  cl_send("set \"hakomanual\" 1\n");
  {char str[128];
  sprintf(str,"set \"hakoliftinggearstateref\" %i\n",DFLTTOOLPOS);
  cl_send(str);}
  cl_send("set \"hakopowertakeoffstateref\" 0\n");
  cl_send("set \"kalmanon\" 1\n");
  cl_send("stream 5 \"$hakoenginespeed\" \"$hakonavigationmode\" \"$odovelocity\" \"$xkalman\" \"$ykalman\" \"$thkalman\" \"$gpsnorthing\" \"$gpseasting\" \n");
  cl_send("stream 5 \"$hakosteeringangle\" \"$targetdist\" \"$gpsquality\" \"$gpssatellites\" \"$gpsdop\" \"$kalmanstatus\" \"$hakoliftinggearpos\"  \"$hakopowertakeoffspeed\" \n");
		
  
  while(!exit_flag)
    {
      //Idle mode, wait for GUI to send a command.
      while(!go_flag && !exit_flag)
	usleep(1000);
      if(exit_flag)
	break;
      //Initialize XML parser
      XML_SetUserData(parser, &data);
      XML_SetElementHandler(parser, startElement, endElement);
      reset();
      //Open mission files
      if(server_mode)
	fp = fopen("../var/mission.xml","r");
      else
	{
	  fp = fopen(missionfile,"r");
	}
      
      if(fp<=0)
	{
	  perror("Error opening XML file");
	  go_flag = 0;
	  continue;
	}
      
      //Start parsing the XML file
      do {
	size_t len = fread(buf, 1, sizeof(buf), fp);
	done = len < sizeof(buf);
	if (XML_Parse(parser, buf, len, done) == XML_STATUS_ERROR) {
	  fprintf(stderr,
		  "%s at line %i\n",
		  XML_ErrorString(XML_GetErrorCode(parser)),
		  (int)XML_GetCurrentLineNumber(parser));
	  go_flag = 0;
	  
	}
      } while (!done);
      fclose(fp);
      printf(" kalmaninit data  %f %f \n", data.kinit.dist, data.kinit.speed);
      XML_ParserReset(parser,NULL);
       if (  data.mission.point[0].ang==NOANGLE)  data.mission.point[0].ang=  data.mission.point[1].ang;
      //Printout data as interpreted from the XML file
      printf("Printout of missiondata/n");
      for (i=0; data.mission.point[i].n && i<MISSIONPOINTS; i++)
	printf("i:%i,N:%lf,E:%lf,a:%lf,sp:%lf,ac:%lf,t:%i,to:%i,p:%i,e:%i,a:%f,r:%f,td:%f,asl:%f,c:%p,as:%f\n",
	       i,
	       data.mission.point[i].n,
	       data.mission.point[i].e,
	       data.mission.point[i].ang,
	       data.mission.speed[i],
	       data.mission.acc[i],
	       data.mission.turntype[i],
	       data.mission.tools[i].toolpos,
	       data.mission.tools[i].power,
	       data.mission.engine_rpm_cmd[i],
	       data.mission.turn_angle[i],
	       data.mission.turn_radius[i],
	       data.mission.targetdist[i],
	       data.mission.arc_step_length[i],
	       data.mission.smrcl_cmd_str[i],
	       data.mission.alt_speed[i]);
      if (data.mission.smrcl_cmd_str[i]) {
	printf("  Embedded command string is : %s \n", data.mission.smrcl_cmd_str[i]);
      }
      
      //Set Hako in automatic mode and raise engine rpm
      cl_send("set \"hakomanual\" 0\n");
      cl_send("set \"hakoenginespeed\" 1600\n");
      

      //Start kalman initialization if defined in mission file.
      if(data.xml.kalmaninit)
	{
	  printf("kalmaninit\n");
	  cl_send("targethere\n");
	   update_sensors();
	  while(sensors.gpsquality == 0 && sensors.odovel == 0)
	    {
	      printf("Waiting for GPS fix\n");
	      sleep(1);
	      update_sensors();
	    }
	  cl_send("set \"kalmanon\" 1\n");
	   
	  if(data.kinit.type == DRIVE)
	    {
	      char tmp[100] = {0};
	      cl_send("targethere\n");	        
	      sprintf(tmp,"drive @v%lf @a%lf : ($drivendist > %lf)\n", data.kinit.speed, data.kinit.acc, data.kinit.dist-0.2);
	      /* Note: the above formatstring ('f') is not safe wrt. a buffer size of 100 chars and 'double' inputs.
	       * Note: "lf" format specifiers says "long double" - will be a problem if target machine has 128 bit floating point values?? */
	      printf("%s \n",tmp);
	      cl_cmdwait(tmp);
	    }
	  cl_send("set \"usekalmanodo\" 1\n");
	}
      //Request data logged as defined in the XML file
      if(data.log.logflag)
	{
	  if(data.log.kalmanflag)
	    cl_send("log \"$xkalman\" \"$ykalman\" \"$thkalman\"\n");
	  if(data.log.gpsflag)
	    cl_send("log \"$gpseasting\" \"$gpsnorthing\"\n");
	  if(data.log.odoflag)
	    cl_send("log \"$odox\" \"$odoy\" \"$odoth\" \"$odovelocity\" \"$hakosteeringangle\" \"$hakocvtpulses\" \n");
	  if(data.log.imuflag)
	    {
	      cl_send("log \"imuroll\" \"imupitch\" \"imuyaw\" \"imuaccx\" \"imuaccy\" \"imuaccz\" \"imutemp\"\n");
	    }
	  cl_send("control \"startlog\"\n");
	}
      
      //Begin mission and drive to all points
      for(i=0; data.mission.point[i].n && i<MISSIONPOINTS && !exit_flag; i++)
	{
	  /* Send any embedded command ... */
	  if (data.mission.smrcl_cmd_str[i]) 
          {
            /* Non-NULL pointer, i.e. command present. Send it! */
            /* Append '\n' - otherwise the program seems to hang! Decide whether the "store" should handle "\n"
            * or keep this implementation. */
            char str[500] = {0};
            
            sprintf(str, "%.256s\n", data.mission.smrcl_cmd_str[i]);

            fprintf(stdout, "Sending embedded SMR-CL command: %s \n", str);
            cl_send(str);
            fprintf(stdout, "  after sending command.\n");
          }

	  //Drive to mission point
	  if (data.mission.turntype[i-1] == NOTURN || i == 0) {
	    Vector pos = {0};
	    
	    pos.x = data.mission.point[i].e;
	    pos.y = data.mission.point[i].n;
	    drivetowards(pos,
			 data.mission.point[i].ang * M_PI / 180.0  /* Angle in radians required! */,
			 data.mission.targetdist[i],
			 data.mission.speed[i],
			 data.mission.acc[i]);
	    /* Note that with this definition, the 'targetdist' and 'speed' for waypoint number 'i'
	     * will be shared between the straight line segment leading up to turn and the "gps_wide"
	     * turn specified for waypoint. Change?? */
	  }
	  
	  
	  //Begin execution of tasks at point
	  //execute pause
	  if(data.mission.pause[i] !=0)
	    {
	      char str[128];
	      cl_send("idle\n");
	      sprintf(str,"wait %i\n",data.mission.pause[i]);
	      cl_send(str);
	    }
	  
	  /* Set engine speed if the route plan has requested it */
	  {
	    if (ENGINE_HIGH_RPM_CMD == data.mission.engine_rpm_cmd[i]) {
	      /* It is requested to set the engine speed to high. */
	      printf("Setting engine rpm to HIGH on user request!\n");
	      cl_send("set \"hakoenginespeed\" 1900\n");
	    }
	    if (ENGINE_LOW_RPM_CMD == data.mission.engine_rpm_cmd[i]) {
	      /* It is requested to set the engine speed to low. */
	      printf("Setting engine rpm to LOW on user request! Power may be insufficient!\n");
	      cl_send("set \"hakoenginespeed\" 900\n");
	    }
	  }

	  //Move tool into position
	  {
	    char str[128];
	    sprintf(str,"set \"hakoliftinggearstateref\" %i\n",data.mission.tools[i].toolpos);
	    cl_send(str);
	    
	    if(data.mission.tools[i].power == NOCHANGE)
	      cl_send("set \"hakopowertakeoffstateref\" 255\n");
	    else
	      if (data.mission.tools[i].power == ON)
		cl_send("set \"hakopowertakeoffstateref\" 1\n");
	      else if(data.mission.tools[i].power == OFF)
		cl_send("set \"hakopowertakeoffstateref\" 0\n");
	  }						 
	  
          //Begin fishtail turntype
          if (data.mission.turntype[i] == FISHTURN && data.mission.point[i+1].n)
          {
            double dist, de, dn, anglefrom, angleto;
            if (0)
            { /* old version with straight reverse leg */
              dn = data.mission.point[i].n-data.mission.point[i+1].n;
              de = data.mission.point[i].e-data.mission.point[i+1].e;
              dist = sqrt(sqr(de)+sqr(dn));
              angleto =  data.mission.point[i+1].ang;
              anglefrom = data.mission.point[i].ang;
              fishtale( dist, anglefrom, angleto);
            }
            else
            { /* new fishtail - calculate signed distance to next row */
              double a,b,c, vx, vy;
              /* current heading (before turn) */
              anglefrom = data.mission.point[i].ang * M_PI / 180.0;
              /* calculate line equation ax + by + c = 0 */
              vx = cos(anglefrom);
              vy = sin(anglefrom);
              c = vy * data.mission.point[i].e - vx * data.mission.point[i].n;
              a = -vy;
              b = vx;
              /* get signed distance from line - positive to the left */
              dist = a * data.mission.point[i+1].e + b * data.mission.point[i+1].n + c;
              /* call the new fishtail function - at commanded speed */
              fishtail2(dist, data.mission.speed[i+1] /*sensors.odovel*/);
            }
            /* continue towards start of next mission line (at specified speed */
            driveto(data.mission.point[i+1].n,
                    data.mission.point[i+1].e,
                    data.mission.point[i+2].ang,
                    data.mission.speed[i],
                    TURNACC);
          }
	  /* Begin omega turn */
	  if (data.mission.turntype[i] == OMEGATURN && data.mission.point[i+1].n) {
	    TURN_omega(data.mission.point[i], data.mission.point[i+1], data.mission.point[i+2]); 
	  }
	  if (data.mission.turntype[i] == ARCTURN && data.mission.point[i+1].n) {
	    printf("Turntype ARC chosen.\n");
	    TURN_arc(data.mission.point[i],
		     data.mission.turn_angle[i],
		     data.mission.turn_radius[i],
		     data.mission.point[i+1],
		     data.mission.point[i+2]); 
	  }
	  if (data.mission.turntype[i] == WIDETURN && data.mission.point[i+1].n) {
	    printf("Turntype WIDE chosen.\n");
	    TURN_wide(data.mission.point[i], data.mission.point[i+1], data.mission.point[i+2]); 
	  }
	  if (data.mission.turntype[i] == GPS_WIDE_TURN && data.mission.point[i+1].n) {
	    printf("Turntype GPS_WIDE chosen.\n");
	    TURN_gps_wide(data.mission.point[i],
			  data.mission.turn_angle[i],
			  data.mission.turn_radius[i],
			  data.mission.speed[i],
			  data.mission.targetdist[i],
			  data.mission.arc_step_length[i],
			  data.mission.point[i+1]);
	    /* Speed should be added to argument list! */
	  }
	  if (data.mission.turntype[i] == GPS_FISHTAIL_TURN && data.mission.point[i+1].n) {
	    printf("Turntype GPS_FISHTAIL chosen.\n");
	    printf("  - currently implemeneted as a GPS_WIDE turn.\n");
	    TURN_gps_fishtail(data.mission.point[i],
			      data.mission.point[i+1],
			      data.mission.point[i+2],
			      data.mission.turn_radius[i],
			      data.mission.speed[i],
			      data.mission.acc[i],
			      data.mission.alt_speed[i]);
	    /* Change function to use 'Vector' and angles in radians? */
	  }
	  if (data.mission.turntype[i] == GPS_LINK_LANES_TURN && data.mission.point[i+1].n) {
	    printf("Turntype GPS_LINK_LANES chosen.\n");
	    TURN_gps_link_lanes(data.mission.point[i],
				data.mission.point[i+1],
				data.mission.point[i+2],
				data.mission.turn_radius[i],
				data.mission.speed[i],
				data.mission.acc[i]);
	  }
	}
      //Mission complete
      //Stop hako, raise tools and save log.
      cl_cmdwait("stop\n");
      {char str[128];
      sprintf(str,"set \"hakoliftinggearstateref\" %i\n",DFLTTOOLPOS);
      cl_send(str);}
      cl_send("set \"hakopowertakeoffstateref\" 0\n");
      cl_send("set \"hakoenginespeed\" 900\n");
      cl_send("set \"hakomanual\" 1\n");
      cl_cmdwait("control \"savelog\"\n");
      cl_send("control \"resetlog\"\n");
      cl_send("control \"stoplog\"\n");
      cl_send("control \"removelogvars\"\n");
      { // add date and time to log filename
	time_t now;
	struct tm *tm_ptr;
	char tmp[32] = {0};
	int e;
	time(&now);
	tm_ptr = localtime(&now);
	e = system("chmod 777 log");
	sprintf(tmp,"cp log ../log/%02d%02d%02d-%02d%02d-log\n", 
		tm_ptr->tm_year%100, tm_ptr->tm_mon+1,tm_ptr->tm_mday,tm_ptr->tm_hour, tm_ptr->tm_min);
	usleep(100000);
	e = system(tmp);
	e = system("cp log ../log/\n");
	e = system("chmod 777 ../log/*");
      }
      
      reset();
      go_flag = 0;
      if(!server_mode)
	exit_flag = 1;
      while(!go_flag && !exit_flag) //Enter idle loop waiting for new mission or exit
	usleep(1000);
      
    }
  printf("Exiting\n");
  cl_cmdwait("stop\n");
  sleep(1);
  close(serv.s);
  cl_disconnect(&ct);
  XML_ParserFree(parser);
  printf("\nAll done! Thank you! Come again!\n");
  exit(0);
}

///UDP receive thread
///This thread handles all incomming communication from the GUI
void *udprxtask(void *not_used)
{
  while(1)
    {
      //receive, parse and carry out mission.
      bzero(serv.buf,STRING_LENGTH);
      if( recvfrom(serv.s, serv.buf, sizeof(serv.buf), 0, (struct sockaddr *) &serv.client, &serv.client_address_size) < 0 )
	{
	  printf("Error recieving UDP message\n");
	  exit(4);
	}
      fprintf(stderr,"msg recieved: %s\n",serv.buf);
      if(strcmp(serv.buf,HELLOSTR) == 0)
	{
	  char buf[10];
	  int i;
	  update_sensors();
	  fprintf(stderr,"Sending Hello\n");
	  buf[0] = 1;
	  buf[1] = 'H';
	  for(i=0;i<8;i++)
	    buf[i+2] = 0;
	  sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
	}
      else if(strcmp(serv.buf,KALMANSTRING) == 0)
	{
	  cl_send("set \"kalmanon\" 1\n");
	  cl_send("set \"usekalmanodo\" 1\n");
	}
      else if(strcmp(serv.buf,GOSTR) == 0)
	{
	  go_flag = 1;
	  pause_flag = 0;
	  cl_send("set \"hakomanual\" 0\n");
	  cl_send("set \"hakoenginespeed\" 1600\n");
	}
      else if(strcmp(serv.buf,EXITSTR) == 0)
	exit_flag = 1;
      else if(strcmp(serv.buf,PAUSESTR) == 0)
	pause_flag = 1;
      else if(strcmp(serv.buf,SLOGSTR) == 0)
	{
	  if(go_flag)
	    {
	      cl_send("control \"savelog\"\n");
	      fprintf(stderr,"control \"savelog\"\n");
	      { // add date and time to log filename
		time_t now;
		struct tm *tm_ptr;
		char tmp[32] = {0};
		time(&now);
		tm_ptr = localtime(&now);
		sprintf(tmp,"cp log ../log/%02d%02d%02d-%02d%02d-log\n", 
			tm_ptr->tm_year, tm_ptr->tm_mon+1,tm_ptr->tm_mday,tm_ptr->tm_hour, tm_ptr->tm_min);
		system(tmp);
		system("cp log ../log/\n");
	      }
	    }
	}
    }
  
  fprintf(stderr,"UDPRXTASK TERMINATED\n");
  pthread_exit(0);
}

///GUI update thread
///This thread sends periodical updates to the GUI with all data
void *updatetask(void *not_used)
{
  union
  {
    double db;
    unsigned char ch[8];
  }a;
  char buf[100];
  int i;
  //struct timeval tp;
  //double t1,t2;
 
  while(1)
    {
      //Get values from smrdemo
      update_sensors();
      sem_wait(&updatesem);
      
      //Wrap data and send to GUI
      a.db = sensors.enginespeed;
      buf[0] = 9;
      buf[1] = 'E';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      buf[0] = 2;
      buf[1] = 'N';
      if (sensors.navmode==1)
        buf[2] = 'M';
      else
        buf[2]='A';
      for(i=0;i<7;i++)
	buf[i+3] = 0;
      sendto( serv.s, buf,10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.odovel;
      buf[0] = 9;
      buf[1] = 'V';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.xkalman;
      buf[0] = 9;
      buf[1] = 'X';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.ykalman;
      buf[0] = 9;
      buf[1] = 'Y';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.thkalman;
      buf[0] = 9;
      buf[1] = 'T';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.gpsn;
      buf[0] = 9;
      buf[1] = 'n';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.gpse;
      buf[0] = 9;
      buf[1] = 'e';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.steeringangle/1800*M_PI;
      buf[0] = 9;
      buf[1] = 'S';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
		
      buf[0] = 2;
      buf[1] = 'L';
      buf[2] = sensors.liftingGearPos;
      for(i=0;i<7;i++)
	buf[i+3] = 0;
      sendto( serv.s, buf,10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      buf[0] = 2;
      buf[1] = 'P';
      buf[2] = sensors.powerTakeoffSpeed;
      for(i=0;i<7;i++)
	buf[i+3] = 0;
      sendto( serv.s, buf,10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      buf[0] = 2;
      buf[1] = 'q';
      buf[2] = sensors.gpsquality;
      for(i=0;i<7;i++)
	buf[i+3] = 0;
      sendto( serv.s, buf,10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      buf[0] = 2;
      buf[1] = 's';
      buf[2] = sensors.satellites;
      for(i=0;i<7;i++)
	buf[i+3] = 0;
      sendto( serv.s, buf,10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.dop;
      buf[0] = 9;
      buf[1] = 'd';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.targetdist;
      buf[0] = 9;
      buf[1] = 'D';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );
      
      a.db = sensors.kalmanstatus;
      buf[0] = 9;
      buf[1] = 'K';
      for(i=0;i<8;i++)
	buf[i+2] = a.ch[7-i];
      sendto( serv.s, buf, 10, 0,(struct sockaddr *) &serv.client, sizeof(serv.client ) );    
    }
  
  fprintf(stderr,"UPDATETASK TERMINATED\n");
  pthread_exit(0);
}

///Recieve thread
void *recievetask(void *not_used)
{
 
  while(1)
  {
     s2line.Nib = recv(ct.sockfd, s2line.ib, STRING_LENGTH, 0);
     if (s2line.Nib >0)
       stream2line(&s2line);
     while (s2line.nlines >0)
     {
       getnextline(ct.answer,&s2line);
       /* printf("received: %s \n",ct.answer); */
       if (strncmp(ct.answer,"syncevent",9)==0)
       {
         sem_post(&cmdsem);
       }
       else if(strncmp(ct.answer,"stream",6)==0)
       {
         read_sensors();
         sem_post(&updatesem);
       }
       else
         ;
     }	      
  }
  
  fprintf(stderr,"Recievetask TERMINATED\n");
  pthread_exit(0);
}
