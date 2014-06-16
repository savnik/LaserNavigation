/** \file robotinit.c
 * \brief Read the XML configuration for the robots at the Department of 
 * Automation, Technical university of Denmark.
 * 
 * The parsing is based om Expat.
 *
 * \author Lars Valdemar Mogensen
 * \author Nils Axel Andersen
 * \date 19/05-2006
 */
 
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#include "../robotinc/odometry.h"
#include "../robotinc/motioncontrol.h"
#include "linesensor.h"
#include "expat.h"

#include "../robotinc/filter.h"
#include "../robotinc/libgps.h"
#include "componentserver.h"
#include "interp.h"
#include "administration.h"
/// Main config file for SMRdemo placed in the robots 
/// local configuration directory.
#define CONFFILE 	"/usr/local/smr/calib/robot.conf"
/// Backup config file for SMRdemo placed in the calib directory 
/// where the program is called.
#define CONFFILE1 	"calib/robot.conf"

// Data for expat
/// Definition of buffersize for the Expat and the max size of the 
/// configuration file.
#define BUFFSIZE	8192

/// Buffer to hold the XML file.
char Buff[BUFFSIZE];

/// Variable to keep track of how deep into the XML struct the parser is.
int Depth;
/// Global variable to indicate the number of elements in the defined matrices.
int matrix_size;
int debug;

/** \brief Holds data from the XML character fields for later parsing.
 * 
 * The struct contains a data area an two pointers to keep track of
 * how much data has been read. 
 */
typedef struct {

   char* start; ///< Pointer to the current starting point of the buffer. 
   char* end; ///< Pointer to the current end point of the buffer.

   char dataField[BUFFSIZE]; ///< Character array to hold the XML character information.

}charField;

/// Definition of the character field struct for Expat XML pasring. 
charField field;
// End data for expat

// #define conftest
#ifdef conftest
enum robottype{DIFFERENTIAL=0,ACKERMAN,VELOMEGA};

odotype odo;

motiontype motcon;

struct {
   double velscalel,velscaler;
   double kp,ki;
}motorcontrol;

struct{
   int type;
   char name[256];
}robotinfo;

double linesensorsize;

struct {
   double k_filt;
}line_data;

IAU_kalman_odotype kalmanodo;

gpsmousetype gps;
#else
extern struct{
   int type;
   char name[256];
}robotinfo;

extern find_line_type line_data;	      
extern odotype odo;
extern IAU_kalman_odotype kalmanodo;
extern motiontype motcon;
extern int linesensorsize;
extern struct {
   double velscalel,velscaler;
   double kp,ki;
}motorcontrol;

extern componentservertype lmssrv,camsrv;
extern gpsmousetype gps;
extern struct{
	       int use;
	       }gpssim;
extern struct{
	      int on;
	      }speak;
extern symboltable sysvar;
// To be put in guidemark module struct
extern int gm_config;
extern int gm_status;
extern int gm_run;
extern int gm_use;

// To be put in ir sensor struct
extern int ir_config;
extern int ir_status;
extern int ir_run;
extern int ir_use;

#endif

/** \brief Comparison of the current tag and the supplied string.
 *
 * \param [in] *s Pointer to the string to compare to.
 * \param [in] *tag Pointer to the parsed XML tag to compare.
 * \return 0 on faliure and 1 on succes.
 */
int tag(char *s,const char *tag){
  // printf(" tag %s  %s  \n",s,tag);
  if ((strcmp(s,tag)==0)) 
    return 1;
  else
    return 0;
}

/** \brief Comparison of a parameter in the attribute array and the supplied string.
 *
 * \param [in] *s Pointer to the string to compare to.
 * \param [in] i Number of the parameter in the attribute array.
 * \param [in] *attr Pointer to the parsed XML attribute array to compare to.
 * \return 0 on faliure and 1 on succes.
 */
int par(char *s,int i,const char **attr){
  //printf("par %s  %s  \n",s,attr[i]);
  if ((strcmp(s,attr[i])==0)) 
    return 1;
  else
    return 0;
}

/** \brief Parsing of the attributes in a "irsensor" tag.
 *
 * \todo Implement the initialization of this module.
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the ir sensor flags "ir_flags".
 */ 
void irsensorfunc(const char **attr){
 
   printf("Module : IR sensor\n");
   //   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);

   ir_config = 1;
   ir_status = 1;
   ir_run = 1;
   ir_use = 1;
}



/** \brief Parsing of the attributes in a "robotinfo" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the robotinfo struct.
 */
void robotinfofunc(const char **attr){
  int i;
  
  printf("Module : robotinfo\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2)
  {
    if (par("type",i,attr)) {
       if(strcmp(attr[i+1],"differential")==0) robotinfo.type=DIFFERENTIAL;
       else if (strcmp(attr[i+1],"ackerman")==0) robotinfo.type=ACKERMAN;
       else if (strcmp(attr[i+1],"velomega")==0) robotinfo.type=VELOMEGA;
       else if (strcmp(attr[i+1],"gripper")==0) robotinfo.type=GRIPPER;
       else if (strcmp(attr[i+1],"hexacopter")==0) robotinfo.type=HEXACOPTER;
       else fprintf(stderr,"robot type not known\n");
       printf("   type: %s\n",attr[i+1]);
    }
    if (par("name",i,attr)){ 
       strcpy(robotinfo.name,attr[i+1]);
       printf("   name: %s\n",robotinfo.name);
    } 
  }
  
}    

/** \brief Parsing of the attributes in a "odometry" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the odo struct.
 */
void odometryfunc(const char **attr){
  int i;
  
  printf("Module : odometry\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2)
  {
    if (par("cl",i,attr)) odo.cl=atof(attr[i+1]); 
    else if (par("cr",i,attr)) odo.cr=atof(attr[i+1]); 
    else if (par("w",i,attr)) odo.w=atof(attr[i+1]); 
    else if (par("robotlength",i,attr)) odo.robotlength=atof(attr[i+1]); 
    else if (par("steeringangleoffset",i,attr)) odo.steeringangleoffset=atof(attr[i+1]); 
    else if (par("ts",i,attr)) odo.ts=atof(attr[i+1]); 
    else if (par("maxtick",i,attr)) odo.maxtick=atof(attr[i+1]); 
    else if (par("control",i,attr)) odo.control=atof(attr[i+1]); 
    else if (par("enctype",i,attr)) odo.enctype=atoi(attr[i+1]); 
    else if (par("encwheel",i,attr)) odo.encwheel=atoi(attr[i+1]); 
    else if (par("gyrotype",i,attr)) odo.gyrotype=atoi(attr[i+1]);
     
    
  }
} 
/** \brief Parsing of the attributes in a "varscale" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the internvar table.
 */
void scalefunc(const char **attr){
  int i,index;
  char name[256],res[256];
  name[0]=0;
  double gain=1,offset=0;
  
  printf("Module : varscale\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2)
  {
    if (par("name",i,attr)) strcpy(name,attr[i+1]); 
    else if (par("offset",i,attr)) offset=atof(attr[i+1]); 
    else if (par("gain",i,attr)) gain=atof(attr[i+1]);    
  }
  if (name[0]!=0){
  
  symrec * var;
      index=isarray(res,name);
      if (index==-1) index=0;
     var=getsym (res, sysvar);
     if (var){
        *(var->value.pvar+index+var->varsize)=offset;
	*(var->value.pvar+index+2*var->varsize)=gain;
	printf("name %s gain %f  offset %f \n",name,gain,offset);
     }
     else
       printf("name %s not found \n",name);
  
  }
} 

/** \brief Parsing of the attributes in a "routeinput" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the internvar table.
 */
void routefunc(const char **attr){
  int i;
  char name[256],parlist[256],param[256],var[256],res[256];
  int index;
  name[0]=0;
  parlist[0]=0;
  param[0]=0;
  var[0]=0;
  printf("Module : route\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2)
  {
    if (par("module",i,attr)) strcpy(name,attr[i+1]); 
    else if (par("parlist",i,attr))  strcpy(parlist,attr[i+1]);
    else if (par("par",i,attr))  strcpy(param,attr[i+1]);
    else if (par("var",i,attr))  strcpy(var,attr[i+1]); 
  }
  if (name[0]==0)
    printf("module missing\n");
  else if (parlist[0]==0)
    printf("parlist missing\n");
  else if (param[0]==0)
    printf("par missing\n");
  else if (var[0]==0)
    printf("var missing\n");
  else {
    parameterelem * parptr;
    symrec *s;
    printf("module %s parlist %s par %s var %s \n",name,parlist,param,var);
    parptr=findpar(name,parlist,param);
    if (parptr){
      index=isarray(res,var);
      if (index==-1) index=0;
      if((s=getsym(res, sysvar))!=NULL){
          *( (double **)  parptr->parptr)=s->value.pvar+index;
      }
     
      else
        printf(" input variable not found \n");
    }
    else
      printf("parameter not found \n");
  }
} 

/** \brief Parsing of the attributes in a "setpar" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the internvar table.
 */
void setparfunc(const char **attr){
  int i;
  char name[256],parlist[256],param[256],val[256],res[256];
  int index;
  name[0]=0;
  parlist[0]=0;
  param[0]=0;
  val[0]=0;
  printf("Module : setpar\n");
//   for(i=0;attr[i];i+=2)
//    printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2)
  {
    if (par("module",i,attr)) strcpy(name,attr[i+1]); 
    else if (par("parlist",i,attr))  strcpy(parlist,attr[i+1]);
    else if (par("par",i,attr))  strcpy(param,attr[i+1]);
    else if (par("val",i,attr))  strcpy(val,attr[i+1]); 
  }
  if (name[0]==0)
    printf("module missing\n");
  else if (parlist[0]==0)
    printf("parlist missing\n");
  else if (param[0]==0)
    printf("par missing\n");
  else if (val[0]==0)
    printf("val missing\n");
  else {
    parameterelem * parptr;
    printf("module %s parlist %s par %s val %s \n",name,parlist,param,val);
    parptr=findpar(name,parlist,param);
    if (parptr){
          *( (double *)  parptr->parptr)=atof(val);
    }
    else
      printf("parameter not found a \n");
  }
} 
   
/** \brief Parsing of the attributes in a "kalmanodometry" tag.
 *
 * \deprecated This is not used any more. It has been replaced 
 * by the filter tag.
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the kalmanodo struct.
 */
void kalmanodometryfunc(const char **attr){
  int i;
  
  printf("Module : kalmanodometry\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if (par("measurementnoise_std_x",i,attr)) 
       kalmanodo.measurement_noise_std_x=atof(attr[i+1]); 
    else if (par("measurementnoise_std_y",i,attr)) 
       kalmanodo.measurement_noise_std_y=atof(attr[i+1]); 
    else if (par("processnoise_std_stearingangle",i,attr)) 
       kalmanodo.process_noise_std_steering_angle=atof(attr[i+1]); 
    else if (par("processnoise_std_dist",i,attr)) 
       kalmanodo.process_noise_std_dist=atof(attr[i+1]); 
  }
      
  kalmanodo.config = 1;
  kalmanodo.status = -1;
}    

/** \brief Parsing of the attributes in a "motioncontrol" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the motcon struct.
 */
void motioncontrolfunc(const char **attr){
  int i;
  
  printf("Module : motion control\n");
//   for(i=0;i<x->n;i++)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  motcon.steerDelay = 0.0;  /* default  delay for steering control (seconds) - 0 means disabled */
  motcon.steerVel = 35.0 * M_PI / 180.0;    /* default change speed of the steering (35 deg/sec for Hako) */
  motcon.steerHistCnt = 0;
  for(i=0;attr[i];i+=2){
    if (par("ts",i,attr)) motcon.ts=atof(attr[i+1]); 
    else if (par("line_gain",i,attr)) motcon.line_gain=atof(attr[i+1]); 
    else if (par("line_tau",i,attr)) motcon.line_tau=atof(attr[i+1]); 
    else if (par("line_alfa",i,attr)) motcon.line_alfa=atof(attr[i+1]); 
    else if (par("wall_gain",i,attr)) motcon.wall_gain=atof(attr[i+1]); 
    else if (par("wall_tau",i,attr)) motcon.wall_tau=atof(attr[i+1]); 
    else if (par("wall_alfa",i,attr)) motcon.wall_alfa=atof(attr[i+1]); 
    else if (par("kp",i,attr)) motcon.kp=atof(attr[i+1]); 
    else if (par("gain",i,attr)) motcon.gain=atof(attr[i+1]); 
    else if (par("tau",i,attr)) motcon.tau=atof(attr[i+1]); 
    else if (par("alfa",i,attr)) motcon.alfa=atof(attr[i+1]); 
    else if (par("drive_kangle",i,attr)) motcon.kangle_drive=atof(attr[i+1]);
    else if (par("drive_kdist",i,attr))  motcon.kdist_drive=atof(attr[i+1]); 
    else if (par("steer_delay",i,attr))
    {
      motcon.steerDelay=atof(attr[i+1]);
      printf("Steering: motion control steering delay set to %.3fsec\n", motcon.steerDelay);
    }
    else if (par("steer_vel",i,attr))
    {
      motcon.steerVel=atof(attr[i+1]);
      printf("Steering: motion control steering velocity set to %.4frad/s (is %.1fdeg)\n", motcon.steerVel, motcon.steerVel * 180.0 / M_PI);
    }
    else if (par("w",i,attr)) motcon.w=atof(attr[i+1]); 
    else if (par("robotlength",i,attr)) motcon.robotlength=atof(attr[i+1]); 
    else if (par("lim",i,attr)) motcon.lim=atof(attr[i+1]); 
    else if (par("stopdist",i,attr)) motcon.stopdist=atof(attr[i+1]); 
    else if (par("alarmdist",i,attr)) motcon.alarmdist=atof(attr[i+1]); 
    else if (par("velcmd",i,attr)) motcon.velcmd=atof(attr[i+1]); 
    else if (par("acccmd",i,attr)) motcon.acccmd=atof(attr[i+1]); 
    else if (par("nolinedist",i,attr)) motcon.nolinedist=atof(attr[i+1]); 
    else printf("Parameter  %s  not known\n" ,attr[i]); 
    }
}    

/** \brief Parsing of the attributes in a "motorcontrol" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the motorcontrol struct.
 */
void motorcontrolfunc(const char **attr){
  int i;
  
  printf("Module : motor control\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if (par("velscalel",i,attr)) motorcontrol.velscalel=atof(attr[i+1]); 
    else if (par("velscaler",i,attr)) motorcontrol.velscaler=atof(attr[i+1]); 
    else if (par("kp",i,attr)) motorcontrol.kp=atof(attr[i+1]); 
    else if (par("ki",i,attr)) motorcontrol.ki=atof(attr[i+1]);      
    else printf("Parameter  %s  not known\n" ,attr[i+1]); 
  }
}

/** \brief Parsing of the attributes in a "linesensor" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the linesensor variables.
 */    
void linesensorfunc(const char **attr){
  int i;
  
  printf("Module : line sensor\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if (par("size",i,attr)) linesensorsize=atof(attr[i+1]); 
    else if (par("k_filt",i,attr)) line_data.k_filt=atof(attr[i+1]); 
    else printf("Parameter  %s  not known\n" ,attr[i+1]); 
  }
}    

/** \brief Parsing of the attributes in a "filter" tag.
 *
 * \todo Finish the configuration method to take the dimensions
 * of the filter from the XML file along with the noise standard 
 * deviatin.
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the kalmanodo struct.
 */
void filterfunc(const char **attr){
  int i;
  
  printf("Module : filter\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if      (par("measurementnoise_std_x",i,attr)) 
       kalmanodo.measurement_noise_std_x=atof(attr[i+1]); 
    else if (par("measurementnoise_std_y",i,attr)) 
       kalmanodo.measurement_noise_std_y=atof(attr[i+1]); 
    else if (par("processnoise_std_stearingangle",i,attr)) 
       kalmanodo.process_noise_std_steering_angle=atof(attr[i+1]); 
    else if (par("processnoise_std_dist",i,attr)) 
       kalmanodo.process_noise_std_dist=atof(attr[i+1]);
    
    else if (par("type",i,attr)) strcpy(kalmanodo.type,attr[i+1]);
    else if (par("run",i,attr)) kalmanodo.run=atoi(attr[i+1]);
    else if (par("use",i,attr)) kalmanodo.use=atoi(attr[i+1]);
    
    
    else if (par("size_input",i,attr)) 
       kalmanodo.size_input=atof(attr[i+1]);
    else if (par("size_state",i,attr)) 
       kalmanodo.size_state=atof(attr[i+1]);
    else if (par("size_noise_process",i,attr)) 
       kalmanodo.size_noise_process=atof(attr[i+1]);

    else if (par("size_output",i,attr)) 
       kalmanodo.size_output=atof(attr[i+1]);
    else if (par("size_noise_output",i,attr)) 
       kalmanodo.size_noise_output=atof(attr[i+1]);
    
    else if (par("gpsoffset_easting",i,attr)) 
       kalmanodo.offset_x=atof(attr[i+1]);
    else if (par("gpsoffset_northing",i,attr)) 
       kalmanodo.offset_y=atof(attr[i+1]);
    
  }

  kalmanodo.config = 1;
  kalmanodo.status = 0;
}    

/** \brief Parsing of the attributes in a "matrix" tag.
 *
 * \todo Combine this with the filter to use it for initialization.
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the matrix_size value.
 */
void matrixfunc(const char **attr){
  int i;
  int row,col;
  
  printf("   Element : matrix\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if      (par("name",i,attr)) printf("   Name %s\n",attr[i+1]);
    else if (par("size",i,attr)) {
       sscanf(attr[i+1],"%i %i",&row,&col);
       matrix_size = row*col;
    }
    else if (par("type",i,attr)) ;//printf("   Type %s\n",attr[i+1]);
    else printf("   Parameter  %s  not known\n" ,attr[i+1]); 
  }
}    

/** \brief Parsing of the attributes in a "gpsmouse" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the gps struct.
 */
void gpsmousefunc(const char **attr){
  int i;

  printf("Module : gpsmouse\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if      (par("hostname",i,attr)) strcpy(gps.hostname,attr[i+1]);
    else if (par("port",i,attr)) gps.port=atoi(attr[i+1]);
    else if (par("SBAS",i,attr)) gps.EGNOS=atoi(attr[i+1]);
    else if (par("run",i,attr)) gps.run=atoi(attr[i+1]);
    else if (par("use",i,attr)) gps.use=atoi(attr[i+1]);
    else    printf("   Parameter  %s  not known\n" ,attr[i+1]); 
  }
  
  gps.client.hostname=gps.hostname;
  gps.client.port=gps.port;
  
  // Indicate status
  gps.config = 1;
  gps.status = -1;

  printf("   Hostname: %s \n",gps.hostname);
  printf("   Port    : %i \n",gps.port);
  printf("   SBAS    : %i \n",gps.EGNOS);

}
/** \brief Parsing of the attributes in a "gpssim" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the gps struct.
 */
void gpssimfunc(const char **attr){
  int i;

  printf("Module : gpssim\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if      (par("use",i,attr))  gpssim.use=atoi(attr[i+1]);  
    else    printf("   Parameter  %s  not known\n" ,attr[i+1]); 
  }
  
  

}


/** \brief Parsing of the attributes in a "speak" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the gps struct.
 */
void speakfunc(const char **attr){
  //int i;

  printf("Module : speak\n");
  speak.on=1;
  
  printf("   Speak on  \n");

}




/** \brief Parsing of the attributes in a "laserserver" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the gps struct.
 */
void laserserverfunc(const char **attr){
  int i;

  printf("Module : laserserver\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if      (par("hostname",i,attr)) strcpy(lmssrv.host,attr[i+1]);
    else if (par("port",i,attr)) lmssrv.port=atoi(attr[i+1]);
    else if (par("run",i,attr)) lmssrv.run=atoi(attr[i+1]);
    else if (par("use",i,attr)) lmssrv.use=atoi(attr[i+1]);
    else    printf("   Parameter  %s  not known\n" ,attr[i+1]); 
  }
  
 
  // Indicate status
  lmssrv.config = 1;
  lmssrv.status = -1;

  printf("   Hostname: %s \n",lmssrv.host);
  printf("   Port    : %i \n",lmssrv.port);
 

}

/** \brief Parsing of the attributes in a "camserver" tag.
 *
 * \param [in] *attr Pointer to the XML attribute array to parse.
 * \return Parsed data is returned in the camsrv struct.
 */
void camserverfunc(const char **attr){
  int i;

  printf("Module : camserver\n");
//   for(i=0;attr[i];i+=2)
//     printf("  %s    %s  \n",attr[i],attr[i+1]);
  for(i=0;attr[i];i+=2){
    if      (par("hostname",i,attr)) strcpy(camsrv.host,attr[i+1]);
    else if (par("port",i,attr)) camsrv.port=atoi(attr[i+1]);
    else if (par("run",i,attr)) camsrv.run=atoi(attr[i+1]);
    else if (par("use",i,attr)) camsrv.use=atoi(attr[i+1]);
    else    printf("   Parameter  %s  not known\n" ,attr[i+1]); 
  }
  
 
  // Indicate status
  camsrv.config = 1;
  camsrv.status = -1;
   gm_config = 1;
   gm_status = 1;
   gm_run = 1;
   gm_use = 1;
  printf("   Hostname: %s \n",camsrv.host);
  printf("   Port    : %i \n",camsrv.port);
 

}



/** \brief Handler for a XML start tag.
 *
 * This is a function defined for the Expat XML paser. The function
 * is used to tell the parser how to handle a start tag.
 * This function is used to parse the tags just after they have been
 * read. The user data space is cleared here.
 * \param [in] *userData Pointer to a common data space for user data.
 * \param [in] *el Pointer to the current XML tag to parse.
 * \param [in] *attr Pointer to the current XML attributes to parse.
 */
void
start(void *userData, const char *el, const char **attr) {

  // Defining the struct to return the configuration data in
  charField *data=(charField *)userData;
  
  // Keep track of the buffer used for data storage
  data->start = data->dataField;
  data->end = data->start;
  *(data->start) = '\0';
  
  // Keep track of the XML tree depth
  Depth++;
  
  //Parsing of the attributes in the tag
  if      (tag("robotinfo",el)) robotinfofunc(attr);
  else if (tag("varscale",el)) scalefunc(attr); 
  else if (tag("routeinput",el)) routefunc(attr); 
  else if (tag("setpar",el)) setparfunc(attr); 
  else if (tag("odometry",el)) odometryfunc(attr);  
  else if (tag("kalmanodometry",el)) kalmanodometryfunc(attr);      
  else if (tag("motioncontrol",el)) motioncontrolfunc(attr);      
  else if (tag("motorcontrol",el)) motorcontrolfunc(attr);
  else if (tag("linesensor",el)) linesensorfunc(attr);
  else if (tag("filter",el)) filterfunc(attr);
  else if (tag("matrix",el)) matrixfunc(attr);
  else if (tag("gpsmouse",el)) gpsmousefunc(attr);
  else if (tag("gpssim",el)) gpssimfunc(attr);
  else if (tag("speak",el)) speakfunc(attr);
  else if (tag("cameraserver",el)) camserverfunc(attr);
  else if (tag("laserserver",el)) laserserverfunc(attr);
  else if (tag("irsensor",el)) irsensorfunc(attr);
  else if (tag("debug",el)) debug = 1;
  else printf("Module : %s not known\n",el);
 
}  /* End of start handler */

/** \brief Handler for a XML end tag.
 *
 * This is a function defined for the Expat XML paser. The function
 * is used to tell the parser how to handle an end tag.
 * This function is used to parse the matrixes defined in the XML
 * language for the filters.
 * \param [in] *userData Pointer to a common data space for user data.
 * \param [in] *el Pointer to the current XML tag to parse.
 */
void end(void *userData, const char *el) {
    
  charField *data=(charField *)userData;
  
  // Temporary variable for the matrix data
  double tmp[BUFFSIZE]={0};
  
  // Terminate the string from the char text field
  *(data->end) = '\0';
  
  //**********************************************
  // Parsing of the matrix char data fields
  
  if(strcasecmp(el,"matrix")==0)
  {
    int i=0;
       
    // CAUTION
    //--------------------------------------------
    // There is no handling of input files with a 
    // wrong data field size.   
    
    // Skip over white space
    while(isspace(*(data->start)) != 0)
       data->start++;
       
    for(i=0;i<matrix_size;i++)
    {
       sscanf(data->start,"%lf",&tmp[i]);
       
       // Skip over printable characters except space
       while(isgraph(*(data->start)) != 0)
          data->start++;
       // Skip over white space
       while(isspace(*(data->start)) != 0)
          data->start++;
    }
  }  
    
  // End parsing
  //**********************************************
  
  // Keep track of the XML tree depth
  Depth--;

}  /* End of end handler */


/** \brief Handler for a XML character field.
 *
 * This is a function defined for the Expat XML paser. The function
 * is used to tell the parser how to handle acharacter field.
 * This function is used to save the data stored in the data fields for
 * parsing by the @see end function. A character field is not parsed by
 * one call to this function, but by severeal calls which is why the 
 * function saves the character data for further processing.
 * \param [in] *userData Pointer to a common data space for user data.
 * \param [in] *txt Pointer to the text parsed by one call to the function.
 * \param [in] txtlen Size of the text filed parsed by one call.
 */
void char_hndl(void *userData, const char *txt, int txtlen) {

  charField *data=(charField *)userData;
  
  //Save data in userData
  memcpy(data->end,txt,txtlen);
  //Move end pointer
  data->end+=txtlen;

}  /* End char_hndl */

/** \brief Function to be used for deugging the initialization.
 * 
 * If the tag \b debug is specified the read initialization
 * data is written back to the file: \b data.debug
 * 
 * \attention When additional parsing capabilities are added
 * to the robotinit.c file, this function must also be
 * expanded to keep the debugging functionality intact.
 * 
 * \author Lars Mogensen
 * \date 15/06-2006
 */
void save_debug(void)
{
   FILE *fp;
   char buf[100]={0};
   
   time_t curtime;
   struct tm *loctime;

   /* Get the current time. */
   curtime = time (NULL);

   /* Convert it to local time representation. */
   loctime = localtime (&curtime);
   
   fp = fopen("data.debug","w");
   if(fp == NULL) {
      fprintf(stderr,"Error opening data.debug\n");
   }
   else {
      fprintf(fp,"Debug file for the robot initialization\n");
      fprintf(fp,"---------------------------------------\n");
      fprintf(fp,"%s",asctime (loctime));
      fprintf(fp,"---------------------------------------\n\n");
      
      fprintf(fp,"Module : robotinfo\n");
      if(robotinfo.type==DIFFERENTIAL)
         strcpy(buf,"differential");
      else if(robotinfo.type==ACKERMAN)
         strcpy(buf,"ackerman");
      else
         strcpy(buf,"Robot type not known");
      fprintf(fp,"   type      %s     enum number %8i\n",
            buf,robotinfo.type);
      fprintf(fp,"   name      %s\n\n",robotinfo.name);
      
      fprintf(fp,"Module : odometry\n");
      fprintf(fp,"   cl           %8lf\n",odo.cl); 
      fprintf(fp,"   cr           %8lf\n",odo.cr); 
      fprintf(fp,"   w            %8lf\n",odo.w); 
      fprintf(fp,"   robotlength  %8lf\n",odo.robotlength); 
      fprintf(fp,"   ts           %8lf\n",odo.ts); 
      fprintf(fp,"   maxtick      %8i\n",odo.maxtick); 
      fprintf(fp,"   control      %8i\n",odo.control); 
      fprintf(fp,"   enctype      %8i\n\n",odo.enctype);    
      
      fprintf(fp,"Module : motion control\n");
      fprintf(fp,"   ts           %8lf\n",motcon.ts); 
      fprintf(fp,"   line_gain    %8lf\n",motcon.line_gain); 
      fprintf(fp,"   line_tau     %8lf\n",motcon.line_tau); 
      fprintf(fp,"   line_alfa    %8lf\n",motcon.line_alfa); 
      fprintf(fp,"   wall_gain    %8lf\n",motcon.wall_gain); 
      fprintf(fp,"   wall_tau     %8lf\n",motcon.wall_tau); 
      fprintf(fp,"   wall_alfa    %8lf\n",motcon.wall_alfa); 
      fprintf(fp,"   gain         %8lf\n",motcon.gain); 
      fprintf(fp,"   tau          %8lf\n",motcon.tau); 
      fprintf(fp,"   alfa         %8lf\n",motcon.alfa); 
      fprintf(fp,"   drive_kangle %8lf\n",motcon.kangle_drive);
      fprintf(fp,"   drive_kdist  %8lf\n",motcon.kdist_drive); 
      fprintf(fp,"   w            %8lf\n",motcon.w); 
      fprintf(fp,"   robotlength  %8lf\n",motcon.robotlength); 
      fprintf(fp,"   lim          %8lf\n",motcon.lim); 
      fprintf(fp,"   stopdist     %8lf\n",motcon.stopdist); 
      fprintf(fp,"   alarmdist    %8lf\n",motcon.alarmdist); 
      fprintf(fp,"   velcmd       %8lf\n",motcon.velcmd); 
      fprintf(fp,"   acccmd       %8lf\n",motcon.acccmd); 
      fprintf(fp,"   nolinedist   %8lf\n\n",motcon.nolinedist); 
      
      fprintf(fp,"Module : motor control\n");
      fprintf(fp,"   velscalel    %8lf\n",motorcontrol.velscalel); 
      fprintf(fp,"   velscaler    %8lf\n",motorcontrol.velscaler); 
      fprintf(fp,"   kp           %8lf\n",motorcontrol.kp); 
      fprintf(fp,"   ki           %8lf\n\n",motorcontrol.ki);      
      
      fprintf(fp,"Module : line sensor\n");
      fprintf(fp,"   size         %8i\n",linesensorsize); 
      fprintf(fp,"   k_filt       %8lf\n\n",line_data.k_filt); 
      
      if(kalmanodo.config)
      {
         fprintf(fp,"Module : filter\n");
         fprintf(fp,"   measurementnoise_std_x           %8lf\n",
               kalmanodo.measurement_noise_std_x); 
         fprintf(fp,"   measurementnoise_std_y           %8lf\n",
               kalmanodo.measurement_noise_std_y); 
         fprintf(fp,"   processnoise_std_stearingangle   %8lf\n",
               kalmanodo.process_noise_std_steering_angle); 
         fprintf(fp,"   processnoise_std_dist            %8lf\n",
               kalmanodo.process_noise_std_dist);
      
         fprintf(fp,"   type                  %s\n",kalmanodo.type);
      
         fprintf(fp,"   run                   %8i\n",kalmanodo.run);
         fprintf(fp,"   use                   %8i\n",kalmanodo.use);
         
         fprintf(fp,"   size_input            %8i\n",
               kalmanodo.size_input);
         fprintf(fp,"   size_state            %8i\n",
               kalmanodo.size_state);
         fprintf(fp,"   size_noise_process    %8i\n",
               kalmanodo.size_noise_process);
   
         fprintf(fp,"   size_output           %8i\n",
               kalmanodo.size_output);
         fprintf(fp,"   size_noise_output     %8i\n",
               kalmanodo.size_noise_output);
         
         fprintf(fp,"   gpsoffset_easting     %8lf\n",
               kalmanodo.offset_x);
         fprintf(fp,"   gpsoffset_northing    %8lf\n\n",
               kalmanodo.offset_y);
   
      }
      
      if(gps.config)
      {
         fprintf(fp,"Module : gpsmouse\n");
         fprintf(fp,"   hostname     %s\n",gps.hostname);
         fprintf(fp,"   port         %8i\n",gps.port);
         fprintf(fp,"   SBAS         %8i\n",gps.EGNOS);
         fprintf(fp,"   run          %8i\n",gps.run);
         fprintf(fp,"   use          %8i\n\n",gps.use);
      }
      
      fclose(fp);
   }
}

/** \brief This function parses the robot 
 * configuration file using the expat XML parser.
 * 
 * \return int 0 on succes, 1 if no configuration file
 * is found and -1 if other errors occour.
 */
int robotinit(void){

  XML_Parser p = XML_ParserCreate(NULL);
  if (! p) {
    fprintf(stderr, "Couldn't allocate memory for parser\n");
    exit(-1);
  }
  
  // initializing debug option
  debug=0;

  // Setting up the struct for char data storage
  field.start = field.dataField;
  field.end = field.start;
  
  // Setting up the expat XML parser
  XML_SetElementHandler(p, start, end);
  XML_SetUserData(p,&field);
  XML_SetCharacterDataHandler(p, char_hndl);
  
  {
    int done;
    int len;
    FILE *fp;
    
    fp = fopen(CONFFILE1,"r");
    if(fp == NULL)
    {
      fp = fopen(CONFFILE,"r");
      if(fp == NULL)
      return(1);
    }
    
    len = fread(Buff, 1, BUFFSIZE, fp);

    if (ferror(stdin)) {
      fprintf(stderr, "Read error\n");
      exit(-1);
    }
    done = feof(stdin);
    
    fclose(fp);

    if (! XML_Parse(p, Buff, len, done)) {
      fprintf(stderr, "Parse error at line %d:\n%s\n",
	     (int) XML_GetCurrentLineNumber(p),
	      XML_ErrorString(XML_GetErrorCode(p)));
      exit(-1);
    }
    
    if(debug == 1)
       save_debug();
    
    XML_ParserReset(p,NULL);
    XML_SetElementHandler(p, start, end);
  }
  
  XML_ParserFree(p);
  
  return 0;

}

#ifdef conftest
/** \brief Test the parsing of the configuration file.
 *
 * \return 0 on succes.
 */ 
int main(int argc, char **argv){
//   int opt,i; 
  robotinit();
  printf(" mc %lf  type %d  motioncontrol %lf \n",
         odo.cl,robotinfo.type,motcon.stopdist);  
  printf(" Specify a \"debug\" tag in the configuration");
  printf(" file in order to get debug file\n");  
  
  return 0;

}
#endif
