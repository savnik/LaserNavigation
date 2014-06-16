/** \mainpage
 * The smrdemo program is developed for educational
 * purposes by the staff at the Oersted*DTU, Automation,
 * Technical University of Denmark.
 * 
 * The aim of the program  is to give students and
 * researchers a fast and reliable platform to
 * operate mobile robots from.
 * 
 * See \htmlonly <a target="_blank" href="http://www.iau.dtu.dk/AG">
 * www.iau.dtu.dk/AG</a>
 * \endhtmlonly for further details.
 * 
 * \author Nils Axel Andersen
 * \author Lars Valdemar Mogensen
 * \date 12/06-2006
 * \version 1.0
 * 
 * \note Old structure is out commented, is comparisons are needed.
 * \todo Expand the Doxygen documentation to the entire 
 * qq
 demo project.
 */

/** \file smrdemo.c

 * \brief Main file of the smrdemo program. 
 * 
 * This file contains all of the core functionality of
 * the program. The structure is defined as this.
 * 
 * \verbatim 
  Initialization
       |
  Configuration
       |
  Calibration
       |
  Main loop
     Sensor read
     Sensor conditioning
  
     Estimation
  
     Motion control
     Actuator update
       |
  Finalization \endverbatim
 * 
 * \author Lars Valdemar Mogensen
 * \author Nils Axel Andersen
 *
 * \date 19/05-2006
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <time.h>
#include <semaphore.h>
#include <pthread.h>
#include "mobilerobot.h"
// ESM - added by Jørgen Eriksen
#include <math.h>
#include "smrsim.h"
#include "libhako.h"
#include "odometry.h"
#include "motioncontrol.h"
#include "ir2dist.h"
#include "linesensor.h"
#include "loader.h"
#include "server1.h"
#include "xmlio.h"
#include "interp.h"
#include "statemachine.h"
#include "stream2line.h"
#include "wallest.h"
#include "blockdetect.h"
#include "componentserver.h"
#include "administration.h"
#include "rhd.h"

#include "filter.h"
#include "libgps.h"


#define SMRSIM_BASE_PORT  8000
#define INSTALL_SEGMENTATIONFAULT_HANDLER 	0
#define ROUND(a) (floor(a+0.5))

enum {KALMANOFF=0, KALMAN_PRE_INIT, KALMANINIT,KALMANGPS3,KALMANGPS4,KALMANGPS3_4};
// Definition of structs to hold the robots
struct smr *robot, smrdummy;
struct smr smrdummy1;
hakotype hako;
symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor,*gyro,*steeringangle;
symTableElement *speedl,*speedr,*steeringangleref,*speedref,*enginespeedref,*liftinggearpos,*liftinggearstref ,             
*hakomanual,*hakoodopulses,*hakoodopulses1,*gpseasting,*gpsnorthing,*gpsquality,*gpssatelites,*gpsdop,
 *transpos,*rotpos,*cmdtransvel,*cmdrotvel,*resetmotorr,*resetmotorl,*isimtime,*curvatureref,*curvature,*thrustref,*pitchref, *rollref,*yawrateref,
// ESM - added by Jørgen Eriksen
*esmcompassheading,*esmpitch,*esmroll,*esmBxmagnetic,*esmBymagnetic,*esmBzmagnetic,*esmtemperature,  // Added by JE
*paintSetServo, *paintGetServo;  // Added by JE

unsigned char irin[8];
sem_t smrsystem_sem;
char smrsystemifbuf[1024];
pthread_t smrsystemptr;
int inputtablesize,outputtablesize;


struct {
   int type;
   char name[256];
} robotinfo;
odotype odo;
covtype odocov;
motiontype motcon;
blocktype bt;

double gyro1 = 0, gyro1off = 0, gyro1gain=0, gyro2 = 0, gyrotemp1 = 0, gyrotemp2 = 0;
// ESM - added by Jørgen Eriksen
double esmCompass = 0, esmPitch = 0, esmRoll = 0, esmBx = 0, esmBy = 0, esmBz = 0, esmTemp = 0, esmCount = 0;  // Added by JE
const double height_gps_ant = 1.75, filter_b = 0.023019559, filter_a = -0.95396088;  // Added by JE
double dist_roll = 0, dist_pitch = 0, dist_rp = 0, th_rp = 0, corrected_UTMe = 0, corrected_UTMn = 0;  // Added by JE
double lp_pitch = 0, lp_roll = 0, dummy_pitch = 0, dummy_roll = 0;  // Added by JE

int msgwait = 0, verbose = 1;
int logging = 1;

double *navstat;
struct {
   double velscalel, velscaler;
   double kp, ki;
} motorcontrol;
 struct{
 	int on;
}speak;

struct{
	int use;
}gpssim;

struct {
         double * dth;
	 double * steeringangle;	 
	}odoconnector;

struct {
         double * stop;	 
	}mrccontrol;


struct {
	 double * w2odox;
	 double * w2odoy;
	 double * w2odoth;
	 double * obstaclefront;
	 double * obstacleback;
	 double *x,*y,*z;
	}motionconnector;	

#define ROUTEOUTPUTTABLESIZE 2
struct { char * name; double *in;symTableElement * out;int index;} routeoutputtable[ROUTEOUTPUTTABLESIZE];
	



#define PARAMETERLISTSIZE 10
double notconnected=0;
double defaultobstaclefront=50,defaultobstacleback=1;

static parameterlisttype odoconnectparlist;
parameterelem odoconnectparlistarray[PARAMETERLISTSIZE];

static parameterlisttype mrccontrolparlist;
parameterelem mrccontrolparlistarray[PARAMETERLISTSIZE];

static parameterlisttype motionconnectparlist;
parameterelem motionconnectparlistarray[PARAMETERLISTSIZE];

functionlistelem mrcfunclist[]={{"odoconnect",&odoconnectparlist},
				{"motionconnect",&motionconnectparlist},
				{"mrccontrol",&mrccontrolparlist},	
				{ "",0}
				};


modulelistelem modulelist[]={ {"motioncontrol",motioncontrolfunclist},
			      {"mrc",mrcfunclist},
                               { 0,0}
			       };
void connectioninit(void){		       
  odoconnectparlist.Nmax=PARAMETERLISTSIZE;
  odoconnectparlist.N=0;
  odoconnectparlist.list=odoconnectparlistarray;
  addpar(&odoconnectparlist,"dth",ADMIN_inp,ADMIN_double,&odoconnector.dth);
  odoconnector.dth=&notconnected;
 
addpar(&odoconnectparlist,"steeringangle",ADMIN_inp,ADMIN_double,&odoconnector.steeringangle);
  odoconnector.steeringangle=&notconnected;

  motionconnectparlist.Nmax=PARAMETERLISTSIZE;
  motionconnectparlist.N=0;
  motionconnectparlist.list=motionconnectparlistarray;  
  addpar(&motionconnectparlist,"w2odox",ADMIN_inp,ADMIN_double,&motionconnector.w2odox);
  addpar(&motionconnectparlist,"w2odoy",ADMIN_inp,ADMIN_double,&motionconnector.w2odoy);
  addpar(&motionconnectparlist,"w2odoth",ADMIN_inp,ADMIN_double,&motionconnector.w2odoth);
  addpar(&motionconnectparlist,"obstaclefront",ADMIN_inp,ADMIN_double,&motionconnector.obstaclefront);
  addpar(&motionconnectparlist,"obstacleback",ADMIN_inp,ADMIN_double,&motionconnector.obstacleback);
  addpar(&motionconnectparlist,"x",ADMIN_inp,ADMIN_double,&motionconnector.x);
  addpar(&motionconnectparlist,"y",ADMIN_inp,ADMIN_double,&motionconnector.y);
  addpar(&motionconnectparlist,"z",ADMIN_inp,ADMIN_double,&motionconnector.z);
  motionconnector.obstacleback=&notconnected;
  motionconnector.obstaclefront=&notconnected;
  motionconnector.w2odox=&notconnected;
  motionconnector.w2odoy=&notconnected;
  motionconnector.w2odoth=&notconnected;
  motionconnector.x=&notconnected;
  motionconnector.y=&notconnected;
  motionconnector.z=&notconnected;
  
  
  mrccontrolparlist.Nmax=PARAMETERLISTSIZE;
  mrccontrolparlist.N=0;
  mrccontrolparlist.list=mrccontrolparlistarray;
  addpar(&mrccontrolparlist,"stop",ADMIN_inp,ADMIN_double,&mrccontrol.stop);
  mrccontrol.stop=&notconnected;
}
 


// Interface initializations
siminterfacetype siminterface;
stream2line_type s2lbuf;
volatile double simtime=0;

// GPS position sensor fusion initialization
IAU_kalman_odotype kalmanodo;
int usekalmanodo = 0, kalmanon = 0,xmlon=0;
int rollpitchon = 0, paintServoStatus = 160, paintServoValue = 0;  // Added by JE
double rolloffset = 0, pitchoffset = 0;  // Added by JE
wallesttype wallest;

// Wall estimator
int we_config = 0;
int we_status = 0;
int we_run = 0;
int we_use = 0;

// Definition of sensor data: GPS mouse
gpsmousetype gps;

// Definition of sensor data: IR sensors
irparamtype ir_param[6];
double ir_dist[6];
irsamptype ir_samp[6];
int ir_config;
int ir_status;
int ir_run;
int ir_use;

componentservertype lmssrv,camsrv;

// Definition of sensor data: line sensor
linesensor_partype ls_param[MAXLINESENSORSIZE];
int ls_input[MAXLINESENSORSIZE];
double ls_corrected[MAXLINESENSORSIZE];
find_line_type line_data;
struct {
   posetype data[8];
   double lsold[8];
   int Ndetect, linedetect;
} lineest;
linregtype lin;
int linetype = LINE_RIGHT_B, linesensorsize = 8;
int ls_config = 0;
int ls_status = 0;
int ls_run = 0;
int ls_use = 0;


// Definition of sensor data: laser scanner
char laserscan[2 * 362];

// Definition of sensor data: Guidemark server
int gm_config = 0;
int gm_status = 0;
int gm_run = 0;
int gm_use = 0;

int robot_port = SMR_PORT;
double cpufreqmhz;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];
double camlinepos;
int nolinecamr = 1, nolinecaml = 1, lcountr = 0, lcountl =
    0, lcountr1, crossingliner, crossingblackliner, rdr = 0, rdl = 0;
char userin[256], userin1[256], userout[256], asyncbuf[32];
double wheretogo;
int asynccount = 0;

double lineoffset = 0;
//double  dlog[10];
double tmpbuf[10000][2];
FILE *soundpipe, *speakpipe, *systempipe;
char track_file[80];
int blockcount = 0, blocked = 0;


// interpreter functions
int (*defaultcond) (void);
int yyparse(void);
void newcmd(void);
void init_table(void);
double *putinternvar(char *,int len);
int getncond();
int calccond();
int getcondres();
void setcurrentplan(plantype * p);
int lineno = 0;
plantype plan;
memtype planm;
#define PLANTABLESIZE 10
struct { struct {int used;
	         plantype plan;
		 }plans[PLANTABLESIZE];
         int size;
	}plantable; 

extern char *bp;
char buf[80];
char planmem[100000];
double logtable[LOGVARMAX][110000];
double intpstatereturn;
unsigned char imglog[10000][160];
int logptr = 0, imgptr = 0;
//int guidemarkok;
int inputready;

extern symbolptrtabtype logvar, streamvar;
extern varptrtabtype logvar1;

struct {
   double vin;
   double vout;
} battery;
struct {
   double vin;
   double vout;
} powersupply;

int robotinit(void);

servertype serv;
struct sockaddr_in serv_adr;
int sockfd;


sm_type sm_main_data;
int sm_main(sm_type * sm);
int sm_square(sm_type * sm);
int sm_interp(sm_type * sm);
int sm_server(sm_type * sm);
int sm_barcode(sm_type * sm);
void sm_userupdate(sm_type * sm);
int putevent(char *event);
void savelog(char *filename);
sm_type kalmanstate;

struct {
  double x,y;
  double dist;
  int count;
} kalman_first_gps;

void serverconnect(componentservertype *s){
  char buf[256];
  int len;
  s->serv_adr.sin_family = AF_INET;
  s->serv_adr.sin_port= htons(s->port);
  s->serv_adr.sin_addr.s_addr = inet_addr(s->host);
  printf("port %d host %s \n",s->port,s->host);
  if ((s->connected=(connect(s->sockfd, (struct sockaddr *) &s->serv_adr, sizeof(s->serv_adr))) >-1)){
    printf(" connected to %s  \n",s->name);
    len=sprintf(buf,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    send(s->sockfd,buf,len,0);
    len=sprintf(buf,"<smrdemo version=\"2.00\" >\n");
    send(s->sockfd,buf,len,0);
    if (fcntl(s->sockfd,F_SETFL,O_NONBLOCK) == -1) {
          fprintf(stderr,"startserver: Unable to set flag O_NONBLOCK on %s fd \n",s->name);
    }
  }
  else{
      printf("Not connected to %s  %d \n",s->name,s->connected);   
  }
}



  symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

 
  int  rhdwrite(symTableElement * var,int index){
     int len;
     if (var){
       len=var->length;
       var->data[index]=(var->inputVar[index]+var->inputVar[index+len])*var->inputVar[index+2*len]+0.5;
       var->updated=1;
       return 0;
     }else
       return 1;
    }




void segfaulthandler(int sig)
{
//    perror(NULL);
   printf("Seg-error\n");
   exit(1);
}

void brokenpipehandler(int sig)
{
   printf("smrdemo: broken pipe \n");
   savelog("log");
   exit(1);
}


void safechange(int *s, int f2t, int Nf2t, int t2f, int Nt2f, int *count)
{
   if (*s) {
      if (t2f) {
         (*count)++;
         if (*count > Nt2f) {
            *count = 0;
            *s = 0;
         }
      } else
         *count = 0;
   } else {
      if (f2t) {
         (*count)++;
         if (*count > Nf2t) {
            *count = 0;
            *s = 1;
         }
      } else
         *count = 0;
   }
}

double abspose(posetype p)
{
   return (fabs(p.x) + fabs(p.y));
}

double statedist(sm_type * sm);

void userprint(char buf[])
{
   send(serv.s, buf, strlen(buf), 0);
}

void xml_proc(struct xml_in *x)
{
   double a;
   while (1) {
      switch (xml_in_nibble(x)) {
      case XML_IN_NONE:
         return;
      case XML_IN_TAG_START:
#if (0)
         {
            int i;
            printf("start tag: %s, %d attributes\n", x->a, x->n);
            for (i = 0; i < x->n; i++) {
               printf("  %s    %s  \n", x->attr[i].name, x->attr[i].value);
            }
         }
#endif
         if (strcmp("gmk", x->a) == 0) {
            printf("  %s    %s  \n", x->attr[0].name, x->attr[0].value);
            if (getdouble(&a, "id", x)) {
               gmk.id = a;
               printf("id= %f\n", gmk.id);
            }
	    if (getdouble(&a, "crcOK", x)) {
               gmk.crc = a;
               printf("crc= %f\n", gmk.crc);
            }
         }
         if (strcmp("pos3d", x->a) == 0) {
            (getdouble(&gmk.x, "x", x));
            (getdouble(&gmk.y, "y", x));
            (getdouble(&gmk.z, "z", x));
         }
         if (strcmp("rot3d", x->a) == 0) {
            (getdouble(&gmk.omega, "Omega", x));
            (getdouble(&gmk.phi, "Phi", x));
            (getdouble(&gmk.kappa, "Kappa", x));
         }
         if (strcmp("vision", x->a) == 0) {
            int i, ix;
            for (i = 0; i < x->n; i++) {
               ix = atoi(x->attr[i].name + 3);
               if (ix > -1 && ix < 10)
                  visionpar[ix] = atof(x->attr[i].value);
            }

         }

         break;
      case XML_IN_TAG_END:
         //printf("end tag: %s\n", x->a);
         break;
      case XML_IN_TEXT:
         //printf("text: %d bytes\n  \"", x->n);
         //fwrite(x->a, 1, x->n, stdout);
         //printf("\"\n");
         break;
      }
   }
}

void xml_proca(struct xml_in *x){

 while(1){
 switch (xml_in_nibble(x)) {
    case XML_IN_NONE:
      return;
    case XML_IN_TAG_START:
    #if (0)
    {int i;
    double a;
      printf("start tag: %s, %d attributes\n", x->a, x->n);
      for(i=0;i<x->n;i++){
        printf("  %s    %s  \n",x->attr[i].name,x->attr[i].value);
      }
    }
    #endif
       if (strcmp("laser",x->a)==0){
       int i,ix;
       for (i=0;i< x->n;i++){
         ix=atoi(x->attr[i].name+1);
	 if (ix >-1 && ix < 10)
	   laserpar[ix]=atof(x->attr[i].value);
       }
       
     }
   
    break;
    case XML_IN_TAG_END:
      //printf("end tag: %s\n", x->a);
      break;
    case XML_IN_TEXT:
      //printf("text: %d bytes\n  \"", x->n);
      //fwrite(x->a, 1, x->n, stdout);
      //printf("\"\n");
      break;
    }
  } 
}   



unsigned char servo(double x, double odo)
{
#define SIGP 0.4

   char msg[4] = { 0 };
   int sig;
   static double servo_signal = (55 + 9) / 2;
   if (x < odo - 0.03)
      servo_signal -= SIGP + (odo - x) * 1.0;
   else if (x > odo + 0.03)
      servo_signal += SIGP + (x - odo) * 1.0;

   sig = (int) servo_signal;
   if (servo_signal - (double) sig > 0.5)
      sig++;


   if (servo_signal > 55.0)
      servo_signal = 55.0;
   else if (servo_signal < 9.0)
      servo_signal = 9.0;


   if (sig > 55)
      sig = 55;
   else if (sig < 9)
      sig = 9;


// printf("s: %lf %lf %i\n",x,odo,sig);
   msg[0] = 0x03;
   msg[1] = 0xF1;
   msg[2] = 0x0B;
   msg[3] = (unsigned char) sig;
   return sig;
}                               /* servo() */

double smrgettime(void)
{
   struct timeval tv;
   struct timezone tz;
   gettimeofday(&tv, &tz);
   if (siminterface.simulating)
     return simtime;
   else  
     return (double) tv.tv_sec + tv.tv_usec / 1000000.;
}

#define GYRO1OFF	2594
#define GYROTEMP1OFF	2520

/* Automatic catches send messages from the server*/
void recvlaser(struct smr *s)
{
   int i, p;
   //Checks to see if messages from the server is laserscans
   if ((ENET_TYPE(s->msg) == 0x40)) {
      p = s->msg[1] * 30;
      if (p < 0 || p > 694)
         printf("laser address error  p= %d mes %x %x %x %x\n",
                p, s->msg[0], s->msg[2], s->msg[2], s->msg[3]);
      else
         for (i = 0; i < 30; i++)
            laserscan[p + i] = s->msg[i + 2];
   } else if((ENET_TYPE(s->msg) == 0x60)){
     if( s->msg[1]== 1){
       simtime=*(double *)(s->msg+2) ;
     }
   }
   else if (s->msg[1] == 0xa5 && s->msg[0] == 6 && s->msg[2] == 1) {


      gyro1 =
          gyro1off + (((int) s->msg[3] + 256 * (int) s->msg[4]) -
                      GYRO1OFF + 0.33 * gyrotemp1) / 12.5;
      gyro2 = (int) s->msg[5] + 256 * (int) s->msg[6];


   } else if (s->msg[1] == 0xa5 && s->msg[0] == 6 && s->msg[2] == 2) {

      gyrotemp1 =
          (((int) s->msg[3] + 256 * (int) s->msg[4])) - GYROTEMP1OFF;
      gyrotemp2 = (int) s->msg[5] + 256 * (int) s->msg[6];

      asynccount++;
   }

}


#if (0)
int check_crossing_black(find_line_type * ld)
{
   double val;

   val = 0.3;
   if (ld->avg_s[0] < val && ld->avg_s[7] < val) {
      /* Crossing line found */
      return (1);
   } else {
      return (0);
   }
}
#endif

int check_crossing_white(find_line_type * ld)
{
   double val, tol = 0.03;
   static double rightdist = 0, leftdist = 0;
   int i, found, retval;
   retval = 0;
   val = 0.75;

   found = 1;
   for (i = 0; i < 4; i++) {
      if (ld->avg_s[i] < val)
         found = 0;
   }
   if (found) {
      rightdist = odo.dist;
      if (fabs(rightdist - leftdist) < tol)
         retval = 1;
   }

   found = 1;
   for (i = 4; i < 8; i++) {
      if (ld->avg_s[i] < val)
         found = 0;
   }
   if (found) {
      leftdist = odo.dist;
      if (fabs(rightdist - leftdist) < tol)
         retval = 1;
   }
   return (retval);
}

int check_crossing_black(find_line_type * ld)
{
   double val, tol = 0.03;
   static double rightdist = 0, leftdist = 0;
   int i, found, retval;
   retval = 0;
   val = 0.2;

   found = 1;
   for (i = 0; i < 4; i++) {
      if (ld->avg_s[i] > val)
         found = 0;
   }
   if (found) {
      rightdist = odo.dist;
      if (fabs(rightdist - leftdist) < tol)
         retval = 1;
   }

   found = 1;
   for (i = linesensorsize - 4; i < linesensorsize; i++) {
      if (ld->avg_s[i] > val)
         found = 0;
   }
   if (found) {
      leftdist = odo.dist;
      if (fabs(rightdist - leftdist) < tol)
         retval = 1;
   }
   return (retval);
}


int line_found(find_line_type * ld)
{
   double val;
   val = 0.3;
   if (ld->avg_s[3] < val && ld->avg_s[4] < val) {
      /* Line found */
      return (1);
   } else {
      return (0);
   }
}

void smrsound_init()
{
   soundpipe = popen("smrsound.tcl", "w");
   if (soundpipe == NULL) {
      printf("can't open soundpipe \n");
      exit(1);
   }
}

void smrsound(char *inp)
{
   if (soundpipe) {
      fprintf(soundpipe, "%s \n", inp);
      fflush(soundpipe);
   }
}

void smrsystemif_init()
{
   systempipe = popen("smrsystem", "w");
   if (systempipe == NULL) {
      printf("can't open systempipe \n");
      exit(1);
   }
}

#if(0)
void smrsystemif(char *inp)
{
   if (systempipe) {
      fprintf(systempipe, "%s \n", inp);
      fflush(systempipe);
   }
}
#endif

static void *smrsystem(void *args){
while(1){
  sem_wait(&smrsystem_sem);
  system(smrsystemifbuf);
 }
 return(0);
}



void smrsystemif(char *inp){
strncpy(smrsystemifbuf,inp,1024);
sem_post(&smrsystem_sem);

}


void smrspeak_init()
{
   speakpipe = popen("smrspeak.tcl", "w");
   if (speakpipe == NULL) {
      printf("can't open speakpipe \n");
      exit(1);
   }
}

void smrspeak(char *inp)
{
   if (speakpipe) {
      fprintf(speakpipe, "\"%s\" \n", inp);
      fflush(speakpipe);
   }
}



typedef struct {
   double *pdrivendist, *pcmdtime,
       *pirdist0, *pirdist1, *pirdist2, *pirdist3, *pirdist4,
       *podox, *podoy, *podoth, *pododist, *pododistleft, *pododistright,
       *podovelocity, *pododencr, *pododencl, *podosteeringangle,*pblacklinefound,
       *pcrossingblack, *pcrossingblackr, *pcrossingwhite, *psupplyvoltage,
       *pbatteryvoltage, *pintpstatereturn, *line0, *line1, *line2, *line3,
       *line4, *line5, *line6, *line7,       
       *plineraw0,*plineraw1,*plineraw2,*plineraw3,*plineraw4,*plineraw5,*plineraw6,*plineraw7,
       *pedgedetect, *pedgex, *pedgey,
       *pedgeth, *pirl, *pirfl, *pirfm, *pirfr, *pirr, *plinepos,
       *pnoline_b, *pnolinecamr, *pnolinecaml, *prdr, *prdl, *pguidemarkok,*pgmkcrc,
       *pgmkx, *pgmky, *pgmkz, *pgmkomega, *pgmkphi, *pgmkkappa,
       *pwallestiswall, *pwallestholes, *pdlog0, *pdlog1, *pdlog2, *pdlog3,
       *pwheretogo, *pmotorpwml, *pmotorpwmr, *pmotorstatusl,
       *pmotorstatusr, *pmotorcmdl, *pmotorcmdr, *pblocked, *pmotionstatus,
       *ptargetx, *ptargety, *ptargetth, *ptargetdist, *pgyro1, *pgyro2,
       *pgyrotemp1, *pgyrotemp2, *preadflags, *phakosteeringangle,
       *phakosteeringangleref, *phakospeedref, *phakocvtpulses,
       *phakoenginespeed, *phakoenginespeedref, *phakonavigationmode,
       *phakonavigationmoderef, *phakostate0, *phakostate1, *phakostate2,
       *phakostate3, *phakodirectionbyte, *phakoliftpos,
       *phakotakeoffspeed, *pimuroll, *pimupitch, *pimuyaw, *pimuaccx,
       *pimuaccy, *pimuaccz, *pimutemp, *pimutime, *pxkalman, *pykalman,
       *pthkalman, *pkalmanstatus, *pgpseasting, *pgpsnorthing,
       *pgpsquality, *pgpssatellites, *pgpsdop, *vis0, *vis1, *vis2, *vis3,
       *vis4, *vis5, *vis6, *vis7, *vis8, *vis9, 
       *l0,*l1,*l2,*l3,*l4,*l5,*l6,*l7,*l8,*l9,
       *pmotcon_rvel,*pmotcon_lvel,*pmotcon_omega,*pmotcon_steeringangle,
       *pclock,*ptick,*ptime,*pkalmanmode,*pcovth,*pcovthx,*pcovthy,*pcovx,*pcovxy,*pcovy;

} internvartype;

void init_internvar(internvartype * p)
{
   p->pdrivendist = putinternvar("drivendist",1);
   p->pcmdtime = putinternvar("cmdtime",1);
   p->pirdist0 = putinternvar("irdistleft",1);
   p->pirdist1 = putinternvar("irdistfrontleft",1);
   p->pirdist2 = putinternvar("irdistfrontmiddle",1);
   p->pirdist3 = putinternvar("irdistfrontright",1);
   p->pirdist4 = putinternvar("irdistright",1);
   p->pirl = putinternvar("irl",1);
   p->pirfl = putinternvar("irfl",1);
   p->pirfm = putinternvar("irfm",1);
   p->pirfr = putinternvar("irfr",1);
   p->pirr = putinternvar("irr",1);
   p->podox = putinternvar("odox",1);
   p->podoy = putinternvar("odoy",1);
   p->podoth = putinternvar("odoth",1);
   p->pododist = putinternvar("ododist",1);
   p->pododistleft = putinternvar("ododistleft",1);
   p->pododistright = putinternvar("ododistright",1);
   p->podovelocity = putinternvar("odovelocity",1);
   p->pododencr = putinternvar("ododencr",1);
   p->pododencl = putinternvar("ododencl",1);
   p->podosteeringangle = putinternvar("odosteeringangle",1);
   p->pblacklinefound = putinternvar("blacklinefound",1);
   p->pcrossingblack = putinternvar("crossingblackline",1);
   p->pcrossingblackr = putinternvar("crossingblackliner",1);
   p->pcrossingwhite = putinternvar("crossingwhiteline",1);
   p->prdr = putinternvar("rowdetectedright",1);
   p->prdl = putinternvar("rowdetectedleft",1);
   p->pbatteryvoltage = putinternvar("batteryvoltage",1);
   p->psupplyvoltage = putinternvar("supplyvoltage",1);
   p->pintpstatereturn = putinternvar("statereturn",1);
   p->line0 = putinternvar("line0",1);
   p->line1 = putinternvar("line1",1);
   p->line2 = putinternvar("line2",1);
   p->line3 = putinternvar("line3",1);
   p->line4 = putinternvar("line4",1);
   p->line5 = putinternvar("line5",1);
   p->line6 = putinternvar("line6",1);
   p->line7 = putinternvar("line7",1);
   p->plineraw0 = putinternvar("lineraw0",1);
   p->plineraw1 = putinternvar("lineraw1",1);
   p->plineraw2 = putinternvar("lineraw2",1);
   p->plineraw3 = putinternvar("lineraw3",1);
   p->plineraw4 = putinternvar("lineraw4",1);
   p->plineraw5 = putinternvar("lineraw5",1);
   p->plineraw6 = putinternvar("lineraw6",1);
   p->plineraw7 = putinternvar("lineraw7",1);
   p->plinepos = putinternvar("linepos",1);
   p->pnoline_b = putinternvar("nolineb",1);
   p->pnolinecamr = putinternvar("nolinecamr",1);
   p->pnolinecaml = putinternvar("nolinecaml",1);
   p->pedgedetect = putinternvar("edgedetect",1);
   p->pedgex = putinternvar("edgex",1);
   p->pedgey = putinternvar("edgey",1);
   p->pedgeth = putinternvar("edgeth",1);
   p->pguidemarkok = putinternvar("guidemarkok",1);
   p->pgmkcrc = putinternvar("gmkcrc",1);
   p->pgmkx = putinternvar("gmkx",1);
   p->pgmky = putinternvar("gmky",1);
   p->pgmkz = putinternvar("gmkz",1);
   p->pgmkomega = putinternvar("gmkomega",1);
   p->pgmkphi = putinternvar("gmkphi",1);
   p->pgmkkappa = putinternvar("gmkkappa",1);
   p->pwallestiswall = putinternvar("wallnow",1);
   p->pwallestholes = putinternvar("wallholes",1);
   p->pdlog0 = putinternvar("log0",1);
   p->pdlog1 = putinternvar("log1",1);
   p->pdlog2 = putinternvar("log2",1);
   p->pdlog3 = putinternvar("log3",1);
   p->pwheretogo = putinternvar("wheretogo",1);
   p->pmotorpwml = putinternvar("motorpwml",1);
   p->pmotorpwmr = putinternvar("motorpwmr",1);
   p->pmotorcmdl = putinternvar("motorcmdl",1);
   p->pmotorcmdr = putinternvar("motorcmdr",1);
   p->pblocked = putinternvar("blocked",1);
   p->pmotionstatus = putinternvar("motionstatus",1);
   p->ptargetx = putinternvar("targetx",1);
   p->ptargety = putinternvar("targety",1);
   p->ptargetth = putinternvar("targetth",1);
   p->ptargetdist = putinternvar("targetdist",1);
   p->ptime = putinternvar("time",1);
   p->pgyro1 = putinternvar("gyro1",1);
   p->pgyro2 = putinternvar("gyro2",1);
   p->pgyrotemp1 = putinternvar("gyrotemp1",1);
   p->pgyrotemp2 = putinternvar("gyrotemp2",1);
   p->preadflags = putinternvar("readflags",1);
   p->phakosteeringangleref = putinternvar("hakosteeringangleref",1);
   p->phakospeedref = putinternvar("hakospeedref",1);
   p->phakocvtpulses = putinternvar("hakocvtpulses",1);
   p->phakoenginespeed = putinternvar("hakoenginespeed",1);
   p->phakoenginespeedref = putinternvar("hakoenginespeedref",1); 
   p->phakonavigationmoderef = putinternvar("hakonavigationmoderef",1);
   p->phakodirectionbyte = putinternvar("hakodirectionbyte",1);
   p->phakoliftpos = putinternvar("hakoliftinggearpos",1);
   p->phakotakeoffspeed = putinternvar("hakopowertakeoffspeed",1);
   p->pimuroll = putinternvar("imuroll",1);
   p->pimupitch = putinternvar("imupitch",1);
   p->pimuyaw = putinternvar("imuyaw",1);
   p->pimuaccx = putinternvar("imuaccx",1);
   p->pimuaccy = putinternvar("imuaccy",1);
   p->pimuaccz = putinternvar("imuaccz",1);
   p->pimutemp = putinternvar("imutemp",1);
   p->pimutime = putinternvar("imutime",1);
   p->pxkalman = putinternvar("xkalman",1);
   p->pykalman = putinternvar("ykalman",1);
   p->pthkalman = putinternvar("thkalman",1);
   p->pkalmanstatus = putinternvar("kalmanstatus",1);
   p->pgpseasting = putinternvar("gpseasting",1);
   p->pgpsnorthing = putinternvar("gpsnorthing",1);
   p->pgpsquality = putinternvar("gpsquality",1);
   p->pgpssatellites = putinternvar("gpssatellites",1);
   p->pgpsdop = putinternvar("gpsdop",1);
   p->vis0 = putinternvar("vis0",1);
   p->vis1 = putinternvar("vis1",1);
   p->vis2 = putinternvar("vis2",1);
   p->vis3 = putinternvar("vis3",1);
   p->vis4 = putinternvar("vis4",1);
   p->vis5 = putinternvar("vis5",1);
   p->vis6 = putinternvar("vis6",1);
   p->vis7 = putinternvar("vis7",1);
   p->vis8 = putinternvar("vis8",1);
   p->vis9 = putinternvar("vis9",1);
    p->l0=putinternvar("l0",1);
      p->l1=putinternvar("l1",1);
      p->l2=putinternvar("l2",1);
      p->l3=putinternvar("l3",1);
      p->l4=putinternvar("l4",1);
      p->l5=putinternvar("l5",1);
      p->l6=putinternvar("l6",1);
      p->l7=putinternvar("l7",1);
      p->l8=putinternvar("l8",1);
      p->l9=putinternvar("l9",1);
      p->pclock=putinternvar("clock",1);
      p->ptick=putinternvar("tick",1);
      p->pmotcon_lvel=putinternvar("motcon_lvel",1);
      p->pmotcon_rvel=putinternvar("motcon_rvel",1);
      p->pmotcon_omega=putinternvar("motcon_omega",1);
      p->pmotcon_steeringangle=putinternvar("motcon_steeringangle",1);
      p->pkalmanmode=putinternvar("kalmanmode",1);
      p->pcovth=putinternvar("odocovth",1);
      p->pcovthx=putinternvar("odocovthx",1);
      p->pcovthy=putinternvar("odocovthy",1);
      p->pcovx=putinternvar("odocovx",1);
      p->pcovxy=putinternvar("odocovxy",1);
      p->pcovy=putinternvar("odocovy",1);    
}


void update_internvar(internvartype * p, sm_type * sm)
{
   *p->pdrivendist = statedist(sm);
   *p->pcmdtime = sm->time * motcon.ts;
   *p->pirdist0 = ir_dist[0];
   *p->pirdist1 = ir_dist[1];
   *p->pirdist2 = ir_dist[2];
   *p->pirdist3 = ir_dist[3];
   *p->pirdist4 = ir_dist[4];
   if (irsensor){
     *p->pirl = irsensor->data[0];
     *p->pirfl = irsensor->data[1];
     *p->pirfm = irsensor->data[2];
     *p->pirfr = irsensor->data[3];
     *p->pirr = irsensor->data[4];
    }
   *p->podox = odo.pose.x;
   *p->podoy = odo.pose.y;
   *p->podoth = odo.pose.th;
   *p->pododist = odo.dist;
   *p->pododistleft = odo.distleft;
   *p->pododistright = odo.distright;
   *p->podovelocity = odo.vel;
   *p->pododencr = odo.dencr;
   *p->pododencl = odo.dencl;
   *p->podosteeringangle = odo.steeringangle;
   *p->pblacklinefound = line_found(&line_data);
   *p->pcrossingblack = check_crossing_black(&line_data);
   *p->pcrossingblackr = crossingblackliner;
   *p->pcrossingwhite = check_crossing_white(&line_data);
   *p->pbatteryvoltage = battery.vout;
   *p->psupplyvoltage = powersupply.vout;
   *p->pintpstatereturn = intpstatereturn;
   *p->line0 = ls_corrected[0];
   *p->line1 = ls_corrected[1];
   *p->line2 = ls_corrected[2];
   *p->line3 = ls_corrected[3];
   *p->line4 = ls_corrected[4];
   *p->line5 = ls_corrected[5];
   *p->line6 = ls_corrected[6];
   *p->line7 = ls_corrected[7];
   if (linesensor){
     *p->plineraw0 = linesensor->data[0];
     *p->plineraw1 = linesensor->data[1];
     *p->plineraw2 = linesensor->data[2];
     *p->plineraw3 = linesensor->data[3];
     *p->plineraw4 = linesensor->data[4];
     *p->plineraw5 = linesensor->data[5];
     *p->plineraw6 = linesensor->data[6];
     *p->plineraw7 = linesensor->data[7];
   }  
   *p->plinepos = motcon.linepos;
   *p->pnoline_b = line_data.noline_b;
   *p->pedgedetect = lineest.linedetect;
   *p->pedgex = lin.line.x;
   *p->pedgey = lin.line.y;
   *p->pedgeth = lin.line.th;
   *p->pnolinecamr = nolinecamr;
   *p->pnolinecaml = nolinecaml;
   *p->prdl = rdl;
   *p->prdr = rdr;
   *p->pguidemarkok = gmk.id;
   *p->pgmkcrc = gmk.crc;
   *p->pgmkx = gmk.x;
   *p->pgmky = gmk.y;
   *p->pgmkz = gmk.z;
   *p->pgmkomega = gmk.omega;
   *p->pgmkphi = gmk.phi;
   *p->pgmkkappa = gmk.kappa;
   *p->pwallestiswall = (double) wallest.is_wall;
   *p->pwallestholes = (double) wallest.holes;
   *p->pdlog0 = *motcon.irdistl;
   *p->pdlog1 = *motcon.irdistr;
   *p->pdlog2 = *motcon.wallanglel;
   *p->pdlog3 = *motcon.wallangler;
   *p->pwheretogo = wheretogo;
   *p->pmotorcmdl = motcon.lvel;
   *p->pmotorcmdr = motcon.rvel;
   *p->pblocked = blocked;
   *p->pmotionstatus = motcon.status;
   *p->ptargetx = motcon.tgt.x;
   *p->ptargety = motcon.tgt.y;
   *p->ptargetth = motcon.tgt.th;
   *p->ptargetdist = motcon.target_dist;
   *p->ptime = smrgettime();
   *p->pgyro1 = gyro1;
   *p->pgyro2 = gyro2;
   *p->pgyrotemp1 = gyrotemp1;
   *p->pgyrotemp2 = gyrotemp2;
   *p->preadflags = robot->read_flags;
   
   *p->phakosteeringangleref = motcon.steeringangle;
   *p->phakospeedref = motcon.lvel;
   *p->phakocvtpulses = hako.cvtPulses;
   *p->phakoenginespeed = hako.engineSpeed;
   *p->phakoenginespeedref = hako.engineSpeedRef;
   if (hakomanual)
     *p->phakonavigationmoderef = hakomanual->data[0];
   else
     *p->phakonavigationmoderef = hako.navigationModeRef;
   *p->phakodirectionbyte = hako.directionByte;
   *p->phakoliftpos = hako.liftingGearPos;
   *p->phakotakeoffspeed = hako.powerTakeoffSpeed;
   *p->pimuroll = hako.xbow.roll;
   *p->pimupitch = hako.xbow.pitch;
   *p->pimuyaw = hako.xbow.yaw;
   *p->pimuaccx = hako.xbow.accX;
   *p->pimuaccy = hako.xbow.accY;
   *p->pimuaccz = hako.xbow.accZ;
   *p->pimutemp = hako.xbow.temp;
   *p->pimutime = hako.xbow.time;
   *p->pxkalman = vget(kalmanodo.kalman->Xpost, 0) + kalmanodo.offset_x;
   *p->pykalman = vget(kalmanodo.kalman->Xpost, 1) + kalmanodo.offset_y;
   *p->pthkalman = vget(kalmanodo.kalman->Xpost, 2);
   *p->pkalmanstatus = kalmanodo.status;
   *p->pgpseasting = hako.gps.easting;
   *p->pgpsnorthing = hako.gps.northing;
   *p->pgpsquality = hako.gps.quality;
   *p->pgpssatellites = hako.gps.satellites;
   *p->pgpsdop = hako.gps.dop;
   *p->vis0 = visionpar[0];
   *p->vis1 = visionpar[1];
   *p->vis2 = visionpar[2];
   *p->vis3 = visionpar[3];
   *p->vis4 = visionpar[4];
   *p->vis5 = visionpar[5];
   *p->vis6 = visionpar[6];
   *p->vis7 = visionpar[7];
   *p->vis8 = visionpar[8];
   *p->vis9 = visionpar[9];
   *p->l0=laserpar[0];
   *p->l1=laserpar[1];
   *p->l2=laserpar[2];
   *p->l3=laserpar[3];
   *p->l4=laserpar[4];
   *p->l5=laserpar[5];
   *p->l6=laserpar[6];
   *p->l7=laserpar[7];
   *p->l8=laserpar[8];
   *p->l9=laserpar[9];
   *p->pclock=robot->ts;
   *p->ptick=robot->tick;
   *p->pmotcon_lvel=motcon.lvel;
   *p->pmotcon_rvel=motcon.rvel;
   *p->pmotcon_steeringangle=motcon.steeringangle;
   *p->pmotcon_omega=motcon.omega;
   *p->pkalmanmode=kalmanstate.state;
   *p->pcovth=odocov.p11;
   *p->pcovthx=odocov.p12;
   *p->pcovthy=odocov.p13;
   *p->pcovx=odocov.p22;
   *p->pcovxy=odocov.p23;
   *p->pcovy=odocov.p33;

}

void inputvar_update(){
 int i,j,len;
  for (i=0;i<inputtablesize;i++){
    len=inputtable[i].length;
    for (j=0;j<len;j++){  
     inputtable[i].inputVar[j]=(inputtable[i].data[j]+inputtable[i].inputVar[j+len])*inputtable[i].inputVar[j+2*len];
    }
  }
}

internvartype internvar;


void reset_motor(int motor_number)

{
   switch (motor_number) {
   case 1:
         resetmotorl->updated=1;
     break;
   case 2:
      resetmotorr->updated=1;
   break;
   default:
   break;  
   }
}

/// fjernet af christian
#if (0)
void set_motor_param(int motor_number, int par, int val)
{
   if (par > -1 && par < 3) {
      if (par != 2) {
         robot->msg[0] = ENET_TYPE_485 | 0x02;
         robot->msg[2] = val;
      } else
         robot->msg[0] = ENET_TYPE_485 | 0x01;
      robot->msg[1] = ((par + 7) << 4) | motor_number;
      robot->write_flags = SMR_FLAG_AM;
      smr_write(robot);
   }
}

void set_digiout(int out)
{
   asyncbuf[0] = ENET_TYPE_485 | 0x02;
   asyncbuf[1] = 0x60 | 9;
   asyncbuf[2] = out;
}

void rs485send(unsigned char *msg)
{
   int i;
   robot->msg[0] = ENET_TYPE_485 | msg[0];
   for (i = 1; i <= msg[0]; i++)
      robot->msg[i] = msg[i];
   robot->write_flags = SMR_FLAG_AM;
   smr_write(robot);
}
#endif


#define NAMELEN		256
#define CALIBDIR	"calib/"

sm_type *intp_smdataptr;
int (*intp_smfuncptr) (sm_type * sm);
void checkwatches(void)
{
   int i, res;
   char buf[80];
   double watchres;
   saveinterpcontext();
   for (i = 0; i < watchtab.p; i++) {
      bp = watchtab.tab[i].exp;
      newcmd();
      res = yyparse();
      watchres = getwatchres();
      if (watchres > watchtab.tab[i].val) {
         if (xmlon){
           sprintf(buf, "<watch name=\"%s\" time=\"%lf\" /> \n", watchtab.tab[i].name,
                 smrgettime());
	 }
	 else
	   sprintf(buf, "watch %s fired %lf \n", watchtab.tab[i].name,smrgettime());
         putevent(buf);
      }
      watchtab.tab[i].val = watchres;
   }
   loadinterpcontext();
}

void resetlog(void)
{
   logptr = 0;
}

void removelogvars(void)
{
   logvar1.n = 0;
}

void savelog(char *fname)
{
   int i, j, loggingback;
   FILE *f;
   f = fopen(fname, "w");
   if (f) {
      loggingback = logging;
      logging = 0;
      for (i = 0; i <= (logptr - 1); i++) {
         for (j = 0; j < logvar1.n; j++)
            fprintf(f, "%lf ", logtable[j][i]);
         fprintf(f, "\n");
      }
      fclose(f);
      logging = loggingback;
   }
   else {
     printf("could not open logfile %s \n",fname);
   }
}


FILE *ftemp, *ftemp1;

int main(int argc, char *argv[])
{

   int i, choice, runsmr, calibration, userif, opt, server = 0, count = -1;
   int pulsecount = 0;
   char ansbuf[81];
   sm_type *cur_smdata;
   int (*cur_smfunc) (sm_type * sm);

   char hostname[NAMELEN];
   char robothost[NAMELEN];
   char ls_calib_file[NAMELEN];
   char ir_calib_file[NAMELEN];
   char odo_calib_file[NAMELEN];
   char temp_name[NAMELEN];
   struct xml_in *xmldata;
   struct xml_in *xmllaser;
   uint32_t smrdtime;

   double L, turnangle;
   unsigned char sig;
   double dummyinp=0;
   double *xyzout1;
//Supervisor variables
   int motionstatus_o = 0;
   printf("-----------------------------------------\n");
   printf("MRC started\n");
   ftemp = popen("grep 'cpu MHz' /proc/cpuinfo", "r");
   if (ftemp) {
      fscanf(ftemp, "cpu MHz : %lf", &cpufreqmhz);
      printf("   Running cpu freq %lf MHz\n", cpufreqmhz);
      printf("-----------------------------------------\n\n");
      pclose(ftemp);
   } else
      cpufreqmhz = 500;

//install sighandlers
   if (INSTALL_SEGMENTATIONFAULT_HANDLER){
     if (signal(SIGSEGV, segfaulthandler) == SIG_ERR) {
        perror("signal");
        exit(1);
     }
   }
   if (signal(SIGPIPE, brokenpipehandler) == SIG_ERR) {
      perror("signal");
      exit(1);
   }

// initialisation
   if (gethostname(hostname, 30)) {
      fprintf(stderr, "hostname error \n");
      exit(EXIT_FAILURE);
   }
   strcpy(robothost,"localhost");
   strcat(hostname, "_demo");
   planm.mem = planmem;
   planm.pmem = planmem;
// init plantable
   for (i=0;i<PLANTABLESIZE;i++){
    plantable.plans[i].used=0;
    plantable.plans[i].plan.name=malloc(100000);
   }
   plantable.size=PLANTABLESIZE;
   strcpy(siminterface.fname, "smrsim.dat");
   calibration = 0;
   userif = 0;
   siminterface.simulating = 0;
   initinterp();
   logvar.n=0;
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   lmssrv.config=1;
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;
  
   
   while (EOF != (opt = getopt(argc, argv, "ct:v:l:s:h:u"))) {
      switch (opt) {
      case 'l':
         if (optarg)
            strcpy(siminterface.fname, optarg);
         break;
      case 'c':
         calibration = 1;
         break;
      case 'u':
         serv.port = 31001;
         if (startserver(&serv))
            exit(1);
         userif = 1;
         break;
      case 'h':
           strcpy(robothost,optarg);
         break;

      case 's':
         if (optarg) {
            robot_port = SMRSIM_BASE_PORT + atoi(optarg);
	    lmssrv.port=20000+ atoi(optarg);
            siminterface.simulating = 1;
            stream2lineinit(&s2lbuf);
         } else
            exit(1);
         break;
      case 't':
         serv.port = 31001;
         if (optarg)
            serv.port = 31000 + atoi(optarg);
         if (startserver(&serv)) {
            printf("Could not start server at port %d \n", serv.port);
            exit(1);
         }
         cur_smfunc = sm_server;
         cur_smdata = &sm_main_data;
         stream2lineinit(&s2lbuf);
         server = 1;
         break;
      case 'v':
         if (strcmp(optarg, "off") == 0)
            verbose = 0;
         else
            verbose = 1;
         printf("verb %d \n", verbose);
         break;
      default:
         ;
      }
   }


#include "smr_const.h"
   gpssim.use=0;
   speak.on=0;
   odo.control = 0;
   odo.gyrotype=0;
   odo.enctype=0;
   odo.encwheel=0;
   odo.robotlength = 0.265;
   motcon.robotlength = 0.265;
   motcon.wallinputmode=0;
   motcon.wall_anglegain=1;
   motcon.wallanglel=&dummyinp;
   motcon.wallangler=&dummyinp;
   motcon.kp=10;
   connectioninit();
   motion_init(&motcon);
   motorcontrol.kp = 0;
   motorcontrol.ki = 0;
   line_data.k_filt = 0.8;
   robotinfo.type = DIFFERENTIAL;
   motionconnector.obstaclefront=&defaultobstaclefront;
   motionconnector.obstacleback=&defaultobstacleback;
   strcpy(robotinfo.name,"smr");
  
/*printf("kalman %lf %lf %lf %lf \n",kalmanodo.measurement_noise_std_x,
kalmanodo.measurement_noise_std_y,kalmanodo.process_noise_std_steering_angle,
kalmanodo.process_noise_std_dist); */
      robot = &smrdummy;
      if (rhdConnect('w',robothost,robot_port+1)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      
 
      inputtablesize=getSymbolTableSize('r');
      for (i=0;i<inputtablesize;i++){
        printf(" %s  length %d \n",inputtable[i].name,inputtable[i].length);
      }
       outputtablesize=getSymbolTableSize('w');
      for (i=0;i<outputtablesize;i++){
        printf(" %s  length %d \n",outputtable[i].name,outputtable[i].length);
      }
  
       // interpreter initialization
      init_table();
     
      init_internvar(&internvar);
      xyzout1=putinternvar("xyzout1",1);
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
      gyro= getinputref("XbowYaw",inputtable);  
      steeringangle= getinputref("hakosteeringangle",inputtable);  
      hakoodopulses= getinputref("hakoodopulses",inputtable);
      hakoodopulses1= getinputref("hakoodopulses1",inputtable);
      gpseasting= getinputref("GPSeasting",inputtable);
      gpsnorthing= getinputref("GPSnorthing",inputtable);
      gpsquality= getinputref("GPSquality",inputtable);
      gpssatelites= getinputref("GPSsatused",inputtable);
      gpsdop= getinputref("GPSdop",inputtable);
      
      transpos= getinputref("transpos",inputtable);
      rotpos= getinputref("rotpos",inputtable);
      isimtime= getinputref("simtime",inputtable);
      curvature= getinputref("curvature",inputtable);
      liftinggearpos=getinputref("liftinggearpos",inputtable);
    
      thrustref=getoutputref("thrustref",outputtable);
      rollref=getoutputref("rollref",outputtable);
      pitchref=getoutputref("pitchref",outputtable);
      yawrateref=getoutputref("yawrateref",outputtable);
  
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
      speedref=getoutputref("speedref",outputtable); 
      steeringangleref=getoutputref("steeringangleref",outputtable);
      enginespeedref=getoutputref("enginespeedref",outputtable);
      hakomanual=getoutputref("hakomanual",outputtable);
      cmdtransvel=getoutputref("cmdtransvel",outputtable);
      cmdrotvel=getoutputref("cmdrotvel",outputtable);
      curvatureref=getoutputref("curvatureref",outputtable);
      liftinggearstref=getinputref("liftinggearstateref",outputtable);

       // ESM - added by Jørgen Eriksen
      esmcompassheading=getinputref("esmcompassheading",inputtable);  // Added by JE
      esmpitch=getinputref("esmpitch",inputtable);  // Added by JE
      esmroll=getinputref("esmroll",inputtable);  // Added by JE
      esmBxmagnetic=getinputref("esmBxmagnetic",inputtable);  // Added by JE
      esmBymagnetic=getinputref("esmBymagnetic",inputtable);  // Added by JE
      esmBzmagnetic=getinputref("esmBzmagnetic",inputtable);  // Added by JE
      esmtemperature=getinputref("esmtemperature",inputtable);  // Added by JE

      paintSetServo=getoutputref("paintServoSetPosition",outputtable);  // Added by JE
      paintGetServo=getinputref("servoposition",inputtable);  // Added by JE

// init routeoutputtable
    for (i=0;i<ROUTEOUTPUTTABLESIZE;i++){
           routeoutputtable[i].name=NULL;
	   routeoutputtable[i].in=NULL;
	   routeoutputtable[i].out=NULL;
    }
     routeoutputtable[0].name="funcgen";
      routeoutputtable[0].in=&motcon.funcgenout;

#if (0)
      while(1){
      int j;
        rhdSync();
	printf("new set \n");
        for (j=0;j<inputtablesize;j++){
          printf("\n %d ",j);
          for (i=0;i<inputtable[j].length;i++)
            printf("%d ",inputtable[j].data[i]);
        }	  
      }
#endif 
  
      {
         int len,j;  
         for (i=0;i<inputtablesize;i++){
	   len=inputtable[i].length;
           inputtable[i].inputVar=putinternvar(inputtable[i].name,len);
	   for (j=0;j<len;j++){
	      inputtable[i].inputVar[j]=0;
	      inputtable[i].inputVar[j+len]=0;
	      inputtable[i].inputVar[j+2*len]=1;
	   }
         }
      }
      {
         int len,j;  
         for (i=0;i<outputtablesize;i++){
	   len=outputtable[i].length;
           outputtable[i].inputVar=putinternvar(outputtable[i].name,len);
	   for (j=0;j<len;j++){
	      outputtable[i].inputVar[j]=0;
	      outputtable[i].inputVar[j+len]=0;
	      outputtable[i].inputVar[j+2*len]=1;
	   }
         }
      }
     
      if (robotinit())
            fprintf(stderr,"File robot.conf not found, default SMR-parameters used\n");


   
   printf("\nConnected to %s\n", robotinfo.name);

//*******************************************************************
// Start of calibration
//
//

   strcpy(ls_calib_file, CALIBDIR);
   if (calibration) {
      strcat(ls_calib_file, "smrxqptest");
      ftemp = fopen((char *) ls_calib_file, "w");
      if (ftemp == NULL) {
         printf("\nDirectory 'calib' not found or unwriteable\n");
         exit(0);
      }
      fclose(ftemp);
   }
   strcpy(ls_calib_file, CALIBDIR);
   strcat(ls_calib_file, hostname);
   strcat(ls_calib_file, "_ls_calib.dat");
   strcpy(odo_calib_file, CALIBDIR);
   strcat(odo_calib_file, hostname);
   strcat(odo_calib_file, "_odo_calib.dat");
   strcpy(ir_calib_file, CALIBDIR);
   strcat(ir_calib_file, hostname);
   strcat(ir_calib_file, "_ir_calib.dat");

// printf("calibration %d\n",calibration);
   if (calibration) {
      int choice = 0;
      double avg_s[MAXLINESENSORSIZE];
     

      while (choice != 6) {
         printf("Choose action \n");
         printf("1. calibrate line sensor \n");
         printf("2. update odometry \n");
         printf("3. calibrate front ir sensors \n");
         printf("4. calibrate left ir sensor \n");
         printf("5. calibrate right ir sensor \n");
         printf("6. quit \n");
	 choice=0;
	 fgets(ansbuf,80,stdin);
         sscanf(ansbuf,"%d", &choice);
         switch (choice) {
         case 1:{
               int j;
               ftemp = fopen((char *) ls_calib_file, "w");
               printf("Put white paper under linesensor and enter return\n");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < linesensorsize; i++)
                  avg_s[i] = 0;
               for (i = 0; i < 100; i++) {
                  rhdSync();
                  for (j = 0; j < linesensorsize; j++)
                     avg_s[j] += linesensor->data[j];
               }
               for (i = 0; i < linesensorsize; i++)
                  fprintf(ftemp, "%6.2f ;max_white(%d) \n",
                          avg_s[i] / 100.0, i + 1);
               printf("Put black paper under linesensor and enter return");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < linesensorsize; i++)
                  avg_s[i] = 0;
               for (i = 0; i < 100; i++) {
	          rhdSync();
                  for (j = 0; j < linesensorsize; j++)
                     avg_s[j] += linesensor->data[j];
                 
               }
               for (i = 0; i < linesensorsize; i++)
                  fprintf(ftemp, "%6.2f ;max_black(%d) \n",
                          avg_s[i] / 100.0, i + 1);
               fclose(ftemp);
            }
            break;
         case 2:{
               double L, xcw, ycw, xccw, yccw, wheelbase, Dr_Dl, c0;
	       int npar=0;
               ftemp = fopen(odo_calib_file, "r");
               if (ftemp != NULL) {
                  c0 = 0.0001026;
                  fscanf(ftemp, "%lf %lf %lf", &wheelbase, &Dr_Dl, &c0);
                  fclose(ftemp);
               } else {
                  wheelbase = 0.26;
                  Dr_Dl = 1;
                  c0 = 0.0001026;
               }
               printf("y-coordinates only? (y/n) \n");
                 fgets(ansbuf,80,stdin);
               if (ansbuf[0] == 'y' || ansbuf[0] == 'Y') {
	           npar=0;
                   while (npar < 3){
                    printf("Enter ycw  yccw and L  (meters)\n");
                    fgets(ansbuf,80,stdin);
                    npar=sscanf(ansbuf,"%lf %lf %lf ",  &ycw,  &yccw, &L);
	            if (npar <3) printf("Too few values entered( %d), try again \n",npar);		
		    else if ( L <= 0) {printf(" L must be positive \n");npar=0;}
		    else {
		     printf("You entered  ycw = %lf  yccw = %lf  L = %lf OK?\n",
		            ycw,yccw,L);
			   fgets(ansbuf,80,stdin);
                     if (ansbuf[0] != 'y' && ansbuf[0] != 'Y') npar=0;
		    }      
		  }	  
                  UMB_updatey(&wheelbase, &Dr_Dl, ycw, yccw, L);
               } else {
                   npar=0;
		  while (npar < 5){
                    printf("Enter xcw ycw xccw yccw and L  (meters)\n");
                    fgets(ansbuf,80,stdin);
                    npar=sscanf(ansbuf,"%lf %lf %lf %lf %lf", &xcw, &ycw, &xccw, &yccw,
                        &L);
	            if (npar <5) printf("Too few values entered( %d), try again \n",npar);		
		    else if ( L <= 0) {printf(" L must be positive \n");npar=0;}
		    else {
		     printf("You entered xcw = %lf ycw = %lf xccw = %lf yccw = %lf  L = %lf OK?\n",
		            xcw,ycw,xccw,yccw,L);
			   fgets(ansbuf,80,stdin);
                     if (ansbuf[0] != 'y' && ansbuf[0] != 'Y') npar=0;
		    }      
		  }	
                  UMB_updatexy(&wheelbase, &Dr_Dl, xcw, ycw, xccw, yccw,
                               L);
               }
               ftemp = fopen(odo_calib_file, "w");
               if (ftemp != NULL) {
                  fprintf(ftemp, "%lf %lf %lf", wheelbase, Dr_Dl, c0);
                  fclose(ftemp);
               }
            }
            break;
         case 3:{
               int j;
               double ir1[6], ir2[6], d1[6], d2[6];
               ftemp = fopen((char *) ir_calib_file, "r");
               if (ftemp != NULL) {
                  for (i = 0; i < 6; i++)
                     fscanf(ftemp, "%lf %lf", &ir_param[i].k1,
                            &ir_param[i].k2);
                  fclose(ftemp);
               } else {
                  for (i = 0; i < 6; i++) {
                     ir_param[i].k1 = 16.0;
                     ir_param[i].k2 = 76.0;
                  }
               }
               printf("Put box 0.15m in front of smr and enter return\n");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < 3; i++) {
                  ir1[i] = 0;
                  ir2[i] = 0;
                  d1[i] = 0.15;
                  d2[i] = 0.40;
               }
               for (i = 0; i < 100; i++) {
                  rhdSync();
                  for (j = 0; j < 3; j++)
                     ir1[j] += irsensor->data[j + 1];
               }
               for (i = 0; i < 6; i++)
                  ir1[i] /= 100.0;
               printf("Put box 0.40m in front of smr and enter return\n");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < 100; i++) {
                  rhdSync();
                  for (j = 0; j < 3; j++)
                     ir2[j] += irsensor->data[j + 1];
               }
               for (i = 0; i < 6; i++)
                  ir2[i] /= 100.0;
               calc_ir_param(&ir_param[1], d1, ir1, d2, ir2, 3);
               ftemp = fopen((char *) ir_calib_file, "w");
               for (i = 0; i < 6; i++)
                  fprintf(ftemp, "%6.2lf %6.2lf \n", ir_param[i].k1,
                          ir_param[i].k2);
               fclose(ftemp);
            }
            break;

         case 4:{
               int j;
               double ir1[6], ir2[6], d1[6], d2[6];
               ftemp = fopen((char *) ir_calib_file, "r");
               if (ftemp != NULL) {
                  for (i = 0; i < 6; i++)
                     fscanf(ftemp, "%lf %lf", &ir_param[i].k1,
                            &ir_param[i].k2);
                  fclose(ftemp);
               } else {
                  for (i = 0; i < 6; i++) {
                     ir_param[i].k1 = 16.0;
                     ir_param[i].k2 = 76.0;
                  }
               }
               printf("Put box 0.15m left of smr and enter return\n");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < 1; i++) {
                  ir1[i] = 0;
                  ir2[i] = 0;
                  d1[i] = 0.15;
                  d2[i] = 0.40;
               }
               for (i = 0; i < 100; i++) {
                  rhdSync();
                  for (j = 0; j < 1; j++)
                     ir1[j] += irsensor->data[j];
               }
               for (i = 0; i < 1; i++)
                  ir1[i] /= 100.0;
               printf("Put box 0.40m left of smr and enter return\n");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < 100; i++) {
                  rhdSync();
                  for (j = 0; j < 1; j++)
                     ir2[j] += irsensor->data[j];
               }
               for (i = 0; i < 6; i++)
                  ir2[i] /= 100.0;
               calc_ir_param(&ir_param[0], d1, ir1, d2, ir2, 1);
               ftemp = fopen((char *) ir_calib_file, "w");
               for (i = 0; i < 6; i++)
                  fprintf(ftemp, "%6.2lf %6.2lf \n", ir_param[i].k1,
                          ir_param[i].k2);
               fclose(ftemp);
            }
            ;
            break;

         case 5:{
               int j;
               double ir1[6], ir2[6], d1[6], d2[6];
               ftemp = fopen((char *) ir_calib_file, "r");
               if (ftemp != NULL) {
                  for (i = 0; i < 6; i++)
                     fscanf(ftemp, "%lf %lf", &ir_param[i].k1,
                            &ir_param[i].k2);
                  fclose(ftemp);
               } else {
                  for (i = 0; i < 6; i++) {
                     ir_param[i].k1 = 16.0;
                     ir_param[i].k2 = 76.0;
                  }
               }
               printf("Put box 0.15m right of smr and enter return\n");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < 1; i++) {
                  ir1[i] = 0;
                  ir2[i] = 0;
                  d1[i] = 0.15;
                  d2[i] = 0.40;
               }
               for (i = 0; i < 100; i++) {
                  rhdSync();
                  for (j = 0; j < 1; j++)
                     ir1[j] += irsensor->data[j + 4];
               }
               for (i = 0; i < 1; i++)
                  ir1[i] /= 100.0;
               printf("Put box 0.40m right of smr and enter return\n");
               fgets(ansbuf,80,stdin);
               for (i = 0; i < 100; i++) {
                  rhdSync();
                  for (j = 0; j < 1; j++)
                     ir2[j] += irsensor->data[j + 4];
               }
               for (i = 0; i < 6; i++)
                  ir2[i] /= 100.0;
               calc_ir_param(&ir_param[4], d1, ir1, d2, ir2, 1);
               ftemp = fopen((char *) ir_calib_file, "w");
               for (i = 0; i < 6; i++)
                  fprintf(ftemp, "%6.2lf %6.2lf \n", ir_param[i].k1,
                          ir_param[i].k2);
               fclose(ftemp);
            }
            ;
            break;



         case 6:
         default:
            ;
         }                      // end switch
      }                         // while
   }
//********************************************************************
//  Initialization of constants 
//

 
   ftemp = fopen((char *) odo_calib_file, "r");
   if (ftemp == NULL) {
      strcpy(temp_name, "/usr/local/smr/");
      strcat(temp_name, odo_calib_file);
      ftemp = fopen((char *) temp_name, "r");
      if (ftemp != NULL)
         fprintf(stderr, "Default odometry calibration file used \n");
   }
   if (ftemp != NULL) {
      double Dr_Dl=0, c0=0;
      int npar=0;
      npar = fscanf(ftemp, "%lf %lf %lf", &odo.w, &Dr_Dl, &c0);
      if (npar == 3) {
         odo.cr = c0;
         odo.cl = c0;
      }
      else {
         printf("Wrong size configuration file\n");
         printf("   odo.cr and odo.cl values from robot.conf used.\n");
         printf("   Calibration might be off\n");
         printf("   odo.cr %lf odo.cl %lf\n",odo.cr,odo.cl);
      }
      
      odo.cr *= (2 * Dr_Dl / (1 + Dr_Dl));
      odo.cl *= (2 / (1 + Dr_Dl));
      
      printf("   Calibrated odo members\n");
      printf("   odo.cr %10.9lf odo.cl %10.9lf\n",odo.cr,odo.cl);
      fclose(ftemp); 
   } else
      fprintf(stderr, "Odometry not calibrated \n");

   ftemp = fopen((char *) ls_calib_file, "r");
   if (ftemp == NULL) {
      strcpy(temp_name, "/usr/local/smr/");
      strcat(temp_name, ls_calib_file);
      ftemp = fopen((char *) temp_name, "r");
      if (ftemp != NULL)
         fprintf(stderr, "Default line sensor calibration file used \n");
   }
   if (ftemp != NULL) {
      int i;
      double white[MAXLINESENSORSIZE], black[MAXLINESENSORSIZE];
      char str[80];

      printf("Line sensor calibration values\n");
      for (i = 0; i < linesensorsize; i++) {
         fscanf(ftemp, "%lf %s\n", &white[i], str);
         printf("%6.2lf ", white[i]);
      }

      for (i = 0; i < linesensorsize; i++) {
         fscanf(ftemp, "%lf %s\n", &black[i], str);
         printf("%6.2lf ", black[i]);

      }
      printf("\n");

      calc_linesensor_param(ls_param, white, black, linesensorsize);
      fclose(ftemp);
   } else
      fprintf(stderr, "line sensor not calibrated\n");


   ftemp = fopen((char *) ir_calib_file, "r");
   if (ftemp == NULL) {
      strcpy(temp_name, "/usr/local/smr/");
      strcat(temp_name, ir_calib_file);
      ftemp = fopen((char *) temp_name, "r");
      if (ftemp != NULL)
         fprintf(stderr, "Default IR-calibration file used \n");
   }
   if (ftemp != NULL) {
      for (i = 0; i < 6; i++)
         fscanf(ftemp, "%lf %lf", &ir_param[i].k1, &ir_param[i].k2);
      fclose(ftemp);
   } else {
      for (i = 0; i < 6; i++) {
         ir_param[i].k1 = 16;
         ir_param[i].k2 = 76;
      }
      fprintf(stderr,
              "Warning IR sensors not calibrated , using standard calibration \n");
   }




   for (i = 0; i < linesensorsize; i++)
      line_data.avg_s[i] = 0;

   if (optind < argc) {
      if (!loadplan(argv[optind], &plan, &planm)) {
         plan.pline = plan.start;
         setcurrentplan(&plan);
      } else {
         printf("File %s not found\n", argv[optind]);
      }
      cur_smfunc = sm_interp;
      cur_smdata = &sm_main_data;
   } else if (server == 1);
   else {
//*******************************************************************
//  Start of menu
//
      printf("Chose action \n");
      printf("1. Square clockwise (right turns) \n");
      printf("2. Square counter clockwise (left turns) \n");
      printf("3. interpreter\n");
      printf("4. Start server\n");
      choice=0;
      fgets(ansbuf,80,stdin);
      sscanf(ansbuf,"%d", &choice);
      switch (choice) {
      case 1:{
            int npar=0;
	    while (npar!=1){
	      printf("\n Enter sidelength (meter)");
	      fgets(ansbuf,80,stdin);
      	      npar=sscanf(ansbuf,"%lf", &L);
	      if (L <=0) {printf("L must be positive \n");npar=0;}
            } 
            turnangle = -M_PI / 2;
            sm_main_data.p[0] = L;
            sm_main_data.p[1] = turnangle;
            cur_smfunc = sm_square;
            cur_smdata = &sm_main_data;
         }
         break;
      case 2:{
             int npar=0;
	     while (npar!=1){
	       printf("\n Enter sidelength (meter)");
	       fgets(ansbuf,80,stdin);
      	       npar=sscanf(ansbuf,"%lf", &L);
	       if (L <=0) {printf("L must be positive \n");npar=0;}
            } 
            turnangle = M_PI / 2;
            sm_main_data.p[0] = L;
            sm_main_data.p[1] = turnangle;
            cur_smfunc = sm_square;
            cur_smdata = &sm_main_data;
         }
         break;
	 
        case 3:{
            int ok = 0;
            while (!ok) {
               printf("\n track file name: ");
               scanf(" %s", track_file);
               getchar();
               if (!loadplan(track_file, &plan, &planm)) {
                  plan.pline = plan.start;
                  setcurrentplan(&plan);
                  ok = 1;
               } else
                  printf("File %s not found\n", track_file);
            }
            cur_smfunc = sm_interp;
            cur_smdata = &sm_main_data;
         }
         break;

      case 4:{
            serv.port = 31001;
            if (startserver(&serv))
               exit(1);
            stream2lineinit(&s2lbuf);
            cur_smfunc = sm_server;
            cur_smdata = &sm_main_data;
         }
         break;


      default:
      exit(1);
         ;
      }
   }

   sm_datapool_init();
//smrsound_init();
//smrspeak_init();
   soundpipe = NULL;
   speakpipe = NULL;
   systempipe = NULL;
   sem_init(&smrsystem_sem,0,0);
   pthread_create(&smrsystemptr, NULL, smrsystem, NULL);
if (speak.on) smrsystemif_init();
//smrsystemif("play smrsound/DingDong.wav");




// **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   if (lmssrv.config) {
      int errno = 0; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);

   xmllaser=xml_in_init(4096,32);
   printf(" laserserver xml initialized \n");

}   
   
// **************************************************
//  GPS mouse initialization
//


   if (gps.config) {
      printf("Trying to connect to UTMgpsd\n");
      sleep(2);
      gps_socket_connect(&gps);
      if (gps.client.error) {
         fprintf(stderr, "   Can't connect to GPS server \n");
//       exit(gps.client.error);
         gps.status = 0;
      } else {
         printf("   Connected to UTMgpsd\n");
         gps.status = 1;
      }
      //Initialize GPS struct
      gps_init(&gps);

   }
   if (gpssim.use){
   double *a;
      a= putinternvar("hakosteeringangle",1);
      *a=0;
      navstat= putinternvar("hakonavigationmode",1);
      *navstat=1;
   }
    

   line_data.noline_b = 0;
   line_data.count = 0;
   read_wdssparam(&motcon);
   printf("First read from robot\n");
   if (resetmotorr)
      resetmotorr->updated=1;
   if (resetmotorl)
      resetmotorl->updated=1; 
   rhdSync();
   if (renc) odo.renc=renc->data[0]; else odo.renc=0;
   if (lenc) odo.lenc=lenc->data[0]; else odo.lenc=0;
   if (steeringangle) odo.steeringangle=steeringangle->inputVar[0]; else odo.steeringangle=0;
   if (curvature) odo.curvature=curvature->inputVar[0]; else odo.curvature=0;
   switch (robotinfo.type){
      case VELOMEGA:
         odo.lenc = *transpos->data;
	 odoconnector.dth=&rotpos->inputVar[0];
	 gyro1gain=1/40000.;
	 odo.dth =*rotpos->data/40000.;
      
      break;
      case ACKERMAN:
      {
        if (strcmp(robotinfo.name, "hako") == 0) {
	  odo.steeringangle = steeringangle->data[0]/1800.*M_PI;
          odo.lenc = hakoodopulses->data[0];
          pulsecount +=hakoodopulses->data[0];
	  if (hakoodopulses1){
	    odo.lenc=hakoodopulses1->data[0];
	  }  
	  odo.renc=0;
	} 
      }
      break;
      case DIFFERENTIAL:
      {
         
      }
      break;
      default:
       // printf("unknown type %d \n",robotinfo.type);
      ;
      }
      smrdtime = robot->ts;
   
   printf("Odoinitialisation\n");
   odo_init(&odo);
//odo.control=0;
   odocov.E2=0.0001;
   odocov.A2=0.01;
   odocov.lK=odo.cl;
   odocov.rK=odo.cr;
   odocov.B=odo.w;
   resetupdatestruct(&odocov);
 
  
   init_blocking(&bt);
   sm_reset(cur_smdata);
   sm_reset(&kalmanstate);
   runsmr = 1;
   battery.vout = 14.0;
   powersupply.vout = 0;
   inputready = 0;
   crossingliner = 0;
   camlinepos = 0;
   motcon.type = robotinfo.type;
   motionconnector.w2odox=&laserpar[0];	    
   motionconnector.w2odoy=&laserpar[1];   
   motionconnector.w2odoth=&laserpar[2];
     
//***************************************************
// initialization of line navigation

   lineest.Ndetect = 0;
   for (i = 0; i < 8; i++)
      ls_input[i] = robot->ls[i];
   linesensor_correct(ls_corrected, ls_input, ls_param, linesensorsize);
   for (i = 0; i < 8; i++)
      lineest.lsold[i] = ls_corrected[i];
   lin.data = lineest.data;
   lin.N = 8;

   printf("Kalman initialisation\n");
// **************************************************
//  Kalman filter initialization
//
   inputready = 0;
   odo.time = 0;
   odo.type = robotinfo.type;


      
   fprintf(stderr,"kalmanodo.offset_x %lf kalmanodo.offset_y %lf\n",kalmanodo.offset_x,kalmanodo.offset_y);
   
   kalmanodo.range = 1000000.0;
   kalmanodo.error_std_x = 1000;        //kalmanodo.offset_x*kalmanodo.offset_x;
   kalmanodo.error_std_y = 1000;        //kalmanodo.offset_y*kalmanodo.offset_y;
   kalmanodo.error_std_th = 1;
   kalmanodo.robot_length = odo.robotlength;

   IAU_kalman_odometry_init(&kalmanodo);
    motcon.irdistl = &ir_dist[0];
    motcon.irdistr = &ir_dist[4];
    
    for (i=0;i< modulelist[0].functionlist[0].parlist->N;i++){
       printf("%s \n",modulelist[0].functionlist[0].parlist->list[i].parname);
       }


// **************************************************
//  main loop
//

   printf("\nStarting processing of commands\n");
   printf("-----------------------------------------\n\n");
    while (runsmr) {
      int arg = 0;
      double sampletime;
      count++;
      if (count % 100 == 0) {
       
#if (0)
	 int i;
	 printf("\n");
	 for (i=0;i<linesensorsize;i++){
          printf("%6.2lf ",ls_corrected[i]);
	 }
         printf("\n");
           int i;
            printf("\n");
            for(i=0;i<362;i++)
            printf("%d ",laserscan[2*i+1]);
            printf("vr, vl %d  %d \n",robot->right.speed,robot->left.speed);
      printf("hakospeed steeringangle %lf %lf \n",hako.speed,hako.steeringAngle);
 #endif
      }
      // Read data from the robot
      //**************************************************
  
      //Get GPS measurement every 1 sec
      if (gps.run && gps.config && gps.status && (count % 100) == 0) {
          fprintf(stderr,"GPS READ - Before GPS\n");
         //      gps_clear(&gps);
//      if((count%100) == 0)
         gps_read(&gps, POLL_GPS);
//         gps_read(&gps,POLL_GPS_STATUS);
         if (gps.use) {
//         fprintf(stderr,"Using gps measurement\n");

            hako.gps.easting = gps.UTM.easting;
            hako.gps.northing = gps.UTM.northing;

#if (0)
         fprintf(stderr,"%8i> Gps pos northing: %8f easting: %f8  Valid: %i \n", 
              count, hako.gps.northing,hako.gps.easting,gps.UTM.valid);
         fprintf(stderr,"h: %2i m: %2i s: %2i Cs: %2i\n", gps.UTM.time_h, 
             gps.UTM.time_m,gps.UTM.time_s, gps.UTM.time_cs);
         printf("     Zone %i Visible satellites %i\n",gps.stat.zone,
             gps.stat.sats_in_view);
        
#endif
         }
      }
    
        
	
      if (lmssrv.config && lmssrv.status && lmssrv.connected){
        char buf[256];
	int len;
	struct timeval tv;
        struct timezone tz;
	if (siminterface.simulating){
	   tv.tv_sec=isimtime->data[0];
	   tv.tv_usec=isimtime->data[1];
	}
	else {   
	  gettimeofday(&tv, &tz);
	}
	len=sprintf(buf,"<pt tod=\"%ld.%06ld\" x=\"%.4f\" y=\"%.4f \" th=\"%.4f \" />\n",
	     tv.tv_sec,tv.tv_usec,*internvar.podox,*internvar.podoy,*internvar.podoth);
        send(lmssrv.sockfd,buf,len,0);
	//if ((count & 0x7f)==0)
          // printf("%s",buf);
        while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
        char buf[256];
	int len;
	struct timeval tv;
        struct timezone tz;
        if (siminterface.simulating){
	   tv.tv_sec=isimtime->data[0];
	   tv.tv_usec=isimtime->data[1];
	}
	else {   
	  gettimeofday(&tv, &tz);
	}
	len=sprintf(buf,"<pt tod=\"%ld.%06ld\" x=\"%.4f\" y=\"%.4f \" th=\"%.4f \" />\n",
	     tv.tv_sec,tv.tv_usec,*internvar.podox,*internvar.podoy,*internvar.podoth);
        send(camsrv.sockfd,buf,len,0);
	//if ((count & 0x7f)==0)
          // printf("%s",buf);
        while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }


       	rhdSync();
       	inputvar_update();
       	odo.dth=(*odoconnector.dth+gyro1off) *gyro1gain; 
	motcon.w2odo.x=*motionconnector.w2odox;
	motcon.w2odo.y=*motionconnector.w2odoy;
	motcon.w2odo.th=*motionconnector.w2odoth;
        motcon.pose3d.x=*motionconnector.x;
	motcon.pose3d.y=*motionconnector.y;
	motcon.pose3d.z=*motionconnector.z;
        motcon.tick = count;
       	if (gpseasting) 		hako.gps.easting=gpseasting->data[0]+1.0e-6*gpseasting->data[1];  
       	if (gpsnorthing)		hako.gps.northing=gpsnorthing->data[0]+1.0e-6*gpsnorthing->data[1];
       	if (gpssatelites)	hako.gps.satellites=gpssatelites->data[0];
       	if (gpsquality)		hako.gps.quality=gpsquality->data[0];
       	if (gpsdop)		{ hako.gps.dop=gpsdop->data[0]+gpsdop->data[1]*1e-6;}
	if (renc) odo.renc=renc->data[0]; else odo.renc=0;
   	if (lenc) odo.lenc=lenc->data[0]; else odo.lenc=0;
   	if (steeringangle) odo.steeringangle=steeringangle->inputVar[0]; else odo.steeringangle=0;
   	if (curvature) odo.curvature=curvature->inputVar[0]; else odo.curvature=0; 
	if (liftinggearpos) hako.liftingGearPos=liftinggearpos->inputVar[0];        
        if (gpssim.use){
	  hako.gps.easting=odo.pose.x;  
       	  hako.gps.northing=odo.pose.y;
       	  hako.gps.satellites=7;
       	  hako.gps.quality=3;
	  *navstat=*internvar.phakonavigationmoderef;
        }
        
         // ESM - added by Jørgen Eriksen
        double deg2rad = M_PI / 180;  // Added by JE
        if (esmcompassheading) esmCompass = esmcompassheading->data[0] / 1000.0;  // Added by JE
        if (esmpitch)          esmPitch   = ((esmpitch->data[0] / 1000.0) + pitchoffset) * deg2rad;  // Added by JE
        if (esmroll)           esmRoll    = ((esmroll->data[0]  / 1000.0) + rolloffset)  * deg2rad;  // Added by JE
        if (esmBxmagnetic)     esmBx      = esmBxmagnetic->data[0] / 1000.0;  // Added by JE
        if (esmBymagnetic)     esmBy      = esmBymagnetic->data[0] / 1000.0;  // Added by JE
        if (esmBzmagnetic)     esmBz      = esmBzmagnetic->data[0] / 1000.0;  // Added by JE
        if (esmtemperature)    esmTemp    = esmtemperature->data[0] / 1000.0;  // Added by JE

        if (rollpitchon)  // Added by JE
        {  // Added by JE
          // Low Pass filter for ESM Pitch values
          lp_pitch    = filter_b * esmPitch + dummy_pitch;  // Added by JE
          dummy_pitch = filter_b * esmPitch - filter_a * lp_pitch;  // Added by JE

          // Low Pass filter for ESM Roll values
          lp_roll     = filter_b * esmRoll  + dummy_roll;  // Added by JE
          dummy_roll  = filter_b * esmRoll  - filter_a * lp_roll;  // Added by JE

          // Calculate pitch / roll distance and angle
          dist_roll  = height_gps_ant * sin(lp_roll);   // Added by JE
          dist_pitch = height_gps_ant * sin(lp_pitch);  // Added by JE
          dist_rp    = sqrt(dist_roll * dist_roll + dist_pitch * dist_pitch);  // Added by JE
          th_rp      = atan2(dist_roll, dist_pitch);  // Added by JE

          // Correct the UTM coordinates for the pitch / roll error
          corrected_UTMe = hako.gps.easting  + dist_rp * cos(vget(kalmanodo.kalman->Xpost, 2) + th_rp);  // Added by JE
          corrected_UTMn = hako.gps.northing + dist_rp * sin(vget(kalmanodo.kalman->Xpost, 2) + th_rp);  // Added by JE

          /* Debug ESM correction
          if (esmCount > 40)  // Added by JE
          {  // Added by JE
             printf("Normal UTM coordinates: %lf, %lf\n",hako.gps.easting,hako.gps.northing);   // Added by JE
             printf("Corrected UTM coordinates: %lf, %lf\n",corrected_UTMe,corrected_UTMn);   // Added by JE
             printf("Roll: %lf - Pitch: %lf\n",esmRoll,esmPitch);  // Added by JE
             printf("LP-Roll: %lf - LP-Pitch: %lf\n",lp_roll,lp_pitch);  // Added by JE
             esmCount = 0;  // Added by JE
          }  // Added by JE
          esmCount++;  // Added by JE
          */ 

          hako.gps.easting  = corrected_UTMe;  // Added by JE
          hako.gps.northing = corrected_UTMn;  // Added by JE
        }

        if (paintGetServo) paintServoValue = paintGetServo->data[0];  // Added by JE

        if (paintServoValue != paintServoStatus && paintSetServo != 0 )  // Added by JE
        {    // Added by JE
          *paintSetServo->data=paintServoStatus;  // Added by JE
          paintSetServo->updated=1;  // Added by JE
        }  // Added by JE

    // Read the rest of robot sensors
        switch(robotinfo.type){
	 case VELOMEGA:
           odo.lenc = *transpos->data; 
	 break;           
         case ACKERMAN :{
            if (strcmp(robotinfo.name, "hako") == 0) {	         
		
               odo.steeringangle = steeringangle->data[0]/1800.*M_PI;
               odo.lenc = hakoodopulses->data[0];
               pulsecount +=hakoodopulses->data[0] ;
	       if (hakoodopulses1){
	         odo.lenc=hakoodopulses1->data[0];
	       }
	     
	       
            } else if (strcmp(robotinfo.name, "ackerbot") == 0)            // ackerbot
            {
	       double tmp;
             
#define SERVO_CENTER 257.7
#define ENDTOEND 111.9
#define ANGLEATEND (atan(265.0/449.9))
               
                tmp=-*renc->data;
               if (tmp > 600)
                 tmp= SERVO_CENTER;
               if ((count % 100)==0)
	       printf(" %lf  \n",tmp);    
               odo.steeringangle =
                   ((double) (tmp  - SERVO_CENTER)
                    / (ENDTOEND / 2.0)) * ANGLEATEND;

               odo.dth = -gyro1 / 180 * M_PI * 1.018;
               odo.time +=
                   (double) (robot->ts - smrdtime) / cpufreqmhz * 1e-6;
               sampletime =
                   ((double) (robot->ts - smrdtime)) / cpufreqmhz * 1e-6;
               smrdtime = robot->ts;
              }else{
	        odo.steeringangle=*odoconnector.steeringangle;//
	       
	      }
            // actual steeringangle is needed by motion control
            motcon.steeringangleNow = odo.steeringangle;
	      
         }
	 break;  
         case DIFFERENTIAL:{
	 #if (0)       
            odo.time +=
                (double) (robot->ts - smrdtime) / cpufreqmhz * 1e-6;
            sampletime =
                ((double) (robot->ts - smrdtime)) / cpufreqmhz * 1e-6;
            smrdtime = robot->ts;
	  #endif  
         }
	 break;
	 default:
	 ;
	  //printf("unknown type %d \n",robotinfo.type);
	 }
            // Transfer variables
            bt.velocity = odo.vel;

            // Run block detecting:
            detect_blocking(&bt);
            // Interpret output:
            blocked = bt.blocked;
	    if (irsensor){
       	      for (i=0;i<6;i++)
	        irin[i]=irsensor->data[i];	
              ir2dist1(ir_dist, irin, ir_param, ir_samp, 6);    
	    }	   
           
	    if (linesensor){
	      linesensor_correct(ls_corrected, linesensor->data, ls_param,
                              linesensorsize);

              find_line(&line_data, ls_corrected, linesensorsize);
            }
            // select line type
            switch (linetype) {
            case LINE_MIDDLE_B:
               motcon.linepos = line_data.linepos_b;
               motcon.noline = line_data.noline_b;
               break;
            case LINE_LEFT_B:
               motcon.linepos = line_data.left_edge_b;
               motcon.noline = line_data.noline_b;
               break;
            case LINE_RIGHT_B:
               motcon.linepos = line_data.right_edge_b;
               motcon.noline = line_data.noline_b;
               break;
            case LINE_MIDDLE_W:
               motcon.linepos = line_data.linepos_w;
               motcon.noline = line_data.noline_w;
               break;
            case LINE_LEFT_W:
               motcon.linepos = line_data.left_edge_w;
               motcon.noline = line_data.noline_w;
               break;
            case LINE_RIGHT_W:
               motcon.linepos = line_data.right_edge_w;
               motcon.noline = line_data.noline_w;
               break;
            case LINE_MIDDLE_B_CAM:
               motcon.linepos = camlinepos;
               motcon.noline = 0;
               break;
            case LINE_TEST:
               motcon.linepos =
                   -(odo.pose.y + odo.pose.th * 0.21) * 100 + lineoffset;
               motcon.noline = 0;
               break;
            default:
               motcon.linepos = 0;
           }
          
         // Run odometry update and 
         //**************************************************
	 if (strcmp(robotinfo.name, "guidebot") == 0){
	    if (lenc->updated && ((count %6)== 0) ){
	      odo.time=lenc->timestamp->tv_sec+(double)lenc->timestamp->tv_usec*0.000001;
	      odo_update(&odo);
	    }
	 }
	 else{
           odo_update(&odo);
	 }
	 odocov.lTick=odo.dencl;
         odocov.rTick=odo.dencr;
         updatecov(&odocov);
     
         // Run estimators and apply changes if configured
         //**************************************************

         kalmanodo.run = kalmanon;
         kalmanodo.use = usekalmanodo;
	 if (kalmanodo.config && kalmanodo.status == 0) {
            // Transfer HAKO variables to the variables of the kalman filter
            kalmanodo.dist = odo.dv;
            kalmanodo.dth = odo.dthout;
	    if (steeringangle)
              kalmanodo.steering_angle = steeringangle->data[0]/1800.*M_PI;
            vput(kalmanodo.measurement, 0,
                 hako.gps.easting - kalmanodo.offset_x);
            vput(kalmanodo.measurement, 1,
                 hako.gps.northing - kalmanodo.offset_y);

	 
	   sm_update(&kalmanstate);
	   switch (kalmanstate.state){
	   case KALMANOFF:{
	     if (kalmanodo.run)
	     {
	        kalmanstate.state=KALMAN_PRE_INIT;
		kalman_first_gps.count = 0;
	     }
	   }
	   break;
	   
           case KALMAN_PRE_INIT:
           {  // initialize kalman filter from two measurements
              // from GPS in fix-mode (quality==3) after 1m
              if (hako.gps.quality == 3)
              { // in fix mode
                if (kalman_first_gps.count == 0)
                { // first measurement
                  kalman_first_gps.x = vget(kalmanodo.measurement, 0);
                  kalman_first_gps.y = vget(kalmanodo.measurement, 1);
                  kalman_first_gps.count = 1;
                  printf("Kalman filter first GPS fix - pre init state at (%.2fx,%.2fy)\n",
                         kalman_first_gps.x, kalman_first_gps.y);
                }
                else
                {
                  kalman_first_gps.count++;
                  if (kalman_first_gps.count % 20 == 0)
                    printf("Kalman filter waiting for more updates to initialize"
                          " (update %d driven %.2fm of 0.5m)\n", kalman_first_gps.count, 
                          fabs(odo.dist - kalmanstate.startdist));
                  if (fabs(odo.dist - kalmanstate.startdist) > 0.5)
                  { // there should be basis for a fine first heading update
                    double x2 = vget(kalmanodo.measurement, 0);
                    double y2 = vget(kalmanodo.measurement, 1);
                    double h2 = atan2(y2 - kalman_first_gps.y, x2 - kalman_first_gps.x);
                    vput(kalmanodo.kalman->Xpost, 0, x2);
                    vput(kalmanodo.kalman->Xpost, 1, y2);
                    vput(kalmanodo.kalman->Xpost, 2, h2);
                    // continue normal kalman initialization
                    kalmanstate.state=KALMANINIT;
                    h2 = 90.0 - h2 * 180.0 / M_PI;
                    if (h2 < 0.0)
                      h2 += 360.0;
                    printf("Kalman pre-initialization finished at (%.2fx,%.2fy) - heading %.2f (compas)\n", x2, y2, h2);
                  }
                }
              }
           }
	   break;
	   case KALMANINIT:{
	     if (!kalmanodo.run)
	        kalmanstate.state=KALMANOFF;
	     else {
	       if (robotinfo.type == ACKERMAN) {
                // Run nonlinear update of the ackerman vehicle
                 IAU_kalman_open_ackerman_predict(&kalmanodo);
	       }else if ( robotinfo.type == DIFFERENTIAL) {
                 IAU_kalman_open_differential_predict(&kalmanodo);
	       }else if ( robotinfo.type == VELOMEGA) {  // Added by JE
                 IAU_kalman_open_differential_predict(&kalmanodo);  // Added by JE
               }  // Added by JE

	       if (hako.gps.quality==3 || hako.gps.quality==4){
                  IAU_kalman_open_correct(&kalmanodo);
	       }
	       else{
                  vput(kalmanodo.kalman->Xpost, 0,
                       vget(kalmanodo.kalman->Xpre, 0));
                  vput(kalmanodo.kalman->Xpost, 1,
                       vget(kalmanodo.kalman->Xpre, 1));
                  vput(kalmanodo.kalman->Xpost, 2,
                       vget(kalmanodo.kalman->Xpre, 2));
               }
	       if (fabs(odo.dist - kalmanstate.startdist) > 2 ){
	          if (hako.gps.quality == 3 )
		     kalmanstate.state=KALMANGPS3;
		  else if (hako.gps.quality == 4 )
		     kalmanstate.state=KALMANGPS3_4;
	       } 
            }
	  }
	  break;
	  
	  case KALMANGPS3:{
	     if (!kalmanodo.run)
	        kalmanstate.state=KALMANOFF;
	     else if (hako.gps.quality==4)
	        kalmanstate.state=KALMANGPS3_4;
	     else {
	       if (robotinfo.type == ACKERMAN) {
                // Run nonlinear update of the ackerman vehicle
                 IAU_kalman_open_ackerman_predict(&kalmanodo);
	       }else if ( robotinfo.type == DIFFERENTIAL) {
                 IAU_kalman_open_differential_predict(&kalmanodo);
	       }else if ( robotinfo.type == VELOMEGA) {  // Added by JE
                 IAU_kalman_open_differential_predict(&kalmanodo);  // Added by JE
               }  // Added by JE

	       if (hako.gps.quality==3  ){
                  IAU_kalman_open_correct(&kalmanodo);
	       }
	       else{
                  vput(kalmanodo.kalman->Xpost, 0,
                       vget(kalmanodo.kalman->Xpre, 0));
                  vput(kalmanodo.kalman->Xpost, 1,
                       vget(kalmanodo.kalman->Xpre, 1));
                  vput(kalmanodo.kalman->Xpost, 2,
                       vget(kalmanodo.kalman->Xpre, 2));
               }
	       
	     }
          }
	  break;
	  case KALMANGPS4:{
	     if (!kalmanodo.run)
	        kalmanstate.state=KALMANOFF;
	     else if (hako.gps.quality==3)
	        kalmanstate.state=KALMANGPS3;
	     else if(!(hako.gps.quality==4))
	     	kalmanstate.state=KALMANGPS3_4;
	     else {
	       if (robotinfo.type == ACKERMAN) {
                // Run nonlinear update of the ackerman vehicle
                 IAU_kalman_open_ackerman_predict(&kalmanodo);
	       }
	       else if ( robotinfo.type == DIFFERENTIAL) {
                 IAU_kalman_open_differential_predict(&kalmanodo);
	       }
	       else if ( robotinfo.type == VELOMEGA) {  // Added by JE
                 IAU_kalman_open_differential_predict(&kalmanodo);  // Added by JE
               }  // Added by JE
               IAU_kalman_open_correct(&kalmanodo);
	     }
          }
	  break;
	  
	  case KALMANGPS3_4:{
	     if (!kalmanodo.run)
	        kalmanstate.state=KALMANOFF;
	     else if (hako.gps.quality==3)
	        kalmanstate.state=KALMANGPS3;
	     else if ((hako.gps.quality==4) && kalmanstate.time >10)
	        kalmanstate.state=KALMANGPS4;
	     else {
	       if (robotinfo.type == ACKERMAN) {
                // Run nonlinear update of the ackerman vehicle
                 IAU_kalman_open_ackerman_predict(&kalmanodo);
	       }else if ( robotinfo.type == DIFFERENTIAL) {
                 IAU_kalman_open_differential_predict(&kalmanodo);
	       }
	       {
                  vput(kalmanodo.kalman->Xpost, 0,
                       vget(kalmanodo.kalman->Xpre, 0));
                  vput(kalmanodo.kalman->Xpost, 1,
                       vget(kalmanodo.kalman->Xpre, 1));
                  vput(kalmanodo.kalman->Xpost, 2,
                       vget(kalmanodo.kalman->Xpre, 2));
               }
	     
	     }
          }
	  break;
	  	     
         }  /* end kalmanstate switch */ 
             
        
         if (kalmanodo.use) 
         {
               motcon.pose.x =
                   vget(kalmanodo.kalman->Xpost, 0) + kalmanodo.offset_x;
               motcon.pose.y =
                   vget(kalmanodo.kalman->Xpost, 1) + kalmanodo.offset_y;
               motcon.pose.th = vget(kalmanodo.kalman->Xpost, 2);
               
          } else {
               motcon.pose.x = odo.pose.x;
               motcon.pose.y = odo.pose.y;
               motcon.pose.th = odo.pose.th;
          }

               
               
    } else                 // If no estimation use normal odometry variables
    {
        motcon.pose.x = odo.pose.x;
        motcon.pose.y = odo.pose.y;
        motcon.pose.th = odo.pose.th;
    }



      if (we_config && we_status) {

        if (we_run) {
            wallest.odox = odo.pose.x;
            wallest.odoy = odo.pose.y;
            wallest.odoth = odo.pose.th;
            wallest.fwddist = odo.dist;
            wallest.measured_distancel = ir_dist[0];
            wallest.measured_distancer = ir_dist[4];
            wallest.new_distance_valuel = ir_samp[0].newval;
            wallest.new_distance_valuer = ir_samp[4].newval;
            wallest.target_speed = motcon.velcmd;
        }

        if (we_use) {
            motcon.wallangregr = wallest.angle;
            motcon.walldistregr = wallest.wall_distance;
            motcon.wallreliant = wallest.reliant;
        }
      }
      // Check up on the update of the drive function
      //**************************************************

      // Execute the current action function
      if (count > 20) {
        if (cur_smfunc(cur_smdata) == SM_FINISHED)
            runsmr = 0;
      }
      // Perform motioncontrol for the robot
      //**************************************************

      // perform motion control data update
      
      switch (robotinfo.type){
      case VELOMEGA:
        motcon.vel = odo.vel;
        motcon.curvell = odo.vell;
        motcon.curvelr = odo.velr;
        motcon.motoroutl = 0;
        motcon.motoroutr = 0;          
  
      break; 
      case ACKERMAN: {           
        motcon.vel = odo.vel;
        motcon.curvell = odo.vell;
        motcon.curvelr = odo.velr;
        if (strcmp(robotinfo.name, "hako") == 0) {
            motcon.motoroutl = 0;
            motcon.motoroutr = 0;           
        } else if (strcmp(robotinfo.name, "ackerbot") == 0)            // ackerbot
        {
            double d;
            int j;
            d = ir_dist[1];
            for (j = 2; j < 4; j++) {
              if (ir_dist[j] < d)
                  d = ir_dist[j];
            }
            defaultobstaclefront = d;
        }
 
      } 
      break;
      case DIFFERENTIAL:     // SMR and MMR    
      {
        
        motcon.vel = odo.vel;
        motcon.curvell = odo.vell;
        motcon.curvelr = odo.velr;
        if (strcmp(robotinfo.name, "smr") == 0) {
            double d;
            int j;
            d = ir_dist[1];
            for (j = 2; j < 4; j++) {
              if (ir_dist[j] < d)
                  d = ir_dist[j];
            }
            defaultobstaclefront = d;
        } 
      } 
      break;
      default:
      ;
       // fprintf(stderr, "Robot type %i not known\n", robotinfo.type);
    }    
      motcon.d_obstac_front=*motionconnector.obstaclefront; 
      motcon.d_obstac_back=*motionconnector.obstacleback;   
      // Update motion of the robot
      motion_update(&motcon);

      // Perform actuator update for the robot
      //**************************************************
      *xyzout1=motcon.xyzout1;
      switch (robotinfo.type){
      case VELOMEGA:
      
        *cmdtransvel->data=motcon.lvel * motorcontrol.velscalel;
        cmdtransvel->updated=1;
        *cmdrotvel->data=motcon.omega*motorcontrol.velscaler;
        cmdrotvel->updated=1;	 
      break;
      case ACKERMAN: {
        if (strcmp(robotinfo.name, "hako") == 0) {
            // Specify special HAKO variables from the motion control update
            *steeringangleref->data=motcon.steeringangle*1800/M_PI;
            steeringangleref->updated=1;
            if (motcon.lvel > 0) 
              *speedref->data= motcon.lvel * motorcontrol.velscalel+motorcontrol.velscaler;
            else if (motcon.lvel < 0)
              *speedref->data= motcon.lvel * motorcontrol.velscalel-motorcontrol.velscaler; 
            else   
              *speedref->data=0;
            speedref->updated=1;  
            {
            #define TOOLTHRESHOLD1  12
            int posref;
            if(hako.liftingGearPos >= hako.liftingGearStateRef + TOOLTHRESHOLD1)
              posref = 1; //Raise the tools
            else if(hako.liftingGearPos <= hako.liftingGearStateRef - TOOLTHRESHOLD1)
              posref = 2; //Lower the tools
            else
              posref = 0; //Stop
            
            liftinggearstref->data[0]=posref;
            liftinggearstref->updated=1;                    
            }
        } else if (strcmp(robotinfo.name, "ackerbot") == 0)            // ackerbot
        {
            sig = servo(motcon.steeringangle, odo.steeringangle);
            if ((count % 100 )==0)
              printf("%d  %lf  %lf   %lf \n",sig,motcon.steeringangle,
              odo.steeringangle,motcon.lvel);
            *steeringangleref->data=sig;
            steeringangleref->updated=1;
            *speedl->data = motcon.lvel * motorcontrol.velscalel;
            speedl->updated=1;             
        } else {
            if (curvatureref) {curvatureref->inputVar[0]=motcon.curvature;rhdwrite(curvatureref,0);}
            if (steeringangleref) {steeringangleref->inputVar[0]=motcon.steeringangle; rhdwrite(steeringangleref,0);}   
	    if (speedr){*speedr->data = motcon.rvel * motorcontrol.velscaler;
             speedr->updated=1;}
            if (speedl) {*speedl->data = motcon.lvel * motorcontrol.velscalel;
             speedl->updated=1;}
	    if (speedref){*speedref->data= (motcon.lvel *
	    motorcontrol.velscalel+motcon.rvel*motorcontrol.velscaler)/2; speedref->updated=1;} 
	     
        }
      }
      break;
      case DIFFERENTIAL:     // SMR and MMR    
      {
        *speedr->data = ROUND(motcon.rvel * motorcontrol.velscaler);
          speedr->updated=1;
        *speedl->data = ROUND(motcon.lvel * motorcontrol.velscalel);
          speedl->updated=1;
    
      }
      break; 
      case GRIPPER:
      break;
      
      case HEXACOPTER:
          if (thrustref) {thrustref->inputVar[0]=motcon.xyzout1;rhdwrite(thrustref,0);}	
	  if (pitchref) {pitchref->inputVar[0]=motcon.xyzout2;rhdwrite(pitchref,0);}
	  if (rollref) {rollref->inputVar[0]=motcon.xyzout3;rhdwrite(rollref,0); }
          if (yawrateref) {yawrateref->inputVar[0]=motcon.xyzout4;rhdwrite(rollref,0); }	  
      break;
      
      default:
        fprintf(stderr, "Robot type %i not known\n", robotinfo.type);
  
    }
    
    //routed output
      { int i=0;
      while (i < ROUTEOUTPUTTABLESIZE &&   routeoutputtable[i].name!="NULL"){
         if ( routeoutputtable[i].out !=NULL){
              routeoutputtable[i].out->inputVar[routeoutputtable[i].index] =*routeoutputtable[i].in;
	      rhdwrite(routeoutputtable[i].out,routeoutputtable[i].index);	
	 } 
         i++;
      }
      }
      
////////////
     
    // Perform supervisor function for the robot
    //**************************************************
    // Supervisor
    //  fprintf(stderr,"READ - Before Supervior\n");
    {
        static int blocked_o = 0;
        char buf[256];
        if (motcon.status != motionstatus_o) {
          if (xmlon){
            sprintf(buf, "<motioncontrol status=\"changed\" />\n");
          }
          else {
            sprintf(buf, "motioncontrol status changed\n");
          }
          motionstatus_o = motcon.status;
          putevent(buf);
        }
        if (blocked > blocked_o) {
          if (xmlon)
            sprintf(buf, "<motioncontrol status=\"blocked\"\n");
          else
            sprintf(buf, "motioncontrol blocked\n");

          putevent(buf);
        }
        blocked_o = blocked;
    }
    //end supervisor
    // check watches
      checkwatches();
      
    // Perform log of data
    //**************************************************
    {
         int i1;
         if (logptr < 100000 && logvar1.n > 0 && logging) {
            for (i1 = 0; i1 < logvar1.n; i1++) {
               logtable[i1][logptr] = *logvar1.ref[i1];
            }
            logptr++;
         }
    }
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)
        runsmr = 0;
    if (*mrccontrol.stop) runsmr=0;	
  }

  printf(" logging %d %d %d \n",logptr, logvar.n ,logging);
  printf("\n-----------------------------------------\n");
  printf("Ending processing of commands\n");

  printf("\nSimulation finished, no of odoerrors %d\n", odo.enc_error);
  printf("asynccount = %d \n\n", asynccount);

  switch (robotinfo.type){
    case VELOMEGA:
            *cmdtransvel->data=0;
              cmdtransvel->updated=1;
              *cmdrotvel->data=0;
              cmdrotvel->updated=1;	 

    
    break;
    case ACKERMAN:  
    if (strcmp(robotinfo.name, "hako") == 0) {
        *speedref->data=0;
          speedref->updated=1;
    } 
    break;
    case DIFFERENTIAL:
    {
              *speedr->data = 0;
              speedr->updated=1;
              *speedl->data = 0;
              speedl->updated=1;
            
    }
  }
    rhdSync();
    savelog("log");
  if (strlen(gps.hostname) != 0 && gps.port != 0
      && gps.client.sockfd != -1) {
    gps_socket_disconnect(&gps);
    printf("Disconnected from the UTMgpsd server, sleeping 1 sec!\n");
    sleep(1);
    printf("Stopped sleeping!\n\n");
  }

  if (systempipe != NULL)
    pclose(systempipe);



  close(serv.s);
  close(serv.ls);


  if (kalmanodo.config) {
    printf("\nKalman error covariance\n");
    mprint(kalmanodo.kalman->Ppost);
    printf("\n");
  }

  IAU_kalman_odometry_free(&kalmanodo);

  return (0);
}

#include "stateexamples.c"
