//#define CV_INLINE static __inline__
#include "motioncontrol.h"
#include "linesensor.h"
#include "statemachine.h"
#include "wallest.h"
#include "odometry.h"
#include "filter.h"
#include "libhako.h"
#include "rhd.h"


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include "componentserver.h"
#include "server1.h"
#include "administration.h"
char sbuf[512];
char ansbuf[2048];
void userprint(char *);
void smrsystemif(char *);
void rs485send(unsigned char *);
void set_motor_param(int,int,int);
int putevent(char * event);
double smrgettime(void);

symTableElement * getoutputref (const char *sym_name, symTableElement * tab);
int  rhdwrite(symTableElement * var,int index);
extern symTableElement *outputtable;  
extern motiontype motcon;
extern odotype odo;
extern IAU_kalman_odotype kalmanodo;
extern wallesttype wallest;
extern componentservertype lmssrv;
extern servertype serv;
extern hakotype hako;
extern int (*defaultcond)(void);
extern int linetype;
extern sm_type *intp_smdataptr;
extern int  (*intp_smfuncptr)(sm_type * sm);
extern covtype odocov;
//extern UCameraGmk gcm;
extern struct {
	posetype data[8];
	double lsold[8];
	int Ndetect,linedetect;
	}lineest;
extern struct {
  double x,y,z,omega,phi,kappa,code,id,crc;
  }gmk;	
extern componentservertype lmssrv,camsrv;
extern int inputready;
extern int rdl,rdr;
extern double gyro1off,gyro1gain;

extern double rolloffset, pitchoffset;  // Added by JE

extern int usekalmanodo,kalmanon,xmlon;

extern int rollpitchon, paintServoStatus;  // Added by JE

extern int logging;
extern struct {
   double velscalel, velscaler;
   double kp, ki;
} motorcontrol;

#define ROUTEOUTPUTTABLESIZE 2
extern struct { char * name; double *in;symTableElement * out;int index;} routeoutputtable[ROUTEOUTPUTTABLESIZE];
	

int sockfd,wificonnected;
struct	sockaddr_in serv_adr;
int conditioncounter[10];
sm_type intp_smdata;
extern symTableElement *enginespeedref,*hakomanual;


void savelog(char *fname);
void resetlog(void);
void removelogvars(void);
void set_digiout(int out);

void setvel(double v){
  motcon.velcmd=v;
}

void setacc(double v){
  motcon.acccmd=v;
}

int isarray(char * res, char * inp){
int i,notarray,index;
i=0;
notarray=1;
index=-1;
while (inp[i]!=0 && notarray){
  if (inp[i]=='[' ){
    notarray=0;
    index=atoi(&inp[i+1]);
  }  
  else {
    res[i]=inp[i];
    i++;
  }
}  
res[i]=0;
return index;
}
 

int nodefault(){
  return 0;
}

int defaulttrue(){
  return(1);
}

double intp_ntrue(double counterno,double condition){
  if (counterno < 0 || counterno > 9)
    return -1.0;
  if  (condition )
    conditioncounter[(int)counterno]++;
  return conditioncounter[(int)counterno];
}  

int intp_fwdcond(void){
  return((sign(motcon.dist)*motcon.target_dist)<0.005);
}

int intp_fwd(void *par){
  motcon.cmd=MC_FWDCMD;
  motcon.dist=*varptrstack[0].pt;
  defaultcond=intp_fwdcond;
  return(0);
}

int intp_funcgencond(void){
  return(motcon.runlength>0.5);
}

int intp_funcgen(void *par){
  int i;
  motcon.cmd=MC_FUNCGEN;
  for (i=0;i<npar;i++)
    motcon.funcgenp[i]=*varptrstack[i].pt;
  defaultcond=intp_funcgencond;
  return(0);
}
  
int intp_turncond(void){
   return(fabs(motcon.poseerr.th)<0.01);
}  
   
int intp_turn(void *par){
  motcon.cmd=MC_TURNCMD;
  if (npar == 2)
    motcon.turnangle=*varptrstack[0].pt;
  else
    motcon.turnangle=*varptrstack[0].pt*M_PI/180.;
  defaultcond=intp_turncond;
  return(0);
} 

int intp_turnrcond(void){
   return(fabs(motcon.target_dist)<0.005);
}  
   
int intp_turnr(void *par){
  motcon.cmd=MC_TURNRCMD;
  if (npar == 3)
    motcon.turnangle=*varptrstack[1].pt;
  else
    motcon.turnangle=*varptrstack[1].pt*M_PI/180.;
  motcon.R=*varptrstack[0].pt;
  defaultcond=intp_turnrcond;
  return(0);
} 


int intp_turnccond(void){
   return(fabs(motcon.target_dist)<0.005);
}  
   
int intp_turnc(void *par){
  motcon.cmd=MC_TURNCCMD;
  if (npar == 3)
    motcon.turnangle=*varptrstack[1].pt;
  else
    motcon.turnangle=*varptrstack[1].pt*M_PI/180.;
  motcon.R=*varptrstack[0].pt;
  defaultcond=intp_turnccond;
  return(0);
} 

int intp_drivetopointcond(void){
   return(motcon.dtp.state==DTP_FINISH);
}  
   
int intp_drivetopoint(void *par){
  motcon.cmd=MC_DRIVETOPOINTCMD; 
  motcon.dtp.xb=*varptrstack[0].pt;
  motcon.dtp.yb=*varptrstack[1].pt;
  motcon.dtp.thetab=*varptrstack[2].pt*M_PI/180;
  motcon.dtp.r=*varptrstack[3].pt;    
  motcon.dtp.coordtype=DTP_COORD_REL;
  if (npar==5 && strcmp((char *)varptrstack[4].pt,"abs")==0)
    motcon.dtp.coordtype=DTP_COORD_ABS;
  defaultcond=intp_drivetopointcond;
  return(0);
} 

int intp_movetoxyz(void *par){
  motcon.cmd=MC_MOVETOXYZCMD; 
  motcon.refpose3d.x=*varptrstack[0].pt;
  motcon.refpose3d.y=*varptrstack[1].pt;
  motcon.refpose3d.z=*varptrstack[2].pt;
  defaultcond=nodefault;
  return(0);
} 


int intp_waitcond(void){
   return(*pcmdtime >= condpar0);
}  
   
int intp_wait(void *par){
  symrec *p;
  p=getsym ("cmdtime", sysvar);
  pcmdtime=&(p->value.pvar[0]);
  condpar0=*varptrstack[0].pt;
  defaultcond=intp_waitcond;
  return(0);
} 

int intp_array(void *par){
  symrec * p;
  p=putsym ((char *)varptrstack[0].pt, VAR,&symbols);
  p->varsize=*varptrstack[1].pt;
  p->value.pvar=&((double*)varpooltable.tab)[varpooltable.N];
  varpooltable.N+=p->varsize;
  defaultcond=defaulttrue;
  return(0);
} 

#if(0)   
int intp_log(void *par){
  int i;
  symrec *s;
  for (i=0; i < npar;i++){ 
    if (logvar.n < 20){
      if (((char *)varptrstack[i].pt)[0]=='$' &&(s=getsym((char
      *)varptrstack[i].pt+1, sysvar))!=NULL){
        logvar.ref[logvar.n]=s;
        logvar.n++;
      }
      if ((s=getsym((char *)varptrstack[i].pt, symbols))!=NULL){
        logvar.ref[logvar.n]=s;
        logvar.n++;
      }
      if (logvar.n > 0){
        printf(" %s \n",logvar.ref[logvar.n-1]->name);
      }
    }
    else 
      printf("logtable full\n");  
  }
  defaultcond=defaulttrue;
  return(0);
} 
#endif

int intp_log(void *par){
  int i,index;
  char res[256];
  symrec *s;
  for (i=0; i < npar;i++){ 
    if (logvar1.n < logvar1.nmax){
      if (((char *)varptrstack[i].pt)[0]=='$'){
         index=isarray(res,(char *)varptrstack[i].pt+1);
	 if (index==-1) index=0;
	 if((s=getsym(res, sysvar))!=NULL){
            logvar1.ref[logvar1.n]=s->value.pvar+index;
            logvar1.n++;
	  }  
      }
      else {
         index=isarray(res,(char *)varptrstack[i].pt);
	 if (index==-1) index=0;
	 if((s=getsym(res, symbols))!=NULL){
            logvar1.ref[logvar1.n]=s->value.pvar+index;
            logvar1.n++;
	  }        
      }
      
    }
    else 
      printf("logtable full\n");  
  }
  defaultcond=defaulttrue;
  return(0);
} 


   
int intp_stream(void *par){
  int i,n1;
  symrec *s;
  if (xmlon && *varptrstack[0].pt==0){
     double time;
     char *ansbufptr;
     ansbufptr=ansbuf;
     time=smrgettime();
     n1=sprintf(ansbufptr,"<data time=\"%lf\" ",time);
     ansbufptr+=n1;
     for (i=1; i < npar;i++){ 
       if ((s=getsym((char *)varptrstack[i].pt+1, sysvar))!=NULL){
         n1=sprintf(ansbufptr,"%s=\"%lf\" ",s->name,s->value.pvar[0]);
         ansbufptr+=n1;      
       }
       if ((s=getsym((char *)varptrstack[i].pt, symbols))!=NULL){
        n1=sprintf(ansbufptr,"%s=\"%lf\" ",s->name,s->value.pvar[0]);
        ansbufptr+=n1;    
       } 
     }
     n1=sprintf(ansbufptr," />\n");
     ansbufptr+=n1;
     send(serv.s,ansbuf,(int)(ansbufptr-ansbuf),0);
  }
  else {
  streamvar.sampletime=*varptrstack[0].pt;
  for (i=1; i < npar;i++){ 
    if ((s=getsym((char *)varptrstack[i].pt+1, sysvar))!=NULL){
      streamvar.ref[streamvar.n]=s;
      streamvar.n++;
    }
    if ((s=getsym((char *)varptrstack[i].pt, symbols))!=NULL){
      streamvar.ref[streamvar.n]=s;
      streamvar.n++;
    }

    printf("streamvar %s %d \n",streamvar.ref[streamvar.n-1]->name,streamvar.n);
  }
  }
  defaultcond=defaulttrue;
  return(0);
} 




int drive_code(void){
  motcon.dist=0;
  if (npar >= 3){
    motcon.refpose.x=*varptrstack[0].pt;
    motcon.refpose.y=*varptrstack[1].pt;
    motcon.refpose.th=*varptrstack[2].pt*M_PI/180;
    if (npar >=4){
       motcon.refpose.th=*varptrstack[2].pt;
    }
  }
  else {
    if (usekalmanodo){
      motcon.refpose.x=vget(kalmanodo.kalman->Xpost,0)+ kalmanodo.offset_x;
      motcon.refpose.y=vget(kalmanodo.kalman->Xpost,1)+kalmanodo.offset_y;
      motcon.refpose.th=vget(kalmanodo.kalman->Xpost,2); 
    }
    else {
      motcon.refpose.x=odo.pose.x;
      motcon.refpose.y=odo.pose.y;
      motcon.refpose.th=odo.pose.th;
    }
  }
  defaultcond=nodefault;
  return(0);
} 

int intp_drive(void *par){
  motcon.cmd=MC_DRIVECMD;
  drive_code();
  return 0;
}

int intp_driveon(void *par){
  motcon.cmd=MC_DRIVEONCMD;
  drive_code();
  return 0;
}
int intp_drivew(void *par){
  motcon.cmd=MC_DRIVECMDW;
  drive_code();
  return 0;
}

int intp_driveonw(void *par){
  motcon.cmd=MC_DRIVEONCMDW;
  drive_code();
  return 0;
}



int intp_motorcmds(void *par){
  motcon.cmd=MC_DIRECTMOTORCMDS;
  motcon.velcmdl=*varptrstack[0].pt;
  motcon.velcmdr=*varptrstack[1].pt;
  defaultcond=defaulttrue;
  return(0);
} 

int intp_trans(void *par){
  double x0,y0,th0,x,y,th,c,s;
  x0=*varptrstack[0].pt;
  y0=*varptrstack[1].pt;
  th0=*varptrstack[2].pt;
  x=*varptrstack[3].pt;
  y=*varptrstack[4].pt;
  th=*varptrstack[5].pt;
  c=cos(th0);
  s=sin(th0);
  *funcres[0]=c*x-s*y+x0;
  *funcres[1]=s*x+c*y+y0;
  *funcres[2]=th+th0; 
  defaultcond=defaulttrue;
  return(0);
} 

int intp_invtrans(void *par){
  double x0,y0,th0,x,y,th,c,s;
  x0=*varptrstack[0].pt;
  y0=*varptrstack[1].pt;
  th0=*varptrstack[2].pt;
  x=*varptrstack[3].pt;
  y=*varptrstack[4].pt;
  th=*varptrstack[5].pt;
  c=cos(th0);
  s=sin(th0);
  *funcres[0]=c*(x-x0)+s*(y-y0);
  *funcres[1]=-s*(x-x0)+c*(y-y0);
  *funcres[2]=th-th0; 
  defaultcond=defaulttrue;
  return(0);
} 

int intp_targethere(void *par){
  motcon.tgt=odo.pose;
//  printf("tgt %lf,%lf %lf \n",motcon.tgt.x,motcon.tgt.y,motcon.tgt.th);
  defaultcond=defaulttrue;
  return(1);
} 


int intp_resetodocov(void *par){
   resetupdatestruct(&odocov);
  defaultcond=defaulttrue;
  return(1);
} 


int intp_stop(void *par){
  motcon.cmd=MC_STOPCMD;
  defaultcond=defaulttrue;
  return(0);
} 

int intp_idle(void *par){
  motcon.cmd=MC_IDLECMD;
  defaultcond=defaulttrue;
  return(0);
}
 
int intp_align(void *par){
  motcon.cmd=MC_ALIGNCMD;
  defaultcond=nodefault;
  return(0);
} 

int intp_resetmotors(void *par){
  intp_smdataptr=&intp_smdata;
  intp_smfuncptr=sm_resetmotors;
  defaultcond=defaulttrue;
  return(0);
} 

int intp_barcode(void *par){
  intp_smdataptr=&intp_smdata;
  intp_smdataptr->p[0]=*varptrstack[0].pt;
  // printf(" p0intp  %f\n",*varptrstack[0].pt);
  intp_smfuncptr=sm_barcode;
  defaultcond=defaulttrue;
  return(0);
} 


int intp_followline(void *par){
 // printf("followline %s \n" ,(char*) varptrstack[0].pt);
  if ( strcmp((char *)varptrstack[0].pt,"bm")==0) linetype=LINE_MIDDLE_B;
  else if ( strcmp((char *)varptrstack[0].pt,"br")==0) linetype=LINE_RIGHT_B;
  else if ( strcmp((char *)varptrstack[0].pt,"bl")==0) linetype=LINE_LEFT_B;
  else if ( strcmp((char *)varptrstack[0].pt,"wm")==0) linetype=LINE_MIDDLE_W;
  else if ( strcmp((char *)varptrstack[0].pt,"wr")==0) linetype=LINE_RIGHT_W;
  else if( strcmp((char*)varptrstack[0].pt,"wl")==0) linetype=LINE_LEFT_W;
  else if ( strcmp((char *)varptrstack[0].pt,"bmc")==0) linetype=LINE_MIDDLE_B_CAM;
  else linetype=LINE_MIDDLE_B;
 //printf("followline %s %d\n" ,(char*) varptrstack[0].pt,linetype);

  motcon.cmd=MC_FOLLOW_LINECMD;
  defaultcond=nodefault;
  return(0);
} 

int intp_followwall(void *par){
  if ( ((char*)varptrstack[0].pt)[0]=='r') 
    motcon.side=MC_RIGHT;
  else if ( ((char*)varptrstack[0].pt)[0]=='l')
    motcon.side=MC_LEFT;
  else  printf("wrong parameter\n");
  
  motcon.walldistref=*varptrstack[1].pt;

  motcon.cmd=MC_FOLLOW_WALLCMD;
  defaultcond=nodefault;
  return(0);
}

int intp_followwall2(void *par){
  if ( ((char*)varptrstack[0].pt)[0]=='r') 
    motcon.side=MC_RIGHT;
  else if ( ((char*)varptrstack[0].pt)[0]=='l')
    motcon.side=MC_LEFT;
  else  printf("wrong parameter\n");
  if ( ((char*)varptrstack[0].pt)[1]=='d') 
    motcon.method=MC_DIRECT;
  else if ( ((char*)varptrstack[0].pt)[1]=='r')
    motcon.method=MC_REGRESSION;
  else printf("Wrong parameter \n");


  motcon.walldistref=*varptrstack[1].pt;

  motcon.cmd=MC_FOLLOW_WALLCMD2;
  defaultcond=nodefault;
  return(0);
}

int intp_goto(void *par){
  char *line;
  line=findlabel((char*)varptrstack[0].pt,currentplan);
  defaultcond=nodefault;
  if (line){
    currentplan->pline=line;
    return(1);
  }
  else
    return(2);
}

int intp_call(void *par){
  char *line;
  line=findlabel((char*)varptrstack[0].pt,currentplan);
  defaultcond=nodefault;

  if (line){
    pushprogramline(currentplan->pline);
    currentplan->pline=line;
    return(1);
  }
  else
    return(2);
}


int intp_return(void *par){
  
  defaultcond=nodefault;
  if (programlinestack.p>0){
    currentplan->pline=popprogramline();
    return(1);
  }
  else
    return(2);
}



int intp_label(void *par){
  defaultcond=nodefault;
  return(1);
}

int iffunc(double exp,char *label){
char *line; 
//  printf(" exp %lf label %s \n",exp,label);
  if ( exp){ 
    line=findlabel(label,currentplan);
    if (line){
      currentplan->pline=line;
      return(1);
    }
    else
      return(2);
  }
  else
    return 1;  
}


struct init_cmd {
       char *fname;
       int (*fnct)(void *);
       char *parstring;
};

int intp_vision(void *par){
  //printf("image %s \n" ,(char*) varptrstack[0].pt);
    if (camsrv.connected){
    char buffer[256];
    int len; 
    if (((char*) varptrstack[0].pt)[0]=='$'){
      len=strlen(sbuf);
      sbuf[len]='\n';
      len++;
      send( camsrv.sockfd, sbuf, len, 0);
    }
    else {
      len = sprintf(buffer, "%s", (char*) varptrstack[0].pt);
      buffer[len]='\n';
      len++;
      send( camsrv.sockfd, buffer, len, 0);
    }    
  }
  else {
    printf("Not connected to cameraserver \n");
  }
  defaultcond=defaulttrue;
  return(0);
}

int intp_laser(void *par){
  //printf("image %s \n" ,(char*) varptrstack[0].pt);
    if (lmssrv.connected){
    char buffer[256];
    int len; 
    if (((char*) varptrstack[0].pt)[0]=='$'){
      len=strlen(sbuf);
      sbuf[len]='\n';
      len++;
      send( lmssrv.sockfd, sbuf, len, 0);
    }
    else {
      len = sprintf(buffer, "%s", (char*) varptrstack[0].pt);
      buffer[len]='\n';
      len++;
      send( lmssrv.sockfd, buffer, len, 0);
    }    
  }
  else {
    printf("No connection to laserserver \n");
  }
  defaultcond=defaulttrue;
  return(0);
}

 


int intp_get(void *par){
 // printf("get %s \n" ,(char*) varptrstack[0].pt);
  if ( strcmp((char *)varptrstack[0].pt,"guidemark")==0){ 
     gmk.id=0;
    if (camsrv.connected){
    char buffer[256];
    int len; 
    sprintf(buffer, "gmkget ");
    if (npar==2) strcat(buffer,(char *)varptrstack[1].pt);
    strcat(buffer," \n");
    len=strlen(buffer);
    send( camsrv.sockfd, buffer, len, 0);
    }    
  }
  defaultcond=defaulttrue;
  return(0);
}


int intp_routeinput(void *par){
  parameterelem * parptr;
  char res[256];
  int index;
  symrec *s;
  parptr=findpar((char *)varptrstack[0].pt,(char *)varptrstack[1].pt,(char*)varptrstack[2].pt);
  if (parptr){
    index=isarray(res,(char *)varptrstack[3].pt);
    if (index==-1) index=0;
    if((s=getsym(res, sysvar))!=NULL){
        *( (double **)  parptr->parptr)=s->value.pvar+index;
    }
    else
      printf(" input variable not found \n");
  }
  else
    printf("parameter not found \n");
  defaultcond=defaulttrue;
  return(0);
}

int intp_routeoutput(void *par){
  int index;
  char res[256];
   symTableElement *s; 
  index=isarray(res,(char *)varptrstack[1].pt);
  if (index==-1) index=0;
  if((s=getoutputref(res, outputtable))!=NULL){
     int i=0;
     int found=0;
     while (i< ROUTEOUTPUTTABLESIZE && !found && (routeoutputtable[i].name!=NULL) ){
	   if ( strcmp((char *)varptrstack[0].pt,routeoutputtable[i].name)==0)
             found=1;
           else
	     i++;
      }
      if (found){
         routeoutputtable[i].out=s;
	 routeoutputtable[i].index=index;
      }
      else
        printf("input not found\n");
   }
   else
     printf("output not found\n");     
  defaultcond=defaulttrue;
  return(0);
}



int intp_setpar(void *par){
  parameterelem * parptr;
  parptr=findpar((char *)varptrstack[0].pt,(char *)varptrstack[1].pt,(char*)varptrstack[2].pt);
  if (parptr)
    *(double *)parptr->parptr=*varptrstack[3].pt;
  else
    printf("parameter not found \n");
  defaultcond=defaulttrue;
  return(0);
}  

int intp_setoffset(void *par){
  int index;
  char res[256];
  symrec *s; 
 index=isarray(res,(char *)varptrstack[0].pt);
  if (index==-1) index=0;
  if((s=getsym(res, sysvar))!=NULL){
     *(s->value.pvar+index+s->varsize)=*varptrstack[1].pt;
     printf(" offset assigned %lf\n",*varptrstack[1].pt);
  }        
  defaultcond=defaulttrue;
  return(0);
}

int intp_setgain(void *par){
  int index;
  char res[256];
  symrec *s; 
  index=isarray(res,(char *)varptrstack[0].pt);
  if (index==-1) index=0;
  if((s=getsym(res, sysvar))!=NULL){
     *(s->value.pvar+index+2*s->varsize)=*varptrstack[1].pt;
     printf(" gain assigned %lf\n",*varptrstack[1].pt);
  }        
  defaultcond=defaulttrue;
  return(0);
}

int intp_setoutput(void *par){
  int index;
  char res[256];
   symTableElement *s; 
  index=isarray(res,(char *)varptrstack[0].pt);
  if (index==-1) index=0;
  if((s=getoutputref(res, outputtable))!=NULL){
     s->inputVar[index]=*varptrstack[1].pt;
     rhdwrite(s,index);
  }        
  defaultcond=defaulttrue;
  return(0);
}




int intp_set(void *par){
  int index;
  char res[256];
  symrec *s; 
 
  if ( strcmp((char *)varptrstack[0].pt,"linedetect")==0)
  lineest.Ndetect=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"conditioncounter")==0)
  conditioncounter[(int)(*varptrstack[1].pt)]=0;
  else if ( strcmp((char *)varptrstack[0].pt,"odocontrol")==0)
  odo.control=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"gyrotype")==0) odo.gyrotype=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"xmlon")==0) xmlon=*varptrstack[1].pt;

  else if( strcmp((char *)varptrstack[0].pt,"rollpitchon")==0) rollpitchon=*varptrstack[1].pt;  // Added by JE
  else if ( strcmp((char *)varptrstack[0].pt,"rolloffset")==0) rolloffset=*varptrstack[1].pt;  // Added by JE
  else if ( strcmp((char *)varptrstack[0].pt,"pitchoffset")==0) pitchoffset=*varptrstack[1].pt;  // Added by JE
  else if ( strcmp((char *)varptrstack[0].pt,"paintServoStatus")==0) paintServoStatus=*varptrstack[1].pt;  // Added by JE

  else if ( strcmp((char *)varptrstack[0].pt,"$odox")==0) odo.pose.x=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"$odoy")==0) odo.pose.y=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"$odoth")==0) odo.pose.th=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"$gyro1off")==0) gyro1off=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"$gyro1gain")==0) gyro1gain=*varptrstack[1].pt; 
  else if ( strcmp((char *)varptrstack[0].pt,"$odocovE2")==0) odocov.E2=*varptrstack[1].pt; 
  else if ( strcmp((char *)varptrstack[0].pt,"$odocovA2")==0) odocov.A2=*varptrstack[1].pt; 
  else if ( strcmp((char *)varptrstack[0].pt,"odowidth")==0) odo.w=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"usekalmanodo")==0)  {
    usekalmanodo=*varptrstack[1].pt;
    motcon.tgt.x=vget(kalmanodo.kalman->Xpost,0)+ kalmanodo.offset_x;
    motcon.tgt.y=vget(kalmanodo.kalman->Xpost,1)+kalmanodo.offset_y;
    motcon.tgt.th=vget(kalmanodo.kalman->Xpost,2);
  }
  else if ( strcmp((char *)varptrstack[0].pt,"kalmanon")==0)  {
    kalmanon=*varptrstack[1].pt;  
  }
  else if ( strcmp((char *)varptrstack[0].pt,"hakomanual")==0)  {
      if (hakomanual){
        hakomanual->data[0]=(*varptrstack[1].pt);
        hakomanual->updated=1;
      }   
      else
        hako.navigationModeRef=(*varptrstack[1].pt);
  }

  else if ( strcmp((char *)varptrstack[0].pt,"hakoenginespeed")==0){ hako.engineSpeedRef=*varptrstack[1].pt;
       if (enginespeedref){enginespeedref->data[0]=*varptrstack[1].pt; /* removed this factor /20.; number now in RPM */
                         enginespeedref->updated=1;
     }
  }
  else if ( strcmp((char *)varptrstack[0].pt,"hakoliftinggearstateref")==0)
  hako.liftingGearStateRef=*varptrstack[1].pt;
  else if ( strcmp((char *)varptrstack[0].pt,"hakopowertakeoffstateref")==0)
  hako.powerTakeoffStateRef=*varptrstack[1].pt; 
  else if ( strcmp((char *)varptrstack[0].pt,"odoratio")==0)
  {
      double k,c0;
       k=*varptrstack[1].pt;
       c0=(odo.cr+odo.cl)/2;
       odo.cr= (2*k/(1+k))*c0;
       odo.cl= (2/(1+k))*c0;
  }
  else if ( strcmp((char *)varptrstack[0].pt,"odoc0")==0){
      double k,c0;
       c0=*varptrstack[1].pt;
       k=(odo.cr/odo.cl);
       odo.cr= (2*k/(1+k))*c0;
       odo.cl= (2/(1+k))*c0;
  }
  else{
  index=isarray(res,(char *)varptrstack[0].pt);
  if (index==-1) index=0;
  if((s=getsym(res, symbols))!=NULL){
     *(s->value.pvar+index)=*varptrstack[1].pt;
  } 
  else if(res[0]=='$' && ((s=getsym(&res[1], sysvar))!=NULL)){
     *(s->value.pvar+index)=*varptrstack[1].pt;
  }        
  }
  
  defaultcond=defaulttrue;
  return(0);
}


int intp_control(void *par){
  if ( strcmp((char *)varptrstack[0].pt,"stoplog")==0) logging=0;
  if ( strcmp((char *)varptrstack[0].pt,"startlog")==0) logging=1;
  if ( strcmp((char *)varptrstack[0].pt,"resetlog")==0) resetlog();
  if ( strcmp((char *)varptrstack[0].pt,"removelogvars")==0) removelogvars();
  if ( strcmp((char *)varptrstack[0].pt,"savelog")==0){
    if (npar > 1) 
     savelog((char *)varptrstack[1].pt);
    else
     savelog("log");
  }    
  defaultcond=defaulttrue;
  return(0);
}

/// fjernet af christian - gammel smrd kode
#if (0)
int intp_rs485send(void *par){
   unsigned char msg[32],h;  
   if (npar >1){
     msg[0]=npar-1;
     msg[1]= *varptrstack[0].pt;
     h=*varptrstack[1].pt;
     msg[1]|=(h<<4);
     for (h=2;h<npar;h++)
        msg[h]=*varptrstack[h].pt;
     rs485send(msg);
   }
   defaultcond=defaulttrue;
   return(0);
}
#endif

int intp_stringcat(void *par){
  char buf[256];
  char tmpbuf[256];
  int i;
   if (varptrstack[0].typ==STRINGPAR){
      if (strcmp((char*)varptrstack[0].pt,"$str")!=0)
        strncpy(tmpbuf,(char*)varptrstack[0].pt,255);
      else
        strncpy(tmpbuf,sbuf,255);
   }
   else {
     sprintf(buf,"%lf ",*varptrstack[0].pt);
     strncpy(tmpbuf,buf,255);
   }
   for (i=1;i<npar;i++){
     if (varptrstack[i].typ==STRINGPAR) {
       if (strcmp((char*)varptrstack[i].pt,"$str")!=0)
         strncat(tmpbuf,(char*)varptrstack[i].pt,255);
       else
         strncat(tmpbuf,sbuf,255);      
     }	 
     else {
       sprintf(buf,"%lf ",*varptrstack[i].pt);
       strncat(tmpbuf,buf,255);
     }
   }
   strncpy(sbuf,tmpbuf,255);
   printf("strbuf %s \n",sbuf);
   defaultcond=defaulttrue;
   return(0);
}

int intp_tointstring(void *par){
  sprintf(sbuf,"%d ",(int) *varptrstack[0].pt);
  printf("strbuf %s \n",sbuf);
  defaultcond=defaulttrue;
  
  return(0);
}

 
int intp_open(void *par){
  printf("open %s \n" ,(char*) varptrstack[0].pt);
  if ( strcmp((char *)varptrstack[0].pt,"wifi")==0){ 
  	sockfd = socket(AF_INET, SOCK_STREAM, 0);
        printf(" sock %d \n",sockfd);
        if ( sockfd < 0 )
    {
      	perror(strerror(errno));
      	fprintf(stderr," Can not make wifi socket\n");
      	exit(errno);
    }
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_port	= htons(7788);
    serv_adr.sin_addr.s_addr = inet_addr("192.38.66.139");
    if ((wificonnected=connect(sockfd, (struct sockaddr *) &serv_adr, sizeof(serv_adr))) <0){
          printf("connect error\n");
    }
  }
  defaultcond=defaulttrue;
  return(0);
}
    
int intp_writewifi(void *par){
  char buf[1000];
  int len;
  len=sprintf(buf,"click  %s %6.2f %6.2f ",(char *)varptrstack[0].pt,*varptrstack[1].pt,*varptrstack[2].pt);
  if (wificonnected > -1)
      send( sockfd, buf, len, 0);    
  defaultcond=defaulttrue;
  return(0);
}  

int intp_play(void *par){
  char buf[1000];
  strcpy(buf,"play ");
  strcat(buf,(char*)varptrstack[0].pt);
  smrsystemif(buf);
  defaultcond=defaulttrue;
  return(0);
}
 
int intp_event(void *par){
  char buf[2048];
  if (xmlon){
  strcpy(buf,"<userevent name=\"");
  strcat(buf,(char*)varptrstack[0].pt);
  strcat(buf,"\" />\n");
  }
  else{
   strcpy(buf,"userevent ");
   strcat(buf,(char*)varptrstack[0].pt);
   strcat(buf,"\n");
  }
  putevent(buf);
  defaultcond=defaulttrue;
  return(1);
} 

int intp_sync(void *par){
  char buf[256];
  strcpy(buf,"syncevent ");
  strcat(buf,(char*)varptrstack[0].pt);
  strcat(buf,"\n");
  send(serv.s,buf,strlen(buf),0);
  defaultcond=defaulttrue;
  return(1);
} 
int intp_speak(void *par){
  char buf[1000];
  strcpy(buf,"/usr/local/bin/flite -t \"");
  if (((char*) varptrstack[0].pt)[0]=='$'){
     strcat(buf,sbuf);
  }else{
    strcat(buf,(char*)varptrstack[0].pt);
  }
  strcat(buf,"\"");
  smrsystemif(buf);
  printf("%s",buf);
  defaultcond=defaulttrue;
  return(0);
} 

int intp_print(void *par){
  userprint((char*)varptrstack[0].pt);
  defaultcond=defaulttrue;
  return(0);
} 

int intp_addwatch(void *par){
  if(watchtab.p <watchtab.max){
     watch=1;
     strncpy(watchtab.tab[watchtab.p].name,(char*)varptrstack[0].pt,32);
     watchtab.tab[watchtab.p].type=*varptrstack[1].pt;
     strncpy(watchtab.tab[watchtab.p].exp,condstart,256);
     watchtab.tab[watchtab.p].val=0;
     watchtab.p++;  
  }
  defaultcond=defaulttrue;
  return(0);
} 


int intp_readcond(void){
   if (inputready >0){
      inputready=0;
      return(1);
   }
   else
     return(0);
}  

int intp_read(void *par){
  userprint((char*)varptrstack[0].pt);
  defaultcond=intp_readcond;
  return(0);
}
     
int intp_wallest(void *par){

  if ( ((char*)varptrstack[0].pt)[0]=='r')
    wallest.side=WE_RIGHT;
  else if ( ((char*)varptrstack[0].pt)[0]=='l')
    wallest.side=WE_LEFT;
  else  printf("wrong parameter\n");
  if ( ((char*)varptrstack[0].pt)[1]=='b')
    wallest.status=WE_ON;
  else if ( ((char*)varptrstack[0].pt)[1]=='c')
    wallest.status=WE_OFF;
  else printf("Wrong parameter \n");

  wallest.line_distance = *varptrstack[1].pt;
	
  if(npar >= 3) wallest.input_min_wall_after_hole = *varptrstack[2].pt;
	else wallest.input_min_wall_after_hole = 0.0;
	
  if(npar >= 3) wallest.input_max_hole_dist = *varptrstack[3].pt;
	else wallest.input_max_hole_dist = 0.0;

  defaultcond=defaulttrue;
  return(0);

}

int intp_ignoreobstacles(void *par){
  motcon.ignore_obstacles=1;
  defaultcond=defaulttrue;
  return(0);
}



struct init_cmd cmd_fncts[] =
     {  {"fwd", intp_fwd,"11d"},
        {"turn",intp_turn,"12ds"},
	{"drive",intp_drive,"04dddd"},
	{"driveon",intp_driveon,"04dddd"},
	{"drivew",intp_drivew,"34dddd"},
	{"driveonw",intp_driveonw,"34dddd"},
	{"motorcmds",intp_motorcmds,"22dd"},
	{"turnr",intp_turnr,"23dds"},
	{"turnc",intp_turnc,"23dds"},
	{"followline",intp_followline,"11s"},
        {"followwall",intp_followwall,"22sd"},
	{"followwall2",intp_followwall2,"22sd"},
	{"label",intp_label,"11s"},
	{"wait",intp_wait,"11d"},
        {"goto",intp_goto,"11s"},
	{"call",intp_call,"11s"},
	{"return",intp_return,"00"},	
	{"log",intp_log,"19s"},
	{"stream",intp_stream,"19ds"},
        {"stop",intp_stop,"00"},
	{"idle",intp_idle,"00"},
        {"resetmotors",intp_resetmotors,"00"},
        {"barcode",intp_barcode,"00"},
        {"get",intp_get,"12s"},
        {"set",intp_set,"22sd"},
	{"setoutput",intp_setoutput,"22sd"},
	{"setoffset",intp_setoffset,"22sd"},
	{"setgain",intp_setgain,"22sd"},
        {"setpar",intp_setpar,"44sssd"},
	{"routeinput",intp_routeinput,"44ssss"},
	{"routeoutput",intp_routeoutput,"22ss"},
	{"control",intp_control,"13sss"},
        {"open",intp_open,"11s"},
        {"writewifi",intp_writewifi,"33sdd"},
        {"play",intp_play,"11s"},  
        {"speak",intp_speak,"11s"},
        {"print",intp_print,"11s"},
        {"read",intp_read,"11s"},
        {"wallest",intp_wallest,"24sddd"},
	{"align",intp_align,"00"},
	{"ignoreobstacles",intp_ignoreobstacles,"00"},
/// fjernet af christian - gammel smrd kode
#if (0)
	{"rs485send",intp_rs485send,"19d"},
#endif
	{"vision",intp_vision,"11s"},
	{"laser",intp_laser,"11s"},
	{"putevent",intp_event,"11s"},
	{"syncevent",intp_sync,"11s"},
	{"targethere",intp_targethere,"00"},
	{"resetodocov",intp_resetodocov,"00"},
	{"addwatch",intp_addwatch,"22sd"},
	{"array",intp_array,"22sd"},
        {"trans",intp_trans,"66dddddd"},
	{"invtrans",intp_invtrans,"66dddddd"},
        {"stringcat",intp_stringcat,"19s"},
	{"tointstring",intp_tointstring,"11d"},
	{"drivetopoint",intp_drivetopoint,"45dddds"},
	{"movetoxyz",intp_movetoxyz,"33ddd"},
	{"funcgen",intp_funcgen,"15d"},
        {0, 0, 0}
     };



