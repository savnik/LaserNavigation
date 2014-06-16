#include "../robotinc/motioncontrol.h"
#include "../robotinc/linesensor.h"
#include "../robotinc/statemachine.h"
#include "../robotinc/wallest.h"
#include "../guidemark/guidemark.h"
#include "odometry.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
void userprint(char *);
void smrsystemif(char *);
void rs485send(unsigned char *);
extern motiontype motcon;
extern odotype odo;
extern wallesttype wallest;

extern int (*defaultcond)(void);
extern int linetype,guidemarkok;
extern sm_type *intp_smdataptr;
extern int  (*intp_smfuncptr)(sm_type * sm);
extern UCameraGmk gcm;
extern int inputready;
extern int rdl,rdr;

int sockfd,wificonnected;
struct	sockaddr_in serv_adr;
int conditioncounter[10];
sm_type intp_smdata;

void set_digiout(int out);

void setvel(double v){
  motcon.velcmd=v;
}

void setacc(double v){
  motcon.acccmd=v;
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
  return(fabs(motcon.target_dist)<0.005);
}

int intp_fwd(void *par){
  motcon.cmd=MC_FWDCMD;
  motcon.dist=*varptrstack[0];
  defaultcond=intp_fwdcond;
  return(0);
}
  
int intp_turncond(void){
   return(fabs(motcon.poseerr.th)<0.01);
}  
   
int intp_turn(void *par){
  motcon.cmd=MC_TURNCMD;
  if (npar == 2)
    motcon.turnangle=*varptrstack[0];
  else
    motcon.turnangle=*varptrstack[0]*M_PI/180.;
  defaultcond=intp_turncond;
  return(0);
} 

int intp_turnrcond(void){
   return(fabs(motcon.target_dist)<0.005);
}  
   
int intp_turnr(void *par){
  motcon.cmd=MC_TURNRCMD;
  if (npar == 3)
    motcon.turnangle=*varptrstack[1];
  else
    motcon.turnangle=*varptrstack[1]*M_PI/180.;
  motcon.R=*varptrstack[0];
  defaultcond=intp_turnrcond;
  return(0);
} 

int intp_waitcond(void){
   return(*pcmdtime >= condpar0);
}  
   
int intp_wait(void *par){
  symrec *p;
  p=getsym ("cmdtime", sysvar);
  pcmdtime=&(p->value.var);
  condpar0=*varptrstack[0];
  defaultcond=intp_waitcond;
  return(0);
} 


   
int intp_log(void *par){
  int i;
  symrec *s;
  for (i=0; i < npar;i++){ 
    if (logvar.n < 20){
      if (((char *)varptrstack[i])[0]=='$' &&(s=getsym((char *)varptrstack[i]+1, sysvar))!=NULL){
        logvar.ref[logvar.n]=s;
        logvar.n++;
      }
      if ((s=getsym((char *)varptrstack[i], symbols))!=NULL){
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
   
int intp_stream(void *par){
  int i;
  symrec *s;
  streamvar.sampletime=*varptrstack[0];
  for (i=1; i < npar;i++){ 
    if ((s=getsym((char *)varptrstack[i]+1, sysvar))!=NULL){
      streamvar.ref[streamvar.n]=s;
      streamvar.n++;
    }
    if ((s=getsym((char *)varptrstack[i], symbols))!=NULL){
      streamvar.ref[streamvar.n]=s;
      streamvar.n++;
    }

    printf("streamvar %s %d \n",streamvar.ref[streamvar.n-1]->name,streamvar.n);
  }
  defaultcond=defaulttrue;
  return(0);
} 


int intp_drive(void *par){
  motcon.cmd=MC_DRIVECMD;
  if (npar == 3){
    motcon.refpose.x=*varptrstack[0];
    motcon.refpose.y=*varptrstack[1];
    motcon.refpose.th=*varptrstack[2]*M_PI/180;
  }
  else {
    motcon.refpose.x=odo.pose.x;
    motcon.refpose.y=odo.pose.y;
    motcon.refpose.th=odo.pose.th;
  }
  defaultcond=nodefault;
  return(0);
} 

int intp_motorcmds(void *par){
  motcon.cmd=MC_DIRECTMOTORCMDS;
  motcon.velcmdl=*varptrstack[0];
  motcon.velcmdr=*varptrstack[1];
  defaultcond=defaulttrue;
  return(0);
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
  intp_smdataptr->p[0]=*varptrstack[0];
  printf(" p0intp  %f\n",*varptrstack[0]);
  intp_smfuncptr=sm_barcode;
  defaultcond=defaulttrue;
  return(0);
} 


int intp_followline(void *par){
  printf("followline %s \n" ,(char*) varptrstack[0]);
  if ( strcmp((char *)varptrstack[0],"bm")==0) linetype=LINE_MIDDLE_B;
  else if ( strcmp((char *)varptrstack[0],"br")==0) linetype=LINE_RIGHT_B;
  else if ( strcmp((char *)varptrstack[0],"bl")==0) linetype=LINE_LEFT_B;
  else if ( strcmp((char *)varptrstack[0],"wm")==0) linetype=LINE_MIDDLE_W;
  else if ( strcmp((char *)varptrstack[0],"wr")==0) linetype=LINE_RIGHT_W;
  else if( strcmp((char*)varptrstack[0],"wl")==0) linetype=LINE_LEFT_W;
  else if ( strcmp((char *)varptrstack[0],"bmc")==0) linetype=LINE_MIDDLE_B_CAM;
  else linetype=LINE_MIDDLE_B;
 printf("followline %s %d\n" ,(char*) varptrstack[0],linetype);

  motcon.cmd=MC_FOLLOW_LINECMD;
  defaultcond=nodefault;
  return(0);
} 

int intp_followwall(void *par){
  if ( ((char*)varptrstack[0])[0]=='r') 
    motcon.side=MC_RIGHT;
  else if ( ((char*)varptrstack[0])[0]=='l')
    motcon.side=MC_LEFT;
  else  printf("wrong parameter\n");
  
  motcon.walldistref=*varptrstack[1];

  motcon.cmd=MC_FOLLOW_WALLCMD;
  defaultcond=nodefault;
  return(0);
}

int intp_followwall2(void *par){
  if ( ((char*)varptrstack[0])[0]=='r') 
    motcon.side=MC_RIGHT;
  else if ( ((char*)varptrstack[0])[0]=='l')
    motcon.side=MC_LEFT;
  else  printf("wrong parameter\n");
  if ( ((char*)varptrstack[0])[1]=='d') 
    motcon.method=MC_DIRECT;
  else if ( ((char*)varptrstack[0])[1]=='r')
    motcon.method=MC_REGRESSION;
  else printf("Wrong parameter \n");


  motcon.walldistref=*varptrstack[1];

  motcon.cmd=MC_FOLLOW_WALLCMD2;
  defaultcond=nodefault;
  return(0);
}

int intp_goto(void *par){
  char *line;
  line=findlabel((char*)varptrstack[0],currentplan);
  defaultcond=nodefault;
  if (line){
    currentplan->pline=line;
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
  printf(" exp %lf label %s \n",exp,label);
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
 
int intp_get(void *par){
  printf("get %s \n" ,(char*) varptrstack[0]);
  if ( strcmp((char *)varptrstack[0],"guidemark")==0){ 
     guidemarkok=0;
    newImage(&gcm);
  }
  defaultcond=defaulttrue;
  return(0);
}

int intp_set(void *par){
  if ( strcmp((char *)varptrstack[0],"rdr")==0) rdr=*varptrstack[1];
  if ( strcmp((char *)varptrstack[0],"rdl")==0) rdl=*varptrstack[1];
  if ( strcmp((char *)varptrstack[0],"conditioncounter")==0) conditioncounter[(int)(*varptrstack[1])]=0;
  if ( strcmp((char *)varptrstack[0],"odocontrol")==0) odo.control=*varptrstack[1];
  if ( strcmp((char *)varptrstack[0],"$odox")==0) odo.pose.x=*varptrstack[1];
  if ( strcmp((char *)varptrstack[0],"$odoy")==0) odo.pose.y=*varptrstack[1];
  if ( strcmp((char *)varptrstack[0],"$odoth")==0) odo.pose.th=*varptrstack[1];
  if ( strcmp((char *)varptrstack[0],"$digiout")==0){
    int dig;
     dig=*varptrstack[1];
     dig=(dig & 0x7)<< 3;
     set_digiout(dig);
  }
  defaultcond=defaulttrue;
  return(0);
}

int intp_rs485send(void *par){
   unsigned char msg[32],h;  
   if (npar >1){
     msg[0]=npar-1;
     msg[1]= *varptrstack[0];
     h=*varptrstack[1];
     msg[1]|=(h<<4);
     for (h=2;h<npar;h++)
        msg[h]=*varptrstack[h];
     rs485send(msg);
   }
   defaultcond=defaulttrue;
   return(0);
}

 
int intp_open(void *par){
  printf("open %s \n" ,(char*) varptrstack[0]);
  if ( strcmp((char *)varptrstack[0],"wifi")==0){ 
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
  len=sprintf(buf,"click  %s %6.2f %6.2f ",(char *)varptrstack[0],*varptrstack[1],*varptrstack[2]);
  if (wificonnected > -1)
      send( sockfd, buf, len, 0);    
  defaultcond=defaulttrue;
  return(0);
}  

int intp_play(void *par){
  char buf[1000];
  strcpy(buf,"play ");
  strcat(buf,(char*)varptrstack[0]);
  smrsystemif(buf);
  defaultcond=defaulttrue;
  return(0);
} 

int intp_speak(void *par){
  char buf[1000];
  strcpy(buf,"/usr/local/bin/flite -t \"");
  strcat(buf,(char*)varptrstack[0]);
  strcat(buf,"\"");
  smrsystemif(buf);
  printf("%s",buf);
  defaultcond=defaulttrue;
  return(0);
} 

int intp_print(void *par){
  userprint((char*)varptrstack[0]);
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
  userprint((char*)varptrstack[0]);
  defaultcond=intp_readcond;
  return(0);
}
     
int intp_wallest(void *par){

  if ( ((char*)varptrstack[0])[0]=='r')
    wallest.side=WE_RIGHT;
  else if ( ((char*)varptrstack[0])[0]=='l')
    wallest.side=WE_LEFT;
  else  printf("wrong parameter\n");
  if ( ((char*)varptrstack[0])[1]=='b')
    wallest.status=WE_ON;
  else if ( ((char*)varptrstack[0])[1]=='c')
    wallest.status=WE_OFF;
  else printf("Wrong parameter \n");

  wallest.line_distance = *varptrstack[1];
	
  if(npar >= 3) wallest.input_min_wall_after_hole = *varptrstack[2];
	else wallest.input_min_wall_after_hole = 0.0;
	
  if(npar >= 3) wallest.input_max_hole_dist = *varptrstack[3];
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
	{"drive",intp_drive,"03ddd"},
	{"motorcmds",intp_motorcmds,"22dd"},
	{"turnr",intp_turnr,"23dds"},
	{"followline",intp_followline,"11s"},
        {"followwall",intp_followwall,"22sd"},
	{"followwall2",intp_followwall2,"22sd"},
	{"label",intp_label,"11s"},
	{"wait",intp_wait,"11d"},
        {"goto",intp_goto,"11s"},
	{"log",intp_log,"19s"},
	{"stream",intp_stream,"19ds"},
        {"stop",intp_stop,"00"},
	{"idle",intp_idle,"00"},
        {"resetmotors",intp_resetmotors,"00"},
        {"barcode",intp_barcode,"00"},
        {"get",intp_get,"11s"},
        {"set",intp_set,"22sd"},
        {"open",intp_open,"11s"},
        {"writewifi",intp_writewifi,"33sdd"},
        {"play",intp_play,"11s"},  
        {"speak",intp_speak,"11s"},
        {"print",intp_print,"11s"},
        {"read",intp_read,"11s"},
        {"wallest",intp_wallest,"24sddd"},
	{"align",intp_align,"00"},
	{"ignoreobstacles",intp_ignoreobstacles,"00"},
	{"rs485send",intp_rs485send,"19d"},
        {0, 0, 0}
     };



