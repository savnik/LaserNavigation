#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <netdb.h>
#include <fcntl.h>
#include <string.h>
#include "../mrc/componentserver.h"
#include "server1.h"
#include "stream2line.h"
#include <pthread.h>
#include "queue.h"

#define INITFILE	"smrclient.init"
#define HTMLDIR		"/srv/httpd/htdocs/"
char rcbuf[256];
char svr_buf[256];
char smrstatus[16];
int smrready;
pthread_t  server_thread;
pthread_attr_t attr;
void *server_task(void *not_used);

Queue smrcmdqueue;

void serverconnect(componentservertype *s){
struct hostent *host  ;
s->serv_adr.sin_family = AF_INET;
s->serv_adr.sin_port= htons(s->port);
host=gethostbyname(s->host);
if (host){
  memcpy(&s->serv_adr.sin_addr.s_addr, host->h_addr_list[0],host->h_length); // s->host
  printf("port %d host %s \n",s->port,s->host);
  if ((s->connected=(connect(s->sockfd, (struct sockaddr *) &s->serv_adr, sizeof(s->serv_adr))) >-1)){
    printf(" connected to %s  \n",s->name);
    
  }
  else{
     printf("Not connected to %s  %d \n",s->name,s->connected);   
  }
}
else {
printf("Not connected to %s  %d \n",s->name,s->connected);   
}
}



void smrcmd(componentservertype *s,char *cmd){
     int len;
     char buf[256];
     len=sprintf(buf,cmd);
     send(s->sockfd,buf,len,0);
      len=recv(s->sockfd,rcbuf,256,0);
     rcbuf[len]=0;
     if (strcmp(rcbuf,"eventtimeout\n")!=0) {
       printf(rcbuf);
       if (strncmp(rcbuf,"userevent",9)==0){
          if (strncmp(&rcbuf[10],"smrready",8)==0){
	    smrready=1;
	  } 
	  else {
	    strcpy(smrstatus,&rcbuf[10]);
	    printf("%s \n",rcbuf);
	  }
       }	    
     }
}

componentservertype smr;

char  cmdqueue[100][256]; 

char cmdbuf[256];
char ansbuf[256];
servertype serv;
stream2line_type s2lbuf;
FILE *f;


int main(int argc,char *argv[]){
int arg=0,runsmr=1;
smrcmdqueue=CreateQueue(100,256);
if (!smrcmdqueue){
   printf("Could not allocate cmdqueue \n");
   exit(1);
}
// **************************************************
//   server code initialization
//

/* Create endpoint */
   smr.config=1;
   if (smr.config) {
      int errno; 
      smr.sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if ( smr.sockfd < 0 )
     { 
       perror(strerror(errno));
       fprintf(stderr," Can not make  socket\n");
       exit(errno);
     }
   }

strcpy(smr.host,"smr2");
if (argc >1){
  strcpy(smr.host,argv[1]);
  
}

strcpy(smr.name,"mysmr");
smr.port=31001;
serverconnect(&smr);
pthread_attr_init(&attr);
pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

if (pthread_create(&server_thread, &attr, server_task, 0))
    {
      perror("can't start server thread");
      exit(-1);
    }


smrready=1;
smrstatus[0]=0;	   
while (runsmr){    
  if (!IsEmpty(smrcmdqueue)){
    FrontAndDequeue(smrcmdqueue,cmdbuf);
    smrcmd(&smr,cmdbuf);    
  }
  rcbuf[0]=0;
  while (strcmp(rcbuf,"eventtimeout\n")!=0){
     smrcmd(&smr,"getevent 0.2\n");
  }
  ioctl(0, FIONREAD, &arg);
  if (arg != 0)
    runsmr = 0;
  }
smrcmd(&smr,"exit\n");
DisposeQueue(smrcmdqueue);
return 0;
}


/* Server functions */
 
void  gotofunc(char *buf, int client){
   char startpos[32];
   char startdir[32];
   char tempbuf[256];
   char target[32];
   char posefile[256];
   char dirfile[256];
   int n;
   startpos[0]=0;
   startdir[0]=0;
   tempbuf[0]=0;
   if (smrready){
     strcpy(target,&buf[5]);
     printf("target %s \n",target);
     strcpy(posefile,HTMLDIR);
     strcat(posefile,"smrpose");
    
     f=fopen(posefile,"r"); 
     fscanf(f,"%s",startpos);
    
     fclose(f);
     strcpy(dirfile,HTMLDIR);
     strcat(dirfile,"smrdir");
    
     f=fopen(dirfile,"r"); 
     fscanf(f,"%s",startdir);
   
     fclose(f);
     sprintf(tempbuf,"%sshortest_pathpig -f %s -t %s -g  %sgraph2.dgl -a %s >plan1",
	              HTMLDIR,startpos,target,HTMLDIR,startdir);
     printf(tempbuf);
     system(tempbuf);
    
     f=fopen("plan1","r"); 
     if (f==NULL){
       printf("File with plan not found\n");
       exit(1);
     }
     printf("\n plan1 found\n");
     n=0;
     while (fgets(tempbuf,256,f)!=0){
       Enqueue(smrcmdqueue,tempbuf);
       printf("%s \n",tempbuf);     
     }
     Enqueue(smrcmdqueue,"putevent \"smrready\" \n");	    
     fclose(f);
     f=fopen(posefile,"w"); 
     fprintf(f,"%s",target);
     fclose(f);
     f=fopen(dirfile,"w"); 
     fprintf(f,"%s",&tempbuf[2]);
     fclose(f);
     smrready=0;
     n=sprintf(ansbuf,"smr comming to %s \n",target);
     send(client,ansbuf,n,0);
   }
   else {
     n=sprintf(ansbuf,"smr occupied try later \n");
     send(client,ansbuf,n,0);
   } 	   	    
 }

void  initfunc(void){
   char startpos[16];
   char startdir[16];
   char tempbuf[256];
   char target[16];
   char posefile[256];
   int n;
     printf("target %s \n",target);
     strcpy(posefile,HTMLDIR);
     strcat(posefile,"smrpose"); 
     f=fopen(posefile,"r"); 
     fscanf(f,"%s",startpos);
     printf("%s \n",startpos);
     fclose(f);
     f=fopen(INITFILE,"r"); 
     if (f==NULL){
       printf("%s is not found\n",INITFILE);
       exit(1);
     }
     printf("%s found\n",INITFILE);
     n=0;
     while (fgets(tempbuf,256,f)!=0){
       Enqueue(smrcmdqueue,tempbuf);
       printf("%s \n",tempbuf);     
     }
     
     fclose(f);
      	    
 }



void robotstatusfunc(char * buf, int client){
  char ansbuf[256];
  int n; 
  smrstatus[15]=0;
  n=sprintf(ansbuf,"Robot at: %s \n",smrstatus);
  send(client,ansbuf,n,0); 
}

void closefunc(char * buf, int client){
  close(client);
}


/*
 * Server
 */

#define SERVER_PORT 31101

void *server_task(void *not_used)
{
  int s,run=1, client;

  fprintf(stderr, "server task running\n");
 
  if ((s = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("server: socket");
      pthread_exit(0);
    }

  {
    struct sockaddr_in sa;
    sa.sin_family =  AF_INET;
    sa.sin_port = htons(SERVER_PORT);
    sa.sin_addr.s_addr = INADDR_ANY;
    if (bind(s,(struct sockaddr *) &sa, sizeof(sa)))
      {
	perror("server: bind");
	pthread_exit(0);
      }
  }

  if (listen(s, 1))
      {
	perror("server: listen");
	pthread_exit(0);
      }
  initfunc();
  stream2lineinit(&s2lbuf);
  while ((client = accept(s, 0, 0)) >= 0){   
     printf("client = %d \n",client);
     run=1;
     {
       int x=1;
       if (setsockopt(client, SOL_TCP, TCP_NODELAY, &x, sizeof(x)))
         perror("setsockopt");
     } 
     
     while (run){
       s2lbuf.Nib=recv(client,s2lbuf.ib,S2L_IBSIZE,0);
       if (s2lbuf.Nib >0){
        stream2line(&s2lbuf);
       }	
       if (s2lbuf.nlines > 0 ){
         getnextline(svr_buf,&s2lbuf);
        
	  
         printf("cmd input %s \n",svr_buf); 
#if (1)
         if (strncmp(svr_buf,"goto",4)==0){ 
	   gotofunc(svr_buf,client);
  
	 } 
	 else if (strncmp(svr_buf,"stop",4)==0){ 
	   closefunc(svr_buf,client);
	   run=0;
	 }
         else if (strncmp(svr_buf,"robotstatus",11)==0){
           robotstatusfunc(svr_buf,client);
         } 
#endif	 
        closefunc(svr_buf,client);
	   run=0;
       }
     }
     printf("accept\n");
   }

  perror("server: accept");
  pthread_exit(0);
}








