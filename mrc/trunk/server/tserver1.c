#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include "server1.h"

#define BUFLEN 100

int main(int argc, char *argv[])
{
	int i,len, no_iter = 10,run,cnt,cmdstate=0,res;
        int  n, pin=0,pout=0,nbuf=0,pcmd=0,ready=0;
	unsigned int tdelay = 100000;
	char	buffer[BUFLEN],buffer1[BUFLEN],cmdbuf[100];
	servertype serv;
	if (argc == 1) serv.port = 31370;
	else
	if (argc == 2) {
		serv.port = atoi(argv[1]);
	} else {
		printf("Usage: %s [portno]\n",argv[0]);
		exit(1);
        }
	res=startserver(&serv);
        if (serv.s==0) printf(" socket error \n");
        no_iter=1000;
	/* Send the data */
        run=1;
        cnt=0;
	while (run) {
               		
                n=recv(serv.s,buffer,100,0);
                if (n>0){
                   for (i=0;i<n;i++){
                      buffer1[pin++]=buffer[i];
                      nbuf++;
                      if (pin==BUFLEN) pin=0;
                   }
            
                 } 
                    
                if (nbuf>0){
                  switch (cmdstate){
                  case 0: while(buffer1[pout]==0xa || buffer1[pout]==0xd){
                            pout++;nbuf--;
                            if (pout == BUFLEN) pout=0;
                            if (nbuf==0) break;
                          }                      
                          pcmd=0;
                  case 1: if (nbuf >0){
                          while(buffer1[pout]!=0xa && buffer1[pout]!=0xd && nbuf>0){
                            cmdbuf[pcmd++]=buffer1[pout++];                         
                            if (pout== BUFLEN) pout=0;
                            nbuf--;
                            if (nbuf==0)
                              cmdstate=1;                        
                          } 
                          if (nbuf > 0){
                              cmdbuf[pcmd]='\0';
                              ready=1;
                             cmdstate=0;
                          }
                    break;
                    default:
	            break;
                    }
                  }   
                }
                if (ready){
                  ready=0;
                  printf("%s\n",cmdbuf);
                  if (strncmp(cmdbuf,"end",3)==0) run=0;
                }
                cnt++;
		usleep(tdelay);
	}
       printf(" cnt %d\n",cnt);
       close(serv.s);
       close(serv.ls);
       exit(0);
}
