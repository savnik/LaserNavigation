#include <stdio.h>

#include <stdlib.h>
#include <string.h>

#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "xmlio.h"

void xml_proc(struct xml_in *x){
 double a,b;
 while(1){
 switch (xml_in_nibble(x)) {
    case XML_IN_NONE:
      return;
    case XML_IN_TAG_START:
    //  printf("start tag: %s, %d attributes\n", x->a, x->n);
    //  for(i=0;i<x->n;i++){
      //  printf("  %s    %s  \n",x->attr[i].name,x->attr[i].value);
     // }
     if (strcmp("pos3d",x->a)==0)
       // printf("  %s    %s  \n",x->attr[1].name,x->attr[1].value);
      if (getdouble(&a,"x",x)) printf(" a %lf\n",a);
      //if (getdouble(&b,"b",x)) printf(" b %lf\n",b);
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
int main()
{
  struct xml_in *x;
  int i;
 
  struct sockaddr_in serv_adr;
  int sockfd,errno,connected;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
//printf(" sock %d \n",sockfd);
if ( sockfd < 0 )
{
  perror(strerror(errno));
  fprintf(stderr," Can not make  socket\n");
  exit(errno);
}
serv_adr.sin_family = AF_INET;
serv_adr.sin_port= htons(24920);
serv_adr.sin_addr.s_addr = inet_addr("192.38.66.89");
#if(1)
if ((connected=connect(sockfd, (struct sockaddr *) &serv_adr, sizeof(serv_adr))) <0){
     printf("Not connected to cameraserver\n");   
}
else{
  int len;
  char buffer[256];
  printf(" connected to cameraserver \n");
  len = sprintf(buffer, "push t=5 cmd=gmkget\n");
  send( sockfd, buffer, len, 0);    
}
#endif




  x = xml_in_init(4096, 32);

  while (xml_in_fd(x, sockfd) > 0) xml_proc(x);
  
  return 0;
}
