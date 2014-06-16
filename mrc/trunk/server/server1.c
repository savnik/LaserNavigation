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

int startserver(servertype *p){
  int status,addr_len,reuseaddr;
  
  /* Create the socket */
  p->ls = socket(AF_INET, SOCK_STREAM, 0);
  if ( p->ls == -1 ) return 1;
  reuseaddr = 1;
  if (setsockopt(p->ls, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr)) == -1) {
	fprintf(stderr,"startserver: Unable to set socket options on p->ls (first attempt)\n");
	fprintf(stderr,"Error: %d (%s)\n",errno,strerror(errno));
  }
  p->name.sin_family = AF_INET;
  p->name.sin_port = htons(p->port);
  p->name.sin_addr.s_addr = INADDR_ANY;
	
  /* Bind socket address to the socket  */
  status = bind(p->ls, (struct sockaddr*) &p->name, sizeof(p->name));
  if ( status == -1 ) return 2;

  /* Make queue where the client processes can ask for connection */
  status = listen(p->ls, 1);
  if ( status == -1 ) return 3;
    
  /* Waits on a connection from a client and returns with
   * the address on the client process ("peer") and a new
   * socket decriptor to the new connection  */
  addr_len = sizeof(struct sockaddr_in);
  p->s = accept(p->ls, (struct sockaddr*) &p->from, &addr_len);
  if ( p->s == -1 ) return 4;

  if (fcntl(p->s,F_SETFL,O_NONBLOCK) == -1) {
	fprintf(stderr,"startserver: Unable to set flag O_NONBLOCK on p->s\n");
	fprintf(stderr,"Error: %d (%s)\n",errno,strerror(errno));
  }
  return(0); 
}

int stopserver(servertype *p) {
  fprintf(stderr,"stopserver: Shutting down socket\n");

  if (shutdown(p->ls,SHUT_RDWR) == -1) {
	fprintf(stderr,"stopserver: Error while shutting down socket\n");
  }
  if (close(p->s) == -1) {
	fprintf(stderr,"stopserver: Error closing p->s\n");
	fprintf(stderr,"Error: %d (%s)\n",errno,strerror(errno));
  }
  if (close(p->ls) == -1) {
	fprintf(stderr,"stopserver: Error closing p->ls\n");
	fprintf(stderr,"Error: %d (%s)\n",errno,strerror(errno));
  }

  return 1;
}
