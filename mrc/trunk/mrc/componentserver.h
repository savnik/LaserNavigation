

#include <sys/socket.h>
#include <netinet/in.h>
typedef struct {
	char name[80];
        struct sockaddr_in serv_adr;
	int connected;
        char host[80];
	int port;
	int sockfd;
	int config;
	int status;
	int run;
	int use;
}componentservertype;
