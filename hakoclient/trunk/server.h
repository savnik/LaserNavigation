#ifndef SERVER_H
#define SERVER_H


#define STRING_LENGTH 1000


typedef struct
{
  // User input:
  int sockint, s;
	unsigned int namelen;
	unsigned int client_address_size;
	struct sockaddr_in client, server;
	char buf[STRING_LENGTH];
  char *hostname;
  int port;

} servertype;

void start_server(void);
void server_rx(void);

#endif
