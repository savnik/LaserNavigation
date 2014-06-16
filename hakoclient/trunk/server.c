#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "functions.h"
#include "server.h"

void start_server()
{
		/* create datagram socket using UDP */
//	printf("Creating datagram socket.\n");
	serv.s = socket(AF_INET, SOCK_DGRAM, 0);
	if( serv.s == -1 )
		printf("Socket was not created.\n");
	else
		printf("Socket created successfully.\n");

	/* set up the server name */
	serv.server.sin_family = AF_INET;
	serv.server.sin_port = serv.port; /* use first available port number */
	serv.server.sin_addr.s_addr = INADDR_ANY;

	if( bind(serv.s, &serv.server, sizeof( serv.server )) < 0 ) {
		printf("Error binding server.\n");
		exit(3);
	}

	/* find out what port was assigned */
	serv.namelen = sizeof( serv.server );
	if( getsockname( serv.s, (struct sockaddr *) &serv.server, &serv.namelen) < 0 ) {
		perror("getsockname()\n");
		exit(3);
	}
	printf("The assigned port is %d\n", ntohs( serv.server.sin_port));

	/* receive message on socket s in buf */
	serv.client_address_size = sizeof( serv.client );
}

void server_rx()
{
	printf("Waiting for a message to arrive.\n");
	if( recvfrom(serv.s, serv.buf, sizeof(serv.buf), 0, (struct sockaddr *) &serv.client, &serv.client_address_size) < 0 )
	{
		printf("recvfrom()\n");
		exit(4);
	}
	/* print the message */
	printf("%s\n", serv.buf );
//	sendto( serv.s, serv.buf, (strlen(serv.buf)+1), 0, &serv.client, sizeof(serv.client ) );
//	close(serv.s);
	printf("Socket closed.\n");
}
