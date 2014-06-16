/* server program, run this first */

#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

main()
{
	int sockint, s, namelen, client_address_size;
	struct sockaddr_in client, server;
	char buf[32];

	/* create datagram socket using UDP */
	printf("Creating datagram socket.\n");
	s = socket(AF_INET, SOCK_DGRAM, 0);
	if( s == -1 )
		printf("Socket was not created.\n");
	else
		printf("Socket created successfully.\n");

	/* set up the server name */
	server.sin_family = AF_INET;
	server.sin_port = htons(2000); /* use first available port number */
	server.sin_addr.s_addr = INADDR_ANY;

	if( bind(s, &server, sizeof( server )) < 0 ) {
		printf("Error binding server.\n");
		exit(3);
	}

	/* find out what port was assigned */
	namelen = sizeof( server );
	if( getsockname( s, (struct sockaddr *) &server, &namelen) < 0 ) {
		perror("getsockname()\n");
		exit(3);
	}
	printf("The assigned port is %d\n", ntohs( server.sin_port));

	/* receive message on socket s in buf */
	client_address_size = sizeof( client );
	printf("Waiting for a message to arrive.\n");
	if( recvfrom(s, buf, sizeof(buf), 0, (struct sockaddr *) &client, &client_address_size) < 0 )
	{
		printf("recvfrom()\n");
		exit(4);
	}
	/* print the message */
	printf("Data has been sent to the socket\n");
	printf("The message was\n");
	printf("%s\n", buf );

	printf("Closing the socket connection.\n");
	close(s);
	printf("Socket closed.\n");
}