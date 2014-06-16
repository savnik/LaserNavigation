#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

main( argc, argv)
int argc;
char **argv;
{
	int s;
	unsigned short port;
	struct sockaddr_in server;
	char buf[32];

	/* argv[1] is internet address of server
	argv[2] is port number
	Convert the port from ascii to integer and then
	from host byte order to network byte order using
	htons()
	*/
	port = htons( 2000 /*atoi( argv[2] )*/);

	/* create datagram socket using UDP */
	printf("Creating datagram socket.\n");
	s = socket(AF_INET, SOCK_DGRAM, 0);
	if( s == -1 )
		printf("Socket was not created.\n");
	else
		printf("Socket created successfully.\n");

	/* set up the server name */
	server.sin_family = AF_INET;
	server.sin_port = port;
	server.sin_addr.s_addr = inet_addr( "127.0.0.1"/*argv[1]*/ );

	strcpy( buf, "Hello" );
	printf("Sending data to the socket.\n");
	sendto( s, buf, (strlen(buf)+1), 0, (struct sockaddr *) &server, sizeof(server ) );
	printf("Data has been sent to the socket\n");
	unsigned char len;
	int i;
	while(1)
	{	
		recvfrom(s, buf, 10, 0, (struct sockaddr *) &server, sizeof(server));
		for(i=0;i<10;i++)
		{
			printf("%2x ", (unsigned char)buf[i]);
		}
		printf("\n--\n");
		
	}
	printf("Closing the socket connection.\n");
	close(s);
	printf("Socket closed.\n");
}
