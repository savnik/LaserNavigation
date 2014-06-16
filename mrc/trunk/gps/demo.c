/**\file demo.c
 * \brief The main file for the gpsclient example program.
 * 
 * It contains all relevant code to show how to interface to
 * the libgps library.
 * 
 * \author Lars Valdemar Mogensen
 * \date 02/04-2006
 */
 
/// Include statement for the libgps library
#include "../gps/libgps.h"

/// How many times should it loop before exiting
#define TEST_RUNS 20

void print_UTM(UTM_gpstype* gps);

/** \brief Test rutine for the GPS client side
 * 
 * \return 0 on success and non-zero on error
 */
int main(int argc, char **argv)
{
	int i=0;
	gpstype gps;
	
	//Connect to socket
	gps.client.hostname=SOCKET_HOSTNAME;
	gps.client.port=SOCKET_PORT;
	gps_socket_connect(&gps);
	if (gps.client.error){
		fprintf(stderr,"Can't connect to GPS server \n");
		exit(gps.client.error);
	}
	printf("Connected to UTMgpsd\n");
	
	//Initialize GPS struct
	gps_init(&gps);
	
	//Read socket to initialise variables
	gps_read(&gps);
	gps_read(&gps);
	gps_read(&gps);
	
	//Loop 20 times
	for(i=0;i<TEST_RUNS;i++)
	{
		gps_read(&gps);
		print_UTM(&gps.UTM);
		printf("%i\n",i);
	}
	
	//EGNOS enabling test
	gps_write(&gps);
	
	//Loop 20 times
	for(i=0;i<TEST_RUNS;i++)
	{
		gps_read(&gps);
		print_UTM(&gps.UTM);
		printf("%i\n",i+TEST_RUNS);
	}
	
	//EGNOS disabling test
	gps_write(&gps);
	
	//Loop 20 times
	for(i=0;i<TEST_RUNS;i++)
	{
		gps_read(&gps);
		print_UTM(&gps.UTM);
		printf("%i\n",i+TEST_RUNS*2);
	}
	
	//Disconnect from socket
	gps_socket_disconnect(&gps);
	
	printf("Disconnected from UTMgpsd\n");
	
	return 0;
}

///Print the UTM struct for debugging and monitoring of the program status
void print_UTM(UTM_gpstype* gps)
{
	printf("**** UTM struct ****\n");
	printf("Valid       : %6i  ",gps->valid);
	printf("Quality     : %6i  ",gps->quality);
	printf("Satellites  : %6i  ",gps->satellites);
	printf("DOP         : %6.4f\n",gps->dop);
	printf("Hour        : %6i  ",gps->time_h);
	printf("Min         : %6i  ",gps->time_m);
	printf("Sec         : %6i  ",gps->time_s);
	printf("C_sec       : %6i\n",gps->time_cs);
	printf("Day         : %6i  ",gps->date_d);
	printf("Month       : %6i  ",gps->date_m);
	printf("Year        : %6i\n",gps->date_y);
	printf("Northing    : %20.8f  ",gps->northing);
	printf("Easting     : %20.8f\n",gps->easting);
	printf("Height      : %6.4f  ",gps->height);
//	printf("Height diff : %d\n",gps->height2);
	
	if(gps->valid==1)
		printf("GPS measurement valid!\n");
	else
		printf("GPS measurement NOT valid!\n");
}
