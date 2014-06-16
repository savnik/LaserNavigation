

#ifndef LIBHAKO_H
#define LIBHAKO_H


#define HAKO_SOCKET_PORT 8060
#define HAKO_SOCKET_HOSTNAME "localhost"
#define RXTIMEOUT 0.2
#define CAN_MSG_LEN 11
#define CAN_MAX_DATA_LEN 8
#define MAX_ENGINE_SPEED 10000
#define MAX_FORWARD_SPEED 10000
#define MAX_BACKWARD_SPEED -10000
#define MAX_STEERING_ANGLE (M_PI/2.0)
#define MIN_ENGINE_SPEED 0
#define XBOW_ACC_RANGE 10.0
#define XBOW_ANG_RANGE 200.0
#define SPEEDCONST 2.0267
#define SPEEDOFF	 0.1297
#define TOOLTHRESHOLD 5
#define TOOLPOSDEFAULT 80

//forward declaration or prototype

#define STRING_LENGTH 100

	typedef struct
	{
	  // User input:
	  char *hostname;
	  int port;
	  
	  // Internal variables:
		unsigned char in_msg[13];
		unsigned char out_msg[13];
	
	  // Error value:
	  int error;   // Returns a number different from 0 if an error occurs
	
	  // Output value:
	  int sockfd;  
	
	} clienttype;
	
	typedef struct
	{
		double roll;
		double pitch;
		double yaw;
		double accX;
		double accY;
		double accZ;
		double temp;
		short time;
	} xbowtype;
	
	typedef struct
	{
		int quality;
		int satellites;
		double dop;
		int time_h;
		int time_m;
		int time_s;
		int time_cs;
		int date_d;
		int date_m;
		int date_y;
		double northing;
		double easting;
		double height;
		int height2;
	} gpstype;

	typedef struct
	{
		double steeringAngle;
		double steeringAngleRef;
		double speedRef;
		int cvtPulses;
		int engineSpeed; 		//rounds per minute
		int engineSpeedRef;
		char navigationMode;		//m=manual, a=automatic
		char navigationModeRef;
		unsigned short state[4];
		unsigned char directionByte;
		
		double liftingGearStateRef;
		double liftingGearPos;
		double powerTakeoffStateRef;
		double powerTakeoffSpeed;
		clienttype client;
		xbowtype xbow;
		gpstype gps;
		FILE *conf;
		double sc1;
		double sc2;
	} hakotype;

	
	void hako_init(hakotype*);
	int hako_read(hakotype*);
	int hako_write(hakotype*);
	
	int socket_connect(hakotype*);
	int socket_disconnect(hakotype*);
	
	int send_message(int id, int lenght, char* data, clienttype* client);
	int recv_message(clienttype*);	
	
	int parse_msg(hakotype*);	

#endif

