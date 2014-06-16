
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "libhako.h"

double degrees_to_rads(double value)
{
	return value*M_PI/180.0;
}

///Hardware server connect function
///Connects to the hardware server using a streaming socket connection
int socket_connect(hakotype *hako)
{
	struct sockaddr_in serv_addr;
  struct hostent *host_param;
  hako->client.error = 0;
  
  if((hako->client.sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
      fprintf(stderr, "Can not create socket\n");
      hako->client.error = 1;
      return 0;
    }
  if((host_param = gethostbyname(hako->client.hostname)) == NULL)
    {
      fprintf(stderr, "Can not connect to %s\n", hako->client.hostname);
      hako->client.error = 2;
      return 0;
    }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *) host_param->h_addr, (char *) &serv_addr.sin_addr.s_addr, host_param->h_length);
  serv_addr.sin_port = htons(hako->client.port);
  
  if(connect(hako->client.sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) != 0)
    {
      fprintf(stderr, "Error while trying to connect to socket on: %s, port: %d\n", hako->client.hostname, hako->client.port);
      hako->client.error = 3;
      return 0;
    }
    
     {
    int x = 1;

    if (setsockopt(hako->client.sockfd, SOL_TCP, TCP_NODELAY, &x, sizeof(x)))
    	{
     fprintf(stderr, "Error setting Socket options\n");
  	 hako->client.error = 4;
  	 }
  	}
  	

		return 1;
}


///Hardware server disconnect function
///Disconnects from the hardware server
int socket_disconnect(hakotype *hako)
{
  close(hako->client.sockfd);
  
  hako->client.error = 0;
	return 1;	
}


///Hako read function
///Polls the hardware server, receives and parses the replies.
int hako_read(hakotype* hako)
{
	int status = 0;

	//Poll requests
	send_message(0x110, 0, NULL, &hako->client); //polling hako, zero data and empty char pointer
	send_message(0x150, 0, NULL, &hako->client); //polling tools, zero data and empty char pointer
	send_message(0x300, 0, NULL, &hako->client); //polling GPS, zero data and empty char pointer
	send_message(0x600, 0, NULL, &hako->client); //polling XBOW, zero data and empty char pointer
	send_message(0xE0F, 0, NULL, &hako->client); //End of sample/transmission
	
	
	//Recieve data until end of frame
	do
	{
		status = recv_message(&hako->client);
		if(status >= 0)
		{
			//Parse the message
			status &= parse_msg(hako);
		
		if((hako->client.in_msg[0] != 0x00 || hako->client.in_msg[1] != 0x50) &&
			 (hako->client.in_msg[0] != 0x01 || hako->client.in_msg[1] != 0x05) &&
			 (hako->client.in_msg[0] != 0x01 || hako->client.in_msg[1] != 0x45) &&
			 (hako->client.in_msg[0] != 0x06 || hako->client.in_msg[1] != 0x01) &&
			 (hako->client.in_msg[0] != 0x06 || hako->client.in_msg[1] != 0x02) &&
			 (hako->client.in_msg[0] != 0x03 || hako->client.in_msg[1] != 0x01) &&
			 (hako->client.in_msg[0] != 0x03 || hako->client.in_msg[1] != 0x02) &&
			 (hako->client.in_msg[0] != 0x03 || hako->client.in_msg[1] != 0x03) &&
			 (hako->client.in_msg[0] != 0x03 || hako->client.in_msg[1] != 0x04) &&
			 (hako->client.in_msg[0] != 0x03 || hako->client.in_msg[1] != 0x05) &&
			 (hako->client.in_msg[0] != 0x0F || hako->client.in_msg[1] != 0x0E))
			 {
			 	fprintf(stderr,"Unexpected communication\n");
			 }
		}
		else
		{
			fprintf(stderr,"Error in hako_read - Timeout waiting for communication\n");
		}
	}while(hako->client.in_msg[0] != 0x0F || hako->client.in_msg[1] != 0x0E);

	return status;	
}

///Hako write function
///Transmits commands to the hardware server
int hako_write(hakotype* hako)
{
	unsigned char posref;
	double speedref;
	char data[8] = {0};
	union{
		short a;
		unsigned char c[2];
	} twobyte1, twobyte2;
	int status = 0;
		
	//scale the commanded speed
	if(hako->speedRef > 0)
		speedref = hako->speedRef * SPEEDCONST + SPEEDOFF;
	else if(hako->speedRef < 0)
		speedref = hako->speedRef * SPEEDCONST - SPEEDOFF;
	else
		speedref = 0;
		
	//Implement controller 
	if(hako->liftingGearPos >= hako->liftingGearStateRef + TOOLTHRESHOLD)
		posref = 1; //Raise the tools
	else if(hako->liftingGearPos <= hako->liftingGearStateRef - TOOLTHRESHOLD)
		posref = 2; //Lower the tools
	else
		posref = 0; //Stop
		
	//writing hako information to data array, using rounding
	twobyte1.a = (short)((hako->steeringAngleRef*hako->sc1 * 180.0/M_PI) * 10.0 + 0.5);
	twobyte2.a = (short)(speedref * 100.0 +0.5);
	data[0] = twobyte1.c[0];
	data[1] = twobyte1.c[1];
	data[2] = twobyte2.c[0];
	data[3] = twobyte2.c[1];		
	data[4] = (unsigned char)(hako->engineSpeedRef/20.0 + 0.5);
	data[6] = (unsigned char)hako->navigationModeRef;
	status = send_message(0x100, 8, data, &hako->client);
	bzero(data, 8);
	data[0] = posref; 
	data[1] = (char)hako->powerTakeoffStateRef;
	status = send_message(0x130, 2, data, &hako->client);	
	return status;
}

///Send one message to the hardware server
int send_message(int id, int len, char* data, clienttype* client)
{
	char to_send[CAN_MSG_LEN+2] = {0};
	int i;
	int status = 1;
	union{
		short a;
		char c[2];
	}u;
	u.a = id;
	
	to_send[0] = u.c[1];
	to_send[1] = u.c[0];
	to_send[2] = (char)(0x00FF & len);
	to_send[11] = '\r';
	to_send[12] = '\n';
	
	for(i=3; i < CAN_MAX_DATA_LEN + 3; i++)
	{
		if((i - 3) < len )
			to_send[i] = data[i-3];
		else
			to_send[i] = 0;	
	}
	status = send(client->sockfd, to_send, CAN_MSG_LEN+2, 0);
	
	return status;
}

///Receive one message from the hardware server
int recv_message(clienttype* client)
{
	double t1,t2;
	struct timeval tp;
	int j=CAN_MSG_LEN+2;
	char	buf_tmp[CAN_MSG_LEN+2];
	char* buf= buf_tmp;
	int	n = 0;
	int status = -1;
	bzero(buf, CAN_MSG_LEN+2);

//Get time to calculate timeout	
	gettimeofday(&tp, NULL);
	t1=(double)tp.tv_sec+(1.e-6)*tp.tv_usec;
	
	//Until full message has been received
	while (j) {
		n=recv(client->sockfd,buf,CAN_MSG_LEN+2,0);
		
		if (n<=0){
			n=0;
			usleep(10);
			printf("maxikuk\n");
		}
		else{
			buf+=n;
			j-=n;
		
			if(status == -1 && buf_tmp[10] == '\r' && buf_tmp[11] == '\n')
			{
				j=0;
			}
			else if(j <= 0 && buf_tmp[11] != '\r' && buf_tmp[12] != '\n') //if not in sync
			{
				j=3;
				buf = buf_tmp + 10;
				status = -1;
				fprintf(stderr,"Message out of sync\n");
			}
		}
		gettimeofday(&tp, NULL);
		t2=(double)tp.tv_sec+(1.e-6)*tp.tv_usec;
		
		//Check for message timeout.
		if(t2-t1 > RXTIMEOUT)
		{
			j=0;
			status = -1;
			fprintf(stderr,"Timeout ");
		}
	}
	
	//If no timeout, accept message
		if(t2-t1 <= RXTIMEOUT)
		{
		status = 1;
		bcopy(buf_tmp,client->in_msg,CAN_MSG_LEN+2);
		}
		
	return status;
}


///parsing socket messages into data struct
int parse_msg(hakotype* hako)
{
	int i;
	union{
		short a;
		char c[2];
	}twobyte;	
	union{
		double a;
		char c[8];
	}u_double;	
	
	twobyte.c[0] = hako->client.in_msg[1];
	twobyte.c[1] = hako->client.in_msg[0];

	switch(twobyte.a)
	{
		case 0x050:
		{
				twobyte.c[0] = hako->client.in_msg[3];
				twobyte.c[1] = hako->client.in_msg[4];
				hako->state[0] = twobyte.a; 
				twobyte.c[0] = hako->client.in_msg[5];
				twobyte.c[1] = hako->client.in_msg[6];
				hako->state[1] = twobyte.a; 
				twobyte.c[0] = hako->client.in_msg[7];
				twobyte.c[1] = hako->client.in_msg[8];
				hako->state[2] = twobyte.a; 
				twobyte.c[0] = hako->client.in_msg[9];
				twobyte.c[1] = hako->client.in_msg[10];
				hako->state[3] = twobyte.a; 
		} 
		break;	
		case 0x105:
		{
			twobyte.c[0] = hako->client.in_msg[3];
			twobyte.c[1] = hako->client.in_msg[4];
			hako->steeringAngle = (twobyte.a/10.0*M_PI/180.0)/hako->sc1;
			twobyte.c[0] = hako->client.in_msg[5];
			twobyte.c[1] = hako->client.in_msg[6];
			hako->cvtPulses = twobyte.a;
			
			hako->engineSpeed = hako->client.in_msg[7]*20;
			hako->navigationMode = hako->client.in_msg[9];
			hako->directionByte = hako->client.in_msg[10];
			
			if((hako->directionByte & (1<<3)) && !(hako->directionByte & (1<<2)))
				hako->cvtPulses *=(-1);
		}
		break;	
		case 0x145:
		{
			hako->liftingGearPos = (int) hako->client.in_msg[3];
			hako->powerTakeoffSpeed = 10*((int)hako->client.in_msg[4]);
		}
		break;
		case 0x601:
		{
			//Data conversions are described in datasheet for imu
		twobyte.c[1] = hako->client.in_msg[3];
		twobyte.c[0] = hako->client.in_msg[4];
		hako->xbow.roll = degrees_to_rads((double)twobyte.a*(XBOW_ANG_RANGE*1.5)/32768); //Conversion to rad/sec
		twobyte.c[1] = hako->client.in_msg[5];
		twobyte.c[0] = hako->client.in_msg[6];
		hako->xbow.pitch = degrees_to_rads((double)twobyte.a*(XBOW_ANG_RANGE*1.5)/32768);//Conversion to rad/sec
		twobyte.c[1] = hako->client.in_msg[7];
		twobyte.c[0] = hako->client.in_msg[8];
		hako->xbow.yaw = degrees_to_rads((double)twobyte.a*(XBOW_ANG_RANGE*1.5)/32768);//Conversion to rad/sec
		twobyte.c[1] = hako->client.in_msg[9];
		twobyte.c[0] = hako->client.in_msg[10];
		hako->xbow.accX = (double)twobyte.a*(XBOW_ACC_RANGE*1.5*9.80)/32768; //Conversion to m/s^2
		}
		break;
		case 0x602:
		{
		twobyte.c[1] = hako->client.in_msg[3];
		twobyte.c[0] = hako->client.in_msg[4];
		hako->xbow.accY = (double)twobyte.a*(XBOW_ACC_RANGE*1.5*9.80)/32768.0;//Conversion to m/s^2
		twobyte.c[1] = hako->client.in_msg[5];
		twobyte.c[0] = hako->client.in_msg[6];
		hako->xbow.accZ = (double)twobyte.a*(XBOW_ACC_RANGE*1.5*9.80)/32768.0;//Conversion to m/s^2
		twobyte.c[1] = hako->client.in_msg[7];
		twobyte.c[0] = hako->client.in_msg[8];
		hako->xbow.temp = 44.4*((double)twobyte.a*5.0/4096.0 - 1.375); //Conversion to deg C
		twobyte.c[1] = hako->client.in_msg[9];
		twobyte.c[0] = hako->client.in_msg[10];
		hako->xbow.time = twobyte.a;
		}
		break;
		case 0x301:
		{
		hako->gps.time_h = hako->client.in_msg[3];
		hako->gps.time_m = hako->client.in_msg[4];
		hako->gps.time_s = hako->client.in_msg[5];
		hako->gps.time_cs = hako->client.in_msg[6];
		hako->gps.date_m = hako->client.in_msg[7];
		hako->gps.date_d = hako->client.in_msg[8];
		hako->gps.date_y = hako->client.in_msg[9];
		}
		break;
		case 0x302:
		{
		u_double.c[0] = hako->client.in_msg[3];
		u_double.c[1] = hako->client.in_msg[4];
		u_double.c[2] = hako->client.in_msg[5];
		u_double.c[3] = hako->client.in_msg[6];
		u_double.c[4] = hako->client.in_msg[7];
		u_double.c[5] = hako->client.in_msg[8];
		u_double.c[6] = hako->client.in_msg[9];
		u_double.c[7] = hako->client.in_msg[10];
		hako->gps.northing = u_double.a;
		}
		break;
		case 0x303:
		{
		u_double.c[0] = hako->client.in_msg[3];
		u_double.c[1] = hako->client.in_msg[4];
		u_double.c[2] = hako->client.in_msg[5];
		u_double.c[3] = hako->client.in_msg[6];
		u_double.c[4] = hako->client.in_msg[7];
		u_double.c[5] = hako->client.in_msg[8];
		u_double.c[6] = hako->client.in_msg[9];
		u_double.c[7] = hako->client.in_msg[10];
		hako->gps.easting = u_double.a;
		}
		break;
		case 0x304:
		{
		u_double.c[0] = hako->client.in_msg[3];
		u_double.c[1] = hako->client.in_msg[4];
		u_double.c[2] = hako->client.in_msg[5];
		u_double.c[3] = hako->client.in_msg[6];
		u_double.c[4] = hako->client.in_msg[7];
		u_double.c[5] = hako->client.in_msg[8];
		u_double.c[6] = hako->client.in_msg[9];
		u_double.c[7] = hako->client.in_msg[10];
		hako->gps.height = u_double.a;
		}
		break;
		case 0x305:
		{
		hako->gps.quality = hako->client.in_msg[3];
		hako->gps.satellites = hako->client.in_msg[4];
		hako->gps.dop = (double)hako->client.in_msg[5]/10.0;
		hako->gps.height2 = hako->client.in_msg[6];
		}
		break;
		case 0xF0E:
		break;
		default:
		{
			printf("Parsing failed ");					
			for(i=0; i<CAN_MSG_LEN+2; i++)
					printf("%2x ", hako->client.in_msg[i]);
			printf("\n");
		}
		break;
	}
	return 1;		
}

void hako_init(hakotype* hako)
{
  int i;
  		//Read steeringAngle scaling from file, should be 1.2
  	if((hako->conf = fopen("calib/hako.conf","r")) < 0)
  		fprintf(stderr,"hako.conf missing\n");
  	fscanf(hako->conf,"%lf %lf",&hako->sc1,&hako->sc2);
		hako->steeringAngle  = 0.0;
		hako->steeringAngleRef = 0.0;
		hako->speedRef = 0.0;
		hako->cvtPulses = 0;
		hako->engineSpeed = 0; 		//rounds per minute
		hako->engineSpeedRef = 1000;
		hako->navigationMode = 'M';		//M=manual, A=automatic
		hako->navigationModeRef = 'M';
		for (i=0;i<4;i++)
		  hako->state[i] = 0;
		hako->directionByte = 0;
		
		hako->liftingGearStateRef = TOOLPOSDEFAULT;
		hako->liftingGearPos = 0;
		hako->powerTakeoffStateRef = 0xFF;
		hako->powerTakeoffSpeed = 0;
		hako->xbow.roll = 0;
		hako->xbow.pitch = 0;
		hako->xbow.yaw = 0;
		hako->xbow.accX = 0;
		hako->xbow.accY = 0;
		hako->xbow.accZ = 0;
		hako->xbow.temp = 0;
		hako->xbow.time = 0;
		hako->gps.quality = 0;
		hako->gps.satellites = 0;
		hako->gps.dop = 0.0;
		hako->gps.time_h = 0;
		hako->gps.time_m = 0;
		hako->gps.time_s = 0;
		hako->gps.time_cs = 0;
		hako->gps.date_d = 0;
		hako->gps.date_m = 0;
		hako->gps.date_y = 0;
		hako->gps.northing = 0.0;
		hako->gps.easting = 0.0;
		hako->gps.height = 0.0;
		hako->gps.height2 = 0;
}
