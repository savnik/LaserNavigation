/** \mainpage
 * This documentation is for the UTMgpsd client side library. 
 * The library provides an easy communication interface
 * to the UTMgpsd server.
 * 
 * Look in demo.c for an introduction on how to use the library.
 * 
 * \author Lars Valdemar Mogensen
 * \date 12-06-2006
 * \version 1.0
 */  
    
#include "libgps.h"

/** \file libgps.c
 * \brief Interface library to the UTMgpsd daemon.
 * 
 * It contains all definitions used for the socket server like
 * buffer sizes and protocol.
 * Structs and external functions are also defined here.
 * 
 * \author Lars Valdemar Mogensen
 * \date 23/06-2006
 */  

/** \brief Hardware server connect function
 * 
 * Connects to the hardware server using a streaming socket connection.
 * \param[in] *gps Pointer to the GPS struct to be connected.
 * \return 1 on succes and 0 on faliure.
 */
int gps_socket_connect(gpsmousetype * gps) 
{
   struct sockaddr_in serv_addr;
   struct hostent *host_param;
   gps->client.error = 0;
   
   if ((gps->client.sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      
      fprintf(stderr, "Can not create socket\n");
      gps->client.error = 1;
      return 0;
   }
   
   if ((host_param = gethostbyname(gps->client.hostname)) == NULL)
   {
      fprintf(stderr, "Can not connect to %s\n", gps->client.hostname);
      gps->client.error = 2;
      return 0;
   }
   
   bzero((char *) &serv_addr, sizeof(serv_addr));
   serv_addr.sin_family = AF_INET;
   bcopy((char *) host_param->h_addr, (char *) &serv_addr.sin_addr.s_addr,
   host_param->h_length);
   serv_addr.sin_port = htons(gps->client.port);
   
   if (connect(gps->client.sockfd, (struct sockaddr *) &serv_addr,
          sizeof(serv_addr)) != 0) {
      
      fprintf(stderr,
             "Error while trying to connect to socket on: %s, port: %d\n",
             gps->client.hostname, gps->client.port);
      gps->client.error = 3;
      return 0;
   } 
   {
      int x = 1;
      
      if (setsockopt
            (gps->client.sockfd, SOL_TCP, TCP_NODELAY, &x, sizeof(x))) {
         
         fprintf(stderr, "Error setting Socket options\n");
         gps->client.error = 4;
      }
   }
   
   if (fcntl(gps->client.sockfd, F_SETFL, O_NONBLOCK) == -1) {
      
      fprintf(stderr,
               "startclient: Unable to set flag O_NONBLOCK on gps->client.sockfd\n");
//              fprintf(stderr,"Error: %d (%s)\n",errno,strerror(errno));
   }
   
   return 1;
}

/** \brief Hardware server disconnect function
 * 
 * Disconnects the GPS from the hardware server.
 * \param[in] *gps Pointer to GPS struct to be disconnected.
 * \return 1 in all situations.
 */
int gps_socket_disconnect(gpsmousetype * gps) 
{
   close(gps->client.sockfd);
   gps->client.error = 0;
   return 1;
}

/** \brief GPS read function
 * 
 * Polls the hardware server, receives and parses the replies.
 * 
 * \param[in] *gps Pointer to the struct where recieved data is stored.
 * \param[in] msg Type of data to be read [POLL_GPS/POLL_GPS_STATUS]
 */
int gps_read(gpsmousetype * gps, int msg) 
{
   int status = 0;
   
   //Poll requests, with zero data and empty char pointer
   if (msg == POLL_GPS)
      gps_send_message(POLL_GPS, 0, NULL, &gps->client);       //polling GPS UTM
   else if (msg == POLL_GPS_STATUS)
      gps_send_message(POLL_GPS_STATUS, 0, NULL, &gps->client);        //polling GPS Status
   
   gps_send_message(M_EOL, 0, NULL, &gps->client);     //Ending communication
   //      printf("Sending POLL request\n");
   
   //Recieve data until end of frame
   do {
      
      status = gps_recv_message(&gps->client);
      
      if (status >= 0) {
         
         //Parse the message
         status &= gps_parse_msg(gps);
         
         if ((gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x01) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x02) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x03) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x04) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x05) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x06) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x07) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x08) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x09) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x0A) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x0B) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x0C) && 
             (gps->client.in_msg[0] != M_GPS || gps->client.in_msg[1] != 0x0D) && 
             (gps->client.in_msg[0] != 0x0F  || gps->client.in_msg[1] != 0x0E)) {
            
            fprintf(stderr, "Unexpected communication\n");
         }
      }
      else {
         
         fprintf(stderr,
                 "Error in gps_read - Timeout waiting for communication\n");
      }
   } while (gps->client.in_msg[0] != 0x0F || gps->client.in_msg[1] != 0x0E);
   
   return status;
}

/** \brief GPS write function
 * 
 * Transmits commands to the hardware server.
 * \param[in] *gps Pointer to the GPS struct to get the data from
 * \return 0 in all situations.
 */
int gps_write(gpsmousetype * gps) 
{
  //EGNOS eanbling /disabling feature
   gps_send_message(SET_GPS_STATE, 0, NULL, &gps->client);  //polling GPS Status
   gps_send_message(M_EOL, 0, NULL, &gps->client);      //Ending communication
   
   return 0;
}

/** \brief Send one message to the hardware server
 * 
 * \param[in] id Identification mark of the data to send.
 * \param[in] len Length of the data field to send.
 * \param[in] *data Pointer to array with the data to send.
 * \param[in] *client Pointer to the struct holding the client data.
 * \return Bytes send by the function.
 */
int gps_send_message(int id, int len, char *data, gpsclienttype * client) 
{
   char to_send[GPS_SOCKET_MSG_LEN + 2] = { 0 };
   int i;
   int status = 1;
   
   union {
      short a;
      char c[2];
   } u;
   
   u.a = id;
   
   to_send[0] = u.c[1];
   to_send[1] = u.c[0];
   to_send[2] = (char) (0x00FF & len);
   to_send[11] = '\r';
   to_send[12] = '\n';
   
   for (i = 3; i < GPS_SOCKET_MAX_DATA_LEN + 3; i++) {
      
      if ((i - 3) < len)
         to_send[i] = data[i - 3];
      else
         to_send[i] = 0;
   }
   
   status = send(client->sockfd, to_send, GPS_SOCKET_MSG_LEN + 2, 0);
   
   return status;
}

/** \brief Receive one message from the hardware server
 * 
 * \param[in] *client Pointer to the struct holding the client data
 * to recieve from
 */
int gps_recv_message(gpsclienttype * client) 
{
   double t1, t2;
   struct timeval tp;
   int j = GPS_SOCKET_MSG_LEN + 2;
   char buf_tmp[GPS_SOCKET_MSG_LEN + 2] = { '\0' };
   char *buf = buf_tmp;
   int n = 0;
   //      int i = 0;
   int status = -1;
   bzero(buf, GPS_SOCKET_MSG_LEN + 2);
   
   //Get time to calculate timeout 
   gettimeofday(&tp, NULL);
   t1 = (double) tp.tv_sec + (1.e-6) * tp.tv_usec;

   //Until full message has been received
   while (j) {
      
      n = recv(client->sockfd, buf, GPS_SOCKET_MSG_LEN + 2, 0);
      if (n <= 0) {
         n = 0;
         usleep(10);
      }
      else {
         buf += n;
         j -= n;
           
         if (status == -1 && buf_tmp[10] == '\r'
                 && buf_tmp[11] == '\n') {
            j = 0;
         }
         else if (j <= 0 && buf_tmp[11] != '\r' && buf_tmp[12] != '\n') { //if not in sync
            j = 3;
            buf = buf_tmp + 10;
            status = -1;
            fprintf(stderr, "Message out of sync RM\n");
         }
      }
      
      gettimeofday(&tp, NULL);
      t2 = (double) tp.tv_sec + (1.e-6) * tp.tv_usec;
      
      //Check for message timeout.
      if (t2 - t1 > GPS_RXTIMEOUT) {
         
         j = 0;
         status = -1;
         fprintf(stderr, "Timeout RM - %lf\n", t2 - t1);
      }
      
   }
   
    //If no timeout, accept message
   if (t2 - t1 < GPS_RXTIMEOUT) {
      
      status = 1;
      bcopy(buf_tmp, client->in_msg, GPS_SOCKET_MSG_LEN + 2);
   }
   
   return status;
}

///parsing socket messages into data struct
int gps_parse_msg(gpsmousetype * gps) 
{
   int i;
   union {
      short a;
      char c[2];
   } twobyte;
   union {
      double a;
      char c[8];
   } u_double;
   
   twobyte.c[0] = gps->client.in_msg[1];
   twobyte.c[1] = gps->client.in_msg[0];
   
   switch (twobyte.a) {
      case M_ERROR:         
         twobyte.c[0] = gps->client.in_msg[3];
         twobyte.c[1] = gps->client.in_msg[4];
         gps->state[0] = twobyte.a;
         twobyte.c[0] = gps->client.in_msg[5];
         twobyte.c[1] = gps->client.in_msg[6];
         gps->state[1] = twobyte.a;
         twobyte.c[0] = gps->client.in_msg[7];
         twobyte.c[1] = gps->client.in_msg[8];
         gps->state[2] = twobyte.a;
         twobyte.c[0] = gps->client.in_msg[9];
         twobyte.c[1] = gps->client.in_msg[10];
         gps->state[3] = twobyte.a;
         break;
      
      case M_GPS_TIMEDATE:
         gps->UTM.time_h = gps->client.in_msg[3];
         gps->UTM.time_m = gps->client.in_msg[4];
         gps->UTM.time_s = gps->client.in_msg[5];
         gps->UTM.time_cs = gps->client.in_msg[6];
         gps->UTM.date_m = gps->client.in_msg[7];
         gps->UTM.date_d = gps->client.in_msg[8];
         gps->UTM.date_y = gps->client.in_msg[9];
         break;
   
      case M_GPS_NORTHING:
         u_double.c[0] = gps->client.in_msg[3];
         u_double.c[1] = gps->client.in_msg[4];
         u_double.c[2] = gps->client.in_msg[5];
         u_double.c[3] = gps->client.in_msg[6];
         u_double.c[4] = gps->client.in_msg[7];
         u_double.c[5] = gps->client.in_msg[8];
         u_double.c[6] = gps->client.in_msg[9];
         u_double.c[7] = gps->client.in_msg[10];
         gps->UTM.northing = u_double.a;
         break;
   
      case M_GPS_EASTING:
         u_double.c[0] = gps->client.in_msg[3];
         u_double.c[1] = gps->client.in_msg[4];
         u_double.c[2] = gps->client.in_msg[5];
         u_double.c[3] = gps->client.in_msg[6];
         u_double.c[4] = gps->client.in_msg[7];
         u_double.c[5] = gps->client.in_msg[8];
         u_double.c[6] = gps->client.in_msg[9];
         u_double.c[7] = gps->client.in_msg[10];
         gps->UTM.easting = u_double.a;
         break;
   
      case M_GPS_HEIGHT:
         u_double.c[0] = gps->client.in_msg[3];
         u_double.c[1] = gps->client.in_msg[4];
         u_double.c[2] = gps->client.in_msg[5];
         u_double.c[3] = gps->client.in_msg[6];
         u_double.c[4] = gps->client.in_msg[7];
         u_double.c[5] = gps->client.in_msg[8];
         u_double.c[6] = gps->client.in_msg[9];
         u_double.c[7] = gps->client.in_msg[10];
         gps->UTM.height = u_double.a;
         break;
   
      case M_GPS_QUALITY:
         gps->UTM.quality = gps->client.in_msg[3];
         gps->UTM.satellites = gps->client.in_msg[4];
         gps->UTM.dop = (double) gps->client.in_msg[5] / 10.0;
         gps->UTM.height2 = gps->client.in_msg[6];
         gps->UTM.valid = gps->client.in_msg[7];
         break;
   
      case M_GPS_STATUS:
         gps->stat.opr_mode = (char) gps->client.in_msg[3];
         gps->stat.sats_in_view = gps->client.in_msg[4];
         gps->stat.EGNOS = gps->client.in_msg[5];
         gps->stat.zone = gps->client.in_msg[6];
         gps->stat.mode = gps->client.in_msg[7];
         break;
   
      case M_GPS_PDOP:
         u_double.c[0] = gps->client.in_msg[3];
         u_double.c[1] = gps->client.in_msg[4];
         u_double.c[2] = gps->client.in_msg[5];
         u_double.c[3] = gps->client.in_msg[6];
         u_double.c[4] = gps->client.in_msg[7];
         u_double.c[5] = gps->client.in_msg[8];
         u_double.c[6] = gps->client.in_msg[9];
         u_double.c[7] = gps->client.in_msg[10];
         gps->stat.PDOP = u_double.a;
         break;
      
      case M_GPS_HDOP:
         u_double.c[0] = gps->client.in_msg[3];
         u_double.c[1] = gps->client.in_msg[4];
         u_double.c[2] = gps->client.in_msg[5];
         u_double.c[3] = gps->client.in_msg[6];
         u_double.c[4] = gps->client.in_msg[7];
         u_double.c[5] = gps->client.in_msg[8];
         u_double.c[6] = gps->client.in_msg[9];
         u_double.c[7] = gps->client.in_msg[10];
         gps->stat.HDOP = u_double.a;
         break;
      
      case M_GPS_VDOP:
         u_double.c[0] = gps->client.in_msg[3];
         u_double.c[1] = gps->client.in_msg[4];
         u_double.c[2] = gps->client.in_msg[5];
         u_double.c[3] = gps->client.in_msg[6];
         u_double.c[4] = gps->client.in_msg[7];
         u_double.c[5] = gps->client.in_msg[8];
         u_double.c[6] = gps->client.in_msg[9];
         u_double.c[7] = gps->client.in_msg[10];
         gps->stat.VDOP = u_double.a;
         break;
   
      case M_GPS_FOM:
         u_double.c[0] = gps->client.in_msg[3];
         u_double.c[1] = gps->client.in_msg[4];
         u_double.c[2] = gps->client.in_msg[5];
         u_double.c[3] = gps->client.in_msg[6];
         u_double.c[4] = gps->client.in_msg[7];
         u_double.c[5] = gps->client.in_msg[8];
         u_double.c[6] = gps->client.in_msg[9];
         u_double.c[7] = gps->client.in_msg[10];
         gps->stat.FOM = u_double.a;
         break;

      case M_GPS_SU1of3:
         gps->stat.sat_used[0] = gps->client.in_msg[3];
         gps->stat.sat_used[1] = gps->client.in_msg[4];
         gps->stat.sat_used[2] = gps->client.in_msg[5];
         gps->stat.sat_used[3] = gps->client.in_msg[6];
         gps->stat.sat_used[4] = gps->client.in_msg[7];
         gps->stat.sat_used[5] = gps->client.in_msg[8];
         gps->stat.sat_used[6] = gps->client.in_msg[9];
         gps->stat.sat_used[7] = gps->client.in_msg[10];
         break;
   
      case M_GPS_SU2of3:
         gps->stat.sat_used[8] = gps->client.in_msg[3];
         gps->stat.sat_used[9] = gps->client.in_msg[4];
         gps->stat.sat_used[10] = gps->client.in_msg[5];
         gps->stat.sat_used[11] = gps->client.in_msg[6];
         gps->stat.sat_used[12] = gps->client.in_msg[7];
         gps->stat.sat_used[13] = gps->client.in_msg[8];
         gps->stat.sat_used[14] = gps->client.in_msg[9];
         gps->stat.sat_used[15] = gps->client.in_msg[10];
         break;
   
      case M_GPS_SU3of3:
         gps->stat.sat_used[16] = gps->client.in_msg[3];
         gps->stat.sat_used[17] = gps->client.in_msg[4];
         gps->stat.sat_used[18] = gps->client.in_msg[5];
         gps->stat.sat_used[19] = gps->client.in_msg[6];
         break;
   
      case M_EOL:
         break;
   
      default:
         printf("Parsing failed ");
         for (i = 0; i < GPS_SOCKET_MSG_LEN + 2; i++)
            printf("%2x ", gps->client.in_msg[i]);
         printf("\n");
         break;
   }
   
   return 1;
}

/** \brief Clear the internal structs "UTM" and "status" of the "gps" struct
 * 
 * Used to clear the struct before filling it with new data from the GPS.
 * 
 * \param[in] *gps Pointer to the struct to be cleared.
 */
void gps_clear(gpsmousetype * gps) 
{
   int i;
   
   for (i = 0; i < 4; i++)
      gps->state[i] = 0;
   
   //Interanal GPS struct
   gps->UTM.valid = 0;
   gps->UTM.quality = 0;
   gps->UTM.satellites = 0;
   gps->UTM.dop = 0.0;
   gps->UTM.time_h = 0;
   gps->UTM.time_m = 0;
   gps->UTM.time_s = 0;
   gps->UTM.time_cs = 0;
   gps->UTM.date_d = 0;
   gps->UTM.date_m = 0;
   gps->UTM.date_y = 0;
   gps->UTM.northing = 0.0;
   gps->UTM.easting = 0.0;
   gps->UTM.height = 0.0;
   gps->UTM.height2 = 0;
   
   //Internal status struct
   gps->stat.opr_mode = '\0';
   gps->stat.mode = 0;
   
   for (i = 0; i < GPS_SATELLITES_SUPPORTED; i++)
      gps->stat.sat_used[i] = 0;       //Satellites used to make calculation
   for (i = 0; i < GPS_SATELLITES_TOTAL; i++) {
      gps->stat.sat_visible[i][0] = 0; //Number
      gps->stat.sat_visible[i][1] = 0;  //Elevation
      gps->stat.sat_visible[i][2] = 0;  //Azimuth
      gps->stat.sat_visible[i][3] = 0;  //Signal to Noise Ratio
   }
   
   gps->stat.PDOP = 0.0;        //?
   gps->stat.HDOP = 0.0;        //Horizontal Dillution of Precision
   gps->stat.VDOP = 0.0;        //Vertical Dillution of precision
   gps->stat.sats_in_view = 0;  //Satellites in view
   gps->stat.FOM = 0.0;         //Precision of the soultion
   
   gps->stat.EGNOS = 0;         //Not used yet, but for augmentation purposes
   gps->stat.zone = 0;          //UTM zone used to specify the base for the coordinate system
}

/** \brief Clear the internal structs "UTM" and "status" of the "gps" 
 * struct and the extra EGNOS variable
 * 
 * Used to initialize the struct clearing the data from the GPS and
 * the flag indicationg if EGNOS (SBAS) should be enabled.
 * \param[in] *gps Pointer to the GPS struct to be cleared.
 */
void gps_init(gpsmousetype * gps) 
{
   //Interanal GPS struct
   gps_clear(gps);   

   //Clear the EGNOS status flag
   gps->EGNOS = 0;
} 
