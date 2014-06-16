/** \file libgps.h
 * \brief Header file for the interface library to the UTMgpsd daemon.
 * 
 * It contains all definitions used for the socket server like
 * buffer sizes and protocol.
 * Structs and external functions are also defined here.
 * 
 * The UTMgpsd server must be running for libgps to connect.
 * 
 * \author Lars Valdemar Mogensen
 * \date 04/13-2006
 */  
    
#ifndef LIBGPS_H
#define LIBGPS_H
    
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
    

// Definitions for the socket connection
/// Socket port for the client to connect to
#define GPS_SOCKET_PORT 9500
/// Host name of the computer where the UTMgpsd server is running
#define GPS_SOCKET_HOSTNAME "localhost"
/// Time limit for the reception of the data packets
#define GPS_RXTIMEOUT 3
/// Length of the messages sent over the socket connection, excluding 
/// terminating characters \\r\\n
#define GPS_SOCKET_MSG_LEN 11
/// Length of the data field in the socket messages
#define GPS_SOCKET_MAX_DATA_LEN 8
    
// Protocol definitions
/// Poll request for the GPS data
#define POLL_GPS		0x0310
/// Poll request for the GPS status
#define POLL_GPS_STATUS		0x0320
/// Set command, here used only to toggle the EGNOS state
#define SET_GPS_STATE		0x0330
    
/// Poll reply indicating that this is a GPS data package
#define M_GPS 			0x03
/// Poll reply indicating that there is an error
#define M_ERROR 		0x0050
/// Poll reply containing the latest time and date
#define M_GPS_TIMEDATE		0x0301
/// Poll reply containing the northing coordinate
#define M_GPS_NORTHING		0x0302
/// Poll reply containing the easting coordinate
#define M_GPS_EASTING		0x0303
/// Poll reply containing height1
#define M_GPS_HEIGHT		0x0304
/// Poll reply containing simple GPS status indicators and height2
#define M_GPS_QUALITY		0x0305
    
/// Poll reply containing advanced GPS status
#define M_GPS_STATUS		0x0306
/// Poll reply containing the Positional DOP (3D)
#define M_GPS_PDOP		0x0307
/// Poll reply containing the Horizontal DOP (On a plane)
#define M_GPS_HDOP		0x0308
/// Poll reply containing the Vertical DOP (Height only)
#define M_GPS_VDOP		0x0309
/// Poll reply containing the Figure of Merit (Not used)
#define M_GPS_FOM		0x030A
/// Poll reply containing the Satellites used for the fix (1 of 3)
#define M_GPS_SU1of3		0x030B
/// Poll reply containing the Satellites used for the fix (2 of 3)
#define M_GPS_SU2of3		0x030C
/// Poll reply containing the Satellites used for the fix (3 of 3)
#define M_GPS_SU3of3		0x030D
    
/// Poll reply indicating end of transmission
#define M_EOL			0x0F0E
    
// Constants specific for the GPS system
/// Maximum number of supported satellites (Not tested)
#define GPS_SATELLITES_SUPPORTED 20
/// There are currently 30 GPS satellites in service so
/// this is to be sure
#define GPS_SATELLITES_TOTAL 32
    
// Constants specific for the program
/// Definition of the maximum string length that can be revieved
/// This is sufficient for the GPS messages and a little more
#define STRING_LENGTH 100
    
// Forward declaration or prototype
    
/// Client type struct for use with the socket connection
typedef struct 
{
   /// User input: Host name
   char *hostname;
   /// User input: port number
   int port;
   /// Internal variables: incomming messages
   unsigned char in_msg[GPS_SOCKET_MSG_LEN + 2];
   /// Internal variables: outgoing messages
   unsigned char out_msg[GPS_SOCKET_MSG_LEN + 2];
   
   /// Error value: Returns a number different from 0 if an error occurs
   int error;
   /// Output value
   int sockfd;

} gpsclienttype;

/// GPS status struct to hold the advanced status information about the GPS system
typedef struct 
{
   /// Operational mode
   char opr_mode;
   /// Calculation mode
   int mode;
   /// Number of the satellites used to make calculation (GID)
   int sat_used[GPS_SATELLITES_SUPPORTED];
   /// Visible satellites with information: Number (GID), elevation (0-90 deg), 
   /// azimuth (0-360 deg), Signal to Noise Ratio (1-99 db-Hz)
   int sat_visible[GPS_SATELLITES_TOTAL][4];
   /// Positional Dillution of Precision (3D)
   double PDOP;
   /// Horizontal Dillution of Precision (In a plane)
   double HDOP;
   /// Vertical Dillution of precision (Height only)
   double VDOP;
   /// Number of satellites in view
   int sats_in_view;
   /// Figure of Merit, Precision of the soultion (Not used)
   double FOM;
   
   /// Show if the GPS is in SBAS augmentation mode (EGNOS or WAAS)
   int EGNOS;
   /// UTM zone used in calculation
   int zone;

} GPS_status;

/// GPS UTM struct to hold the GPS fix information in the Universal Transversal Metacore system
typedef struct 
 {
    /// Show if the fix is valid
   int valid;
   /// Show the quality of the fix (No gps/gps/dgps)
   int quality;
   /// Show the nuber of satellites used in the fix
   int satellites;
   /// Dillution of Precision for the fix
   double dop;
   /// GMT hours
   int time_h;
   /// GMT minutes
   int time_m;
   /// GMT seconds
   int time_s;
   /// GMT centiseconds
   int time_cs;
   /// GMT day
   int date_d;
   /// GMT month
   int date_m;
   /// GMT year
   int date_y;
   /// Northing coordinate UTM (zone 32)
   double northing;
   /// Easting coordinate UTM (zone 32)
   double easting;
   /// Antenna height data
   double height;
   /// Not used
   int height2;

} UTM_gpstype;

/// Struct to hold all information used for communication and storage of data
typedef struct 
{
   /// Internal variable for the socket server
   unsigned short state[4];
   /// Struct to hold the socket client data
   gpsclienttype client;
   /// Struct to hold the GPS UTM data
   UTM_gpstype UTM;
   /// Internal variable for the socket server
   FILE * conf;
   /// Struct to hold the advanced information about the GPS system
   GPS_status stat;
   
   /// Extra variable to indicate the SBAS (EGNOS/WAAS) state (Not used)
   int EGNOS;
   /// Host name of the server to connect to, usually localhost
   char hostname[STRING_LENGTH];
   /// Port number on the server to connect to
   int port;
   
   int config;    ///< Flag to indicate if the module is configured
   int status;    ///< Flag to indicate the status of the module
   int run;       ///< Flag to indicate if the module is to be updated
   /// Flag to indicate if the date returned from the module is to 
   /// be used in the following calculations.
   int use;       
   
} gpsmousetype;


void gps_init(gpsmousetype *);
void gps_clear(gpsmousetype *);
int gps_read(gpsmousetype *, int);
int gps_write(gpsmousetype *);

int gps_socket_connect(gpsmousetype *);
int gps_socket_disconnect(gpsmousetype *);

int gps_send_message(int id, int lenght, char *data,
                       gpsclienttype * client);
int gps_recv_message(gpsclienttype *);
int gps_parse_msg(gpsmousetype *);

#endif
