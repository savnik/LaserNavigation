/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef USOCKCLIENT_H
#define USOCKCLIENT_H

#include <pthread.h>
#include <string.h>
#include <sys/poll.h>

// default string length
#define MAX_SC_INFO_SIZE 256
// receive buffer for TCP/IP client
// may well be less than max message length
//#define MAX_BUFFER_LENGTH_RX 512
#define MAX_BUFFER_LENGTH_RX 20000
// max string length for host name (IP number)
#define MAX_HOST_LENGTH 100

#define scInfo    0
#define scWarning 1
#define scError   2
#define scDebug   3

/**Socket client class.
  * @todo deprecated -- use UClientPort - (but pt used by USmrCl)
  * @author (Jens) Christian Andersen,326/030,3548,
  */

class USockClient
{
public:
  /**
  Constructor */
  USockClient();
  /**
  Destructor */
  virtual ~USockClient();
  /**
  Connect to host */
  bool tryConnect(bool andStartReceiveThread = true);
  /**
  Disconnect from server and stop read thread.
  Use closeConnection() to just close socket. */
  bool doDisconnect();
  /**
  Shutdown socket connection, but leave read thread running
  (in a wait loop).
  Use doDisconnect() to also terminate thread. */
  void closeConnection();
  /**
  Is calle when commection is lost, to enable
  of disable functions at higher levels. */
  virtual void connectionLost();
  /**
  Set socket port number */
  inline int setPort(int iPort) { return port = iPort;};
  /**
  Set socket port number */
  inline int getPort() { return port;};
  /**
  Set socket port number */
  int setHost(const char * iHost)
  {
    strncpy(host, iHost, MAX_HOST_LENGTH);
    return strlen(host);
  };
  /**
  Set socket port number */
  inline char * getHost() { return host;};

  /**
  Is connected to server */
  bool isConnected() { return connected;};
  /**
  Send this buffer of data */
  bool blockSend(const char * buffer, int length);
  /**
  Handle connection - i.e. send buffers, when something to send
  and display received messages. */
  void handleConnection(bool terminateOnDisconnect = false);
  /**
  Poll socket and read data is souch is available.
  Returns true if data is received. */
  bool getDataFromLine(int pollTimeoutMs);
  /**
  Process message
  This routine should be overwritten by something more meaningfull
  than just printing to log widget. */
  virtual void processMessage(unsigned char * message, int length);

  /**
  Called when something is to be displayed or logged.
  if not handled by virtual function, then
  messages are just printed to console. */
  virtual void sInfo(const char * message, int type);
  /**
  Is called if transmission starts or ends.
  Corresponding to turn on or off transmission lamp.
  is called when receiving or transmission is
  started or stopped. */
  virtual void transmissionTraffic(bool traffic);
  /**
  This poll idle function is called at poll timeout
  times (everty 'pollTimeout' ms), and can
  be used to send new commands to the socket, if
  this is not done by main thread (not both!). */
  virtual void pollTimeout();

protected:
  /**
  Name of host to connect to */
  char host[MAX_HOST_LENGTH];
  /**
  Port used when connecting to host */
  int port;
  /**
  Number of reads (with >0 bytes returned). */
  unsigned int readCnt;
  /**
  Poll read timeout value.
  At poll timeout the pollTimeout() is called.
  value is in milli seconds */
  int pollTimeoutMs;

private:
  /**
  Connected is true when connected */
  bool connected;
  /**
  Listening is true when read thread is running - listening to the socket */
  bool recvThreadRunning;
  /**
  True during a send */
  bool sending;
  /**
  True as long as there is unfetched data */
  bool receiving;
  /**
  Flag to stop receiving thread. */
  bool stop;
  /**
  Socket */
  int sock; // socket
  /**
  Handle to thread listening to messages */
  pthread_t  thClient;
  /**
  Mutex lock */
  pthread_mutex_t ptLock;
  /**
  Another stop flag that can stop an connection attempt - I think. */
  bool stopNow;
  /**
  Buffer fused by recv(...) call for new partial message */
  unsigned char buf[MAX_BUFFER_LENGTH_RX]; // rx buffer
  /**
  Poll status used when polling for available data */
  struct pollfd recvPollStatus;
  /**
  Detection of lost connection */
  bool connectionGotLost;
};

#endif
