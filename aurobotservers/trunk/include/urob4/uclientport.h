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
#ifndef UCLIENTPORT_H
#define UCLIENTPORT_H

#include <pthread.h>
#include <sys/poll.h>

#include <ugen4/usmltagin.h>
#include <ugen4/ulock.h>

#include "usmlsource.h"
#include "ulogfile.h"

class USmlTag;

// max string length for host name (IP number)
#define MAX_HOST_LENGTH 100
/**
Longest tag send to client, i.e. from '<' to '>'
(both included), but a longer buffer may be an advantage
for skipping large binary blocks. */
//#define MAX_CLIENT_RX_BUFFER 10000

/**
Base class for a socket client

@author Christian Andersen
*/
class UClientPort : public ULock
{
public:
  /**
  Constructor */
  UClientPort();
  /**
  Destructor */
  virtual ~UClientPort();
  /**
  Set verbose messages - mostly for debug purpose */
  inline void setVerbose(bool value)
    { verboseMessages = value;};
  /**
  is verbose messages true - mostly for debug purpose */
  inline bool isVerbose()
  { return verboseMessages;};
  /**
  Get host name */
  inline const char * getHost()
    { return host; };
  /**
  Set host name */
  inline void setHost(const char * toHost)
  { strncpy(host, toHost, MAX_HOST_LENGTH); };
  /**
  Get host IP adress (as string) */
  inline const char * getHostIP()
  { return hostIP; };
  /**
  Get port number */
  inline int getPort()
    { return port; };
  /**
  Set port number */
  inline void setPort(int toPort)
  { port = toPort; };
  /**
  Is client connected */
  inline bool isConnected()
    { return connected; };
  /**
  Do an hang-up if socket is valid
  and call the connectionChange function. */
  void closeConnection();
  /**
  Send data to the server
  Returns false if not send (or partially send).
  If an error occurs during transmission, the connection
  will be closed.
  The line is TX-locked during transmission.
  The call is blocked until data is send (no timeout). */
  bool blockSend(const char * buffer, int length);
  /**
  Send a message to the server assuming that the connection is already locked */
  bool sendMsg(const char * message);
  /**
  Send a message to the server keeping the connection locked during transmission */
  bool sendWithLock(const char * message);

public:
  /**
  Try connect to this host-port combination.
  Returns true if connected.
  Host IP address will be in host string on successfull connect. */
  bool tryConnect();
  /**
  Get more data for line
  Gets at most buffer length of data.
  Returns number of bytes received.
  Returns 0 if timeout occured.
  Returns -1 if line got disconnected or other error situations. */
  int getDataFromLine(char * buffer, int bufferSize, int pollTimeoutMs);
protected:
  /**
  Called when connection is created or gets lost */
  virtual void connectionChange(bool nowConnected);
  /**
  Virtual function that can be used to send an opening remark to the server */
  virtual void sendNamespaceOpenTag();

public:
  /**
  Mutex lock for transmission - should be used by clients
  where conflicts might occur to frame set of transmissions.
  - not used by this class! */
  ULock tx;
  /**
  Number of bytes received drom port. */
  int rxByteCnt;

protected:
  /**
  Name of host to connect to */
  char host[MAX_HOST_LENGTH];
  /**
  Name of host to connect to */
  char hostIP[MAX_HOST_LENGTH];
  /**
  Port used when connecting to host */
  int port;
  /**
  Number of reads (polls). */
  unsigned int readCnt;
  /**
  Print pore information when relevant */
  bool verboseMessages;
private:
  /**
  Connected is true when connected */
  bool connected;
  /**
  Socket */
  int sock; // socket
  /**
  Poll status used when polling for available data */
  struct pollfd recvPollStatus;
  /**
   * debug logfile */
  //ULogFile debugLog;
};

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
class that extends the client post functions with a receive buffer and
functions to handle simple SML formatted tags.
*/
class UClientPortSml : public UClientPort, public USmlSource
{
public:
  /**
  Constructor */
  UClientPortSml();
  /**
  * Got an unusable message - just skip to other end.
  * 'tag' should point to the start tag, of which the content is unwanted.
  * timeout is in miliseconds, when waiting for new data.
  * Returns true when relevant end-tag is removed from the stream or false when a
    period of 'msTimeout' has passed with no data.*/
//   virtual bool skipToEndTag(USmlTagIn * tag, int msTimeout);
//   /**
//   Skip N bytes (from a binary message).
//   Returns true if n bytes skipped.
//   Returns false if timeout or source is down. */
//   virtual bool skipNBytes(int n,  int msTimeout);
//   /**
//   Get the next n bytes loaded into the provided buffer.
//   N may be of any length (that can be received within the timeout
//   period).
//   Returns the number of bytes loaded into the buffer.
//   If source is invalid or no data is available zero is returned. */
//   virtual int getNBytes(char * buffer, int n, int msTimeout);
//   /**
//   Read the next available tag into the provided buffer.
//   The found tag is loaded into the 'tag' buffer.
//   if 'beforeTagBuffer' != NULL then the data before the
//   next tag is stored in this buffer. The buffer size is
//   initiated in 'beforeTagCnt'. The number of bytes befor the next tag is
//   also returned into 'beforeTagCnt'.
//   If there is more data before the next tag than the space in 'beforeTagBuffer' then
//   the function returns with 'beforeTagBuffer' equal to the buffer size and
//   the function returnes true.
//   Note the buffer is NOT nul-terminated (to allow binary data), so the user
//   must add a nul at the 'beforeTagCnt' position if the result is used as a string.
//   The search is also stopped if an end-tag is found matching the 'failEndTag'
//   (makes the function return false).
//   Returns true if tag is valid (and the beforeTagBuffer has no overflow). */
//   virtual bool getNextTag(USmlTag * tag, int msTimeout,
//                   USmlTagIn * failEndTag = NULL,
//           char * beforeTagBuffer = NULL,
//           int * beforeTagCnt = NULL);
  /**
   * Get source name pointer for the SML source */
  virtual const char * getSourceName()
  { return getHost(); };

protected:

    /**
    * Get more data, if more data is available.
    * \param buffer, put the new data in the buffer
    * \param bufferSize, but not more that this number of bytes
    * \param wait at maximum this number of ms for more data
    * \return the number of bytes available in buffer,
    * or -1 if source is no longer available.    */
  virtual int getMoreData(char * buffer, int bufferSize, int pollTimeoutMs);
  /**
    * Should additional messaves be printed to console */
  virtual bool doVerboseMessages();

  /**
  Discard all data in buffer -- assumed garbage, e.g server reply error */
 // void clearRxBuffer();
  /**
   * Is tha data source (still) open.
   * \returns true if open */
  virtual bool isSourceAvailable();
  /**
   * Output a reply to the source (if possible, e.g. a socket connection)
   * \param message is 0-terminated
   * \returns true if send. */
  virtual bool outputData(const char * message);

public:
  /**
  Pointer to interface name - to be used in data source tracking (mostly for debug and status) */
  //const char * ifName;

protected:
  /**
  Valid data in data buffer - relative to buffer start */
  //int dataCnt;
  /**
  Flag for receptionof the first tag */
  //bool gotFirstTag;

private:
  /**
  Pointer to next unused data */
  //const char * dataNext;
  /**
  Buffer for new data from socket,
  plus space for a zero when searching for '<' and '>' */
  //char data[MAX_CLIENT_RX_BUFFER + 1];
};


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////




#endif
