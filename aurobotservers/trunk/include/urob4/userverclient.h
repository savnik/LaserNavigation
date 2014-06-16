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
#ifndef USERVERCLIENT_H
#define USERVERCLIENT_H

#include <arpa/inet.h>

#include <ugen4/utime.h>

#include "userverqueue.h"
//#include "uresposehist.h"
#include "ureslink.h"
#include "ulogfile.h"

/**
Class of functions to handle one accepted connection for a socket server.

@author Christian Andersen
*/
class UServerClient
{
public:
  /**
  Constructor
  \param index is the client number of this client connection.
  \param parent port is a pointer to the port object where this client belong. */
  UServerClient(int index);
  /**
  Destructor */
  virtual  ~UServerClient();
  /**
  This is also the place to send any welcome messages.
  The virtual function is called before receive thread is activated.
  Must returns true if connection is acceptable. */
  virtual bool justConnected();
  /**
  returns true if connection is open. */
  bool isActive() {return connected;};
  /**
  Get connection stream handle - for use in poll, etc. */
  inline int getCnn() { return conn; };
  /**
  Send data to client.
  if connection to client is open the data is send.
  returns false if client do not exist or data could not be send
  within timeout period. */
  bool blockSend(const char * buffer, int length, int msTimeout);
  /**
  Set connection handle (clnt) and the struct with the client info - name etc. */
  bool initConnection(int clientNumber, int clnt, struct sockaddr_in from,
                      UServerInQueue * queue,
                     UTime * aliveTime);
  /**
  Get IP number of client as string. */
  char * getClientName();
  /**
  Stop connection and send a HUP (hangup) if
  'sendHUP' is true. */
  void stopConnection(bool sendHUP, const char * namespaceName);
  /**
  Data is available to receive, so
  receive the data and process as needed (i.e. put in queue) */
  bool receiveData();
  /**
  Print client status */
  void print(char * preStr);
  /**
  Lock communication to client.
  Returns false, if an error occured.
  Returns true when the client can be and is locked. */
  inline bool lock()
    { return (pthread_mutex_lock(&txlock) == 0); };
  /**
  Lock communication to client and return emmidiately.
  Returns false, if an error occured or the client is locked already.
  Returns true if the client is locked by the calling thread. */
  inline bool tryLock()
    { return (pthread_mutex_trylock(&txlock) == 0); };
  /**
  Unlock communication to client and return emmidiately. */
  inline void unlock()
    { pthread_mutex_unlock(&txlock); };
  /**
  Get namespace (last used namespace for client) .*/
  inline const char * getNamespace()
  { return clientNamespaceTag.getTagName(); };
  /**
  Get the the attribute part of the current namespace message from this client.*/
  inline const char * getNamespaceAttributes()
  { return clientNamespaceMsg; };
  /**
  Get namespace tag (last used namespace for client) .*/
  inline USmlTagIn * getNamespaceTag()
  { return &clientNamespaceTag; };
  /**
  Set ressource for fast responce messages */
  void setResLink(UResLink * resourceLinks);
  /**
  Ressource is updated - action may be needed */
  void resourceUpdated();
  /**
  Get logfile name */
  inline const char * getLogFilename()
  { return logCmd.getLogFileName();};
  /**
  Is logfile open? */
  inline bool isLogOpen()
  { return (logCmd.isOpen());};
  /**
  is logfile open for reply? */
  inline bool getLogReply()
  { return logReply;};
  /**
  is a timestamp to be added to logfile */
  inline bool getLogTimestamp()
  { return logTimestamp;};
  /**
  Set logging of reply messages. */
  inline void setLogReply(bool value)
  {
    logReply = value;
    printf("UServerClient:: logging all (%s) client %d to %s\n", bool2str(value), clientIndex, getLogFilename());
  };
  /**
  Set timestamping of logged items. */
  inline void setLogTimestamp(bool value)
  { logTimestamp = value;};
  /**
  Open a logfile dataPath with the application name with client-number added,
  e.g. /mnt/ram/userver_client_1.log. */
  bool logOpen();
  /**
  Close logfile.
  Resets any flags used (e.g. log also reply) */
  void logClose();
  /**
   * Get serial number for the connection */
  inline int getSerial()
  { return clientSerial; };
  /**
   * Get pointer to time of last received message for this client */
  const UTime * getTimeOfLastMessage()
  { return &msgBufTime; };
  /**
   * Send message to client with alive time for for the main thread in this server */
  void sendAliveReply();

protected:
  /**
  Called when data or additional data is received from
  socket port. message is in msgBuff with a length of msgBuffCnt. */
  void gotNewMessage();
  /**
  Search from 'fromPos' and at maximum 'size' characters
  for an matshing end tag (or a '\n') return 0 if not found. */
  int findEndTag(const char * fromPos, int size);
  /**
  Send request for streaming data.
  Typically used for request of robot pose. */
  void sendStreamRequest(const char * reqTyp);
  /**
  Connection is lost - if ant cleanup is needed */
  void connectionLost();
  /**
  Log this string to logfile.
  Returns false if no logfile is open.
  File must be opened by a logOpen(.) call, and
  closed with a logClose().
  Only the first 'dataCnt' characters are saved, or until a '\0' is found..*/
  bool logWrite(const char * data, const int dataCnt, bool isReply = false);
private:
  /**
  Send a ping reply directly from client handler */
  void sendPingReply(USmlTagIn * tag, int length);
  /**
  Trap high speed messages that needs immidiate
  action and should not be queued */
  bool trapMessage(int client, const char * msg, int size);

private:
  /**
  Is a client connected */
  bool connected;
  /** Structure with adress information etc. */
  struct sockaddr_in clientInfo; // address info for connection
  /** Connection handle */
  int conn;            // connection handle
  /**
  Message buffer for partially received messages */
  char msgBuff[MAX_MESSAGE_LENGTH_TO_CAM * 2 + 1];
  /**
  Already received part of message buffer */
  int msgBufCnt;
  /**
  Time of last received data,
  used to empty part of buffer if out of byte sync. */
  UTime msgBufTime;
  /**
  statistics - messages send to this client */
  unsigned int msgSend;
  /**
  statistics - Number of received TCP/IP packeges */
  unsigned int tcpipReceived;
  /**
  statistics - Number of received messages */
  unsigned int msgReceived;
  /**
  Statistics - skipped bytes in seach of byte-sync */
  unsigned int msgSkippedBytes;
  /**
  Index number of client - set at creation */
  int clientIndex;
  /**
  Queue, where to put received data */
  UServerInQueue * rxQueue;
  /** lock, when reading or writing.
  This lock is locked, when a function is initiated and
  unlocked after the function returns.
  So this lock should not be tested or used inside a function. */
  pthread_mutex_t txlock; // pthread_mutex_lock
  /**
  A pose history resource pointer - if module is loaded */
  UResLink * resLink;
  /**
   * Server alive time.
   * The last time the server thread was reported alive.
   */
  UTime * serverAlive;
  /**
   * debug log */
  //debug
  //ULogFile debugLog;
  // debug end

protected:
  /**
  Tag level 0 = no namespace, 1 = root level, 2 = normal instde namespace */
  int clientNamespaceLevel;
  /**
  The attribute string send after the namespace name */
  char clientNamespaceMsg[MAX_MESSAGE_LENGTH_TO_CAM];
  /**
  Tag with namespace */
  USmlTagIn clientNamespaceTag;
  /**
  Namespace close tag to look for */
  char clientNamespaceCloseTag[MAX_SML_NAME_LENGTH + 3];
  /**
  Length of the namespace close tag */
  int clientNamespaceCloseTagCnt;
  /**
  Logfile handle for logging of commands (and reply) */
  ULogFile logCmd;
  /**
  Should reply be logged too? */
  bool logReply;
  /**
  Should logging be timestamped. */
  bool logTimestamp;
  /**
  Lock on logfile.
  As tx and reply is executed in different threads a lock is needed */
  ULock logLock;
public:
    /**
     * queue the raw data - one entry per read */
  bool queueRawData;
  /**
  The client connection serial number - set at creation, and is unique for the session */
  int clientSerial;
  /**
   * connect time - set when connection was established */
  UTime connectTime;
  /**
  Time of last punk beeing send (to test if client connection is alive) */
  UTime punkTime;
};

#endif
