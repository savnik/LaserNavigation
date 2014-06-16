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
#ifndef USERVERPORT_H
#define USERVERPORT_H

#include <pthread.h>
#include <arpa/inet.h>

#include "userverclient.h"
#include "umsgqueue.h"
#include "ureslink.h"

/**
Number of clients beeing served at one time */
#define MAX_SOCKET_CLIENTS_SERVED    20

/**
Max length of attributes in namespace message */
#define MAX_NAMESPACE_ATTRIBUTE_LENGTH 200

/**
Class used by a socket server to control and accept connections to one port number.
The port number are determined during initialization.

@author Christian Andersen
*/
class UServerPort
{
public:
  /**
  Constructor */
    UServerPort();
  /**
  Destructor */
  virtual ~UServerPort();
  /**
  Get server port number */
  inline int getPort() { return serverPort; };
  /**
  Set server port number */
  inline void setPort(const int toPort)
  { serverPort = toPort; };
  /**
  Get hostname - just a wrap of the normal 'gethostname' call.
  Returns a pointer to the provided string. */
  char * getHostName(char * nameBuffer, const int nameBufferCnt);
  /**
  Get receive message queue */
  inline UServerInQueue * getRxQueue() { return &rxQueue; };
  /**
  Start server port reveive loop. This loop will not start if
  port number is less than 1000. One loop handles all clients and adds messages to the
  command queue. An error is printed on console if port is not opened. */
  bool start();
  /**
  Stops the socket server and terminates all connection
  and terminates the thread.
  If andWait, then stop will wait until thread is terminated before
  return, otherwise function returns immidiately */
  bool stop(bool andWait);
  /**
  Set verbose messages - mostly for debug purpose */
  inline void setVerbose(bool value) { verboseMessages = value;};
  /**
  The thread that handles all port tasks.
  Runs until stop-flag is set, or if port can not be bound.
  Should not be called!, call start() to create the thread
  calling this function. */
  bool runServerThread();
  /**
  Print server status to console,
  preseded by this string (preStr). */
  void print(const char * preStr);
  /**
  Get number of  active clients */
  int getActiveClientCnt();
  /**
  Return number of initialized clients.
  The clients may be active or inactive (closed) */
  inline int getClientCnt() { return clientCnt; };
  /**
  Get handle to connection - returns NULL if out of range */
  UServerClient * getClient(const int i);
  /**
  Function to trigger events, when new data is available.
  Should be overwritten by decendent classes. */
  virtual void messageReceived();
  /**
  This function is called, when a new client is connected */
  virtual void gotNewClient(UServerClient * cnn);
  /**
  Get a pointer to the namespace element name.*/
  inline const char * getServerNamespace()
  { return serverNamespace; };
  /**
  Set name of namespace, that will be used as the opening element
  on start of communication with a new client. */
  inline void setServerNamespace(const char * name)
  { strncpy(serverNamespace, name, MAX_SML_NAME_LENGTH); };
  /**
  Set use flag for opening namespace message to new clients. */
  inline void setServerNamespaceUse(bool value)
  { serverNamespaceUse = value; };
  /**
  Is the use flag for opening namespace message to new clients set. */
  inline bool isServerNamespaceUse()
  { return serverNamespaceUse; };
  /**
  Get a pointer to the namespace attribite string. */
  inline const char * getServerNamespaceAttribute()
  { return serverNamespaceAttribute; };
  /**
  Set attribute part of communication namestae element. */
  inline void setServerNamespaceAttribute(const char * attStr)
  { strncpy(serverNamespaceAttribute, attStr, MAX_NAMESPACE_ATTRIBUTE_LENGTH); };
  /**
  Set ressource for fast responce messages */
  bool setResource(UResBase * resource, bool remove);
  /**
  Is server open for connections - i.e. port number is valid */
  bool isOpen4Connections()
  { return open4Connections; };
  /**
   * Alive call from server thread */
  void serverIsAlive()
  { serverAlive.now(); };
  /**
   * Get time passed since the server was last reported alive.
   * \returns time in seconds since that server thread last set the alive timestamp. */
  double serverAliveLast();
  /**
   * Set the allow connections flag
   * Existing connections are not closed
   * \param value set to true, if to allow new connections */
  void setAllowConnections(bool value)
  { allowConnections = value; };
  /**
   * Get the allow connections flag
   * if false, then server do not allow (new) connections
   * \param value set to true, if to allow connections */
  bool getAllowConnections()
  { return allowConnections; };
  /**
   * Get client number of latest clinet connected - mostly for debug */
  int getLastClient()
  { return lastClientNumber; };
  /**
   * Should be called whenever a client is disconnected - to allow cleanup of obligations for this client */
  void connectionLost(int client);
  /**
   * Get last used client serial number */
  inline int getLastClientSerial()
  { return lastClientSerial; };

protected:
  /**
  Get a free client handle */
  int getFreeClientHandle();
  /**
  Service receive channel of all clients.
  Returns true is fata were processed.
  Returns false if nothing to process, either
  after poll-timeout or just no connections. */
  bool serviceClients(int msTimeout);
  /**
  Update var-pool variables */
  void updateVars();


  /**
  Pointer to ressources that nee immidiate data */
  UResLink resLink;
  /**
  Is the selected port valid, i.e. open for connections */
  bool open4Connections;
  /**
   * Should server allow connections, if not, then close server port - until changed. */
  bool allowConnections;

private:
  /**
  Running is true when accepting-calls thread is running */
  bool running;
  /**
  Flag to stop active connections and stop accepting connections. */
  bool terminate;
  /**
  Handle to accept thread */
  pthread_t  thServ;
  /**
  Structure for the accepted connections */
  UServerClient * client[MAX_SOCKET_CLIENTS_SERVED];
  /**
  Last used client connection */
  int clientCnt;
  /**
  Number of active clients */
  int clientCntActive;
  /**
  Print more to info connection / console */
  bool verboseMessages;
  /**
  Statistics - receive loops */
  int recvLoops;
  /**
  Queue for received messages from all clients */
  UServerInQueue rxQueue;
  /**
  Namespace to tell clients */
  char serverNamespace[MAX_SML_NAME_LENGTH];
  /**
  Server namespace attributes.
  This string must be coded to follow XML standard, and
  is send after the namespace and before the close '>' character */
  char serverNamespaceAttribute[MAX_NAMESPACE_ATTRIBUTE_LENGTH];
  /**
   * Sent namespace as opening message to the new client */
  bool serverNamespaceUse;
  /**
  Pointer to core var pool */
  UResVarPool * varPool;
  /**
   * The last time the server thread was reported alive */
  UTime serverAlive;
  /**
   * Server that should be notified of events */
  UCmdExe * eventServer;
  /**
   * client number to be used used for the last connection (-1 = no clients yet) */
  int lastClientNumber;
  /**
   * last client serial number used */
  int lastClientSerial;
public:
  /**
   * Should clients be punk'ed with alive messages to see if they are alive */
  UVariable * varAlivePunkTime;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////



#endif
