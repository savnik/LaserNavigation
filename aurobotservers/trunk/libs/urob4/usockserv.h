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

#ifndef USOCKSERV_H
#define USOCKSERV_H

#include <pthread.h>
#include <arpa/inet.h>

#include <ugen4/utime.h>

#define MAX_CLIENTS_SERVED 10
#define MAX_COM_INFO_SIZE 256

/**
Significance is an attribute to system messages
in the sXinfo(char*, USig, int) call. */
typedef enum significance
{einfo, ewarning, eerror, edebug}
             USig;

class USockServ;

/**
  example server from 31370 Exersize 6: TCP/IP
  *@author (Jens) Christian Andersen,326/030,3548,
  */

class USockServConnection
{
public:
  /**
  Constructor */
  USockServConnection();
  /**
  Destructor - stops connection */
  virtual ~USockServConnection();
  /**
  Get name of connecting client */
  char * getClientName();
  /**
  Handle this connection */
  bool handleConnection(int iConn, struct sockaddr_in iConnInfo);
  /**
  tests wether a newly accepted connection can be accepted,
  and initiates any queues etc. that needs to be set up for the
  connection.
  This is also the place to send any welcome messages.
  The virtual function is called before receive thread is activated.
  Returns true if connection is acceptable. */
  virtual bool justConnected();
  /**
  returns true if connection is open. */
  bool isActive() {return connected;};
  /**
  Stop the connection if active */
  bool stopConnection();
  /**
  This routine is running in a thread, and handles the
  connection protocol to this client.
  Returns when thread stopps. */
  int runConnection();
  /**
  Send this bufferfull of data within this timeout period.
  Returns true if send.
  Returns false if not possible within timeout opperiod. */
  bool blockSend(const char * buffer, int length, int msTimeout);
  /**
  Called at start of client connection and sends a welcome message
  if so desired.
  Should return false if no message is to be send. */
  virtual bool getWelcomeMessage(unsigned char * buf, int * len);
  /**
  A new message (or part of a message) has arraived.
  The message is assumed handled by this virtual function. */
  virtual void gotNewMessage(unsigned char * message, int length);
protected:
  /**
  Information that might be relevant for the user and might be printed
  on console or into logfile. */
  virtual void info(const char * info, USig significance, int interface);

protected:
  /** Closing connection if true */
  bool stop;           // closing connection if true
  /** Read thread handle */
  pthread_t  thClient; // read thread for connection
  /** Connection established flag */
  bool connected;      // connection open flag
  /** Connection handle */
  int conn;            // connection handle
  /** Structure with adress information etc. */
  struct sockaddr_in connInfo; // address info for connection
public:
  /** index number of client connection (0 to MAX_CLIENTS_SERVED-1) */
  int client;
  /** parent structure */
  USockServ * serv;
  /** statistics */
  int msgSend;
  /** receive loops (poll loops) */
  int recvLoops;
  /** id of the read process */
  pid_t pid;
};


/**
Socket server bas class, that opens a port and accepts connection
until no more client handles.
Each client are given a welcome message - if such is defined.
When client messages are received (in a receive thread) the butes
are transferred in a call to gotNewMessage(), and are assumed
consumed here.
Transmissions to the client are performed by a call to sendBlock().*/
class USockServ 
{
public:
  /**
  Constructor */
  USockServ();
  /**
  Destructor */
  virtual ~USockServ();
  /**
  Set server port number */
  int setPort(int iPort) {return port = iPort;};
  /**
  Start server with set port number */
  bool startServer();
  /**
  Stop server */
  int stopServer();
  /**
  start function for socket server thread */
  int runServerThread();
  /**
  Send data to client.
  if connection to client is open the data is send.
  returns false if client do not exist or data could not be send
  within timeout period. */
  bool blockSend(int client, const char * buffer, int length, int msTimeout);
  /**
  Tests if there is an active connection for this client,
  returns true if client is active (connected). */
  bool isActive(int client);
  /**
  Is server running. Returns false if server is not accepting connections. */
  bool isRunning() { return running;};
protected:
  /**
  Information that might be relevant for the user and might be printed
  on console or into logfile. */
  virtual void info(const char * info, USig significance, int interface);
  /**
  tests wether a newly accepted connection can be accepted,
  and initiates any queues etc. that needs to be set up for the
  connection.
  Returns true if connection is acceptable. */
  virtual bool justConnected(int client);
  /**
  Handle connection for client number clnt. clnt is index to client handle. */
  virtual int handleConnection(int iConn, struct sockaddr_in iConnInfo);
  /**
  Get free handle to a client connection.
  Returns -1 if no handle is available. */
  virtual int getFreeClientHandle();
public:
  /**
  Get count of used handles. */
  virtual int getUsedClientHandles();
  /**
  Connection error has occured, this is the details */
  virtual void gotConnectionProblems(int client, int errnum, const char * leadText);
private:
  /**
  Port where to accept connections */
  int port;
  /**
  Running is true when accepting calls */
  bool running;
  /**
  Flag to stop active connections and stop accepting connections. */
  bool stop;
  /**
  Handle to accept thread */
  pthread_t  thServ;
public:
  /**
  Structure for the accepted connections */
  USockServConnection * clientConn[MAX_CLIENTS_SERVED];
  /**
  Print more to info connection / console */
  bool verboseMessages;
  /**
  Start time - used fior timing analysis */
  UTime startTime;
  //unsigned char sourceIndex;
};

#endif
