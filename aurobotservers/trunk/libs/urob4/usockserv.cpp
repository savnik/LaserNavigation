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

#include <sys/utsname.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
//#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>

#include <ugen4/utime.h>
#include <ugen4/ucommon.h> // Wait()
//#include "urobformat.h"
#include "usockserv.h"

//////////////////////////////////////////////////////////////

void * runServerConn(void * connectionObject)
{ // thread start function
  //int err = 0;
  USockServConnection * connObj = (USockServConnection *) connectionObject;
  // run the connection
  connObj->runConnection();
  // thread is finished
  pthread_exit(NULL);
  // the rest from here is not executed
  return NULL;
}

//////////////////////////////////////////////////////////////

USockServConnection::USockServConnection()
{
  connected = false;
  conn = -1;
  stop = true;
  msgSend = 0;
  recvLoops = 0;
}

//////////////////////////////////////////////////////////////

USockServConnection::~USockServConnection()
{
  if (connected)
    stopConnection();
}

//////////////////////////////////////////////////////////////

bool USockServConnection::handleConnection(int iConn, struct sockaddr_in iConnInfo)
{ //  Handle this connection
  int err;
  pthread_attr_t  thConAttr;
  bool isOK;
  //
  if (not connected)
  {
    conn = iConn;
    connInfo = iConnInfo;
    //
    connected = true;
    // call this virtual method to allow setting of receive queues etc.
    // sending of welcome message before anything is read
    // and possible validity check for connection
    isOK = justConnected();
    if (isOK)
    { // ready for the connection
      stop = false;
      // Starts socket server thread 'runSockServer'
      pthread_attr_init(&thConAttr);
      // create socket thread to handle connection
      err = pthread_create(&thClient, &thConAttr, &runServerConn, (void *)this);
      pthread_attr_destroy(&thConAttr);
      if (err != 0)
      { // failed to create receive thread
        stop = true;
        connected = false;
      }
    }
  }
  return connected;
}

///////////////////////////////////////////////

bool USockServConnection::justConnected()
{ // should be overwritten if
  // some initialization is required
  return true;
}

///////////////////////////////////////////////

char * USockServConnection::getClientName()
{
  char * result = NULL;
  //
  if (connected)
    result = inet_ntoa(connInfo.sin_addr);
  //
  return result;
}

///////////////////////////////////////////////

int USockServConnection::runConnection()
{
  const int MAX_CMD_LNG = 500;
//  unsigned int tdelay = 1000;
  int err = 0;
  int len;
  int slen;
//  bool finished = false;
  unsigned char buf[MAX_CMD_LNG + 1]; // send buffer
  bool isOK, dataAvailable;
  struct pollfd pollStatus;
  // preare status poll
  pollStatus.fd = conn;
  pollStatus.events = POLLIN  +   /*  0x0001  There is data to read */
                      POLLPRI +   /*  0x0002  There is urgent data to read */
                      /* POLLOUT 0x0004  Writing now will not block */
                      POLLERR +   /*  0x0008  Error condition */
                      POLLHUP +   /*  0x0010  Hung up */
                      POLLNVAL;   /*  0x0020  not valid */
  pollStatus.revents = 0;
  //
  pid = getpid();
  // send welcome message
  isOK = getWelcomeMessage(buf, &len);
  if (isOK and (len > 0))
    slen = send(conn, buf, len, 0);
  if (slen < len)
    printf("USockServConnection::runConnection: failed to send welcome message - rather bad!\n");
  //
  snprintf((char *)buf, MAX_CMD_LNG, "New thread is listening (client %s)", getClientName());
  info((char *)buf, einfo, client);
  //
  while (not stop)
  { // poll for status on this socket (timeout 200 ms)
    dataAvailable = false;
    err = poll(&pollStatus, 1, 200);
    if (err < 0)
    { // not a valid call - socket no longer valid
      serv->gotConnectionProblems(client, errno, "Connection lost (poll err)");
      // debug do not accept poll error as an error
      err = 0;
    }
    else if (pollStatus.revents > 0)
    { // there is a returned events to handle
      err = 0;
      if ((pollStatus.revents & POLLIN) > 0)
        dataAvailable = true;
      else if ((pollStatus.revents & POLLPRI) > 0)
        dataAvailable = true;
      else if ((pollStatus.revents & POLLOUT) > 0)
        Wait(0.1); // should not be, but such shit happens
      else if (((pollStatus.revents & POLLERR) > 0) or
          ((pollStatus.revents & POLLHUP) > 0) or
          ((pollStatus.revents & POLLNVAL) > 0))
      { // error situation
        serv->gotConnectionProblems(client, errno, "Connection lost (poll)");
        err = -1;
      }
      else
      { // error situation - other
        serv->gotConnectionProblems(client, errno, "Connection lost (poll error)");
        err = -1;
      }
    }
    if (dataAvailable)
    { // get available data at this point (up to a bufer length)
      len = recv(conn, buf, MAX_CMD_LNG, 0);
      if (len > 0)
      { // data received
        // zero terminate (to allow display as string)
        buf[len] = 0;
        gotNewMessage(buf, len);
      }
      else if (len == 0)
      { // no data, often, when connection is
        // closed, but no HUP received
        // spend time waiting until reconnected or closed
        serv->gotConnectionProblems(client, 0, "Receive error - no more data");
        err = -1;
//        Wait(0.1);
      }
      else // len = -1
      { // not a message - idle os OK, else terminate
        if (errno != EAGAIN)
        { // other signal that retry
          serv->gotConnectionProblems(client, errno, "Receive error (not EAGAIN)");
          err = -1;
        }
      }
    }
    //
    if (err != 0)
      break;
    recvLoops++;
  }
  // shutdown connection for both read and write
  shutdown(conn, SHUT_RDWR);
  // close client socket handle
  close(conn);
  //
  printf("Connection shut down from client %s\n", getClientName());
  // mark as closed
  connected = false;
  return err;
}

//////////////////////////////////////////////////////////////

bool USockServConnection::stopConnection()
{ // terminate this connection
  stop = true;
  // send
  kill(pid, SIGINT);
  // wait for connection thread to stop
  pthread_join(thClient, NULL);
  // close socket
  close(conn);
  connected = false;
  //
  return true;
}

/////////////////////////////////////////////////////////////

bool USockServConnection::blockSend(const char * buffer, int length, int msTimeout)
{ // returns true if send and false if connection timeout
  const int pollTime = 50; // miliseconds
  bool result = true;
  int d = 0, n, t = 0;
  struct pollfd pollStatus;
  //
  // preare status poll
  pollStatus.fd = conn;
  pollStatus.events = POLLOUT;
  pollStatus.revents = 0;
  // status out is not used, if no error, then just try
  if (connected)
  { // still connected (no error yet)  buffer[4]
    // send length bytes
    while ((d < length) and (t < msTimeout) and result)
    { // get status
      poll(&pollStatus, 1, pollTime);
      //
      // poll status > POLLOUT is an error condition
      if ((pollStatus.revents & 0x3f) <= POLLOUT)
      { // no error
        if ((pollStatus.revents & POLLOUT) == POLLOUT)
        { // socket is ready for output - send some bytes
          n = send(conn, &buffer[d], length - d, 0);
          // count bytes send
          d += n;
        }
        else
          // not ready for output - count timeouts from poll
          t += pollTime;
      }
      else
      { // timeout - so connection has failed
        // and should probably be dropped.
        result = false;
      }
    }
  }
  else
    result = false;
  // count messages
  if (result)
  {
    msgSend++;
  }
  //
  return result;
}


////////////////////////////////////////////////////////

void USockServConnection::gotNewMessage(unsigned char * message, int length)
{ // Got new message from client for parsing
  // default is just to print info to console with first 4 characters
  //UTime t;
  //char s[MAX_COM_INFO_SIZE];
  //
  //t.Now();
  //t.GetTimeAsString(s, true);
  printf("From  client %2d length%5d bytes (0,1,2,3: %02x,%02x,%02x,%02x)\n",
               client, length,
               message[0], message[1], message[2], message[3]);
}


////////////////////////////////////////////////////////

bool USockServConnection::getWelcomeMessage(unsigned char * /*buf*/, int * /*len*/)
{ // should be overwritten if
  // welcome message is to be send
  // the message will be sed as is (i.e. conform to desired protocol).
  return false;
}

////////////////////////////////////////////////////////


void USockServConnection::info(const char * info,
                             USig significance,
                             int interface)
{ // normally overwritten by higher level virtual method.
  printf("USockServConnection::%s (%d;%d)\n", info, significance, interface);
}


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
// TcpIpServer

USockServ::USockServ()
{
  int i;
  //
  port = 31370;
  running = false;
  stop = false;
  // set reference to parent structure
  for (i = 0; i < MAX_CLIENTS_SERVED; i++)
    // clients MUST be allocated by a derived class
    clientConn[i] = NULL;
  // used in binary messages and should be set by
  // to another value before use.
  //sourceIndex = 0;
  verboseMessages = true;
}

///////////////////////////////////////////////////

void * runSockServer(void * server)
{ // this is a separate thread
  USockServ * s = (USockServ *) server;
  //
  s->runServerThread();
  //
  pthread_exit(NULL);
  return NULL;
}


///////////////////////////////////////////////

bool USockServ::startServer()
{
  bool result = true;
  pthread_attr_t  thConAttr;
  //
  if (port >1000 and not running)
  { // Starts socket server thread 'runSockServer'
    pthread_attr_init(&thConAttr);
    // disable stop flag
    stop = false;
    // create socket server thread
    pthread_create(&thServ, &thConAttr, &runSockServer, (void *)this);
    pthread_attr_destroy(&thConAttr);
  }
  else
    result = false;
  return result;
}

//////////////////////////////////////////////////////

int USockServ::runServerThread()
{ // running the server
  const int bufLen = 100;
  int err = 0;
  unsigned int tdelay = 100000; // 0.1 second
  struct sockaddr_in name, from;
  int sock; // server socket
  int clnt; // client connection
  char buf[bufLen];
  int i;
  socklen_t addr_len;
  //
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    //perror(strerror(errno));
    gotConnectionProblems(-1, errno, "USockServ::runServer: Could not create socket");
    err = -1;
  }
  // set socket port reuse option
  i = 1; // true
  err = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &i, sizeof(i));
  if (err < 0)
    gotConnectionProblems(-1, errno, "USockServ::runServer: Could not set reuse option");
  // get specific socket
  if (err == 0)
  { // prepare bind to protocol and port
    name.sin_family = AF_INET;
    name.sin_port = htons(port);
    name.sin_addr.s_addr =INADDR_ANY;
    /**/
    err = bind(sock, (struct sockaddr *) &name, sizeof(name));
    if (err < 0)
      // bind failed
      gotConnectionProblems(-1, errno, "USockServ::runServer: Could not bind to port");
  }
  //
  if (err == 0)
  { // listen to queue, and allow only 1 pending connections
    err = listen(sock, 1);
    if (err != 0)
      // listen failed, most likely becourse another process already
      // listens at this port.
      gotConnectionProblems(-1, errno, "USockServ::runServer: listen failed");
  }
  //
  if (err == 0)
  { // listen is sucessfull, now accept connections
    i = gethostname(buf, bufLen-1);
    buf[bufLen-1] = 0; // just in case of a very long hostname
    printf("                  - %s is open on port %d for requests.\n", buf, port);
    //
    running = true;
    addr_len = sizeof(from);
    // set server to non-blocking
    fcntl(sock, F_SETFL, O_NONBLOCK);
    // accept connections
    while ((not stop) and (err == 0))
    {
      clnt = accept(sock, (struct sockaddr*) & from, &addr_len);
      if (clnt == -1)
      { // not a connect - may be a ctrl-c
        i = errno;
        if (i != EAGAIN)
        { // other signal that retry
          gotConnectionProblems(-1, i, "USockServ::runServer: Accept returned error");
          err = -1;
        }
      }
      else
      { // connection available
        // get name of requester
        err = getpeername(clnt,
                          (struct sockaddr *)&from,
                          &addr_len);
        if (err == 0)
          // name is available
          printf("Connection request from IP: %s\n", inet_ntoa(from.sin_addr));
        //
        // activate a client object
        i = handleConnection(clnt, from);
        if (i < 0)
        { // close client connection again - no more space
          gotConnectionProblems(-1, i, "USockServ::runServer: out of client handles");
          close(clnt);
        }
      }
      // wait a bit (e.g. 0.1 sec)
      usleep(tdelay);
    }
  }
  // close socket
  if (sock >= 0)
    close(sock);
  //
  running = false;
  return err;
}

////////////////////////////////////////////////

int USockServ::handleConnection(int iConn, struct sockaddr_in iConnInfo)
{ // transfer control to an available client object
  // if returns -1, then connection is closed
  int i;
  //
  i = getFreeClientHandle();
  if (i >= 0)
  {
    if (justConnected(i))
    { // not refused
      if (verboseMessages)
        printf("Connection handled by slot %d\n", i);
      clientConn[i]->handleConnection(iConn, iConnInfo);
    }
    else
      // connection refused by justConnected()
      i = -1;
  }
  return i;
}

////////////////////////////////////////////////

bool USockServ::justConnected(int)
{ // virtuel function, that
  // can be used to refuce connection
  // and handle other server server side initializations
  return true;
}

////////////////////////////////////////////////

int USockServ::getFreeClientHandle()
{ // returns unused handle or -1 if no handle is avaulable
  int result = -1;
  int i;
  //
  for (i = 0; i < MAX_CLIENTS_SERVED; i++)
    if (not isActive(i))
    { // available connection is found
      result = i;
      break;
    }
  //
  return result;
}

////////////////////////////////////////////////

int USockServ::getUsedClientHandles()
{
  int result = 0;
  int i;
  //
  for (i = 0; i < MAX_CLIENTS_SERVED; i++)
    if (isActive(i))
      result++;
  //
  return result;
}


//////////////////////////////////////////////////////

int USockServ::stopServer()
{
  int err = 0;
  if (running)
  { // flag server thread to stop
    stop = true;
    // wait for server thread to terminate
    pthread_join(thServ, NULL);
  }
  return err;
}

////////////////////////////////////////////////////////


void USockServ::gotConnectionProblems(int client, int errnum, const char * leadText)
{ // Connection error has occured, this is the details
  // just print to console
//  UTime t;
  char s[MAX_COM_INFO_SIZE];
  char s2[MAX_COM_INFO_SIZE];
  //
//  t.Now();
//  t.getTimeAsString(s, true);
  if (errnum != 0)
  {
    strerror_r(errnum, s2, MAX_COM_INFO_SIZE);
    printf("Error client %2d errno%5d msg: %s (%s)\n",
            client, errnum, leadText, s2);
  }
  else
    printf("Error client %2d at %s\n",
            client, s);
}


//////////////////////////////////////////////////////////

bool USockServ::isActive(int client)
{
  bool result = false;
  // is client number legal
  if ((client >= 0) and (client < MAX_CLIENTS_SERVED))
    if (clientConn[client] != NULL)
      result = clientConn[client]->isActive();
  //
  return result;
}

////////////////////////////////////////////////////////

void USockServ::info(const char * info,
                             USig significance,
                             int interface)
{ // normally overwritten by higher level virtual method.
  if (verboseMessages or (significance == eerror))
    printf("USockServ::%s (%d;%d)\n", info, significance, interface);
}

////////////////////////////////////////////////////////

USockServ::~USockServ()
{
}
