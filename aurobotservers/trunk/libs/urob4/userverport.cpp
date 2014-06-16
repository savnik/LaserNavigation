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

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <ugen4/ucommon.h>

#include "uvarcalc.h"
#include "userverport.h"

/////////////////////////////////////////////////

UServerPort::UServerPort()
{
  int i;
  //
  varPool = NULL;
  for (i = 0; i < MAX_SOCKET_CLIENTS_SERVED; i++)
    client[i] = NULL;
  clientCnt = 0;
  clientCntActive = 0;
  //port = 24920; // default
  open4Connections = false;
  running = false;
  terminate = true;
  verboseMessages = false;
  recvLoops = 0;
  strncpy(serverNamespace,
          "namespace", MAX_SML_NAME_LENGTH);
  strncpy(serverNamespaceAttribute,
          "name=\"none\"", MAX_NAMESPACE_ATTRIBUTE_LENGTH);
  serverAlive.now();
  serverNamespaceUse = true;
  eventServer = NULL;
  lastClientNumber = -1;
  varAlivePunkTime = NULL;
}

/////////////////////////////////////////////////

UServerPort::~UServerPort()
{
  int i;
  // free client objects
  for (i = 0; i < clientCnt; i++)
    delete client[i];
}

///////////////////////////////////////////////

void * runServerPort(void * server)
{ // this is a separate thread
  UServerPort * obj = (UServerPort *) server;
  //
  obj->runServerThread();
  //
  pthread_exit(NULL);
  return NULL;
}


///////////////////////////////////////////////

bool UServerPort::start()
{
  bool result = true;
  pthread_attr_t  thConAttr;
  //
  // else use default port
  if (not running)
  { // Starts socket server thread 'runSockServer'
    pthread_attr_init(&thConAttr);
    // disable stop flag
    terminate = false;
    // create socket server thread
    pthread_create(&thServ, &thConAttr, &runServerPort, (void *)this);
    pthread_attr_destroy(&thConAttr);
  }
  else
  {
    fprintf(stderr, "UServerPort::start: Server port read loop is running already\n");
    result = false;
  }
  return result;
}

//////////////////////////////////////////////////////

bool UServerPort::runServerThread()
{ // running the server
  bool result = true;
  int err = 0;
  struct sockaddr_in name, from;
  int sock = -1; // server socket
  int clnt; // client connection
  const int NSL = 100;
  char hostname[NSL];
  int i, j = 0;
  socklen_t addr_len;
  bool retryCnt = 0;
  int lastErrNo = 0;
  //
  while (not terminate)
  { // thread that service clients is now running
    running = true;
    if (sock >= 0)
    { // test if new port is to be usedAHEAD
      if (name.sin_port != htons(serverPort) or not allowConnections)
      { // new port - close (but leave existing clients)
        close(sock);
        sock = -1;
        open4Connections = false;
        updateVars();
      }
    }
    if (sock < 0 and (serverPort >= 1000) and allowConnections)
    {
      sock = socket(AF_INET, SOCK_STREAM, 0);
      result = (sock >= 0);
      if (not result)
        perror("*** UServerPort::runServerThread (socket)");
      if (result)
      { // set socket port reuse option
        i = 1; // true
        err = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &i, sizeof(i));
        result = (err == 0);
        if (not result)
          perror(strerror(errno));
      }
      // get specific socket
      if (result)
      { // prepare bind to protocol and port
        name.sin_family = AF_INET;
        name.sin_port = htons(serverPort);
        name.sin_addr.s_addr =INADDR_ANY;
        /**/
        err = bind(sock, (struct sockaddr *) &name, sizeof(name));
        if (err != 0)
        { // bind failed - port in use by another server
          if (retryCnt == 0)
            perror("*** UServerPort::runServerThread (bind port)");
          result = false;
          retryCnt++;
        }
        else
          retryCnt = 0;
      }
      //
      if (result)
      { // listen to queue, and allow only 1 pending connections
        err = listen(sock, 3);
        if (err != 0)
        { // listen failed, - no more space?
          perror("*** UServerPort::runServerThread (listen)");
          result = false;
        }
        else
        {
          open4Connections = true;
          updateVars();
        }
      }
      //
      if (result)
      { // listen is sucessfull, now accept connections
        if (verboseMessages)
        { // print a connect message
          i = gethostname(hostname, NSL - 1);
          hostname[NSL - 1] = 0; // just in case of a very long hostname
          printf("      --- %s is open on port %d for requests.\n", hostname, serverPort);
        }
        //
        addr_len = sizeof(from);
        // set server to non-blocking
        fcntl(sock, F_SETFL, O_NONBLOCK);
      }
      if (not result)
      { // cound not bind socket port - close
        if (sock >= 0)
        {
          close(sock);
          sock = -1;
        }
      }
    }
    if (sock >= 0)
    { // accept connections
      clnt = accept(sock, (struct sockaddr*) & from, &addr_len);
      if (clnt == -1)
      { // not a connect - may be a ctrl-c
        i = errno;
/*        j = EOPNOTSUPP; // 95
        j = ENOTSOCK;   // 88
        j = EBADF;      //  9
        j = EWOULDBLOCK;// 11
        Too many open files: errno = 24
        k = EAGAIN;     // 11*/
        if (i !=EWOULDBLOCK)
        { // other signal that retry
          if (i != lastErrNo)
            perror("*** UServerPort::runServerThread (accept)");
          // debug
          else if (i == 24)
          { // too many open files - how is this possible??
            // ulimit say max is 1024, so either an error or someone eats file handles
            Wait(0.05);
            lastErrNo = errno;
            j++;
            if (j > 100)
            { // time to report
              j = 0;
              lastErrNo = 0;
            }
          }
          // debug end
        }
      }
      else
      { // connection available
        // get name of requester
        err = getpeername(clnt,
                          (struct sockaddr *)&from,
                          &addr_len);
        if ((err == 0) and verboseMessages)
          // name is available
          printf("Connection request from IP: %s\n", inet_ntoa(from.sin_addr));
        //
        // activate a client object
        i = getFreeClientHandle();
        if (i >= 0)
        { // set client
          client[i]->initConnection(i, clnt, from, &rxQueue, &serverAlive);
          gotNewClient(client[i]);
        }
        else
        { // close client connection again - no more space
          if (verboseMessages)
            fprintf(stderr, "*** UServerPort::runServerThread: out of client handles => HUP\n");
          shutdown(clnt, SHUT_RDWR);
          close(clnt);
        }
      }
      // service clients
      if (getActiveClientCnt() > 0)
        // poll clients with a 200 ms timeout
        serviceClients(200);
      else
        // idle - wait a bit (e.g. 0.2 sec)
        Wait(0.2);
    }
    else
      // socket not open - wait for available port number
      Wait(1.0);
  }
  // close socket and terminate
  if (sock >= 0)
    close(sock);
  //
  running = false;
  return err;
}

////////////////////////////////////////////////////////

void UServerPort::updateVars()
{
  UVarPool * vp;
  //
  if (varPool != NULL)
  {
    vp = varPool->getVarPool();
    if (vp != NULL)
    {
      vp->setLocalVar("open4Connections", (double)open4Connections, false, UVariable::d);
      vp->setLocalVar("port", (double)serverPort, false, UVariable::d);
      vp->setLocalVar("clients", (double)clientCntActive, false, UVariable::d);
    }
  }
}

////////////////////////////////////////////////

void UServerPort::gotNewClient(UServerClient * cnn)
{
  const int MRL = 60;
  char reply[MRL];
  //
  printf("Got new client (%s)\n", cnn->getClientName());
  if (serverNamespaceUse)
  {
    snprintf(reply, MRL, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    cnn->blockSend(reply, strlen(reply), 1000);
    snprintf(reply, MRL, "<%s %s>\n", getServerNamespace(), getServerNamespaceAttribute());
    cnn->blockSend(reply, strlen(reply), 1000);
  }
  //
}

////////////////////////////////////////////////

int UServerPort::getFreeClientHandle()
{ // returns unused handle or -1 if no handle is avaulable
  int result = -1;
  int i;
  UServerClient * cnn;
  int cln;
  //
  for (i = 0; i < MAX_SOCKET_CLIENTS_SERVED; i++)
  {
    lastClientNumber = (lastClientNumber + 1) % MAX_SOCKET_CLIENTS_SERVED;
    cln = lastClientNumber;
    cnn = client[cln];
    lastClientSerial++;
    if (cnn == NULL)
    {
      cnn = new UServerClient(cln);
      cnn->setResLink(&resLink);
      client[cln] = cnn;
    }
    if (cnn != NULL)
      if (not cnn->isActive())
      { // available connection is found
        result = cln;
        cnn->clientSerial = lastClientSerial;
        if (cln >= clientCnt)
          clientCnt = cln + 1;
        break;
      }
  }
  //
  return result;
}

////////////////////////////////////////////////

bool UServerPort::serviceClients(int msTimeout)
{
  bool result = false;
  int err = 0;
  int i;
  int nActive;
  UServerClient * cnn;
  bool dataAvailable;
  struct pollfd pollStatus[MAX_SOCKET_CLIENTS_SERVED];
  int clientNum[MAX_SOCKET_CLIENTS_SERVED];
  int revents;
  //
  nActive = 0;
  for (i = 0; i < clientCnt; i++)
  { // prepare poll structures for all clients
    cnn = client[i];
    if (cnn->isActive())
    { // active, so
      pollStatus[nActive].fd = cnn->getCnn();
      pollStatus[nActive].revents = 0;
      pollStatus[nActive].events = POLLIN  |   /*  0x0001  There is data to read */
                                   POLLPRI;   /*  0x0002  There is urgent data to read */
                                /* POLLOUT 0x0004  Writing now will not block */
      // mark index to client structure
      clientNum[nActive] = i;
      // count active clients
      nActive++;
      if (varAlivePunkTime != NULL)
      {
        if (varAlivePunkTime->getDouble() > 0.1)
        {
          UTime * clTime = (UTime *)cnn->getTimeOfLastMessage();
          if (clTime->getTimePassed() > varAlivePunkTime->getDouble() and cnn->punkTime.getTimePassed() > varAlivePunkTime->getDouble())
          {
//            printf("client %d has no communication within the last %.4f seconds - sending alive reply\n", i, clTime->getTimePassed());
            cnn->sendAliveReply();
            cnn->punkTime.now();
          }
        }
      }
    }
  }
  //
  if (nActive != clientCntActive)
  {
    updateVars();
    clientCntActive = nActive;
  }
  //
  if (nActive > 0)
  { // poll for status on this socket (timeout 200 ms)
    err = poll(pollStatus, nActive, msTimeout);
    if (err < 0)
    { // not a valid call (may be debugger interrrupted)
      perror("UServerPort::serviceClients (poll)");
    }
    else if (err > 0)
    { // at least one connection has data (or status change)
      for (i = 0; i < nActive; i++)
      { // test all connections
        dataAvailable = false;
        revents = pollStatus[i].revents;
        if (((revents & POLLIN) != 0) or
            ((revents & POLLPRI) != 0))
          dataAvailable = true;
        else if (((revents & POLLERR) > 0) or
                 ((revents & POLLHUP) > 0) or
                 ((revents & POLLNVAL) > 0))
        { // error situation
          cnn = client[clientNum[i]];
          printf("Client %s (%d) has left the building (poll)\n",
            cnn->getClientName(), clientNum[i]);
          // connection is lost - send no HUP
          cnn->stopConnection(false, NULL);
        }
        // other conditions are ignored
        //
        if (dataAvailable)
        { // get available data at this point (up to a bufer length)
          cnn = client[clientNum[i]];
          result = cnn->receiveData();
          if (verboseMessages and not result)
          {
            printf("Client %s (%d) has left the building (recv)\n",
              cnn->getClientName(), clientNum[i]);
            cnn->stopConnection(true, serverNamespace);
          }
          // notify of new data
          if (result)
            messageReceived();
        }
      }
    }
  }
  // count loops
  recvLoops++;
  return result;
}

////////////////////////////////////////////////

void UServerPort::print(const char * preStr)
{
  const int NSL = 100;
  char hostname[NSL];
  int i;
  //
  gethostname(hostname, NSL);
  printf("%son %s port %d ", preStr, hostname, serverPort);
  if (running)
  {
    printf(" running, has %d clients (rx loops %d) verbose %s\n",
      getActiveClientCnt(), recvLoops, bool2str(verboseMessages));
    for (i = 0; i < clientCnt; i++)
    {
      snprintf(hostname, NSL, " - clnt#%d ", i);
      client[i]->print(hostname);
    }
    rxQueue.print(" - rxQueue");
  }
  else
    printf("is not running\n");
}

//////////////////////////////////////////////////

int UServerPort::getActiveClientCnt()
{
   int result = 0;
   int i;
   //
   for (i = 0; i < clientCnt; i++)
     if (client[i]->isActive())
       result++;
   //
   return result;
}

//////////////////////////////////////////////////

bool UServerPort::stop(bool andWait)
{
  int i;
  // stop server thread.
  terminate = true;
  // wait for server thread to terminate
  if (andWait and running)
    pthread_join(thServ, NULL);
  // terminate all connections
  for (i = 0; i < clientCnt; i++)
    client[i]->stopConnection(true, serverNamespace);
  // return thread state
  return not running;
}

/////////////////////////////////////////////////

void UServerPort::messageReceived()
{ // new (mail) received (is in queue)
  if (verboseMessages)
    printf("Received data on port %d (%d/%d in rx queue)\n",
       serverPort, rxQueue.getUsedMsgCnt(), rxQueue.getElements());
  if (eventServer != NULL)
    eventServer->event();
}


/////////////////////////////////////////////////

UServerClient * UServerPort::getClient(const int i)
{
  UServerClient * result = NULL;
  //
  if ((i >= 0) and (i < clientCnt))
    result = client[i];
  //
  return result;
}

/////////////////////////////////////////////////

bool UServerPort::setResource(UResBase * resource, bool remove)
{
  int i;
  bool result;
  //
  if (resource->isAlsoA(UCmdExe::getResClassID()))
  {
    if (remove)
      eventServer = NULL;
    else
      eventServer = (UCmdExe *)resource;
  }
  result =  resLink.setResource(resource, remove);
  for (i = 0; i < clientCnt; i++)
    client[i]->resourceUpdated();
  //
  return result;
}

///////////////////////////////////////////////////

char * UServerPort::getHostName(char * nameBuffer, const int nameBufferCnt)
{
  char * result = NULL;
  int err;
  if ((nameBuffer != NULL) and (nameBufferCnt > 0))
  {
    err = gethostname(nameBuffer, nameBufferCnt);
    if (err == 0)
      result = nameBuffer;
  }
  return result;
}

/////////////////////////////////////////////

double UServerPort::serverAliveLast()
{
  return serverAlive.getTimePassed();
}


