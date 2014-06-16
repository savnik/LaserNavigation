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
#include "uclientport.h"

#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "usmltag.h"

////////////////////////////////////////////////////////////////////

UClientPort::UClientPort()
{ // mark socket as not created
  sock = -1;
  // default port
  port = 24920;
  // default host
  strncpy(host, "localhost", MAX_HOST_LENGTH);
  //
  readCnt = 0;
  connected = false;
  verboseMessages = false;
  // debug
  verboseMessages = true;
  // debug end
  rxByteCnt = 0;
}

////////////////////////////////////////////////////////////////////

UClientPort::~UClientPort()
{
  if (connected)
    closeConnection();
}

////////////////////////////////////////////////////////////////////

void UClientPort::connectionChange(bool nowConnected)
{
  if (nowConnected)
    printf("Server %s:%d is on-line\n", host, port);
  else
    printf("Server %s:%d has left the building\n", host, port);
}

////////////////////////////////////////////////////////////////////

void UClientPort::closeConnection()
{ // shutdown connection -- tell the other end
  if (connected)
  {
    connected = false;
    if (sock > 0)
    { // do an orderly shutdown
      shutdown(sock, SHUT_RDWR);
      close(sock);
      sock = -1;
    }
    // inform at higher levels
    connectionChange(connected);
  }
  else
    printf("Not connected\n");

}

////////////////////////////////////////////////////////////////////

bool UClientPort::tryConnect()
{
  bool result;
  int err;
  struct sockaddr_in name;
  struct hostent * hostEntry;
  char * hostaddr;
  struct in_addr IPadr;
  bool valid;
  //
  if (tryLock())
  {
    readCnt = 0;
    // if connected already, then disconnect
    if (connected)
       closeConnection();
    //
/*    if (toHost != NULL)
      strncpy(host, toHost, MAX_HOST_LENGTH);
    if (port > 1000)
      port = toPort;*/
    //
    sock = socket(AF_INET, SOCK_STREAM, 0);
    result = (sock >= 0);
    if (not result)
    {
      if (verboseMessages)
        perror("UClientPort::tryConnect");
      err = -1;
    }
    if (result)
    { // resolve name
  /*    if (false and verboseMessages)
        printf("UClientPort::tryConnect: Socket created ...\n");*/
      // try convert host name to an IP number
      valid = inet_aton(host, &IPadr);
      if (valid)
        strncpy(hostIP, host, MAX_HOST_LENGTH);
      else
      { // host name is not an IP number, so try resolve name
        // try resolve host name to IP number
        h_errno = 0;
        hostEntry = gethostbyname(host);
        if (hostEntry != NULL)
        { // name to IP resolution succeded
          hostaddr = hostEntry->h_addr_list[0]; // first address in net byte order
          IPadr = * (struct in_addr *) hostaddr;
  /*        if (verboseMessages)
            printf("UClientPort::tryConnect: to %s at addr %s - connecting ...\n",
                hostEntry->h_name,  inet_ntoa(IPadr));*/
          // save IP address of host name (may be equal)
          strncpy(hostIP, inet_ntoa(IPadr), MAX_HOST_LENGTH);
        }
        else
        { // h_errno should be set by the gethostname() function
          if (verboseMessages)
            switch (h_errno)
            { // show error text
              case HOST_NOT_FOUND:
                printf("UClientPort::tryConnect: Host not found\n");
                break;
              case NO_DATA:
              //case NO_ADDRESS: - same as NO_DATA
                printf("UClientPort::tryConnect: No IP adress found\n");
                break;
              case NO_RECOVERY:
                printf("UClientPort::tryConnect: Error while resolving name (no recovery)\n");
                break;
              case TRY_AGAIN:
                printf("UClientPort::tryConnect: Error at name server - try later.\n");
                break;
              default:
                printf("UClientPort::tryConnect: Unknown error occured while resolving host mane.\n");
            }
          result = false;
        }
      }
    }
    //
    if (result)
    { // set up structure for connect
      name.sin_family = AF_INET;
      name.sin_port = htons(port);
      name.sin_addr = IPadr; // .s_addr = inet_addr(hostIP);
      // set to non-blocking, to be able to stop
      // fcntl(sock, F_SETFL, O_NONBLOCK);
      // try a connect
      err = connect(sock, (struct sockaddr *) &name, sizeof(name));
      //
      result = (err == 0);
      if (verboseMessages and not result)
        // connection error
        if (false and verboseMessages)
          perror("UClientPort::tryConnect");
    }
    if (result)
    { // mark as connected
      connected = true;
      // preare status poll for receive
      recvPollStatus.fd = sock;
      recvPollStatus.events = POLLIN  +   /*  0x0001  There is data to read */
                          POLLPRI +   /*  0x0002  There is urgent data to read */
                          /* POLLOUT 0x0004  Writing now will not block */
                          POLLERR +   /*  0x0008  Error condition */
                          POLLHUP +   /*  0x0010  Hung up */
                          POLLNVAL;   /*  0x0020  not valid */
      recvPollStatus.revents = 0;  // clear returned events
      //
      // inform at higher levels - NB! they may set namespace
      connectionChange(connected);
      Wait(0.05);
      // moved to uclienthandler.cpp - gotNewData(...) function, when nametag is received
      //sendNamespaceOpenTag();
    }
    //
    if (not connected and (sock >= 0))
    { // release socket handle
      close(sock);
      sock = -1;
    }
    unlock();
  }
  else
    // debug
    printf("TryConnect found locked?\n");
  // debug end
  //
  return connected;
}

//////////////////////////////////////////////////////////////////////

void UClientPort::sendNamespaceOpenTag()
{ // no post-connect issues are handled here
  // is likely overwritten at higher levels
}

//////////////////////////////////////////////////////////////////////

int UClientPort::getDataFromLine(char * buffer, int bufferSize, int pollTimeoutMs)
{
  int err;
  bool dataAvailable = false;
  int len = 0;
  bool connectionGotLost = false;
  //
  dataAvailable = false;
  // get connection status
  err = poll(&recvPollStatus, 1, pollTimeoutMs);
  readCnt++;
  if (err < 0)
  { // poll failed (not timeout)
    if (false)
    { // internal polling error (EAGAIN or EINTR) - most likely ignorable
      if (verboseMessages)
        printf("UClientPort::getDataFromLine: Connection lost (poll err)\n");
      // do not drop connection here, may be recoverable.
      //connectionGotLost = true;
    }
  }
  else if (recvPollStatus.revents != 0)
  { // status is not timeout
    if ((recvPollStatus.revents & POLLIN) > 0)
      // normal data available
      dataAvailable = true;
    else if ((recvPollStatus.revents & POLLPRI) > 0)
      // priority data
      dataAvailable = true;
    else if (((recvPollStatus.revents & POLLERR) > 0) or
        ((recvPollStatus.revents & POLLHUP) > 0) or
        ((recvPollStatus.revents & POLLNVAL) > 0))
    { // error situation (error, hangup or not valid)
      if (verboseMessages)
        printf("UClientPort::getDataFromLine: Connection lost (HUP)\n");
      connectionGotLost = true;
    }
    else
    { // error situation (error, hangup or not valid)
      if (verboseMessages)
        printf("UClientPort::getDataFromLine: Connection lost (unknown poll error)\n");
      connectionGotLost = true;
    }
  }
  if (dataAvailable)
  { // data available to read
    // just get what is available up to buffer length
    len = recv(sock, buffer, bufferSize, 0);
    if (len < 0)
    { // error - connection is lost       h_errno
      if (verboseMessages)
        perror("UClientPort::getDataFromLine:");
      connectionGotLost = true;
    }
    else if (len == 0)
    { // orederly hang up - should be caught by poll
      if (verboseMessages)
        printf("UClientPort::getDataFromLine: got no data, when poll say data (assume HUP)\n");
      connectionGotLost = true;
      Wait(0.005);
    }
    else
      rxByteCnt += len;
  }
  //
  if (connectionGotLost)
  { // shut down connection properly
    // debug
    printf("UClientPort::getDataFromLine Connection got lost (%s:%d) closing\n", getHost(), getPort());
    // debug end
    closeConnection();
    len = -1;
  }
  //
  return len;
}

///////////////////////////////////////////////////////////////

bool UClientPort::blockSend(const char * buffer, int length)
{
  const int pollTime = 50; // miliseconds
  int msTimeout = 100 + length; // timeout in miliseconds (allows 10000 bit/sec plus a bit)
  bool result = true;
  int d = 0, n, t = 0;
  struct pollfd pollStatus;
  int err;
  //
  // preare status poll
  pollStatus.fd = sock;
  pollStatus.events = POLLOUT;
  pollStatus.revents = 0;
  // status out is not used, if no error, then just try
  if (connected)
  { // still connected (no error yet)  buffer[4]
    // send length bytes
    while ((d < length) and (t < msTimeout) and result)
    { // get status
      err = poll(&pollStatus, 1, pollTime);
      if (err == -1)
      { // poll error
        perror("UServerClient::blockSend");
        result = false;
        break;
      }
      else if (err > 0)
      { // poll status > POLLOUT is an error condition
        if (((pollStatus.revents & POLLERR) != 0) or
                 ((pollStatus.revents & POLLHUP) != 0) or
                 ((pollStatus.revents & POLLNVAL) != 0))
        { // connection dropped
          printf("server has left the building\n");
          result = false;
        }
        else if ((pollStatus.revents & POLLOUT) == POLLOUT)
        { // socket is ready for output - send some bytes
          n = send(sock, &buffer[d], length - d, 0);
          // count bytes send
          d += n;
        }
      }
      // assume minimum poll time has passed
      // ( either it has, or some data is send or some
      //   other event has passed (non masked event)
      t += pollTime;
    }
    if (not result)
    {
      // debug
      printf("UClientPort::blockSend Connection got lost\n");
      // debug end
      closeConnection();
    }
  }
  else
    result = false;
  //
  return result;
}

/////////////////////////////////////////////////////////

bool UClientPort::sendMsg(const char * message)
{
  bool result;
  result = blockSend(message, strlen(message));
  return result;
}

/////////////////////////////////////////////////////////

bool UClientPort::sendWithLock(const char * message)
{
  bool result;
  lock();
  result = blockSend(message, strlen(message));
  unlock();
  return result;
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

UClientPortSml::UClientPortSml()
{
/*  dataCnt = 0;
  dataNext = data;
  gotFirstTag = false;*/
  //ifName = NULL;
}


int UClientPortSml::getMoreData(char * buffer, int bufferSize, int pollTimeoutMs)
{
  return getDataFromLine(buffer, bufferSize, pollTimeoutMs);
}

////////////////////////////////////////////

bool UClientPortSml::doVerboseMessages()
{
  return verboseMessages;
}
  
  
////////////////////////////////////////////

bool UClientPortSml::isSourceAvailable()
{
  return isConnected();
}

////////////////////////////////////////////

bool UClientPortSml::outputData(const char * message)
{
  return sendWithLock(message);
}

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////



