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

#include <stdio.h>
#include <sys/utsname.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#include <ugen4/ucommon.h>
#include "usockclient.h"



USockClient::USockClient()
{
  port = 31001;
  connected = false;
  stop = true;
  sock = -1;
  sending = false;
  receiving = false;
  recvThreadRunning = false;
  connectionGotLost = false;
  pthread_mutex_init(&ptLock, NULL);
  host[0] = '\0';
  pollTimeoutMs = 200; // msec
  readCnt = 0;
}

//////////////////////////////////

USockClient::~USockClient()
{
  if (connected)
    doDisconnect();
}


//////////////////////////////////

void * runSockClient(void * obj)
{ // call connection handler in referenced object
  USockClient * cl = (USockClient *) obj;
  //
  if (cl != NULL)
    cl->handleConnection();
  //
  pthread_exit(NULL);
  return NULL;
}

////////////////////////////////////

bool USockClient::tryConnect(bool andStartReceiveThread /* =true */)
{
  int err = 0;
  struct sockaddr_in name;
  char s[MAX_SC_INFO_SIZE];
  pthread_attr_t  thConAttr;
  struct hostent * hostEntry;
  char * hostaddr;
  struct in_addr IPadr;
  bool valid;
  struct pollfd pollStatus;
  int n, t = 0;
  //
  // if connected already, then disconnect
  if (connected)
    doDisconnect();
  //
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == -1)
  {
    //perror(strerror(errno));
    sInfo(strerror(errno), scInfo);
    err = -1;
  }
  if (err == 0)
  { // resolve name
    sInfo("Socket created ... ", scInfo);
    // try convert host name to an IP number
    valid = inet_aton(host, &IPadr);
    if (not valid)
    { // host name is not an IP number, so try resolve name
      // try resolve host name to IP number
      h_errno = 0;
      hostEntry = gethostbyname(host);
      if (hostEntry != NULL)
      { // name to IP resolution succeded
        hostaddr = hostEntry->h_addr_list[0]; // first address in net byte order
        IPadr = * (struct in_addr *) hostaddr;
        // debug
/*          snprintf(s, MAX_SC_INFO_SIZE, "%s has addr %s - connecting ...",
                hostEntry->h_name,
                inet_ntoa(IPadr));
          sInfo(s, scInfo);*/
        // debug end
        // save IP address in place of host name (may be equal)
        strncpy(host, inet_ntoa(IPadr), MAX_HOST_LENGTH);
      }
      else
      { // h_errno should be set by the gethostname() function
        switch (h_errno)
        { // show error text
          case HOST_NOT_FOUND:
            sInfo("Host not found", scWarning);
            break;
          case NO_DATA:
          //case NO_ADDRESS: - same as NO_DATA
            sInfo("No IP adress found", scWarning);
            break;
          case NO_RECOVERY:
            sInfo("Error while resolving name (no recovery)", scError);
            break;
          case TRY_AGAIN:
            sInfo("Error at name server - try later.", scWarning);
            break;
          default:
            sInfo("Unknown error occured while resolving host mane.", scError);
        }
        err = -1;
      }
    }
  }
  //
  if (err == 0)
  { // set up structure for connect
    name.sin_family = AF_INET;
    name.sin_port = htons(port);
    name.sin_addr.s_addr = inet_addr(host);
    // set to non-blocking, to be able to stop
    fcntl(sock, F_SETFL, O_NONBLOCK);
    // try a connect
    err = connect(sock, (struct sockaddr *) &name, sizeof(name));
    //
    if ((err != 0) and (errno == EINPROGRESS))
    { // wait for successful status or timeout
      pollStatus.fd = sock;
      pollStatus.events = POLLOUT;
      pollStatus.revents = 0;
      stopNow = false;
      t = 0;
      while (not stopNow)
      { // poll every 50 ms for write status for this socket
        n = poll(&pollStatus, 1, 50);
        //
        if (n <= 0)
          // poll error
          break;
        if ((pollStatus.revents & POLLOUT) == POLLOUT)
        { // socket is writeable, so just continue
          err = 0;
          break;
        }
        else if (pollStatus.revents > POLLOUT)
        { // error condition no need to poll anymore
          break;
        }
        t++;
        if (t > 100) // timeout after 2 seconds
        { // timeout in error
          close (sock);
          break;
        }
      }
      if (stopNow)
        sInfo("Connection canceled by user", scWarning);
    }
    //
    if (err != 0)
    { // connection error
      if (t >= 100)
        snprintf(s, MAX_SC_INFO_SIZE, "Connect timeout (%s)", strerror(errno));
      else
        if ((pollStatus.revents & POLLHUP) == POLLHUP)
          snprintf(s, MAX_SC_INFO_SIZE, "Connect error (Hang up)");
        else if ((pollStatus.revents & POLLERR) == POLLERR)
          snprintf(s, MAX_SC_INFO_SIZE, "Connect error (PollErr - not Hang Up)");
      sInfo(s, scWarning);
    }
  }
  if (err == 0)
  { // mark as connected
    connected = true;
    // no traffic (yet)
    transmissionTraffic(false);
    // preare status poll
    recvPollStatus.fd = sock;
    recvPollStatus.events = POLLIN  +   /*  0x0001  There is data to read */
                        POLLPRI +   /*  0x0002  There is urgent data to read */
                        /* POLLOUT 0x0004  Writing now will not block */
                        POLLERR +   /*  0x0008  Error condition */
                        POLLHUP +   /*  0x0010  Hung up */
                        POLLNVAL;   /*  0x0020  not valid */
    recvPollStatus.revents = 0;  // clear returned events
  }
  if (andStartReceiveThread and (err == 0))
  {
    if (recvThreadRunning)
    { // test if thread is killed
      recvThreadRunning = false;
      // wait for receive thread to set flag again
      Wait(0.3);
      // by now the flag should be set to
      // true by thread.
      if (not recvThreadRunning)
        sInfo("Thread has been killed - creating a new", scWarning);
    }
    if (not recvThreadRunning)
    { // create thread to handle connection
      stop = false;
      // Starts socket client thread 'runSockClient'
      pthread_attr_init(&thConAttr);
      // create socket client thread
      err = pthread_create(&thClient, &thConAttr, &runSockClient, (void *)this);
      pthread_attr_destroy(&thConAttr);
    }
  }
  //
  return connected;
}

////////////////////////////////////
  /**
  Disconnect from server */
bool USockClient::doDisconnect()
{ // terminate connection and stop read thread
  bool result = false;
  //
  if (recvThreadRunning)
  {
    stop = true;
    // wait for client read thread to stop
    pthread_join(thClient, NULL);
    // close socket connection
    closeConnection();
    // report status
    sInfo("Disconnected", scInfo);
    result = true;
  }
  /*
  else
    sInfo("Not connected (no read thread)", scInfo);
  */
  //
  return result;
}


////////////////////////////////////
/**
  Send this buffer of data */
bool USockClient::blockSend(const char * buffer, int length)
{
  bool result;
  int d = 0, j, n = 0;
  char s[MAX_SC_INFO_SIZE];
  struct pollfd pollStatus;
  int err;
  //
  // preare status poll
  pollStatus.fd = sock;
  pollStatus.events = POLLOUT +   /*  0x0004  */
                      POLLERR +   /*  0x0008  Error condition */
                      POLLHUP +   /*  0x0010  Hung up */
                      POLLNVAL;   /*  0x0020  not valid */
  pollStatus.revents = 0;
  err = pthread_mutex_lock(&ptLock);
  result = (err == 0);
  if (result)
  { // get status
    j = poll(&pollStatus, 1, 50);
    if ((j > 0) and ((pollStatus.revents & 0x0038) > 0))
    { // not POLLOUT, connection lost
      sInfo("Connection lost", scWarning);
      doDisconnect();
    }
    // status out is not used, if no error, then just try
    if (connected)
    { // still connected (no error (yet))  buffer[4]
      for (j = 0; j < 10; j++)
      {
        sending = true;
        transmissionTraffic(sending or receiving);
        n = send(sock, &buffer[d], length - d, 0);
        d += n;
        if ((d == length) or not connected)
          // may be disconnected by read status
          break;
      }
    }
    if (sending)
    {
      sending = false;
      transmissionTraffic(sending or receiving);
    }
    // release for next transmission
    pthread_mutex_unlock(&ptLock);
    result = (d == length);
    if (connected and false)
    {
      snprintf(s, MAX_SC_INFO_SIZE, "Send %d bytes of '%s'", d, buffer);
      sInfo(s, scDebug);
    }
  }
  else
    perror("USockClient::blockSend: ptherad_mutex_lock returned error");
  //
  return result;
}

//////////////////////////////////////////////////////

void USockClient::handleConnection(bool terminateOnDisconnect /* = false */)
{ // this is the socket read thread
  // the socket remains open after connection is closed
  // and then handles next connection when it is accepted
  //int len;
  //int err;
  // bool dataAvailable;
  //
  // socket handle not valid - terminate
  if (sock < 0)
    stop = true;
  //
  // debug
  sInfo("Connection thread listening", scWarning);
  // debug end
  // continue reading until ordered to stop
  while (not stop)
  { // set listening for every loop
    recvThreadRunning = true;
    if (connected)
    { // listen to socket
      //err = 0;
      // dataAvailable = getDataFromLine(200);
      /*
      connectionGotLost = false;
      dataAvailable = false;
      //
      // get connection status and wait for up to 200 ms
      // if nothing to report
      if (err == 0)
        err = poll(&recvPollStatus, 1, 200);
      if (err < 0)
      { // could not poll - no connection
        sInfo("Connection lost (poll err)", scWarning);
        connectionGotLost = true;
      }
      else if (recvPollStatus.revents != 0)
      { // status is not timeout
        if ((recvPollStatus.revents & POLLIN) > 0)
          // normal data available
          dataAvailable = true;
        else if ((recvPollStatus.revents & POLLPRI) > 0)
          // priority data
          dataAvailable = true;
        else if ((recvPollStatus.revents & POLLOUT) > 0)
          Wait(0.1); // should not be (not masked), but such shit happens?
        else if (((recvPollStatus.revents & POLLERR) > 0) or
            ((recvPollStatus.revents & POLLHUP) > 0) or
            ((recvPollStatus.revents & POLLNVAL) > 0))
        { // error situation (error, hangup or not valid)
          sInfo("Connection lost (poll)", scWarning);
          connectionGotLost = true;
        }
        else
        { // error situation (error, hangup or not valid)
          sInfo("Connection lost (unknown poll error)", scWarning);
          connectionGotLost = true;
        }
      }
      if (dataAvailable)
      { // data available to read
        // just get what is available up to buffer length
        len = recv(sock, buf, MAX_BUFFER_LENGTH_RX - 1, 0);
        if (len > 0)
        { // process result
          if (not receiving)
          { // inform about receive status (led on)
            receiving = true;
            transmissionTraffic(sending or receiving);
          }
          // terminate (if string) and process
          buf[len] = 0;
          processMessage(buf, len);
          //
        }
        if ((len <= 0) and (errno != EAGAIN))
        { // error - connection is lost       h_errno
          sInfo("Connection is lost (conn error)", scWarning);
          connectionGotLost = true;
        }
      }
      //
      if (not dataAvailable)
      { // no data, so call idle function
        // to issue new commands
        pollTimeout();
      }
      // turn off led if no more traffic
      if (receiving and not dataAvailable)
      { // change in status - turn off led
        receiving = false;
        transmissionTraffic(sending or receiving);
      }
      if (connectionGotLost and connected)
        // shut down connection properly
        closeConnection();
      if (connected and terminateOnDisconnect)
        stop = true;
      */
    }
    else
    { // not connected, so just wait a bit
      Wait(0.161);
    }
  }
  if (receiving)
  { // turn off traffic led
    receiving = false;
    transmissionTraffic(sending or receiving);
  }
  // debug
  sInfo("Connection thread stopped", scWarning);
  // debug end
  // shutdown connection
  recvThreadRunning = false;
  if (connected)
    closeConnection();
}

////////////////////////////////////////////////////////

bool USockClient::getDataFromLine(int pollTimeoutMs)
{
  int err;
  bool dataAvailable;
  int len;
  //
  err = 0;
  connectionGotLost = false;
  dataAvailable = false;
  // get connection status
  if (err == 0)
    err = poll(&recvPollStatus, 1, pollTimeoutMs);
  if (err < 0)
  { // could not poll - no connection
    sInfo("Connection lost (poll err)", scWarning);
    connectionGotLost = true;
  }
  else if (recvPollStatus.revents != 0)
  { // status is not timeout
    if ((recvPollStatus.revents & POLLIN) > 0)
      // normal data available
      dataAvailable = true;
    else if ((recvPollStatus.revents & POLLPRI) > 0)
      // priority data
      dataAvailable = true;
    else if ((recvPollStatus.revents & POLLOUT) > 0)
      Wait(0.1); // should not be (not masked), but such shit happens?
    else if (((recvPollStatus.revents & POLLERR) > 0) or
        ((recvPollStatus.revents & POLLHUP) > 0) or
        ((recvPollStatus.revents & POLLNVAL) > 0))
    { // error situation (error, hangup or not valid)
      sInfo("Connection lost (poll)", scWarning);
      connectionGotLost = true;
    }
    else
    { // error situation (error, hangup or not valid)
      sInfo("Connection lost (unknown poll error)", scWarning);
      connectionGotLost = true;
    }
  }
  if (dataAvailable)
  { // data available to read
    // just get what is available up to buffer length
    len = recv(sock, buf, MAX_BUFFER_LENGTH_RX - 1, 0);
    if (len > 0)
    { // process result
      if (not receiving)
      { // inform about receive status (led on)
        receiving = true;
        transmissionTraffic(sending or receiving);
      }
      // terminate (if string) and process
      buf[len] = 0;
      processMessage(buf, len);
      //
    }
    if ((len <= 0) and (errno != EAGAIN))
    { // error - connection is lost       h_errno
      sInfo("Connection is lost (conn error)", scWarning);
      connectionGotLost = true;
    }
  }
  //
  if (not dataAvailable)
  { // no data, so call idle function
    // to issue new commands
    pollTimeout();
  }
  // turn off led if no more traffic
  if (receiving and not dataAvailable)
  { // change in status - turn off led
    receiving = false;
    transmissionTraffic(sending or receiving);
  }
  if (connectionGotLost and connected)
    // shut down connection properly
    closeConnection();
  //
  return dataAvailable;
}

///////////////////////////////////////////////////

void USockClient::closeConnection()
{ // shutdown connection -- tell the other end
  connected = false;
  if (sock > 0)
  {
    transmissionTraffic(false);
    shutdown(sock, SHUT_RDWR);
    close(sock);
    sock = -1;
    // inform at higher levels
    connectionLost();
  }
}

////////////////////////////////////////////////////

void USockClient::connectionLost()
{ // should be overwritten at higher levels
  sInfo("Connection shutdown", scInfo);
}

////////////////////////////////////////////////////

void USockClient::processMessage(unsigned char * message, int length)
{ // this should be overwritten by a message user
  // if not owerwritten the message will be printed as
  // a character message.
  char s[MAX_SC_INFO_SIZE];
  char s2[MAX_SC_INFO_SIZE];
  // NB! should not be
  strncpy(s, (char *) message, mini(length, MAX_SC_INFO_SIZE-1));
  s[mini(length, MAX_SC_INFO_SIZE - 1)] = 0;
  snprintf(s2, MAX_SC_INFO_SIZE, "Message not used '%s'\n", s);
  sInfo(s2, scDebug);
}

////////////////////////////////////////////////////

void USockClient::sInfo(const char * message, int type)
{ // should probably be overwritten and handled at a
  // higher level.
  // at this level just print at console
  printf("(%d) : %s\n", type, message);
}

////////////////////////////////////////////////////

void USockClient::transmissionTraffic(bool traffic)
{ // called whenever something is received or transmitted
  // at start and stop
  // if used this function should be overwritten
  if (false)
  {
    if (traffic)
      printf("Traffic LED is on\n");
    else
      printf("Traffic LED is off\n");
  }
}

////////////////////////////////////////////////////

void USockClient::pollTimeout()
{ // should be overwritten by something else
  printf(".");
}
