/***************************************************************************
 *   Copyright (C) 2009 by Christian Andersen   *
 *   jca@elektro.dtu.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef URAWSERVERPORT_H
#define URAWSERVERPORT_H

#include <urob4/userverport.h>

#define __SERVER_VERSION__ "2.1966"

void shutDownHandler(int signal);

/**
This adds some setup to the userver server port and message queue

	@author Christian Andersen <jca@elektro.dtu.dk>
*/
class URawServerPort : public UServerPort
{
public:
  /**
   * Constructor */
  URawServerPort();
  /**
   * Destructor */
  ~URawServerPort();
  /**
   * Run the server console prompt */
  bool runServer(const char * xmlTag);
  /**
   * Run the receive queue empty thread */
  void run();
  /**
   * Terminate server */
  void terminate();

protected:
  /**
   * start the thread that empties the rx queue */
  void startQueueEmpty();
  /**
   * stop the thread that empties the rx queue */
  void stopQueueEmpty(bool andWait);
  /**
   * Print one message from queue */
  bool handleOneMessageFromQueue();
  /**
   * Add line to command history queue */
  bool addCmdHist(const char * line, char * lineHist[],
                  const int MHL, const int MLL,
                  int * lineHistCnt, int * lineHistNewest);
  /**
  This function is called, when a new client is connected */
  virtual void gotNewClient(UServerClient * cnn);

private:

  /**
  Handle to queue empty thread */
  pthread_t  thQE;
  /**
   * is queue emptying running */
  bool qeRunning;
  /**
   * should queue emptying stop */
  bool qeStop;
  /**
   * console print lock */
  ULock consoleLock;
public:
  /**
   * Is the server stopping aælready */
  bool isStopping;
  /**
   * should log be timestamped */
  bool logTimeStamp;
};

#endif
