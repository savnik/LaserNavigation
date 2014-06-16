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
#ifndef UPUSH_H
#define UPUSH_H

#include "ucmdexe.h"

/**
Functions to add, delete and use a push queue

@author Christian Andersen
*/
class UPush : public UServerPushQueue
{
public:
  /**
  Constructor */
  UPush();
  /**
  Destructor */
  virtual ~UPush();
  /**
  A new image is received.
  NB! is called by some function, when push commands
  arer to be send (evaluated). */
  virtual void pushEvent(void * pushObject);
  /**
  Set command executor reference for camera push functions */
  inline void setCmdExe(UCmdExe * executor)
  { cmdExe = executor; } ;
  /**
  Add a push command.
  A push commands get triggered by a call to
  pushEvent(pushObj).
  Returns */
  int addPushCommand(UServerInMsg * msg);
  /**
  Flush all pending commands from this client */
  void flushClientCmds(int clientIdx);

protected:
  /**
  Pointer to push server queue */
  UCmdExe * cmdExe;

};

#endif
