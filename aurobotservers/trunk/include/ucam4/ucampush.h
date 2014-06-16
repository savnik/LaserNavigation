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
#ifndef UCAMPUSH_H
#define UCAMPUSH_H

#include <urob4/ucmdexe.h>

#include "ucammount.h"

/**
Camera that can push functions. I.e. send a command to the queue every time an image is detected.

@author Christian Andersen
*/
class UCamPush : public UCamMounted, public UServerPush
{
public:
  /**
  Constructor */
  UCamPush(UCamDevBase * dev);
  /**
  destructor */
  ~UCamPush();
  /**
  A new image is received.
  NB! is called bythe read-frame thread for the camera. */
  inline virtual void gotNewImage(UImage * raw)
  { gotNewData(raw); }
  /**
  Add a camera push command.
  A camera push command is triggered when a
  new image is available. */
  inline int addCamPushCommand(UServerInMsg * msg)
  { return addPushCommand(msg); };
  /**
  Debug print of camera status */
  void print(const char * preString);
  /**
  Print status to string buffer. */
  const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Request to test push commands for need of new data.
  If new data is needed then 'gotNewImage()' should be called. */
  inline virtual bool needNewPushData()
  { return UServerPush::needNewData(); }
  /**
  Called from main server thread, when ready to handle push command.
  But need (optional) push object, so this function must (eventually) call 'gotNewData(object)' */
  inline virtual void callGotNewDataWithObject()
  {
    if (dev == NULL)
      gotNewData(NULL);
    else
      dev->callGotNewDataWithObject();
  }
  /**
  Image is updated now, note this in client handler */
  inline virtual void imgUpdated()
  { // tell push system that a new image event has occured
    setUpdated("");
  };

private:
};




#endif
