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
#ifndef URESLINK_H
#define URESLINK_H

#include <ugen4/ulock.h>

#include "uresposehist.h"

/**
Small class to hold ressource link pointer and a lock to ensure that the ressource is valid while used

@author Christian Andersen
*/
class UResLink : public ULock
{
public:
  /**
  Constructor */
  UResLink();
  /**
  Destructor */
  ~UResLink();
  /**
  Get link to pose hist in a locked state */
  UResPoseHist * lockPoseHist()
  {
    poseHistLock.lock();
    return poseHist;
  };
  /**
  Unlock pose histort resource */
  void unlockPoseHist()
  { poseHistLock.unlock(); };
  /**
  Set ressource (or remove) ressource link.
  Returns true if the ressource situation is changed. */
  bool setResource(UResBase * resource, bool remove);
  /**
  Is pose beeing streamed from MRC */
  bool isStreamingPoseHist()
  { return (poseHistStreamingClient >= 0); };
  /**
  Set pose streaming client */
  void setStreamingPoseHist(int client)
  { poseHistStreamingClient = client; };
  /**
  Connection is lost to this client - if ant cleanup is needed */
  void connectionLost(int client);
  
protected:
  /**
  Ressource lock for the pose hist ressource link. */
  ULock poseHistLock;
  /**
  Link to pose hist ressource */
  UResPoseHist * poseHist;
  /**
  Is pose hist streaming beeing received */
  int poseHistStreamingClient;
};


#endif
