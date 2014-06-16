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
#include "ureslink.h"

UResLink::UResLink()
{
  poseHist = NULL;
  poseHistStreamingClient = -1;
}

////////////////////////////////////////////////////

UResLink::~UResLink()
{
}

////////////////////////////////////////////////////

bool UResLink::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  //
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  {
    poseHistLock.lock();
    if (remove)
      // resource module unloaded
      poseHist = NULL;
    else if (poseHist != (UResPoseHist *)resource)
      // resource changed or new
      poseHist = (UResPoseHist *)resource;
    else
      // no change - i.e. not used
      result = false;
    poseHistLock.unlock();
  }
  //
  return result;
}

////////////////////////////////////////////////////

void UResLink::connectionLost(int client)
{
  if (poseHistStreamingClient == client)
  {
    poseHistStreamingClient = -1;
    // debug
    // @todo test this branch - lost pose streaming client
    printf("UResLink::connectionLost: Streaming stopped from client %d\n", client);
    // debug end
  }
}
