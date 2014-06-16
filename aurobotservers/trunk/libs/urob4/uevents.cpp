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

#include <stdlib.h>

#include <ugen4/ucommon.h>

#include "uevents.h"

//////////////////////////////

UEvents::UEvents(int maxEventTraps) 
{
  eventsMax = maxEventTraps;
  events = (UEventTrap *)malloc(eventsMax * sizeof(UEventTrap));
  eventsCnt = 0;
  eventSerial = 1;
}


UEvents::~UEvents()
{ // deallocate memory used
  if (events != NULL)
    free(events);
}

///////////////////////////////////////////////////

unsigned long UEvents::add(const char * key, EVENT_CALL onEvent, void * obj)
{
  int n;
  UEventTrap * et = events;
  unsigned long id;
  //
  for (n = 0; n < eventsCnt; n++)
  {
    if (not et->isValid())
      break;
    et++;
  }
  if (n < eventsMax)
  { // there is space to add new event
    id = ++eventSerial;
    et->set(id, key, onEvent, obj);
    eventsCnt = maxi(n + 1, eventsCnt);
  }
  else
    id = 0;
  return id;
}

/////////////////////////////////////////////////

void UEvents::del(unsigned long serial)
{
  int n;
  UEventTrap * et = events;
  //
  for (n = 0; n < eventsCnt; n++)
  {
    if (et->isValid() and (et->getID() == serial))
    {
      et->setInvalid();
      break;
    }
    et++;
  }
}

///////////////////////////////////////////////

UEventTrap * UEvents::getEvent(const char * key, UEventTrap * fromEvent)
{
  int n, f;
  UEventTrap * et = events;
  //
  if (fromEvent < events)
    // get first compatible event
    f = 0;
  else
  { // get event number
    f = (fromEvent - events)/ sizeof(UEventTrap);
    f++; // advance to first untested
    et = fromEvent + 1;
  }
  for (n = f; n < eventsCnt; n++)
  { // test compatibility
    if (et->isValid() and et->isThis(key))
      break;
    et++;
  }
  if (n < eventsCnt)
    return et;
  else
    return NULL;
}

