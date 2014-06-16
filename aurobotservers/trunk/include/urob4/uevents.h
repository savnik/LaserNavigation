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
#ifndef UEVENTS_H
#define UEVENTS_H

#include "ueventtrap.h"

/**
List of events

@author Christian Andersen
*/
class UEvents
{
public:
  /**
  Constructor */
  UEvents(int maxEventTraps);
  /**
  Destructor */
  ~UEvents();
  /**
  Add event trap
  Takes a event trigger key, a function to call, and
  an object to use as first parameter in the call.
  Returns a serial number for the trap, so that it can be deleted
  using this key.
  If no more space the returned serial is 0;*/
  unsigned long add(const char * key,
                    EVENT_CALL onEvent,
                    void * obj);
  /**
  Delete a trap (mark as invalid) */
  void del(unsigned long serial);
  /**
  Get a pointer to event compatible with this key
  If 'fromEvent != NULL, then the next event after fromEvent is found.
  Returns NULL if no event is found. */
  UEventTrap * getEvent(const char * key, UEventTrap * fromEvent);

protected:
  /**
  Events */
  UEventTrap * events;
  /**
  Number of active events */
  int eventsCnt;
  /**
  Maximum number of events allocated */
  int eventsMax;
  /**
  Event serial number */
  unsigned int eventSerial;
};

#endif
