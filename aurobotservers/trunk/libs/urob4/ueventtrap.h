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
#ifndef UEVENTTRAP_H
#define UEVENTTRAP_H

#include <stdio.h>

/**
Event trap, that can host a client for an event, so that data can be returned, when the event occurs.

@author Christian Andersen
*/

class UEventTrap;

typedef bool (*EVENT_CALL)(void * obj, void * data);


class UEventTrap
{
public:
  /**
  Constructor */
  UEventTrap();
  /**
  Destructor */
  ~UEventTrap();
  /**
  Set trap date.*/
  void set(unsigned long id,
           const char * onKey,
           EVENT_CALL onEvent,
           void * obj);
  /**
  Is event trap valid */
  inline bool isValid()
  { return valid; };
  /**
  Set event invalid */
  inline void setInvalid()
  { valid = false; };
  /**
  Get event ID */
  inline unsigned long getID()
  { return serial; };
  /**
  Returns true if this eventkey is compatible with the parameter key */
  bool isThis(const char * compKey);
  /**
  Call the trap function with these data */
  bool callTrap(void * data);

public:
  /**
  Max length of keyword string */
  static const int MAX_TRAP_KEY_LENGTH = 100;

protected:
  /**
  Is the trap valid (not deleted) */
  bool valid;
  /**
  Function to call */
  EVENT_CALL trapCall;
  /**
  Object to reply to */
  void * object;
  /**
  Ketword that should trigger the event */
  char key[MAX_TRAP_KEY_LENGTH + 1];
  /**
  ID if the trap, uset to delete the trap. */
  unsigned long serial;
};

///////////////////////////////////////////////////////////////////







#endif
