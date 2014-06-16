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

#include <string.h>

#include "ueventtrap.h"

UEventTrap::UEventTrap()
{
  key[MAX_TRAP_KEY_LENGTH] = '\0';
  object = NULL;
  trapCall = NULL;
  valid = false;
  serial = 0;
}

//////////////////////////////////////

UEventTrap::~UEventTrap()
{
}

//////////////////////////////////////

void UEventTrap::set(unsigned long id,
                    const char * onKey,
                    EVENT_CALL onEvent,
                    void * obj)
{
  serial = id;
  object = obj;
  trapCall = onEvent;
  strncpy(key, onKey, MAX_TRAP_KEY_LENGTH);
  key[MAX_TRAP_KEY_LENGTH] = '\0';
}

//////////////////////////////////////

bool UEventTrap::isThis(const char * compKey)
{
  bool result;
  //
  result = (strcasecmp(key, compKey) == 0);
  //
  return result;
}

///////////////////////////////////////

// void doTheFuckingTrapCall(EVENT_CALL onEvent, void * obj, void * data)
// {
//   onEvent(obj, data);
// }

bool UEventTrap::callTrap(void * data)
{
  return trapCall(object, data);
  //doTheFuckingTrapCall(trapCall, object, data);
}
