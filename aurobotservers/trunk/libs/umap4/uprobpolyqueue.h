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
#ifndef UPROBPOLYQUEUE_H
#define UPROBPOLYQUEUE_H

#include <urob4/umsgqueue.h>
#include "uprobpoly.h"

/**
size of polygon stack for incoming polygons */
#define MAX_POLYGON_QUEUE_ELEMENTS 20

/**
Queue for incoming free path messages.

@author Christian Andersen
*/
class UProbPolyQueue : public UMsgQueue
{
public:
  /**
  Constructor */
  UProbPolyQueue();
  /**
  Destructor */
  virtual ~UProbPolyQueue();
  /**
  Gets a pointer to the next free polygon
  if none is free then the odest will be
  overwritten.
  The message is locked on return. */
  UProbPoly * lockNextIn();
  /**
  Unlocks the next message after modifiing
  the content. if valid4use the message
  is added to the queue and the in-point
  is advanced. */
  void unlockNextIn(bool valid4Use);
  /**
  Gets a pointer to the next polygon to use
  of queue is empty a NULL pointer is returned.
  The queue is locked on return. */
  UProbPoly * lockNextOut();
  /**
  Unlocks queue after message use */
  void unlockNextOut();
  /**
  print buffer information to console */
  void print(const char * preStr);
protected:
  /**
  Get oldest element and remove it from queue. */
  UProbPoly * skipToNext();
};

#endif
