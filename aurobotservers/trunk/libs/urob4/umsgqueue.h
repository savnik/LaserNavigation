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

#ifndef UMSGQUEUE_H
#define UMSGQUEUE_H

#include <pthread.h>

#include <ugen4/ulock.h>

/**
  *@author Christian Andersen
  */

class UMsgQueue : public ULock
{
public:
  /**
  Constructor */
  UMsgQueue();
  /**
  Destructor */
  ~UMsgQueue();
  /**
  Allocate queue space for a queue of this size.
  NB! allocates memory on heap, so do not allocate and free
  too much, if a real-time system is desired.
  Returns true if allocated. */
  bool create(int iElements, int iElementSize);
  /**
  Deallocate memory occupied by queue */
  void freeQueue();
  /**
  Get number of elements.
  Returns size of queue in elements. */
  int getElements();
  /**
  Returns size of each element. */
  int getElementSize();
  /**
  Returns a reference to the next available message.
  Returns NULL if queue is empty.
  NB! the message until 'skipMessage()' is called. */
  unsigned char * getOutMessageRef();
  /**
  Actual message is delt with, skip to next unread message.
  Return pointer to next message or
  Returns NULL if no next message or the queue is invalid. */
  //unsigned char * skipToNextMessage();
  /**
  Actual message is delt with, skip to next unread message.
  Return pointer to next message or
  Returns NULL if no next message or the queue is invalid.
  If 'checkForDublicates' then the message queue is
  inspected for diplicates ignoring the sequence number in
  msg[2], but comparing length msg[0..1] and msg[3..length].
  if a duplicate is found, the this first message is skipped
  and the next is serviced (and checked for dublicates). */
  unsigned char * skipToNextMessage(bool checkForDublicates);
  /**
  Add a message to the queue.
  Returns true if added. owerwrites an old message if no space left. */
  bool addMessage(unsigned char msg[], int size);
  /**
  Clears the queue to an empty state.
  Returns true if the queue is valid. */
  bool clear();
  /**
  Returns true if the queue is valid */
  inline bool isValid() { return valid; };
  /**
  Returns true if queue is empty or not valid.
  Returns false if not empty. */
  bool isEmpty();
  /**
  Returns number of elements waiting in queue
  Returns -1 if queue is not valid. */
  int getUsedMsgCnt();
  /**
  Returns true if queue is full.
  Returns false if queue is not empty or invalid. */
  bool isFull();
  /**
  Get queue element index number for next message to read */
  inline int getNextOut()
  { return nextOut; };
  
protected:
  /**
  Get a reference to queue element 'i'.
  Returns a pointer to the message queue if 'i' is valid, else
  Returns NULL requested element is invalid (or queue not allocated) */
  unsigned char * getMsg(int i);
  /**
  Lock the queue pointers.
  Returns true if locked. */
  //bool lock();
  /**
  Unlocks the queue. */
  //void unlock();
  //
protected:
  /**
  Mutex lock for meassge queue */
  //pthread_mutex_t mLock; // pthread_mutex_lock
  /**
  Array of pointers to messages */
  unsigned char ** pMsg; //
  /**
  Number of elements (buffers) available (filled or not) in queue */
  int elements;
  /**
  Size of each element in queue */
  int elementSize;
  /**
  Queue is created successfully */
  bool valid;
  /**
  Number of queue element just inserted */
  int justIn;  // just inserted element
  /**
  Next element to use */
  int nextOut; // next element to read
};

#endif
