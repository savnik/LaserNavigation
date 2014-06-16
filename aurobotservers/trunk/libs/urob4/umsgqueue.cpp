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
#include <string.h>
#include <stdio.h>

#include "umsgqueue.h"

UMsgQueue::UMsgQueue()
{
  valid = false;
}

//////////////////////////////////////////////

UMsgQueue::~UMsgQueue()
{
  freeQueue();
}

//////////////////////////////////////////////

void UMsgQueue::freeQueue()
{
  int i;
  //
  if (valid)
  { // free allocated memory
    valid = false;
    for (i = 0; i < elements; i++)
      free(pMsg[i]);
    // and free queuePointer
    free(pMsg);
  }
}

////////////////////////////////////////////////

bool UMsgQueue::create(int iElements, int iElementSize)
{ // allocate memory for queue
  bool result = false;
  int i, last;
  //
  if (not valid)
  { // init lock
    //pthread_mutex_init(&mLock, NULL);
    elements = iElements;
    elementSize = iElementSize;
    // get memory for array of pointers to messages
    pMsg = (unsigned char **) malloc(sizeof(unsigned char *) * elements);
    if (pMsg != NULL)
    { //
      last = -1;
      for (i = 0; i < elements; i++)
      { // reserve heap space for each message buffer
        pMsg[i] = (unsigned char *) malloc(sizeof(unsigned char) * elementSize);
        if (pMsg[i] == NULL)
        { // request failed
          last = i;
          break;
        }
      }
      if (last < 0)
        // all memory reserved
        result = true;
      else
      { // could not get all requested memory, free the rest
        for (i = 0; i < last; i++)
          free(pMsg[i]);
        // free also pointer area
        free(pMsg);
      }
    }
    // clear message pointers to empty state
    clear();
    // set valid if queues are created
    valid = result;
  }
  return result;
}

////////////////////////////////////////////////

unsigned char * UMsgQueue::getMsg(int i)
{
  unsigned char * result = NULL;
  //
  if (valid and (i >= 0) and (i < elements))
    result = pMsg[i];
  //
  return result;
}

////////////////////////////////////////////////

int UMsgQueue::getElements()
{
  int result = -1;
  //
  if (valid)
    result = elements;
  //
  return result;
}

////////////////////////////////////////////////

int UMsgQueue::getElementSize()
{
  int result = -1;
  //
  if (valid)
    result = elementSize;
  //
  return result;
}

////////////////////////////////////////////////

unsigned char * UMsgQueue::getOutMessageRef()
{
  unsigned char * result = NULL;
  //
  if (valid)
  { // no need to lock
    if (not isEmpty())
    { // not empty - get next
      result = getMsg(nextOut);
    }
  }
  //
  return result;
}

////////////////////////////////////////////////
/*
unsigned char * UMsgQueue::skipToNextMessage()
{
  unsigned char * result = NULL;
  //
  if (valid)
    if (lock())
    { // state is now stable
      if (not isEmpty())
      { // move pointer to next available element
        nextOut = (nextOut + 1) % elements;
        // and return message
        result = getMsg(nextOut);
      }
      unlock();
    }
  //
  return result;
}
*/
unsigned char * UMsgQueue::skipToNextMessage(bool checkForDublicates)
{
  unsigned char * result = NULL;
  unsigned char * newer = NULL;
  int len;
  int i;
  int match = 0;
  //
  if (valid)
    /*! @note UMsgQueue::skipToNextMessage no longer locks the queue while
              getting pointer to next message.
              This must be done by the funsction using the pointer to the data.
              13 feb 2005/chr */
    //if (lock())
    { // state is now stable
      if (not isEmpty())
      { // move pointer to next available element
        while (match == 0)
        { // find message
          nextOut = (nextOut + 1) % elements;
          // and return message
          result = getMsg(nextOut);
          match = -1;
          if (checkForDublicates and not isEmpty())
          { // test for dublicates
            len = (result[1] << 8) + result[0];
            i = nextOut;
            do
            { // try all in queue (until match)
              i = (i + 1) % elements;
              newer = getMsg(i);
              if (newer != NULL)
                if (result[3] == newer[3])
                { // same type
                  match = strncmp((char *)&result[3],
                                  (char *)&newer[3], len - 3);
                  if (match == 0)
                  { // duplicate found
                    break;
                  }
                }
            } while (i != justIn);
            if (match == 0)
              printf("Duplicate message %d and %d (in %d, out %d)\n",
                       nextOut, i, justIn, nextOut);
          }
        }
      }
      //unlock();
    }
  //
  return result;
}

////////////////////////////////////////////////

bool UMsgQueue::addMessage(unsigned char msg[], int size)
{
  bool result = false;
  unsigned char * nMsg;
  //
  if (valid)
  { // lock queue before add
    if (lock())
    { // queue state is now locked
      if (isFull())
      { //skip oldest message
        nextOut = (nextOut + 1) % elements;
        fprintf(stderr, "*** UMsgQueue::addMessage -- queue overflow - skipped oldest\n");
      }
      if (not isFull() and (size <= elementSize))
      {
        justIn = (justIn + 1) % elements;
        nMsg = getMsg(justIn);
        if (nMsg != NULL)
        {
          memcpy(nMsg, msg, size);
          result = true;
        }
      }
      unlock();
    }
  }
  //
  return result;
}

////////////////////////////////////////////////

bool UMsgQueue::clear()
{
  justIn = 0;
  nextOut = 0;
  return true;
}

////////////////////////////////////////////////

bool UMsgQueue::isFull()
{
  bool result = true;
  int i;
  //
  if (valid)
  {
    i = (justIn + 1) % elements;
    result = (i == nextOut);
  }
  //
  return result;
}

////////////////////////////////////////////////

bool UMsgQueue::isEmpty()
{
  bool result = true;
  //
  if (valid)
    result = (justIn == nextOut);
  //
  return result;
}

////////////////////////////////////////////////

int UMsgQueue::getUsedMsgCnt()
{
  int result = -1;
  //
  if (valid)
  { // distance is also used elements
    result = justIn - nextOut;
    // check for not in same cycle
    if (result < 0)
      result += elements;
  }
  return result;
}

////////////////////////////////////////////////

// bool UMsgQueue::lock()
// {
//   bool result = false;
//   result = (pthread_mutex_lock(&mLock) == 0);
//   return result;
// }
// 
// ////////////////////////////////////////////////
// 
// void UMsgQueue::unlock()
// {
//   pthread_mutex_unlock(&mLock);
// }

