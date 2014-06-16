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
#include "uprobpolyqueue.h"

////////////////////////////////////////////////

UProbPolyQueue::UProbPolyQueue()
 : UMsgQueue()
{
  create(MAX_POLYGON_QUEUE_ELEMENTS, sizeof(UProbPoly));
}

////////////////////////////////////////////////

UProbPolyQueue::~UProbPolyQueue()
{
}

////////////////////////////////////////////////

UProbPoly * UProbPolyQueue::lockNextIn()
{
  UProbPoly * result = NULL;
  int n;
  //
  if (valid)
  { // lock queue before add
    if (lock())
    { // queue state is now fixed
      // always space for next message
      n = (justIn + 1) % elements;
      result = (UProbPoly * )getMsg(n);
      result->setValid(false);
      if (isFull())
        //skip oldest message
        nextOut = (nextOut + 1) % elements;
    }
  }
  return result;
}

////////////////////////////////////////////////

void UProbPolyQueue::unlockNextIn(bool valid4Use)
{
  UProbPoly * pe;
  int n;
  //
  if (valid)
  { // lock queue before add
    if (valid4Use)
    { // queue state is now fixed
      // always space for next message
      n = (justIn + 1) % elements;
      pe = (UProbPoly * )getMsg(n);
      pe->setValid(true);
      justIn = n;
    }
    unlock();
  }
}

////////////////////////////////////////////////

UProbPoly * UProbPolyQueue::lockNextOut()
{
  UProbPoly * result = NULL;
  //
  if (valid and not isEmpty())
    // lock queue before add
    if (lock())
      // queue state is now fixed
      result = skipToNext();
  return result;
}

////////////////////////////////////////////////

void UProbPolyQueue::unlockNextOut()
{
  unlock();
}

////////////////////////////////////////////////

UProbPoly * UProbPolyQueue::skipToNext()
{ // check for duplicates not supported yet
  return (UProbPoly *) UMsgQueue::skipToNextMessage(false);
}

////////////////////////////////////////////////

void UProbPolyQueue::print(const char * preStr)
{
  int i, j;
  UProbPoly * m;
  int n;
  const int SL = 20;
  char s[SL];
  //
  if (isEmpty())
    printf("%s queue is empty (out %d, in %d)\n", preStr, nextOut, justIn);
  else
  {
    n = getUsedMsgCnt();
    printf("%s queue has %d/%d used elements each %d bytes (out %d, in %d)\n",
          preStr, n, getElements(), getElementSize(), nextOut, justIn);
    i = nextOut;
    for (j = 0; j < n; j++)
    {
      i = (i + 1) % elements;
      m = (UProbPoly *) getMsg(i);
      snprintf(s, SL, " - #%02d ", i);
      m->print(s, false);
    }
  }
}
