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

#include <stdio.h>
#include <string.h>

#include <ugen4/ucommon.h> // mini()

#include "userverqueue.h"

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////


bool UServerInMsg::setMessage(int clientIdx, const char * msg, int msgSize, bool raw)
{
  int i;
  const char * cs;
  char * cd;
  bool preWhiteSpace;
  int  nonWhiteSpace = 0;
  //
  client = clientIdx;
  size = 0;
  serverPushCommand = false;
  // then message and size
  // trim for whitespace before and after
  cs = msg;
  cd = message;
  rxTime.now();
  preWhiteSpace = true;
  for (i = 0; i < msgSize; i++)
  {
    if (*cs > ' ')
      preWhiteSpace = false;
    if (not preWhiteSpace)
    { // move to message buffer
      *cd++ = *cs;
      size++;
      if (size > MAX_MESSAGE_LENGTH_TO_CAM)
      { // text message is too long
        fprintf(stderr, "*** UServerMsg::setMessage Too long message - truncated\n");
        break;
      }
    }
    if (*cs > ' ')
      // save position of last good character
      nonWhiteSpace = size;
    cs++;
  }
  // truncate end white space off
  size = nonWhiteSpace;
  // terminate with zero (buffer has 1 byte oversize)
  message[size] = 0;
  // decode the name and type of the message
  if (raw)
    tag.setValid(false);
  else
    tag.setTag(message);
  // debug
/*  if (not tag.isTagEndFound())
  {
    printf("*** tag end not found in '%s'\n", message);
  }*/
  //
  return (size > 0);
}

////////////////////////////////////////////////

void UServerInMsg::print(const char * preStr)
{
  printf("%s from client %d, size %3d is '%s'\n",
      preStr, client, size, message);
}

////////////////////////////////////////////////

const char * UServerInMsg::print(const char * preStr, char * buff, const int buffCnt)
{
  snprintf(buff, buffCnt, "%s %lu.%06lu client %2d, size %3d/%d is '%s'\n",
         preStr, rxTime.getSec(), rxTime.getMicrosec(), client,
         size, MAX_MESSAGE_LENGTH_TO_CAM, message);
  return buff;
}

////////////////////////////////////////////////

UServerInMsg UServerInMsg::operator= (UServerInMsg source)
{
  setMessage(source.client, source.message, source.size, not source.tag.isValid());
  serverPushCommand = source.serverPushCommand;
  return *this;
}

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

UServerInQueue::UServerInQueue()
{ // create queue
  create(MAX_SOCKET_RECEIVE_QUEUE, sizeof(UServerInMsg));
}

////////////////////////////////////////////////

UServerInQueue::~UServerInQueue()
{
  // nothing to do here
}

////////////////////////////////////////////////

int UServerInQueue::addMessage(int client, const char * msg, int size, bool raw)
{
  int used = size;
  UServerInMsg * nMsg;
  int n;
  //
  if (valid)
  { // lock queue before add
    if (msg == NULL or size < 0 or size > MAX_MESSAGE_LENGTH_TO_CAM)
    {
      fprintf(stderr, "*** UServerInQueue::addMessage -- queue error (size=%d more than %d) -- truncated\n", size, MAX_MESSAGE_LENGTH_TO_CAM);
      used = MAX_MESSAGE_LENGTH_TO_CAM;
    }
    if (lock())
    { // queue state is now fixed
      // always space for next message
      n = (justIn + 1) % elements;
      nMsg = (UServerInMsg *)getMsg(n);
      if (nMsg->setMessage(client, msg, used, raw))
      { // this may give an overflow - skip oldest if so
        if (isFull())
        { //skip oldest message
          nextOut = (nextOut + 1) % elements;
          fprintf(stderr, "*** UServerInQueue::addMessage -- queue overflow - skipped oldest\n");
        }
        // move queue pointer
        justIn = n;
      }
      unlock();
    }
  }
  //
  return used;
}

////////////////////////////////////////////////

UServerInMsg * UServerInQueue::skipToNextMessage(bool checkForDublicates)
{ // check for duplicates not supported yet
  return (UServerInMsg *) UMsgQueue::skipToNextMessage(false);
}

////////////////////////////////////////////////

void UServerInQueue::print(const char * preStr)
{
  int i, j;
  UServerInMsg * m;
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
      m = (UServerInMsg *) getMsg(i);
      snprintf(s, SL, " - #%02d ", i);
      m->print(s);
    }
  }
}

//////////////////////////////////////////////////////

const char * UServerInQueue::list(const char * preStr, char * buff, const int buffCnt, int maxOldElements)
{
  UServerInMsg * qe;
  int i, ie, n, m;
  const int SL = 20;
  char s[SL];
  char * p1;
  //
  lock();
  m = mini(maxOldElements, elements - getUsedMsgCnt() - 2);
  if (m < 0)
    m = 0;
  p1 = buff;
  n = 0;
  i = nextOut - m;
  if (i < 0)
    i += elements;
  snprintf(p1, buffCnt - n, "%s has %d elements (current %d, pending %d)\n", preStr, elements, nextOut, getUsedMsgCnt());
  n += strlen(p1);
  p1 = &buff[n];
  ie = (nextOut + getUsedMsgCnt() + 1) % elements;
  while (i != ie)
  {
    qe = (UServerInMsg*) getMsg(i);
    if (qe->rxTime.valid and qe->rxTime.getDecSec() > 1e8 and (qe->size == (int)strlen(qe->message)))
    {
      snprintf(s, SL, "%4d", -m);
      qe->print(s, p1, buffCnt - n);
      n += strlen(p1);
      p1 = &buff[n];
    }
    m--;
    i = (i + 1) % elements;
  }
  unlock();
  return buff;
}
