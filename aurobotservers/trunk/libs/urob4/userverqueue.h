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
#ifndef USERVERQUEUE_H
#define USERVERQUEUE_H

#include <ugen4/usmltagin.h>

#include "umsgqueue.h"

/**
Number os messages that the receive queue can hold */
#define MAX_SOCKET_RECEIVE_QUEUE    2000

#define MAX_MESSAGE_LENGTH_TO_CAM  1000

class UCmdExe;

/**
Class structure for one received message */
class UServerInMsg
{
public:
  /**
  Set message data, and trim whitespace if a text message
   * \param client is the index of the client posting the message.
 * \param msg is the message to be saved in the queue - should be no larger than 1000 (MAX_MESSAGE_LENGTH_TO_CAM).
   * \param size is the size of the message - a '0' will be placed here so that message will be a terminated string.
   * \param raw if false, then the message is assumed to be a xml tag - with or without breckets.
   * \returns true if space for message */
  bool setMessage(int client, const char * msg, int size, bool raw);
  /**
  Print message info on console
  after the provided prestring 'preStr'. */
  void print(const char * preStr);
  /**
   * print this queue element to string buffer
   * \param preStr ia text to print at start of message
   * \param buff is the string buffer to use
   * \param buffCnt is the max length to use of the string buffer
   * \returns a pointer to the buffer */
  const char * print(const char * preStr, char * buff, const int buffCnt);
  /**
  Set as a copy of source */
  UServerInMsg operator= (UServerInMsg source);
  /**
  Get pointer to SML-tag structure */
  inline USmlTagIn * getTag()
    { return &tag; };
  /* *
   * Get a boolean attribute value
   * \param att is the attribute to get the value from
   * \param defaultNoAttribute is the default value if attribute is not found
   * \param defaultNoValue is the default value, if attribute exist, but has an empty value.
   * \returns an integer with any of the default values, or the value of the attribute (as boolean) * /
/ *  inline int getAttBool(const char * att, const int defaultNoAttribute, const int defaultNoValue)
  {
    bool val;
    bool exist = tag.getAttBool(att, &val, defaultNoValue);
    if (not exist)
      return defaultNoAttribute;
    else
      return val;
  }*/
  

public:
  /**
  Index number for source client */
  int client;
  /**
  Reply handler */
  //UCmdExe * exe;
  /**
  Actual message lemgth */
  int size;
  /**
  Message (is zero terminated) */
  char message[MAX_MESSAGE_LENGTH_TO_CAM + 1];
  /**
  Is this a command generated as a server push command.
  This type should NOT reply to client if the request
  can not be fulfilled. (but return false to function handler. */
  bool serverPushCommand;
  /**
  Command decomposition values (assumed to be a SML tag) */
  USmlTagIn tag;
  /**
   * Queue time */
  UTime rxTime;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/**
Message queue for messages from clients */
class UServerInQueue : public UMsgQueue
{
  public:
  /**
  Constructor */
  UServerInQueue();
  /**
  Destructor */
  virtual ~UServerInQueue();
  /**
  Add message
   * \param client is index to posting client
   * \param msg is the message to pose
   * \param size is length of message - will be terminated here
   * \param raw is false, then assumed to be an XML tag (with or without brackets)
   * \returns ?? */
  int addMessage(int client, const char * msg, int size, bool raw);
  /**
  Get message and remove it from queue.
  NB! Cheeck for dublicates is not supported yet. */
  UServerInMsg * skipToNextMessage(bool checkForDublicates);
  /**
  print buffer information to console */
  void print(const char * preStr);
  /**
   * List queue elements
   * List all pending elements and a maximum number of old elements
   * \param preStr is a string to print atin the start of the buffer
   * \param buff is the buffer to use
   * \param buffCnt is the maximum length to use of the buffer
   * \param maxOldElements is the maximum number of history elements to list
   * \returns a pointer to the buffer. */
  const char * list(const char * preStr, char * buff, const int buffCnt, int maxOldElements);

};

#endif
