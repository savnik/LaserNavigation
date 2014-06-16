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
#ifndef UCLIENTFUNCBASE_H
#define UCLIENTFUNCBASE_H

#include <ugen4/usmltagin.h>

#include "uclientport.h"

class UOnEvent;
class USmlTag;

/**
* Base class that encapsulates the ability to make a call-back, when a
  data source based on UOnEvent has new data.
* The function 'onEvent' should be overwritten and will be called when the data
  source is updated.
* To start the call back function the 'addOnEvent('dataSourceObject')' should be called */
class UCallBack
{
public:
  /**
  Constructor */
  UCallBack();
  /**
  Destructor */
  virtual ~UCallBack();
  /**
  Call back function */
  virtual bool onEvent(const char * interface, const char * dataType, void * data);
  /**
  Add this callback */
  bool addOnEvent(UOnEvent * dataObject);
};

////////////////////////////////////////////////////////////////////////


class UOnEvent
{
  public:
  /**
  Constructor */
  UOnEvent();
  /**
  Destructor */
  ~UOnEvent();
  /**
  Type method to call when event happends */
  typedef bool (UCallBack::*Method)(const char * interface, const char * dataType, void * data);
  /**
  Set callbach instance */
  bool addEventHandler(UCallBack * object, Method methodToCall);
  /**
  When an event happends do the call */
  bool event(const char * interface, const char * dataType, void * dataPtr);

protected:
  static const int MAX_CALL_BACKS = 20;
  /**
  objects to calle */
  UCallBack * obj[MAX_CALL_BACKS];
  /**
  Method pointer in object */
  Method method[MAX_CALL_BACKS];
  /**
  Number of objects used */
  int objCnt;
};



/**
The client data structure for a given server function is a decendent of this class.
The class holds the basic functionality and some basic variables.

@author Christian Andersen
*/
class UClientFuncBase
{
public:
  /**
  Constructor */
  UClientFuncBase();
  /**
  Destructor */
  virtual ~UClientFuncBase();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name();
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList();
  /**
  Set connection pointer for server communication */
/*  inline void setConnectionPort(UClientPortSml * connection)
    { cnn = connection; };*/
  /**
  Set verbose messages - mostly for debug purpose */
  inline void setVerbose(bool value)
    { verboseMessages = value;};
  /**
  Set verbose messages - mostly for debug purpose */
  inline bool getVerbose()
    { return verboseMessages; };
  /**
  Got fresh data destined to this function. */
  virtual void handleNewData(USmlTag * tag);
  /**
  The server has set (or changed) the namespace */
  virtual void changedNamespace(const char * newNamespace);
  /**
  Send data to the server (thread-safe) */
  inline bool sendMsg(UClientPortSml * cnn, const char * buff, int buffCnt)
  {  if (cnn != NULL)
        return cnn->blockSend(buff, buffCnt);
      else
        return false;
  };
  /**
  Send data to the server (thread-safe) */
  inline bool sendMsg(UClientPortSml * cnn, const char * buff)
  {     return sendMsg(cnn, buff, strlen(buff));  };
  /**
  A time tick function that is called
  evert 0.1 second of the line is idle */
  virtual void doTimeTick();
  /**
  Get connection pointer for connection functions */
/*  inline UClientPortSml * getCnn()
    { return cnn; };*/
  /**
  Get number of handled messages by this data handler resource */
  int getMsgCnt()
  { return msgHandled; };

protected:
  /**
  Just print reply on console with the prestring */
  void  printReply(USmlTagIn * tag, const char * preString);
  /**
  Print help messages */
  void handleHelp(USmlTag * tag);


protected:
  /**
  Pointer to the communication port, where
  transmissions and data are exchanged with server */
//  UClientPortSml * cnn;
  /**
  Print pore information when relevant */
  bool verboseMessages;
    /**
  Server namespace tag */
  char serverNamespace[MAX_SML_NAME_LENGTH];
  /**
  Server namespace value
  legal values are determined by client,
  E.g. laser scanner server ulmsserver
  may use value 2, but 2 can be used by e.g. ucamserver too.
  and is a translation from serverNamespace
  0 = not supported */
  int serverNamespaceValue;
  /**
  Number of messages handled */
  int msgHandled;
  
public:
  /**
  Data decode lock, to ensure that a data handler used by more than one
  interface is not decoding two messages at the same time. This
  could be the case for obstacles from laser scanner and vision interfaces.
  Is locked by the client handler befor the call to 'handleNewData(...)'  */
  ULock decodeLock;
};

#endif
