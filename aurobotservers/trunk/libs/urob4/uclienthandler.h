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
#ifndef UCLIENTHANDLER_H
#define UCLIENTHANDLER_H

#include <pthread.h>

#include <ugen4/utime.h>

#include "uclientport.h"
#include "uclientfuncbase.h"
#include "uevents.h"
#include "uresbase.h"

/**
Number of functions that cal be loaded for a client, i.e.
decendents of UClientFuncBase. */
#define MAX_FUNCTION_COUNT_CLIENT 20

/**
max length of a tag attribute list.
Here used for namespace attribute list. */
#define MAX_NORMAL_MESSAGE_LENGTH 200
/**
Services a client port and transfers data to the appropriate data functions

@author Christian Andersen
*/
class UClientHandler : public UClientPortSml
{
public:
  /**
  Constructor */
  UClientHandler();
  /**
  Destructor */
  virtual ~UClientHandler();
  /**
  Set verbose flag for handler and functions */
  void setVerbose(bool value);
  /**
  Stop receive thread */
  void stop(bool andWait);
  /**
  Send a close tag and close connection */
  void closeConnection();
  /**
  Add function to handle some of the incomming data.
  If a function handles exactly the same tags, then it will be replaced with this
  new handler.
  If 'remove' is true, then handler will be removed from the list.
  Returns true if action is performed. Prints on console if not. */
  bool addFunction(UClientFuncBase * functionHandler, bool remove = false);
  /**
  Print status for client */
  void print(const char * preString);
  /**
  Print current status to buffer string with this initial prestring. */
  const char * snprint(const char * preString, char * buff, const int buffCnt);
  /**
  Start handling incomming messages in
  seperate thread.
  Returns true if thread started. */
  bool start();
  /**
  The loop called by the read thread.
  It will return when the threadStop is set true. */
  void threadRunLoop();
  /**
  Is the read thread running?.
  Returns true if running */
  inline bool isThreadRunning()
    { return threadRunning; };
  /**
  Get the time offset that needs to be added to servertime
  to get local time. */
  inline double getServerToLocalTime()
    { return serverToLocalTime; };
  /**
  Is time difference between server and client valid? */
  inline bool isServerToLocalTimeValid()
    { return serverToLocalTimeValid; };
  /**
  Send a ping command to server to get
  time difference between servers. */
  void sendPing();
  /**
  Is called when connection status is changed.
  NB! if overwritten, then remember to call
  same function from ancestor class. */
  virtual void connectionChange(bool nowConnected);
  /**
  Set namespace name, and a string with XML coded attributes.
  Name may be line "clientTalk".
  Name may not be empty (default is "client").
  Attributes may be e.g. "name=\"myclient\" version=\"1.0\" ",
  remember the quotes for the value part of attribute names.
  Attribute string may be an empty string. */
  void setNamespace(const char * toName, const char * attributeString);
  /**
  Set server namespace lost - i.e. waiting for new namespace from server */
  void setNamespaceLost();
  /**
  Should namespace opening and closing tags be used?*/
  inline void setNamespaceUse(bool value)
  { namespaceUse = value; };
  /**
  Get setting of namespaceUse flag */
  inline bool getNamespaceUse()
  { return namespaceUse; };
  /**
  Get the last communicated server namespace name.
  Also available after the connection is closed. */
  inline const char * getServerNamespaceName()
  { return serverNamespaceName; };
  /**
  Add a data trap - to get data from the stream for other purposes.
  The trap will be effective until deleted. */
  unsigned long addDataTrap(const char * key,
                            EVENT_CALL onEvent,
                            void * object);
  /**
  Delete a data trap */
  void delDataTrap(unsigned long serial);
  /**
  Get number of loaded functions */
/*  inline int getFuncCnt()
  { return funcCnt;};*/
  /**
  Get pointer to function with this index */
//   inline UClientFuncBase * getFunc(int index)
//   { return func[index]; };
  /**
  Set hold connection - this should result in a connection attempt within 0.05 secs.
  and an attempt to maintain connection with an attempt every abot 1-1.5 second. */
  bool openConnection();
  /**
  Set host, and if the host is new, then reopen to the new host */
  void setHost(const char * host);
  /**
  Set port, and if the port is new, then reopen to the new host */
  void setPort(const int port);

public:
  /**
  Number of data trapc created - when used.
  Created by first 'addDataTrap' */
  static const int MAX_DATA_TRAPS = 100;
  /**
  Try reconnect if connection is lost */
  bool tryHoldConnection;
  /**
  Show warnings (and errors) on console */
  bool showWarnings;
  /**
  Show info messages on console */
  bool showInfo;

protected:
  /**
  Sent the namespace close tag */
  void sendNamespaceCloseTag();
  /**
  Send namespace open tag with loaded attribute string */
  virtual void sendNamespaceOpenTag();
  /**
   * Time tick from receive data loop */
  virtual void interfaceTick() {};
  /**
   * Alive tick from receives data loop */
  virtual void interfaceAliveTag(USmlTag * tag) {};

private:
  /**
  Find if this string is one of the strings in the strlist */
  bool inThisStringList(const char * str,
                        const char * strList);
  /**
  New data is received - use the data (if possible)
  Data is in data and dataCnt is count of valid data.
  on return the used parts of the data is removed. */
  void gotNewData(USmlTag * tag);
  /**
  Tell each client function that time has passed
  and connection is idle. Can be used
  to update the GUI */
  void doTimeTick();
  /**
  Calculate transmission delay and time offset from server */
  void handlePingReply(USmlTagIn * tag);


protected:
  /**
  Try reconnect, that is disconnect and reconnect - may be change in port or host */
  bool tryReConnect;
  /**
  Tag level 0 = no namespace, 1 = root level, 2 = normal instde namespace */
  int serverNamespaceLevel;
  /**
  Namespace name used by the connected server */
  char serverNamespaceName[MAX_SML_NAME_LENGTH];
  /**
  Namespace to be used by this client */
  char namespaceName[MAX_SML_NAME_LENGTH];
  /**
  The attribute string send after the namespace name */
  char namespaceAttributes[MAX_NORMAL_MESSAGE_LENGTH];
  /**
  Some connextions do not like namespace (like MRC), this
  flag determines if it should be used */
  bool namespaceUse;

protected:
  /** Number of client read thread loops that the client been not-connected */
  int notConnectedLoop;
  /// binary tag name
  static const int MAX_BIN_NAME_LENGTH = 30;
  char binTagName[MAX_BIN_NAME_LENGTH + 1];
  /// is all data binary
  bool binData;
  /// rx timeout value in ms
  int rxTimeoutMs;

private:
  /**
  Functions to handle the commands */
  UClientFuncBase * func[MAX_FUNCTION_COUNT_CLIENT];
  /**
  Number of functions loaded */
  int funcCnt;
  /**
  Thread running flag */
  bool threadRunning;
  /**
  Flag to stop thread - terminates the thread */
  bool threadStop;
  /**
  Thread handle for command read thrad. */
  pthread_t threadHandle;
  /**
  Time of last received data (by client handler) */
  UTime dataTime;
  /**
  Time to add to get from server time to local time.
  This time is set by a ping command. */
  double serverToLocalTime;
  /**
  Is server time offset valid.
  Is set to valid after a ping with an out-home
  time difference of less than 0.2 ms */
  bool serverToLocalTimeValid;
  /**
  OnEvent stack */
  UEvents * dataTrap;
  /**
  Lock for receiving data and changing data handling modyules */
  ULock moduleLock;
  /**
   * pointer to the base data handler, so that it can be released properly */
  UClientFuncBase * baseHandler;
};

#endif
