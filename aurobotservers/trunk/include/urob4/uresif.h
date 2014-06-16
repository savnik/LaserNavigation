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

#ifndef URESNAVIF_H
#define URESNAVIF_H

#include <cstdlib>

#include "uresbase.h"
#include "uresvarpool.h"
#include "uclienthandler.h"
#include "ucmdexe.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/

class UResIf : public UClientHandler, public UResVarPool, public UServerPush
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResIf) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResIf(const char * aliasID);
  /**
  Destructor */
  virtual ~UResIf();
  /**
  Fixed name of this resource type */
  const char * getResID()
  { return aliasResID; };
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * snprint(const char * preString, char * buff, int buffCnt)
  { return print(preString, buff, buffCnt); };
  /**
  * Called by server core when new resources are available.
  * return true is resouurce is used
  * Save a pointer to the resource as needed. */
  bool setResource(UResBase * resource, bool remove);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  /**
  The varPool has methods, and a call to one of these is needed.
  Do what is needed and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  The 'paramOrder' indicates the valid parameters d, s or c for double, string or class that is available as input values for the call.
 * If the returnStruct and returnStructCnt is not NULL, then
  a number (no more than initial value of returnStructCnt) of
  structures based on UDataBase may be returned into returnStruct array
  of pointers. The returnStructCnt should be modified to the actual
  number of used pointers (if needed). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /**
   * Set replay flag.
   * replay actions is relayed to server in other end, if flag is true.
   * \param value if true replay is enabled, and interface will respond to replay actions
   * \Returns true  */
  virtual inline bool setReplay(bool value)
  {
    replay = value;
    replayTime.clear();
    return true;
  };
  /**
  Get name associated to binary data */
  const char * binName()
  { return binTagName; };
  /**
  Set name associated to binary data */
  void setBinName(const char * handlerName)
  { varBinaryName->setValues(handlerName, 0, true); };
  /**
  Get status of data as binary */
  bool binNameValid()
  { return binData; };
  /**
  Set status of data as binary */
  void setBinNameValid(bool value)
  { varBinaryValid->setValued(value, 0, false); };
  /**
  Get timeout value for RX */
  double rxTimeout()
  { return rxTimeoutMs / 1000.0; };
  /**
   * Set hostname to connect to */
  void setHost(const char * newHost)
  {
    varHost->setValues(newHost, 0, true);
    //UClientHandler::setHost(newHost);
  };
  /**
   * Set port number to connect to */
  void setPort(const int newPort)
  {
    varPort->setInt(port);
    //UClientHandler::setPort(newPort);
  }
  /**
   * try to open connection */
  void openConnection(const char * newHost, const int newPort)
  {
    if (strlen(newHost) > 0)
      varHost->setValues(newHost, 0, true);
    if (newPort > 0)
      varPort->setInt(newPort);
    UClientHandler::setHost(varHost->getString(0));
    UClientHandler::setPort(varPort->getInt(0));
    varConnected->setInt(1, 0);
  };

  /**
   * try to open connection */
  void connectionHangUp()
  {
    varConnected->setBool(false, 0);
  };


protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();
  /**
  Connection status is changed */
  virtual void connectionChange(bool connected);
  /**
   * Time tick from receive data loop */
  virtual void interfaceTick();
  /**
   * Alive tick from receives data loop */
  virtual void interfaceAliveTag(USmlTag * tag);
  /**
  Advance replay for this resources - if in replay mode - until just before this time
   * \param untilTime advance with steps in logfile until this time is reached
   * \returns true if advanced */
  virtual bool replayToTime(UTime untilTime);

protected:
  static const int MAX_ALIAS_LENGTH = 30;
  /**
  The alias name for this interface. */
  char aliasResID[MAX_ALIAS_LENGTH];
  /**
   * Time last alive statement were received */
  UTime aliveTime;
  /**
   * Time last alive statement were received */
  UTime aliveTimeSend;
  /**
   * Replay time used */
  UTime replayTime;
  /// time of last connect
  UTime connectedTime;
  /**
   * time since last server loop */
  UVariable * varHostLoopAlive;
  /**
   * time since last alive message was received */
  UVariable * varAliveReceived;
  /**
   * sent an alive test every second - if true */
  UVariable * varExableXml;
  /**
   * sent an alive test every second - if true */
  UVariable * varHostAlive;
  /**
   * Should interface try hold connection */
  //UVariable * varTryHold;
  /**
   * try hold the connection */
  UVariable * varConnected;
  /// length of last connection
  UVariable * varConnectedTime;
  /// number of successful connections
  UVariable * varConnectionCnt;
  /// host name used when next connected
  UVariable * varHost;
  /// portnumber when nex connected
  UVariable * varPort;
  /**
   * how long is the longest allowed pause before deemed dead */
  UVariable * varAliveLimit;
  /// time to maximum wait for more data on receive - befor give-up or return.
  UVariable * varRxTimeout;
  /// size of receive buffer
  UVariable * varRxBufSize;
  /// binary alias name
  UVariable * varBinaryName;
  /// data handled as binary
  UVariable * varBinaryValid;
  /// number of bytes received
  UVariable * varRxBytes;
  /// number of bytes received each second
  UVariable * varRxBytesPerSec;
  /// UTime last second
  UTime rxByteSec;
  /// number of bytes at last sec time
  int rxBytseSecCnt;
private:
  /// flag to make an event a little while after connection is established 
  bool makeUpdateEvent;

};

#endif

