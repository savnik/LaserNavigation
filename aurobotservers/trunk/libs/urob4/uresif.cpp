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
#include <math.h>

#include "uvarcalc.h"
#include "uresifbase.h"
#include "usmltag.h"

#include "uresif.h"

UResIf::UResIf(const char * aliasID)
{ // save the alias name as new ID
  strncpy(aliasResID, aliasID, MAX_ALIAS_LENGTH);
  // save the ID and version number
  setResID(getResID(), 200);
  // other local initializations
  createVarSpace(10, 0, 2, "socket connection status", false);
  createBaseVar();
  // default port
  port = 24920;
  // default host
  strncpy(host, "localhost", MAX_HOST_LENGTH);
  // try to connect
  tryHoldConnection = false;
  // allow data distribution to know the interface alias name
  ifName = aliasResID;
  makeUpdateEvent = false;
}


///////////////////////////////////////////

void UResIf::createBaseVar()
{
  //addVar("version", getResVersion() / 100.0, "d", "Resource version");
  varConnected = addVarA("connected", "0 0", "d", "(rw) [0]: Connect when '1'; [1] is connected when 1");
  varConnectionCnt = addVar("connectedCnt", 0.0, "d", "(r) number of successful connections");
  varConnectedTime = addVar("connectedTime", 0.0, "d", "(r) up-time for last connection");
  varHost = addVarA("host", "localhost", "s", "(rw) host (on next connect)");
  varPort = addVar("port", 24920, "d", "(rw) port number (on next connect)");
//  varTryHold = addVar("tryHold", 1.0, "d", "(r) Try hold the connection - set from interface command");
  varHostLoopAlive = addVar("hostLoopAlive", 0.0, "d", "(r) Seconds since last command loop on server side (from alive status)");
  varAliveReceived = addVar("aliveReceived", 0.0, "d", "(r) Time since last alive status (sec)");
  varExableXml = addVar("enableXML", 2.0, "d", "(rw) 0:off, 1:enable XML handshake, 2: and run alivetest");
  //varRunAliveTest = addVar("runAliveTest", 0.0, "d", "(r/w) Run an alive test every second");
  varHostAlive = addVar("hostAlive", 0.0, "d", "(r) is conneced server alive");
  varAliveLimit = addVar("aliveLimit", 4.0, "d", "(r/w) at no resonce if this length (seconds) the server is deemed dead");
  varRxTimeout = addVar("rxTimeout", 0.04, "d", "(r/w) timeout when waiting for more data");
  varRxBufSize = addVar("rxBufSize", double(MAX_CLIENT_RX_BUFFER), "d", "(r/w) size of receive buffer");
  varBinaryName = addVarA("binaryName", "none", "s", "(r/w) tag name for binary data (all data if binary is enabled)");
  varBinaryValid = addVar("binaryEnabled", 0.0, "d", "(r/w) is all data handled as binary (1 = true)");
  varRxBytes = addVar("rxBytes", 0.0, "d", "(r) number of received bytes from socket port");
  varRxBytesPerSec = addVar("rxByteRate", 0.0, "d", "(r) number bytes from socket port last second");
  //
  addMethod("send", "s", "send this command - do not append a new line (\\n) after string");
  addMethod("sendln", "s", "send this command - adds a new line (\\n) at end of string");
}

///////////////////////////////////////////

UResIf::~UResIf()
{
}

///////////////////////////////////////////

const char * UResIf::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
/*  snprintf(buff, buffCnt, "%s laserIf connected=%s to %s port %d\n", preString,
           bool2str(isConnected()), getHost(), getPort());*/
  return UClientHandler::snprint(preString, buff, buffCnt);
}

///////////////////////////////////////////

void UResIf::connectionChange(bool connected)
{ // do low level stuff also
  UClientHandler::connectionChange(connected);
  //
  varConnected->setValued(connected, 1, false);
//   if (not varConnected->getBool())
//     // probably reconnected
//     varConnected->setBool(connected);
  // we are connected, so continue
  // varTryHold->setValued(tryHoldConnection, 0, false);
  if (connected)
  { // wait a second with first alive test
    aliveTimeSend.now();
    // set alive time to "alive"
    aliveTime.now();
    // mark connected time
    connectedTime.now();
    varConnectedTime->setDouble(0);
    // update connection counter
    varConnectionCnt->add(1);
    // tell push system to execute on-connect commands
    makeUpdateEvent = true;
    //setUpdated("");
  }
}

///////////////////////////////////////////

bool UResIf::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  UResIfBase * rb;
  //
  if (resource->isAlsoA(UCmdExe::getResClassID()))
  { // this ressource may hold variables that can be accessed by this module
    if (remove)
      UServerPush::setCmdExe(NULL);
    else
    { // resource is also a var-pool resource, so access as so.
      UServerPush::setCmdExe((UCmdExe *) resource);
    }
    result = true;
  }
  else if (resource->isAlsoA("if"))
  { // interface data resource - add to interface
    // this means that all interfaces can handle all data types
    // slightly overkill and slightly more overhead to resolve
    // tag names, but so be it for the moment
    rb = (UResIfBase *) resource;
    // resource need to be typecase in two steps, as the subclass UClientFuncBase
    // can be resolved in two ways from UResBase.
    result = addFunction((UClientFuncBase *) rb, remove);
  }
  result |= UResVarPool::setResource(resource, remove);
  //
  return result;
}

//////////////////////////////////////////////////////

bool UResIf::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  bool isOK;
  const char * ln = "\n";
  // evaluate standard functions
  if ((strcasecmp(name, "send") == 0) and (strcasecmp(paramOrder, "s") == 0))
  {
    isOK = sendWithLock(strings[0]);
    if (value != NULL)
      *value = isOK;
  }
  else if ((strcasecmp(name, "sendln") == 0) and (strcasecmp(paramOrder, "s") == 0))
  {
    isOK = sendWithLock(strings[0]);
    if (isOK)
      isOK = sendWithLock(ln);
    if (value != NULL)
      *value = isOK;
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////

void UResIf::interfaceTick()
{
  double dt = 100.0;
  if (aliveTime.getSec() > 0)
  {
    dt = aliveTime.getTimePassed();
    varAliveReceived->setValued(dt, 0, false);
  }
  //
  if (not isConnected())
    namespaceUse = varExableXml->getInt(0) > 0;
  //    
  if (isConnected() and not varConnected->getBool(0))
  { // should close connection (and therefore do not try to reconnect)
    tryHoldConnection = false;
    closeConnection();
  }
  if (not isConnected() and varConnected->getBool(0))
  { // should open connection
    tryHoldConnection = true;
    //openConnection();
  }
  if (isConnected())
  {
    if (varExableXml->getInt(0) > 1)
    { // run alive test
      dt = aliveTimeSend.getTimePassed();
      if (dt > 0.5)
      { // every about 1 second, as interfaceTick is called every 0.4 second
        if (UClientHandler::tryLock())
        { // noone is sending, so send
          sendMsg("<alive/>\n");
          aliveTimeSend.now();
          UClientHandler::unlock();
        }
      }
      // evaluate if server is alive
      double aliveLimit = varAliveLimit->getValued(0);
      double hostLoopTime = varHostLoopAlive->getValued(0);
      double aliveReceived = varAliveReceived->getValued();
      if (dt < aliveLimit and hostLoopTime < aliveLimit and aliveReceived < aliveLimit)
        // host is alive
        varHostAlive->setBool(true);
      else
      { // else host is dead
        varHostAlive->setBool(false);
        // debug
        aliveLimit += 10.0;
        if (dt < aliveLimit and hostLoopTime < aliveLimit and aliveReceived < aliveLimit)
          // print debug message for 10 seconds
          printf("UResIf::interfaceTick: server %s on '%s:%d' is connected but seems dead for (%.1f | %.1f | %.1f) > %.1f\n",
                aliasResID, getHost(), getPort(), dt, hostLoopTime, aliveReceived, aliveLimit);
      }
    }
    varConnectedTime->setDouble(connectedTime.getTimePassed());
    if (makeUpdateEvent and connectedTime.getTimePassed() > 1.5)
    { // what a while before triggering onConnect push events
      makeUpdateEvent = false;
      setUpdated("");
    }

  }
  if (roundi(varRxTimeout->getValued()* 1000) != rxTimeoutMs)
    rxTimeoutMs = roundi(varRxTimeout->getValued()* 1000);
  if (roundi(varRxBufSize->getInt()) != getRxDataBufferCnt())
  { 
    int got = setRxDataBufferCnt(absi(varRxBufSize->getInt()));
    if (got != varRxBufSize->getInt())
    {
      printf("size limited to %d bytes\n", got);
      varRxBufSize->setInt(got);
    }
  }
  if (varBinaryValid->getBool() != binData)
    binData = varBinaryValid->getBool();
  if (strcmp(varBinaryName->getValues(), binTagName) != 0)
    strncpy(binTagName, varBinaryName->getValues(), MAX_BIN_NAME_LENGTH);
  ///
  varRxBytes->setInt(rxByteCnt);
  double sec1 = rxByteSec.getTimePassed();
  if (sec1 > 1.0)
  {
    varRxBytesPerSec->setDouble(double(rxByteCnt - rxBytseSecCnt)/sec1);
    rxBytseSecCnt = rxByteCnt;
    rxByteSec.now();
  }
}

/////////////////////////////////////////////

void UResIf::interfaceAliveTag(USmlTag * tag)
{ // like <alive last="0.008890"/>
  double at = -1.0;
  aliveTime.now();
  tag->getAttDouble("last", &at, -1);
  if (at > 0.0)
  {
    varHostLoopAlive->setValued(at, 0, false);
    aliveTime.now();
  }
}

////////////////////////////////////////////

bool UResIf::replayToTime(UTime untilTime)
{
  bool result = false;
  const int MSL = 100;
  char s[MSL];
  //
  if (replay and isConnected() and untilTime > replayTime)
  {
    snprintf(s, MSL, "<server replayTime=\"%lu.%06lu\"/>\n", untilTime.getSec(), untilTime.getMicrosec());
    result = sendWithLock(s);
    replayTime = untilTime;
  }
  return result;
}
