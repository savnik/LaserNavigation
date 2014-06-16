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

#include "usmltag.h"
#include "uclienthandler.h"

/////////////////////////////////////////////////////

UClientHandler::UClientHandler()
 : UClientPortSml()
{
  // initialize
  funcCnt = 0;
  threadRunning = false;
  threadStop = true;
  serverToLocalTime = 0.0;
  serverToLocalTimeValid = false;
  serverNamespaceLevel = 0;
  // add a dummy function to handle
  // what the others do not want
  // it discards all message (including binary ones)
  baseHandler = new UClientFuncBase();
  //dummy->setConnectionPort(this);
  addFunction(baseHandler);
  setNamespace("client", "");
  namespaceUse = true;
  dataTrap = NULL;
  tryHoldConnection = false;
  tryReConnect = false;
  showWarnings = true;
  showInfo = false;
  binData = false;
  rxTimeoutMs = 40;
  strncpy(binTagName, "none", MAX_BIN_NAME_LENGTH);
}

////////////////////////////////////////////////////////////////////

void UClientHandler::setVerbose(bool value)
{
  int i;
  //
  verboseMessages = value;
  moduleLock.lock();
  for (i = 0; i < funcCnt; i++)
    func[i]->setVerbose(value);
  moduleLock.unlock();
}

/////////////////////////////////////////////////////

UClientHandler::~UClientHandler()
{
  if (threadRunning)
    stop(true);
  if (baseHandler != NULL)
    delete baseHandler;
}

/////////////////////////////////////////////////////

void UClientHandler::stop(bool andWait)
{ // stop server thread.
  threadStop = true;
  // wait for server thread to terminate
  if (andWait and threadRunning)
    pthread_join(threadHandle, NULL);
}

/////////////////////////////////////////////////////

void * clientThreadRunCall(void * obj)
{ // call the handling function in provided object
  UClientHandler * ce = (UClientHandler *)obj;
  ce->threadRunLoop();
  pthread_exit((void*)NULL);
  return NULL;
}

//////////////////////////////////////////////

bool UClientHandler::start()
{
  bool result;
  pthread_attr_t  thAttr;
  //
  result = (not threadRunning);
  if (result)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandle, &thAttr,
                  &clientThreadRunCall, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

//////////////////////////////////////////////

void UClientHandler::closeConnection()
{
  if (serverNamespaceLevel > 0)
    // is a namespace server, so send a close tag before HUP
    sendNamespaceCloseTag();
  // now close
  UClientPort::closeConnection();
}

//////////////////////////////////////////////

void UClientHandler::setNamespace(const char * toName, const char * attributeString)
{
  strncpy(namespaceName, toName, MAX_SML_NAME_LENGTH);
  strncpy(namespaceAttributes, attributeString, MAX_NORMAL_MESSAGE_LENGTH);
}

//////////////////////////////////////////////

void UClientHandler::threadRunLoop()
{
  UTime t, t1;
  USmlTag tag;
  const double BUFFER_TIMEOUT_SEC = 10.0;
  bool gotTag;
  int pingCnt = 0;
  const int timeout = 400;
  //
  threadRunning = true;
  t1.Now();
  notConnectedLoop = 0;
  while (not threadStop)
  {
    if (isConnected() and not tryReConnect)
    {
      if (binData)
      {
        char * data = getRxDataBuffer();
        int n = getMoreData(data, getRxDataBufferCnt(), rxTimeoutMs);
        if (n > 0)
        {
          data[n] = '\0';
          tag.setBinaryTag(binTagName, data, n);
        }
        gotTag = n > 0;
        // debug
        if (n < 0)
          printf("UClientHandler::threadRunLoop failed to get binary data\n");
        // debug end
      }
      else
        gotTag = getNextTag(&tag, timeout);
      if (threadStop)
        break;
      if (gotTag)
      { // update data time
        rxByteCnt += tag.getTagCnt();
        dataTime.Now();
        // use data
        gotNewData(&tag);
      }
      else if (dataCnt > 0)
      { // check for too old data in buffer
        t.Now();
        if ((t - dataTime) > BUFFER_TIMEOUT_SEC)
          // drop data in buffer
          dataCnt = 0;
      }
      else if (not binData and
               (not serverToLocalTimeValid or
               ((readCnt % 4000) == 0)))
      { // update time difference - try again
        if (false and gotFirstTag)
        {
          if (false and verboseMessages)
            printf("Sending a ping for time check\n");
          sendPing();
          pingCnt++;
        }
      }
      notConnectedLoop = 0;
    }
    else
    { // not connected or need a reconnect
      if (tryReConnect)
      { // try connect now
        tryReConnect = false;
        if (isConnected())
          closeConnection();
        else
          notConnectedLoop = 15;
      }
      if (tryHoldConnection and ((notConnectedLoop & 0x1f) == 16))
        // try a (re)connect
        tryConnect();
      //
      if (not isConnected())
      {
        notConnectedLoop++;
        Wait(0.05);
      }
    }
    // allow display and other maintenance items
    if (t1.getTimePassed() > (double(timeout) / 1000.0))
    {
      doTimeTick();
      t1.Now();
    }
  }
  threadRunning = false;
}

/////////////////////////////////////////////////////

bool UClientHandler::addFunction(UClientFuncBase * functionHandler, bool remove)
{
  bool result = true;
  int found = -1;
  int i;
  UClientFuncBase ** cf;
  const char * cfName;
  const char * fName;
  //
  cf = func;
  moduleLock.lock();
  if (functionHandler != NULL)
  {
    fName = functionHandler->commandList();
    for (i = 0; i < funcCnt; i++)
    {
      cfName = (*cf)->commandList();
      if (strcmp(fName, cfName) == 0)
      { // exist already - do not add anything
        found = i;
        break;
      }
      cf++;
    }
    if (not remove)
    { // add
      if (found >= 0)
      { // replace with new handler
        func[found] = functionHandler;
      }
      else
      {
        result = funcCnt < MAX_FUNCTION_COUNT_CLIENT;
        if (result)
        {
          func[funcCnt] = functionHandler;
          funcCnt++;
        }
        else
        {
          printf("UClientHandler::addFunction: out of module space (%d/%d)\n",
                funcCnt, MAX_FUNCTION_COUNT_CLIENT);
          result = false;
        }
      }
    }
    else if (found >= 0)
    { // remove the reference to to the deleted function
      if (found < (funcCnt - 1))
        memmove(&func[found], &func[found + 1], sizeof(void*) * (funcCnt - found - 1));
      funcCnt--;
    }
    else
    {
      printf("UClientHandler::addFunction: remove failed, as handler of "
          "'%s' do not exist\n", fName);
      result = false;
    }
  }
  else
    result = false;
  moduleLock.unlock();
  //
  return result;
}

/////////////////////////////////////////////////////

void UClientHandler::connectionChange(bool nowConnected)
{
  UClientPort::connectionChange(nowConnected);
  if (nowConnected)
  {
    if ((strcasecmp(host, "localhost") == 0) or
        (strcmp(host, "127.0.0.1") == 0))
    {
      serverToLocalTimeValid = true;
      serverToLocalTime = 0.0;
    }
    else
      serverToLocalTimeValid = false;
    // debug
    // printf("Time offset is now %s\n", bool2str(serverToLocalTimeValid));
    // debug end
    gotFirstTag = false;
  }
  else
  {
    setNamespaceLost();
    if (tryHoldConnection)
      printf("Trying to reestablish connection to %s port %d ...\n",
          host, port);
  }
}

/////////////////////////////////////////////////////

void UClientHandler::print(const char * preString)
{
  int i;
  //
  printf("%s connected (%s) to %s (%s) port %d (%d polls)\n",
         preString, bool2str(isConnected()),
         getHost(), getHostIP(), getPort(), readCnt);
  printf(" - Server to local time %f secs (valid %s)\n",
            serverToLocalTime, bool2str(serverToLocalTimeValid));
  printf("Client has %d functions:\n", funcCnt);
  moduleLock.lock();
  for (i = 0; i < funcCnt; i++)
  {
    printf(" - #%d is '%s', handling '%s'\n", i,
        func[i]->name(), func[i]->commandList());
  }
  moduleLock.unlock();
}

/////////////////////////////////////////////////////

const char * UClientHandler::snprint(const char * preString, char * buff, const int buffCnt)
{
  int i;
  char * p1;
  int n;
  //
  snprintf(buff, buffCnt, "%s Connected (%s) to %s (%s) port %d (%d polls),\n",
         preString, bool2str(isConnected()),
         getHost(), getHostIP(), getPort(), readCnt);
  n = strlen(buff);
  p1 = &buff[n];;
  snprintf(p1, buffCnt - n, "    server to local time %f secs (valid %s), data parsers:\n",
         serverToLocalTime, bool2str(serverToLocalTimeValid));
  moduleLock.lock();
  for (i = 0; i < funcCnt; i++)
  {
    n += strlen(p1);
    p1 = &buff[n];;
    snprintf(p1, buffCnt - n, " -  #%d '%s' -- '%s'\n", i,
             func[i]->commandList(), func[i]->name());
  }
  moduleLock.unlock();
  return buff;
}

/////////////////////////////////////////////////////

bool UClientHandler::inThisStringList(const char * str,
                                      const char * strList)
{
  bool result = false;
  const char * p1 = strList;
  const char * p2;
  int sl = strlen(str);
  //
  while (not result)
  {
    if ((strncasecmp(str, p1, sl) == 0) and
        (p1[sl] <= ' '))
      // string is found (and is not a substring)
      result = true;
    else
    { // try the next in list
      p2 = strchr(p1, ' ');
      if (p2 == NULL)
        // string not found
        break;
      else
      { // advance to first pointer past the space
        p1 = p2;
        p1++;
      }
    }
  }
  return result;
}

//////////////////////////////////////////////

void UClientHandler::doTimeTick()
{
  int i;
  //
  if (not threadStop)
  {
    moduleLock.lock();
    for (i = 1; i < funcCnt; i++)
      func[i]->doTimeTick();
    moduleLock.unlock();
    interfaceTick();
  }
}

//////////////////////////////////////////////

void UClientHandler::gotNewData(USmlTag * tag)
{ // new tag is available - pass to owner
  int i;
  bool foundOwner = false;
  UClientFuncBase * fb = NULL;
  UEventTrap * et = NULL;
  bool dataUsed = false;
  const char * tagName;
  const char * cmdList;
  //
  moduleLock.lock();
  if (tag->isAPhonyTag() and not binData)
  { // is no namespace, then expect a namespace value
    if (serverNamespaceLevel == 0)
      serverNamespaceLevel = 1;
    // send also name-tag to server
    sendNamespaceOpenTag();
  }
  else if (serverNamespaceLevel == 1 and tag->isAStartTag())
  { // namespace start - save namespace
    strncpy(serverNamespaceName, tag->getTagName(), MAX_SML_NAME_LENGTH);
    serverNamespaceLevel = 2;
    printf("Entered into namespace %s\n", serverNamespaceName);
    // inform client functions
    for (i = 1; i < funcCnt; i++)
    { // look for owner of message tag
      fb = func[i];
      fb->changedNamespace(tag->getTagName());
    }
  }
  else if ((serverNamespaceLevel == 2) and tag->isTagAnEnd(serverNamespaceName))
  {
    serverNamespaceLevel = 0;
    printf("Finished namespace %s\n", serverNamespaceName);
    if (isConnected())
      sendNamespaceCloseTag();
  }
  else if (tag->isTagA("ping"))
    //handle ping reply
    handlePingReply(tag);
  else
  { // find owner
    if (not tag->isAnEndTag())
    { // end tags must be a left-over,
      // so handle start and full tags only.
      // there may be a trap
      if (dataTrap != NULL)
        // trap list exist
        et = dataTrap->getEvent(tag->getTagName(), NULL);
      // find ordinary owner of data
      // or an ordinary client data client function
      tagName = tag->getTagName();
      for (i = 1; i < funcCnt; i++)
      { // look for owner of message tag
        fb = func[i];
        cmdList = fb->commandList();
        foundOwner = inThisStringList(tagName, cmdList);
        if (foundOwner)
          break;
      }
    }
    if (et != NULL)
      // a data trap gets the data
      dataUsed = et->callTrap((void *)tag);
    //
    if (tag->isTagA("alive"))
      interfaceAliveTag(tag);
    //
    if (not dataUsed)
    { // trap warnings and errors
      if (showInfo and tag->isAFullTag() and tag->getAttValue("info", NULL, 0))
        tag->print("");
      else if (showWarnings and tag->isAFullTag() and
          (tag->getAttValue("warning", NULL, 0) or
           tag->getAttValue("error", NULL, 0)))
        tag->print("");
      if (not foundOwner)
        // use dummy handler
        fb = func[0];
      // let the function do its best, and return
      // any remining data at next of length nextCnt
      // a data client gets the data
      fb->decodeLock.lock();
      fb->handleNewData(tag);
      fb->decodeLock.unlock();
    }
  }
  moduleLock.unlock();
}

/////////////////////////////////////////////////////

void UClientHandler::sendPing()
{
  const int MCL = 80;
  char cmd[MCL];
  UTime t;
  //
  t.Now();
  snprintf(cmd, MCL, "<ping tod=\"%lu.%06lu\"/>\n",
     t.getSec(), t.getMicrosec());
  sendMsg(cmd);
}

/////////////////////////////////////////////////////

void UClientHandler::sendNamespaceCloseTag()
{
  const int MRL = MAX_SML_NAME_LENGTH + 4;
  char reply[MRL];
  //
  if (namespaceUse)
  {
    snprintf(reply, MRL, "</%s>\n", namespaceName);
    sendMsg(reply);
  }
}

/////////////////////////////////////////////////////

void UClientHandler::sendNamespaceOpenTag()
{
  const int MRL = MAX_NORMAL_MESSAGE_LENGTH + MAX_SML_NAME_LENGTH + 4;
  char reply[MRL];
  //
  if (namespaceUse)
  {
    snprintf(reply, MRL, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    sendMsg(reply);
    snprintf(reply, MRL, "<%s %s>\n", namespaceName, namespaceAttributes);
    sendMsg(reply);
  }
}

/////////////////////////////////////////////////////

void UClientHandler::handlePingReply(USmlTagIn * tag)
{
  char attName[MAX_SML_NAME_LENGTH];
  const int MVL = 320; //
  char attVal[MVL];
  unsigned long sec = 0, usec;
  UTime t1, ts, t;
  double clientToServer;
  double serverToClient;
  bool gotC2S = false;
  double transmission;
  //
  t.Now();
  while (tag->getNextAttribute(attName, attVal, MVL))
  {
    if (strcasecmp(attName, "tod") == 0)
      // time att server
      sscanf(attVal, "%lu.%lu", &sec, &usec);
    else if (strcasecmp(attName, "clientToServerSec") == 0)
    {
      sscanf(attVal, "%lf", &clientToServer);
      gotC2S = true;
    }
    // ignore other attributes
  }
  if (false and verboseMessages)
    // print ping-reply
    tag->print("got");
  if ((sec > 0) and gotC2S)
  { // basis for delay and time-add calculation
    ts.setTime(sec, usec);    // local time at server
    serverToClient = t - ts;  // time offset and comm. delay
    transmission = clientToServer + serverToClient; // transmission time
    if ((absd(serverToClient) - absd(clientToServer)) < 0.002)
    { // not too bad, so use
      serverToLocalTime = serverToClient - transmission/2.0;
      serverToLocalTimeValid = true;
    }
    //
    if (false and verboseMessages)
      // print ping result
      printf("ping out=%f, home=%f, transm=%f, timeOfset=%f, valid=%s\n",
          clientToServer, serverToClient, transmission,
          serverToClient - transmission/2.0,
          bool2str(serverToLocalTimeValid));
  }
}

/////////////////////////////////////////////////////

unsigned long UClientHandler::addDataTrap(const char * key,
                                          EVENT_CALL onEvent,
                                          void * object)
{
  unsigned long result = 0;
  //
  if (dataTrap == NULL)
    dataTrap = new UEvents(MAX_DATA_TRAPS);
  if (dataTrap != NULL)
  {
    dataTrap->add(key, onEvent, object);
  }
  //
  return result;
}

////////////////////////////////////////////////////////

void UClientHandler::delDataTrap(unsigned long serial)
{
  if (dataTrap != NULL)
    dataTrap->del(serial);
}

///////////////////////////////////////////////////////

void UClientHandler::setNamespaceLost()
{
  if (serverNamespaceLevel > 0)
    printf("Left namespace %s\n", serverNamespaceName);
  serverNamespaceLevel = 0;
}

///////////////////////////////////////////////////////

bool UClientHandler::openConnection()
{
  int i = 0;
  // maintain hold connection status
  // bool retry = tryHoldConnection;
  //
  // try hold needs to be true to establish the connection
  //tryHoldConnection = true;
  //
  // connect or reconnect now
  tryReConnect = true;
  Wait(0.05);
  // if not running already, then start read thread now.
  if (not threadRunning)
    start();
  while (not isConnected())
  {
    i++;
    if (i > 60)
      // wait at maximum 3 seconds
      break;
    Wait(0.05);
  }
  // tryHoldConnection = retry;
  return isConnected();
}

///////////////////////////////////////////////////////

void UClientHandler::setHost(const char * newHost)
{
  bool recon = (strcmp(newHost, host) != 0);
  //
  UClientPort::setHost(newHost);
  if (recon)
    tryReConnect = true;
}

///////////////////////////////////////////////////////

void UClientHandler::setPort(const int newPort)
{
  bool recon = (port != newPort);
  //
  UClientPort::setPort(newPort);
  if (recon)
    tryReConnect = true;
}

///////////////////////////////////////////////////////


