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
#include <ctype.h>

#include <ugen4/usmltagin.h>
// for dynamic load functions
#include <dlfcn.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "ufunctionbase.h"
#include "ucmdexe.h"
#include "ufunctionposehist.h"
#include "uvarcalc.h"
#include "userverport.h"
#include "ulogfile.h"

// the types of the class factories - for dynamic load of functions
typedef UFunctionBase* create_func(void * p);
typedef void delete_func(UFunctionBase*);

//////////////////////////////////////////////////////

void UCmdExe::UCmdExeInit()
{
  // this is a resource too, so it needs a name and a version
/*  setResID(getResID());
  resVersion = getResVersion();*/
  //
  funcCnt = 0;
  server = NULL;
  verboseMessages = false;
  //logServer = NULL;
  // thread variables
  threadRunning = false;
  threadStop = true;
  servName = "No Name";
  pushWatchCnt = 0;
  idleLoopCnt = 0;
  resPool = new UResPool();
  // set core resource - to var-pool
  UResVarPool::createVarSpace( 10, 0, 5, "Variables related to the server core.", false);
  UResVarPool::setResource( this, false);
  createBaseVar();
  //
  resFuncIdx = MAX_FUNCTION_COUNT;
  setCorePointer(this);
  resPool->addResource(this);
  stopping = false;
  callNotice = false;
}

//////////////////////////////////////////////

UCmdExe::~UCmdExe()
{
  stop(true);
  logServer.closeLog();
  delete resPool;
  if (server != NULL)
    delete server;
  server = NULL;
}

//////////////////////////////////////////////

const char * UCmdExe::name()
{
  return servName;
}

//////////////////////////////////////////////

const char * UCmdExe::commandList()
{ // command tags handled by the server core
  return "push q server help shelp module do BASH alive quit exit";
}


///////////////////////////////////////////

void UCmdExe::createBaseVar()
{
    addVar("version", getResVersion() / 100.0, "d", "Resource version");
    varPort = addVar("port", 0.0, "d", "Interface port to this server");
    varAllowConnection = addVar("allowConnections", 1.0, "d", "(r/w) Is new connections allowed (1=true)");
    varOpen4connections = addVar("open4connections", 0.0, "d", "(r) Is server open for new connections (1=true)");
    varClients = addVar("clients", 0.0, "d", "Number of clients connect on TCP/IP port");
    varLastClient = addVar("newestClient", 0.0, "d", "client number of newest client");
    varLastClientSerial = addVar("clientsEver", 0.0, "d", "number of clients ever served (in this session)");
    varTime = addVar("time", 0.0, "t", "Last time the core loop passed");
    varIdle = addVar("idle", 0.0, "d", "idle time [0..1] for main loop the last 5 seconds");
    varAlivePunkTime = addVar("clientPunkTime", 0.0, "d", "(r/w) if client is silent for this time [sec], then punk with an alive message");
    addMethod("send", "s", "Send a command 's' to the command queue of this server (client -2)");
    addMethod("send", "sd", "Send a command 's' to the command queue of this server, specifying client number 'd': -2 is null output, -1 is server console, 0..N is socket client");
    addMethod("bash", "s", "Execute this bash command, and return the value returned by the called function");
}

//////////////////////////////////////////////////

bool UCmdExe::methodCall(const char * name, const char * paramOrder,
                           char ** strings, const double * pars,
                           double * value,
                           UDataBase ** returnStruct,
                           int * returnStructCnt)
{
  bool result = true;
  int clnt;
//  UDataString * sc = NULL;
  // evaluate function(s)
  if ((strcasecmp(name, "send") == 0) and (strcmp(paramOrder, "s") == 0))
  { // pose the command as for client -2
    *value = postCommand(-2, strings[0]);
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "send") == 0) and (strcmp(paramOrder, "sd") == 0))
  { // pose the command as for specified client
    clnt = roundi(pars[0]);
    *value = postCommand(clnt, strings[0]);
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "bash") == 0) and (strcmp(paramOrder, "s") == 0))
  { // execute a bash command, and return the console output
/*    if (returnStruct != NULL)
    {
      if (returnStruct[0]->isA("string"))
        sc = (UDataString *) returnStruct[0];
    }*/
    *value = system(strings[0]);
    //*value = doBashCmd(strings[0], sc);
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else
    result = false;
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::addFunction(UFunctionBase * function)
{
  bool result;
  int i;
  const int MFLL = 500;
  char fl[MFLL];
  char * p1, * p2 = fl;
  const char * fn;
  //
  //lock();
  strncpy(fl, function->commandList(), MFLL);
  // test if function has an illegal alias name
  result = (strcasecmp(function->getAliasName(), "if") != 0);
  if (not result)
  {
    printf("A module may not be called the reserved name 'if'\n");
  }
  if (result)
  { // test to see if names are known already
    while (p2 != NULL)
    {
      p1 = strsep(&p2, " ");
      if (*p1 <= ' ')
        break;
      i = findFunctionOwner(p1);
      if (i != -1)
      { // known already
        if (i == MAX_FUNCTION_COUNT)
          fn = "System function";
        else
          fn = func[i]->name();
        printf("Warning! Command '%s' is already used in function '%s'!\n"
              "  -- i.e. one of the functions will get the '%s' command only!)\n",
              p1, fn, p1);
        // do not allow name clash
        // reult = false
      }
    }
  }
  if (result)
  { // find first available function slot for the new module
    for (i = 0; i < funcCnt; i++)
    {
      if (func[i] == NULL)
        break;
    }
    result = i < MAX_FUNCTION_COUNT;
  }
  if (result)
  {
    func[i] = function;
// func[i]->setCmdHandler(this);
    funcCnt = maxi(i + 1, funcCnt);
  }
  //
  //
  if (result)
  { // tell new module about existing resources
    resourcesUpdate(i);
    // allow module to create its own resources
    addNewRessources(function);
  }
  //unlock();
  //
  return result;
}


/////////////////////////////////////////////

bool UCmdExe::addNewRessources(UFunctionBase * function)
{
  const int MFLL = 500;
  char fl[MFLL];
  char * p1; //, * p2 = fl;
  const char *rl;
//  UResBase * ress;
  bool newResources = false;
  int idx; //, n;
  bool result;
  //
  idx = findFunctionIndex(function);
  result = (idx >= 0);
  if (result)
  { // let the new function create new resources - if needed
    // ensure the plug-in resource has an ID
    if (function->isA("plugin"))
    { // this name is not the final plugin name, so use the first part of name
      // with a 'f' as preposition
      rl = function->getAliasName();
      if (strlen(rl) == 0)
        rl = function->name();
      p1 = fl;
      *p1++ = 'f';
      for (int n = 0; n < 10; n++)
      { // up to 10 characters
        if (isAlphaNum(*rl))
          *p1++ = *rl;
        if (*rl <= ' ' and p1 - fl > 3)
          break;
        else if (*rl == '\0')
          break;
        rl++;
      }
      *p1 = '\0';
      if (resPool->getResource(fl) != NULL)
        // extend name with resource number - last resort
        snprintf(p1, MFLL - strlen(fl), "%d", resPool->getResCnt());
      // debug
      p1 = fl;
      // debug end
      function->setResID(fl, 915);
    }
    // the new function is also a resource, so add to the resource list
    // as a self owner
    addResource(function, function);
    //
    // tell all modules and resources about the new
    // resource situation
    if (newResources)
      // all are updated with resource list
      resourcesUpdated();
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::addResource(UResBase * res, UFunctionBase * owner)
{
  int idx;
  bool result;
  //
  idx = findFunctionIndex(owner);
  result = (idx >= 0);
  if (result)
  { //
    res->setResFuncIdx(idx);
            // set pointer to this core resource
            // to allow resources to access static resources
    res->setCorePointer(this);
    result = resPool->addResource(res);
    // let the new resource start or create anything that needs
    // the core to be available
    resourcesUpdated();
    res->createResources();
  }
  else
    printf("UCmdExe::addResource failed to add %s resource from module %s\n"
          "   module not found in module list!\n", res->getResID(), owner->name());
  //
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::deleteFunction(UFunctionBase * function)
{
  bool result = false;
  int idx, f;
  UResBase * res, * resF;
  UFunctionBase * fu;
  bool didUse;
  //
  //lock();
  result = function != NULL;
  if (result)
  {
    result = false;
    for (idx = 0; idx < funcCnt; idx++)
    {
      if (func[idx] == function)
      { // this is the function to remove
        func[idx] = NULL;
        result = true;
        break;
      }
    }
    while ((func[funcCnt - 1] == NULL) and (funcCnt > 0))
      funcCnt--;
  }
  // the function may have had resources
  if (result)
  {
    while (true)
    {
      res = resPool->removeResourceFunc(idx);
      if (res != NULL)
      { // tell the other modules that the
        // resource is no longer valid
        for (f = 0; f < funcCnt; f++)
        { // tell all functions
          fu = func[f];
          if (fu != NULL)
          {
            didUse = fu->setResource(res, true);
            if (didUse)
              // function used the ressource
              fu->resourceUpdated();
          }
        }
        // tell also fast responce server
        server->setResource(res, true);
        // tell also other ressources
        for (f = 0; f < resPool->getResCnt(); f++)
        {
          resF = resPool->getResource(f);
          if ((resF != res) and (resF != NULL))
            resF->setResource(res, true);
        }
        //
        printf("Ressource %s is removed\n", res->getResID());
        // do not remove from memory - this must be done by the function
        // delete res;
      }
      else
        // no resources provided by this function
        break;
    }
  }
  // release core for further processing
  //unlock();
  //
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::handleOneMessageFromQueue()
{
  bool result = false;
  UServerInMsg * msg;
  int i, n;
  UTime t;
  //
  if (server == NULL)
  {
    fprintf(stderr, "No Server to receive commands from!\n"
                "    - terminating (press quit and try again\n");
    threadStop = true;
  }
  else
  {
    msg = server->getRxQueue()->skipToNextMessage(false);
    if (msg != NULL)
    { // ensure prompt is not printed to server console
      //consoleLock.tryLock();
      // lock function list
      lock();
      strncpy(currentMessage, msg->message, MMS);
      t.now();
      i = getFunctionOwner(msg);
      result = (i >= 0);
      if (result)
      { // a function is found -> execute with this message
        // debug(
        strcat(currentMessage, " <");
        // debug end
        executeFunction(i, msg, NULL, false);
        // debug(
        strcat(currentMessage, "OK>");
        // debug end
      }
      if (not result)
      { // send error reply to client
        sendWarning(msg, "Unsupported function keyword");
        // print on console also
        if (verboseMessages)
          msg->print("Not used");
        if (logServer.isOpen())
        {
          n = server->getRxQueue()->getNextOut();
          fprintf(logServer.getF(), "%lu.%06lu (%.5f( %2d %2d %2d (no owner) %s\n",
                  t.getSec(), t.getMicrosec(), t.getTimePassed(),
                  n, server->getRxQueue()->getUsedMsgCnt(),
                  msg->client, msg->message);
        }
      }
      strncpy(currentMessage, "(idle)", MMS);
      unlock();
    }
    else
    { // allow console to print a prompt
      //consoleLock.post();
      /* if (verboseMessages)
        printf("UCmdExe::handleOneMessage: Queue is empty\n");*/
    }
  }
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::handleOneServerPushMessage()
{
  bool result = false;
  UServerPushElement * qe;
  bool success = false;
  UTime t;
  int n;
  const double holdOffTime = 3.0; // wait this many seconds before service push commands for a client
  //
  qe = push.getNextTimedPushElement();
  if (qe != NULL)
  {
    if (isClientAlive(qe->toDo.client, holdOffTime))
    { // alive, so  coommand
      t.now();
      if (qe->activeCmd)
      {
        if (qe->functionIndex == MAX_FUNCTION_COUNT)
          success = systemFunction(&qe->toDo);
        else
        {
          lock();
          if (func[qe->functionIndex] != NULL)
            success = func[qe->functionIndex]->newCommand(&qe->toDo, NULL);
          unlock();
        }
        // test for last function call
        qe->update(success);
      }
      else if (qe->activeCall)
      {
        success = push.doPushCall(getVarPool(), qe, "");
      }
      if (logServer.isOpen())
      {
        n = server->getRxQueue()->getNextOut();
        fprintf(logServer.getF(), "%lu.%06lu (%.5f) %2d %2d %d (%s) %s\n",
                t.getSec(), t.getMicrosec(), t.getTimePassed(),
                n, server->getRxQueue()->getUsedMsgCnt(),
                qe->toDo.client, bool2str(success), qe->toDo.message);
      }
    }
    else
    { // client is dead - flush the pending commands
      qe->activeCmd = false;
      qe->activeCall = false;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

int UCmdExe::findFunctionOwner(const char * tagName)
{
  int result = -1;
  int i;
  // find owner
  for (i = funcCnt - 1; i >= 0 ; i--)
  { // compare with handled tag names
    if (func[i] != NULL)
    {
      if (func[i]->isMine(tagName))
      {
        result = i;
        break;
      }
/*      IF (INTHISSTRINGLIST(TAGNAME, FUNC[I]->COMMANDLIST()))
      {  // MESSAGE IS HANDLED
        RESULT = I;
        BREAK;
      }*/
    }
  }
  if (result == -1)
  { // not a loaded function - may be a system
    if (inThisStringList(tagName, commandList()))
      result = MAX_FUNCTION_COUNT;
  }
  return result;
}

//////////////////////////////////////////////

int UCmdExe::findFunctionIndex(UFunctionBase * function)
{
  int result = -1;
  int i;
  // find owner
  for (i = funcCnt - 1; i >= 0 ; i--)
  { // compare with handled tag names
    if (func[i] == function)
    {  // function is found
      result = i;
      break;
    }
  }
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::sendMsg(int clientIdx, const char * message)
{
  return sendMsg(clientIdx, message, strlen(message));
}

//////////////////////////////////////////////

bool UCmdExe::sendMsg(UServerInMsg * msg, const char * message)
{
  return sendMsg(msg, message, strlen(message));
}

//////////////////////////////////////////////

bool UCmdExe::sendMsg(int clientIdx, const char * message, int size)
{
  bool result = (message != NULL);
  UServerClient * cnn;
  const int MSL = 100;
  int n, m;
  char s[MSL + 1];
  const char * p1;
  const char * p2;
  const int max_xml_escape_seq_length = 6;
  //
  if (result)
  {
    cnn = server->getClient(clientIdx);
    if (cnn == NULL and (clientIdx < -2))
    { // null connection - dump reply
      result = true;
    }
    else if (cnn == NULL)
    { // from server console - print result back to consiole
      p1 = message;
      n = 0;
      while (n < size)
      {
        m = mini(size - n, MSL);
        strncpy(s, p1, m);
        s[m] = '\0';
        // look for escape sequence start (from end of string)
        p2 = strrchr(s,'&');
        if ((p2 != NULL) and ((p2 - s) > (m - max_xml_escape_seq_length)) and (m == MSL))
        { // do not split a string inside an escape sequence
          m = p2 - s;
          s[m] = '\0';
        }
        // remove special XML coding of '&><"
        // NB! removal may fail if XML coded character is
        // across the string length border (assumed OK)
        xml2str(s, MSL, s, m);
        printf("%s", s);
        n += m;
        p1 = &message[n];
      }
      result = true;
    }
    else
    {
      if (cnn->isActive())
        result = cnn->blockSend(message, size, 12000);
    }
    if (not result)
    { // no client - send to console
      //printf("UCmdExe::sendMsg: Message not send (%d bytes)\n", size);
    }
  }
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::sendMsg(UServerInMsg * msg, const char * message, int size)
{
  return sendMsg(msg->client, message, size);
}

//////////////////////////////////////////////

bool UCmdExe::sendMsgAll(const char * message, bool lockedUser)
{
  bool result = (message != NULL);
  UServerClient * cnn;
  int clnt;
  int size;
  //
  if (result)
  {
    size = strlen(message);
    result = false;
    for (clnt = 0; clnt < server->getClientCnt(); clnt++)
    {
      cnn = server->getClient(clnt);
      if (cnn != NULL)
        if (cnn->isActive())
        { // do not lock marked user
          if (clnt != lockedUser)
            cnn->lock();
          // send message (assumed in XML format already)
          if (cnn->blockSend(message, size, 500))
            result = true;
          // unlock connection
          if (clnt != lockedUser)
            cnn->unlock();
        }
    }
  }
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::systemFunction(UServerInMsg * msg)
{
  bool result;
  //const int MTL = 30;
  //char tag[MTL];
  const int MSL = 200;
  char s[MSL];
  const char reply[] = "<warning msg=\"System functions not supported yet\" tag=\"%s\"/>\n";
  //
  result = msg->tag.isValid();
  //getTagType(msg->message, tag, MTL);
  if (result)
  { // find out what to do
    if (msg->tag.isTagA("q"))
      // close connection to this client
      closeClient(msg->client);
    else if (msg->tag.isTagA("push") or
             msg->tag.isTagA("timePush"))
      result = handleServerPushCommand(msg);
    else if (msg->tag.isTagA("sysGet") or
             msg->tag.isTagA("server"))
      // system wide commands
      result = handleServerCommand(msg);
    else if (msg->tag.isTagA("module"))
      result = sysModuleCmd(msg);
    else if (msg->tag.isTagA("do") or msg->tag.isTagA("bash"))
      result = handleShellCmd(msg);
    else if ((msg->tag.isTagA("help")) or
              (msg->tag.isTagA("shelp")))
      result = sysServerHelp(msg);
    else if (msg->tag.isTagA("alive"))
      result = handleAlive(msg);
    else if (msg->tag.isTagA("quit") or msg->tag.isTagA("exit"))
      result = handleQuit(msg);
    else
    {
      snprintf(s, MSL, reply, msg->tag.getTagName());
      if (verboseMessages)
        printf("%s", s);
      result = false;
    }
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::sysModuleCmd(UServerInMsg * msg)
{
  bool result;
  char att[MAX_SML_NAME_LENGTH];
  const int MSL = 200;
  char val[MSL];
  char s[MSL];
//  char rl[MSL];
  const int MRL = 5000;
  char reply[MRL];
  char unknown[MRL];
  char mName[MRL];
  bool doLoad = false;
  bool doUnload = false;
  bool doList = false;
  bool doResList = false;
  bool ask4help = false;
  bool doUnknown = false;
  int i = 0, n, j;
  UFunctionBase * fb;
//  char * p1, *p2;
//  bool isMine;
  const int MAL = UFunctionBase::MAX_ID_LENGTH;
  char aliasName[MAL] = "";
  //
  result = msg->tag.isValid();
  //getTagType(msg->message, tag, MTL);
  while( msg->tag.getNextAttribute(att, val, MSL))
  { // look for a time-of-day tag
    if (strcasecmp(att, "load") == 0)
    {
      doLoad = true;
      strncpy(mName, val, MRL);
    }
    else if (strcasecmp(att, "alias") == 0)
      strncpy(aliasName, val, MAL);
    else if (strcasecmp(att, "unload") == 0)
    {
      doUnload = true;
      strncpy(mName, val, MRL);
    }
    else if (strcasecmp(att, "list") == 0)
      doList = true;
    else if (strcasecmp(att, "resList") == 0)
      doResList = true;
    else if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else
    { // unknown option
      doUnknown = true;
      n = strlen(unknown);
      snprintf(&unknown[n], MRL-n, " %s='%s'", att, val);
      i--;
    }
    i++;
  }
  if (strcasecmp(mName, "poseHist") == 0)
  { // old name of odoPose name
    printf("Deprecated use of 'poseHist', changed to the new name 'odoPose'\n");
    strncpy(mName, "odoPose", MRL);
  }
  if (i == 0)
    // no options -- show help
    ask4help = true;
  if (doUnknown)
  {
    snprintf(reply, MRL, "<%s warning=\"unused options %s\"/>\n",
             msg->tag.getTagName(), unknown);
    sendMsg( msg, reply);
  }
  else if (ask4help)
  {
    sendMsg( msg, "<help subject=\"module help\">\n");
    sendText( msg, "----- Available MODULE options:\n");
    sendText( msg, "list               List current modules\n");
    sendText( msg, "reslist            List current resources\n");
    sendText( msg, "load='filename'    Load plugin lodule with this filename\n");
    if (getStaticHelpList(reply, MRL))
      sendText(msg, reply);
    sendText( msg, "unload='module'    Unload this module using index, filename or first part of name\n");
    sendText( msg, "help               Show this list\n");
    sendText( msg, "---\n");
    sendMsg( msg->client, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (doList)
  {
    snprintf(reply, MRL, "<help subject=\"%s list\">\n", msg->tag.getTagName());
    sendMsg( msg->client, reply);
    snprintf(s, MSL, "Module core '%s'\n",
           name());
    sendText( msg, s);
    snprintf(s, MSL, "      Handles: %s\n",
           commandList());
    sendText( msg, s);
    snprintf(s, MSL, "      Resources: %s (got all)\n",  resID);
    sendText( msg, s);
    for (j = 0; j < funcCnt; j++)
    {
      fb = func[j];
      if (fb != NULL)
      { // send name and key-names
        if (strlen(fb->getAliasName()) > 0)
          snprintf(s, MSL, "Module %d %s a %s\n"
              "      Handles: %s\n", j, fb->getAliasName(), fb->name(), fb->commandList());
        else
          snprintf(s, MSL, "Module %d %s\n"
                   "      Handles: %s\n", j, fb->name(), fb->commandList());
        sendText( msg, s);
        // now the resource list
        if (fb->getLoadedModuleRef() != NULL)
        {
          snprintf(reply, MRL, "      Loaded from '%s'\n", fb->getLoadedFileName());
          sendText(msg, reply);
        }
      }
    }
    if (doResList)
    { // may be both list and resource list
      resPool->print("", reply, MRL);
      sendText(msg, reply);
    }
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (doResList)
  {
    snprintf(reply, MRL, "<help subject=\"%s resource list\">\n", msg->tag.getTagName());
    sendMsg( msg, reply);
    resPool->print("", reply, MRL);
    sendText( msg, reply);
    sendHelpDone(msg);
  }
  else if (doLoad)
  {
    reply[0] = '\0';
    // try load as static module
    result = loadStaticModule( mName, aliasName, reply, MRL);
    if (not result and strlen(reply) == 0)
      // try load as plugin, assuming mName is a filename
      result = loadFunctionModule( mName, reply, MRL, aliasName);
    if (result)
      sendInfo( msg, "done");
    else
      sendWarning( msg, reply);
  }
  else if (doUnload)
  {
    result = unloadFunctionModule( mName);
    if (result)
      sendInfo( msg, "done");
    else
      sendWarning( msg, "Unload failed - use index, filename or first part of name");
  }
  else
    sendWarning( msg, "Unsupported option" );
  //
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::handleServerCommand(UServerInMsg * msg)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  bool aClients = false;
  bool aQueue = false;
  int aQueueCnt = 10;
  bool aName = false;
  bool aNameSpace = false;
  bool aPath = false;
  bool aRead = false;
  bool aPort = false;
  bool aPortSet = false;
  bool aVerbose = false;
  bool aVerboseSet = false;
  bool aVerboseValue = false;
  bool ask4help = false;
  bool aLog = false;
  bool aLogReply = false;
  bool logReply = false;
  bool aLogTimestamp = false;
  bool logTimestamp = true;
  bool aLogClose = false;
  bool logOpen = false;
  bool aTime = false;
  const char * logOpenName = "none";
  int portSet = 0;
  const int MRL = 60000;
  char reply[MRL];
  char script[MRL];
  //char scriptLog[MRL];
  const int MSL = 60;
  char s[MSL];
  char s2[MSL];
  char s3[MSL];
  char s4[MSL];
  int i;
  UServerClient * scl;
  bool aLogServer = false;
  bool aLogServerOpen = true;
  bool aLoad = false;
  bool aQuit = false;
  bool aReplayTime = false;
  bool aFlush = false;
  USmlTagIn * tag;
  UTime t;
  UTime replayTime;
  ULogFile nolog;
  double replayTimeAdd = 0.0;
  //
  i = 0; // param count
  while( msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strncasecmp(attName, "clients", 3) == 0)
      aClients = true;
    else if (strcasecmp(attName, "path") == 0)
      aPath=true;
    else if (strcasecmp(attName, "imagepath") == 0)
    {
      if (strlen(attValue) > 0)
        strncpy(imagePath, attValue, MAX_PATH_LENGTH);
      aPath=true;
    }
    else if (strcasecmp(attName, "datapath") == 0)
    {
      if (strlen(attValue) > 0)
        strncpy(dataPath, attValue, MAX_PATH_LENGTH);
      aPath=true;
    }
    else if (strcasecmp(attName, "replayTime") == 0)
    {
      replayTimeAdd = strtod(attValue, NULL);
      if (replayTimeAdd > 1e8 or (replayTimeAdd < 600.0 and replayTimeAdd > 0.001))
      { // a possible recent timestamp - use
        aReplayTime = true;
        if (replayTimeAdd > 600.0)
        {
          replayTime.setTimeTod(attValue);
          replayTimeAdd = -1.0;
        }
      }
      else
        printf("attribute replayTime has bad time (%s=%s)\n", attName, attValue);
    }
    else if (strcasecmp(attName, "replayPath") == 0)
    {
      if (strlen(attValue) >= 2)
      { // new path
        if (attValue[0] == '/' or
            ((attValue[0]=='~' or attValue[0] == '.') and attValue[1] == '/') or
            (attValue[0] == '.' and attValue[1] == '.' and attValue[2] =='/'))
        { // full path, use as is
          strncpy(replayPath, attValue, MAX_PATH_LENGTH);
        }
        else
        { // relative path - append to data path
          snprintf(replayPath, MAX_PATH_LENGTH, "%s/%s", dataPath, attValue);
        }
      }
      aPath=true;
    }
    else if (strcasecmp(attName, "read") == 0)
    {
      if (strlen(attValue) > 0)
        strncpy(script, attValue, MRL);
      aRead=true;
    }
    else if (strcasecmp(attName, "logClient") == 0)
    {
      aLog=true;
      aLogClose = not str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "logAll") == 0)
    {
      aLogReply=true;
      logReply = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "logTimestamp") == 0)
    {
      aLogTimestamp=true;
      logTimestamp = str2bool2(attValue, true);
    }
/*    else if (strncasecmp(attName, "logClose", 3) == 0)
      aLogClose = true;*/
    else if (strncasecmp(attName, "queue", 3) == 0)
    {
      aQueue = true;
      if (strlen(attValue) > 0)
        aQueueCnt = strtol(attValue, NULL, 0);
    }
    else if (strcasecmp(attName, "name") == 0)
      aName = true;
    else if (strcasecmp(attName, "namespace") == 0)
      aNameSpace = true;
    else if (strcasecmp(attName, "serverLog") == 0)
    {
      aLogServer = true;
      aLogServerOpen = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "callNotice") == 0)
    {
      callNotice = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "port") == 0)
    {
      portSet = strtol(attValue, NULL, 0);
      if ((portSet > 1000) and (portSet < 0x8000))
        aPortSet = true;
      else
        aPort = true;
    }
    else if (strcasecmp(attName, "verbose") == 0)
    {
      aVerbose = true;
      if (strlen(attValue) > 0)
      {
        aVerboseValue = str2bool(attValue);
        aVerboseSet=true;
      }
    }
    else if (strcasecmp(attName, "time") == 0)
      aTime = true;
    else if (strcasecmp(attName, "load") == 0)
      aLoad = true;
    else if (strcasecmp(attName, "quit") == 0)
      aQuit = true;
    else if (strcasecmp(attName, "flush") == 0)
      aFlush = true;
    else
      ask4help = true;
    i++;
  }
  if (i == 0)
    ask4help = true;
/*  if (logServer == NULL)
    snprintf(logServerName, MAX_FILENAME_SIZE, "%s/%s.log", dataPath, appName);*/
  if (ask4help)
  {
    scl = server->getClient(msg->client);
    if (scl != NULL)
    {
      logOpen = scl->isLogOpen();
      logOpenName = scl->getLogFilename();
      logReply = scl->getLogReply();
      logTimestamp = scl->getLogTimestamp();
    }
    //
    sendHelpStart(msg,  "server");
    sendText( msg,  "----- available SERVER options:\n");
    sscanf(name(), "%40s", s);
    snprintf(reply, MRL, "This is %s, a server in the AU Robot Servers series\n",
             servName);
    sendText( msg, reply);
    snprintf(reply, MRL, "port[=port]              Returns (or sets) server port (is %d)\n",
             server->getPort());
    sendText( msg,  reply);
    snprintf(reply, MRL, "load                     Returns server load (is %.1f%%)\n", getLoad() * 100.0);
    sendText( msg,  reply);
    snprintf(reply, MRL, "namespace                Returns namespace of server (is %s)\n", server->getServerNamespace());
    sendText( msg,  reply);
    snprintf(reply, MRL, "clients                  Clients on server (%d client(s))\n",
             server->getClientCnt());
    sendText( msg,  reply);
    sendText( msg,       "path                     Get path settings at server\n");
    snprintf(reply, MRL, "dataPath=\"path\"          Set path to data (logfiles) (is %s)\n", dataPath);
    sendText( msg,  reply);
    snprintf(reply, MRL, "imagePath=\"path\"         Set path to saved images    (is %s)\n", imagePath);
    sendText( msg,  reply);
    snprintf(reply, MRL, "replayPath=\"path\"        Set path for replay files   (is %s)\n", replayPath);
    sendText( msg,  reply);
    snprintf(reply, MRL, "replayTime=\"tod\"         Set replay time (or add if < 600) (is %lu.%06lu)\n", replayTimeNow.getSec(), replayTimeNow.getMicrosec());
    sendText( msg,  reply);
    sendText( msg,       "read=\"file\"              Read a file with server commands (a script)\n");
    snprintf(reply, MRL, "logClient[=false]        Log this clients communication, is open (%s) to %s\n",
               bool2str(logOpen), logOpenName);
    sendText( msg, reply);
    snprintf(reply, MRL, "logAll[=false]           Client log, log also reply to this client (is %s)\n",
               bool2str(logReply));
    sendText( msg, reply);
    snprintf(reply, MRL, "logTime[=false]          Client log timestamp for each entry (is %s)\n", bool2str(logTimestamp));
    sendText( msg, reply);
    snprintf(reply, MRL, "serverLog[=false]        Server log (command queue etc) is open (%s) to %s\n",
             bool2str(logServer.isOpen()), logServer.getLogFileName());
    sendText( msg, reply);
    snprintf(reply, MRL, "callNotice[=false]       Make a console notice before/after queued commands (is %s)\n",
             bool2str(callNotice));
    sendText( msg, reply);
    sendText( msg,       "verbose[=false]          Print more to console if true\n");
    sendText( msg,       "time                     Get the server time\n");
    snprintf(reply, MRL, "queue=N                  list queue with N history (current %d, pending %d)\n",
              server->getRxQueue()->getNextOut(), server->getRxQueue()->getUsedMsgCnt());
    sendText( msg, reply);
    sendText( msg,       "flush                    Calls a fflush(NULL) to flush unsaved (logdata) to disk\n");
    sendText( msg,       "quit                     Kill the server process\n");
    sendText( msg,       "help                     This message\n");
    sendText( msg,       "(on server console see also h for more commands)\n");
    sendText( msg,  "---\n");
    sendText( msg,  "See also HELP MODULE PUSH\n");
    sendText( msg,  "---\n");
    sendHelpDone(msg);
  }
  else
  {
    if (aLoad)
    {
      sscanf(name(), "%40s", s);
      snprintf(reply, MRL, "<%s load=\"%.2f\" name=\"%s\" appName=\"%s\"/>\n",
               msg->tag.getTagName(), getLoad() * 100.0, s, appName);
      sendMsg( msg->client, reply);
      result = true;
    }
    if (aClients)
    { // send client sesponce
      snprintf(reply, MRL, "%d active client(s) on %d", server->getActiveClientCnt(), serverPort);
      sendHelp(msg, reply);
      for (i = 0; i < server->getClientCnt(); i++)
      { //
        if (server->getClient(i)->isActive())
        {
          tag = server->getClient(i)->getNamespaceTag();
          strcpy(s, "noname");
          tag->getAttValue("name", s, MSL);
          strcpy(s2, "0");
          tag->getAttValue("version", s2, MSL);
          strcpy(s3, "0");
          tag->getAttValue("port", s3, MSL);
          strcpy(s4, "0");
          tag->getAttValue("host", s4, MSL);
          snprintf(reply, MRL, "client %d from %s (%s) name='%s' version='%s' port='%s' namespace '%s'", i,
                  server->getClient(i)->getClientName(), s4,
                  s, s2, s3, tag->getTagName());
          sendHelp(msg, reply);
        }
      }
      sendInfo(msg, "done");
      result = true;
    }
    if (aLogServer)
    { // open or close server log
      if (aLogServerOpen)
      { // open server logfile
        // set logname first, as log path may have changed since ULogFile was created
        logServer.setLogName(appName);
        result = logServer.openLog();
        if (result)
          fprintf(logServer.getF(), "# format: Time, (exe-time), queue-index, pending-cmds, client, (result), command\n");
        snprintf(reply, MRL, "server logfile '%s' opened (%s)",
                 logServer.getLogFileName(), bool2str(result));
        sendInfo(msg, reply);
      }
      else
      { // close logfile
        logServer.closeLog();
        sendInfo(msg, "server logfile closed");
      }
    }
    if (aLog)
    {
      scl = server->getClient(msg->client);
      result = (scl != NULL);
      if (result)
      {
        if (aLogClose)
          scl->logClose();
        else
          result = scl->logOpen();
        if (result)
          sendInfo(msg, "done");
        else
        {
          snprintf(reply, MRL, "Failed to open '%s'", scl->getLogFilename());
          sendWarning(msg, reply);
        }
      }
      else
      {
        snprintf(reply, MRL, "No such client '%d'", msg->client);
        sendWarning(msg, reply);
      }
    }
    if (aLogReply)
    {
      scl = server->getClient(msg->client);
      result = (scl != NULL);
      if (result)
      {
        scl->setLogReply( logReply);
        sendInfo(msg, "done");
      }
      else
      {
        snprintf(reply, MRL, "No such client '%d'", msg->client);
        sendWarning(msg, reply);
      }
    }
    if (aLogTimestamp)
    {
      scl = server->getClient(msg->client);
      result = (scl != NULL);
      if (result)
      {
        scl->setLogTimestamp(logTimestamp);
        sendInfo(msg, "done");
      }
      else
      {
        snprintf(reply, MRL, "No such client '%d'", msg->client);
        sendWarning(msg, reply);
      }
    }
    if (aRead)
    {
      result = executeScriptFile( script, false, msg->client);
      if (result)
        sendInfo(msg, "script queued");
      else
        sendWarning(msg, "read script failed");
    }
    if (aPath)
    {
      sendHelpStart(msg, "Path settings");
      snprintf(reply, MRL, "    imagePath: %s\n", imagePath);
      sendText(msg, reply);
      snprintf(reply, MRL, "     dataPath: %s\n", dataPath);
      sendText(msg, reply);
      snprintf(reply, MRL, "   replayPath: %s\n", replayPath);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      // send also as tag
      snprintf(reply, MRL, "<%s imagePath=\"%s\" dataPath=\"%s\" replayPath=\"%s\"/>\n",
               msg->tag.getTagName(), imagePath, dataPath, replayPath);
      sendMsg(msg, reply);
      result = true;
    }
    if (aQueue)
    { // send client sesponce
      server->getRxQueue()->list("command queue", reply, MRL, aQueueCnt);
      sendHelpStart(msg, "Server command queue list");
      sendText(msg, reply);
      sendHelpDone(msg);
      result = true;
    }
    if (aName)
    { // send server name
      sscanf(name(), "%40s", s);
      snprintf(reply, MRL, "<%s name=\"%s\"/>\n", msg->tag.getTagName(), s);
      sendMsg(msg, reply);
      result = true;
    }
    if (aNameSpace)
    { // send server namespace
      snprintf(reply, MRL, "<%s namespace=\"%s\"/>\n",
               msg->tag.getTagName(), server->getServerNamespace());
      sendMsg(msg, reply);
      result = true;
    }
    if (aPort)
    { // send port number
      snprintf(reply, MRL, "<%s port=\"%d\"/>\n",
               msg->tag.getTagName(), server->getPort());
      sendMsg(msg, reply);
      result = true;
    }
    if (aPortSet)
    { // send server name
/*      snprintf(reply, MRL, "Trying to change to port %d! (try: server port)", portSet);
      sendWarning( msg, reply);*/
      if (server->getPort() != portSet and portSet > 0)
      { // new port value
        server->setPort(portSet);
        // wait for port change
        Wait(2.1);
        if (not server->isOpen4Connections())
        {
          fprintf(stderr, "Failed to change port number to %d\n", portSet);
          snprintf(reply, MRL, "Failed to use port %d! (already in use?)", portSet);
          sendWarning(msg, reply);
        }
        else
          sendInfo(msg, "done");
      }
      else
      {
        snprintf(reply, MRL, "Port is set to %d already", portSet);
        sendInfo(msg, reply);
      }
      result = true;
    }
    if (aVerbose)
    {
      if (aVerboseSet)
      {
        verboseMessages = aVerboseValue;
        server->setVerbose(aVerboseValue);
      }
      snprintf(reply, MRL, "<%s verbose=\"%s\"/>\n",
               msg->tag.getTagName(), bool2str(verboseMessages));
      result = sendMsg(msg, reply);
    }
    if (aFlush)
    { // kill the server
      fflush(NULL);
      result = sendMsg(msg, "<server info=\"fflush(NULL) called\"/>\n");
    }
    if (aQuit)
    { // kill the server
      killServer(msg);
    }
    if (aTime)
    {
      t.Now();
      t.getTimeAsString(s, true);
      t.GetDateString(s2);
      snprintf(reply, MRL, "<%s tod=\"%lu.%lu\"  date=\"%s\" time=\"%s\"/>\n",
               msg->tag.getTagName(), t.getSec(), t.getMicrosec(), s2, s);
      result = sendMsg(msg, reply);
    }
    if (aReplayTime)
    { // add to current replay time if small, or use as is if big
      if (replayTimeAdd > 0.0)
        replayTime = replayTimeNow + replayTimeAdd;
      replayAdvanceTime(replayTime);
      result = sendMsg(msg, "<server info=\"done\"/>\n");
    }
    if (not result)
      sendInfo(msg, name());
  }
  return result;
}

//////////////////////////////////////////////

void UCmdExe::killServer(UServerInMsg * msg)
{ // send kill signam to server - is traped, and shut down
  // in an orderly fasion
  UTime t;
  int pid;
  const int MSL = 300;
  char s[MSL];
  //
  pid = getpid();
  t.now();
  snprintf(s, MSL, "*** Server (pid:%d) to be killed (TERM)", pid);
  if (msg != NULL)
    sendInfo(msg, s);
  if (logServer.isOpen() and msg != NULL)
  {
    fprintf(logServer.getF(), "#%lu.%06lu %s  by client %d\n", t.getSec(),
            t.getMicrosec(), s, msg->client);
  }
  fprintf(stdout, "#%lu.%06lu %s  by client %d\n", t.getSec(),
          t.getMicrosec(), s, msg->client);
  raise(SIGTERM);
  //kill(getpid(), SIGTERM);
  if (msg != NULL)
  {
    snprintf(s, MSL, "Signal (TERM) is now send to server process %d", pid);
    sendInfo(msg, s);
    printf("%s\n", s);
  }
}
//////////////////////////////////////////////

bool UCmdExe::sysServerHelp(UServerInMsg * msg)
{
  bool result = true;
  bool ask4help = false;
  const int MRL = 200;
  char reply[MRL];
  const int MSL = 200;
  char s[MSL];
  int i;
  //
  i = 0; // param count
  ask4help = true;
  if (ask4help)
  {
    sendMsg( msg, "<help subject=\"server help\">\n");
    sendText( msg, "----- Main help:\n");
    snprintf(reply, MRL, "This is %s a server in the AU Robot Server series\n",
             servName);
    sendText( msg, reply);
    server->getHostName(s, MSL);
    snprintf(reply, MRL, "running on host %s port %d\n", s, server->getPort());
    sendText( msg, reply);
    snprintf(reply, MRL, "with %d modules and %d ressources.\n",
             getFuncCnt(), resPool->getResCnt());
    sendText( msg, reply);
    sendText( msg, "---\n");
    sendText( msg, "Available commands (from currently loaded modules):\n");
    snprintf(reply, MRL, " - %s\n", commandList());
    sendText( msg, reply);
    for (i = 0; i < funcCnt; i++)
    {
      if (func[i] != NULL)
      {
        snprintf(reply, MRL, " - %s\n", func[i]->commandList());
        sendText( msg, reply);
      }
    }
    sendText( msg, "All commands has a help option, e.g.:\n");
    sendText( msg, "server help       Help on server core functions\n");
    sendText( msg, "module help       Module list and addition/removal of modules\n");
    sendText( msg, "q                 Close connection to this client by server\n");
    sendText( msg, "(ping [tod=sss.usec] Time difference analysis (TCP only))\n");
    sendText( msg, "---\n");
    sendText( msg, "A command should be packed in XML brackets and must be terminated by a \\n (newline)\n");
    sendText( msg, "e.g.:\n");
    sendText( msg, "<module list/>   or if you are lazy just:\n");
    sendText( msg, "module list\n");
    sendText( msg, "---\n");
    sendMsg( msg, "</help>\n");
    sendInfo(msg, "done");
  }
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::handleServerPushCommand(UServerInMsg * msg)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int MVL = MAX_MESSAGE_LENGTH_TO_CAM;
  char attValue[MVL];
  char pushCmd[MVL] = "";
  char pushCall[MVL] = "";
  int n;
  double interval = 1.0;
  int countGood = -1;
  int countTotal = -1;
  bool isAFlush = false;
  UServerPushElement * qe;
  const int MRL = 2000;
  char reply[MRL];
  bool ask4list = false;
  bool ask4help = false;
  //
  while (msg->tag.getNextAttribute(attName, attValue, MVL))
  {
    if (strcmp(attName, "flush") == 0)
    { // flush
      isAFlush = true;
      if (pushCmd[0] == 0)
        strncpy(pushCmd, attValue, MVL);
    }
    else if ((strcasecmp(attName, "good") == 0) or
              (strcasecmp(attName, "g") == 0))
      // position value
      n = sscanf(attValue, "%d", &countGood);
    else if ((strcasecmp(attName, "total") == 0) or
              (strcasecmp(attName, "n") == 0))
      // position value
      n = sscanf(attValue, "%d", &countTotal);
    else if ((strcasecmp(attName, "interval") == 0) or
              (strcasecmp(attName, "t") == 0))
      // position value
      n = sscanf(attValue, "%lf", &interval);
    else if (strcasecmp(attName, "cmd") == 0)
      // the command to push
      strncpy(pushCmd, attValue, MVL);
    else if (strcasecmp(attName, "call") == 0)
      // the command to push
      strncpy(pushCall, attValue, MVL);
    else if (strcasecmp(attName, "list") == 0)
      ask4list = true;
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    // else ignore attribute
  }
  //
  if (ask4help)
  {
    sendMsg( msg, "<help subject=\"PUSH help\">\n");
    sendText( msg, "---- Available push options:\n");
    sendText( msg, "n=count    Repeat command 'count' times\n");
    sendText( msg, "g=count    Repeat command 'count' successful times\n");
    sendText( msg, "t=sec      Repeat command every 'sec' seconds (decimal)\n");
    sendText( msg, "cmd=\"cmd\"  Command to execute at interval\n");
    sendText( msg, "flush=cmd  Flush commands of 'cmd' type (default all)\n");
    sendText( msg, "list       List status of push queue\n");
    sendText( msg, "---\n");
    sendMsg( msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else
  {
    if (isAFlush)
    { // flush and senr reply
      n = push.systemQueueflush(msg->client, pushCmd);
      snprintf(reply, MRL, "<%s done=\"%s\" flushed=\"%d\"/>\n",
              msg->tag.getTagName(), bool2str(n > 0), n);
      sendMsg(msg, reply); //, strlen(reply));
    }
    else if (strlen(pushCmd) > 0 or strlen(pushCall) > 0)
    { // space for more?
      result = false;
      qe = push.getFreeQueueElement();
      if (qe != NULL)
      { // add to queue
        qe->clear();
        qe->interval = interval;
        qe->countGoodTarget = countGood;
        qe->countTotalTarget = countTotal;
        qe->nextExeTime.Now();
        qe->nextExeTime += qe->interval;
        if (strlen(pushCmd) > 0)
        { // is a queueable command
          qe->toDo.setMessage(msg->client, pushCmd, strlen(pushCmd), false);
          qe->functionIndex = getFunctionOwner(&qe->toDo);
          if (qe->functionIndex >= 0)
          { // must be a valid function
            qe->toDo.serverPushCommand = true;
            qe->activeCmd = true;
            result = true;
          }
        }
        if (strlen(pushCall) > 0)
        { // is a callable command
          result = qe->setCall(msg->client, pushCall);
          qe->activeCall = result;
        }
      }
      //
      snprintf(reply, MRL, "<%s done=\"%s\"/>\n",
      msg->tag.getTagName(), bool2str(result));
      // tell client
      sendMsg(msg, reply); //, strlen(reply));
    }
    else if (ask4list)
    {
      sendMsg(msg, "<help subject=\"timed push command list\">\n");
      push.print("push", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    {
      sendWarning(msg, "No command specified");
    }
  }
  //
  return result;
}


//////////////////////////////////////////////

void UCmdExe::closeClient(int clientIdx)
{
  UServerClient * cnn;
  //
  cnn = server->getClient(clientIdx);
  if (cnn != NULL)
    cnn->stopConnection(true, server->getServerNamespace());
}

////////////////////////////////////////////////

void UCmdExe::print(const char * preString)
{
/*  int i;
  const int SL = 30;
  char s[SL];*/
  //
  server->print(preString);
/*  printf("%shas %d loaded functions\n",
        preString, funcCnt);
  printf(" - func core is '%s'\n",
          name());
  printf("      Handling: '%s'\n",
          commandList());
  printf("      Resources: %s\n",  resID);
  for (i = 0; i < funcCnt; i++)
  {
    if (func[i] != NULL)
    {
      snprintf(s, SL, " - func #%d ", i);
      func[i]->print(s);
    }
  } */
  printf("Idle loop %d, current command: %s\n", idleLoopCnt, currentMessage);
  push.print("Push queue");
  //printRess("Resources");
}

///////////////////////////////////////////////////

bool UCmdExe::sendWarning(UServerInMsg * msg, const char * warningText)
{
  bool result = false;
  const int MSL = 1000;
  char s[MSL];
  char s1[MSL];
  //
  snprintf(s, MSL, "<%s warning=\"%s\"/>\n",
      msg->tag.getTagName(), str2xml(s1, MSL, warningText));
  if (msg->client >= 0)
    result = sendMsg(msg->client, s, strlen(s));
  else
    printf("%s", s);
  //
  return result;
}

///////////////////////////////////////////////////

bool UCmdExe::sendError(UServerInMsg * msg, const char * errorText)
{
  bool result = false;
  const int MSL = 1000;
  char s[MSL];
  char s1[MSL];
  //
  snprintf(s, MSL, "<%s error=\"%s\"/>\n",
      msg->tag.getTagName(), str2xml(s1, MSL, errorText));
  result = sendMsg(msg->client, s, strlen(s));
  //
  return result;
}

///////////////////////////////////////////////////

bool UCmdExe::sendDebug(UServerInMsg * msg, const char * debugText)
{
  bool result = false;
  const int MSL = 1000;
  char s[MSL];
  char s1[MSL];
  //
  snprintf(s, MSL, "<%s debug=\"%s\"/>\n",
      msg->tag.getTagName(), str2xml(s1, MSL, debugText));
  result = sendMsg(msg->client, s, strlen(s));
  //
  return result;
}

///////////////////////////////////////////////////

bool UCmdExe::sendInfo(UServerInMsg * msg, const char * infoText)
{
  bool result = false;
  const int MSL = 1000;
  char s[MSL];
  char s1[MSL];
  //
  snprintf(s, MSL, "<%s info=\"%s\"/>\n",
      msg->tag.getTagName(), str2xml(s1, MSL, infoText));
  result = sendMsg(msg->client, s, strlen(s));
  //
  return result;
}

///////////////////////////////////////////////////

bool UCmdExe::sendHelp(UServerInMsg * msg, const char * infoText)
{
  bool result = false;
  const int MSL = 1000;
  char s[MSL];
  char s1[MSL];
  //
  snprintf(s, MSL, "<help info=\"%s\"/>\n",
      str2xml(s1, MSL, infoText));
  result = sendMsg(msg->client, s, strlen(s));
  //
  return result;
}

///////////////////////////////////////////////////

bool UCmdExe::sendHelpStart(UServerInMsg * msg, const char * infoText)
{
  const int MSL = 1000;
  char s[MSL];
  char s1[MSL];
  //
  snprintf(s, MSL, "<help subject=\"%s\">\n",
           str2xml(s1, MSL, infoText));
  return sendMsg(msg->client, s, strlen(s));
}

///////////////////////////////////////////////////

bool UCmdExe::sendHelpDone(UServerInMsg * msg)
{
  sendMsg(msg->client, "</help>\n");
  return sendInfo(msg, "done");
}


//////////////////////////////////////////////

void * handleMessagesThreadCall(void * obj)
{ // call the hadling function in provided object
  UCmdExe * ce = (UCmdExe *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

//////////////////////////////////////////////

void * callRunIdle(void * obj)
{ // call the hadling function in provided object
  UCmdExe * ce = (UCmdExe *)obj;
  ce->runIdle();
  pthread_exit((void*)NULL);
  return NULL;
}

//////////////////////////////////////////////

bool UCmdExe::handleMessagesThreadStart()
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
                  &handleMessagesThreadCall, (void *)this) == 0);
    // start also the idle thread
    result = (pthread_create(&threadHandle, &thAttr,
                  &callRunIdle, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}


//////////////////////////////////////////////

void UCmdExe::runIdle()
{ // wake main thread at regulat intervals, if nothing else to do
  const double wakeupSec = 1.25;
  while (not threadStop)
  {
    Wait(wakeupSec);
    event();
  }
}

//////////////////////////////////////////////

void UCmdExe::run()
{
  bool isOK;
  int i, n;
  UServerPush ** sp;
  UTime t, t2, t5s;
  double lastIdleTime = 0.0;
  int lastVarPoolUpdate = -1;
  //
  threadRunning = true;
  t.now();
  t5s.now();
  while (not threadStop)
  { // tell clients that server is alive
    server->serverIsAlive();
    // is vital parameters changed?
    if (isVarPoolUpdated(lastVarPoolUpdate, &lastVarPoolUpdate))
    {
      server->setAllowConnections(varAllowConnection->getValueBool());
    }
    // test if any resource in replay mode has advanced its replay state
    isOK = resPool->handleReplay();
    if (not isOK)
      isOK = handleOneMessageFromQueue();
    if (not isOK)
      // no client pull, so test for
      // timed server push commands
      isOK = handleOneServerPushMessage();
    //
    // service event based push commands
    if (not isOK and (pushWatchCnt > 0))
    { // test for threadless push commands
      // from this list of pointers to updated assets
      sp = pushWatch;
      // service just the existing pushes (new may be added while servicing others)
      n = pushWatchCnt;
      for (i = 0; i < n; i++)
      { // asset is updated - test for push commands
        (*sp)->servicePendingPushCmds();
        sp++;
      }
      // all should now be serviced remove used pointers
      pushWatchLock.lock();
      if (n == pushWatchCnt)
        // no new jobs added, so we are finished
        pushWatchCnt = 0;
      else if (pushWatchCnt > 0 and n > 0)
      { // new jobs are added while servicing others, remove serviced n jobs from list
        pushWatchCnt -= n;
        memmove(pushWatch, &pushWatch[n],
                sizeof(UServerPush *) * pushWatchCnt);
        // debug
        // printf("UCmdExe::(run) new job arrived - now %d - while serviced %d (is OK)\n", pushWatchCnt, n);
        // debug end
      }
      else
      { // highly unlikely to get here, but OK
      }
      isOK = (n > 0);
      // release stack for more additions
      pushWatchLock.unlock();
    }
    updateLocalVar();
    if (not isOK and (pushWatchCnt == 0))
    { // nothing to do last round
      idleStart.now();
      // wait for some action
      // debug
      // ensure logfile data is flushed
//       if (logServer.isLogOpen())
//       {
//         fprintf(logServer.getF(), "%lu.%lu going idle, queue  %d\n", idleStart.getSec(),
//                 idleStart.getMicrosec(),
//                 server->getRxQueue()->getUsedMsgCnt());
//         fflush(logServer.getF());
//       }
      // debug end
//      struct timespec timeout;
//       timeout.tv_sec =  1;
//       timeout.tv_nsec = 0;
      sem_wait(&actionFlag); //, &timeout);
      // debug
      double waited = idleStart.getTimePassed();
      if (waited < 0.01)
        Wait(0.01);
      // debug end
//       if (logServer.isLogOpen())
//       {
//         UTime tNow;
//         tNow.now();
//         fprintf(logServer.getF(), "%lu.%lu awake idle, queue  %d\n", tNow.getSec(),
//                 tNow.getMicrosec(),
//                 server->getRxQueue()->getUsedMsgCnt());
//         fflush(logServer.getF());
//       }
      // summ idle time
      idleTime += idleStart.getTimePassed();
      if (t5s.getTimePassed() > 5.0)
      { // time to calculate new load
        double idle5sec = t5s.getTimePassed();
        double idleSec = fmax(0.0, (idleTime - lastIdleTime));
        lastIdleTime = idleTime;
        varIdle->setDouble(idleSec/idle5sec, 0);
        t5s.now();
      }
    }
  }
  threadRunning = false;
}

//////////////////////////////////////////////

void UCmdExe::stop(bool andWait)
{ // stop function handler
  //
  if (not stopping)
  {
    stopping = true;
    if (server != NULL)
      // stop accepting clients, and disconnect current clients
      server->stop(andWait);
    // stop handling commands
    threadStop = true;
    Wait(0.5);
    // remove all entries from command-queue
    server->getRxQueue()->clear();
    // unload modules - and stop their threads
    unloadAllModules();
    // wait for server thread to terminate
    if (andWait and threadRunning)
      pthread_join(threadHandle, NULL);
    //stopping = false;
    printf("server stopped\n");
  }
}

//////////////////////////////////////////////

bool UCmdExe::executePushFunction(int functionIndex, UServerInMsg * msg, void * extra)
{
  bool result;
  // lock for function load/unload during operation
  lock();

  result = executeFunction(functionIndex, msg, extra, true);
  unlock();
  return result;
}

/////////////////////////////////////////////////

bool UCmdExe::executeFunction(int functionIndex, UServerInMsg * msg, void * extra, bool aPush)
{ // start a function
  bool success = false;
  UServerClient * cnn;
  bool locked = false;
  UTime t;
  int n;
  //
  // lock communication to client
  cnn = server->getClient(msg->client);
  // debug(
  //strcat(currentMessage, "1");
  // debug end
  if (cnn != NULL)
    if (cnn->isActive())
      locked = (cnn->lock());
  // debug(
  //strcat(currentMessage, "2");
  // debug end
  if (locked or (msg->client < 0))
  { // either a locked connection or a client is the server console
    t.now();
    if (logServer.isOpen())
    {
      n = server->getRxQueue()->getNextOut();
      fprintf(logServer.getF(), "%lu.%06lu %2dn (%2dq) %2dc entry: %s\n",
              t.getSec(), t.getMicrosec(),
              n, server->getRxQueue()->getUsedMsgCnt(),
              msg->client, msg->message);
    }
    if (callNotice and not aPush)
    {
      n = server->getRxQueue()->getNextOut();
      fprintf(stdout, "%lu.%06lu %2dn (%2dq) %2dc entry: %s\n",
              t.getSec(), t.getMicrosec(),
              n, server->getRxQueue()->getUsedMsgCnt(),
              msg->client, msg->message);
    }
    //msg->exe = this;
    if (functionIndex == MAX_FUNCTION_COUNT)
      success = systemFunction(msg);
    else
    {
  // debug(
  //strcat(currentMessage, "3");
  // debug end
      UFunctionBase * fb = func[functionIndex];
      if (fb != NULL)
      { // a push command has not set index to sub-command
        if (aPush)
          fb->isMine(msg->tag.getTagName());
        // time to go
        success = fb->newCommand(msg, extra);
      }
  // debug(
  //strcat(currentMessage, "4");
  // debug end
    }
    if (callNotice and not aPush)
    {
      n = server->getRxQueue()->getNextOut();
      fprintf(stdout, "%lu.%06lu (took %.5fs) %2dn (%2dq) %2dc (%s) exit: %s\n",
              t.getSec(), t.getMicrosec(), t.getTimePassed(),
              n, server->getRxQueue()->getUsedMsgCnt(),
              msg->client, bool2str(success), msg->message);
    }
    if (logServer.isOpen())
    {
      n = server->getRxQueue()->getNextOut();
      fprintf(logServer.getF(), "%lu.%06lu (took %.5fs) %2dn (%2dq) %2dc (%s) exit: %s\n",
              t.getSec(), t.getMicrosec(), t.getTimePassed(),
              n, server->getRxQueue()->getUsedMsgCnt(),
              msg->client, bool2str(success), msg->message);
    }
    // debug(
  //strcat(currentMessage, "5");
  // debug end
  }
  if (locked)
    cnn->unlock();
  // debug(
  //strcat(currentMessage, "6");
  // debug end
  //
  return success;
}

//////////////////////////////////////////////

bool UCmdExe::isClientAlive(int clientIdx, double holdOffTime)
{
  bool result;
  UServerClient * cnn;
  //
  if (clientIdx < 0)
    // console, sequencer or other phony client
    // these are per definition always alive
    result = true;
  else
  {
    cnn = server->getClient(clientIdx);
    result = (cnn != NULL);
    if (result)
      result = cnn->isActive();
    if (result and holdOffTime > 1e-3)
      result = cnn->connectTime.getTimePassed() > holdOffTime;
  }
  //
  return result;
}

//////////////////////////////////////////////

void * UCmdExe::getLoadedModuleRef(int functionIndex)
{
  void * result = NULL;
  UFunctionBase * f;
  //
  if ((functionIndex >= 0) and (functionIndex < funcCnt))
  {
    f = func[functionIndex];
    if (f != NULL)
      // function still valid
      result = f->getLoadedModuleRef();
  }
  return result;
}


///////////////////////////////////////////////


bool UCmdExe::unloadAllModules()
{
  bool result = true;
  bool unloaded;
  int i;
  // flush any open output files
  fflush(NULL);
  // stop all threads in ressources prior to unload
  resPool->stop(true);
  // save settings
  resPool->saveSettings();
  // unload modules
  for (i = funcCnt - 1; i >= 0; i--)
  {
    unloaded = unloadThisModule(i);
    if (not unloaded)
    {
      result = false;
      printf("Unload of module %d failed\n", i);
    }
  }
  return result;
}

///////////////////////////////////////////////

bool UCmdExe::unloadFunctionModule(const char * value)
{
  UFunctionBase * fu = NULL;
  int i = -1;
  int n;
  bool result = false;
  void * module = NULL;
  char * p1;
  const char * p2;
  //
  if (strlen(value) > 2)
  {
    for (i = funcCnt - 1; i >= 0; i--)
    { // search for a match in either filename or module name
      fu = func[i];
      if (fu != NULL)
      {
        module = getLoadedModuleRef(i);
        if ((module != NULL) and
             (strstr(fu->getLoadedFileName(), value) != NULL))
        {
          result = true;
          break;
        }
        else if (strlen(fu->getAliasName()) > 0)
        { // try the alias name
          if (strstr(fu->getAliasName(), value) != NULL)
          {
            result = true;
            break;
          }
        }
        else
        { // try if function module (pre) name is mentionen
          p2 = fu->name();
          p1 = (char*)strchr(p2, ' ');
          if (p1 != NULL)
            n = p1 - p2; // compare to first space only
          else
            n = strlen(p2); // no space, so use full name
          if ((n == int(strlen(value))) and (strncasecmp(p2, value, n) == 0))
          { // this is OK too
            result = true;
            break;
          }
        }
      }
    }
  }
  else
  { // assume value is the module resource number
    i = strtol(value, &p1, 10);
    // value must start with a valid integer number
    result = (p1 != value);
  }
  if (result)
    result = unloadThisModule(i);
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool UCmdExe::unloadThisModule(int funcIdx)
{
  bool result;
  UFunctionBase * fu = NULL;
  void * module = NULL;
  //delete_func * delBase;
  //
  result = ((funcIdx >= 0) and (funcIdx < funcCnt));
  if (result)
  {
    fu = func[funcIdx];
    if (fu != NULL)
      module = getLoadedModuleRef(funcIdx);
    else
      result = false;
  }
  if (result)
  { // unload module
    printf("releasing module %d %s - %s\n", funcIdx, fu->getLoadedFileName(), fu->name());
    // remove from server list
    deleteFunction(fu);
    // delete object will call object destructor
    delete fu;
    // unload shared library module
    if (module != NULL)
    { // function is from a loaded module
      // get destructor function from referenced loadable module
      //delBase = (delete_func*) dlsym(module, "deleteFunc"); // dlfunc
      dlerror();
      //*(void **) (&delBase) = dlsym(module, "deleteFunc");
      // call destructor to release memory
      //if (delBase != NULL)
      //  delBase(fu);
      // close library - not needed anymore
      dlclose(module);
    }
  }
  return result;
}

///////////////////////////////////////////////

UFunctionBase * UCmdExe::loadFunctionModule(const char * moduleFileName, char * buff, int buffCnt, const char * aliasName)
{
  void * module = NULL;
  UFunctionBase * func = NULL;
  create_func * newBase;
  bool result;
  const int MSL = 300;
  char s[MSL];
  char mxs[MSL];
  char *p1 = buff;
  char *p2;
  int m = 0, mx = buffCnt;
  UTime loadTime;
  const char * errTxt;
  //
  if (p1 == NULL)
  { // use local error buffer
    p1 = mxs;
    mx = MSL;
  }
  p2 = p1;
  result = strlen(moduleFileName) > 0;
  if (result)
  { // try to load as as a plufin file
    loadTime.now();
    module = dlopen (moduleFileName, RTLD_NOW | RTLD_GLOBAL); // RTLD_LAZY); // RTLD_LAZY|RTLD_GLOBAL
    //module = dlopen (moduleFileName, RTLD_LAZY);
    result = (module != NULL);
    if (not result)
    {
      strncpy(s, dlerror(), MSL);
      snprintf(p1, mx, "Module load error: %s\n", s);
      if (strstr(p1, "FunctionBase") != NULL)
      {
        m += strlen(p1);
        p1 = &p2[m];
        snprintf(p1, mx - m, "(Server has changed."
            " Plugin probably needs a recompile with new server header files).\n");
      }
      m += strlen(p1);
      p1 = &p2[m];
    }
  }
  if (result)
  { // module loaded, create function
    // clear error
    dlerror();
    //newBase = (create_func *) dlsym(module, "createFunc");
    *(void **) (&newBase) = dlsym(module, "createFunc");
    // get potential error string
    errTxt = dlerror();
    // fail if error
    result = (errTxt == NULL);
    if (not result)
    {
      snprintf(p1, mx - m, "Module load symbol error: %s\n", errTxt);
      m += strlen(p1);
      p1 = &p2[m];
    }
  }
  if (result)
  { // add function
    func = newBase(NULL);
    result = (func != NULL);
    if (not result)
    {
      snprintf(p1, mx - m, "Module create function failed! :%s\n", moduleFileName);
      m += strlen(p1);
      p1 = &p2[m];
    }
  }
  if (result)
  {
    //  printf("Loaded dynamic module '%s' from %s\n", func->name(), p1);
    // save pointer and name in function
    func->setAliasName(aliasName);
    func->setLoadedModuleRef(module, moduleFileName);
    func->setLoadTime(loadTime);
    addFunction(func);
    printf("Module %s loaded\n", moduleFileName);
  }
  else
  { // release unusable handle
    if (module != NULL)
      dlclose(module);
    // print error on server console
    fprintf(stderr, "%s\n", p2);
  }
  return func;
}

//////////////////////////////////////////////

void UCmdExe::resourcesUpdated()
{ // update all loaded functions
  int i, j;
  UResBase * rr, * rrJ;
  //
  for (i = 0; i < funcCnt; i++)
    resourcesUpdate(i);
  //
  for (i = 0; i < resPool->getResCnt(); i++)
  {
    rr = resPool->getResource(i);
    if (rr != NULL)
    { // tell also fast responce server (server port)
      server->setResource(rr, false);
      // tell other resources about the change
      for (j = 0; j < resPool->getResCnt(); j++)
      {
        rrJ = resPool->getResource(j);
        if (rrJ != NULL and rr != rrJ)
          rrJ->setResource( rr, false);
      }
    }
  }
}

//////////////////////////////////////////////

void UCmdExe::resourcesUpdate(int funcIdx)
{
  UFunctionBase * fn;
  int j;
  UResBase * re;
  bool updated;
  //
  fn = func[funcIdx];
  if (fn != NULL)
  {
    updated = false;
    for (j = 0; j < resPool->getResCnt(); j++)
    {
      re = resPool->getResource(j);
      if (re != NULL)
      {
        if (resPool->getResFunc(j) != funcIdx)
          // this is not the owner of the resource
          if (fn->setResource(re, false))
            updated = true;
      }
    }
    if (updated)
      fn->resourceUpdated();
  }
}

//////////////////////////////////////////////

bool UCmdExe::isResourceOwner(const char * resID, UFunctionBase * owner)
{
  UResBase * res;
  int funcIdx;
  bool result = false;
  //
  res = resPool->getResource(resID);
  if (res != NULL)
  { // there is a resourrce, get the ressource owner index
    funcIdx =res->getResFuncIdx();
    // compare owner and resource owner
    result = (func[funcIdx] == owner);
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::postCommand(int client, const char * command)
{
  bool result;
  result = server->getRxQueue()->addMessage( client, command, strlen(command), false);
  // this is also an event that should wake main thread
  // debug
  logServer.toLog("posted Command:", command);
  // debug end
  event();
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::isRxQueueFull()
{
  bool result;
  result = server->getRxQueue()->isFull();
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::isRxQueueEmpty()
{
  bool result;
  result = server->getRxQueue()->isEmpty();
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::executeScriptFile(const char * cfn, bool andWait, int forClient)
{
  FILE * sf;
  const int MSL = 1300;
  char sl[MSL];
//  char ffn[MSL];
  const int MRL = 200;
  char reply[MRL];
  bool result;
  char * s, *e;
  int line = 0;
  //
  result = (cfn != NULL);
//   if (result)
//   { // add path if needed
//     if ((cfn[0] == '/') or (cfn[0] == '.'))
//       strncpy(ffn, cfn, MSL);
//     else
//       snprintf(ffn, MSL, "%s/%s", dataPath, cfn);
//   }
  //
  sf = fopen(cfn, "r");
  result = (sf != NULL);
  if (result)
  {
    printf("### executing script in file '%s' (client %d)\n", cfn, forClient);
    while (true)
    {
      if (isRxQueueEmpty() or not andWait)
      { // space for at least one command more
        s = fgets(sl, MSL, sf);
        if (s == NULL)
          break;
        line++;
        // skip white space
        while (isspace(*s))
          s++;
        if (*s > ' ')
        {
          if (isRxQueueFull())
          {
            result = false;
            break;
          }
          e = &s[strlen(s)];
          while (*e <= ' ') // remove trailing linefeed (etc)
            *e-- = '\0';
          if (isalpha(*s) or (*s == '<'))
          { // assumed to be a command
            postCommand(forClient, s);
            printf("#%d: %s\n", line, s);
          }
          else if (*s > ' ')
            // not a command (probably a remark)
            printf("#%d: %s\n", line, s);
        }
        // else just dump - probably an empty line
      }
      else
        // wait for some last command to be executed
        Wait(0.04);
    }
    if (verboseMessages);
      printf("### finished %d lines in script\n\n", line);
    if (forClient >= 0)
    {
      snprintf(reply, MRL, "<help info=\"finished %d lines in script\"/>\n", line);
      sendMsg(forClient, reply);
    }
  }
  else
  { // print to server console
    printf("### scriptfile '%s' could not be opened!\n", cfn);
    if (forClient >= 0)
    { // tell client if from a client
      snprintf(reply, MRL, "<help warning=\"scriptfile '%s' could not be opened!\"/>\n", cfn);
      sendMsg(forClient, reply);
    }
  }
  if (sf != NULL)
    fclose(sf);
  return result;
}

///////////////////////////////////////////////////

bool UCmdExe::sendText(UServerInMsg * msg, const char * text)
{
  bool result = false;
  const int MSL = 300;
  char s[MSL + 1];
  int n;
  const char * p1 = text;
  const int MRL = 1500;
  char txml[MRL];
  //
  do
  {
    n = strlen(p1);
    if (n < MSL)
      str2xmlMin(txml, MRL, p1);
    else
    { // take a chunk only
      strncpy(s, p1, MSL);
      s[MSL] = '\0';
      // expand to XML compatible
      str2xmlMin(txml, MRL, s);
      // advance to next chunk
      p1 = &p1[MSL];
    }
    result = sendMsg(msg, txml, strlen(txml));
  } while ((n >= MSL) and result);
  //
  return result;
}

//////////////////////////////////////////////

bool UCmdExe::getStaticHelpList(char * list, const int listCnt)
{ // static modules available from here
  char * p1 = list;
  int n = 0;
  //
  snprintf(p1, listCnt - n, "load='odoPose'    Load odometry pose history resource\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='utmPose'    Load GPS based UTM pose history resource\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='mapPose'    Load map pose history resource\n");
  n += strlen(p1);
  p1 = &list[n];
  //
  return true;
}

//////////////////////////////////////////////

bool UCmdExe::loadStaticModule(const char * moduleName, const char * aliasName, char * why, const int whyCnt)
{ // static modules available here
  bool result = true;
  UFunctionPoseHist * fposeHist;
  //
  if (strcasecmp(moduleName, UResPoseHist::getOdoPoseID()) == 0)
  {
    fposeHist = new UFunctionPoseHist();
    if (fposeHist != NULL)
    {
      if (strlen(aliasName) > 0)
        fposeHist->setAliasName(aliasName);
      else
        fposeHist->setAliasName(UResPoseHist::getOdoPoseID());
      addFunction(fposeHist);
    }
  }
  else if (strcasecmp(moduleName, UResPoseHist::getUtmPoseID()) == 0)
  { // the alias neme is implicit
    fposeHist = new UFunctionPoseHist();
    if (fposeHist != NULL)
    {
      if (strlen(aliasName) > 0)
        fposeHist->setAliasName(aliasName);
      else
        fposeHist->setAliasName(UResPoseHist::getUtmPoseID());
      addFunction(fposeHist);
    }
  }
  else if (strcasecmp(moduleName, UResPoseHist::getMapPoseID()) == 0)
  { // the alias neme is implicit
    fposeHist = new UFunctionPoseHist();
    if (fposeHist != NULL)
    {
      if (strlen(aliasName) > 0)
        fposeHist->setAliasName(aliasName);
      else
        fposeHist->setAliasName(UResPoseHist::getMapPoseID());
      addFunction(fposeHist);
    }
  }
  else
    result = false;
  //
  return result;
}

//////////////////////////////////////////////

void UCmdExe::setServer(UServerPort * socketServer)
{
  server = socketServer;
  if (server != NULL)
  {
    server->setResource( this, false);
    updateLocalVar();
    server->varAlivePunkTime = varAlivePunkTime;
  }
}

//////////////////////////////////////////////

void UCmdExe::updateLocalVar()
{
  varOpen4connections->setBool(server->isOpen4Connections());
  varClients->setInt(server->getActiveClientCnt());
  varPort->setInt(server->getPort());
  varTime->setTimeNow();
  varLastClient->setInt(server->getLastClient());
  varLastClientSerial->setInt(server->getLastClientSerial());
}

//////////////////////////////////////////////

void UCmdExe::closeLogServer()
{
  logServer.closeLog();
}

//////////////////////////////////////////////

void UCmdExe::setName(const char * serverName)
{
  servName = serverName;
}

/////////////////////////////////////////////////

bool UCmdExe::handleShellCmd(UServerInMsg * msg)
{
  bool doCmd;
  bool ask4help;
  const int MCL = 1000;
  char cmd[MCL];
  char * p1;
  bool result = true;
  //
  ask4help = msg->tag.getAttValue("help", cmd, MCL);
  doCmd = msg->tag.getAttValue("cmd", cmd, MCL);
  if (ask4help)
  {
    sendHelpStart(msg, "DO BASH shell command options");
    sendText(msg, "DO BASH is a method to issue shell commands on the server platform\n");
    sendText(msg, "the result of the command is returned to the client packed as help\n");
    sendText(msg, "NB! do not issue commands that requires input or do not return\n");
    sendText(msg, "---\n");
    sendText(msg, "cmd='cmd'      execute this command and return result\n"
                  "               XML characters '<>&\"'' may be used\n");
    sendText(msg, "'cmd'          same as above, but a shorter syntax,\n");
    sendText(msg, "               note! XML brackets must here be escaped, like:\n"
                  "               '<'=&lt; '>'=&gt; '''=&apos; '\"'=&quot; '&'=&amp;\n");
    sendText(msg, "help           This help text\n");
    sendText(msg, "---\n");
    sendHelpDone(msg);
  }
  else
  {
    if (not doCmd)
    {
      msg->tag.reset();
      strncpy(cmd, msg->tag.getNext(), MCL);
      p1 = strrchr(cmd, '>');
      if (p1 != NULL)
      { // remove any close tag character
        if (p1[-1] == '/')
        { // is actually a close tag, so remove
          p1--;
          *p1 = '\0';
        }
      }
      doCmd = strlen(cmd) > 0;
    }
    if (doCmd)
    {
      {
        printf("%s\n", cmd);
        xml2str(cmd, MCL, cmd, MCL);
      }
      p1 = cmd;
      while (isspace(*p1))
        p1++;
      result = doBashCmd(msg, p1);
    }
    else
      result = false;
  }
  return result;
}

////////////////////////////////////////////////

bool UCmdExe::handleAlive(UServerInMsg * msg)
{
  bool ask4help;
  const int MRL = 1000;
  char reply[MRL];
  bool result = true;
  //
  ask4help = msg->tag.getAttValue("help", reply, MRL);
  if (ask4help)
  {
    sendHelpStart(msg, "Alive command options");
    sendText(msg, "Alive returns number of seconds since last server thread loop\n");
    sendText(msg, "this should not exceed the maximum command time,\n");
    sendText(msg, "and this should not (normally) exceed 1 second.\n");
    sendText(msg, "help           This help text\n");
    sendText(msg, "---\n");
    sendHelpDone(msg);
  }
  else
  {
    snprintf(reply, MRL, "<%s last=\"%f\"/>\n",
             msg->tag.getTagName(), server->serverAliveLast());
    sendMsg(msg, reply);
  }
  return result;
}

/////////////////////////////////////////////////

bool UCmdExe::handleQuit(UServerInMsg * msg)
{
  bool ask4help;
  const int MRL = 1000;
  char reply[MRL];
  bool result = true;
  //
  ask4help = msg->tag.getAttValue("help", reply, MRL);
  if (ask4help)
  {
    sendHelpStart(msg, "Quit command options");
    sendText(msg, "The quit command shut this server down.\n");
    snprintf(reply, MRL, "This server is %s\n", servName);
    sendText( msg, reply);
    sendText(msg, "help           This help text\n");
    sendText(msg, "---\n");
    sendHelpDone(msg);
  }
  else
  {
    killServer(msg);
  }
  return result;
}

/////////////////////////////////////////////////

bool UCmdExe::doBashCmd(UServerInMsg * msg, const char * cmdStr)
{
  FILE * pf;
  const int MSL = 500;
  char s[MSL];
  int i = 0;
  char * p1;
  //
  pf = popen(cmdStr, "r");
  if (pf != NULL)
  {
    sendHelpStart(msg, cmdStr);
    while (not feof(pf))
    {
      p1 = fgets(s, MSL, pf);
      if (p1 != NULL and strlen(s) > 0)
      {
        //printf("#%d, %s", i, s);
        sendText(msg, s);
      }
      i++;
    }
    sendHelpDone(msg);
    pclose(pf);
  }
  //printf("Finished");
  return (pf != NULL);
}

////////////////////////////////////////////////

UResBase * UCmdExe::getStaticResource(const char * resName, bool mayCreate,
                                      bool staticOnly)
{
  UResBase * res;
  // test if resoource is available already
  res = resPool->getResource(resName);
  if (res == NULL and mayCreate)
  { // it is not available, but may be created - load full module
    loadStaticModule(resName, "", NULL, 0);
    // if it exists, then it is available now
    res = resPool->getResource(resName);
  }
  if (res != NULL and staticOnly)
  { // return static resources only - the rest may be unloaded
    // without control
    if (not isModuleStatic(res->getResFuncIdx()))
      res = NULL;
  }
  return res;
}

//////////////////////////////////////////////

bool UCmdExe::isModuleStatic(int moduleIndex)
{
  bool result = false;
  if (moduleIndex < funcCnt)
  {
    if (func[moduleIndex] != NULL)
      result = func[moduleIndex]->isStatic();
  }
  return result;
}


