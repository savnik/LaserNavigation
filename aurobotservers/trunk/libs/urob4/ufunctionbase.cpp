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
#include <string.h>

#include "ufunctionbase.h"
#include "ucmdexe.h"

#ifdef LIBRARY_OPEN_NEEDED

void libraryOpen(void)
{ // called when server opens this plugin (i.e. calls dlopen())
  // printf("Opening extract line features library\n");
}
//
// ///////////////////////////////////////////////////
//
void libraryClose(void)
{ // called when server unloads this plugin (i.e. calls dlclose())
  // printf("Closing extract line features library\n");
}

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

/*UFunctionBase * createFunc()
{ // create an object of this type
  return new UFuncEfLine();
}*/
//
void deleteFunc(UFunctionBase * p)
{ // delete the object
  delete p;
}

#endif

UFunctionBase::UFunctionBase()
{
  ldModule = NULL;
  ldFileName[0] = '\0';
  aliasName[0] = '\0';
  strncpy(commandsHandled, "none", MAX_RESOURCE_LIST_SIZE);
  cmdHandler = NULL;
  verboseMessages = false;
  msg = NULL;
  setResID("plugin", 915);
  silent = false;
}

///////////////////////////////////////////////////

UFunctionBase::~UFunctionBase()
{
}

///////////////////////////////////////////////////

bool UFunctionBase::addResource(UResBase * res, UFunctionBase * owner)
{
  if (cmdHandler != NULL)
    return cmdHandler->addResource(res, owner);
  else
  {
    printf("addResource:: could not add add resource %s \n"
           "   as module is not proberly initialized - core pointer is missing\n",
           res->getResID());
    return false;
  }
}


///////////////////////////////////////////////////

bool UFunctionBase::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  sendDebug(msg, "No commands handled");
  return true;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendMsg(int clientIdx, const char * message, int size)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendMsg(clientIdx, message, size);
  //
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendMsgInt(int clientIdx, const char * message)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendMsg(clientIdx, message, strlen(message));
  //
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendMsg(UServerInMsg * msg, const char * message, int size)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendMsg(msg, message, size);
  //
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendMsg(UServerInMsg * msg, const char * message)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendMsg(msg, message, strlen(message));
  //
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendMsg(const char * message)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendMsg(msg, message, strlen(message));
  //
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendText(UServerInMsg * msg, const char * text)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendText(msg, text);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendText(const char * text)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendText(msg, text);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendHelpStart(UServerInMsg * msg, const char * text)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendHelpStart(msg, text);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendHelpStart(const char * text)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendHelpStart(msg, text);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendHelpStart()
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendHelpStart(msg, aliasName);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendHelpDone(UServerInMsg * msg)
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendHelpDone(msg);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendHelpDone()
{
  bool result = false;
  //
  if (cmdHandler != NULL)
    result = cmdHandler->sendHelpDone(msg);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendEndTag(UServerInMsg * msg)
{
  const int MRL = 50;
  char reply[MRL];
  //
  snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
  return sendMsg(msg, reply);
}

///////////////////////////////////////////////////

bool UFunctionBase::sendEndTag()
{
  const int MRL = 50;
  char reply[MRL];
  //
  snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
  return sendMsg(msg, reply);
}

///////////////////////////////////////////////////

bool UFunctionBase::sendStartTag(const char * attributes)
{
  const int MRL = 950;
  char reply[MRL];
  //
  snprintf(reply, MRL, "<%s %s>\n", msg->tag.getTagName(), attributes);
  return sendMsg(msg, reply);
}

///////////////////////////////////////////////////

bool UFunctionBase::sendFullTag(const char * attributes)
{
  const int MRL = 950;
  char reply[MRL];
  //
  snprintf(reply, MRL, "<%s %s/>\n", msg->tag.getTagName(), attributes);
  return sendMsg(msg, reply);
}

///////////////////////////////////////////////////

void UFunctionBase::print(const char * preString)
{
//  char * p1, *p2;
//  const int MRL = 200;
//  char rl[MRL];
//  bool isMine;
//  char missing[MRL];
  //
  printf("%s is '%s'\n      Handling:   '%s'\n",
    preString, name(), commandList());
/*  strncpy(rl, resourceList(), MRL);
  if (strlen(rl) > 0)
  {
    printf("      Resources:");
    p2 = rl;
    while (p2 != NULL)
    {
      p1 = strsep(&p2, " ");
      if (*p1 <= ' ')
        break;
      isMine = cmdHandler->isResourceOwner(p1, this);
      if (isMine)
        printf(" %s", p1);
      else
        printf(" (%s)", p1);
    }
    if (gotAllResources(missing, MRL))
      printf(" (got all)\n");
    else
      printf(" (missing %s)\n", missing);
  }*/
  if (ldModule != NULL)
    printf("      Loaded from '%s'\n", ldFileName);
}

///////////////////////////////////////////////////

bool UFunctionBase::sendWarning(UServerInMsg * msg, const char * warningText)
{
  bool result = false;
  if (cmdHandler != NULL and not silent)
    result = cmdHandler->sendWarning(msg, warningText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendWarning(const char * warningText)
{
  bool result = false;
  if (cmdHandler != NULL and not silent)
    result = cmdHandler->sendWarning(msg, warningText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendError(UServerInMsg * msg, const char * errorText)
{
  bool result = false;
  if (cmdHandler != NULL)
    result = cmdHandler->sendError(msg, errorText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendError(const char * errorText)
{
  bool result = false;
  if (cmdHandler != NULL)
    result = cmdHandler->sendError(msg, errorText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendDebug(UServerInMsg * msg, const char * debugText)
{
  bool result = false;
  if (cmdHandler != NULL and not silent)
    result = cmdHandler->sendDebug(msg, debugText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendDebug(const char * debugText)
{
  bool result = false;
  if (cmdHandler != NULL and not silent)
    result = cmdHandler->sendDebug(msg, debugText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendInfo(UServerInMsg * msg, const char * infoText)
{
  bool result = false;
  bool silent = false;
  msg->tag.getAttBool("silent", &silent, true);
  if (not silent and cmdHandler != NULL)
    result = cmdHandler->sendInfo(msg, infoText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendInfo(const char * infoText)
{
  bool result = false;
  if (cmdHandler != NULL and not silent)
    result = cmdHandler->sendInfo(msg, infoText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::sendHelp(UServerInMsg * msg, const char * infoText)
{
  bool result = false;
  if (cmdHandler != NULL)
    result = cmdHandler->sendHelp(msg, infoText);
  return result;
}

bool UFunctionBase::sendHelp(const char * infoText)
{
  bool result = false;
  if (cmdHandler != NULL)
    result = cmdHandler->sendHelp(msg, infoText);
  return result;
}

///////////////////////////////////////////////////

bool UFunctionBase::isClientAlive(int clientIdx, double holdOffTime)
{
  return cmdHandler->isClientAlive(clientIdx, holdOffTime);
}

///////////////////////////////////////////////////

void UFunctionBase::setLoadedModuleRef(void * module, const char * moduleFileName)
{
  ldModule = module;
  strncpy(ldFileName, moduleFileName, MAX_FILENAME_LENGTH);
}

///////////////////////////////////////////////////

bool UFunctionBase::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UCmdExe::getResClassID()))
  { // pointer to server core fnctionality
    if (remove)
      cmdHandler = NULL;
    else if (cmdHandler != (UCmdExe *)resource)
      cmdHandler = (UCmdExe *)resource;
    else
      result = false;
  }
  else
    result = false;
  return result;
}

///////////////////////////////////////////////////

UResBase * UFunctionBase::setThisResource(const char * ID, UResBase * resource, bool remove, bool * changed,
                                       UResBase * currentResPtr, bool * isLocal)
{
  bool local = false;
  bool isUsed = true;
  UResBase * result = currentResPtr;
  //
  if (strcasecmp(resource->getResID(), ID) == 0)
  {
    if (isLocal != NULL)
      local = *isLocal;
    // use the provided resource
    if (local)
      result = currentResPtr; // resource is owned by this plugin, so do not change (or delete)
    else if (remove)
      // remove from function handler list
      result = NULL; // the resource is unloaded, so reference must be removed
    else if (result != resource)
      // use new version
      result = resource;
    else
      isUsed = false; // not used
    if (changed != NULL)
      *changed = isUsed;
  }
  //
  return result;
}

//////////////////////////////////////////////////////////////////

bool UFunctionBase::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result;
  result = (cmdHandler != NULL);
  if ((missingThese != NULL) and not result)
    snprintf(missingThese, missingTheseCnt, " %s", UCmdExe::getResClassID());
  return result;
}

//////////////////////////////////////////////////////////////////

UResBase * UFunctionBase::getStaticResource(const char * resName, bool mayCreate,
                                            bool staticOnly)
{
  UResBase * res = NULL;
  if (cmdHandler != NULL)
    res = cmdHandler->getStaticResource(resName, mayCreate, staticOnly);
  return res;
}

/////////////////////////////////////////////////////////////////////

void UFunctionBase::setCommand(const char * cmdList, const char * name, const char * note)
{ // alias is not used by this module
  char first[MAX_RESOURCE_LIST_SIZE];
  char * p1, *p2, *p3 = NULL, *p4;
  int n, m = 0;
  bool hasAlias;
  // save command list
  hasAlias = strlen(aliasName) > 0;
  // make a copy of command list
  if (not hasAlias)
    strncpy(commandsHandled, cmdList, MAX_RESOURCE_LIST_SIZE);
  // make token list and modify with alias if needed  
  strncpy(first, commandsHandled, MAX_RESOURCE_LIST_SIZE);
  p1 = strtok_r(first, " ", &p4);
  p2 = p1;
  n = strlen(p1);
  if (hasAlias)
  {
    commandsHandled[0] = '\0';
    p3 = commandsHandled;
  }
  cmdToksCnt = 0;
  while (p2 != NULL)
  {
    if (hasAlias)
    {
      if (strncmp(p2, p1, n) == 0)
      {
          snprintf(p3, MAX_RESOURCE_LIST_SIZE - m, "%s%s ", aliasName, &p2[n]);
      }
      else
        strncat(p3, p2, MAX_RESOURCE_LIST_SIZE - m);
      cmdToks[cmdToksCnt++] = p3;
      m += strlen(p3);
      p3 = &commandsHandled[m];
    }
    else
      // just save pointer to start of token in space separated list
      cmdToks[cmdToksCnt++] = &commandsHandled[p2 - p1];
    p2 = strtok_r(NULL, " ", &p4);
  }
  if (hasAlias)
    // remove last space
    *--p3 = '\0';
    // use first command as part of function name - and default alias name
  //strncpy(first, cmdList, MAX_ID_LENGTH);
  p1 = strchr(first, ' ');
  if (p1 != NULL)
    *p1 = '\0';
  if (strlen(getAliasName()) == 0)
    strncpy(aliasName, first, MAX_ID_LENGTH);
  //
  if (note != NULL)
    snprintf(pluginName, MAX_RESOURCE_LIST_SIZE, "%s %s (compiled %s %s)", name, note, __DATE__, __TIME__);
  // for resource list and variable list
  setResID(name, 0);
  // set note description for
  if (note != NULL)
    setDescription(note, false);
}

////////////////////////////////////////////

bool UFunctionBase::newCommand(UServerInMsg * newMsg, void * extra)
{
  bool handled;
  //
  msg = newMsg;
  //handled = handleCmd(&msg->tag, extra);
  //if (not handled)
    handled = handleCommand(msg, extra);
  msg = NULL;
  return handled;
}

////////////////////////////////////////////

bool UFunctionBase::newCmd(UServerInMsg * newMsg, UDataBase * pushData)
{
  bool handled = false;
  //
  msg = newMsg;
  printf("UFunctionBase::newCmd is not implemented, should replay the newCommand(msg, void*) call!\n");
  //handled = handleCmd(&msg->tag, pushData);
  if (not handled and pushData == NULL)
    // can be handled by the old method
    handled = newCmd(msg, NULL);
  msg = NULL;
  return handled;
}

////////////////////////////////////////////

void UFunctionBase::setAliasName(const char* name)
{ // may be overloaded for special use
  if (strlen(name) > 0)
  { // use alias name as first and only command name and also as resource name
    bool resGotSameName = (strcmp(getResID(), aliasName) == 0);
    strncpy(aliasName, name, MAX_ID_LENGTH);
    // replace command and resource ID with alias name
    if (resGotSameName)
      setCommand(aliasName, aliasName, NULL);
    else
      // let resource keep its name - probably no variables of its own
      setCommand(aliasName, getResID(), NULL);
  }
}

//////////////////////////////////////

bool UFunctionBase::isMine(const char * cmdName)
{
  int n = strlen(cmdName);
  bool result = false;
  for (int i = 0; i < cmdToksCnt; i++)
  {
    if (strncasecmp(cmdToks[i], cmdName, n) == 0 and
         cmdToks[i][n] <= ' ')
    {
      cmdIndex = i;
      result = true;
      break;
    }
  }
  return result;
}

