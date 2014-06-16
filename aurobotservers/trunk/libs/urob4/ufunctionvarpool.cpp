/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <ctype.h>

#include <ugen4/uline.h>
#include <umap4/uposev.h>

#include "uvarcalc.h"
#include "ufunctionvarpool.h"
#include "ucmdexe.h"

UFunctionVarPool::UFunctionVarPool()
{
  setCommand("var varPush", "varpool", "handles variable communication");
  varPool = NULL;
//  varPoolLocal = false;
  leftMargin = 30;
}


UFunctionVarPool::~UFunctionVarPool()
{
  if (varPool != NULL)
  {
    delete varPool;
  }
}

///////////////////////////////////////////////////

void UFunctionVarPool::createResources()
{ // load resource for this plugin
  varPool = new UResVarPool();
  // tell server about the resource, and that this object is the owner
  addResource(varPool, this);
}

////////////////////////////////////////////////////

bool UFunctionVarPool::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  UVarPool * vp;
  UResVarPool * vpr;
  //
  if ((varPool != NULL) and resource->isAlsoA(UResVarPool::getResClassID()) and not resource->isA(getResClassID()))
  { // this ressource may hold variables that can be accessed by this module
    vp = varPool->getVarPool();
    if (vp != NULL)
    {
      if (remove)
      {
        vp->deleteStruct(resource->getResID());
        // it is no longer possible to remove individual methods based on owner
        //vp->deleteMethods(resource);
      }
      else
      { // resource is also a var-pool resource, so access as so.
        vpr = (UResVarPool *) resource;
        result = vp->addStruct(resource->getResID(), vpr->getVarPool());
      }
    }
  }
  // may be needed at lower levels
  result |= UFunctionBase::setResource(resource, remove);
  //
  return result;
}

////////////////////////////////////////////////////

bool UFunctionVarPool::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA("var"))
    result = handleVar(msg, extra);
  else if (msg->tag.isTagA("varPush"))
    result = handleVarPush(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionVarPool::handleVar(UServerInMsg * msg, void * extra)
{ // extract parameters
  bool result = false;
  // decode vars
  const int MVNL = MAX_SML_NAME_LENGTH * 5;
  char attName[MVNL];
  const int VAL_BUFF_LNG = 500;
  char attValue[VAL_BUFF_LNG];
  const int MRL = 600;
  char reply[MRL];
  // parameters
  bool getList = false;
  // bool getFuncs = false;
  bool ask4help = false;
  UTime t;
  bool srGotReply = false;
  bool asSingle = false;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char allName[MVL] = "";
  char structName[MVL] = "";
  UVarPool * vp;
  char * p1;
  const int MTL = 15;
  char type[MTL]; // default type is double (else 'dq', 'pose', '3d' ...)
  bool aCall = false;
  const int MCL = 100;
  char aCallStr[MCL];
  const int MCSL = 32;
  char aCallStruct[MCSL] = "";
  bool aLogOpen = false;
  bool aLogOpenValue = false;
  bool aLogging = false;
  bool mayBeVar = false;
  bool andDesc = false;
  //bool aSilent = false;
  vp = varPool->getVarPool();
  UVariable * theVar = NULL;
  UVarPool * theStruct = NULL;
  const int MVC = 15;
  char * varNames[MVC];
  char * varValues[MVC];
  int varNamesCnt = 0;

  // test if special var type is requested
  if (not msg->tag.getAttValue("type", type, MTL))
    strcpy(type, "none");
  // extract parameters
  msg->tag.reset();
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG, MVNL))
  { // camera device
    if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "all") == 0)
      getList = true;
    else if (strcasecmp(attName, "list") == 0)
      getList = true;
    else if (strcasecmp(attName, "copy") == 0)
      asSingle = true;
    else if ((p1 = strcasestr(attName, ".all")) != NULL)
    {
      if (strcasecmp(p1, ".allCopy") == 0)
        asSingle = true;
      if (strcasecmp(p1, ".all") == 0 or asSingle)
      {
        getList = true;
        strncpy(allName, attName, MVL);
        // prepare structure identifier
        strncpy(structName, attName, MVL);
        structName[p1 - attName + 1] = '\0'; // terminate before 'all'
      }
      else
        mayBeVar = true;
    }
    else if (strcasecmp(attName, "allCopy") == 0)
    {
      getList = true;
      asSingle = true;
    }
    else if (strcasecmp(attName, "funcs") == 0)
      ; //getFuncs = true;
    else if (strcasecmp(attName, "desc") == 0)
    {
      andDesc = true;
    }
    else if (strcasecmp(attName, "call") == 0)
    {
      aCall = true;
      strncpy(aCallStr, attValue, MCL);
    }
    else if (strcasecmp(attName, "returnType") == 0)
    {
      strncpy(aCallStruct, attValue, MCSL);
    }
    else if (strcasecmp(attName, "type") == 0)
      ;//already handled;
    else if (strcasecmp(attName, "log") == 0)
    {
      aLogOpen = true;
      aLogOpenValue = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "isLogging") == 0)
      aLogging = true;
    else if (strcasecmp(attName, "desc") == 0)
      andDesc = true;
    else if (strcasecmp(attName, "silent") == 0)
      ; //aSilent = true;
    else if (strcasecmp(attName, "setHist") == 0 or
             strcasecmp(attName, "maxUpdFrq") == 0 or
             strcasecmp(attName, "replay") == 0 or
             strcasecmp(attName, "step") == 0 or
             strcasecmp(attName, "size") == 0 or
             strcasecmp(attName, "dest") == 0 or
             strcasecmp(attName, "getHist") == 0)
      ; // unused here
    else
      mayBeVar = true;
    if (mayBeVar)
    { // may be a calculator variable
      if (varNamesCnt < MVC)
      {
        varNames[varNamesCnt] = (char*)malloc(strlen(attName) + 1);
        strcpy(varNames[varNamesCnt], attName);
        varValues[varNamesCnt] = (char*)malloc(strlen(attValue) + 1);
        strcpy(varValues[varNamesCnt], attValue);
        varNamesCnt++;
      }
      mayBeVar = false;
    }
  } // while
  //
  if (ask4help)
  {
    sendHelpStart("VAR");
    sendText("-------------------- Available VAR options:\n");
    sendText("help                    This help message\n");
    sendText("list                    Get list of variables in the root of variable structure\n");
    sendText("struct                  Get list of all variables in a structure\n");
    sendText("struct copy             Get an xml coded copy of variable(s)\n");
    sendText("dest=struct.|var        This destination struct (name ending with a period) or "
                                      "variable name is included in reply, "
                                      "and if received by another server, then the requested variable or struct is placed"
                                      "in a structure or variable with this name. For structs with COPY option only\n");
//    sendText("type=T                  semi-structure where T='pose' | '3d' | 'dq' | 's' | 't'\n");
    sendText("call='me(par1,par2,..)' Test a method call with name 'me' and some parameters\n");
    sendText("returnType=[lineseg|3d|pose|...] Expected struct result type of call\n");
    snprintf(reply, MRL,
             "log[=false]             Open (or close) var logfile (open %s) %s\n",
                       bool2str(varPool->isLogFileOpen()), varPool->getLogFileName());
    sendText(reply);
    sendText("log[=false] struct      Start or stop logging of this struct\n");
    sendText("isLogging   struct      Is this structure beeing logged?\n");
    sendText("silent                  Suppress most (info) messages\n");
    sendText("---- single variables\n");
    sendText("variable [variable]*    Get value of variable(s)\n");
    sendText("variable=\"value\"        Assign a value to a variable (simple calculations may be used)\n");
    sendText("                        Special values 'true', 'false', 'ones', 'zeros', 'unit' may be used\n");
    sendText("size='R,C'  varaible    Set array or matrix size of an already defined variable\n");
    sendText("desc                    Get variable description(s) too\n");
    sendText("desc='text' varaible    Set on-line available description of a variable\n");
    sendText("log[=false] varaible    Start or stop logging of this single variable\n");
    sendText("isLogging   variable    Is this variable beeing logged?\n");
    sendText("sethist=N   variable    Allocate memory for N elements (N=0 is off)\n");
    sendText("maxUpdFrq=H variable    Set maksimum update frequency [Hz] (min=1 uHz)\n");
    sendText("gethist=N   variable    get N historic elements for this variable (N=0 means all)\n");
    sendText("replay[=false] variable Start (or stop) replay from replayPath/full-var-name.log\n");
    sendText("step        variable    Advance to next line in replay file\n");
    sendText("----\n");
    sendText("See also VARPUSH (update-event handling for structs)\n");
    sendHelpDone();
    srGotReply = true;
  }
  else if (vp != NULL)
  { // global log handling
    if (varNamesCnt == 0)
    {
      if (aLogOpen)
      {
        if (aLogOpenValue)
        {
          varPool->logFileOpen();
          snprintf(reply, MRL, "Opened logfile (%s) as %s",
                  bool2str(varPool->isLogFileOpen()), varPool->getLogFileName());
          sendInfo(msg, reply);
          srGotReply = true;
        }
        else
          varPool->logFileClose();
      }
      if (getList)
      { // send list of root variables
        srGotReply = sendVarList(msg, vp, andDesc, asSingle);
      }
    }
    // handling of individual vars
    for (int j = 0; j < varNamesCnt; j++)
    { // there may be more than one variable in a command - take one at a time
      // find variable or struct
      attName[0] = '\0';
      theVar = vp->getGlobalVariable(varNames[j], NULL);
      if (theVar != NULL)
        theStruct = NULL;
      else
        theStruct = vp->getStructDeep(varNames[j], false, attName, MVNL);
      //
      if (theStruct == NULL or (attName[0] != '\0'))
      { // Not a structure - a known variable or an unknown variable (new)
        // most likely an unhandled var name (and value)
        srGotReply = handleVarValues(msg, (UVarCalc*)vp, varNames[j], varValues[j], type, andDesc);
        if (not srGotReply)
        { // failed to find variable of this name
          snprintf(reply, MRL, "%s=%s is not valid", varNames[j], varValues[j]);
          srGotReply = sendWarning(msg, reply);
        }
      }
      if (aLogOpen)
      {
        if (theStruct != NULL)
        {
          varPool->logFileStart(theStruct->getFullPreName(), aLogOpenValue);
          snprintf(reply, MRL, "Struct %s is logged (%s) to %s, file open (%s)", allName,
                  bool2str(theStruct->isLogfileOpen()), varPool->getLogFileName(),
                  bool2str(varPool->isLogFileOpen()));
          sendInfo(msg, reply);
          srGotReply = true;
        }
        else if (theVar != NULL)
        { // is logfile for a private variable
          bool isOpen;
          isOpen = theVar->openVarLog(aLogOpenValue);
          snprintf(reply, MRL, "variable %s is logged (%s) to %s", allName,
                  bool2str(isOpen), theVar->getLogFilename());
          sendInfo(msg, reply);
          srGotReply = true;
        }
      }
      else if (aLogging)
      {
        if (theStruct != NULL)
          snprintf(reply, MRL, "Struct %s is logged %s", allName,
                  bool2str(theStruct->isLogfileOpen()));
        else if (theVar != NULL)
        {
          if (theVar->hasHist())
            snprintf(reply, MRL, "Variable %s is logged %s", allName,
                  bool2str(theVar->hist->isLogOpen()));
          else
            snprintf(reply, MRL, "Variable %s is not logged", allName);
        }
        sendInfo(msg, reply);
        srGotReply = true;
      }
      else if (theStruct != NULL and not srGotReply)
        getList = true;
      //
      if (getList and theStruct != NULL)
      { // send list of variables in the structure
        sendVarList(msg, theStruct, andDesc, asSingle);
        srGotReply = true;
      }
    }
    if (aCall)
    { // one call per line only
      makeAMethodCall(msg, aCallStr, aCallStruct);
      srGotReply = true;
    }
  }
  else
  {
    result = sendError(msg, "No varPool object");
    srGotReply = true;
  }
  // last - if none else has send a reply ...
  if (not srGotReply)
  { // rply with tagName srGet is not send
    result = sendInfo(msg, "done");
  }
  for (int j = 0; j< varNamesCnt; j++)
  { // release allocated memory
    free(varNames[j]);
    free(varValues[j]);
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

void UFunctionVarPool::sendAllVar(UServerInMsg * msg, UVarCalc * varPool, const char * structPreName, bool andDesc)
{
  int n, m;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char structName[MVL];
  char * p1;
  //
  for (n = 0; n < varPool->getVarsCnt(); n++)
  {
    UVariable * vv;
    vv = varPool->getLocalVar(n);
    varPool->lock();
    sendVar(msg, vv, structPreName, andDesc, NULL);
    varPool->unlock();
  }
  m = strlen(structPreName);
  if (m > 0)
  {
    strncpy(structName, structPreName, MVL);
    p1 = &structName[m];
  }
  else
  {
    structName[0] = '\0';
    p1 = structName;
  }
  for (n = 0; n < varPool->getStructCnt(); n++)
  {
    UVarPool * vp;
    vp = varPool->getStruct(n);
    if (vp->getVarsCnt() > 0 or vp->getStructCnt() > 0 or vp->getMethodCnt() > 0)
    { // do not send empty structs
      strncpy(p1, vp->getPreName(), MVL - m - 1);
      strcat(p1, ".");
      sendAllVar(msg, (UVarCalc*)vp, structName, andDesc);
    }
  }
}

///////////////////////////////////////////////////////////

bool UFunctionVarPool::sendVar(UServerInMsg * msg,
                               UVariable * vv,
                               const char * structPreName, bool andDesc, const char * extra)
{
  const int MRL = 3600;
  char reply[MRL];
  const int MSL = 3000;
  char s[MSL];
  UVariable::varType type;
  const int MTL = 5;
  char vt[MTL];
  UTime t;
  char * p1;
  int n;
  bool result = false;
  //
  bool getHist = false;
  int getHistN = 10;
  bool helpList = false;
  bool silent = false;
  const int MWL = MAX_STRUCT_NAMES * MAX_VAR_NAME_SIZE;
  char dest[MWL] = "";
  //
  // is there a request for a help-packed list - else XML-tag copy
  msg->getTag()->getAttBool("list", &helpList, true);
  msg->getTag()->getAttBool("silent", &silent, true);
  msg->getTag()->getAttValue("dest", dest, MWL);
  //
  if (vv != NULL)
  { // is there a legal request for history - else just normal
    if (vv->hasHist())
      getHist = msg->getTag()->getAttInteger("getHist", &getHistN, 0);
    // get from variable type d, 3d, pose, ...
    type = vv->getType();
    //
    int m;
    if ((getHist or helpList) and vv->hasHist() and vv->hist->rowMaxCnt > 1)
      m = mini(vv->hist->rowCnt, getHistN);
    else
      m = 0;
    if (helpList)
    {
      sendHelpStart("variable");
      snprintf(reply, MRL, "%s%s typ=%s size=%d (%dx%d)\n",
               structPreName, vv->name, vv->getTypeChar(vt, MTL),
               vv->getSize(), vv->rows(), vv->cols());
      sendText(reply);
      if (vv->hasHist())
      {
        snprintf(reply, MRL, "   hist=%s size=%d/%d, max update Frq=%gHz;\n",
                 bool2str(vv->hist->rowMaxCnt>1), vv->hist->rowCnt,
                 vv->hist->rowMaxCnt, 1.0/vv->hist->minUpdTime);
        sendText(reply);
        snprintf(reply, MRL, "   replay=%s, replayline=%d from %s\n",
                 bool2str(vv->hist->isReplayFileOpen()), vv->hist->getReplayLogLine(),
                 vv->hist->getReplayFileName(s, MSL));
        sendText(reply);
        snprintf(reply, MRL, "   log=%s, max update Frq=%gHz, to %s\n",
                 bool2str(vv->hist->isLogOpen()), 1.0/vv->hist->minUpdTime,
                 vv->hist->ULogFile::getLogFileName());
        sendText(reply);
      }
    }
    for (int histIdx = m; histIdx >= 0; histIdx--)
    { // send oldest history element first (can then be captured by another variable with history)
      // make common first part
      if (helpList)
      {
        p1 = reply;
        n = 0;
        if (m > 0)
        { // timed series - use update time as tod
          UTime ut = vv->hist->getUpdTime(histIdx);
          snprintf(p1, MRL - n, "%lu.%06lu: ",
                ut.GetSec(), ut.getMicrosec());
          n += strlen(p1);
          p1 = &reply[n];
        }
        snprintf(p1, MRL - n, "%s%s=\"%s\"",
          structPreName, vv->name,
          vv->getValuesAsString(s, MSL, histIdx));
      }
      else
        snprintf(reply, MRL, "<%s name=\"%s%s\" typ=\"%s\" size=\"%d\" "
          "value=%s",
          msg->tag.getTagName(), structPreName, vv->name,
          vv->getTypeChar(vt, MTL), vv->getSize(),
          vv->getValuesAsStringQuoted(s, MSL, histIdx));
      n = strlen(reply);
      p1 = &reply[n];
      //
      if (m == 0 and not getHist)
      {
        switch (type) // if (strcasecmp(type, "d") == 0)
        { // additional attributes
          case UVariable::t:
            t.setTime(vv->getValued(0));
            snprintf(p1, MRL - n, " tod=\"%lu.%06lu\" time=\"%s\"", t.getSec(), t.getMicrosec(), t.getForFilename(s));
            break;
          case UVariable::dq:
            snprintf(p1, MRL - n, " qual=\"%.12g\"", vv->getValued(1));
            break;
          case UVariable::pose:
            snprintf(p1, MRL - n, " x=\"%.12g\" y=\"%.12g\" th=\"%.12g\"",
                  vv->getValued(0), vv->getValued(1), vv->getValued(2));
            break;
          case UVariable::d3d:
            snprintf(p1, MRL - n, " x=\"%.12g\" y=\"%.12g\" z=\"%.12g\"",
                  vv->getValued(0), vv->getValued(1), vv->getValued(2));
            break;
          case UVariable::rot:
            snprintf(p1, MRL - n, " Omega=\"%.12g\" Phi=\"%.12g\" Kappa=\"%.12g\"",
                  vv->getValued(0), vv->getValued(1), vv->getValued(2));
            break;
          case UVariable::d2d:
            snprintf(p1, MRL - n, " x=\"%.12g\" y=\"%.12g\"",
                vv->getValued(0), vv->getValued(1));
            break;
          default:
            // most should be handled with no extra data
            break;
        }
      }
      n += strlen(p1);
      p1 = &reply[n];
      if (getHist and not helpList)
      { // timed series - use update time as tod
        UTime ut = vv->hist->getUpdTime(histIdx);
        snprintf(p1, MRL - n, " tod=\"%lu.%06lu\"",
              ut.GetSec(), ut.getMicrosec());
        n += strlen(p1);
        p1 = &reply[n];
      }
      if (extra != NULL)
      {
        snprintf(p1, MRL - n, " %s", extra);
        n += strlen(p1);
        p1 = &reply[n];
      }
      if (strlen(dest) > 0)
      {
        snprintf(p1, MRL - n, " dest=\"%s\"", dest);
        n += strlen(p1);
        p1 = &reply[n];
      }
      if (andDesc)
      {
        if (vv->description != NULL)
          str2xml(s, MSL, vv->description);
        else
          s[0] = '\0';
        snprintf(p1, MRL - n, " desc=\"%s\"", s);
        n += strlen(p1);
        p1 = &reply[n];
      }
      if (helpList)
      {
        strncat(p1, "\n", MRL);
        sendText(reply);
      }
      else if (not silent)
      {
        strncat(p1, "/>\n", MRL);
        result = sendMsg(msg, reply);
      }
      else
        result = true;
    }
    if (helpList)
      result = sendHelpDone();
  }
  else
  { // variable not found in this var pool
    snprintf(reply, MRL, "not found in struct '%s' - try: var help", structPreName);
    result = sendWarning(reply);
  }
  return result;
}

////////////////////////////////////////////////////////////////////

bool UFunctionVarPool::handleVarValues(UServerInMsg * msg, UVarCalc * varP,
                                       const char * attName, const char * attValue,
                                       const char * defType,
                                      bool andDesc)
{
  int varIdx, elem;
  bool result = false;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char varName[MVL];
  const int MRL = 600;
  char s2[MRL], s3[MRL], s4[MRL];
  UVarPool * vp;
  UVariable * var = NULL;
  bool setHist = false;
  int setHistN = -1;
  bool setUpdFrq = false;
  double setUpdFrqVal = 200.0;
  bool setReplay;
  bool setReplayValue = false;
  bool setStep;
  int setStepN = 1;
  bool mayAdd = true;
  bool isDone = false;
  bool silent = false;
  bool setDesc;
  const int MDL = 1000;
  char setDescTo[MDL] = "";
  bool setCopy = false;
  bool setSize;
  const int MZL = 20;
  char setSizeVal[MZL];
  //
  setHist = msg->getTag()->getAttInteger("setHist", &setHistN, 100);
  setUpdFrq = msg->getTag()->getAttDouble("maxUpdFrq", &setUpdFrqVal, 200.0);
  setReplay = msg->getTag()->getAttBool("replay", &setReplayValue, true);
  setStep = msg->getTag()->getAttInteger("step", &setStepN, 1);
  setDesc = msg->getTag()->getAttValue("desc", setDescTo, MDL);
  setDesc &= strlen(setDescTo) > 0;
  msg->getTag()->getAttBool("silent", &silent, true);
  msg->getTag()->getAttBool("copy", &setCopy, true);
  setSize = msg->getTag()->getAttValue("size", setSizeVal, MZL);
  // make local copy of var name
  strncpy(varName, attName, MVL);
  // separate struct and name into varPool (return value) and localName (in 'varName')
  vp = varP->getStructDeep(attName, true, varName, MVL);
  //
  if ((vp != NULL) and (strlen(varName) == 0))
  { // structure name with no variable
    result = true;
    //printf("UFunctionVarPool::handleVarValues missing variable name in struct\n");
  }
  //
  if ((vp != NULL) and (strlen(varName) > 0))
  { // get variable
    varIdx = vp->getLocalVarIndex(varName, &elem);
    if (elem >= 0)
      mayAdd = false;
    else
      elem = 0;
    if (varIdx >= 0)
      // assignment to existing variable
      var = vp->getLocalVar(varIdx);
    // handle variable action
    if (var != NULL)
    { // variable is available
      isDone = true;
      if (setHist or setUpdFrq)
      {
        if (var->hasHist() and not setUpdFrq)
          setUpdFrqVal = 1.0/var->hist->minUpdTime;
        if (setHist)
          var->makeTimeSeries(setHistN, setUpdFrqVal);
        else if (var->hasHist())
          var->setMaxUpdateRate(setUpdFrqVal);
        if (not silent)
        {
          snprintf(s2, MRL, "<%s info=\"variable now has space for %d updates max update rate=%g Hz\" name=\"%s\"/>\n",
                  msg->tag.getTagName(), var->hist->rowMaxCnt, 1.0/var->hist->minUpdTime, attName);
          result = sendMsg(s2);
        }
      }
      else if (setReplay)
      {
        bool isOK = var->setReplay(setReplayValue, vp->getFullPreName(), var->name, this);
        if (not silent)
        {
          if (var->hasHist())
            snprintf(s2, MRL, "replay %s from file %s", bool2str(isOK), var->hist->getReplayFileName(s3, MRL));
          else
            snprintf(s2, MRL, "failed to add replay structure to %s", var->name);
          result = sendInfo(s2);
        }
      }
      else if (setStep)
      {
        bool isOK = var->hasHist();
        int n = -1;
        if (isOK)
          n = var->hist->replayStep(setStepN);
        if (not silent)
        {
          isOK = (n >= 0);
          snprintf(s2, MRL, "replay of %d step(s) result=%s - to line %d", setStepN, bool2str(isOK), n);
          result = sendInfo(s2);
        }
      }
      else
        isDone = false;
      // other actions on variable
      if (setDesc)
        // set description
        var->setDescription(setDescTo, true);
      if (setSize and strlen(setSizeVal) > 0)
      { // set variable size
        char * p1 = setSizeVal;
        int r, c = 0;
        r = strtol(p1, &p1, 10);
        while (*p1 != '\0' and not isdigit(*p1))
          p1++;
        if (isdigit(*p1))
        {
          c = strtol(p1, &p1, 10);
          if (c > 1 or var->isTypeA(UVariable::m2))
            var->setSize(r, c);
        }
        if (r >= 1 and c <= 1)
          var->setSize(r);
      }
    }
    //
    // assignment and information reply is
    // not handled yet
    if (not isDone)
    { // not handled yet - get value or set value
      if (strlen(attValue) == 0)
      { // request for existing value or semi structure
        result = sendVar(msg, var, vp->getFullPreName(s2, MRL), andDesc, NULL);
        isDone = true;
      }
      else
      { // value may be another variable name or an expression
        UVariable varSrc;
        bool foundValue;
        const char * p2;
        foundValue = varP->evaluateV(attValue, attValue, &p2, &varSrc, false);
        foundValue &= strlen(p2) == 0;
        if (foundValue)
        { // the value could be evaluated
          if (var != NULL)
            var->setValue(&varSrc, elem);
          else
          { // add a new variable
            var = vp->addVar(varName, &varSrc, NULL);
            if (var != NULL and setDesc)
               var->setDescription(setDescTo, true);
          }
          result = sendVar(msg, var, vp->getFullPreName(s2, MRL), andDesc, NULL);
          isDone = true;
        }
      }
    }
    // assign constant to or add a new value
    if (not isDone)
    { // assign a value to a variable or add a new variable
      if (varIdx >= 0)
      { // assignment to existing variable
        snprintf(s4, MRL, "oldValue=\"%s\"", var->getValuesAsString(s3, MRL, 0));
        if (var->isString())
        {
          var->setValues(attValue, elem, true);
        }
        else
        {
          result = true;
          if (strcasecmp(attValue, "zeros") == 0)
            var->setAll(0.0);
          else if (strcasecmp(attValue, "ones") == 0)
            var->setAll(1.0);
          else if (strcasecmp(attValue, "unit") == 0 and var->isTypeA(UVariable::m2))
            var->setUnit();
          else
            var->setValued(attValue, elem, mayAdd);
        }
      }
      else
      { // add a new variable
        vp->addVarA(varName, attValue, defType, "new");
        var = vp->getLocalVariable(varName, NULL);
        snprintf(s4, MRL, "oldValue=\"(null)\"");
      }
      result = sendVar(msg, var, vp->getFullPreName(s2, MRL), andDesc, s4);
    }
  }
  return result;
}


/////////////////////////////////////////////////

bool UFunctionVarPool::handleVarPush(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 500;
  char val[VBL];
  const int MRL = 2000;
  char reply[MRL];
  const int MVNL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char structure[MVNL] = "";
  bool ask4help;
  bool ask4List = false;
  bool isAflush;
  bool gotACmd = false;
  UVarPool * vp;
  int n, cm, ca;
  bool gotStruct = false;
  bool replySend = false;
  UServerPushImplement * spi;
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4List = msg->tag.getAttValue("list", val, VBL);
  isAflush = msg->tag.getAttValue("flush", val, VBL);
  if (msg->tag.getAttValue("struct", val, VBL))
  {// get (non default) device
    strncpy(structure, val, MVNL - 1);
    gotStruct = true;
  }
  if (msg->tag.getAttValue("cmd", val, VBL))
   // is a push command
    gotACmd = strlen(val) > 0;
  if (msg->tag.getAttValue("call", val, VBL))
   // is a push call
    gotACmd = strlen(val) > 0;
  // ignore all other attributes - handled in ush structure
  if (ask4help)
  {
    sendHelpStart("VARPUSH");
    sendText("---- Available VARPUSH settings:\n");
    sendText("Executes push cmd's on update of a variable in a structure.\n");
    sendText("struct=m        Push when structure 'm' is updated (use '.' for root)\n");
    sendText("good=k or g=k   Stop push after k good commands (def: no stop).\n");
    sendText("total=k or n=k  Stop push after k commands (def: no stop.\n");
    sendText("interval=k or i=k Push with this interval (def = 1).\n");
    sendText("flush[=cmd]     Remove push command(s) from client (default all)\n");
    sendText("cmd=\"command\" Do execute 'command' at every push event\n");
    sendText("call=\"method(par)\" Do call 'method' with these parameters at every push event.\n");
    sendText("list            Show active push commands\n");
    sendText("e.g.:   <varPush struct=road cmd='var road.allCopy'/>\n");
    sendText("See also PUSH (timed push), var (variable manipulation)\n");
    sendText("---\n");
    sendHelpDone();
    result = true;
  }
  else if ((varPool != NULL) and gotStruct)
  { // push command
    if ((strlen(structure) > 1) and (structure[0] != '.'))
      // add a period, so that only structures are matched
      strcat(structure, ".");
    // get structure var pool
    vp = varPool->getVarPool()->getStructDeep(structure, NULL, 0);
    if (vp != NULL)
    {
      if (gotACmd or isAflush)
      {
        if (not vp->gotCmdExe())
        {
          spi = cmdHandler;
          vp->setCmdExe(spi);
        }
        n = vp->addPushCommand(msg);
        vp->getPushCmdCnt(&cm, &ca);
        if (isAflush)
        { // flush and senr reply
          snprintf(reply, MRL, "<%s done=\"%s\" flushed=\"%d\" pushcmds=\"%d\" pushcalls=\"%d\"/>\n",
                  msg->tag.getTagName(), bool2str(n > 0), n, cm, ca);
          result = (n > 0);
        }
        else
        { // a new push command
          snprintf(reply, MRL, "<%s done=\"%s\" pushcmds=\"%d\" pushcalls=\"%d\"/>\n",
                  msg->tag.getTagName(), bool2str(n > 0), cm, ca);
          result = (n == 1);
        }
        sendMsg(msg, reply);
        replySend = true;
      }
      if (ask4List)
      { // send list of current push commands as help text
        snprintf(reply, MRL, "<help subject=\"VARPUSH struct='%s'\">\n", structure);
        sendMsg(msg, reply);
        vp->print(structure, reply, MRL);
        sendText(msg, reply);
        sendMsg(msg, "</help>\n");
        sendInfo(msg, "done");
        result = true;
        replySend = true;
      }
      if (not replySend)
        sendWarning(msg, "No (valid) action specified");
    }
    else
      sendWarning(msg, "No such structure");
  }
  else if (not gotStruct)
    sendWarning(msg, "You must speciy a structure -- see: varPush help");
  else
    sendWarning(msg, "No var pool resource? (a so called software error)");
  //
  return result;
}

///////////////////////////////////////////////

void UFunctionVarPool::sendDescription(UServerInMsg * msg,
                                       const char * prestring,
                                       const char * desc)
{
  const int MLL = 200;
  char line[MLL];
  int n, m, k;
  char * p1 = NULL;
  const char * p2;
  bool last;
  int width = 90;      // width of console in characters
  const char * descVal = desc;
  //
  if (int(strlen(prestring)) > leftMargin)
    leftMargin = mini(width - 35, strlen(prestring));
  //
  m = maxi(30, width - leftMargin);
  // copy text in left margin
  strncpy(line, prestring, MLL);
  line[MLL - 1] = '\0';
  n = strlen(line);
  if (n < leftMargin)
    memset(&line[n], ' ', leftMargin - n);
  line[leftMargin] = '\0';
  // debug
  // printf("UFunctionVarPool::sendDescription: line length=%d)\n", n);
  // debug end
  //
  if (descVal == NULL)
    descVal = "-";
  snprintf(&line[leftMargin], m, "%s\n", descVal);
  last = (int(strlen(descVal)) < m-1);
  if (not last)
  { // find first new-line or last space
    p1 = strchr(line, '\n');
    if (p1 == NULL)
      p1 = strrchr(line, ' ');
    if (p1 != NULL)
    { // replace with new-line and terminate
      *p1++ = '\n';
      *p1 = '\0';
    }
  }
  sendMsg(msg, line);
  // debug
  // printf("UFunctionVarPool::sendDescription: first line send length=%d)\n", strlen(line));
  // debug end
  if (not last)
  {
    k = 0;
    //l = strlen(descVal);
    memset(line, ' ', leftMargin);
    last = false;
    while (not last)
    {
      k += p1 - line - leftMargin;
      p2 = &descVal[k];
      snprintf(&line[leftMargin], m, "%s\n", p2);
      last = (int(strlen(p2)) < m-1);
      if (not last)
      {
        p1 = strchr(line, '\n');
        if (p1 == NULL)
          p1 = strrchr(line, ' ');
        if (p1 != NULL)
        { // replace with new-line and termination
          *p1++ = '\n';
          *p1 = '\0';
        }
      }
      sendMsg(msg, line);
    }
  }
}

/////////////////////////////////////////////////////

void UFunctionVarPool::makeAMethodCall(UServerInMsg * msg, char * aCallStr, char * returnStructType)
{
  char *p1, *p2;
  // double prameters
  const int MDP = 10;
  double dPar[MDP];
  int dParCnt = 0;
  // string parameters
  const int MSP = 5;
  char * sPar[MSP];
  int sParCnt = 0;
  // method to call
  char * method;
  // parameter order
  const int MOL = 15;
  char oPar[MOL];    // parameter order
  int oParCnt = 0;
  // return struct
  const int MRP = 12;
  UDataBase * rPar[MRP];
  UDataBase * rParOrg[MRP];
  int rParCnt;
  bool returnedStructs;
  const int MPL = 15;
  char ps[MPL];
  //
  char sep[2] = "a";
  int i;
  double value;
  bool exist, structKnown;
  const int MRL = 500;
  char reply[MRL];
  //
  // just to make it look nice during debug
  sPar[0] = (char *)"none";
  dPar[0] = 0;
  //
  p2 = aCallStr;
  method = strsep(&p2, "(");
  // skip white space
  if (p2 != NULL)
  {
    while (isspace(*p2))
      p2++;
    // get potential parameters
    while (*p2 != ')')
    {
      if ((*p2 == '\'') or (*p2 == '"'))
      { // string parameter
        oPar[oParCnt++] = 's';
        sep[0] = *p2++;
        // find end of string parameter
        p1 = strsep(&p2, sep);
        // save as next string parameter
        sPar[sParCnt++] = p1;
      }
      else // assume double parameter
      {
        dPar[dParCnt++] = strtod(p2, &p1);
        p2 = p1;
        oPar[oParCnt++] = 'd';
      }
      while (isspace(*p2))
        p2++;
      if (*p2 == ',')
        p2++;
      else
        // must be a comma or an end
        break;
      while (isspace(*p2))
        p2++;
      if ((dParCnt >= MDP) or (sParCnt >= MSP) or (oParCnt >= MOL))
        break;
    }
    if (*p2 == ')')
    { // got it all
      rParCnt = MRP;
      // initialize pointer array for structs to NULL
      structKnown = initCallReturnStructType(rParOrg, MRP, returnStructType);
      for (i = 0; i < MRP; i++)
        // copy pointers to used array, as these may be modified by the called method
        rPar[i] = rParOrg[i];
      // set return value (to false)
      value = 0.0;
      // terminate parameter order list
      oPar[oParCnt] = '\0';
      // make the function call
      exist = varPool->callGlobal(method, oPar, sPar, dPar, &value, rPar, &rParCnt);
      if (not exist)
      { // may be a callV type
        UVariable * vpars[20];
        int sn = 0, dn = 0;
        for (int i = 0; i < oParCnt; i++)
        {
          vpars[i] = new UVariable();
          if (oPar[i] == 's')
            vpars[i]->setValues(sPar[sn++], 0, true);
          else
            vpars[i]->setDouble(dPar[dn++], 0, true);
        }
        exist = varPool->callGlobalV(method, oPar, vpars, rPar, &rParCnt);
        value = rParCnt;
      }
      // return the result
      if (value > 10.0 and value < 2e9) // assume it could be an UTM or time value - show with 4 decimals
        snprintf(reply, MRL, "<%s method=\"%s\" par=\"%s\" found=\"%s\" value=\"%.4f\" structCnt=\"%d\">\n",
              msg->tag.getTagName(), method, oPar, bool2str(exist), value, rParCnt);
      else
        snprintf(reply, MRL, "<%s method=\"%s\" par=\"%s\" found=\"%s\" value=\"%g\" structCnt=\"%d\">\n",
                 msg->tag.getTagName(), method, oPar, bool2str(exist), value, rParCnt);
      sendMsg(msg, reply);
      returnedStructs = (rParCnt < MRP);
      if (returnedStructs)
      { // returned static structs
        for (i = 0; i < rParCnt; i++)
        {
          if (rPar[i] != NULL)
          {
            snprintf(ps, MPL, " #%d a %s: ", i, rPar[i]->getDataType());
            rPar[i]->snprint(ps, reply, MRL);
            sendText(msg, reply);
          }
        }
      }
      if (not structKnown)
      {
        snprintf(reply, MRL, "Specified struct type %s is not supported - (OK if resource owns returned structures)\n", returnStructType);
        sendText(msg, reply);
      }
      sendEndTag(msg);
      // delete the return struct array
      for (i = 0; i < MRP; i++)
      {
        if (rParOrg != NULL)
          delete rParOrg[i];
      }
    }
    else
      sendWarning(msg, "method parameter parsing error");
  }
  else
    sendWarning(msg, "method parsing error - must have prackets");
}

/////////////////////////////////////////////////////////////////////////

bool UFunctionVarPool::initCallReturnStructType(UDataBase ** rParOrg, const int MRP, char * returnStructType)
{
  int i;
  ULineSegment lineSeg;
  UPose pose;
  UPosition pos;
  UPoseV poseV;
  UPoseTime poseT;
  UDataString str;
  UVariable var;
  bool result = true;
  //
  for (i = 0 ; i < MRP; i++)
  {
    if (strcmp(returnStructType, lineSeg.getDataType()) == 0)
      rParOrg[i] = new ULineSegment();
    else if (strcmp(returnStructType, pose.getDataType()) == 0)
      rParOrg[i] = new UPose();
    else if (strcmp(returnStructType, poseT.getDataType()) == 0)
      rParOrg[i] = new UPoseTime();
    else if (strcmp(returnStructType, poseV.getDataType()) == 0)
      rParOrg[i] = new UPoseV();
    else if (strcmp(returnStructType, pos.getDataType()) == 0)
      rParOrg[i] = new UPosition();
    else if (strcmp(returnStructType, str.getDataType()) == 0)
      rParOrg[i] = new UDataString();
    else if (var.isA(returnStructType))
      rParOrg[i] = new UVariable();
    else
    { // not a (known) structure name
      rParOrg[i] = NULL;
      if (strlen(returnStructType) > 0)
        result = false;
    }
  }
  return result;
}

////////////////////////////////////////////////////////

bool UFunctionVarPool::sendVarList(UServerInMsg * msg, UVarPool * vp, bool andDesc, bool asSingle)
{
  const int MRL = 2000;
  char reply[MRL];
  bool result = true;
  const int MSL = 2000;
  char s[MSL];
  //
/*  if (leftMargin > 30)
    leftMargin -= 2;*/
  leftMargin = 30;
  if (vp == NULL)
    sendWarning(msg, "No such structure");
  else
  {
    if ((vp->getVarsCnt() == 0) and (vp->getStructCnt() == 0) and (vp->getMethodCnt() == 0))
      sendInfo(msg, "No variables, structures or methods available");
    else
    {
      if (asSingle)
      { // send as XML tags
        sendAllVar(msg, (UVarCalc*)vp, vp->getFullPreName(), andDesc);
      }
      else
      { // send as help list
        snprintf(reply, MRL, "<help subject=\"var list\" name=\"%s\">\n", vp->getPreName());
        sendMsg(msg, reply);
        sendDescription(msg, "Description:", vp->getDescription());
        for (int n = 0; n < vp->getVarsCnt(); n++)
        {
          UVariable * va = vp->getLocalVar(n);
          char hh[3] = "  ";
          if (va->hasHist())
          {
            if (va->hist->isLogOpen())
              hh[1] = 'L';
            if (va->hist->rowMaxCnt > 1)
              hh[0] = 'H';
            if (va->hist->isReplayFileOpen())
              hh[1] = 'R';
          }
          if (va->isTypeA(UVariable::m2))
          {
            snprintf(reply, MRL, "%c%c %s=\"%dx%d\"", hh[0], hh[1], va->name, va->rows(), va->cols());
            sendDescription(msg, reply, vp->getVarDescription(n));
            va->getValuesmAsStringLimited(reply, MRL, 8, 7);
            strcat(reply, "\n");
            sendMsg(reply);
          }
          else
          {
            if (va->isString())
            {
              snprintf(reply, MRL, "%c%c %s=\"%s\"", hh[0], hh[1], va->name, va->getValuesAsString(s, MSL, 0));
              // debug
              // printf("UFunctionVarPool::handleVar: got string (%s)\n", reply);
              // debuge end
            }
            else
              snprintf(reply, MRL, "%c%c %s=%s", hh[0], hh[1], va->name, va->getValuesdAsString(s, MSL, 0));
            sendDescription(msg, reply, vp->getVarDescription(n));
/*              snprintf(&reply[m], MRL - m, "%s\n", vp->getVarDescription(n));
            result = sendMsg(msg, reply);*/
              // debug
              // printf("UFunctionVarPool::handleVar: send string variable  (%s=%s)\n", va->name, reply);
              // debuge end
          }
        }
        for (int n = 0; n < vp->getStructCnt(); n++)
        {
          UVarPool * vp2 = vp->getStruct(n);
          if (vp2 != NULL)
          {
            if (vp2->getVarsCnt() > 0 or vp2->getStructCnt() > 0 or vp2->getMethodCnt() > 0)
            { // show struct if it has any variables only
              snprintf(reply, MRL, "   %s=\'struct\';", vp2->getPreName());
              sendDescription(msg, reply, vp2->getDescription());
            }
          }
        }
        for (int n = 0; n < vp->getMethodCnt(); n++)
        {
          if (vp->getLocalMethod(n)->valid())
          {
            snprintf(reply, MRL, "   %s(%s) ",
                      vp->getLocalMethod(n)->name,
                      vp->getLocalMethod(n)->paramOrder);
            sendDescription(msg, reply, vp->getLocalMethod(n)->description);
          }
        }
        sendMsg("(H: has time series, L: is logged, R: replay)\n");
        sendMsg(msg, "</help>\n");
      }
      result = sendInfo(msg, "done");
    }
  }
  return result;
}
