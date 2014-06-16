//
// C++ Implementation: userverpush
//
// Description:
//
//
// Author: Christian Andersen <jca@elektro.dtu.dk>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include <ctype.h>

#include "userverpush.h"
#include "ucmdexe.h"
#include "uvarpool.h"


//////////////////////////////////////////////

int UServerPushImplement::getFunctionOwner(UServerInMsg * msg)
{
  int result = -1;
  const char * tagName;
  //
  // find tag name
  tagName = msg->tag.getTagName();
  // find owner
  result = findFunctionOwner(tagName);
  return result;
}

//////////////////////////////////////////////

bool UServerPushImplement::addPushWatch(UServerPush * obj)
{
  int i;
  UServerPush ** sp;
  bool found = false;
  //
  pushWatchLock.lock();
  sp = pushWatch;
  for (i = 0; i < pushWatchCnt; i++)
  {
    found = (*sp == obj);
    if (found)
      break;
    sp++;
  }
  if (not found)
  {
    if (pushWatchCnt < MAX_PUSH_WATCH_CNT)
      pushWatch[pushWatchCnt++] = obj;
    else
      printf("****UCmdExe::addPushWatch: out of space, last watch object is ignored\n");
  }
  pushWatchLock.unlock();
  event();
  //
  return not found;
}


//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////


UServerPushElement::UServerPushElement()
{
  clear();
}

//////////////////////////////////////////////

void UServerPushElement::clear()
{
  activeCmd = false;
  activeCall = false;
  countGood = 0;
  countTotal = 0;
  events = 0;
  parOrder[0] = '\0';
  callName = toDo.message;
}

//////////////////////////////////////////////

void UServerPushElement::print(const char * preString)
{
  const int MRL = 500;
  char reply[MRL];
  //
  printf("%s\n", print(preString, reply, MRL));
}

//////////////////////////////////////////////

const char * UServerPushElement::print(const char * preString, char * buff, int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  const int MRL = 500;
  char reply[MRL];
  //
  if (activeCmd or activeCall)
  {
    snprintf(p1, buffCnt, "%s client %d",
             preString, toDo.client);
    n += strlen(p1);
    p1 = &buff[n];
    if (activeCmd)
      snprintf(p1, buffCnt - n, " cmd:'%s'",
               toDo.message);
    else if (activeCmd)
      snprintf(p1, buffCnt - n, " cmd:'%s'", printCall("", reply, MRL));
    n += strlen(p1);
    p1 = &buff[n];
    snprintf(p1, buffCnt - n, " every %g", interval);
    n += strlen(p1);
    p1 = &buff[n];
    if (countGoodTarget > 0)
    {
      snprintf(p1, buffCnt - n, " good %d/%d",
               countGood, countGoodTarget);
      n += strlen(p1);
      p1 = &buff[n];
    }
    else if (countTotalTarget > 0)
    {
      snprintf(p1, buffCnt - n, " fired %d/%d",
               countTotal, countTotalTarget);
      n += strlen(p1);
      p1 = &buff[n];
    }
    else
    {
      snprintf(p1, buffCnt - n, " fired %d times",
               countTotal);
      n += strlen(p1);
      p1 = &buff[n];
    }
    snprintf(p1, buffCnt - n, "\n");
  }
  else
    snprintf(buff, buffCnt, "%s not active\n", preString);
  return buff;
}

//////////////////////////////////////////////

bool UServerPushElement::update(bool success)
{
  countTotal++;
  if (success)
    countGood++;
  if (((countGoodTarget > 0) and (countGood >= countGoodTarget)) or
     ((countTotalTarget > 0) and (countTotal >= countTotalTarget)))
  {
    activeCmd = false;
    activeCall = false;
  }
  if (activeCmd or activeCall)
  {
    nextExeTime += interval;
    if (activeCmd)
      toDo.tag.reset();
  }
  //
  return activeCall or activeCmd;
}

//////////////////////////////////////////////////

const char * UServerPushElement::printCall(const char * preStr, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0, i, ns = 0, nd = 0;
  //
  snprintf(p1, buffCnt - n, "%s active=\"%s\" every=\"%g\" count=\"%d\" call=\"%s(",
           preStr, bool2str(activeCall), interval, countTotal, callName);
  n += strlen(p1);
  p1 = &buff[n];
  for (i = 0; i < int(strlen(parOrder)); i++)
  {
    if (i > 0)
    {
      strncpy(p1, ", ", buffCnt - n);
      n += strlen(p1);
      p1 = &buff[n];
    }
    if (parOrder[i] == 's')
      strncpy(p1, parString[ns++], buffCnt - n);
    else
      snprintf(p1, buffCnt - n, "%g", parDouble[nd++]);
    n += strlen(p1);
    p1 = &buff[n];
  }
  strncpy(p1, ")", buffCnt - n);
  return buff;
}

//////////////////////////////////////////////////

bool UServerPushElement::setCall(const int client, const char * callStr)
{
  toDo.client = client;
  toDo.rxTime.now();
  toDo.size = mini(strlen(callStr), MAX_MESSAGE_LENGTH_TO_CAM);
  strncpy(toDo.message, callStr, toDo.size);
  toDo.message[toDo.size] = '\0';
  return unpackCall();
}

///////////////////////////////////////////////////

bool UServerPushElement::unpackCall()
{
  char *p2;
  char * p1;
  // double prameters
  int dParCnt = 0;
  // string parameters
  int sParCnt = 0;
  // parameter order
  int oParCnt = 0;
  bool result = false;
  //
  p2 = toDo.message;
  callName = strsep(&p2, "(");
  if (p2 != NULL)
  { // skip white space
    while (isspace(*p2))
      p2++;
    // get potential parameters
    while (*p2 != ')')
    {
      if ((*p2 == '\'') or (*p2 == '"'))
      { // string parameter
        parOrder[oParCnt++] = 's';
        // find end of string parameter
        p1 = (char *)stringSep(p2++);
        // terminate this parameter by setting a zero
        if (p1 != '\0')
          *p1++='\0';
        // save as next string parameter
        parString[sParCnt++] = p2;
        // advance to next parameter
        p2 = p1;
      }
      else // assume double literal parameter
      {
        parDouble[dParCnt++] = strtod(p2, &p1);
        p2 = p1;
        parOrder[oParCnt++] = 'd';
      }
      while (isspace(*p2))
        p2++;
      if (*p2 == ',')
        p2++;
      else
        // must be a comma or an end (no comment allowed
        break;
      while (isspace(*p2))
        p2++;
      if (oParCnt >= MAX_PAR)
        break;
    }
    if (*p2 == ')')
    { // got it all
      result = true;
    }
  }
  parOrder[oParCnt] = '\0';
  return result;
}

//////////////////////////////////////////////


//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

UServerPushQueue::UServerPushQueue()
{
  pushCmdNext = 0;
  pushCmdCnt = 0;
}

//////////////////////////////////////////////

void UServerPushQueue::print(const char * preString)
{
  int i;
  const int SL = 30;
  char s[SL];
  //
  printf("%s, %d server push commands (next %d)\n",
         preString, pushCmdCnt, pushCmdNext);
  for (i = 0; i < pushCmdCnt; i++)
  {
    snprintf(s, SL, "   - push #%d", i);
    pushCmd[i].print(s);
  }
}

//////////////////////////////////////////////

void UServerPushQueue::print(const char * preString, char * buff, int buffCnt)
{
  int i;
  const int SL = 30;
  char s[SL];
  char * p1 = buff;
  int m = 0;
  //
  snprintf(buff, buffCnt, "%shas %d push command(s) (next %d)\n",
           preString, pushCmdCnt, pushCmdNext);
  for (i = 0; i < pushCmdCnt; i++)
  {
    m += strlen(p1);
    p1 = &buff[m];
    // pre string
    snprintf(s, SL, "   - push #%d", i);
    // push details
    pushCmd[i].print(s, p1, buffCnt - m);
  }
}

//////////////////////////////////////////////

UServerPushElement * UServerPushQueue::getFreeQueueElement()
{
  UServerPushElement * result = NULL;
  UServerPushElement * qe = pushCmd;
  int i;
  //
  for (i = 0; i < MAX_SERVER_PUSH_CMDS; i++)
  {
    if (not qe->activeCmd or qe->activeCall)
    {
      result = qe;
      if (i >= pushCmdCnt)
        pushCmdCnt = i + 1;
      break;
    }
    qe++;
  }
  return result;
}

///////////////////////////////////////////////////////

UServerPushElement * UServerPushQueue::getNextTimedPushElement()
{
  int i;
  UServerPushElement * result;
  UTime t;
  bool found = false;
  //
  t.Now();
  result = &pushCmd[pushCmdNext];
  for (i = 0; i < pushCmdCnt; i++)
  {
    if (result->activeCmd or result->activeCall)
      // test for time to push
      found = ((t - result->nextExeTime) > 0.0);
    // set next element to try
    pushCmdNext = (pushCmdNext + 1) % pushCmdCnt;
    if (found)
    { // reset tag-decoding to just after tag-name
      if (result->activeCmd)
        result->toDo.tag.reset();
      break; // got a push to activate
    }
    // advance to next element
    if (pushCmdNext == 0)
      result = &pushCmd[0];
    else
      result++;
  }
  if (not found)
    result = NULL;
  return result;
}

//////////////////////////////////////////////

int UServerPushQueue::systemQueueflush(const int client, const char * attValue)
{
  int i;
  UServerPushElement * qe = pushCmd;
  int cnt = 0; // count of unaffected queue elements
  bool active = true;
  //
  // debug
  printf("UServerPushQueue::systemQueueflush: for client %d att='%s'\n",
         client, attValue);
  // debug end
  //
  for (i = 0; i < pushCmdCnt; i++)
  {
    if ((qe->activeCmd or qe->activeCall) and (qe->toDo.client == client))
    { // right client
      active = true;
      if (strlen(attValue) == 0)
        // all functions to this client
        active = false;
      else if (strcasecmp(qe->toDo.getTag()->getTagName(), attValue) == 0)
        // function to right function
        active = false;
      else if (strncasecmp(qe->toDo.message, attValue, strlen(qe->toDo.message)) == 0)
        active = false;
      else
        cnt++;
      if (not active)
      { // deactivate
        qe->activeCall = false;
        qe->activeCmd = false;
      }
    }
    else
      cnt++;
    // advance to next push message
    qe++;
  }
  return pushCmdCnt - cnt;
}

/////////////////////////////////////////////////////////

int UServerPushQueue::getPushCmdActiveCnt(int * cmdCnt, int * callCnt)
{
  int result = 0;
  int i, cm = 0, ca = 0;
  UServerPushElement * pe = pushCmd;
  //
  // debug
  if (pushCmdCnt < 0 or pushCmdCnt > MAX_SERVER_PUSH_CMDS)
  {
    printf("UServerPushQueue::getPushCmdActiveCnt: push cmd count too big %d! - case of tramp?\n", pushCmdCnt);
  }
  else
  // debug end
    for (i = 0; i < pushCmdCnt; i++)
    {
      if (pe->activeCmd or pe->activeCall)
      {
        if (pe->activeCmd)
          cm++;
        if (pe->activeCall)
          ca++;
        result++;
      }
      pe++;
    }
  if (cmdCnt != NULL)
    *cmdCnt = cm;
  if (callCnt != NULL)
    *callCnt = ca;
  //
  return result;
}

///////////////////////////////////////////////////////

UTime UServerPushQueue::getNextExeTime()
{
  int i;
  UServerPushElement * pe;
  UTime t;
  bool found = false;
  //
  pe = pushCmd;
  for (i = 0; i < pushCmdCnt; i++)
  {
    if (pe->activeCmd or pe->activeCall)
    { // test for time to push
      if (not found)
      {
        t = pe->nextExeTime;
        found = true;
      }
      else
      {
        if (t < pe->nextExeTime)
          t = pe->nextExeTime;
      }
    }
    pe++;
  }
  if (not found)
  { // no timed jobs, so set sometime in future
    t.now();
    t += 10.0;
  }
  return t;
}

/////////////////////////////////////////////////////

bool UServerPushQueue::doPushCall(UVarPool * vp, UServerPushElement * pe, const char * value)
{
  int j, n;
  const int MPL = 400;
  char ps[MPL];
  const char * p1;
  char *p4;
  const char *p2, *p3;
  const char * pp[UServerPushElement::MAX_PAR];
  double v;
  bool isOK = false;
  //
  if (pe->activeCall and vp != NULL and pe != NULL)
  { // is a call find parameters with optional extra value
    p1 = pe->parOrder;
    for (j = 0; j < (int)strlen(pe->parOrder); j++)
    {
      if (*p1 == 's')
      {
        p3 = pe->parString[j];
        p2 = strstr(p3, "%s");
        if (p2 == NULL)
        { // use string parameter as is
          pp[j] = p3;
        }
        else
        { // make new extended parameter
              // first part of parameter - before %s
          n = p2 - p3;
          if (n > 0)
            strncpy(ps, p3, n);
              // new part from value
          p4 = ps + n;
          n = strlen(value);
          if (n > 0)
            strncpy(p4, value, n);
              // new insert point
          p4 += n;
              // new source point
          p2 += 2;
              // last part of parameter
          strcpy(p4, p2);
          //
          pp[j] = ps;
        }
      }
    }
    // now string parameters are ready in pp
    v = 0.0; // call false
    vp->callGlobal(pe->callName, pe->parOrder, (char**)pp, pe->parDouble, &v, NULL, 0);
    isOK = (v >= 0.5);
    pe->update(isOK);
  }
  return isOK;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

UServerPush::UServerPush()
{
  cmdImpl = NULL;
  updateCnt = 0;
  callImpl = NULL;
}

////////////////////////////////////////////////////////////

UServerPush::~UServerPush()
{
}

////////////////////////////////////////////////////////////

void UServerPush::gotNewData(void * data)
{
  UServerPushElement * qe;
  int i, n;
  bool success;
  //
  if (cmdImpl != NULL)
  {
    for (i = 0; i < push.getPushCmdCnt(); i++)
    {
      qe = push.get(i);
      if (qe->activeCmd)
      {
        if (cmdImpl->isClientAlive(qe->toDo.client, 0))
        { // test event counter
          n = roundi(qe->interval);
          if ((n <= 1) or (qe->events % n) == 0)
          { // execute function
            success = cmdImpl->executePushFunction(qe->functionIndex, &qe->toDo, data);
            // update push element with result
            qe->update(success);
          }
        }
        else
          qe->activeCmd = false;
        qe->events++;
      }
    }
  }
}

////////////////////////////////////////////////////////////

bool UServerPush::needNewData()
{
  UServerPushElement * qe;
  int i, n;
  bool result = false;
  //
  if (cmdImpl != NULL)
  {
    for (i = 0; i < push.getPushCmdCnt(); i++)
    { // test all push queue elements
      qe = push.get(i);
      if (qe->activeCmd)
      { // test if client is alive
        if (cmdImpl->isClientAlive(qe->toDo.client, 0))
        { // test event counter
          n = roundi(qe->interval);
          if ((n <= 1) or (qe->events % n) == 0)
          {  // someone alive needs the data
            result = true;
            break;
          }
        }
        else
          qe->activeCmd = false;
      }
    }
    if (not result)
    { // noone needs data now, so just update event counter
      for (i = 0; i < push.getPushCmdCnt(); i++)
      { // update all (active) push queue elements
        qe = push.get(i);
        if (qe->activeCmd)
        { // update evet counter
          qe->events++;
        }
      }
    }
  }
  return result;
}

////////////////////////////////////////////////////////////

int UServerPush::addPushCommand(UServerInMsg * msg)
{
  int result = 0;
//  char attName[MAX_SML_NAME_LENGTH];
  const int MVL = MAX_MESSAGE_LENGTH_TO_CAM;
  char attValue[MVL];
  char pushCmd[MVL];
  char pushCall[MVL];
  //char pushCmd[VAL_BUFF_LNG] = "";
  double interval = 1.0;
  int countGood = -1;
  int countTotal = -1;
  bool isAFlush = false;
  bool gotCmd, gotCall;
  UServerPushElement * qe;
  // reset tag to read all attributes (again)
  if ((cmdImpl != NULL) and (msg != NULL))
  {
    msg->tag.reset();
    // read attributes (except device)
    isAFlush = msg->tag.getAttValue("flush", attValue, MVL);
    gotCmd = (msg->tag.getAttValue("cmd", pushCmd, MVL));
    gotCall = (msg->tag.getAttValue("call", pushCall, MVL));
    //
    if (isAFlush)
    { // flush
      result = push.systemQueueflush(msg->client, attValue);
    }
    else if (gotCmd or gotCall)
    { // space for more?
      // get requested camera
      qe = push.getFreeQueueElement();
      if (qe != NULL)
      { // add to queue
        qe->clear();
        if (gotCmd and cmdImpl != NULL)
        {
          qe->toDo.setMessage(msg->client, pushCmd, strlen(pushCmd), false);
          qe->functionIndex = cmdImpl->getFunctionOwner(&qe->toDo);
          if (qe->functionIndex >= 0)
          {
            qe->toDo.serverPushCommand = true;
            qe->activeCmd = true;
          }
          else
            printf("UServerPush::addPushCommand failed to locate command owner for '%s'\n", pushCmd);
        }
        if (gotCall)
        {
          qe->setCall(msg->client, pushCall);
          qe->activeCall = true;
        }
        if ((msg->tag.getAttValue("i", attValue, MVL)) or
              (msg->tag.getAttValue("t", attValue, MVL)) or
              (msg->tag.getAttValue("interval", attValue, MVL)))
          interval = strtol(attValue, NULL, 0);
        qe->interval = interval;
        if ((msg->tag.getAttValue("g", attValue, MVL)) or
            (msg->tag.getAttValue("good", attValue, MVL)))
          countGood = strtol(attValue, NULL, 0);
        qe->countGoodTarget = countGood;
        if ((msg->tag.getAttValue("n", attValue, MVL)) or
              (msg->tag.getAttValue("total", attValue, MVL)))
          countTotal = strtol(attValue, NULL, 0);
        qe->countTotalTarget = countTotal;
        qe->nextExeTime.Now();
        qe->nextExeTime += qe->interval;
        if (gotCmd or gotCall)
          result = 1;
      }
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////////

void UServerPush::flushClientCmds(int clientIdx)
{
  push.systemQueueflush(clientIdx, "");
}

////////////////////////////////////////////////////////////

void UServerPush::print(const char * preString)
{
  push.print(preString);
}

////////////////////////////////////////////////////////////

void UServerPush::print(const char * preString, char * buff, int buffCnt)
{
  char * p1 = buff;
  int n;
  snprintf(p1, buffCnt, "%s has received %d updates\n", preString, updateCnt);
  n = strlen(p1);
  p1 = &buff[n];
  push.print("   ", p1, buffCnt - n);
}

////////////////////////////////////////////////////////////

// bool UServerPush::setUpdated()
// { // add this object to the server push list
//   bool added = false;
//   // update count
//   updateCnt++;
//   // add core trap if needed
//   if (cmdImpl != NULL)
//     if (getPushCmdCnt(NULL, NULL) > 0)
//       added = cmdImpl->addPushWatch(this);
//   return added;
// }

/////////////////////////////////////////////////////////////

bool UServerPush::setUpdated(const char * value)
{ // add this object to the server push list
  bool isOK = false;
  int i, ca, cm;
  UServerPushElement * pe;
  // update count
  updateCnt++;
  // add core trap if needed
  getPushCmdCnt(&cm, &ca);
  if (cmdImpl != NULL)
  { // normal cmd implemented event
    if (cm > 0)
      isOK = cmdImpl->addPushWatch(this);
    if (callImpl == NULL and ca > 0)
    { // get var-pool resource
      callImpl = (UResVarPool*)cmdImpl->getStaticResource("varPool", false, true);
    }
  }
  if (callImpl != NULL and ca > 0)
  { // call implemented event handling
    if (callLock.tryLock())
    {
      for (i = 0; i < push.getPushCmdCnt(); i++)
      {
        pe = push.get(i);
        isOK = doPushCall(pe, value);
      }
      callLock.unlock();
    }
  }
  return isOK;
}

/////////////////////////////////////////////////////////////



bool UServerPush::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  UCmdExe * ce;
  //
  if (resource->isA("core")) // UCmdExe::getResClassID()))
  { // this ressource may hold variables that can be accessed by this module
    if (remove)
      cmdImpl = NULL;
    else
    { // resource is usable as implementor of push commands.
      ce = (UCmdExe *) resource;
      result = (ce != cmdImpl);
      if (result)
        cmdImpl = ce;
    }
  }
  if (resource->isAlsoA(UResVarPool::getResClassID()))
  {
    if (remove)
      callImpl = NULL;
    else if (callImpl != (UResVarPool *) resource)
    { // resource is also a var-pool resource,
      // so save for implement of call events
      callImpl = (UResVarPool *) resource;
      result = true;
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////////

bool UServerPush::doPushCall(UServerPushElement * pe, const char * value)
{
  UVarCalc * vc;
  UVarPool * vp;
  bool result = false;
  //
  if (callImpl != NULL)
  {
    vc = callImpl->getVarPool();
    if (vc != NULL)
    {
      vp = (UVarPool*)vc;
      result = push.doPushCall(vp, pe, value);
    }
  }
  return result;
}
