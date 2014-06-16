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
#include "uclientfuncbase.h"


////////////////////////////////////////////////////////////////////

UCallBack::UCallBack()
{
}

////////////////////////////////////////////////////////////////////

UCallBack::~UCallBack()
{
}

////////////////////////////////////////////////////////////////////

bool UCallBack::onEvent(const char * interface, const char * dataType, void * data)
{
  printf("An event happend on interface %s for dataType %s (data=NULL (%s))\n",
         interface, dataType, bool2str(data));
  return false;
}

////////////////////////////////////////////////////////////////////

bool UCallBack::addOnEvent(UOnEvent * dataObject)
{
  return dataObject->addEventHandler( this, &UCallBack::onEvent);
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

UOnEvent::UOnEvent()
{
  objCnt = 0;
}

////////////////////////////////////////////////////////////////////

UOnEvent::~UOnEvent()
{
}

////////////////////////////////////////////////////////////////////

bool UOnEvent::addEventHandler(UCallBack * object, Method methodToCall)
{
  bool result = false;
  //
  if (objCnt < MAX_CALL_BACKS)
  {
    obj[objCnt] = object;
    method[objCnt] = methodToCall;
    objCnt++;
    result = true;
  }
  return result;
}

////////////////////////////////////////////////////////////////////

bool UOnEvent::event(const char * interface, const char * dataType, void * dataPtr)
{
  int i;
  UCallBack ** cb = obj;
  Method * met = method;
  bool dataUsed = false;
  //
  for (i = 0; i < objCnt; i++)
  {
    dataUsed = dataUsed or (obj[i]->*met[i])(interface, dataType, dataPtr);
    cb++;
    met++;
  }
  return dataUsed;
}


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

UClientFuncBase::UClientFuncBase()
{
  serverNamespaceValue = 0; // not supported namespace
  serverNamespace[0] = '\0';
//  cnn = NULL;
  verboseMessages = false;
  msgHandled = 0;
}

////////////////////////////////////////////////////////////////////

UClientFuncBase::~UClientFuncBase()
{
}

////////////////////////////////////////////////////////////////////

const char * UClientFuncBase::name()
{
  return "- (no additional info)";
}

////////////////////////////////////////////////////////////////////

const char * UClientFuncBase::commandList()
{
  return "help (all-the-rest)";
}

////////////////////////////////////////////////////////////////////

void UClientFuncBase::handleNewData(USmlTag * tag)
{
  // is for me?
  // default is skip all messages
  if (tag->isTagA("help"))
    handleHelp(tag);
  else
  {
    if (verboseMessages)
      tag->print("");
    if (tag->isAStartTag())
      // there may be a long way to the end
      tag->skipToEndTag(1000);
  }
  msgHandled++;
}

////////////////////////////////////////////////////////////////////

void UClientFuncBase::handleHelp(USmlTag * tag)
{
  const int MHL = 1000;
  char helpStr[MHL];
  int strCnt;
  USmlTag eTag;
  bool gotData;
  //
  if (tag->getAttValue("info", helpStr, MHL))
    printf("%s\n", helpStr);
  else
    tag->print("");
  if (tag->isAStartTag())
  { // there is more - just print
    strCnt = MHL - 1;
    while (true)
    {
      gotData = false;
      tag->getNextTag(&eTag, 1000, NULL, helpStr, &strCnt);
      if (strCnt > 0)
      {
        helpStr[strCnt] = '\0';
        xml2str(helpStr, MHL, helpStr, strCnt);
        printf("%s", helpStr);
        gotData = true;
      }
      if (eTag.isValid())
      {
        eTag.print("");
        if (eTag.isTagAnEnd(tag->getTagName()))
          // end of help
          break;
      }
      else if (not gotData)
        // timeout - just quit
        break;
    }
  }
}

////////////////////////////////////////////////////////////////////

void UClientFuncBase::changedNamespace(const char * newNamespace)
{ // should be overwritten by specific data client
  strncpy(serverNamespace, newNamespace, MAX_SML_NAME_LENGTH);
  // set namespace not supported
  serverNamespaceValue = 0;
  // debug
  if (verboseMessages)
    ;
  // debug end
}

////////////////////////////////////////////////////////////////////

void UClientFuncBase::doTimeTick()
{ // just a virtual (interface) function - nothing to
} // do at this level

////////////////////////////////////////////////////////////////////

void UClientFuncBase::printReply(USmlTagIn * tag, const char * preString)
{
  const int MSL = 100;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  //
  // copy attribute string to buffer
  printf("%s <%s ", preString, tag->getTagName());
  while (tag->getNextAttribute(att, val, MSL))
    printf(" %s=\"%s\"", att, val);
  if (tag->isAFullTag())
    printf("/>\n");
  else
    printf(">\n");
}
