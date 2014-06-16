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
#include <urob4/usmltag.h>

#include "uclientfuncpath.h"

///////////////////////////////////////////////////

UClientFuncPath::UClientFuncPath()
{
  int i;
  //poly = &polyData;
  for (i = 0; i < MAX_VIS_DATA; i++)
    visData[i] = NULL; // new UVisData();
  visDataMax = 5;
  visDataCnt = 0;
}

///////////////////////////////////////////////////

UClientFuncPath::~UClientFuncPath()
{
}

///////////////////////////////////////////////////

const char * UClientFuncPath::name()
{
  return "free path polygon to 'camPath' resource";
}

///////////////////////////////////////////////////

const char * UClientFuncPath::commandList()
{
  return "pathGet";
}

///////////////////////////////////////////////////

void UClientFuncPath::handleNewData(USmlTag * tag)
{
  if (tag->isTagA("pathGet") )
    handlePathGetMsg(tag);
  else
    printReply(tag, "UClientFuncPath::handleNewData: not mine");
  msgHandled++;
}

///////////////////////////////////////////////////

void UClientFuncPath::handlePathGetMsg(USmlTag * tag)
{ // handle pathGet data
  const int MSL = 100;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
//  char name[MSL];
  USmlTag endTag;
  UVisData * poly;
  // unpack polygon - test for result
  // debug
  //printf("Got polygon data\n");
  // debug end
  if (tag->cnnVerbose())
    tag->print("");
  while (tag->getNextAttribute(att, val, MSL))
  {
    if (strcasecmp(att, "warning") == 0)
    {
      printf("UClientFuncPath::handlePathGetMsg *** failed: %s\n", val);
      if (tag->isAStartTag())
        tag->skipToEndTag(200);
      break;
    }
    else if (strcasecmp(att, "type") == 0)
    {
      if (strcasecmp(val, "UProbPoly") == 0)
      { // there is additional polygon data
        if (visDataCnt < visDataMax)
          // need more data
          poly = new UVisData();
        else
        { // need to reuse structures
          poly = visData[visDataCnt - 1];
        }
        if (tag->getAttValue("name", val, MSL))
          poly->setPolyName(val);
        // debug
        // printf("Got UProbPoly data\n");
        // debug end
        handlePathPolygonData(tag, poly);
        // insert new polygon at atart of array
        if (visDataCnt > 0)
          memmove(&visData[1], visData, sizeof(void*) * (visDataMax - 1));
        visData[0] = poly;
        if (visDataCnt < visDataMax)
          visDataCnt++;
        // skip the end tag
        tag->getNextTag(&endTag, 200);
      }
    }
  }
}

///////////////////////////////////////////////////

void UClientFuncPath::handlePathPolygonData(USmlTag * tag, UVisData * destination)
{
  bool result;
  USmlTag nTag;
  const int MSL = 100;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  int n, cnt = 0;
  //bool hasEdge = false;
  UPosition * pos;
  UTime t;
  bool * isObst;
  UProbPoly * poly = NULL;
  // lock polygon data
  destination->polyLock.lock();
  // get pointer po th polygon itself
  poly = destination->getPoly();
  poly->clear();
  tag->getNextTag(&nTag, 200);
  if (tag->cnnVerbose())
    nTag.print("");
  result = nTag.isTagA("UProbPoly");
  if ((not result) and nTag.isValid())
  { // unknown tag
    printReply(&nTag, "UClientFuncPath::handlePathPolygonData: not mine?");
    // skip the rest until endtag.
    if (nTag.isAStartTag())
      nTag.skipToEndTag(200);
  }
  if (result and not nTag.isAStartTag())
  { // is just a message or a false
    result = false;
    nTag.print("");
  }
  if (result)
  { // read size
    while (nTag.getNextAttribute(att, val, MSL))
    {
      if (strcasecmp(att, "count") == 0)
        n = sscanf(val, "%d", &cnt);
//       else if (strcasecmp(att, "obstFlags") == 0)
//         hasEdge = str2bool(val);
    }
    result = (cnt > 0);
  }
  if (result)
  { // now get the guts of the message
    // that is either time or polygon data
    pos = poly->getPoints();
    isObst = poly->getIsObst();
    n = 0;
    while (tag->getNextTag(&nTag, 200))
    {
      if (tag->cnnVerbose())
        nTag.print("");
      if (nTag.isTagA("pos2db"))
      {
        pos->z=0.0;
        while (nTag.getNextAttribute(att, val, MSL))
        { // read 2d position with obstacle flag
          if (strcasecmp(att, "x") == 0)
            sscanf(val, "%le", &pos->x);
          if (strcasecmp(att, "y") == 0)
            sscanf(val, "%le", &pos->y);
          if (strcasecmp(att, "obst") == 0)
            *isObst = str2bool(val);
        }
        // debug
        //if (verboseMessages)
        //  printf("Got %d pos2db at %f,%f obst(%s)\n",
        //    n, pos->x, pos->y, bool2str(*isObst));
        // debug
        if (n < poly->getPointsMax())
        { // advance --- if space
          n++;
          pos++;
          isObst++;
        }
      }
      else if (nTag.isTagA("time"))
      {
        nTag.getTime(&t);
        poly->setTime(t);
        // debug
        //if (verboseMessages)
        //  printf("Got time\n");
        // debug end
      }
      else
      { // either an unknown extra information or an end tag
        if (nTag.isAnEndTag())
          break;
        if (nTag.isAStartTag())
          // extra grouped info
          nTag.skipToEndTag(200);
      }
    }
    poly->setPointsCnt(n);
    result = n > 0;
    // debug
    //if (verboseMessages)
    //  printf("Got %d positions in polygon\n", n);
    // debug end
  }
  if (result)
  {
    poly->setValid(true);
    destination->setPolyNew(true);
    newDataAvailable(poly);
  }
  // release polygon data
  destination->polyLock.unlock();
}

//////////////////////////////////////////////////////////

void UClientFuncPath::newDataAvailable(UProbPoly * poly)
{
  //printf("Got new polygon with %d elements\n", poly->getPolyCnt());
}

//////////////////////////////////////////////////////////

