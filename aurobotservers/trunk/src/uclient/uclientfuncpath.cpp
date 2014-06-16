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
 : UClientFuncBase()
{
  poly = &polyData;
}

///////////////////////////////////////////////////

UClientFuncPath::~UClientFuncPath()
{
}

///////////////////////////////////////////////////

const char * UClientFuncPath::name()
{
  return "Path_from_vision-0.01 (2005 jca@oersted.dtu.dk)";
}

///////////////////////////////////////////////////

const char * UClientFuncPath::commandList()
{
  // OLD - not used
  return "pathGetErr";
}

///////////////////////////////////////////////////

void UClientFuncPath::handleNewData(USmlTag * tag)
{
  if (tag->isTagA("pathGetErr") )
    handlePathGetMsg(tag);
  else
    printReply(tag, "UClientFuncPath::handleNewData: not mine");
}

///////////////////////////////////////////////////

void UClientFuncPath::handlePathGetMsg(USmlTag * tag)
{ // handle pathGet data
  const int MSL = 200;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  USmlTag endTag;
  // unpack polygon - test for result
  while (tag->getNextAttribute(att, val, MSL))
  {
    if (strcasecmp(att, "name") == 0)
      strncpy(polyName, val, MAX_SML_NAME_LENGTH);
    else if (strcasecmp(att, "warning") == 0)
    {
      printf("*** failed: %s\n", val);
      break;
    }
    else if (strcasecmp(att, "type") == 0)
    {
      if (strcasecmp(val, "UProbPoly") == 0)
      { // there is additional data
        handlePathPolygonData(tag);
        // skip the end tag
        tag->getNextTag(&endTag, 200);
      }
    }
  }
}

///////////////////////////////////////////////////

void UClientFuncPath::handlePathPolygonData(USmlTag * iTag)
{
  bool result;
  USmlTag tag;
  const int MSL = 100;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  int n, cnt = 0;
  //bool hasEdge = false;
  UPosition * pos;
  UTime t;
  bool * isObst;
  //
  iTag->getNextTag(&tag, 200);
  result = tag.isTagA("UProbPoly");
  if (not result)
  { // unknown tag
    printReply(&tag, "UClientFuncPath::handlePathPolygonData: not mine?");
    // skip the rest until endtag.
    if (tag.isAStartTag())
      tag.skipToEndTag(200);
  }
  if (result and not tag.isAStartTag())
  { // is just a message or a false
    result = false;
    tag.print("");
  }
  if (result)
  { // read size
    while (tag.getNextAttribute(att, val, MSL))
    {
      if (strcasecmp(att, "count") == 0)
        n = sscanf(val, "%d", &cnt);
      else if (strcasecmp(att, "obstFlags") == 0)
        ; //hasEdge = str2bool(val);
    }
    result = (cnt > 0);
  }
  if (result)
  { // now get the guts of the message
    // that is either time or polygon data
    pos = poly->getPoints();
    isObst = poly->getIsObst();
    n = 0;
    while (iTag->getNextTag(&tag, 200))
    {
      if (tag.isTagA("pos2db"))
      {
        pos->z=0.0;
        while (tag.getNextAttribute(att, val, MSL))
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
      else if (tag.isTagA("time"))
      {
        tag.getTime(&t);
        poly->setTime(t);
        // debug
        //if (verboseMessages)
        //  printf("Got time\n");
        // debug end
      }
      else
      { // either an unknown extra information or an end tag
        if (tag.isAnEndTag())
          break;
        if (tag.isAStartTag())
          // extra grouped info
          tag.skipToEndTag(200);
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
    newDataAvailable(poly);
}

//////////////////////////////////////////////////////////

void UClientFuncPath::newDataAvailable(UProbPoly * poly)
{
  printf("Got new polygon with %d elements\n", poly->getPointsCnt());
}

//////////////////////////////////////////////////////////

