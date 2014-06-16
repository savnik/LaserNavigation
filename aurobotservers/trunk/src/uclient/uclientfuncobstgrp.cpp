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

#include "uclientfuncobstgrp.h"

UClientFuncObstGrp::UClientFuncObstGrp()
 : UClientFuncBase()
{
  obsts = new UObstHist();
}

///////////////////////////////////////

UClientFuncObstGrp::~UClientFuncObstGrp()
{
  if (obsts != NULL)
    delete obsts;
}

//////////////////////////////////////////

const char * UClientFuncObstGrp::name()
{
  return "obstacle_group_client-1.0 (jca@oersted.dtu.dk)";
}

//////////////////////////////////////////

const char * UClientFuncObstGrp::commandList()
{
  return "obstGet obst";
}

// //////////////////////////////////////////
//
// void UClientFuncObstGrp::changedNamespace(const char * newNamespace)
// { // set namespace
//   UClientFuncBase::changedNamespace(newNamespace);
//   // set name space value
//   if (strcmp(newNamespace, "mmrd") == 0)
//     serverNamespaceValue = NAMESPACE_MMRD;
//   else if (strcmp(newNamespace, "mmrsr2") == 0)
//     serverNamespaceValue = NAMESPACE_MMRSR2;
//   else if (strcmp(newNamespace, "laserServer") == 0)
//     serverNamespaceValue = NAMESPACE_LASER;
// //  else
// //    not a valid namespace for me.
// //    printf("Unknown namespace (%d) '%s'\n", serverNamespaceValue, laserNamespace);
// }

//////////////////////////////////////////

void UClientFuncObstGrp::handleNewData(USmlTag * tag)
{ // print if desired
  bool isInfo;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  // het first attribute
  tag->getNextAttribute(att, val, MVL);
  tag->reset();
  isInfo = (strcmp(att, "warning") == 0) or
      (strcmp(att, "info") == 0) or
      (strcmp(att, "debug") == 0) or
      (strcmp(att, "error") == 0);
  if (verboseMessages)
    tag->print("");
  else if (isInfo)
    tag->print("");
  // distribute to sub-functions
  if (tag->isTagA("obstGet"))
    handleObst(tag);
  else if (tag->isTagA("obst"))
    handleObst(tag);
  else
    printReply(tag, "UClientFuncLaser::handleNewData: not mine");
}

////////////////////////////////////////

bool UClientFuncObstGrp::handleObst(USmlTag * tag)
{
  bool result = true;
//   const int MVL = 320; //
//   char att[MAX_SML_NAME_LENGTH];
//   char val[MVL];
  USmlTag oTag;
  UTime t;
  //int cnt = 0;
//s  int n;
  UObstacleGroup * og;
  //
//   while (tag->getNextAttribute(att, val, MVL))
//   {
//     if (strcasecmp(att, "cnt") == 0)
//       ; //cnt = strtoll(val, NULL, 10);
//   }
  if (tag->isAStartTag())
  { // get next tag - should be an obstGrp
    obsts->setGroupsCnt(0);
    while (tag->getNextTag(&oTag, 400, tag))
    {
      if (oTag.isTagA("obstGrp"))
      {
        og = obsts->getNewGrp();
        if (og != NULL)
        { // counld be in use by display thread, so lock protect
          og->lock();
          og->clear();
          oTag.getObstacleGroup(og, NULL);
          result = true;
          og->unlock();
        }
        else
        { // store is full - ignore the rest
          oTag.skipToEndTag(400);
          printf("UClientFuncObstGrp::handleObst - no more space - skipping obst group\n");
        }
      }
      // no other types expected - will be ignored
    }
  }
  // announce the new data
  if (result)
  {
    obsts->event("lasif", "obst", NULL);
  }
/*  else
    printf("UClientFuncObstGrp::handleObst: "
        "failed to decode data (no close tag found)\n");*/
  return result;
}

