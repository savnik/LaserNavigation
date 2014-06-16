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

//#include "ucalcmmr.h"
#include "uclientfuncgmk.h"

///////////////////////////////////////////////////

UClientFuncGmk::UClientFuncGmk()
{
  gmkPool = new UGmkPool();
  //calc = NULL;
}

///////////////////////////////////////////////////

UClientFuncGmk::~UClientFuncGmk()
{
  delete gmkPool;
}

///////////////////////////////////////////////////

const char * UClientFuncGmk::name()
{
  return "camera guidemark data handler";
}

///////////////////////////////////////////////////

const char * UClientFuncGmk::commandList()
{
  return "gmkGet"; // not implemeented camget camset push time poollist";
}

///////////////////////////////////////////////////

void UClientFuncGmk::handleNewData(USmlTag * tag)
{
  if (tag->isTagA("gmkGet") )
    handleGmkGet(tag);
  else
    printf("Not implelemted by me (UClientFuncGmk)");
  msgHandled++;
}

///////////////////////////////////////////////////

void UClientFuncGmk::handleGmkGet(USmlTag * tag)
{ // handle pathGet data
//   const int MSL = 100;
//   char att[MAX_SML_NAME_LENGTH];
//   char val[MSL];
  USmlTag itag;
  USmlTag gtag;
//  int gmksCnt;
  UTime gmksTime;
  char codeStr[UGmk::MAX_CODE_LENGTH];
  int n;
  UGmk * gmk = NULL;
  unsigned long gmkID;
  const int MBL = 100;
  char tagBuf[MBL];
  int updCnt = 0;
  /*
  <gmkget gmkCnt="1" camName="Left">
  <gmk code="abc1000" cam="Left" device="1">
  <timeofday tod="1129466404.528579"/>
  <pos3d name="gmkPosition" x="0.66967" y="0.036835" z="0.113016"/>
  <rot3d name="gmkRotation" Omega="1.6143908062221" Phi="0.14146691871041" Kappa="0.087895071018558"/>
  </gmk>
  </gmkget>*/
  // unpack guidemark
  gmksTime.now();
//   while (tag->getNextAttribute(att, val, MSL))
//   {
//     if (strcasecmp(att, "gmkCnt") == 0)
//       gmksCnt = strtol(val, NULL, 10);
//   }
  if (tag->cnnVerbose())
    tag->print("");
  if (tag->isAStartTag())
  { // save tag for variable settings
    strncpy(tagBuf, tag->getTagStart(), tag->getTagCnt() + 1);
    tagBuf[tag->getTagCnt() + 1] = '\0';
    // get the rest
    while (tag->getNextTag(&itag, 200))
    {
      if (tag->cnnVerbose())
        itag.print("");
      if (itag.isTagA("gmk"))
      {
        codeStr[0] = '\0';
        itag.getAttValue("id", codeStr, UGmk::MAX_CODE_LENGTH);
        n = strlen(codeStr);
        gmk = NULL;
        if (n > 0)
        {
          gmkID = strtoul(codeStr, NULL, 10);
          if (gmkPool != NULL)
            gmk = gmkPool->getGmk(gmkID, true);
          else
            fprintf(stdout, "handleGmkGet: no gmkPool!\n");
        }
        if (gmk != NULL)
        {
          itag.getGmk(gmk);
          gmksTime = gmk->getTime();
          updCnt++;
        }
        else
        {
          fprintf(stdout, "handleGmkGet: no place to store GMK data - skipping gmk\n");
          itag.skipToEndTag(200);
        }
      }
      else if (itag.isTagAnEnd(tag->getTagName()))
        // end of guidemark group (gmkget)
        break;
      else
      { // either an unknown extra information or an end tag
        if (itag.isAStartTag())
          // extra grouped info
          itag.skipToEndTag(200);
      }
    }
    if (updCnt > 0)
    {
      newDataAvailable(updCnt, gmksTime);
    }
  }
}

//////////////////////////////////////////////////////////

void UClientFuncGmk::newDataAvailable(int updCnt, UTime updTime)
{
  printf("Got %d guidemarks\n", updCnt);
}

