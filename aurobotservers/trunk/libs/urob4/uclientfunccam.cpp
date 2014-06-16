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
#include "uclientfunccam.h"


UClientFuncCam::UClientFuncCam()
{
  cams = new UClientCams();
}

///////////////////////////////////

UClientFuncCam::~UClientFuncCam()
{
  delete cams;
}

//////////////////////////////////////////

const char * UClientFuncCam::name()
{
  return "camera pose and size to camCam resource";
}

//////////////////////////////////////////

const char * UClientFuncCam::commandList()
{
  return "camGet";
}

//////////////////////////////////////////

void UClientFuncCam::handleNewData(USmlTag * tag)
{ // distribute to sub-functions
  if (tag->isTagA("camGet"))
    handleCamGet(tag);
  else
    printReply(tag, "UClientFuncCam::handleNewData: not mine");
}

//////////////////////////////////////////

bool UClientFuncCam::handleCamGet(USmlTag * tag)
{
  bool result = false;
  const int MVL = 320; //
//  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  int device = 0;
  UClientCamData * camd = NULL;
  bool isOK;
  //
  isOK = tag->getAttValue("device", val, MVL);
  if (isOK)
    device = strtol(val, NULL, 10);
  else
    device = 0;
  camd = cams->getCamData(device, true);
  if (camd != NULL)
    result = camd->handleCamGet(tag);
  if (result)
    gotNewData(device);
  if (tag->cnnVerbose())
    tag->print("tag: ");
  return result;
}

//////////////////////////////////////////

