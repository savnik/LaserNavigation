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


#include <urob4/ucmdexe.h>
#include <urob4/uresclientifvar.h>
#include <urob4/uvarpool.h>

#include "ufunctioncamif.h"
#include "uclientfuncgmk.h"
#include "uclientcamifpath.h"
#include "uclientcamifgmk.h"
#include "urescamifimg.h"
#include "urescamifcam.h"


///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'ULasFunc1' with your classname, as used in the headerfile */
  return new UFunctionCamData();
}


#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFunctionCamData::UFunctionCamData()
{
  setCommand("camdata", "cameraData", "handles client-side of camera data (by " __DATE__ " " __TIME__ ")");
  camifPath = NULL;
  camifPathLocal = false;
  camifGmk = NULL;
  camifGmkLocal = false;
  camifImg = NULL;
  camifImgLocal = false;
  camifCam = NULL;
  camifCamLocal = false;
//  strncpy(resList, "camif", MAX_RESOURCE_LIST_SIZE);
}

////////////////////////////////////////

UFunctionCamData::~UFunctionCamData()
{
  if (camifPathLocal)
    delete camifPath;
  if (camifGmkLocal)
    delete camifGmk;
  if (camifImgLocal)
    delete camifImg;
  if (camifCamLocal)
    delete camifCam;
}

///////////////////////////////////////////////////

// const char * UFunctionCamData::resourceList()
// {
//   return resList;
// }

///////////////////////////////////////////////////

bool UFunctionCamData::handleCommand(UServerInMsg * msg, void * extra)
{
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 500;
  char attValue[VAL_BUFF_LNG];
  const int MCL = 500;
  char unused[MCL] = "";
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool anAdd = false;
  char addName[MAX_RESOURCE_ID_LENGTH];
  bool gotUnused = false;
  bool aStatus = false;
  bool aVerbose = false;
  bool verbose = false;
  bool result = false;
  const char * p3;
  bool replyOK = false;
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(attName, "help") == 0)
    { // is help, so do not test the rest
      ask4help = true;
      break;
    }
    else if (strcasecmp(attName, "status") == 0)
      aStatus = true;
    else if (strcasecmp(attName, "add") == 0)
    {
      strncpy(addName, attValue, MAX_RESOURCE_ID_LENGTH);
      anAdd = true;
    }
    else if (strcasecmp(attName, "verbose") == 0)
    {
      aVerbose = true;
      verbose = (strlen(attValue) == 0);
      if (not verbose)
        verbose = str2bool(attValue);
    }
    else
    { // unused options - assume the rest to be to camera server
      gotUnused = true;
      // else assumed to be an attribute to the keyword
      p3 = msg->tag.getNext();
      snprintf(unused, MCL, "%s=\"%s\" %s\n", attName, attValue, p3);
      break;
    }
  }
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"CAM\">\n");
    sendText(msg, "----------- available CAM options\n");
    sendText(msg,        "help      This help tekst\n");
    sendText(msg,        "status    Status for camera interface\n");
/*    snprintf(reply, MRL, "verbose[=false]             Print more to server console (is %s)\n", bool2str(verbose));
    sendText(msg, reply);*/
    snprintf(reply, MRL, "add=img   Add data handler for incomming image data (is added %s)\n", bool2str(camifImg != NULL));
    sendText(msg, reply);
    snprintf(reply, MRL, "add=cam   Add data handler for incomming camera settings data (is added %s)\n", bool2str(camifCam != NULL));
    sendText(msg, reply);
    snprintf(reply, MRL, "add=gmk   Add data handler for incomming guidemark data (is added %s)\n", bool2str(camifGmk != NULL));
    sendText(msg, reply);
    snprintf(reply, MRL, "add=path  Add data handler for incomming road outline data (is added %s)\n", bool2str(camifPath != NULL));
    sendText(msg, reply);
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (gotUnused)
  {
    snprintf(reply, MRL, "unknown attributes from: '%s'", unused);
    sendWarning(msg, reply);
  }
  else
  {
    if (anAdd)
    {
      bool isAdded = false;
      result = true;
      if (strcasecmp(addName, "path") == 0)
      {
        //addResource( UResCamIfPath::getResClassID());
        if (camifPath == NULL)
        { // no pool - so (try to) create one
          camifPath = new UResCamIfPath();
          camifPath->setVerbose(true);
          camifPathLocal = (camifPath != NULL);
          isAdded = addResource(camifPath, this);
        }
      }
      else if (strcasecmp(addName, "gmk") == 0)
      {
        //addResource( UResCamIfGmk::getResClassID());
        if (camifGmk == NULL)
        { // no pool - so (try to) create one
          camifGmk = new UResCamIfGmk();
          camifGmk->setVerbose(true);
          camifGmkLocal = (camifGmk != NULL);
          isAdded = addResource(camifGmk, this);
        }
      }
      else if (strcasecmp(addName, "img") == 0)
      {
        //addResource( UResCamIfImg::getResClassID());
        if (camifImg == NULL)
        { // no image handler - so (try to) create one
          camifImg = new UResCamIfImg();
          camifImg->setVerbose(false);
          camifImgLocal = (camifImg != NULL);
          isAdded = addResource(camifImg, this);
        }
      }
      else if (strcasecmp(addName, "cam") == 0)
      {
        //addResource( UResCamIfCam::getResClassID());
        if (camifCam == NULL)
        { // no image handler - so (try to) create one
          camifCam = new UResCamIfCam();
          camifCam->setVerbose(false);
          camifCamLocal = (camifCam != NULL);
          isAdded = addResource(camifCam, this);
        }
      }
      else
        result = false;
      if (result)
      {
        if (isAdded)
          replyOK = sendInfo(msg, "added (to all interfaces)");
        else
          replyOK = sendInfo(msg, "is added already");
      }
      else
        replyOK = sendWarning(msg, "Unknown interface handler");
    }
    if (aVerbose)
    {
      replyOK = sendInfo(msg, "done");
    }
    if (aStatus)
    {
      sendMsg(msg, "<help subject=\"CAMIF data handler status\">\n");
      if (camifGmk == NULL)
        sendText(msg, " gmk  No Guidemark receive resource (use add=gmk option)\n");
      else
      {
        camifGmk->snprint(" gmk  ", reply, MRL);
        sendText( msg, reply);
      }
      //
      if (camifPath == NULL)
        sendText(msg, " path No Path (road outline) receive resource (use add=path option)\n");
      else
      {
/*          vp = camifPath->getVarPool();
        snprintf(reply, MRL, "  --- Path data handler has (%d/%d var %d/%d structs %d/%d funcs) handled %d msgs.\n",
                vp->getVarsCnt(), vp->getVarMax(),
                vp->getStructCnt(), vp->getStructMax(),
                vp->getMethodCnt(), vp->getMethodMax(),
                camifPath->getMsgCnt());*/
        camifPath->snprint(" path ", reply , MRL);
        sendText( msg, reply);
      }
      //
      if (camifImg == NULL)
        sendText(msg, " img  No image pool receive resource (use add=image option)\n");
      else
      {
/*          snprintf(reply, MRL, "  --- Image handler is %s version %.2f and handled %d messages\n", camifImg->getResID(), camifImg->getResVersion()/100.0, camifImg->getMsgCnt());*/

        camifImg->snprint(" img  ", reply , MRL);
        sendText(msg, reply);
      }
      if (camifCam == NULL)
        sendText(msg, " cam  No camera info receive resource (use add=cam option)\n");
      else
      {
        /*          snprintf(reply, MRL, "  --- Image handler is %s version %.2f and handled %d messages\n", camifImg->getResID(), camifImg->getResVersion()/100.0, camifImg->getMsgCnt());*/

        camifCam->snprint(" cam  ", reply , MRL);
        sendText(msg, reply);
      }
      sendMsg(msg, "</help>\n");
      replyOK = sendInfo(msg, "done");
    }
    if (not replyOK)
      sendWarning(msg, "Unknown cam subject");
  }
  return result;
}

//////////////////////////////////////////////

// bool UFunctionCamData::dataTrap(USmlTag * tag)
// {
//   bool result = false;
//   const int MBL = 20;
//   char buff[MBL];
//   int buffCnt = MBL;
//   USmlTag tagE;
//   const int MRL = 200 + MBL;
//   char reply[MRL];
//   int n;
//   bool binTag = false;
//   int binTagSize = 0;
//   int binTagCnt = 0;
//   //
//   n = tag->getTagCnt();
//   n++;
//   strncpy(reply, tag->getTagStart(), n);
//   reply[n] = '\0';
//   strcat(reply, "\n");
//   sendMsg(dataTrapClient, reply);
//   if (tag->isAStartTag())
//   { // get all data to until end-tag
//     result = true;
//     while (result)
//     {
//       buffCnt = MBL;
//       if (binTag)
//       { // get binary data without check for tag
//         buffCnt = mini(MBL, binTagSize - binTagCnt);
//         result = camif->getNBytes(buff, buffCnt, 400);
//         if (result)
//           binTagCnt += buffCnt;
//         tagE.setValid(false);
//         if (binTagCnt == binTagSize)
//           // end of binary area
//           binTag = false;
//       }
//       else
//         result = camif->getNextTag(&tagE, 400, NULL, buff, &buffCnt);
//       if (result)
//       {
//         n = 0;
//         if (buffCnt > 0)
//         {  // copy any pre-tag data to reply buffer
//           memmove(reply, buff, buffCnt);
//           n = buffCnt;
//         }
//         if (tagE.isValid())
//         { // this is the rest - add to reply (including terminating '>')
//           memmove(&reply[n], tagE.getTagStart(), tagE.getTagCnt() + 1);
//           n += tagE.getTagCnt() + 1;
//           if (tagE.isTagA("bin") and tagE.isAStartTag())
//           { // only a proper bin tag if a size attribute exist
//             binTag = tagE.getAttValue("size", buff, MBL);
//             if (binTag)
//               binTagSize = strtol(buff, NULL, 10);
//             binTagCnt = 0;
//           }
//         }
//         result = sendMsg(dataTrapClient, reply, n);
//         if (tagE.isTagAnEnd(tag->getTagName()))
//         { // this is the end of this story - send a terminating linefeed
//           result = sendMsg(dataTrapClient, "\n", 1);
//           break;
//         }
//       }
//     }
//     result = true;
//   }
//   return result;
// }

////////////////////////////////////////////////////

bool UFunctionCamData::setResource(UResBase * resource, bool remove)
{
  bool result;
  //
  // for camea handler
  camifCam = (UResCamIfCam*) setThisResource(UResCamIfCam::getResClassID(), resource, remove, &result,
                          (UResBase*) camifCam, &camifCamLocal);
  if (not result)
  { // for Manoeuvre handler
    camifGmk = (UResCamIfGmk*) setThisResource(UResCamIfGmk::getResClassID(), resource, remove, &result,
                          (UResBase*) camifGmk, &camifGmkLocal);
  }
  if (not result)
  { // for image handler
    camifImg = (UResCamIfImg*) setThisResource(UResCamIfImg::getResClassID(), resource, remove, &result,
                          (UResBase*) camifImg, &camifImgLocal);
  }
  if (not result)
  { // for road outline path handler
    camifPath = (UResCamIfPath*) setThisResource(UResCamIfPath::getResClassID(), resource, remove, &result,
                          (UResBase*) camifPath, &camifPathLocal);
  }
  // may be needed at lower levels
  result |= UFunctionBase::setResource(resource, remove);
  //
  return result;
}

////////////////////////////////////////////////////

// bool UFunctionCamData::gotAllResources(char * missingThese, int missingTheseCnt)
// {
//   bool result = true;
//   result = UFunctionBase::gotAllResources(missingThese, missingTheseCnt);
//   return result;
// }

////////////////////////////////////////////////////

// UResBase * UFunctionCamData::getResource(const char * resID)
// {
//   UResBase * result = NULL;
// /*  const int MSL = 100;
//   char s[MSL];*/
//   //
// /*  if (strcmp(resID, UResCamIf::getResID()) == 0)
//   {
//     if (camif == NULL)
//     { // no pool - so (try to) create one
//       camif = new UResCamIf();
//       camif->setVerbose(true);
//       snprintf(s, MSL, "name=\"%s\" version=\"%d\"", camif->getResID(), camif->getResVersion());
//       camif->setNamespace(camif->getResID(), s);
//       camif->start();
//       camifLocal = (camif != NULL);
//     }
//     result = camif;
//   }*/
//   if (strcmp(resID, UResCamIfGmk::getResClassID()) == 0)
//   {
//     if (camifGmk == NULL)
//     { // no pool - so (try to) create one
//       camifGmk = new UResCamIfGmk();
//       camifGmk->setVerbose(true);
//       camifGmkLocal = (camifGmk != NULL);
//     }
//     result = camifGmk;
//   }
//   else if (strcmp(resID, UResCamIfPath::getResClassID()) == 0)
//   {
//     if (camifPath == NULL)
//     { // no pool - so (try to) create one
//       camifPath = new UResCamIfPath();
//       camifPath->setVerbose(true);
//       camifPathLocal = (camifPath != NULL);
//     }
//     result = camifPath;
//   }
// /*  else if (strcmp(resID, UResIfVar::getResID()) == 0)
//   {
//     if (camifVar == NULL)
//     { // no pool - so (try to) create one
//       camifVar = new UResIfVar();
//       camifVar->setVerbose(true);
//       camifVar->addTags("odoPose");
//       camifVarLocal = (camifVar != NULL);
//       camif->addFunction(camifVar);
//     }
//     result = camifVar;
//   }*/
//   else if (strcmp(resID, UResCamIfImg::getResClassID()) == 0)
//   {
//     if (camifImg == NULL)
//     { // no image handler - so (try to) create one
//       camifImg = new UResCamIfImg();
//       camifImg->setVerbose(false);
//       camifImgLocal = (camifImg != NULL);
//     }
//     result = camifImg;
//   }
//   else if (strcmp(resID, UResCamIfCam::getResClassID()) == 0)
//   {
//     if (camifCam == NULL)
//     { // no image handler - so (try to) create one
//       camifCam = new UResCamIfCam();
//       camifCam->setVerbose(false);
//       camifCamLocal = (camifCam != NULL);
//     }
//     result = camifCam;
//   }
// /*      cfb = new UClientFuncPath();
//   cfb = new UClientFuncGmk();*/
//   //
//   if (result == NULL)
//     // may need a different ressource
//     result = UFunctionBase::getResource(resID);
//   return result;
// }

////////////////////////////////////////////////////

// bool UFunctionCamData::addResource(const char * resName)
// {
//   bool result = true;
//   char * p1, * p2;
//   int n, m;
//   //
//   p1 = strstr(resList, resName);
//   result = p1 == NULL;
//   m = strlen(resName);
//   n = strlen(resList);
//   if (not result)
//   { // may be a substring
//     p2 = p1-- + strlen(resName);
//     // OK to add if either side of the found string is not a separator
//     result = (*p2 > ' ') or (*p1 > ' ');
//   }
//   if (result)
//     // add
//     result = (n < ( MAX_RESOURCE_LIST_SIZE - m - 1));
//   if (result)
//   {
//     strncat(resList, " ", MAX_RESOURCE_LIST_SIZE);
//     strncat(resList, resName, MAX_RESOURCE_LIST_SIZE);
//     cmdHandler->addNewRessources(this);
//   }
//   //
//   return result;
// }

////////////////////////////////////////////////////

// bool UFunctionCamData::handleCamIfPush(UServerInMsg * msg)
// { // get parameters
//   bool result = false;
//   const int VBL = 50;
//   char val[VBL];
//   bool ask4help = false;
//   bool ask4List = false;
//   const int MRL = 1000;
//   char reply[MRL];
//   //
//   // get relevalt attributes
//   ask4help = msg->tag.getAttValue("help", val, VBL);
//   ask4List = msg->tag.getAttValue("list", val, VBL);
//   // ignore all other attributes
//   if (ask4help)
//   {
//     sendMsg(msg, "<help subject=\"CamOnConnect\">\n");
//     sendText(msg, "----------------Available CamOnConnect options:\n");
//     sendText(msg, "flush=cmd       Remove 'cmd' command(s) defined by client (default all)\n");
//     sendText(msg, "cmd=\"cmd\"       Do execute 'cmd' command on connect\n");
//     sendText(msg, "list            List all defined commands (as help text)\n");
//     sendText(msg, "---\n");
//     sendMsg(msg, "</help>\n");
//     sendInfo(msg, "done");
//     result = true;
//   }
//   else if (camif != NULL)
//   {
//     if (ask4List)
//     {
//       sendMsg(msg, "<help subject=\"IfConnect command list\">\n");
//       camif->UServerPush::print("camif", reply, MRL);
//       sendText(msg, reply);
//       sendMsg(msg, "</help>\n");
//       sendInfo(msg, "done");
//     }
//     else
//     { // push or flush command
//       result = camif->addPushCommand(msg);
//       if (result)
//       {
//         snprintf(val, VBL, "push command succeded - now %d",
//                  camif->getPushQueue()->getPushCmdActiveCnt());
//         sendInfo(msg, val);
//       }
//       else
//         sendWarning(msg, "push command failed");
//     }
//   }
//   else
//     sendWarning(msg, "No interface resource");
//
//   return result;
// }

