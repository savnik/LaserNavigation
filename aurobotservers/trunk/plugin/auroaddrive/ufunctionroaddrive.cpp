/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include <stdio.h>

#include <urob4/usmltag.h>

#include "ufunctionroaddrive.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunctionRoadDrive' with your classname, as used in the headerfile */
  return new UFunctionRoadDrive();
}
//

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionRoadDrive::UFunctionRoadDrive()
{ // initialization of variables in class - as needed
  setCommand("roadDrive", "roadDrive", "roadDrive (v2.1514) (" __DATE__ " " __TIME__ " by Christian)");
  roadDrive = NULL;       // initially the resource is not created
  //roadDriveLocal = false; // ... and is thus not local
  side = 0;
  dist = 0.5;
}

///////////////////////////////////////////////////

UFunctionRoadDrive::~UFunctionRoadDrive()
{ // possibly remove allocated variables here - if needed
  if (roadDrive != NULL)
    delete roadDrive;
}

///////////////////////////////////////////////////

// const char * UFunctionRoadDrive::name()
// {
//   return "roadDrive (v1.72) (" __DATE__ " " __TIME__ " by Christian)";
// }

///////////////////////////////////////////////////

// const char * UFunctionRoadDrive::commandList()
// { // space separated list og command keywords handled by this plugin
//   return "roadDrive";
// }

///////////////////////////////////////////////////

// bool UFunctionRoadDrive::setResource(UResBase * resource, bool remove)
// { // load resource as provided by the server (or other plugins)
//   bool result = true;
//   // test if the provided resource is relevant
//   if (resource->isA(UResRoadDrive::getResClassID()))
//   { // pointer to server the resource that this plugin can provide too
//     // but as there might be more plugins that can provide the same resource
//     // use the provided
//     if (remove)
//       // the resource is unloaded, so reference must be removed
//       roadDrive = NULL;
//     else if (roadDrive != (UResRoadDrive *)resource)
//       // resource is new or is moved, save the new reference
//       roadDrive = (UResRoadDrive *)resource;
//     else
//       // reference is not used
//       result = false;
//   }
//   else
//     // other resource types may be needed by base function.
//     result = UFunctionBase::setResource(resource, remove);
//   return result;
// }

//////////////////////////////////////////////////

void UFunctionRoadDrive::createResources()
{
  if (roadDrive == NULL)
  { // no cam pool loaded - create one now
    roadDrive = new UResRoadDrive();
    addResource(roadDrive, this);
  }  
}


///////////////////////////////////////////////////

// UResBase * UFunctionRoadDrive::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UResRoadDrive::getResClassID()) == 0)
//   {
//     if (roadDrive == NULL)
//     { // no cam pool loaded - create one now
//       roadDrive = new UResRoadDrive();
//       if (roadDrive != NULL)
//       { // mark as locally created (to delete when appropriate)
//         roadDriveLocal = true;
//       }
//     }
//     result = roadDrive;
//   }
//   if (result == NULL)
//     // requested resource may be held by the base class
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

// bool UFunctionRoadDrive::gotAllResources(char * missingThese, int missingTheseCnt)
// { // called by the server core when a status is needed, but may be used
//   // locally from this plugin too (missingThese may be NULL if not needed)
//   bool result = true;
//   bool isOK;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   isOK = (roadDrive != NULL);
//   if (not isOK)
//   { // missin this resource
//     if  (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResRoadDrive::getResClassID());
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//     result = false;
//   }
//   // ask if the function base is OK too
//   isOK = UFunctionBase::gotAllResources(p1, missingTheseCnt - n);
//   return result and isOK;
// }

///////////////////////////////////////////////////

bool UFunctionRoadDrive::handleCommand(UServerInMsg * msg, void * extra)
{ // handle the one command type in this plug-in
  char att[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 500;
  char val[VAL_BUFF_LNG];
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool ask4man = false;
  bool result = false;
  bool unusedOptions = false;
  char unused[MRL] = "";
  bool ask4target = false;
  bool ask2follow = false;
  UPoseV pv;
  USmlTag nTag;
  double targetDist = 6.0;
  UManSeq * manseq;
  UManPPSeq * manpp;
  int i;
  bool replySend = false;
  //
  while (msg->tag.getNextAttribute(att, val, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else if (strcasecmp(att, "target") == 0)
    {
      ask4target = true;
      if (strlen(val) > 0)
        targetDist = strtod(val, NULL);
      else
        targetDist = 6.0;
    }
    else if (strcasecmp(att, "side") == 0)
    {
      if (strcasecmp(val, "left") == 0)
        side = 0;
      else if (strcasecmp(val, "top") == 0)
        side = 1;
      else if (strcasecmp(val, "right") == 0)
        side = 2;
      else
        side = -1;
    }
    else if (strcasecmp(att, "dist") == 0)
      dist = strtod(val, NULL);
    else if (strcasecmp(att, "manOnly") == 0)
      ask4man = str2bool2(val, true);
    else if (strcasecmp(att, "follow") == 0)
      ask2follow = str2bool2(val, true);
    else
    { // unused options
      unusedOptions = true;
      snprintf(unused, MRL, "%s=\"%s\" ", att, val);
    }
  }
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"RoadDrive\">\n");
    sendText(msg, "----------- available RoadDrive options\n");
    sendText(msg, "target=X           Find road drive target pose this distance ahead (def=6m) \n");
    sendText(msg, "side[=left | top | right]   Relative to this road side \n");
    sendText(msg, "dist=d             Try keep this distance from road (positive is left) \n");
    sendText(msg, "follow             Implement roaddrive with obstacle avoidance (return man)\n");
    sendText(msg, "manOnly            Get last calculated manoeuvre sequence \n");
    sendText(msg, "help               This help tekst\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (unusedOptions)
  {
    snprintf(reply, MRL, "unused options: %s", unused);
    sendWarning(msg, reply);
    replySend = true;
  }
  else if (roadDrive != NULL)
  {
    // the rest to go here
    if (ask4target)
    { // find path to exit pose
      result = roadDrive->findExitPose(side, dist, targetDist, &pv);
      if (not result)
        sendWarning(msg, "Failed to get exit-pose, no road or laserroad module or call just failed");
      else
      {
        snprintf(reply, MRL, "<%s side=\"%d\" dist=\"%g\" ahead=\"%g\">\n",
                msg->tag.getTagName(), side, dist, targetDist);
        sendMsg(msg, reply);
        nTag.codePoseV(pv, reply, MRL, "target");
        sendMsg(msg, reply);
        snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
        sendMsg(msg, reply);
      }
      replySend = true;
    }
    else if (ask2follow)
    {
      result = roadDrive->driveSide(side, dist, 0);
      if (result)
        ask4man = true;
      else
      {
        snprintf(reply, MRL, "Follow roadside (%d) at dist=%gm failed - see also server console",
                 side, dist);
        sendWarning(msg, reply);
      }
    }
    if (ask4man)
    {
      manseq = roadDrive->getManLocked();
      if (manseq != NULL)
      {
        snprintf(reply, MRL, "<%s cnt=\"%d\" man=\"current drive path\">\n", msg->tag.getTagName(), manseq->getP2PCnt());
        sendMsg(msg, reply);
        snprintf(reply, MRL, "<path intervals=\"%d\" pathUsed=\"true\">\n",
                manseq->getP2PCnt());
        sendMsg(msg, reply);
        for (i = 0; i < manseq->getP2PCnt(); i++)
        {
          manpp = manseq->getP2P(i);
          if (nTag.codeManPPSeq(manpp, reply, MRL, NULL))
            result = sendMsg(msg, reply);
          else
            printf("UFunctionAvoid::sendCurrentPath: buffer (size %d) too small for message\n",
                  MRL);
          if (not result)
            break;
        }
        sendMsg(msg, "</path>\n");
        sendEndTag(msg);
      }
      else
        sendWarning(msg, "No manoeuver available - no roaddrive ever performed?");
      // manoeuvre or not, the lock is engaged and must be released
      roadDrive->setManUnlocked();
      replySend = true;
    }
    if (not  replySend)
      sendInfo(msg, "unknown command, try 'roadDrive help'");
  }
  else
    sendWarning(msg, "No roadDrive resource");
  return result;
}


///////////////////////////////////////////////////

const char * UFunctionRoadDrive::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s road drive interface (got resource %s)\n",
                  preString, bool2str(roadDrive != NULL));
  return buff;
}




