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
#include <urob4/uvarcalc.h>

#include "ufunctiondrivepos.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunctionDrivePos' with your classname, as used in the headerfile */
  return new UFunctionDrivePos();
}
//

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionDrivePos::UFunctionDrivePos()
{ // initialization of variables in class - as needed
  setCommand("drivePos", "drivePosIf", "interface to drive to position");
  drivePos = NULL;       // initially the resource is not created
  side = 0;
  dist = 0.5;
}

///////////////////////////////////////////////////

UFunctionDrivePos::~UFunctionDrivePos()
{ // possibly remove allocated variables here - if needed
  if (drivePos != NULL)
    delete drivePos;
}

///////////////////////////////////////////////////

// bool UFunctionDrivePos::setResource(UResBase * resource, bool remove)
// { // load resource as provided by the server (or other plugins)
//   bool result = true;
//   // test if the provided resource is relevant
//   if (resource->isA(UResDrivePos::getResClassID()))
//   { // pointer to server the resource that this plugin can provide too
//     // but as there might be more plugins that can provide the same resource
//     // use the provided
//     if (drivePosLocal)
//       // resource is owned by this plugin, so do not change (or delete)
//       result = false;
//     else if (remove)
//       // the resource is unloaded, so reference must be removed
//       drivePos = NULL;
//     else if (drivePos != (UResDrivePos *)resource)
//       // resource is new or is moved, save the new reference
//       drivePos = (UResDrivePos *)resource;
//     else
//       // reference is not used
//       result = false;
//   }
//   else
//     // other resource types may be needed by base function.
//     result = UFunctionBase::setResource(resource, remove);
//   return result;
// }

///////////////////////////////////////////////////

void UFunctionDrivePos::createResources()
{
  if (drivePos == NULL)
  { // no cam pool loaded - create one now
    drivePos = new UResDrivePos();
    addResource(drivePos, this);
  }
}


// UResBase * UFunctionDrivePos::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UResDrivePos::getResClassID()) == 0)
//   {
//     if (drivePos == NULL)
//     { // no cam pool loaded - create one now
//       drivePos = new UResDrivePos();
//       if (drivePos != NULL)
//       { // mark as locally created (to delete when appropriate)
//         drivePosLocal = true;
//       }
//     }
//     result = drivePos;
//   }
//   if (result == NULL)
//     // requested resource may be held by the base class
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

// bool UFunctionDrivePos::gotAllResources(char * missingThese, int missingTheseCnt)
// { // called by the server core when a status is needed, but may be used
//   // locally from this plugin too (missingThese may be NULL if not needed)
//   bool result = true;
//   bool isOK;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   isOK = (drivePos != NULL);
//   if (not isOK)
//   { // missin this resource
//     if  (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResDrivePos::getResClassID());
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

bool UFunctionDrivePos::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("drivePos"))
    result = handleDrivePos(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionDrivePos::handleDrivePos(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  const int VAL_BUFF_LNG = 500;
  char val[VAL_BUFF_LNG];
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool ask4man = false;
  bool ask4manOnly = false;
  bool result = false;
  int posIsRelOdoGps = 0; // 0=rel, 1=odo, 2=gps
  UPoseV pose, newPose;
  USmlTag nTag;
  UManSeq * manseq;
  UManPPSeq * manpp;
  int i;
  bool replySend = false;
  double v;
  UVarPool * vp = NULL;
  //
  if (drivePos != NULL)
  {
    vp = drivePos->getVarPool();
    if (vp != NULL)
    {
      absPose.clear();
      vp->getLocalValue("odoX", &absPose.x);
      vp->getLocalValue("odoY", &absPose.y);
      vp->getLocalValue("odoH", &absPose.h);
      vp->getLocalValue("odoV", &v);
      absPose.setVel(v);
      relPose.set(5.0, 0.0, 0.0, v);
    }
  }
  // get attributes
  if (msg->tag.getAttValue("rel", val, VAL_BUFF_LNG))
    posIsRelOdoGps=0;
  if (msg->tag.getAttValue("odo", val, VAL_BUFF_LNG))
    posIsRelOdoGps=1;
  if (msg->tag.getAttValue("gps", val, VAL_BUFF_LNG))
    posIsRelOdoGps=2;
  // get default new pose
  switch (posIsRelOdoGps)
  { // get default destination value
    case 0: newPose = relPose; break;
    case 1: newPose = absPose; break;
    //case 2: newPose = gpsPose; break;
    default: break;
  }
  if (msg->tag.getAttValue("x", val, VAL_BUFF_LNG))
    newPose.x = strtod(val, NULL);
  if (msg->tag.getAttValue("y", val, VAL_BUFF_LNG))
    newPose.y = strtod(val, NULL);
  if (msg->tag.getAttValue("h", val, VAL_BUFF_LNG))
    newPose.h = strtod(val, NULL);
  if (msg->tag.getAttValue("th", val, VAL_BUFF_LNG))
    newPose.h = strtod(val, NULL);
  if (msg->tag.getAttValue("vel", val, VAL_BUFF_LNG))
    newPose.setVel(strtod(val, NULL));
  if (msg->tag.getAttValue("v", val, VAL_BUFF_LNG))
    newPose.setVel(strtod(val, NULL));
  ask4help = msg->tag.getAttValue("help", val, VAL_BUFF_LNG);
  ask4man  = msg->tag.getAttValue("man", val, VAL_BUFF_LNG);
  ask4manOnly = msg->tag.getAttValue("manOnly", val, VAL_BUFF_LNG);
  //
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"DrivePos\">\n");
    sendText(msg, "----------- available DrivePos options\n");
    snprintf(reply, MRL, "X=m, Y=m, H=rad  Set target pose (x is forward, y is left, h is heading (+ is left))\n");
    sendText(msg, reply);
    snprintf(reply, MRL, "vel=m/s          Set desired end velocity [m/s]\n");
    sendText(msg, reply);
    snprintf(reply, MRL, "rel              Pose is relative to robot (is %gx, %gy, %grad, %gm/s) (default)\n",
             relPose.x, relPose.y, relPose.h, relPose.getVel());

    sendText(msg, reply);
    snprintf(reply, MRL, "odo              Pose is in odometry coordinates (is %gx, %gy, %grad %gm/s)\n",
             absPose.h, absPose.h, absPose.h, absPose.getVel());
    sendText(msg, reply);
    snprintf(reply, MRL, "man              Find manoeuvre to target pose, avoiding near obstacles\n");
    sendText(msg, reply);
    sendText(msg, "manOnly            Get last calculated manoeuvre sequence \n");
    sendText(msg, "help               This help tekst\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (vp != NULL) // implies that drivePos != NULL too
  {
    if (ask4man)
    { // calculate manoeuvre to a fixed position
      switch (posIsRelOdoGps)
      { // get default destination value
        case 0:
          pose = drivePos->getCurrentPose();
          absPose = pose.getPoseToMapPose(newPose.getPose());
          absPose.setVel(newPose.getVel());
          break;
        case 1: absPose = newPose; break;
        //case 2: pose = convert gps to odo; break;
        default: break;
      }
      result = drivePos->driveOdo(absPose, false, 0);
      if (result)
        ask4manOnly = true;
      else
      {
        pose = drivePos->getCurrentPose();
        snprintf(reply, MRL, "Failed to find path from %.2fx, %.2fy, %.4fh to %.2fx, %.2fy, %.4fh (%.1fdeg)",
                 pose.x, pose.y, pose.h, absPose.x, absPose.y, absPose.h, absPose.h * 180.0 / M_PI);
        sendWarning(msg, reply);
        replySend = true;
      }
    }
    if (ask4manOnly)
    {
      manseq = drivePos->getManLocked();
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
      drivePos->setManUnlocked();
      replySend = true;
    }
    if (not  replySend)
      sendInfo(msg, "unknown command, try 'drivePos help'");
  }
  else
    sendWarning(msg, "No roadDrive resource");
  return result;
}


///////////////////////////////////////////////////

const char * UFunctionDrivePos::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s drive to position (got resource %s)\n",
                  preString, bool2str(drivePos != NULL));
  return buff;
}




