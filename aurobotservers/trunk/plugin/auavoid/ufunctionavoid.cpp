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
#include "ufunctionavoid.h"

#ifdef LIBRARY_OPEN_NEEDED


///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunctionAvoid' with your classname, as used in the headerfile */
  return new UFunctionAvoid();
}
//

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionAvoid::UFunctionAvoid()
{ // initialization of variables in class - as needed
  avoid = NULL;       // initially the resource is not created
  //avoidLocal = false; // ... and is thus not local
  // set plugin command list, name and description note
  setCommand("avoid pp", "avoidPlugin", "obstacle avoidance planning (Christian)");
}

///////////////////////////////////////////////////

UFunctionAvoid::~UFunctionAvoid()
{ // possibly remove allocated variables here - if needed
  // debug
  printf("---- deleting avoid resource\n");
  // debug end
  if (avoid != NULL)
    delete avoid;
}

///////////////////////////////////////////////////

// const char * UFunctionAvoid::name()
// {
//   return "avoid (v2.816) (" __DATE__ " " __TIME__ " by Christian)";
// }

///////////////////////////////////////////////////

// const char * UFunctionAvoid::commandList()
// { // space separated list og command keywords handled by this plugin
//   return "avoid";
// }

///////////////////////////////////////////////////

// bool UFunctionAvoid::setResource(UResBase * resource, bool remove)
// { // load resource as provided by the server (or other plugins)
//   bool result = true;
//   // test if the provided resource is relevant
//   if (resource->isA(UResAvoid::getResClassID()))
//   { // pointer to server the resource that this plugin can provide too
//     // but as there might be more plugins that can provide the same resource
//     // use the provided
//     if (avoidLocal)
//       // resource is owned by this plugin, so do not change (or delete)
//       result = false;
//     else if (remove)
//       // the resource is unloaded, so reference must be removed
//       avoid = NULL;
//     else if (avoid != (UResAvoid *)resource)
//       // resource is new or is moved, save the new reference
//       avoid = (UResAvoid *)resource;
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

void UFunctionAvoid::createResources()
{
  if (avoid == NULL)
  { // no cam pool loaded - create one now
    avoid = new UResAvoid();
    addResource(avoid, this);
  }
}

// UResBase * UFunctionAvoid::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UResAvoid::getResClassID()) == 0)
//   {
//     if (avoid == NULL)
//     { // no cam pool loaded - create one now
//       avoid = new UResAvoid();
//       if (avoid != NULL)
//       { // mark as locally created (to delete when appropriate)
//         avoidLocal = true;
//       }
//     }
//     result = avoid;
//   }
//   if (result == NULL)
//     // requested resource may be held by the base class
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

// bool UFunctionAvoid::gotAllResources(char * missingThese, int missingTheseCnt)
// { // called by the server core when a status is needed, but may be used
//   // locally from this plugin too (missingThese may be NULL if not needed)
//   bool result = true;
//   bool isOK;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   isOK = (avoid != NULL);
//   if (not isOK)
//   { // missin this resource
//     if  (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResAvoid::getResClassID());
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

bool UFunctionAvoid::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("avoid"))
    result = handleAvoid(msg);
  else if (msg->tag.isTagA("pp"))
    result = handlePoseToPose(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionAvoid::handleAvoid(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  char att[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 500;
  char val[VAL_BUFF_LNG];
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool result = false;
  bool unusedOptions = false;
  char unused[MRL] = "";
  bool ask4path = false;
  bool aGetOnly = false;
  bool aUsedOnly = false;
  bool aLog = false;
  bool aLogOpen = false;
  int tanSeq = -1;
  UTime tod;
  UManSeq * path;
  bool aUtm = false;
  bool aMap = false;
  bool aOdo = false;
  bool aDirect = false;
  bool aVisLines = false;
  bool aAvoidPoints = false;
  UPose pose;
  UResPoseHist * poseHist;
  //
  exitPose.set(8.0, -1.0, 0.0);
  exitVel = 0.25;
  while (msg->tag.getNextAttribute(att, val, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else if (strcasecmp(att, "path") == 0)
      ask4path = true;
    else if (strcasecmp(att, "x") == 0)
      exitPose.x = strtod(val, NULL);
    else if (strcasecmp(att, "y") == 0)
      exitPose.y = strtod(val, NULL);
    else if (strcasecmp(att, "h") == 0)
      exitPose.h = strtod(val, NULL);
    else if (strcasecmp(att, "v") == 0)
      exitVel = strtod(val, NULL);
    else if (strcasecmp(att, "tanSeq") == 0)
      tanSeq = strtol(val, NULL, 10);
    else if (strcasecmp(att, "getOnly") == 0)
      aGetOnly = str2bool2(val, true);
    else if (strcasecmp(att, "map") == 0)
      aMap = true;
    else if (strcasecmp(att, "odo") == 0)
      aOdo = true;
    else if (strcasecmp(att, "utm") == 0)
      aUtm = true;
    else if (strcasecmp(att, "direct") == 0)
      aDirect = true;
    else if (strcasecmp(att, "log") == 0)
    {
      aLog = true;
      aLogOpen = str2bool2(val, true);
    }
    else if (strcasecmp(att, "usedOnly") == 0)
      aUsedOnly = str2bool2(val, true);
    else if (strcasecmp(att, "points") == 0)
      aAvoidPoints = str2bool2(val, true);
    else if (strcasecmp(att, "visLines") == 0)
      aVisLines = str2bool2(val, true);
    else
    { // unused options
      unusedOptions = true;
      snprintf(unused, MRL, "%s=\"%s\" ", att, val);
    }
  }
  if (ask4help)
  {
    sendHelpStart(msg, "AVOID");
    sendText(msg, "----------- available AVOID options\n");
    sendText(msg, "path         Find a path to target position\n");
    snprintf(reply, MRL, "x=a y=b h=c  Target pose (relative) is (%gx,%gy,%gh)\n",
             exitPose.x, exitPose.y, exitPose.h);
    sendText(msg, reply);
    snprintf(reply, MRL, "v=velocity   Desired exit velocity - is %gm/s\n",
             exitVel);
    sendText(msg, reply);
    sendText(msg, "odo          Position is specified in odometry (odoPose) coordinates\n");
    sendText(msg, "map          Position is specified in map (mapPose) coordinates\n");
    sendText(msg, "utm          Position is specified in utm (utmPose) coordinates\n");
    sendText(msg, "direct       test direct path only (fail if direct is blocked)\n");
    sendText(msg, "getOnly      Get the latest calculated avoid path and the candidates\n");
    sendText(msg, "usedOnly     Get the latest calculated used avoid path\n");
    if (avoid != NULL)
    {
      snprintf(reply, MRL, "log=[false]  Open [or close] logfile (open=%s) to %s\n",
             bool2str(avoid->isLogAvoidOpen()), avoid->getLogAvoidFileName());
      sendText(msg, reply);
    }
    snprintf(reply, MRL, "tanSeq=N     get visibility graph tangent sequence N  (0=best 1=next,.., def=-1)\n");
    sendText(msg, reply);
    sendText(msg,        "visLines     Get all visibility lines\n");
    sendText(msg,        "points       Get some path support points\n");
    sendText(msg,        "help         This help tekst\n");
    sendText(msg, "---------\n");
    sendText(msg, "See also parameters in 'var avoid'\n");
    sendHelpDone(msg);
  }
  else if (unusedOptions)
  {
    snprintf(reply, MRL, "unused options: %s", unused);
    sendWarning(msg, reply);
  }
  else if (avoid != NULL)
  { // the rest to go here
    if (aLog)
    {
      if (aLogOpen)
        avoid->openLogAvoid();
      else
        avoid->closeLogAvoid();
      sendInfo(msg, "done -- see avoid help for status");
    }
    else if (ask4path)
    { // find path to exit pose
      path = NULL;
      if (not aGetOnly)
      {
        if (aMap)
          poseHist = (UResPoseHist*) getStaticResource("mapPose", false);
        else if (aOdo)
          poseHist = (UResPoseHist*) getStaticResource("odoPose", false);
        else if (aUtm)
          poseHist = (UResPoseHist*) getStaticResource("utmPose", false);
        else poseHist = NULL;
        if (poseHist != NULL)
        { // get current pose
          pose = poseHist->getNewest();
          // find destination in robot coordinates
          exitPose = pose.getMapToPosePose(&exitPose);
        }
        path = avoid->findPathToHere( exitPose, exitVel, true, &tod, aDirect);
        result = (path != NULL);
      }
      else
        tod = avoid->getCalcTime();
      // make sure noone makes a plan during transmission
      avoid->getPathPool()->lock();
      // send all found path candidates - including the actually used one
      if (avoid->useRev2())
        sendCurrentAvoidPath(msg, tod, aUsedOnly, tanSeq, aVisLines, aAvoidPoints);
      else
        sendCurrentPath(msg, tod, aUsedOnly);
      // release the manoeuvre sequence to the pathPool
      if (path != NULL)
        path->unlock();
      avoid->getPathPool()->unlock();
    }
    else
      sendInfo(msg, "try 'avoid help'");
  }
  else
    sendWarning(msg, "No avoid resource");
  return result;
}

///////////////////////////////////////////////////

bool  UFunctionAvoid::handlePoseToPose(UServerInMsg * msg)
{
  char att[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 500;
  char val[VAL_BUFF_LNG];
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool result = false;
  bool unusedOptions = false;
  char unused[MRL] = "";
  UTime tod;
  bool aUtm = false;
  bool aMap = false;
  bool aOdo = false;
  UPose pose;
  UResPoseHist * poseHist, *odoPose;
  UManSeq * man = NULL;
  double turnRad = 0.5; // m
  //
  exitPose.set(8.0, -1.0, 0.0);
  exitVel = 0.25;
  while (msg->tag.getNextAttribute(att, val, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else if (strcasecmp(att, "x") == 0)
      exitPose.x = strtod(val, NULL);
    else if (strcasecmp(att, "y") == 0)
      exitPose.y = strtod(val, NULL);
    else if (strcasecmp(att, "h") == 0)
      exitPose.h = strtod(val, NULL);
    else if (strcasecmp(att, "v") == 0)
      exitVel = strtod(val, NULL);
    else if (strcasecmp(att, "map") == 0)
      aMap = true;
    else if (strcasecmp(att, "odo") == 0)
      aOdo = true;
    else if (strcasecmp(att, "utm") == 0)
      aUtm = true;
    else if (strcasecmp(att, "turnRad") == 0)
      turnRad = strtod(val, NULL);
    else
    { // unused options
      unusedOptions = true;
      snprintf(unused, MRL, "%s=\"%s\" ", att, val);
    }
  }
  if (ask4help)
  {
    sendHelpStart(msg, "PP");
    sendText(msg, "Tool to test Pose to pose path generation - when no obstacles\n");
    sendText(msg, "----------- available PP options\n");
    snprintf(reply, MRL, "x=a y=b h=c  Target pose (relative) is (%gx,%gy,%gh)\n",
             exitPose.x, exitPose.y, exitPose.h);
    sendText(msg, reply);
    snprintf(reply, MRL, "v=velocity   Desired exit velocity - is %gm/s\n",
             exitVel);
    sendText(msg, reply);
    sendText(msg, "odo          Position is specified in odometry (odoPose) coordinates\n");
    sendText(msg, "map          Position is specified in map (mapPose) coordinates\n");
    sendText(msg, "utm          Position is specified in utm (utmPose) coordinates\n");
    sendText(msg, "turnrad      Turn radius to be used - default=0.5m\n");
    sendText(msg,        "help         This help tekst\n");
    sendText(msg, "---------\n");
    sendText(msg, "See also parameters in 'var avoid'\n");
    sendHelpDone(msg);
  }
  else if (unusedOptions)
  {
    snprintf(reply, MRL, "unused options: %s", unused);
    sendWarning(msg, reply);
  }
  else
  {
    UPoseV poFrom, poTo;
    man = new UManSeq();
    odoPose = (UResPoseHist*) getStaticResource("odoPose", false);
    if (aMap)
      poseHist = (UResPoseHist*) getStaticResource("mapPose", false);
    else if (aOdo)
      poseHist = odoPose;
    else if (aUtm)
      poseHist = (UResPoseHist*) getStaticResource("utmPose", false);
    else poseHist = NULL;
    //
    if (poseHist != NULL)
    { // get current pose
      pose = poseHist->getNewest();
      // find destination in robot coordinates
      exitPose = pose.getMapToPosePose(&exitPose);
    }
    if (odoPose == NULL)
      poFrom.set(0.0, 0.0, 0.0, 1.0);
    else
      poFrom = odoPose->getNewest();
    // convert relative destination in exitPose to odometry coordinates
    poTo = poFrom.getPoseToMapPose(exitPose);
    poTo.setVel(exitVel);
    // convert to manoeuvre sequence
    man->addManDriveon(poFrom, poTo, turnRad);
    // send as XML
    sendStartTag("cnt=\"1\"");
    sendManSeq(man);
    sendEndTag();
    delete man;
  }
  return result;
}

///////////////////////////////////////////////////

const char * UFunctionAvoid::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s smr interface resource (empty %s)\n",
                  preString, bool2str(avoid == NULL));
  return buff;
}

//////////////////////////////////////////////////

bool UFunctionAvoid::sendCurrentPath(UServerInMsg * msg, UTime tod, bool usedOnly)
{
  int i, j;
  const int MRL = 10000;
  char reply[MRL];
  UAvoidPathPool * pPool;
  UReactivePath * path;
  UManSeq * manseq;
  USmlTag nTag;
  UManPPSeq * manpp;
  bool result = false;
  //
  pPool = avoid->getPathPool();
  snprintf(reply, MRL, "<%s cnt=\"%d\">\n",
           msg->tag.getTagName(),
           pPool->getValidPathsCnt());
  sendMsg(msg, reply);
  for (j = 0; j < pPool->getPathsCnt(); j++)
  {
    path = pPool->getPath(j);
    if (path->isValid())
    { // this path is valid, and thus a candidate
      manseq = path->getManSeq();
      if (path->getPathUsed() or not usedOnly)
      {
        snprintf(reply, MRL, "<path intervals=\"%d\" pathUsed=\"%s\" tod=\"%lu.%06lu\" aCrash=\"%s\">\n",
                manseq->getP2PCnt(), bool2str(path->getPathUsed()),
                tod.getSec(), tod.getMicrosec(), bool2str(path->isACrash()));
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
        if (path->getMidPosesCnt() > 0)
        {
          for (i = 0; i < path->getMidPosesCnt(); i++)
          {
            nTag.codePose(path->getMidPose(i), reply, MRL, NULL, NULL);
            sendMsg(msg, reply);
          }
        }
        sendMsg(msg, "</path>\n");
      }
    }
  }
  // send end tag
  snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
  result = sendMsg(msg, reply);
  //
  return result;
}

/////////////////////////////////////////////////////////////////

void UFunctionAvoid::sendManSeq(UManSeq * manseq)
{
  const int MRL = 10000;
  char reply[MRL];
  USmlTag nTag;
  UManPPSeq * manpp;
  UTime t;
  //
  t.now();
  snprintf(reply, MRL, "<path intervals=\"%d\" pathUsed=\"true\" tod=\"%lu.%06lu\">\n",
          manseq->getP2PCnt(), t.getSec(), t.getMicrosec());
  sendMsg(reply);
  for (int i = 0; i < manseq->getP2PCnt(); i++)
  {
    manpp = manseq->getP2P(i);
    if (nTag.codeManPPSeq(manpp, reply, MRL, NULL))
      sendMsg(reply);
    else
      printf("UFunctionAvoid::sendCurrentPath: buffer (size %d) too small for message\n",
            MRL);
  }
  sendMsg("</path>\n");
}

/////////////////////////////////////////////////////////////////

bool UFunctionAvoid::sendCurrentAvoidPath(UServerInMsg * msg, UTime tod,
                                          bool usedOnly, int tanSeq,
                                          bool visLines, bool avoidPoints)
{
  int i, j, k, n;
  const int MRL = 10000;
  char reply[MRL];
  UAvoidPathPool * pPool;
  UAvoidPath2 * path;
  UManSeq * manseq;
  USmlTag nTag;
  UManPPSeq * manpp;
  UAvoidObst * aog;
  bool result = false;
  UAvoidNoVis * segNoVis;
  ULineSegment seg, tSeg;
  UAvoidLink * aLnk;
  UPosition p1, p2;
  UAvoidLnkSeq * lnkSeq;
  const int MSL = 50;
  char s[MSL] = "";
  UAvoidPoint *ap, *ap2;
  UPose pose;
  double a;
//  UObstacle * ob;
//  UPosition pos;
  //
  pPool = avoid->getPathPool();
  snprintf(reply, MRL, "<%s cnt=\"%d\">\n",
           msg->tag.getTagName(),
           pPool->getValidAvoidPathsCnt());
  sendMsg(reply);
  for (j = 0; j < pPool->getAvoidPathsCnt(); j++)
  {
    path = pPool->getAvoidPath(j);
    if (path->isValid())
    { // this path is valid, and thus a candidate
      manseq = path->getManSeq();
      if (path->getPathUsed() or not usedOnly)
      {
        snprintf(reply, MRL, "<path intervals=\"%d\" pathUsed=\"%s\" tod=\"%lu.%06lu\" aCrash=\"%s\">\n",
                 manseq->getP2PCnt(), bool2str(path->getPathUsed()),
                 tod.getSec(), tod.getMicrosec(), bool2str(path->isACrash()));
        sendMsg(reply);
        for (i = 0; i < manseq->getP2PCnt(); i++)
        {
          manpp = manseq->getP2P(i);
          if (nTag.codeManPPSeq(manpp, reply, MRL, NULL))
            result = sendMsg(reply);
          else
            printf("UFunctionAvoid::sendCurrentPath: buffer (size %d) too small for message\n",
                   MRL);
          if (not result)
            break;
        }
        for (i = 0; i < path->getAvoidObstGrpCnt(); i++)
        {
          aog = path->getAvoidObstGrp(i);
          while (aog != NULL)
          { // send special non-visibility lines between
            // obstacles in the same group
            segNoVis = aog->noVis;
            for (k = 0; k < aog->noVisCnt; k++)
            {
              for (n = 0; n < 2; n++)
              {
                seg = segNoVis->getNoVisSegment(n);
                nTag.codeLineSegment(&seg, reply, MRL, "noVisSeg");
                sendMsg(reply);
              }
              segNoVis++;
            }
            // send also tangent lines
            if (visLines)
            {
              aLnk = aog->links;
              while (aLnk != NULL)
              { // there is tangent line description, with potentially 4 links
                if (aLnk->mirror != NULL)
                { // each tangent line has a mirror line - do not send duplicate
                  for (k = 0; k < 4; k++)
                  {
                    if (aLnk->valid[k])
                    {
                      p1 = aog->obst->getPoint(aLnk->idx[k]);
                      p2 = aLnk->aob->obst->getPoint(aLnk->aobIdx[k]);
                      tSeg.setFromPoints(p1, p2);
                      // debug
                      //printf("Send tangent %d.%d from %.2fx %.2fy to %.2fx %.2fy\n", aLnk->serial, k, p1.x, p1.y, p2.x, p2.y);
                      // debug end
                      snprintf(s, MSL, "serial=\"%d\"", aLnk->serial);
                      nTag.codeLineSegment(&tSeg, reply, MRL, "tangent", s);
                      sendMsg(reply);
                    }
                  }
                }
                aLnk = aLnk->next;
              }
            }
            // next obstacle in group
            aog = aog->grp;
          }
        }
        if ((tanSeq >= 0) and (path->getLnkSeqsCnt() > tanSeq))
        { // send tanget sequence (for fat display)
          // number 0 is the best
          lnkSeq = path->getLnkSeq(tanSeq);
          k = 0;
          while (lnkSeq != NULL)
          {
            tSeg = lnkSeq->tangLine->getTangentLine(lnkSeq->tangIdx);
            // debug - add serial number
            p1 = tSeg.pos;
            p2 = tSeg.getOtherEnd();
            //printf("Send tangseq %d %d.%d from %.2fx %.2fy to %.2fx %.2fy\n", k, lnkSeq->tangLine->serial,
            //       lnkSeq->tangIdx, p1.x, p1.y, p2.x, p2.y);
            // debug end
            snprintf(s, MSL, "serial=\"%d\"", lnkSeq->tangLine->serial);
            nTag.codeLineSegment(&tSeg, reply, MRL, "seqTangent", s);
            sendMsg(msg, reply);
            lnkSeq = lnkSeq->next;
            k++;
          }
        }
        if (avoidPoints)
        { // send also support points, and their closest path partner
          // send first robot front points (in odometry coordianates
          avoid->codeXmlRobotFront(reply, MRL);
          sendMsg(reply);
          // then the path points
          ap = path->getPointList();
          if (ap != NULL)
          {
            ap2 = ap->prev;
            a = 0.0;
            while (ap != NULL)
            {
              if (ap->prev != NULL and ap->next != NULL)
              {
                ap->mid.codeXml("mid", reply, MRL, NULL);
                sendMsg(reply);
              }
              // send used turn centre
              ap->mCent.codeXml("mCent", reply, MRL, NULL);
              sendMsg(reply);
              // send the point to be avoided and angle in the back direction
              if (ap2 != NULL)
                a = atan2(ap2->aPos.y - ap->aPos.y, ap2->aPos.x - ap->aPos.x);
              pose.set(ap->aPos.x, ap->aPos.y, a);
              pose.codeXml("avoid", reply, MRL, NULL);
              sendMsg(reply);
              //
              if (ap->oob != NULL)
              { // send also the closest neighhbor point - pointing to avoid point
                a = atan2(ap->aPos.y - ap->oPos.y, ap->aPos.x - ap->oPos.x);
                pose.set(ap->oPos.x, ap->oPos.y, a);
                pose.codeXml("avoidNb", reply, MRL, NULL);
                sendMsg(reply);
              }
              //
              ap2 = ap;
              ap = ap->next;
            }
          }
        }
      sendMsg("</path>\n");
      }
    }
  }
  // send end tag
  snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
  result = sendMsg(reply);
  //
  return result;
}


