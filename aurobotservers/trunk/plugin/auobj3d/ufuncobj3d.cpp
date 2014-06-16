/***************************************************************************
 *   Copyright (C) 2008 by DTU (Christian Andersen)                        *
 *   rse@elektro.dtu.dk                                                    *
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


#include <ugen4/uline.h>
#include <urob4/usmltag.h>
#include <urob4/uresposehist.h>

#include "ufuncobj3d.h"


#ifdef LIBRARY_OPEN_NEEDED

/////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncObj3d' with your classname, as used in the headerfile */
  return new UFuncObj3d();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncObj3d::UFuncObj3d()
{ // initialization of variables in class - as needed
  setCommand("obj3d", "objectsIn3D", "handling and filtering of 3D objects (clouds)");
  obj3d = NULL;
  silent = false;
}

///////////////////////////////////////////////////

UFuncObj3d::~UFuncObj3d()
{ // possibly remove allocated variables here - if needed
  if (obj3d != NULL)
    delete obj3d;
}

///////////////////////////////////////////////////

void UFuncObj3d::createResources()
{
  obj3d = new UResObj3d();
  addResource(obj3d, this);
}

///////////////////////////////////////////////////

bool UFuncObj3d::handleCommand(UServerInMsg * msg, void * extra)
{ // do the needed processing and send a reply back to the client
  bool result = false;
  const int MVL = 50;
  char val[MVL];
//  const int MRL = 500;
//  char reply[MRL];
  bool ask4help = false;
  bool updatesOnly = false;
  bool anAdd = false;
  bool aClearAll = false;
  bool aClearGrp = false;
  int aClearGrpIsx = 0;
  //bool aSvs = false;
  bool anAny = false;
  bool aMakeOnly = false;
  bool anObstAll = false;
  bool anObstHuman = false;
  bool aGround = false;
  bool aGndEdgeObst = false;
  UPosition pos1, pos2, pos3;
  UPoseTime poseT;
  UObj3dGroup * og;
  UPolygon40 poly;
  double mergeDist = 0.1;
  UResPoseHist * odoPose;
  bool sendReply = false;
  int maxCnt = 3;
  //USmlTag tag;
  //
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  {
    msg->tag.getAttValueInt("max", &maxCnt);
    msg->tag.getAttValueBool("update", &updatesOnly, true);
    anAdd =  msg->tag.getAttValue("add", val, MVL);
    msg->tag.getAttValueD("x", &pos1.x);
    msg->tag.getAttValueD("x1", &pos1.x);
    msg->tag.getAttValueD("y", &pos1.y);
    msg->tag.getAttValueD("y1", &pos1.y);
    msg->tag.getAttValueD("z", &pos1.z);
    msg->tag.getAttValueD("z1", &pos1.z);
    msg->tag.getAttValueD("x2", &pos2.x);
    msg->tag.getAttValueD("y2", &pos2.y);
    msg->tag.getAttValueD("z2", &pos2.z);
    msg->tag.getAttValueD("x3", &pos3.x);
    msg->tag.getAttValueD("y3", &pos3.y);
    msg->tag.getAttValueD("z3", &pos3.x);
    msg->tag.getAttValueD("merge", &mergeDist);
    //aSvs = msg->tag.getAttValue("svs", val, MVL);
    msg->tag.getAttValueBool("obstAll", &anObstAll, true);
    msg->tag.getAttValueBool("obstHuman", &anObstHuman, true);
    msg->tag.getAttValueBool("ground", &aGround, true);
    msg->tag.getAttValueBool("obstGndEdge", &aGndEdgeObst, true);
    anAny = msg->tag.getAttValue("any", val, MVL);
    aMakeOnly = msg->tag.getAttValue("makeOnly", val, MVL);
    aClearAll = msg->tag.getAttValue("clearAll", val, MVL);
    aClearGrp = msg->tag.getAttValueInt("clearGrp", &aClearGrpIsx);
    msg->tag.getAttValueBool("silent", &silent, true);
  }
  //
  if (ask4help or obj3d == NULL)
  {
    sendHelpStart(msg, "OBJ3D");
    sendText(msg, "3D object management and generation - based on stereo vision (svs plug-in)\n");
    sendText(msg, "Should get data with push command, e.g. stereoPush cmd='obj3d ground obstAll'\n");
    sendText(msg, "--- Available OBJ3D options:\n");
    if (obj3d != NULL)
    {
//      sendText(msg, "svs            Get 3D points from SVS system and update object database\n");
      sendText(msg, "any            Get any available 3D point cloud from SVS system\n");
      sendText(msg, "ground         Find ground plane from svs 3D cloud\n");
      sendText(msg, "obstAll        Find obstacles (may be combined with ground)\n");
      sendText(msg, "obstHuman      Find just human sized obstacles (may be combined with ground)\n");
      sendText(msg, "obstGndEdge    Find obstacles at edge of ground polygon (with ground only)\n");
      sendText(msg, "makeOnly       Process the data, but do not send result polygons\n");
      sendText(msg, "max=N          Sends the most recent N groups (def = 3)\n");
      sendText(msg, "update[=false] Sends updates only (default = false)\n");
      sendText(msg, "add x=Afwd y=Aleft [x2=B y2=B [x3=C y3=C]] [merge=dist] (in robot coordinates)\n");
      sendText(msg, "      Add an obstacle area ABC and merge with other obstacles\n");
      sendText(msg, "clearAll       Remove all detected obstacles (all obstacle groups)\n");
      sendText(msg, "clearGrp=N     Remove obstacle group N (n=0 is the newest (and default))\n");
      sendText(msg, "silent         Less messages to client and console\n");
      sendText(msg, "help           Send this text\n");
      sendText(msg, "---\n");
      sendText(msg, "See also commands: svs help\n");
      sendText(msg, "Object detection parameters see: 'var obj3d'\n");
    }
    else
      sendText(msg, "3D obstacle resource is missing, reload module\n");
    sendHelpDone(msg);
    result = true;
  }
  else
  {
    if (anAdd)
    {
      odoPose = (UResPoseHist*)getStaticResource("odoPose", false);
      if (pos1.dist() < 0.01)
        sendWarning(msg, "The A-position must not be 0.0");
      else if (odoPose == NULL)
        sendWarning(msg, "Needs poseHist resource to add obstacles");
      else
      { // get current pose and newest obstacle group
        poseT = odoPose->getNewest(NULL);
        og = obj3d->getObstGrp(poseT);
        if (og != NULL)
        { // add if not zero and move to odometry coordinates
          poly.add(poseT.getPoseToMap(pos1));
          if (pos2.dist() > 0.1)
            poly.add(poseT.getPoseToMap(pos2));
          if (pos3.dist() > 0.1)
            poly.add(poseT.getPoseToMap(pos3));
          og->addObstPoly(&poly, poseT, false, false);
          obj3d->obstDataUpdated( poseT.t);
        }
        else
          sendWarning(msg, "Failed to get obstacle group for add");
      }
      sendReply = true;
    }
    if (aClearAll)
    {
      obj3d->clear();
      if (not silent)
        sendReply = sendInfo(msg, "removed all obstacles");
    }
    else if (aClearGrp)
    {
      obj3d->clearGrp(aClearGrpIsx);
      if (not silent)
        sendReply = sendInfo(msg, "removed obstacles from group");
    }
    if (anObstAll or anObstHuman or aGround)
    {
      sendReply = handleCloud(msg, (UDataBase *)extra, anAny, aMakeOnly, anObstAll, anObstHuman,
                              aGround, aGndEdgeObst, maxCnt, updatesOnly);
    }
    result = true;
    if (not sendReply and not silent)
      sendWarning(msg, "missing action options - done nothing");
  }
  return result;
}

//////////////////////////////////////////////////

bool UFuncObj3d::handleCloud(UServerInMsg * msg, UDataBase * pushedCloud,
                             bool getAny,
                             bool makeOnly,
                             bool doObstAll, bool doObstHuman, bool doGround,
                             bool doGndEdgeObst,
                             int maxCnt, bool updatesOnly)
{
  bool result;
  UImg3Dpoints * cloud3d = NULL;
  UPlane plane;
  bool sendReply = false;
  const int MRL = 10000;
  char reply[MRL];
  const int MSL = 100;
  char src[MSL];
  //
  if (pushedCloud != NULL)
    if (pushedCloud->isA("img3d"))
      cloud3d = (UImg3Dpoints *) pushedCloud;
  result = cloud3d != NULL;
  if (not result)
  {
    if (msg->tag.getAttValue("source", src, MSL))
      result = obj3d->get3dCloud(src, &cloud3d);
    else
      // try to get from svs plugin
      result = obj3d->get3dCloudFromSvs(getAny, &cloud3d);
  }
  if (not result and not silent)
    sendReply = sendWarning(msg, "no (new) 3D point cloud available");
/*  if (doObst and cloud3d != NULL)
  { // do get 3D cloud from SVS and process
    result = obj3d->do3dCloudFromSvs(cloud3d);
    if (not result)
      sendWarning(msg, "3D cloud processing failed");
    if (not makeOnly)
      sendReply = sendObjects(msg, maxCnt, updatesOnly);
    else if (not sendReply)
      sendReply = sendInfo(msg, "done");
  }*/
  if (result)
  {
    cloud3d->lock();
    if (doGround or doGndEdgeObst)
    { // do get 3D cloud from SVS and process
      result = obj3d->do3dGroundPlane(cloud3d, doGndEdgeObst, true);
      if (not result and not silent)
        sendReply = sendWarning(msg, "3D ground plane processing failed");
      else if (not silent)
      {
        plane = obj3d->getGroundPlane();
        snprintf(reply, MRL, "<%s a=\"%g\" b=\"%g\" c=\"%g\" d=\"%g\" q=\"%g\"/>\n",
                msg->tag.getTagName(), plane.a, plane.b, plane.c, plane.d,
                                    obj3d->getGroundPlaneQuality());
        sendReply = sendMsg(msg, reply);
      }
    }
    if (doObstAll or doObstHuman or doGndEdgeObst)
    {
      if (doObstAll or doObstHuman)
        obj3d->do3dVoxels(cloud3d, doObstHuman, true);
      // send the detected obstacles 
      sendObjects(msg, maxCnt, updatesOnly);
      // send a finished note
      if (not silent)
        sendReply = sendInfo(msg, "3D voxel processing complete");
    }
    cloud3d->unlock();
  }
  return sendReply;
}

//////////////////////////////////////////////////

bool UFuncObj3d::sendObjects(UServerInMsg * msg, int maxCnt,
                                     bool updateOnly)
{
  bool result = true;
  int cnt;
  int n, m, n2;
  const int MRL = 10000;
  char reply[MRL];
  const char * OBST_GRP_TAG = "obstGrp";
  UObj3dGroup * og;
  UObstacle * ob;
  USmlTag tag;
  UTime ut;
  //
  n = obj3d->getGroupsCnt();
  cnt = mini(n, maxCnt);
    // count number of groups with obstacles
  if (cnt > 0)
  { // get newest update tiime
    og = obj3d->getGroup(0);
    ut = og->getPPoseLast()->t;
  }
  n2 = 0;
  for (n = 0; n < cnt; n++)
  {
    og = obj3d->getGroup(n);
    if ((og->getObstsCnt() > 0) and
        (not updateOnly or (ut == og->getPPoseLast()->t)))
      n2++;
  }
  snprintf(reply, MRL, "<%s cnt=\"%d\" tod=\"%lu.%06lu\">\n",
           msg->tag.getTagName(), n2, ut.getSec(), ut.getMicrosec());
  sendMsg(msg, reply);
  //
  for (n = 0; n < cnt; n++)
  {
    /*
    <obstGrp name="maybe" cnt="31" update="false">
    <poset name="first">
    <pose x="0.034" y="-0.001" h="-0.062"/>
    <tod />
    <timeofday tod="12345678.123456"/>\n"
    </poset>
    <poset name="last"> ...
    </poset>
    <obst ...> ... </obst>
    <obst ...> ... </obst>
    </obstGrp> */
    og = obj3d->getGroup(n);
    if ((og->getObstsCnt() > 0) and (not updateOnly or (ut == og->getPPoseLast()->t)))
    {
      snprintf(reply, MRL, "<%s cnt=\"%d\" serial=\"%lu\" update=\"%s\">\n",
               OBST_GRP_TAG, og->getObstsCnt(),
               og->getSerial(), bool2str(updateOnly));
      sendMsg(msg, reply);
          // send first pose
      tag.codePoseTime(og->getPoseFirst(), reply, MRL, "first");
      sendMsg(msg, reply);
          // send last pose
      tag.codePoseTime(og->getPoseLast(), reply, MRL, "last");
      sendMsg(msg, reply);
      // get newest update time
      ut = og->getPPoseLast()->t;
          // send obstacles
      for (m = 0; m < og->getObstsCnt(); m++)
      {
        ob = og->getObstacle(m);
        if ((not updateOnly) or (ut == ob->getPPoseLast()->t))
        { // obstacle is updated (or all obstacles are requested)
          tag.codeObstacle(og->getObstacle(m), reply, MRL, NULL);
          result = sendMsg(msg, reply);
          if (not result)
                // TX-timeout - stop here and try to send end tags
            break;
        }
        else
        { // no change to this obstacle
          snprintf(reply, MRL, "<obst serial=\"%lu\" noChange=\"true\"/>\n",
                   ob->getSerial());
          result = sendMsg(msg, reply);
        }
        // debug
        //printf("**** Send(%s) :%d obsts\n", bool2str(result), og->getObstsCnt());
        // debug end
      }
          // send end tag
      snprintf(reply, MRL, "</%s>\n", OBST_GRP_TAG);
      sendMsg(msg, reply);
    }
  }
  //
  snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
  result = sendMsg(msg, reply);
  //
  return result;
}

//////////////////////////////////////////////////


