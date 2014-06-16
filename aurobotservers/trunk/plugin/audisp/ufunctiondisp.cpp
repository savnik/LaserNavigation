/***************************************************************************
 *   Copyright (C) 2006 by Christian   *
 *   chrand@mail.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <urob4/ucmdexe.h>
#include <urob4/uresclientifvar.h>
#include <urob4/uvarpool.h>

#include "ufunctiondisp.h"


#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** @todo 'ULasFunc1' with your classname, as used in the headerfile */
  return new UFunctionDisp();
}


#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFunctionDisp::~UFunctionDisp()
{
  if (disp != NULL)
    delete disp;
}

///////////////////////////////////////////////////

void UFunctionDisp::createResources()
{
  disp = new UResDisp();
  if (disp != NULL)
  {
    disp->start();
    addResource(disp, this);
  }
}


////////////////////////////////////////////////////

bool UFunctionDisp::handleCommand(UServerInMsg * msg, void * extra)
{  // handle disp command
  char att[MAX_SML_NAME_LENGTH];
  char att2[MAX_SML_NAME_LENGTH] = "";
  const int VBL = 500;
  char val[VBL];
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool result = false;
  bool replyOK = false;
  bool aLoad = false;
  const int MFL = 300;
  char aLoadName[MFL];
  int theImgNum = 0;
  double v;
  UNavPaint * np;
  bool aPaint = false;
//  bool aCapture = false;
//  int aCaptureValue = 0;
  //
  if (disp == NULL)
    sendError(msg, "No display resource!!");
  else
  {
    np = disp->getNavPaint();
    while (msg->tag.getNextAttribute(att, val, VBL))
    { // camera device
      if (strcasecmp(att, "help") == 0)
        ask4help = true;
      else if (strcasecmp(att, "load") == 0)
      {
        aLoad = true;
        strncpy(aLoadName, val, MFL);
      }
      else if (strcasecmp(att, "img") == 0)
        theImgNum = strtol(val, NULL, 0);
      else if (strcasecmp(att, "uvimg") == 0)
      {
        disp->setUVSource(strtol(val, NULL, 0));
        aPaint = true;
      }
      else if (strcasecmp(att, "croma") == 0)
      {
        disp->setCROMASource(strtol(val, NULL, 0));
        aPaint = true;
      }
      else if (strcasecmp(att, "intensMin") == 0)
      {
        disp->setIntensMin(strtol(val, NULL, 0));
        aPaint = true;
      }
      else if (strcasecmp(att, "intensMax") == 0)
      {
        disp->setIntensMax(strtol(val, NULL, 0));
        aPaint = true;
      }
/*      else if (strcasecmp(att, "testcap") == 0)
      {
        aCapture = true;
        if (strlen(val) > 0)
          aCaptureValue = strtol(val, NULL, 0);
      }*/
      else if (disp != NULL)
      {
        aPaint = true;
        if (strcasecmp(att, "scale") == 0)
          disp->setScale(strtod(val, NULL));
//          np->maxRange = maxd(1.0, strtod(val, NULL));
        else if (strcasecmp(att, "pos") == 0)
          disp->setRobotPose(val);
//          np->robotPos = strtod(val, NULL);
        else if (strcasecmp(att, "bold") == 0)
          disp->paintBold(str2bool2(val, true));
        else if (strcasecmp(att, "curves") == 0)
          np->paintCurves = str2bool2(val, true);
        else if (strcasecmp(att, "cam") == 0)
          np->paintCam = str2bool2(val, true);
        else if (strcasecmp(att, "gmk") == 0)
          np->paintGmk = str2bool2(val, true);
        else if (strcasecmp(att, "hereNow") == 0)
          np->setRefSystemsHere();
        else if (strcasecmp(att, "autoHereNow") == 0)
          disp->setAutoHereNow(str2bool2(val, true));
        else if (strcasecmp(att, "gridSys") == 0)
          np->paintPoseRef = strtol(val, NULL, 0);
        else if (strcasecmp(att, "grid") == 0)
        {
          v = strtod(val, NULL);
          np->paintGridOdo = (v > 0);
          if (v > 0)
            np->paintGridSize = v;
        }
        else if (strcasecmp(att, "rangeRings") == 0)
          disp->setRangeRingCnt(strtol(val, NULL, 10));
        else if (strcasecmp(att, "pass") == 0)
          np->paintIntervalLines = str2bool2(val, true);
        else if (strcasecmp(att, "robot") == 0)
          disp->setRobot(val);
        else if (strcasecmp(att, "obst") == 0)
          np->paintObstCnt = strtol(val, NULL, 0);
        else if (strcasecmp(att, "poseHist") == 0)
        {
          np->paintPoseHistCnt = strtol(val, NULL, 0);
        }
        else if (strcasecmp(att, "poseHistVecCnt") == 0)
          np->paintPoseHistVecCnt = strtol(val, NULL, 0);
        else if (strcasecmp(att, "poseHistVecLng") == 0)
          np->paintPoseHistVecLng = strtol(val, NULL, 0);
        else if (strcasecmp(att, "path") == 0)
          np->paintPathLinesCnt = strtol(val, NULL, 0);
        else if (strcasecmp(att, "pathMid") == 0)
          np->paintPathMidPoses = str2bool2(val, true);
        else if (strcasecmp(att, "road") == 0)
          np->paintRoadHistCnt = strtol(val, NULL, 0);
        else if (strcasecmp(att, "roadAll") == 0)
          np->paintRoadAll = str2bool2(val, true);
        else if (strcasecmp(att, "scan") == 0)
          np->paintScanHistCnt = strtol(val, NULL, 0);
        else if (strcasecmp(att, "visPoly") == 0)
          np->paintVisPolyCnt = strtol(val, NULL, 0);
        else if (strcasecmp(att, "var") == 0)
          np->paintVar = str2bool2(val, true);
        else if (strcasecmp(att, "varAdd") == 0)
        {
          if (np->paintVarAdd(val, true))
            np->paintVar = true;
          else
            strncpy(att2, att, MAX_SML_NAME_LENGTH);
        }
        else if (strcasecmp(att, "varDel") == 0)
        {
          if (not np->paintVarAdd(val, false))
            strncpy(att2, att, MAX_SML_NAME_LENGTH);
        }
        else if (strcasecmp(att, "poly") == 0)
          np->paintPoly = str2bool2(val, true);
        else if (strcasecmp(att, "polyNameCnt") == 0)
          np->paintPolyNameCnt = strtol(val, NULL, 10);
        else if (strcasecmp(att, "polyHide") == 0)
          strncpy(np->paintPolyHide, val, np->maxStrLng);
        else if (strcasecmp(att, "polyShow") == 0)
          strncpy(np->paintPolyShow, val, np->maxStrLng);
        else if (strcasecmp(att, "odoPose") == 0)
          np->paintOdoPose = strtol(val, NULL, 0);
        else if (strcasecmp(att, "utmPose") == 0)
          np->paintUtmPose = strtol(val, NULL, 0);
        else if (strcasecmp(att, "mapPose") == 0)
          np->paintMapPose = strtol(val, NULL, 0);
        else if (strcasecmp(att, "do") == 0)
          disp->setNewDataNav();
        else
          // not found - make a copy for reply
          strncpy(att2, att, MAX_SML_NAME_LENGTH);
      }
    }
    if (ask4help)
    {
      sendMsg(msg, "<help subject=\"disp\">\n");
      sendText(msg, "----------- available disp options\n");
      sendText(msg, "load=filename             Load this image to imagepool image img=N (default N=0)");
      sendText(msg, "img=N                     Use this image buffer number\n");
      sendText(msg, "------ Image colour analysis options ----------\n");
      sendText(msg, "uvimg=N                   Do UV analysis for this image number\n");
      sendText(msg, "croma=N                   Do cromaticity analysis on this image number\n");
      sendText(msg, "intensMin                 Minimum intensity to display colour\n");
      sendText(msg, "intensMax                 Maximum intensity to display colour\n");
      sendText(msg, "------ top-view (laser) image options ----------\n");
      sendText(msg, "do                        update display\n");
      snprintf(reply, MRL,
               "scale=height              Scale image for image height in meter (is %gm)\n", np->maxRange);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "pos=X[,Y]                 Place robot relative to bottom-center of image (is %gx, %gy)\n", np->robotPose.x, np->robotPose.y);
      sendText(msg, reply);
      sendText(msg, "bold[=false]              Paint navigation image using bold lines (for presentations)\n");
      snprintf(reply, MRL,
               "curves[=false]            Paint laser line-fit variance curves (is %s)\n", bool2str(np->paintCurves));
      sendText(msg, reply);
      snprintf(reply, MRL,
               "gridSys=0 | 1 | 2         Paint grid based on 0=odometry, 1=UTM, 2=Map coordinates (is %d)\n", np->paintPoseRef);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "odoPose[=false]           Show odometry pose at bottom of display (is %s)\n", bool2str(np->paintOdoPose));
      sendText(msg, reply);
      snprintf(reply, MRL,
               "utmPose[=false]           Show UTM (GPS) pose at bottom of display (is %s)\n", bool2str(np->paintUtmPose));
      sendText(msg, reply);
      snprintf(reply, MRL,
               "mapPose[=false]           Show map pose at bottom of display (is %s)\n", bool2str(np->paintMapPose));
      sendText(msg, reply);
      sendText(msg, "hereNow                   Synchronoze all coordinate systems to here now\n");
      snprintf(reply, MRL,
               "grid[=M]                  Paint coordinate grid every M meter (is %gm)\n", np->paintGridSize);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "rangeRings[=M]            Paint M range rings around laser scanner (is %dm)\n", np->rangeRingCnt);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "pass[=false]              Paint passable lines from laser scanner (is %s)\n", bool2str(np->paintIntervalLines));
      sendText(msg, reply);
      snprintf(reply, MRL,
               "poly[=false]              Paint poly items - planned mission lines etc. (is %s)\n", bool2str(np->paintPoly));
      sendText(msg, reply);
      snprintf(reply, MRL,
               "polyNameCnt=N             Paint polygon name, max N characters (last), N=%d\n", np->paintPolyNameCnt);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "polyHide=\"name\"           Hide selected poly items - accept wildchards (is '%s')\n", np->paintPolyHide);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "polyShow=\"name\"           Show among hidden poly items - accept wildchards (is '%s')\n", np->paintPolyShow);
      sendText(msg, reply);
      sendText(msg, "robot=[smr|mmr|hako|iRobot|guidebot]  Paint robot outline as SMR, MMR...\n");
      snprintf(reply, MRL,
               "obst=N                    Paint N obstacle groups (is %d)\n", np->paintObstCnt);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "poseHist=N                Paint N pose history positions for robot (is %d)\n", np->paintPoseHistCnt);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "poseHistVecCnt=N          Paint every N pose hist cnt a heading vector (is %d)\n", np->paintPoseHistVecCnt);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "poseHistVecLng=N          Paint pose hist heading vector N pixels long (is %d)\n", np->paintPoseHistVecLng);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "path=[0 | 1 | N]          Paint navigation path plan 0=no, 1=best, N=all (is %d)\n",
              np->paintPathLinesCnt);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "pathMid[=false]           Paint mid-poses used in path calculation (requires path > 0) is %s\n",
               bool2str(np->paintPathMidPoses));
      sendText(msg, reply);
      snprintf(reply, MRL,
               "road[=N]                  Paint Road lines (if n > 0) and N road line updates (is %d)\n", np->paintRoadHistCnt);
      sendText(msg, reply);
      snprintf(reply, MRL,
               "roadAll                   Paint all available road lines (not just current best road) (is %s)\n", bool2str(np->paintRoadAll));
      sendText(msg, reply);
      snprintf(reply, MRL,
               "scan=N                    Paint laserscan and history - up to N scans (is %d)\n", np->paintScanHistCnt);
      sendText(msg, reply);
      sendText(msg, "var[=false]               Paint variables in struct list\n");
      sendText(msg, "varAdd=struct             Paint all variables in this struct\n");
      sendText(msg, "varDel=struct             Hide  all variables in this struct\n");
      snprintf(reply, MRL,
               "visPoly=N                 Paint N polygons from vision road detection (is %d)\n", np->paintVisPolyCnt);
      sendText(msg, reply);
//      sendText(msg, "testcap                   openCV test capture function\n");
      sendText(msg, "help                      This help tekst\n");
      sendText(msg, "--------\n");
      sendText(msg, "See also POOLGET for image save or POOLLIST for available images\n");
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else if (disp != NULL)
    {
      if (aLoad)
      {
        result = disp->loadImgToPool(aLoadName, theImgNum);
        sendInfo(msg, "loaded");
        replyOK = true;
      }
      if (strlen(att2) > 0)
      {
        snprintf(reply, MRL, "Unknown attribute %s", att2);
        sendWarning(msg, reply);
        replyOK = true;
      }
      else if (aPaint)
      { // an OK paint setting
        sendInfo(msg, "done");
        replyOK = true;
        disp->setNewDataNav();
        disp->setUvRedisplay(true);
      }
/*      else if (aCapture)
      { // an OK paint setting
        disp->capture(aCaptureValue);
        sendWarning(msg, "did a capture hmmm? may work");
        replyOK = true;
      }*/
      if (not replyOK)
      {
        snprintf(reply, MRL, "Unknown subject %s - try 'disp help'\n",
                msg->tag.getTagStart());
        sendWarning(msg, reply);
      }
    }
    else
      sendWarning(msg, "No camera interface resource");
  }
  return result;
}
