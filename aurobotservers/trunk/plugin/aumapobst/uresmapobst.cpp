/***************************************************************************
 *   Copyright (C) 2009 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <urob4/uresposehist.h>
#include <urob4/uobstacle.h>
#include <ugen4/ucommon.h>
#include <uresmapbase.h>

#include "uresmapobst.h"

/////////////////////////////////////////////

void UResMapObst::UResMapObstInit()
{
  // to save the ID and version number
  setLogName("mapobst");
  openLog();
  // create status variables
  createBaseVar();
  // the this class execute the push jobs
  verbose = false;
}

///////////////////////////////////////////

UResMapObst::~UResMapObst()
{
  // stop job starter thread
  stop(true);
  // the jobs are terminated in the job destructor
}

///////////////////////////////////////////

const char * UResMapObst::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  int n = 0;
  char * p1 = buff;
  UTime lastTime; // should be replaced by something more appropriate
  //
  snprintf(p1, buffCnt - n, "%s Obstacle extraction feature\n", preString);
  n += strlen(p1);
  p1 = &buff[n];
  return buff;
}

///////////////////////////////////////////

void UResMapObst::createBaseVar()
{
  varMapPose  = addVar("mappose",  0.0, "d", "(r) robot pose at last call");
  varObstCnt  = addVar("obstCnt",   0.0, "d", "(r) number of extracted obstacles at last run");
//  varLastTime = addVar("exeTime",  0.0, "d", "(r) last time obstacles were extracted");
  varFront = addVar("front", 25.0, "d", "(rw) distance from robot, where to search for map lines (all directions)");
  varBack = addVar("back", 8.0, "d", "(rw) unused");
  varWidth = addVar("width", 20.0, "d", "(rw) unused");
  varMarginSolidFactor = addVar("marginSolidFactor", 0.5, "d", "(rw) part of 'perimeter' that is solid obstacle");
  varMarginFluffyFactor = addVar("marginFluffyFactor", 1.3, "d", "(rw) part of 'perimeter' where false obstacles can be expected");
  varMapLineCnt = addVar("mapLineCnt", 0.0, "d", "(r) Number of map lines loaded to aulocalizer");
  //
  addMethod("getObst", "dddddd", "Get obstacles near map pose (first ddd), "
      "and convert obstacles to local (odometry) coordinates based on "
      "origin of odometry coordinates in the last ddd."
      "The result is returned in the obstacle group pointed to by the class parameter");
  addMethodV("getObst", "cc", "Get obstacles near map pose (first parameter), "
      "and convert obstacles to local (odometry) coordinates based on "
      "origin of odometry coordinates in the second parameter."
      "The result is returned in the obstacle group pointed to by the class result pointer");
}

//////////////////////////////////////////////

bool UResMapObst::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  UPose odoPoseOrigin, mapPose;
  int obstCnt;
  // evaluate standard functions
  if ((strcasecmp(name, "getObst") == 0) and (strcmp(paramOrder, "dddddd") == 0))
  { // return the obstacles in the visinity as obstacles
    mapPose.set(pars[0], pars[1], pars[2]);
    odoPoseOrigin.set(pars[3], pars[4], pars[5]);
    obstCnt = getNearObstacles(mapPose, odoPoseOrigin, returnStruct[0]);
    // return result - if a location is provided
    if (value != NULL)
      *value = obstCnt;
      // it is goot praktice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 1;
  }
  else
    // call name is unknown
    result = false;
  return result;
}

//////////////////////////////////////////////////

bool UResMapObst::methodCallV(const char * name, const char * paramOrder,
                                UVariable * params[],
                                UDataBase ** returnStruct,
                                int * returnStructCnt)
{ // implement method call from math
  bool result = true;
  UPose mapPose, odoPoseOrigin;
  UVariable * var;
  bool structCntSet = false; // is the return struct count set
  double value = 0.0;
  // evaluate standard functions
  if ((strcasecmp(name, "getObst") == 0) and (strcmp(paramOrder, "cc") == 0))
  {
    if (returnStruct[0]->isA("obstGroup"))
    { // a posetime and an obstacle group is available
      mapPose = params[0]->getPose();
      odoPoseOrigin = params[1]->getPose();
      value = getNearObstacles(mapPose, odoPoseOrigin, returnStruct[0]);
    }
  }
  else
    result = false;

  if (not structCntSet and returnStructCnt != NULL)
  { // returnStructCnt is not set, so set return value to result
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setDouble(value);
        *returnStructCnt = 1;
      }
    }
  }
  return result;
}

///////////////////////////////////////////////////

int UResMapObst::getNearObstacles(UPose mapPose, UPose odoPoseOrigin, UDataBase * obstGrp)
{
  int obstCnt = 0;
  UResMapbase * mapbase;
  bool isOK;
  UObstacle * ob;
  mapline ml;
  int i, j;
  U2Dseg line; //, sb, sf, sl, sr;
  double f, d; //r, d, f, b, w;
//  U2Dpos bl, fl, br, fr, xp; // search box
  UObstacleGroup * og = (UObstacleGroup *) obstGrp;
  UPose ps1, ps2; //, ps3;
  U2Dpos po1, po2, po3;
  UPosition pos1, pos2;
  const int NUM_OF_END_POINTS = 4;
  int n;
  UPolygon40 p40;
  UPoseTime pt;
  double margin;
  // Acquire mapbase resource:
  mapbase = (UResMapbase *) getStaticResource("mapbase", false, false);
  if (mapbase != NULL)
  { // resources are available
    // set local pose of area of interest (AOI)
    f = varFront->getValued();
/*    b = varBack->getValued();
    w = varWidth->getValued();
    bl.set(-b, w/2.0);
    br.set(-b, -w/2.0);
    fl.set(f, w/2.0);
    fr.set(f, -w/2.0);
    // convert to map coordinates
    bl = mapPose.getPoseToMap(bl);
    br = mapPose.getPoseToMap(br);
    fl = mapPose.getPoseToMap(fl);
    fr = mapPose.getPoseToMap(fr);
    sb.setFromPoints(bl, br);
    sf.setFromPoints(fl, fr);
    sl.setFromPoints(bl, fl);
    sr.setFromPoints(br, fr);
    // get AOI radius
    r = hypot(f, w/2);*/
    og->clear();
    // find lines within AOI
    for(i = 0; i < (int) mapbase->maplines.size(); i++)
    {
      ml = mapbase->maplines[i];
      isOK = ml.isObstacle;
      if (isOK)
      {
        line.setFromPoints(ml.x_s, ml.y_s, ml.x_e, ml.y_e);
        d = line.getDistanceSigned(mapPose.x, mapPose.y, NULL);
        isOK = (fabs(d) < f);
      }
/*      if (isOK)
      { // this is likely a near line
        isOK = line.isCrossing(&sb, &xp);
        if (not isOK)
          isOK = line.isCrossing(&sf, &xp);
        if (not isOK)
          isOK = line.isCrossing(&sl, &xp);
        if (not isOK)
          isOK = line.isCrossing(&sr, &xp);
      }*/
      if (isOK and true)
      { // this line is to be included
        // expands map-line to polygon
        ob = og->getNewObst();
        ps1.set(line.x, line.y, line.getHeading());
        // and for other end
        po1 = line.getOtherEnd();
        ps2.set(po1.x, po1.y, line.getHeading());
        margin = ml.perimeter * varMarginSolidFactor->getValued();
        if (margin < 0.1)
          n = 2; // this reduces the obstacle to a rectangle
        else
          // use more rounded ends
          n = NUM_OF_END_POINTS;
        d = M_PI / double(n-1);
        p40.clear();
        for (j = 0; j < 2*n; j++)
        { // divide end circle in n points
          if (j < n)
          { // set some arc points in local end coordinates (behind source end)
            po3.set(-sin(d * j) * margin, cos(d * j) * margin);
            // convert to map coordinates
            po2 = ps1.getPoseToMap(po3);
          }
          else
          { // set some arc points in local end coordinates (front of other end)
            po3.set(sin(d * (j - n)) * margin, cos(d * (j - n)) * margin);
            // convert to map coordinates
            po2 = ps2.getPoseToMap(po3);
          }
          // convert to odometry coorinates
          po1 = odoPoseOrigin.getMapToPose(po2);
          // add point to obstacle
          p40.add(po1.x, po1.y, 0.0);
        }
        // ensure convex and in the right order
        p40.extractConvexTo(ob);
        ob->setAsPolygon();
        // set margin
        ob->setMargin(ml.perimeter * varMarginFluffyFactor->getValued() - margin);
        pt = mapPose;
        pt.t.now();
        ob->setPoseFirst(pt);
        ob->setPoseLast(pt);
      }
      else if (isOK)
      { //this line are to be included
        // keeps obstacle as line, but with a margin
        ob = og->getNewObst();
        // add the two endpoints
        ob->add(line.x, line.y, 0.0);
        po1 = line.getOtherEnd();
        ob->add(po1.x, po1.y, 0.0);
        // set as polygon (not polyline)
        ob->setAsPolygon();
        // set margin
        ob->setMargin(ml.perimeter);
        pt = mapPose;
        pt.t.now();
        ob->setPoseFirst(pt);
        ob->setPoseLast(pt);
      }
    }
    obstCnt = og->getObstsCnt();
    varObstCnt->setInt(obstCnt);
    varMapPose->setPose(&mapPose);
  }
  return obstCnt;
}

//////////////////////////////////////////////////////////////////////

int UResMapObst::sendMapLinesToLocalizer(int client, bool justObstacles)
{
  UResMapbase * mapbase;
  bool isOK;
  mapline ml;
  int i, n = 0;
  U2Dseg line, sb, sf, sl, sr;
  U2Dpos bl, fl, br, fr, xp; // search box
  UPose ps1, ps2, ps3;
  U2Dpos po1, po2, po3, po4;
  UPosition pos1, pos2;
  UPoseTime pt;
  const int MRL = 500;
  char reply[MRL];
  // Acquire mapbase resource:
  mapbase = (UResMapbase *) getStaticResource("mapbase", false, false);
  if (mapbase != NULL)
  { // resources are available
    for(i = 0; i < (int) mapbase->maplines.size(); i++)
    {
      ml = mapbase->maplines[i];
      isOK = ml.isObstacle or not justObstacles;
      if (isOK and true)
      { // this line is to be included
        snprintf(reply, MRL, "addline startx=\"%.6f\" starty=\"%.6f\" endx=\"%.6f\" endy=\"%.6f\" name=\"%s\"\n",
               ml.x_s, ml.y_s, ml.x_e, ml.y_e, ml.number);
        getCorePointer()->postCommand(client, reply);
        n++;
        // debug
        printf("----- %s", reply);
        // debug end
      }
    }
    varMapLineCnt->setInt(n);
  }
  return n;
}


