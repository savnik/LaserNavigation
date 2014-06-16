/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)                        *
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

#include <stdio.h>
#include <math.h>

#include <urob4/uvarcalc.h>
#include <urob4/uresposehist.h>

#include "uresobstacle.h"

///////////////////////////////////////////

UResObstacle::~UResObstacle()
{
  delete fixeds;
}

///////////////////////////////////////////

void UResObstacle::addObstacleParameters()
{
  addVar("version", getResVersion()/100.0, "d", "(ro) Resource version");  // version of this module
  var.outdoorCombineDist = addVar("outdoorCombineDist", 0.25, "d",
                                       "(rw) Combine obstacles if separation is less than this");  //
  var.indoorCombineDist = addVar("indoorCombineDist", 0.03, "d",
                                    "(rw) Combine obstacles if separation is less than this");   //
  var.outdoorContext = addVar("outdoorMode", 1.0, "d",
                                  "(rw) Use outdoor settings (when '1', else indoor settings)");
  var.extDist = addVar("obstExtDistance", 0.7, "d",
                          "(rw) Search distance for obstacles away from passable intervals");
  var.grpMaxDist = addVar("obstGrpMaxDistance", 7.0, "d",
                              "(rw) [m] max pose distance in one obstacle group");
  var.grpMaxTime = addVar("obstGrpMaxTime", 7.0, "d",
                              "(rw) [sec] max pose time span in one group");
  var.obstMerge = addVar("obstMerge", 1.0, "d",
                            "(rw) Should obstacles be merged over scans based on combine distance specified above");
  var.obstCogMerge = addVar("obstCogMerge", 1.0, "d",
                            "(rw) Should obstacles be merged if COG is embedded in another obstacle and it do not extend more than mergeCogLimit outside the other obstacle."
                            " Relevant only if obstMerge is false.");
  var.cogMergeLimit = addVar("cogMergeLimit", 2.0, "d", "(rw) Allowed limit that a COG (Center Of Gravity) embedded obstacle may extend outside another obstacle to allow merge.");
  var.obstSingleMargin = addVar("singleLimit", 0.2, "d", "(rw) Distance limit for merge of single point obstacles");
  var.obstSingleMerge = addVar("singleMerge", 1.0, "d", "(rw) should simgle point obstacles merge using the singleDistance limit (not relevant if obstMerge in general with a larget limit)");
  var.updateTime = addVar("updTime", 0.0, "t",
                              "(ro) Time (tod) of last obstacle update");
  var.grps = addVar("obstGrps", 0.0, "d", "(r) number of obstacle groups in pool");
  var.group = addVar("obstGroup", 0.0, "d", "(r) newest obstacle group");
  var.fixedObsts = addVar("fixedObsts", 0.0, "d", "(r) Number of fixed (mapped) obstacles from mapbase.");
  addMethodV("updateMappedObst", "", "Get a fresh set of nearby fixed obstacles from mapbase, and save in obstacle pool");
}

////////////////////////////////////////////////////////

bool UResObstacle::methodCallV(const char * name, const char * paramOrder,
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
  if ((strcasecmp(name, "updateMappedObst") == 0) and (strlen(paramOrder) == 0))
  {
    if (updateMappedObstacles())
      value = 1.0;
    else
      value = 2.0;
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

////////////////////////////////////////////////////////

// bool UResObstacle::isOutdoorContext()
// {
//   bool result = true; // defalt
//   result = getValueBool(idxOutdoorContext);
//   return result;
// }

////////////////////////////////////////////////////////

void UResObstacle::getObstacleGroupSettings(UObstacleGroupLaser * og)
{
  double v;
  bool outdoor;
  //
  // get newest obstacle group
  outdoor = var.outdoorContext->getBool();
  setNewGrpDist(var.grpMaxDist->getValued());
  setNewGrpTime(var.grpMaxTime->getValued());
  if (og != NULL)
  {
    if (outdoor)
      v = var.outdoorCombineDist->getValued();
    else
      v = var.indoorCombineDist->getValued();
    og->setMergeDistance(v, var.obstMerge->getBool(),
                         var.cogMergeLimit->getDouble(),
                         var.obstCogMerge->getBool(),
                         var.obstSingleMargin->getDouble(),
                         var.obstSingleMerge->getBool());
  }
}

///////////////////////////////////////////

const char * UResObstacle::snprint(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  int i, n, m;
  char * p1;
  UObstacleGroupLaser * og;
  //
  snprintf(buff, buffCnt, "%s groupsCnt=%d\n", preString, groupsCnt);
  n = strlen(buff);
  p1 = &buff[n];
  m = mini(groupsCnt, 4);
  for (i = 0; i < m; i++)
  {
    og = getGroup(i);
    og->print("   ", p1, buffCnt - n, false);
    n += strlen(p1);
    p1 = &buff[n];
  }
  if (m < groupsCnt)
    snprintf(p1, buffCnt - n, "    ...\n");
  return buff;
}

///////////////////////////////////////////

void UResObstacle::obstDataUpdated(UTime poseTime)
{
  //printf("www\n");
  var.updateTime->setTime(poseTime);
  var.grps->setInt(groupsCnt);
  var.group->setInt(newest);
}

//////////////////////////////////////////////////////////////

bool UResObstacle::updateMappedObstacles()
{
  UPoseTime odoPoseOrigin, mapPose, odoPose;
  int n;
  bool isOK = false;
  UVariable vpo1, vpo2;
  UVariable * vpar[2] = {&vpo1, &vpo2};;
  UResPoseHist * mapPoseHist;
  //
  // get mapped obstacles in odometry coordinates
  mapPoseHist = (UResPoseHist*) getStaticResource("mapPose", false);
  if (mapPoseHist != NULL)
  { // use odo pose origin
    mapPose = mapPoseHist->getNewest();
    odoPoseOrigin = mapPoseHist->getOdoPoseOrigin();
    n = 1;
    vpo1.setPose(&mapPose);
    vpo2.setPose(&odoPoseOrigin);
    fixeds->clear();
    // get near obstacles in odometry coordinate system
    isOK = callGlobalV("mapobst.getobst", "cc", vpar, (UDataBase**)&fixeds, &n);
    var.fixedObsts->setInt(fixeds->getObstsCnt());
  }
  return isOK;
}
