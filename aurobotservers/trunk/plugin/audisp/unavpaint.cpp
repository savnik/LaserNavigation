/***************************************************************************
 *   Copyright (C) 2007 by Christian Andersen   *
 *   jca@oersted.dtu.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef OPENCV2
#else
#include <opencv/cv.h>
//#include <opencv/highgui.h>
#endif

#include <ugen4/ucommon.h>
#include <ugen4/u3d.h>
#include <urob4/uresbase.h>
#include <urob4/uresposehist.h>
#include <urob4/uimagepool.h>
#include <urob4/uvarcalc.h>
#include <umap4/umanseq.h>
#include <umap4/umanarc.h>
#include <umap4/umanline.h>

//#include "ulaserdataset.h"
#include <uobstaclehist.h>
#include <uclientcamifpath.h>
#include <ufeaturepool.h>
#include <ureslaserifroad.h>
#include <ureslaserifobst.h>
#include <ureslaserifscan.h>
#include <ureslaserifsf.h>
#include <uresnavifman.h>
#include <urescamifcam.h>
#include <uclientcamifgmk.h>
#include <urespoly.h>

#include "unavpaint.h"


UNavPaint::UNavPaint()
{
  img = NULL;
  obsts = NULL;
  resObst = NULL;
  road = NULL;
  varRoot = NULL;
  imgPool = NULL;
  poseOdo = NULL;
  camPath = NULL;
  laserData = NULL;
  resLaserData = NULL;
  laserFeatures = NULL;
  camGmk = NULL;
  camCam = NULL;
  resPoly = NULL;
  // default paint setting
  maxRange = 10.0;
  robotPose.set(1.5, 0, 0);
  paintBold = false;
//  paintPlan = true;
//  paintPathAll = true;
  paintPathLinesCnt = 1;
  paintPathMidPoses = true;
  paintPoseHistCnt = 20;
  paintPoseHistVecCnt = -1;
  paintPoseHistVecLng = 50;
  paintScanHistCnt = 5;
  paintVisPolyCnt = 1;
  paintGridSize = 1.0;
  paintGridOdo = true;
  paintRoadHistCnt = 100;
  paintRobot = 0;
  rangeRingCnt = 8;
  paintObstCnt = 1;
  paintVar = true;
//  paintStructs[0] = "laserif";
//  paintStructs[1] = "laserScan";
  paintStructsCnt = 0;
  paintPathSupportLines = true;
  paintPathMidPoses = true;
  paintCam = true;
  paintGmk = true;
  paintPoseRef = 0;
  paintPoly = true;
  paintPolyNameCnt = 5;
  paintOdoPose = true;
  paintMapPose = true;
  paintUtmPose = true;
  paintPolyHide[0] = '\0';
  paintPolyShow[0] = '\0';
  paintIntervalLines = false;
  paintRoadAll = false;
  paintCurves = false;
  poseOdo = NULL;
  poseUtm = NULL;
  poseMap = NULL;
  navMan = NULL;
}

//////////////////////////////

UNavPaint::~UNavPaint()
{
}

//////////////////////////////

bool UNavPaint::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // odometry pose
    if (remove)
      poseOdo = NULL;
    else if (poseOdo != resource)
      poseOdo = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResPoseHist::getUtmPoseID()))
  { // in UTM coordinates
    if (remove)
      poseUtm = NULL;
    else if (poseUtm != resource)
      poseUtm = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResPoseHist::getMapPoseID()))
  { // im Map coordinates
    if (remove)
      poseMap = NULL;
    else if (poseMap != resource)
      poseMap = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResVarPool::getResClassID()))
  { // delete any local
    if (remove)
      varRoot = NULL;
    else if (varRoot != resource)
      varRoot = (UResVarPool *)resource;
    else
      result = false;
  }
  else if (resource->isA(UImagePool::getResClassID()))
  { // delete any local
    if (remove)
      imgPool = NULL;
    else if (imgPool != resource)
      imgPool = (UImagePool *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResCamIfPath::getResClassID()))
  { // delete any local
    if (remove)
      camPath = NULL;
    else if (camPath != resource)
      camPath = (UResCamIfPath *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResLaserIfRoad::getResClassID()))
  { // delete any local
    if (remove)
      road = NULL;
    else if (road != resource)
      road = (UResLaserIfRoad *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResLaserIfObst::getResClassID()))
  { // delete any local
    if (remove)
      resObst = NULL;
    else if (resObst != resource)
      resObst = (UResLaserIfObst *)resource;
    else
      result = false;
    if (result)
      obsts = resObst;
  }
  else if (resource->isA(UResLaserIfScan::getResClassID()))
  { // delete any local
    if (remove)
      resLaserData = NULL;
    else if (resLaserData != resource)
      resLaserData = (UResLaserIfScan *)resource;
    else
      result = false;
    if (result)
    {
      if (resLaserData != NULL)
        laserData = resLaserData->getScanHist();
      else
        laserData = NULL;
    }
  }
  else if (resource->isA(UResLaserIfSf::getResClassID()))
  { // delete any local
    if (remove)
      laserFeatures = NULL;
    else if (laserFeatures != resource)
      laserFeatures = (UResLaserIfSf *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResNavIfMan::getResClassID()))
  { // delete any local
    if (remove)
      navMan = NULL;
    else if (navMan != resource)
      navMan = (UResNavIfMan *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResCamIfCam::getResClassID()))
  { // delete any local
    if (remove)
      camCam = NULL;
    else if (camCam != resource)
      camCam = (UResCamIfCam *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResCamIfGmk::getResClassID()))
  { // delete any local
    if (remove)
      camGmk = NULL;
    else if (camGmk != resource)
      camGmk = (UResCamIfGmk *)resource;
    else
      result = false;
  }
  else if (resource->isA("poly"))
  { // delete any local
    if (remove)
      resPoly = NULL;
    else if (resPoly != resource)
      resPoly = (UResPoly *)resource;
    else
      result = false;
  }
  else
    result = false;
  return result;
}

//////////////////////////////

void UNavPaint::paint(int imgPoolNum)
{
  bool result;
  int i, n, m;
  CvPoint p1;
  const int w = 800;
  const int h = 600;
  //CvScalar white = CV_RGB(255, 255, 255);
  CvScalar black = CV_RGB(0, 0, 0);
  //CvScalar red = CV_RGB(255, 0, 0);
  CvScalar redMag = CV_RGB(180, 0, 100);
  CvScalar orange = CV_RGB(180, 100, 0);
  //CvScalar blue = CV_RGB(0, 0, 255);
  //CvScalar lblue = CV_RGB(0, 0, 100);
  CvScalar green = CV_RGB(0, 155, 0);
  CvScalar yellow = CV_RGB(175, 175, 0);
  CvScalar magenta = CV_RGB(155, 0, 155);
  //CvScalar cyan = CV_RGB(0, 155, 155);
  CvFont font;
  ULaserDataSet * histScan;
  ULaserDataSet * scan = NULL;
  const int MSL = 100;
  char s[MSL];
  int pz = 3; // point size (radius in pixels)
  //int lw; // line width
  UPoseTime lastPose;
  UPoseTime seenFromPose;
//  ULaserPathResult * usedPath = NULL;
  UProbPoly * poly;
//  UMatrix4 mLtoR;
  UPosition posr;
//  double poseVel; // latest velocity
  UPosRot sensorPose;
  UClientManSeq ** pMans, *bestPath;
  UPoseTVQ pvt;
  //const int MTL = 30;
  //char text[MTL];
  // debug
#ifdef USE_LOCAL
  printf("UNavPaint::paint: USE_LOCAL is defined\n");
#endif
  // debug end
  // printf("scale %.2fm; ppm=%.3f; pr=%dx,%dy; pose = %.2fx,%.2fy,%.4fh\n", maxRange, ppm, pr.x, pr.y, robotPose.x, robotPose.y, robotPose.h);
  //
  if (paintBold)
  {
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
                 1.0, 1.0, 0.0, 2, 8);
    redMag = CV_RGB(100, 0, 40);
    orange = CV_RGB(100, 40, 0);
    green = CV_RGB(0, 120, 0);
    yellow = CV_RGB(75, 75, 0);
    magenta = CV_RGB(75, 0, 75);
  }
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);
  if (maxRange > 30)
    pz = 1;
  else if (maxRange > 20)
    pz = 2;
  if (paintBold)
  {
    pz *= 2;
    //lw = 2;
  }
  //
  if (imgPool != NULL)
  {
    img = imgPool->getImage(imgPoolNum, true, h, w, 3, 8);
    if (img != NULL)
    {
      if ((h != (int)img->height()) or (w != (int)img->width()))
        img->setSize(h, w, 3, 8, "RGB");
      img->setName("topView");
      img->clear(255);
    }
  }
  result = (img != NULL);
/*  if (result and (cvGetWindowHandle("topView") == NULL))
  { // create a opneCV window for the data
    cvNamedWindow("topView", CV_WINDOW_AUTOSIZE);
    result = (cvGetWindowHandle("topView") != NULL);
  }*/
  // update new laser measurements
  // laser data comes before other data
  if (result)
  { //
    if (poseOdo != NULL)
      pvt = poseOdo->getNewest();
    else if (poseUtm != NULL)
      pvt = poseUtm->getNewest();
    else if (poseMap != NULL)
      pvt = poseMap->getNewest();
      // get perspective reference
    switch(paintPoseRef)
    {
      case 0:
        if (poseOdo != NULL)
          lastPose = poseOdo->getPoseAtTime(pvt.t);
        break;
      case 1:
        if (poseUtm != NULL)
          lastPose = poseUtm->getPoseAtTime(pvt.t);
        break;
      case 2:
        if (poseMap != NULL)
          lastPose = poseMap->getPoseAtTime(pvt.t);
        break;
      default:
        break;
    }
    seenFromPose = lastPose;
    // set scale - pixels per meter
    ppm = img->height() / maxRange;
    // calculate robot position on display in pixels
    pr.y = img->height() - roundi(ppm * robotPose.x);
    pr.x = img->width()/2  + roundi(ppm * robotPose.y);
    //
    // seenFromPose.print("seen from pose");
    // printf("scale %.2fm; ppm=%.3f; pr=%dx,%dy; pose = %.2fx,%.2fy,%.4fh\n", maxRange, ppm, pr.x, pr.y, robotPose.x, robotPose.y, robotPose.h);
    //
    if (paintGridOdo)
    { // adjust grid scale
      if (maxRange / paintGridSize > 150)
        paintGridSize *= 10.0;
      if (maxRange / paintGridSize < 1.5)
        paintGridSize /= 10.0;
      paintOdoGrid(seenFromPose, paintGridSize, false);
      paintOdoGrid(seenFromPose, paintGridSize * 10.0, true);
    }
    // robot position
    if (laserData != NULL)
    {
      scan = laserData->getNewest();
      if (scan != NULL)
      {
        sensorPose = *scan->getSensorPose();
        paintRangeRings(&sensorPose, rangeRingCnt);
      }
      else
        sensorPose.set(0.22, 0.0, 0.4, 0.0, 0.0, 0.0);
    }
    else
      scan = NULL;
    // paint statistics data window
    if (scan != NULL)
    {
      if (scan->isStatValid() and paintCurves)
        paintScanStatData(scan, false, true, false);
    }
    //
    // paint poly items (mission plan and other marks)
    if (paintPoly)
      paintPolyItems(seenFromPose);
    //
    // obstacles
    if (result and (obsts != NULL))
    {
      obsts->ogLock.lock();
      n = mini(obsts->getGroupsCnt(), paintObstCnt);
      for (i = n - 1; i >= 0; i--)
        // paint from oldest to newest
        paintObstGrp(obsts->getGroupNewest(i), seenFromPose,
                     obsts->getGroupNewest() + i);
      if (paintObstCnt > 0 and obsts->getGroupFixed() != NULL)
        // paint also fixed obstacles
        paintObstGrp(obsts->getGroupFixed(), seenFromPose, -1);
      obsts->ogLock.unlock();
    }
  }
  if (scan != NULL)
  { // paint history scan data
    n = mini(paintScanHistCnt, laserData->getScansCnt());
    if ((seenFromPose.t - scan->getScanTime()) < 0.1)
      m = 1; // new scan is aviailable, so first scan is not history
    else
      m = 0; // all scans are history
    for (i = m; i < n; i++)
    {
      histScan = laserData->getScan(i);
      if (histScan == NULL)
        break;
      paintHistScan(histScan, seenFromPose);
    }
    // paint newest scan
    if (n > 0 and m > 0)
      paintScanNewest(scan);
  }
    //
  if (paintIntervalLines and (scan != NULL))
  { // paint other new stuff
    if (scan->getPisCnt() > 0)
      paintPis(scan, seenFromPose, 0);
  }
  // paint robot history path
  paintPoseHistLines(seenFromPose);
  // road lines
  if (result and (paintRoadHistCnt > 0) and (road != NULL))
  {
    if (paintRoadAll)
      // paint all lines in gray
      for (n = 0; n < road->getRoadLinesCnt(); n++)
        paintRoadLine(road->getRoadLine(n), seenFromPose, true);
    // paint just the current road
    paintRoadLine(road->getRoadCurrent(0), seenFromPose, false); // left
    paintRoadLine(road->getRoadCurrent(1), seenFromPose, false); // center
    paintRoadLine(road->getRoadCurrent(2), seenFromPose, false); // top
  }
  // navigation planned path
  if (result and (navMan != NULL) and (paintPathLinesCnt > 0))
  { // path lines and alternative too
    if (navMan->getMansCnt() > 0)
    {
      pMans = navMan->getMans();
      bestPath = NULL;
      for (i = 0; i < navMan->getMansCnt(); i++)
      { // paint road edges and drive path
        if (pMans[i] != NULL)
        {
          if (pMans[i]->isBest())
            bestPath = pMans[i];
          else if (paintPathLinesCnt > 1)
            // paint unused first
            paintManData(pMans[i], i, seenFromPose);
        }
      }
      if (bestPath != NULL)
        // paint used path on top
        paintManData(bestPath, 0, seenFromPose);
    }
  }
  // vision polygon
  if (result and (camPath != NULL))
  { // paint vision polygon data
    n = mini(paintVisPolyCnt, camPath->getVisDataCnt());
    if (n > 0)
    { // paint old ones first
      for (i = n - 1; i >= 0; i--)
      { // paint first history
        poly = camPath->getVisData(i)->getPoly();
        paintFreePoly(poly, seenFromPose, i > 0);
      }
    }
  }
  // paint planner and variables
  if (result and (paintVar))
      paintVarDataText(paintVar);
  // paint interval lines and scan number
  p1.x = img->width()/2 + 30;
  p1.y = 20 + 13;
  if (paintCurves)
    p1.y += img->height()/5;
  //
/*  if (result and navPath != NULL)
  { // path lines and alternative too
    for (i = 0; i < pathsCnt; i++)
    { // paint road edges and drive path
      if (paths[i] != NULL)
      {
        if (paths[i]->isPathUsed())
          usedPath = paths[i];
        else
          // paint unused first
          paintPathData(paths[i], i);
      }
    }
    if (usedPath != NULL)
      // paint used path on top
      paintPathData(usedPath, 0);
  }*/
  if (result and (laserFeatures != NULL))
    paintFeatures(laserFeatures->getSfPool(), seenFromPose);
  //
  if (scan != NULL)
  { // paint scannumber - top right corner
    if (paintBold)
    {
      p1.x = img->width() - 140;
      p1.y = 24;
    }
    else
    {
      p1.x = img->width() - 80;
      p1.y = 14;
    }
    snprintf(s, MSL, "#%u", scan->getSerial());
    cvPutText(img->cvArr(), s, p1, &font, black);
  }
  // paint robot
  if (paintRobot == 0)
    paintRobotMmr(sensorPose.getX()); // parameter is distance from origin to front
  else if (paintRobot == 2)
    paintRobotHako(sensorPose.getX()); // parameter is distance from origin to front
  else if (paintRobot == 3)
    paintRobotIrobot(sensorPose.getX()); // parameter is distance from origin to front
  else if (paintRobot == 4)
    paintRobotGuidebot(sensorPose.getX()); // parameter is distance from origin to front
  else
    paintRobotSmr(sensorPose.getX()); // parameter is distance from origin to front
  //
  // paint pose text
  if (result)
    paintOdoDataText();
  if (paintCam)
    paintCams(seenFromPose);
  if (paintGmk)
    paintGmks(seenFromPose);

  //
  if (img != NULL)
  { // send to display handler
    img->used = false;
    if (pvt.t.getSec() < 10)
      // no posetime
      img->imgTime.now();
    else
      // use pose time ad update itme
      img->imgTime = pvt.t;
    if (scan != NULL)
    {
      snprintf(img97name, IMG97SIZE, "topView_%07d_", scan->getSerial());
      img->setName(img97name);
    }
    img->imageNumber++;
    img->imgUpdated();
  }
  else
    printf("UClientFuncLaserGui::gotNewData no image handle!\n");
}

///////////////////////////////////////////////////////////////

////////////////////////////////////////////////////

bool UNavPaint::paintOdoGrid(UPose odoPose, double stepSize, bool bold)
{
  CvPoint p1, p2;
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lcyan = CV_RGB(180, 240 , 240);
  CvScalar lblue = CV_RGB(180, 180 , 250);
  CvScalar black = CV_RGB(0, 0, 0);
  int i;
  bool result;
  int mr = int(maxRange * 1.2);
  int steps = roundi(mr / stepSize);
  //
  UPosition posG1, posG2, posR1, posR2;
  //
  if (bold)
  {
    lyellow = CV_RGB(140, 140 , 80);
    lcyan = CV_RGB(80, 140 , 140);
    lblue = CV_RGB(10, 10 , 150);
    black = CV_RGB(0, 0, 0);
  }
  //
  result = (img != NULL);
  if (result)
  { // paint odometry grid
    posG1.set(floor(odoPose.x / stepSize) * stepSize - double(steps) * stepSize,
              floor(odoPose.y / stepSize) * stepSize - double(steps) * stepSize, 0.0);
    posG2 = posG1;
    posG2.x = posG1.x + double((steps + 1) * 2) * stepSize;
    for (i = -steps; i <= steps; i++)
    { // paint odo-grid (Y)
      posR1 = odoPose.getMapToPose(posG1);
      posR2 = odoPose.getMapToPose(posG2);
      p1.y = pr.y - roundi(posR1.x * ppm);
      p1.x = pr.x - roundi(posR1.y * ppm);
      p2.y = pr.y - roundi(posR2.x * ppm);
      p2.x = pr.x - roundi(posR2.y * ppm);
      if (roundi(posG1.y) == 0)
        cvLine(img->cvArr(), p1, p2, lblue, 1, 4, 0);
      else
        cvLine(img->cvArr(), p1, p2, lyellow, 1, 4, 0);
      posG1.y += stepSize;
      posG2.y += stepSize;
    }
    //
    posG1.set(floor(odoPose.x / stepSize) * stepSize - double(steps) * stepSize,
              floor(odoPose.y / stepSize) * stepSize - double(steps) * stepSize, 0.0);
    posG2 = posG1;
    posG2.y = posG1.y + double((steps + 1) * 2) * stepSize;
    for (i = -steps; i <= steps; i++)
    { // paint odo-grid (X)
      posR1 = odoPose.getMapToPose(posG1);
      posR2 = odoPose.getMapToPose(posG2);
      p1.y = pr.y - roundi(posR1.x * ppm);
      p1.x = pr.x - roundi(posR1.y * ppm);
      p2.y = pr.y - roundi(posR2.x * ppm);
      p2.x = pr.x - roundi(posR2.y * ppm);
      if (roundi(posG1.x) == 0)
        cvLine(img->cvArr(), p1, p2, lblue, 1, 4, 0);
      else
        cvLine(img->cvArr(), p1, p2, lcyan, 1, 4, 0);
      posG1.x += stepSize;
      posG2.x += stepSize;
    }
  }
  return result;
}

/////////////////////////////////////////////////

bool UNavPaint::paintRangeRings(UPosRot * sensorPos, int ringCnt)
{
  CvPoint p1;
  CvFont font;
  CvScalar lblue = CV_RGB(200, 200 , 250);
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lblack = CV_RGB(100, 100, 100);
  const int MTL = 30;
  char text[MTL];
  int i;
  bool result;
  //UPosRot devPos;
  //const int RING_CNT = 8; // number of range rings (each 1 meter)
  //
  if (paintBold)
  {
    lblue = CV_RGB(50, 50 , 250);
    lyellow = CV_RGB(200, 200 , 0);
    lblack = CV_RGB(10, 10, 10);
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
                 1.0, 1.0, 0.0, 1, 8);
  }
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);

  result = (img != NULL) and (laserData != NULL);
  if (result)
  { // paint laser range rings
    // get device position
    //devPos = laserData->getDevPos();
    // convert to pixels
    p1.y = pr.y - roundi(sensorPos->getX() * ppm);
    p1.x = pr.x - roundi(sensorPos->getY() * ppm);
    // paint laser position
    cvCircle(img->cvArr(), p1, 3, lblue, 1, 4, 0);
    // paint range rings
    for (i = 1; i <= rangeRingCnt; i++)
    {
       //cvEllipse( CvArr* img, CvPoint center, CvSize axes, double angle,
       //         double start_angle, double end_angle, CvScalar color,
       //         int thickness=1, int line_type=8, int shift=0 );
      //printf("UNavPaint::rangerings: size(%d, %d), ppm=%.2f, i=%d\n",  roundi(ppm * i), roundi(ppm * i), ppm, i);

#ifdef OPENCV2
      cvEllipse(img->cvArr(), p1,
                cvSize(roundi(ppm * i), roundi(ppm * i)),
                0.0, 0.0, -180.0, lblue, 1, 4, 0);
//       cvEllipse(img->cvArr(), p1,
//                 cvSize(roundi(ppm * i), roundi(ppm * i)),
//                 0.0, 180.0, 0.0, lblue, 1, 4, 0);
#else
      cvEllipse(img->cvArr(), p1,
                cvSize(roundi(ppm * i), roundi(ppm * i)),
                90.0, 180.0, 0.0, lyellow, 1, 4, 0);
//       cvEllipse(img->cvArr(), p1,
//                 cvSize(roundi(ppm * i), roundi(ppm * i)),
//                 0.0, 180.0, 0.0, lblue, 1, 4, 0);
#endif
      //cvCircle(img->cvArr(), p1, roundi(ppm * i), lblue, 1, 4, 0);
      if (maxRange < 11)
      {
        snprintf(text, MTL, "%dm", i);
        cvPutText(img->cvArr(), text, cvPoint(p1.x + roundi(ppm * i)-15, p1.y+15), &font, lblack);
      }
      else if (maxRange < 25)
      {
        snprintf(text, MTL, "%d", i);
        cvPutText(img->cvArr(), text, cvPoint(p1.x + roundi(ppm * i)-5, p1.y+15), &font, lblack);
      }
    }
  }
  return result;
}

///////////////////////////////////////////////////////////////

bool UNavPaint::paintScanStatData(ULaserDataSet * scan,
                                            bool paintVar, //bool paintEdge, bool paintCurv,
                                            bool paintVarL, bool paintTilt)
{
  CvPoint p1, p2;
  CvPoint p1VarL, p1VarI, p1VarLC, p1Tilt, p1Curv, p1X;
  CvPoint topLeft;
  CvPoint botRight;
  CvFont font;
  CvScalar lBlue = CV_RGB(150, 150 , 255);
  //CvScalar dBlue = CV_RGB(20, 20 , 155);
  //CvScalar blue = CV_RGB(0, 0 , 200);
  //CvScalar dMagenta = CV_RGB(245, 50 , 245);
  //CvScalar dCyan = CV_RGB(90, 195 , 195);
  CvScalar black = CV_RGB(0, 0, 0);
  CvScalar red = CV_RGB(255, 0, 0);
  //CvScalar redMag = CV_RGB(180, 0, 100);
  //CvScalar orange = CV_RGB(180, 100, 0);
  CvScalar green = CV_RGB(0, 155, 0);
  CvScalar yellow = CV_RGB(175, 175, 0);
  //CvScalar brown = CV_RGB(100, 100, 0);
  //CvScalar magenta = CV_RGB(155, 0, 155);
  bool result;
  int i, n, h, w;
  UClientLaserData * lp;
  //const double maxVar = 0.02;  // top limit
  const double maxVarLR = 0.2;  // top limit of SD of variance to the left
//  const double maxEdge = 0.4;
//  const double maxCurv = M_PI/2.0;
  const double maxTilt = M_PI/2.0;
  const double maxX = 4.0;
  UPosition pos;
  UPosition posZero(0.0, 0.0, 0.0);
  double cosLaserTilt;
  int txtLineH;
  CvPoint piL, piR;
  //
  if (paintBold)
  {
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.4, 0.0, 1, 8);
    txtLineH = 15;
    topLeft.y = 26 + 5;
  }
  else
  {
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);
    txtLineH = 12;
    topLeft.y = 13 + 5;
  }
  result = (img != NULL);
  if (result)
  { // get position of varianve curve
    n = (img->width() - 10)/(2 * scan->getCount());
    w = scan->getCount() * n;
    h = img->height() / 5;
    botRight.x = img->width() - 5;
    topLeft.x = botRight.x - w;
    botRight.y = topLeft.y + h;
    p1VarL = botRight;
    p1VarI = botRight;
    p1VarLC = botRight;
    p1Tilt = botRight;
    p1Tilt.y -= h/2;
    p1Curv = p1Tilt;
    p1X = botRight;
    // Paint coordinate system
    cvRectangle(img->cvArr(), topLeft, botRight, lBlue);
    p1.y = topLeft.y;
    p2.y = botRight.y;
    for (i = 1; i < 4; i++)
    {
      p1.x = topLeft.x + 45 * i * n;
      p2.x = p1.x;
      cvLine(img->cvArr(), p1, p2, lBlue);
    }
    // paint zero line
    p1.x = topLeft.x;
    p2.x = botRight.x;
    p1.y = (topLeft.y + botRight.y)/2;
    p2.y = p1.y;
    cvLine(img->cvArr(), p1, p2, lBlue);
    p1.y = (topLeft.y + botRight.y)/2 + h/4;
    p2.y = p1.y;
    cvLine(img->cvArr(), p1, p2, lBlue);
    p2.x -= 90;
    p1.y = (topLeft.y + botRight.y)/2 - h/4;
    p2.y = p1.y;
    cvLine(img->cvArr(), p1, p2, lBlue);
    // paint legend
    p1.x = botRight.x - 75;
    p1.y = 16 + 13;
    if (paintVar)
    {
      cvPutText(img->cvArr(), "-var bad", p1, &font, yellow);
      p1.y += txtLineH;
      cvPutText(img->cvArr(), "-var OK ", p1, &font, green);
      p1.y += txtLineH;
    }
/*    if (paintEdge)
    {
    cvPutText(img->cvArr(), "-edge", p1, &font, dMagenta);
    p1.y += 12;
  }
    if (paintCurv)
    {
    cvPutText(img->cvArr(), "-curv", p1, &font, dMagenta);
    p1.y += 12;
  }*/
    if (paintVarL)
    {
      p1.y = topLeft.y + txtLineH;
      cvPutText(img->cvArr(), "-sdLeft", p1, &font, red);
/*      cvPutText(img->cvArr(), "-sdLC", p1, &font, brown);
      p1.y += 12;*/
      p2.x = topLeft.x - 37;
      if (paintBold)
        p2.x -= 5;
      p2.y = topLeft.y + 7;
      cvPutText(img->cvArr(), " cm", p2, &font, red);
      p2.y += h/4;
      cvPutText(img->cvArr(), "  15", p2, &font, red);
      p2.y += h/4;
      cvPutText(img->cvArr(), "  10", p2, &font, red);
      p2.y += h/4;
      cvPutText(img->cvArr(), "   5", p2, &font, red);
      p2.y += h/4;
      cvPutText(img->cvArr(), "   0", p2, &font, red);
    }
    if (paintTilt)
    {
      p1.y += txtLineH;
      cvPutText(img->cvArr(), "-tilt", p1, &font, red);
    }
    if (true)
    {
      p1.y += txtLineH;
      cvPutText(img->cvArr(), "-X", p1, &font, black);
    }
    // paint curve
    lp = scan->getData();
    p2.y = (topLeft.y + botRight.y)/2;
    cosLaserTilt = cos(scan->getLaserTilt());
    piL.x = -1;
    piL.y = botRight.y + 1;
    piR.x = -1;
    piR.y = piL.y - h - 1;
    for (i = 0; i < scan->getCount(); i++)
    { // get position of this angle
      p1.x = botRight.x -
          roundi((90.0 + lp->getAngle() * 180.0 / M_PI) * double(n));
      if (true)
      { // paint scan x-value
        pos = lp->getPosition(posZero, cosLaserTilt);
        p1.y = botRight.y - mini(h, roundi(double(h) *
            pos.x / maxX));
        cvLine(img->cvArr(), p1X, p1, black);
        p1X = p1;
      }

      if (paintVarL)
      { // "integrated" - running average - varianve to the left (from right).
/*        p1.y = botRight.y - mini(h, roundi(double(h) *
        lp->getVarI() / maxVarLR));
        cvLine(img->cvArr(), p1VarI, p1, dBlue);
        p1VarI = p1;*/
        // normal varianve to the left over a constant width
/*        p1.y = botRight.y - mini(h, roundi(double(h) *
        lp->getSdLC() / maxVarLR));
        cvLine(img->cvArr(), p1VarLC, p1, brown);
        p1VarLC = p1;*/
        // normal varianve to the left over a constant width
        p1.y = botRight.y - mini(h, roundi(double(h) *
            lp->getSdL() / maxVarLR));
        cvLine(img->cvArr(), p1VarL, p1, red);
        p1VarL = p1;
        if (lp->getFlag() >= 10)
        {
          if (lp->getFlag() >= 30)
          { // start and stop at same position - just paint line
            piL.x = p1.x;
            piR.x = p1.x;
            cvLine(img->cvArr(), piL, piR, green);
            piL.x = -1;
            piR.x = -1;
          }
          else if (piR.x < 0)
            piR.x = p1.x;
          else if (piL.x < 0)
          { // both ends available - paint square
            piL.x = p1.x;
            cvRectangle(img->cvArr(), piL, piR, green);
            piL.x = -1;
            piR.x = -1;
          }
        }
      }
      if (paintTilt)
      {
        p1.y = p2.y - maxi(-h/2, mini(h/2, roundi(double(h/2) * lp->getTilt() / maxTilt)));
        if (not lp->isValid())
          cvLine(img->cvArr(), p1Tilt, p1, red);
        else
          cvLine(img->cvArr(), p1Tilt, p1, green);
        p1Tilt = p1;
      }
      lp++;
    }
    //
    // print odometer position
    //snprintf(text, MTL, "odoPos %6.2fx, %6.2fy, %6.1fdeg",
    //           odoPose.x, odoPose.y, odoPose.h * 180.0 / M_PI);
    //cvPutText(img->cvArr(), text, p1, &font, black);
    // print odometer time
  }
  return result;
}

////////////////////////////////////////////////////

void UNavPaint::paintHistScan(ULaserDataSet * scan,
                              UPose seenFromPose)
{
  int i;
  CvPoint p1;
  UPosition posm, posr, pos;
  UClientLaserData * pd;
  //CvScalar red = CV_RGB(255, 0, 0);
  //CvScalar lred = CV_RGB(180, 80, 80);
  CvScalar lgreen = CV_RGB(90, 210, 70);
  //CvScalar lyellow = CV_RGB(180, 80, 80);
  //CvScalar lmagenta = CV_RGB(155, 0, 155);
  UPixel pixGray(100, 100, 100);
  UPixel pixGreen(15, 210, 15);
  UPose poseScan;
  int pz = 2; // radius of circles
  int lw = 1;
  UMatrix4 mLtoR;
  UResPoseHist * por;
  UPose systemOrigin;
  //
  if (maxRange > 15.0)
    pz = 1;
  if (paintBold)
  {
    lw = 2;
  }
  //
  switch (paintPoseRef)
  {
    case 0: por = poseOdo; break;
    case 1: por = poseUtm; break;
    case 2: por = poseMap; break;
    default: por = NULL; break;
  }
  if (por != NULL)
    poseScan = por->getPoseAtTime(scan->getScanTime());
  else
    // no way to get scanner pose at scantime, use pose provided by scan (odoPose)
    poseScan = scan->getPose();
  //
  // make a transformation matrix from laser coordinates to robot coordinates.
  mLtoR = scan->getSensorPose()->getRtoMMatrix();
  // get data reference of first measurement
  pd = scan->getData();
  for (i = 0; i < scan->getCount(); i++)
  {
    if (pd->isValid()) // or (pz > 1))
    { // get position in robot coordinates
      posr = pd->getPosition(&mLtoR);
      // get position in odo coordinates at scantime
      posm = poseScan.getPoseToMap(posr);
      // get position as seen from current position
      pos = seenFromPose.getMapToPose(posm);
      // and in pixels in image
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      //cvLine(img->cvArr(), p1, p2, cyan, 1, 8, 0);
      if (true) // (pz > 1)
      {
/*        if (pd->inZoneA())
          cvCircle(img->cvArr(), p1, pz, lred, lw, 8, 0);
        else if (pd->inZoneB())
          cvCircle(img->cvArr(), p1, pz, lyellow, lw, 8, 0);
        else if (pd->isDazzled())
          cvCircle(img->cvArr(), p1, pz, lmagenta, lw, 8, 0);
        else*/
          cvCircle(img->cvArr(), p1, pz, lgreen, lw, 8, 0);
      }
      else
      { // just one pixel (or 2)
        if (img->inRange(p1.y, p1.x))
        {
          if (pd->getFlag() > 0)
          {
            img->setPix(p1.y, p1.x, pixGray);
            if (lw >= 2 and (p1.x > 0))
              img->setPix(p1.y, p1.x-1, pixGray);
          }
          else
          {
            img->setPix(p1.y, p1.x, pixGreen);
            if (lw >= 2 and (p1.x > 0))
              img->setPix(p1.y, p1.x-1, pixGreen);
          }
        }
      }
    }
    pd++;
  }
}

/////////////////////////////////////////////////

void UNavPaint::paintScanNewest(ULaserDataSet * scan)
{
  UMatrix4 mLtoR;
  UClientLaserData * pd;
  CvPoint p2;
  CvScalar redMag = CV_RGB(180, 0, 100);
  //CvScalar orange = CV_RGB(180, 100, 0);
  CvScalar green = CV_RGB(0, 155, 0);
  //CvScalar yellow = CV_RGB(175, 175, 0);
  //CvScalar magenta = CV_RGB(155, 0, 155);
  int i;
  UPosition posr;
  int pz = 3;
  int lw = 1;
  //
  if (maxRange >= 20.0)
    pz = 2;
  if (paintBold)
    lw = 2;
  //
  pd = scan->getData();
  mLtoR = scan->getSensorPose()->getRtoMMatrix();
        // robot position
        //cosTilt = cos(scan->getLaserTilt());
  for (i = 0; i < scan->getCount(); i++)
  {
    posr = pd->getPosition(&mLtoR);
          //x = pd->getDistance() * cos(pd->getAngle() * cosTilt); // forward
          //y = pd->getDistance() * sin(pd->getAngle()); // right
    p2.x = pr.x - roundi(posr.y * ppm);
    p2.y = pr.y - roundi(posr.x * ppm);
          //cvLine(img->cvArr(), p1, p2, cyan, 1, 8, 0);
    if (false)
      paintCross(img, p2.x, p2.y, pz+1, redMag, lw);
            //cvCircle(img->cvArr(), p2, pz, redMag, 1, 8, 0);
/*    else if (pd->inZoneA())
      paintCross(img, p2.x, p2.y, pz+1, orange, lw);
            //cvCircle(img->cvArr(), p2, pz, orange, 1, 8, 0);
    else if (pd->inZoneB())
            //cvCircle(img->cvArr(), p2, pz, yellow, 1, 8, 0);
      paintCross(img, p2.x, p2.y, pz+1, yellow, lw);
    else if (pd->isDazzled())
      paintCross(img, p2.x, p2.y, pz+1, magenta, lw);*/
    else
      cvCircle(img->cvArr(), p2, pz, green, lw, 8, 0);
    pd++;
  }
}

///////////////////////////////////////

void UNavPaint::paintPoseHistLine(UPoseTime seenFromPose, UResPoseHist * path,
                                  int colidx, bool convert, UPose systemOrigin)
{
  int i, n;
  CvPoint p1, p2, pv;
  UPosition posm, posr, pos;
  CvScalar red = CV_RGB(255, 0, 0);
  CvScalar cya = CV_RGB(0, 155, 155);
  CvScalar mag = CV_RGB(155, 0, 155);
  UPoseTime pose, pose2;
  int lw = 1;
  int m;
  UPoseTime lastPose;
  //
  if (paintBold)
    lw = 2;
  if (colidx == 1)
    red = cya;
  else if (colidx == 2)
    red = mag;
  //
  n = mini(paintPoseHistCnt, path->getPosesCnt());
  m = paintPoseHistVecCnt;
  // get pixel position of start pose
  p2 = pr;
  for (i = 1; i < n; i++)
  { // get pose with this age - older and older
    pose = path->getPose(i);
    if (convert)
      // convert from other coordinate system
      pose = systemOrigin.getPoseToMapPose(pose);
    // robot pose for this update
    pos = seenFromPose.getMapToPose(pose.getPos());
    p1.y = pr.y - roundi(pos.x * ppm);
    p1.x = pr.x - roundi(pos.y * ppm);
    if (p1.x > -100 and p1.x < 3000 and
        p1.y > -100 and p1.y < 3000)
    { // paint if inside image only
      if (maxRange < 15.0)
        // paint also circle at each pose
        cvCircle(img->cvArr(), p1, 2, red, lw, 8, 0);
      // paint line
      cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);
      // paint also heading vector
      if (m > 0)
      {
        if (i % m == 0)
        {
          pose2.set(paintPoseHistVecLng/ppm, 0.0, 0.0);
          pose2 = pose.getPoseToMapPose(&pose2);
          pos = seenFromPose.getMapToPose(pose2.getPos());
          pv.y = pr.y - roundi(pos.x * ppm);
          pv.x = pr.x - roundi(pos.y * ppm);
          cvLine(img->cvArr(), p1, pv, red, lw, 8, 0);
          if (maxRange < 15.0)
          { // paint also circle at each pose
            cvCircle(img->cvArr(), pv, 4, red, lw, 8, 0);
            cvCircle(img->cvArr(), p1, 4, red, lw, 8, 0);
          }
        }
      }
      // save last painted pose position
      p2 = p1;
    }
  }
}

///////////////////////////////////////

void UNavPaint::paintRoadLine(URoadLineData * road, UPoseTime seenFromPose, bool inGray)
{ // paint road segment and poly.line
  int i, n, m;
  CvPoint p1, p2;
  UPosition posm, posr, pos;
  CvScalar red = CV_RGB(255, 0, 0);
  CvScalar green = CV_RGB(0, 255, 0);
  CvScalar yellow = CV_RGB(128, 128, 0);
  CvScalar gray = CV_RGB(98, 98, 98);
  CvScalar col;
  UPoseTime pose;
  int lw = 1;
  UPoseTime lastPose;
  //
  if (road != NULL)
  {
    if (paintBold)
      lw = 2;
    if (inGray)
      col = gray;
    else
      switch (road->edge)
      { // 0=left, 1=top, 2=right
        case 0: col = red; break;
        case 1: col = yellow; break;
        default: col = green; break;
      }
      //
    pos = seenFromPose.getMapToPose(road->line.pos);
    p1.y = pr.y - roundi(pos.x * ppm);
    p1.x = pr.x - roundi(pos.y * ppm);
    pos = seenFromPose.getMapToPose(road->line.getOtherEnd());
    p2.y = pr.y - roundi(pos.x * ppm);
    p2.x = pr.x - roundi(pos.y * ppm);
    cvLine(img->cvArr(), p1, p2, col, lw, 8, 0);
    // continue with polyline
    m = road->edgeLine.getPointsCnt();
    n = mini(paintRoadHistCnt, m);
    // get pixel position of start pose
    for (i = 0; i < n; i++)
    { // get pose with this age - older and older
      m--;
      p2 = p1; // save last pixel position
      // robot pose for this update
      pos = seenFromPose.getMapToPose(road->edgeLine.getPoint(m));
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      if (maxRange < 25.0)
        // paint also circle at each pose
        cvCircle(img->cvArr(), p1, 2, col, lw, 8, 0);
      // paint line
      cvLine(img->cvArr(), p1, p2, col, lw, 8, 0);
    }
  }
}

//////////////////////////////////////////////////////

void UNavPaint::paintObstGrp(UObstacleGroup *obst,
                             UPose seenFromPose, int grpIdx)
{
  int i, n;
  CvPoint p1, p2;
  UPosition pos;
  UPosition po1, po2;
  const CvScalar purpOdd   = CV_RGB(100, 50, 100);
  const CvScalar yellowOdd = CV_RGB(100, 100, 50);
  const CvScalar purp      = CV_RGB(190, 0, 190);
  const CvScalar yellow    = CV_RGB(190, 190, 0);
  const CvScalar gray      = CV_RGB(100, 100, 100);
  //
  CvScalar obsValid;
  CvScalar obsInvalid;
//  CvScalar hist = CV_RGB(150, 100, 100);
  CvScalar * col = &obsValid;
  CvFont font;
  int lw = 3;
  UObstacle * ob;
  UPose sysPose;
  bool convert;
  // obstacles are always in odometry coordinates.
  sysPose = getSystemOrigin(0, &convert);
  //
  if (paintBold)
    lw = 4;
  if (grpIdx == -1)
  { // fixed obstacles
    obsValid = gray;
    obsInvalid = gray;
  }
  else if (grpIdx % 2 == 0)
  {
    obsValid = purp;
    obsInvalid = yellow;
  }
  else
  {
    obsValid = purpOdd;
    obsInvalid = yellowOdd;
  }
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               1.0, 1.0, 0.0, 1, 8);
  //
  for (n = 0; n < obst->getObstsCnt(); n++)
  {
    ob = obst->getObstacle(n);
    // paint obstacle
    po1 = ob->getPoint(ob->getPointsCnt() - 1);
    if (convert)
      po1 = sysPose.getPoseToMap(po1);
    // select colour
    if (ob->isValid())
      col = &obsValid;
    else
      col = &obsInvalid;
    //pose = scan->getPose();
    for (i = 0; i < ob->getPointsCnt(); i++)
    {
      po2 = po1;
      po1 = ob->getPoint(i);
      if (convert)
        po1 = sysPose.getPoseToMap(po1);
      pos = seenFromPose.getMapToPose(po1);
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      if (ob->getPointsCnt() > 1)
      {
        pos = seenFromPose.getMapToPose(po2);
        p2.y = pr.y - roundi(pos.x * ppm);
        p2.x = pr.x - roundi(pos.y * ppm);
        cvLine(img->cvArr(), p1, p2, *col, lw, 8, 0);
      }
      else
      { // one point only, paint a circle
        cvCircle(img->cvArr(), p1, 5, *col, lw, 8, 0);
      }
    }
  }
}


///////////////////////////////////////////////////////

void UNavPaint::paintFreePoly(UProbPoly * poly,
                              UPose seenFromPose,
                              bool historic)
{
  int i;
  CvPoint p1, p2;
  UPosition pos;
  UPosition *po1, *po2;
  bool * poObst;
  CvScalar red = CV_RGB(155, 0, 0);
  CvScalar pale = CV_RGB(10, 200, 200);
  CvScalar hist = CV_RGB(150, 100, 100);
  CvScalar * col = &red;
//  const int top = img->height() / 3 + 18;
  CvFont font;
  int lw = 1;
//  const int MTL = 60;
//  char text[MTL];
  //
  if (paintBold)
    lw = 2;

  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               1.0, 1.0, 0.0, 1, 8);
  //
  po1 = poly->getPoints();
  po2 = po1 + 1;
  poObst = poly->getIsObst();
  //pose = scan->getPose();
  for (i = 1; i < poly->getPointsCnt(); i++)
  {
    pos = seenFromPose.getMapToPose(*po1);
    p1.y = pr.y - roundi(pos.x * ppm);
    p1.x = pr.x - roundi(pos.y * ppm);
    pos = seenFromPose.getMapToPose(*po2);
    p2.y = pr.y - roundi(pos.x * ppm);
    p2.x = pr.x - roundi(pos.y * ppm);
    if (*poObst and not historic)
      col = &red;
    else if (*poObst and historic)
      col = &hist;
    else if (not historic)
      col = &pale;
    if (*poObst or not historic)
      cvLine(img->cvArr(), p1, p2, *col, lw, 8, 0);
    // advance to next
    poObst++;
    po1++;
    po2++;
  }
}

///////////////////////////////////////////////

bool UNavPaint::paintOdoDataText()
{
  CvPoint p1, p2;
  CvFont font;
  CvScalar black = CV_RGB(0, 0 , 0);
  const int MTL = 130;
  char text[MTL];
  char textPre[MTL];
  const int MSL = 30;
  char t1[MSL];
  char t2[MSL];
  bool result;
  UPosition pos;
  int i;
  UTime t;
  UPoseTVQ pose;
  UResPoseHist * ph;
  const char * sys = NULL;
  const char * syst = NULL; // time from system
  CvSize text_size;
  int baseline;
  bool disp = false;
  //
  if (poseOdo != NULL)
    t = poseOdo->getNewest().t;
  else if (poseUtm != NULL)
    t = poseUtm->getNewest().t;
  else if (poseMap != NULL)
    t = poseMap->getNewest().t;
  else
    t.clear();

  if (paintBold)
  {
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
                 1.0, 1.0, 0.0, 2, 8);
    //lw = 2;
  }
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result)
  { // paint odo position bottom-left as text
    p1.x = 2;
    p1.y = img->height() - 4;
    //strncpy(textPre, "init", MTL);
    //
    for (i = 2; i >= 0; i--)
    { // paint position of known coordinate systems
      switch (i)
      {
        case 0: ph = poseOdo; sys = "odo "; disp = paintOdoPose; break;
        case 1: ph = poseUtm; sys = "utm"; disp = paintUtmPose; break;
        case 2: ph = poseMap; sys = "map "; disp = paintMapPose; break;
        default: ph = NULL; sys = "NULL"; break;
      }
      if (ph != NULL)
      { // system is available and should be displayed
        pose = ph->getPoseAtTime(t);
        // save time (newest for all systems)
        if (pose.t > t)
        {
          t = pose.t;
          syst = sys;
        }
        if (disp)
        { // display position
          p1.y -= 15;
          if (paintBold)
          {
            p1.y -= 15;
            snprintf(textPre, MTL - 1, " %s %.2fx %.2fy %.0f",
                  sys, pose.x, pose.y, pose.h * 180.0 / M_PI);
            if (paintPoseRef == i)
              textPre[0] = '*';
            cvGetTextSize(textPre, &font, &text_size, &baseline);
            snprintf(text, MTL, "%s  %.1fm/s %gq",
                    textPre, pose.vel, pose.q);
          }
          else
          {
            snprintf(textPre,MTL - 1, " %s %10.2fx %11.2fy %6.1f",
                    sys, pose.x, pose.y, pose.h * 180.0 / M_PI);
            if (paintPoseRef == i)
              textPre[0] = '*';
            cvGetTextSize(textPre, &font, &text_size, &baseline);
            snprintf(text, MTL, "%s  (mat) %.2fm/s %gq",
                    textPre, pose.vel, pose.q);
          }
          cvPutText(img->cvArr(), text, p1, &font, black);
          p2.y = p1.y - (text_size.height * 3) / 5;
          p2.x = p1.x + text_size.width - 2;
          cvPutText(img->cvArr(), "o", p2, &font, black);
          // advance to next pose system
        }
      }
    }

    // debug
/*    p1.y -= 15;
    for (i = 0; i < 32; i++)
    {
      text[i] = i+32+64;
      text[i+1] = '\0';
    }
    cvPutText(img->cvArr(), text, p1, &font, black);
    p1.y -= 15;
    for (i = 0; i < 64; i++)
    {
      text[i] = i+32;
      text[i+1] = '\0';
    }
    cvPutText(img->cvArr(), text, p1, &font, black);*/
    // debug

    // time is at bottom line
    p1.y = img->height() - 4;
    // paint time
    if (t.getDecSec() < 1.0)
      // no timestamp available - use current time
      t.Now();
    t.getDateString(t1, true);
    t.getTimeAsString(t2, true);
    if (paintBold)
      snprintf(text, MTL, "%s %s (%s)",
             t1, t2, syst);
    else
      snprintf(text, MTL, "%s %s (%lu.%06lu scale=%gm)",
             t1, t2, t.getSec(), t.getMicrosec(), maxRange);
    cvPutText(img->cvArr(), text, p1, &font, black);
  }
  return result;
}

////////////////////////////////////////////////////

bool UNavPaint::paintVarDataText(bool paintVar)
{
  CvPoint p1;
  CvFont font;
  //CvScalar bblue = CV_RGB(0, 0 , 100);
  CvScalar brown = CV_RGB(100, 100 , 0);
  CvScalar red = CV_RGB(255, 0 , 0);
  //CvScalar black = CV_RGB(0, 0 , 0);
  bool result = true;
  const int MSL = 240;
  char s[MSL];
  int i, n, m;
  int col1, col2, col3, col4w;
  UTime t;
  //UVarPool * vp;
  UVarPool * vpl;
  UVariable ** var;
  const int MTL = 5;
  char vt[MTL];
  //
  // <planget planner=true file="mission.txt" usingFile=true
  //  cmd="gotowaypoint odo 10.0 0.0  (get GPS)" isIdle=false/>
  // paint variables starting top-left
  p1.x = 10;
  p1.y = 2;
  //
  p1.y += 18;
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                0.9, 1.0, 0.0, 1, 8);
  // either display old stop-crits, or some new variables
  col1 = 10;
  col2 = col1 + 50;
  col3 = col2 + 85;
  col4w = 75;
  //
  //vp = varRoot->getVarPool();
  for (n = 0; n < paintStructsCnt; n++)
  {
    vpl = paintStructs[n];
    if (vpl == NULL)
      // this structure should not be painted
      continue;
    p1.x = col1;
    cvPutText(img->cvArr(), vpl->getFullPreName(s, MSL), p1, &font, red);
    p1.y += 14;
    var = vpl->getVars();
    for (i = 0; i < vpl->getVarsCnt(); i++)
    {
      p1.x = col1 + 5;
      snprintf(s, MSL, "%s (%s)", (*var)->name, (*var)->getTypeChar(vt , MTL));
      cvPutText(img->cvArr(), s, p1, &font, brown);
      p1.x = col3;
      if ((*var)->isTypeA(UVariable::t))
      { // double value
        snprintf(s, MSL, "%8.3f", (*var)->getValued());
        cvPutText(img->cvArr(), s, p1, &font, brown);
        t.setTime((*var)->getValued());
        t.getTimeAsString(s, true);
        p1.x += 2 * col4w;
        cvPutText(img->cvArr(), s, p1, &font, brown);
      }
      else if ((*var)->isString())
      { // string
        cvPutText(img->cvArr(), (*var)->getValues(0), p1, &font, brown);
      }
      else
      { // any variable type will do
        for (m = 0; m < (*var)->getElementCnt(); m++)
        {
          snprintf(s, MSL, "%8.3f", (*var)->getValued(m));
          cvPutText(img->cvArr(), s, p1, &font, brown);
          p1.x += col4w;
        }
      }
      p1.y += 14;
      var++;
    }
  }
  return result;
}

///////////////////////////////////////////////////////

void UNavPaint::paintPis(ULaserDataSet * scan,
                        UPose seenFromPose,
                        const int cnt)
{
  int i;
  CvPoint p1, p2;
  UPosition pos;
  UClientLaserPi * pp;
  CvScalar red = CV_RGB(155, 0, 0);
  CvScalar mag = CV_RGB(100, 0, 100);
  //CvScalar cyan = CV_RGB(0, 100, 100);
  int lw = 1;
//  const int top = img->height() / 3 + 18;
  CvFont font;
//  const int MTL = 60;
//  char text[MTL];
  //
  if (paintBold)
  {
    cvInitFont( &font, CV_FONT_HERSHEY_SCRIPT_COMPLEX,
                 1.0, 1.0, 0.0, 2, 8);
    lw = 2;
  }
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);
  //
  pp = scan->getPis();
  //pose = scan->getPose();
  for (i = 0; i < scan->getPisCnt(); i++)
  {
    pos = seenFromPose.getMapToPose(pp->getLeftPos());
    p1.y = pr.y - roundi(pos.x * ppm);
    p1.x = pr.x - roundi(pos.y * ppm);
    pos = seenFromPose.getMapToPose(pp->getRightPos());
    p2.y = pr.y - roundi(pos.x * ppm);
    p2.x = pr.x - roundi(pos.y * ppm);
    cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);
    // mark left side of road
    if (maxRange < 20)
    {
      pos = seenFromPose.getMapToPose(pp->getLeftSide());
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      cvCircle(img->cvArr(), p1, 4, mag, lw, 8, 0);
      // mark right side of road
      pos = seenFromPose.getMapToPose(pp->getRightSide());
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      cvCircle(img->cvArr(), p1, 4, mag, lw, 8, 0);
      // mark top of road
      pos = seenFromPose.getMapToPose(pp->getTopPos());
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      cvCircle(img->cvArr(), p1, 6, mag, lw, 8, 0);
    }
    //
    pp++;
  }
}

////////////////////////////////////////////////////

void UNavPaint::paintFeatures(UFeaturePool * featurePool, UPoseTime seenFromPose)
{
  int i, n, rl;
  CvPoint p1, p2;
  UPosition pos;
  UPosition po1, po2;
  //const CvScalar purpOdd   = CV_RGB(100, 50, 100);
  //const CvScalar yellowOdd = CV_RGB(100, 100, 50);
  const CvScalar purp      = CV_RGB(190, 0, 190);
  const CvScalar yellow    = CV_RGB(145, 145, 0);
  const CvScalar green     = CV_RGB(0, 150, 0);
  const CvScalar red     = CV_RGB(150, 0, 0);
  const CvScalar blue     = CV_RGB(0, 0, 190);
  const CvScalar gray     = CV_RGB(120, 120, 120);
  UFeatureData * sf;
  ULineSegment * seg;
  const CvScalar * col = &purp;
  int lw = 2;
  int ulw;
  int * segInt;
  bool isRoad;
  //bool showLineOnly;
  const int MAX_TYPES = 4;
  int cnt[MAX_TYPES];
  int typ;
  CvFont font;
  const int MSL = 30;
  char text[MSL];
  double * segVal;
  const char * segStr;
  bool convert = false;
  UPose systemOrigin; //
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               1.0, 1.0, 0.0, 1, 8);
  if (paintBold)
    lw = 4;
  //
  //cnt = mini(sfPool->getScansCnt(), paintScanHistCnt);
  // debug
  //cnt = mini(1, cnt);
  for (i = 0; i < MAX_TYPES; i++)
    cnt[i] = 0;
  // debug end
  for (n = 0; n < featurePool->getScansCnt(); n++)
  {
    sf = featurePool->getScan(n);
    isRoad = false;
    //showLineOnly = false;
    // get segment
    convert = false;
    if (strcasecmp(sf->getDataType(), "sf") == 0)
    {
      col = &purp;
      typ = 1;
      systemOrigin = getSystemOrigin(0, &convert);
    }
    if (strcasecmp(sf->getDataType(), "mapl") == 0)
    { // short for maplines
      col = &blue;
      typ = 4;
      systemOrigin = getSystemOrigin(2, &convert);
    }
    else if (strcasecmp(sf->getDataType(), "pass") == 0)
    {
      col = &green;
      typ = 2;
      systemOrigin = getSystemOrigin(0, &convert);
    }
    else if (strcasecmp(sf->getDataType(), "road") == 0)
    {
      isRoad = true;
      typ = 3;
      //showLineOnly = (cnt[typ] > 5);
      systemOrigin = getSystemOrigin(0, &convert);
    }
    else
    {
      col = &yellow;
      typ = 0;
      systemOrigin = getSystemOrigin(0, &convert);
    }
    if (cnt[typ]++ < paintScanHistCnt)
    { // make newest more fat than older entries
      if (sf->isNewest)
        ulw = lw;
      else
        ulw = lw / 2;
      //
      seg = sf->getSegs();
      segInt = sf->getSegsInt();
      segVal = sf->getSegsVal();
      for (i = 0; i < sf->getSegsCnt(); i++)
      {
        segStr = sf->segsStr[i];
        rl = -1;
        if (isRoad)
        {
          if (*segInt < 10)
            col = &gray;
          else
          {
            rl = *segInt % 10;
            // get last road line position
            if (rl == 0)
              col = &red;
            else if (rl == 2)
              col = &green;
            else
              col = &yellow;
          }
        }
        pos = seg->getOtherEnd();
        if (convert)
          // convert from other coordinate system
          pos = systemOrigin.getPoseToMap(pos);
        po1 = seenFromPose.getMapToPose(pos);
        p1.y = pr.y - roundi(po1.x * ppm);
        p1.x = pr.x - roundi(po1.y * ppm);
        pos = seg->pos;
        if (convert)
          pos = systemOrigin.getPoseToMap(pos);
        po2 = seenFromPose.getMapToPose(pos);
        p2.y = pr.y - roundi(po2.x * ppm);
        p2.x = pr.x - roundi(po2.y * ppm);
        if ((p1.x >= 0 or p2.x >=0) and (p1.y >= 0 or p2.y >= 0) and
             (p1.x < (int)img->width() or p2.x < (int)img->width()) and
             (p1.y < (int)img->height() or p2.y < (int)img->height()))
        {
          cvLine(img->cvArr(), p1, p2, *col, ulw, 8, 0);
          if (false and isRoad)
          { // show text near line
            snprintf(text, MSL, "%d", sf->getSegsCnt());
            cvPutText(img->cvArr(), text, p2, &font, *col);
          }
          else if (segStr[0] != 0)
          { // show text near line
            cvPutText(img->cvArr(), segStr, p2, &font, *col);
          }
          if (typ == 2)
          { // paint center position of passable interval too
            po2 = seenFromPose.getMapToPose(seg->getPositionOnLine(*segVal));
            p2.y = pr.y - roundi(po2.x * ppm);
            p2.x = pr.x - roundi(po2.y * ppm);
            cvCircle(img->cvArr(), p2, 4, *col, lw/2, 8, 0);
          }
        }
        seg++;
        segInt++;
      }
    }
  }
}

/////////////////////////////////////////////////////////

bool UNavPaint::paintRobotMmr(double toFront)
{
  CvPoint p1, p2;
  CvScalar lblue = CV_RGB(200, 200 , 250);
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lblack = CV_RGB(100, 100, 100);
  bool result;
  const double ROBOT_WIDTH = 0.62;
  const double ROBOT_BODY_W = 0.40;
  const double ROBOT_LENGTH = 0.70;
  const double WHEEL_DIAM = 0.35;
  //
  if (paintBold)
  {
    lblue = CV_RGB(50, 50 , 250);
    lyellow = CV_RGB(200, 200 , 0);
    lblack = CV_RGB(10, 10, 10);
  }

  result = (img != NULL);
  if (result)
  { // paint robot body
/*    p1.y = pr.y - roundi(toFront * ppm);
    p1.x = pr.x;*/
    // bottom left corner (pr=laser scanner pos)
    p1.x = pr.x - roundi(ROBOT_BODY_W/2.0 * ppm);
    p1.y = pr.y - roundi(toFront * ppm) + roundi(ROBOT_LENGTH * ppm);
    p2 = p1;
    // top - right
    p2.x += roundi(ROBOT_BODY_W * ppm);
    p2.y -= roundi(ROBOT_LENGTH * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // left wheel
    p1.x -= roundi((ROBOT_WIDTH - ROBOT_BODY_W)/2.0 * ppm);
    p2.x = p1.x + roundi(0.1 * ppm);
    p2.y += roundi((toFront - WHEEL_DIAM/2.0) * ppm);
    p1.y = p2.y + roundi(WHEEL_DIAM * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // right wheel
    p1.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    p2.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
  }
  return result;
}

//////////////////////////////////////////////

bool UNavPaint::paintRobotIrobot(double toFront)
{ // toFront is distance from robot origin to laser scanner
  CvPoint p1, p2;
  CvScalar lblue = CV_RGB(200, 200 , 250);
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lblack = CV_RGB(100, 100, 100);
  bool result;
  // 0.0 is center, x is right, y is up, size in meters
  // type 0 is start of new structure
  // type 4 is a square (as z value)
  // type 2 is a line
  const int MLC = 15;
  UPosition rob[MLC];
  int cnt = 0;
  rob[cnt++].set(-0.2, -0.32, 0); // body - back left
  rob[cnt++].set(+0.2, +0.32, 4); // square - front right
  rob[cnt++].set(-0.32, -0.35, 0); // back left
  rob[cnt++].set(-0.22, -0.05, 4);
  rob[cnt++].set(+0.32, -0.35, 0); // back right
  rob[cnt++].set(+0.22, -0.05, 4);
  rob[cnt++].set(-0.32, +0.35, 0); // front left
  rob[cnt++].set(-0.22, +0.05, 4);
  rob[cnt++].set(+0.32, +0.35, 0); // front right
  rob[cnt++].set(+0.22, +0.05, 4);
  rob[cnt++].set(+0.20, +0.55, 0); // front bumper
  rob[cnt++].set(-0.20, +0.50, 4);
  rob[cnt++].set(+0.20, -0.45, 0); // rear bumper
  rob[cnt++].set(-0.20, -0.40, 4);
  //
  if (paintBold)
  {
    lblue = CV_RGB(50, 50 , 250);
    lyellow = CV_RGB(200, 200 , 0);
    lblack = CV_RGB(10, 10, 10);
  }

  result = (img != NULL);
  if (result)
  { // paint robot body
/*    p1.y = pr.y - roundi(toFront * ppm);
    p1.x = pr.x;*/
    // bottom left corner (pr=laser scanner pos)
    p1 = pr;
    for (int i = 0; i < cnt; i++)
    {
      p2 = p1;
      p1.x = pr.x + roundi(rob[i].x * ppm);
      p1.y = pr.y - roundi(rob[i].y * ppm);
      switch (roundi(rob[i].z))
      {
        case 4:
          cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
          break;
        case 3: // circle
          cvLine(img->cvArr(), p1, p2, lblack, 2, 8, 0);
          break;
        case 2:
          cvLine(img->cvArr(), p1, p2, lblack, 2, 8, 0);
          break;
  //       case 1:
  //         cvEllipse(img->cvArr(), p1, p2, lblack, 2, 8, 0);
  //         break;
        default:
          // nothing - wait for more data
          break;
      }
    }
  }
  return result;
}

///////////////////////////////////////////////////

bool UNavPaint::paintRobotGuidebot(double toFront)
{ // toFront is distance from robot origin to laser scanner
  CvPoint p1, p2;
  CvScalar lblue = CV_RGB(200, 200 , 250);
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lblack = CV_RGB(100, 100, 100);
  bool result;
  // 0.0 is center, x is right, y is up, size in meters
  // type 0 is start of new structure
  // type 4 is a square (as z value)
  // type 2 is a line
  const int MLC = 17;
  UPosition rob[MLC];
  int cnt = 0;
  rob[cnt++].set(-0.34, +0.0, 0); // body - left side
  rob[cnt++].set(-0.29, +0.17, 2); // circle clockwise
  rob[cnt++].set(-0.17, +0.29, 2); // circle
  rob[cnt++].set(-0.00, +0.34, 2); // circle
  rob[cnt++].set(+0.17, +0.29, 2); // circle
  rob[cnt++].set(+0.29, +0.17, 2); // circle
  rob[cnt++].set(+0.34, +0.00, 2); // circle
  rob[cnt++].set(+0.29, -0.17, 2); // circle
  rob[cnt++].set(+0.17, -0.29, 2); // circle
  rob[cnt++].set(+0.00, -0.34, 2); // circle
  rob[cnt++].set(-0.17, -0.29, 2); // circle
  rob[cnt++].set(-0.29, -0.17, 2); // circle
  rob[cnt++].set(-0.34, -0.00, 2); // circle
  rob[cnt++].set(-0.26, -0.10, 0); // circle
  rob[cnt++].set(-0.21, +0.10, 4); // circle
  rob[cnt++].set(+0.26, -0.10, 0); // circle
  rob[cnt++].set(+0.21, +0.10, 4); // circle
  //
  if (paintBold)
  {
    lblue = CV_RGB(50, 50 , 250);
    lyellow = CV_RGB(200, 200 , 0);
    lblack = CV_RGB(10, 10, 10);
  }

  result = (img != NULL);
  if (result)
  { // paint robot body
/*    p1.y = pr.y - roundi(toFront * ppm);
    p1.x = pr.x;*/
    // bottom left corner (pr=laser scanner pos)
    p1 = pr;
    for (int i = 0; i < cnt; i++)
    {
      p2 = p1;
      p1.x = pr.x + roundi(rob[i].x * ppm);
      p1.y = pr.y - roundi(rob[i].y * ppm);
      switch (roundi(rob[i].z))
      {
        case 4:
          cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
          break;
        case 3: // circle
          cvLine(img->cvArr(), p1, p2, lblack, 2, 8, 0);
          break;
        case 2:
          cvLine(img->cvArr(), p1, p2, lblack, 2, 8, 0);
          break;
  //       case 1:
  //         cvEllipse(img->cvArr(), p1, p2, lblack, 2, 8, 0);
  //         break;
        default:
          // nothing - wait for more data
          break;
      }
    }
  }
  return result;
}

///////////////////////////////////////////////

bool UNavPaint::paintRobotSmr(double toFront)
{
  CvPoint p1, p2, p3;
  CvScalar lblue = CV_RGB(200, 200 , 250);
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lblack = CV_RGB(100, 100, 100);
  bool result;
  const double ROBOT_WIDTH = 0.28;
  const double ROBOT_BODY_W = 0.20;
  const double ROBOT_LENGTH = 0.28;
  const double WHEEL_DIAM = 0.09;
  //
  if (paintBold)
  {
    lblue = CV_RGB(50, 50 , 250);
    lyellow = CV_RGB(200, 200 , 0);
    lblack = CV_RGB(10, 10, 10);
  }
  result = (img != NULL);
  if (result)
  { // paint robot body
    p3.y = pr.y - roundi(toFront * ppm);
    p3.x = pr.x;
    // bottom left corner (pr=laser scanner pos)
    p1.x = p3.x - roundi(ROBOT_BODY_W/2.0 * ppm);
    p1.y = p3.y + roundi(ROBOT_LENGTH * ppm);
    p2 = p1;
    // top - right
    p2.x += roundi(ROBOT_BODY_W * ppm);
    p2.y -= roundi(ROBOT_LENGTH * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // left wheel
    p1.x -= roundi((ROBOT_WIDTH - ROBOT_BODY_W)/2.0 * ppm);
    p2.x = p1.x + roundi(0.1 * ppm);
    p2.y += roundi((toFront - WHEEL_DIAM/2.0) * ppm);
    p1.y = p2.y + roundi(WHEEL_DIAM * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // right wheel
    p1.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    p2.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
  }
  return result;
}

////////////////////////////////////////////////////

bool UNavPaint::paintRobotHako(double toFront)
{ // to front is distance from robot origo to laser scanner origo
  CvPoint p1, p2, p3;
  CvScalar lblue = CV_RGB(200, 200 , 250);
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lblack = CV_RGB(100, 100, 100);
  bool result;
  const double LASER_TO_FRONT = 0.15;
  const double WHEEL_DIAMf = 0.57;
  const double WHEEL_DIAMr = 0.92;
  const double WHEEL_WIDTHf = 0.16;
  const double WHEEL_WIDTHr = 0.25;
  const double ROBOT_WIDTH = 1.02; // track width - center of wheels
  const double ROBOT_BODY_Wf = 0.50;
  const double ROBOT_BODY_Wr = 1.0;
  const double ROBOT_LENGTHf = 1.1;
  const double ROBOT_LENGTH = 2.15;
  //
  if (paintBold)
  {
    lblue = CV_RGB(50, 50 , 250);
    lyellow = CV_RGB(200, 200 , 0);
    lblack = CV_RGB(10, 10, 10);
  }
  result = (img != NULL);
  if (result)
  { // paint robot body - p3 = front centre
    p3.y = pr.y - roundi((toFront - LASER_TO_FRONT) * ppm);
    p3.x = pr.x;
    // top left corner
    p1.x = p3.x - roundi(ROBOT_BODY_Wf/2.0 * ppm);
    p1.y = p3.y;
    // bottom right of fromt
    p2.x = p3.x + roundi(ROBOT_BODY_Wf/2.0 * ppm);
    p2.y = p3.y + roundi(ROBOT_LENGTHf * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // top left back
    p1.x = p3.x - roundi(ROBOT_BODY_Wr/2.0 * ppm);
    p1.y = p3.y + roundi(ROBOT_LENGTHf * ppm);
    // bottom right of fromt
    p2.x = p3.x + roundi(ROBOT_BODY_Wr/2.0 * ppm);
    p2.y = p3.y + roundi(ROBOT_LENGTH * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // left rear wheel - p1 is leftmost
    p1.x = p3.x - roundi((ROBOT_WIDTH + WHEEL_WIDTHr) / 2.0 * ppm);
    p2.x = p1.x + roundi(WHEEL_WIDTHr * ppm);
    p1.y = pr.y - roundi(WHEEL_DIAMr/2.0 * ppm);
    p2.y = p1.y + roundi(WHEEL_DIAMr * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // right rear wheel - p1 is leftmost
    p1.x = p3.x + roundi((ROBOT_WIDTH + WHEEL_WIDTHr) / 2.0 * ppm);
    p2.x = p1.x - roundi(WHEEL_WIDTHr * ppm);
    p1.y = pr.y - roundi(WHEEL_DIAMr/2.0 * ppm);
    p2.y = p1.y + roundi(WHEEL_DIAMr * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // left front wheel - p1 is leftmost
    p1.x = p3.x - roundi((ROBOT_WIDTH + WHEEL_WIDTHf) / 2.0 * ppm);
    p2.x = p1.x + roundi(WHEEL_WIDTHf * ppm);
    p1.y = p3.y;
    p2.y = p1.y + roundi(WHEEL_DIAMf * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // right front wheel - p1 is leftmost
    p1.x = p3.x + roundi((ROBOT_WIDTH + WHEEL_WIDTHf) / 2.0 * ppm);
    p2.x = p1.x - roundi(WHEEL_WIDTHf * ppm);
    p1.y = p3.y;
    p2.y = p1.y + roundi(WHEEL_DIAMf * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
  }
  return result;
}

////////////////////////////////////////////////////

void UNavPaint::paintCross(UImage * img, int x, int y, int size, CvScalar col, int lineWidth)
{
  //size /= 2;
  cvLine(img->cvArr(), cvPoint(x-size, y-size),
         cvPoint(x+size, y+size), col, lineWidth, 4, 0);
  cvLine(img->cvArr(), cvPoint(x+size, y-size),
         cvPoint(x-size, y+size), col, lineWidth, 4, 0);
}

///////////////////////////////////////

void UNavPaint::paintPose(UImage * img, int x, int y, double h, int size, CvScalar col, int lineWidth)
{
  int dx, dy;
  //
  dx = -roundi(sin(h)*double(size)*5.0);
  dy = -roundi(cos(h)*double(size)*5.0);
  cvCircle(img->cvArr(), cvPoint(x, y), size, col, lineWidth, size/2, 0);
  cvLine(img->cvArr(), cvPoint(x, y),
         cvPoint(x+dx, y+dy), col, lineWidth, 4, 0);
}

///////////////////////////////////////

bool UNavPaint::paintVarAdd(const char * name, bool add)
{
  UVarPool * vp, *vp2, **vp3 = NULL;
  int i;
  bool found = false;
  bool result = true;
  //
  if (varRoot != NULL)
  {
    vp = varRoot->getVarPool();
    vp2 = vp->getStructDeep(name, NULL, 0);
    if (vp2 != NULL)
    { // such structure exist - find it
      // see if it is in list already
      for (i = 0; i < paintStructsCnt; i++)
      {
        if (paintStructs[i] == NULL)
        { // an empty slot - save if new addition
          if (vp3 == NULL and add)
            vp3 = &paintStructs[i];
        }
        else if (paintStructs[i] == vp2)
        {
          if (not add)
            paintStructs[i] = NULL;
          found = true;
          break;
        }
      }
      if (not found and add)
      { // need to add
        if (vp3 == NULL and (paintStructsCnt < PAINT_MAX_STRUCTS))
            vp3 = &paintStructs[paintStructsCnt++];
        if (vp3 != NULL)
          *vp3 = vp2;
      }
    }
    else
      result = false;
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////

bool UNavPaint::paintManData(UClientManSeq * man, int num, UPoseTime seenFromPose)
{
  CvPoint p1, p2;
  CvFont font;
  bool result;
  UPosition pos;
  UPose *ppos;
  CvScalar red;
  CvScalar green;
  CvScalar rtCol;
  //CvScalar yellow = CV_RGB(150, 150, 0);
  CvScalar blue = CV_RGB(0, 0, 255);
  CvScalar magenta = CV_RGB(150, 0, 150);
  int i, j, redWidth;
  int lw = 1;
  UManPPSeq * mpp;
  UManoeuvre * mm;
  UManArc * mma;
  UPoseV pv1, pv2;
  UPosition pos1, pos2;
  UPose pose1;
  double a1, a2;
  CvSize sz;
  const int MSL = 20;
  char s[MSL];
  ULineSegment * seg;
  char * segChars;
  UPose systemOrigin;
  bool convert;
/*  U2Dpos pd1;
  double d;*/
  //
  if (man->isBest())
  {
    green = CV_RGB(0, 200, 0);
    red = CV_RGB(200, 0, 0);
    rtCol = CV_RGB(250, 0, 0);
    redWidth = 3;
  }
  else
  { // unused path lines
    green = CV_RGB(0, 200, 200);
    red = CV_RGB(200, 100, 100);
    rtCol = CV_RGB(180, 180, 0);
    blue = CV_RGB(128, 128, 255);
    redWidth = 2;
  }
  if (paintBold)
    lw = 2;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               0.8, 1.0, 0.0, 1, 8);
  result = (img != NULL) and (man != NULL);
  if (result)
  { // debug - changing colours
    if (not man->isBest())
      rtCol = CV_RGB(90, (num % 2) * 90, ((num / 2) % 3) * 90);
    // debug end
    // get possible conversion to other origin
    systemOrigin = getSystemOrigin(0, &convert);
    // get all parts
    for (i = 0; i < man->getP2PCnt(); i++)
    { // a path is divided into groups of 1 to 5 (line, arc, line, arc, line) simple manoeuvres
      mpp = man->getP2P(i);
      pv1 = mpp->getStartPoseV();
/*      pv2 = mpp->getEndPoseV();
      pos = seenFromPose.getMapToPose(pv1.getPos());
      // and to pixel coordinates
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      pos = seenFromPose.getMapToPose(pv2.getPos());
      // and to pixel coordinates
      p2.y = pr.y - roundi(pos.x * ppm);
      p2.x = pr.x - roundi(pos.y * ppm);*/
      // paint direct (thin) line
      //cvLine(img->cvArr(), p1, p2, rtCol, 1);
      // now paint curved line too
      for (j = 0; j < mpp->getSeqCnt(); j++)
      { // paint each of the simple manoeuvres (line or arc)
        mm = mpp->getMan(j);
        pv2 = mm->getEndPoseV(pv1);
        switch (mm->getManType())
        { // either MAN_ARC, MAN_LINE or MAN_STOP
        case UManoeuvre::MAN_ARC:
          // convert to arc type manoeuvre
          mma = (UManArc*)mm;
          if (mma->getTurnRadius() < 0.0)
          {
            mma->setTurnAngle(-mma->getTurnAngle());
            mma->setTurnRadius(-mma->getTurnRadius());
          }
          // if too big, then draw a line, too big is set to r> ~ 200 meter
          if (mma->getTurnRadius() > (10.0 * maxRange))
          { // just a line from start (pv1) to finish (pv2)
            p1 = toPixels(pv1.getPos(), seenFromPose, systemOrigin, convert);
            p2 = toPixels(pv2.getPos(), seenFromPose, systemOrigin, convert);
            // paint direct line
            cvLine(img->cvArr(), p1, p2, rtCol, redWidth * lw);
          }
          else
          { // OK, draw as arc
            // find centre
            if (mma->getTurnAngle() > 0.0)
            {
              pos2.y = mma->getTurnRadius();
              a1 = (pv1.h - seenFromPose.h) * (180.0 / M_PI) - 90.0;
              a2 = a1 + (180.0 / M_PI) * mma->getTurnAngle();
            }
            else
            {
              pos2.y = -mma->getTurnRadius();
              a2 = (pv1.h - seenFromPose.h) * (180.0 / M_PI) + 90.0;
              a1 = a2 + (180.0 / M_PI) * mma->getTurnAngle();
            }
            pos2.x = 0.0;
            // in real map coordinates
            pos = pv1.getPoseToMap(pos2);
            // to displayed coordinate  system
            if (convert)
            {
              pos = systemOrigin.getPoseToMap(pos);
              a1 += systemOrigin.h * 180.0 / M_PI;
              a2 += systemOrigin.h * 180.0 / M_PI;
            }
            // and to robot coordinates
            pos = seenFromPose.getMapToPose(pos);
            // and to pixel coordinates
            p1 = toPixels(pos);
            // set arc radius
            sz.width = roundi(mma->getTurnRadius() * ppm);
            sz.height = sz.width;
            cvEllipse( img->cvArr(), p1, sz, -90.0,
                        -a1, -a2, rtCol, redWidth * lw);
          }
          break;
        case UManoeuvre::MAN_LINE:
          p1 = toPixels(pv1.getPos(), seenFromPose, systemOrigin, convert);
          p2 = toPixels(pv2.getPos(), seenFromPose, systemOrigin, convert);
          // paint direct line
          cvLine(img->cvArr(), p1, p2, rtCol, redWidth * lw);
          if (maxRange < 35.0)
          { // paint also circle at end of line
            cvCircle(img->cvArr(), p1, 4, red, lw, 4, 0);
            cvCircle(img->cvArr(), p2, 4, red, lw, 4, 0);
          }
          break;
        case UManoeuvre::MAN_STOP:
          break;
        }
        pv1 = pv2;
      }
    }
    if (paintPathMidPoses)
    {
      ppos = man->getPoses();
      // make color stronger for text and crosses
      rtCol.val[0] = rtCol.val[0] / 2;
      rtCol.val[1] = rtCol.val[1] / 2;
      rtCol.val[2] = rtCol.val[2] / 2;
      for (i = 0; i < man->getPosesCnt(); i++)
      { // convert to robot coordinates
        pose1 = *ppos;
        if (convert)
          pose1 = systemOrigin.getPoseToMapPose(pose1);
        pose1 = seenFromPose.getMapToPosePose(&pose1);
                  // and to pixel coordinates
        p1 = toPixels(pose1);
/*        p1.y = pr.y - roundi(pose1.x * ppm);
        p1.x = pr.x - roundi(pose1.y * ppm);*/
        // paint cross at position (or a fat circle if crashed)
        if (man->isPathFailed())
          cvCircle(img->cvArr(), p1, 6, rtCol, 2, 8, 0);
        else
          paintCross(img, p1.x, p1.y, 4, rtCol, lw);
        //
        p2.y = pr.y - roundi(pose1.x * ppm + 40.0 * cos(pose1.h));
        p2.x = pr.x - roundi(pose1.y * ppm + 40.0 * sin(pose1.h));
        cvLine(img->cvArr(), p1, p2, rtCol, lw);
        // and midPose serial number
        snprintf(s, MSL, "%d(%d)", num, i);
        cvPutText(img->cvArr(), s, p1, &font, blue);
        //
        ppos++;
      }
    }
    if (paintPathSupportLines)
    {
      seg = man->getSegs();
      segChars = man->getSegChars();
      for (i = 0; i < man->getSegsCnt(); i++)
      { // convert to robot coordinates
        pos1 = seg->pos;
        if (convert)
          pos1 = systemOrigin.getPoseToMap(pos1);
        pos1 = seenFromPose.getMapToPose(pos1);
        pos2 = seg->getOtherEnd();
        if (convert)
          pos2 = systemOrigin.getPoseToMap(pos2);
        pos2 = seenFromPose.getMapToPose(pos2);
                  // and to pixel coordinates
        p1.y = pr.y - roundi(pos1.x * ppm);
        p1.x = pr.x - roundi(pos1.y * ppm);
        p2.y = pr.y - roundi(pos2.x * ppm);
        p2.x = pr.x - roundi(pos2.y * ppm);
        // get colour
        if (segChars[i] == 't')
          rtCol = blue;
        else if (segChars[i] == 's')
        {
          rtCol = blue;
          if (paintBold)
            lw = 4;
          else
            lw = 2;
        }
        else
          rtCol = magenta;
        // paint no-visibility line
        cvLine(img->cvArr(), p1, p2, rtCol, lw);
        //
        seg++;
      }
    }
  }
  return result;
}

////////////////////////////////////////////////

void UNavPaint::paintCams(UPoseTime seenFromPose)
{
  int i;
  CvPoint p1, p2;
  UPosition posm, posr, pos;
  CvScalar red = CV_RGB(255, 0, 0);
  int lw = 1;
  UClientCamData * cam;
  UMatrix4 mCam;
  UPosRot camPose;
  //
  if (paintBold)
    lw = 2;
  //
  if (camCam != NULL)
  {
    for (i = 0; i < camCam->getCams()->getCamsCnt(); i++)
    { // get pose with this age - older and older
      cam = camCam->getCams()->getCam(i);
      if (cam != NULL)
        if (cam->parValid)
        {
          camPose.set(cam->pos, cam->rot);
          mCam = camPose.getRtoMMatrix();
          // robot pose for this update
          p1.y = pr.y - roundi(cam->pos.x * ppm);
          p1.x = pr.x - roundi(cam->pos.y * ppm);
          if (maxRange < 15.0)
            // paint also circle at camera position
            cvCircle(img->cvArr(), p1, 6, red, lw, 8, 0);
          // line in direction of left pixel (center)
          pos = cam->getPtoCRob(0, cam->height/2, 1.0);
          pos.transfer(&mCam);
          p2.y = pr.y - roundi(pos.x * ppm);
          p2.x = pr.x - roundi(pos.y * ppm);
          cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);
          // line in direction of right pixel (center)
          pos = cam->getPtoCRob(cam->width, cam->height/2, 1.0);
          pos.transfer(&mCam);
          p2.y = pr.y - roundi(pos.x * ppm);
          p2.x = pr.x - roundi(pos.y * ppm);
          cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);
        }
    }
  }
}

////////////////////////////////////////////////

void UNavPaint::paintGmks(UPoseTime seenFromPose)
{
  int i;
  CvPoint p1, p2, p3;
  UPosition posm, posr, pos;
  CvScalar red = CV_RGB(255, 0, 0);
  CvScalar blue = CV_RGB(85, 0, 85);
  int lw = 3;
  UGmk * gmk;
  UMatrix4 mGmk;
  const int MSL = 20;
  char s[MSL];
  CvFont font;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               0.8, 1.0, 0.0, 1, 8);
  //
  if (paintBold)
    lw = 5;
  //
  if (camGmk != NULL)
  {
    for (i = 0; i < camGmk->getGmkPool()->getGmkCnt(); i++)
    { // get pose with this age - older and older
      gmk = camGmk->getGmkPool()->getGmkNum(i);
      if (gmk != NULL)
      {
        mGmk = gmk->getRtoMMatrix();
        // robot pose for this update
        p1.y = pr.y - roundi(gmk->getX() * ppm);
        p1.x = pr.x - roundi(gmk->getY() * ppm);
        if (maxRange < 15.0)
          // paint also circle at camera position
          cvCircle(img->cvArr(), p1, 4, red, lw/2, 8, 0);
        // line across guidemark
        pos.set(0.0, 0.12, 0.0);
        pos.transfer(&mGmk);;
        p2.y = pr.y - roundi(pos.x * ppm);
        p2.x = pr.x - roundi(pos.y * ppm);
        pos.set(0.0, -0.12, 0.0);
        pos.transfer(&mGmk);;
        p3.y = pr.y - roundi(pos.x * ppm);
        p3.x = pr.x - roundi(pos.y * ppm);
        cvLine(img->cvArr(), p2, p3, red, lw, 8, 0);
        snprintf(s, MSL, "%lu", gmk->getCodeInt());
        p1.x += 5;
        p1.y -= 5;
        cvPutText(img->cvArr(), s, p1, &font, blue);
      }
    }
  }
}

///////////////////////////////////////////////

void UNavPaint::printRefSystems()
{
  int i,j;
  UPose p;
  UTime t;
  const int MSL = 30;
  char s[MSL];
  //
  if (poseOdo != NULL)
    t = poseOdo->getNewest().t;
  else if (poseUtm != NULL)
    t = poseUtm->getNewest().t;
  else if (poseMap != NULL)
    t = poseMap->getNewest().t;
  printf("source \\ dest: --------------odo--------------   --------------utm--------------   --------------map--------------\n");
  printf("current ");
  p = poseOdo->getPoseAtTime(t);
  printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
  p = poseUtm->getPoseAtTime(t);
  printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
  p = poseMap->getPoseAtTime(t);
  printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
  printf("\n");
  for (i = 0; i < MAX_REF_SYS; i++)
  {
    switch(i)
    {
      case 0: printf("odo     "); break;
      case 1: printf("utm     "); break;
      case 2: printf("map     "); break;
      default: break;
    }
    for (j = 0; j < MAX_REF_SYS; j++)
    {
      p = poseToRef[i][j];
      printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
    }
    printf("\n");
  }
  printf("at time %lu.%06lu %s\n", t.getSec(), t.getMicrosec(), t.getDateTimeAsString(s, true));
}

///////////////////////////////////////////////

void UNavPaint::setRefSystemsHere()
{
  int i, j;
  UResPoseHist *psi, *psj;
  UPose pd, ps, pz1, pz;
  UTime t;
  //
  if (poseOdo != NULL)
    t = poseOdo->getNewest().t;
  else if (poseUtm != NULL)
    t = poseUtm->getNewest().t;
  else if (poseMap != NULL)
    t = poseMap->getNewest().t;
  //
  for (i = 0; i < MAX_REF_SYS; i++)
  { // i is the source system
    switch (i)
    {
      case 0: psi = poseOdo; break;
      case 1: psi = poseUtm; break;
      case 2: psi = poseMap; break;
      default: psi = NULL; break;
    }
    for (j = 0; j < MAX_REF_SYS; j++)
    { // j is the destination system
      if (psi == NULL)
        poseToRef[i][j].clear();
      else
      {
        switch (j)
        {
          case 0: psj = poseOdo; break;
          case 1: psj = poseUtm; break;
          case 2: psj = poseMap; break;
          default: psj = NULL; break;
        }
        if (psj == NULL)
          poseToRef[i][j].clear();
        else
        { // both systems exist
          if (psi == psj)
            // 1:1
            poseToRef[i][j].clear();
          else
          { // both systems exist and is different
            // a coordinate in the i system can be converted to
            // destination system by:
            // poseDest = poseToRef[i][j].getPoseToMapPose(poseSource)
            ps.clear();
            ps = psi->getPoseAtTime(t);
            pd = psj->getPoseAtTime(t);
            // source origo relative current source pose
            pz.clear();
            pz1 = ps.getMapToPosePose(&pz);
            // convert this local coordinate to a map pose in destination system
            poseToRef[i][j] = pd.getPoseToMapPose(pz1);
          }
        }
      }
    }
  }
}

///////////////////////////////////////////

void UNavPaint::paintPoseHistLines(UPoseTime seenFromPose)
{
  int j;
  UPose pr;
  UResPoseHist * psj;
  //
  for (j = 0; j < MAX_REF_SYS; j++)
  {
    switch (j)
    {
      case 0: psj = poseOdo; break;
      case 1: psj = poseUtm; break;
      case 2: psj = poseMap; break;
      default: psj = NULL; break;
    }
    if (psj != NULL)
    { // system exist, paint lines
      pr = poseToRef[j][paintPoseRef];
      // paint pose history for this coordinate system
      paintPoseHistLine(seenFromPose, psj, j, j != paintPoseRef, pr);
    }
  }
}

/////////////////////////////////////////////

UPose UNavPaint::getSystemOrigin(int myOrigin,
                                 bool * convertNeeded)
{
  UPose result;
  //
  if (myOrigin >= 0 and myOrigin < MAX_REF_SYS)
  { // system exist, paint lines
    result = poseToRef[myOrigin][paintPoseRef];
  }
  else
    result.clear();
  if (convertNeeded != NULL)
    *convertNeeded = myOrigin != paintPoseRef;
  return result;
}

////////////////////////////////////////////////

CvScalar UNavPaint::getColor(char col)
{
  CvScalar result;
  switch(col)
  { // some inspiration from scilab color_list
    case 'k': result = CV_RGB(0, 0, 0); break;
    case 'r': result = CV_RGB(255, 0, 0); break;
    case 'g': result = CV_RGB(0, 255, 0); break;
    case 'b': result = CV_RGB(0, 0, 255); break;
    case 'y': result = CV_RGB(200, 200, 0); break;
    case 'm': result = CV_RGB(200, 0, 200); break;
    case 'c': result = CV_RGB(0, 200, 200); break;
    case '0': result = CV_RGB(90, 50, 50); break;
    case '1': result = CV_RGB(100, 60, 60); break;
    case '2': result = CV_RGB(110, 70, 70); break;
    case '3': result = CV_RGB(120, 80, 80); break;
    case '4': result = CV_RGB(130, 90, 90); break;
    case '5': result = CV_RGB(140, 100, 100); break;
    case '6': result = CV_RGB(150, 110, 110); break;
    case '7': result = CV_RGB(160, 120, 120); break;
    case '8': result = CV_RGB(140, 130, 120); break;
    case '9': result = CV_RGB(130, 140, 120); break;
    case 'p': result = CV_RGB(255,200,200); break; // pink
    case 'w': result = CV_RGB(255,231,186); break; // wheat
    case 'n': result = CV_RGB(0, 0, 128); break;   // navy
    default:  result = CV_RGB(180, 0, 180); break;
  }
  return result;
}


void UNavPaint::paintPolyItems(UPoseTime seenFromPose)
{
  UPose sysPose;
  bool convert;
  //int mySystem = 1; // is UTM
  CvPoint p1 = {0,0}, p2, p0, pcog = {0,0};
  UPosition pos;
  int i, j, im, jm;
  UPolyItem * pi;
  CvScalar magd = CV_RGB(180, 0, 180); // default polygon colour
  CvScalar mag1 = CV_RGB(240, 50, 180); // odd number
  CvScalar mag2 = CV_RGB(180, 50, 240); // even number
  CvScalar mag3 = CV_RGB(240, 100, 240); // has period in name
  CvScalar scicol;
  CvScalar * col = &magd;
  int lw=2;
  char * c1, *c2;
  bool show;
  bool hide;
/*  const int MSL = 20;
  char s[MSL];*/
  CvFont font;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               0.8, 1.0, 0.0, 1, 8);
  //
  if (paintBold)
    lw = 5;
  //
  if (resPoly != NULL)
  {
    im = resPoly->getPolysCnt();
    for (i = 0; i < im; i++)
    {
      pi = resPoly->getItem(i);
      lw = 2;
      if (isdigit(pi->color[1]))
        lw = pi->color[1] - '0';
      if (paintBold)
        lw = lw + 2;
      hide = strlen(paintPolyHide) > 0;
      if (hide)
        hide = pattern_match(pi->name, paintPolyHide);
      show = not hide;
      if (not show and strlen(paintPolyShow) > 0)
        show = pattern_match(pi->name, paintPolyShow);
      if (show and pi->getPointsCnt() > 0)
      { // coordinate conversion option
        sysPose = getSystemOrigin(pi->cooSys, &convert);
        // check name for colour options
        if (pi->color[0] != 'd')
        { // use the specified color
          scicol = getColor(pi->color[0]);
          col = &scicol;
        }
        c1 = strrchr(pi->name, '.');
        if (c1 != NULL)
        { // if .99 notation, then change color after 99 number
          c1++;
          // get number
          j = strtol(c1, &c2, 10);
          if (pi->color[0] == 'd')
          { // default color - shift using number
            if (c2 == c1)
              // no number
              col = &mag3;
            else if (j % 2 == 0)
              col = &mag2;
            else
              col = &mag1;
          }
        }
        if (paintPolyNameCnt > 0)
        { // paint polygon name, or just last part of name
          if ((int)strlen(pi->name) > paintPolyNameCnt)
            c1 = &pi->name[strlen(pi->name) - paintPolyNameCnt];
          else
            c1 = pi->name;
          if (strlen(c1) > 0)
          {
            pcog = toPixels(pi->getCogXY(), seenFromPose, sysPose, convert);
            cvPutText(img->cvArr(), c1, pcog, &font, *col);
          }
        }
        jm = pi->getPointsCnt();
        for (j = 0; j < jm; j++)
        {
          p1 = toPixels(pi->getPoint(j), seenFromPose, sysPose, convert);
          if (pi->color[2] == 'd' or pi->color[3] == 'o')
            cvCircle(img->cvArr(), p1, 3, *col, lw, 8, 0);
          if (c1 != NULL)
          { // polygon has number
            // paint polygon line (sort of) inside polygon
            if (p1.x > pcog.x + lw)
              p1.x -= lw;
            else if (p1.x < pcog.x - lw)
              p1.x += lw;
            if (p1.y > pcog.y + lw)
              p1.y -= lw;
            else if (p1.y < pcog.y - lw)
              p1.y += lw;
          }
          if (j > 0)
          { // paint line
            if (pi->color[3] != ' ')
              cvLine(img->cvArr(), p1, p2, *col, lw, 8, 0);
              // save last painted pose position
          }
          else
            // save first point
            p0 = p1;
          p2 = p1;
        }
        if (pi->isPolygon()  and pi->color[3] != ' ')
          // paint closing line too
          cvLine(img->cvArr(), p1, p0, *col, lw, 8, 0);
      }
    }
  }
}

///////////////////////////////////////////////////

CvPoint UNavPaint::toPixels(UPosition pos, int coordinateSystem, UPose seenFromPose)
{
  bool convert;
  UPose systemOrigin;
  //
  systemOrigin = getSystemOrigin(coordinateSystem, &convert);
  return toPixels(pos, seenFromPose, systemOrigin, convert);
}

///////////////////////////////////////////////////

CvPoint UNavPaint::toPixels(UPosition pos, UPose seenFromPose, UPose systemOrigin, bool convert)
{
  if (convert)
    pos = systemOrigin.getPoseToMap(pos);
  pos = seenFromPose.getMapToPose(pos);
  return toPixels(pos);
}

///////////////////////////////////////////////////

CvPoint UNavPaint::toPixels(UPosition pos)
{
  U2Dpos pd1;
  double d;
  CvPoint p1;
  //
  pd1.y = pr.y - pos.x * ppm;
  pd1.x = pr.x - pos.y * ppm;
  d = pd1.dist();
  if (d > 10000.0)
  { // too far away in pixels - limit towards visible area
    pd1.x *= 10000.0 / d;
    pd1.y *= 10000.0 / d;
  }
  p1.y = roundi(pd1.y);
  p1.x = roundi(pd1.x);
  return p1;
}

//////////////////////////////////////////////////

void UNavPaint::mousePan(CvPoint pix, double newScale)
{
/*  robotPose.x -= (pix.y - pr.y)/ppm;
  robotPose.y += (pix.x - pr.x)/ppm;*/
  UPose newCenter;
  UPose p2;
  //
  // get get click in metric screen coordinates (relative to lower center of screen)
  newCenter.x = (img->height() - pix.y)/ppm;
  newCenter.y = (pix.x - img->width()/2.0)/ppm;
  newCenter.h = 0.0;

//printf("new center on screen in meters %.2fm, ppm=%.4f, new center = %.2fx,%.2fy(meters)\n", maxRange, ppm, newCenter.x, newCenter.y);

  // get robot pose relative to this new center position
  p2 = newCenter.getMapToPosePose(robotPose);
  // move so that center is screen center rather than bottom of screen - in new scale
  p2.x += newScale / 2.0;
  robotPose = p2;
  maxRange = newScale;

//printf("after  Pan %.2fm, ppm=%.4f, robot pose = %.2fx,%.2fy,%.4fh\n", maxRange, ppm, robotPose.x, robotPose.y, robotPose.h);
//printRefSystems();
}

//////////////////////////////////////////////////

void UNavPaint::mouseScale(CvPoint pix, CvPoint dx)
{
  double sc; // new scale (height of window in meters)
  CvPoint nc;
  double x = absi(dx.x);
  double y = absi(dx.y);
  //
  if (x < 5 or y < 5)
    sc = maxRange;
  else
  {  // new height of image in meters -- zooming in
    if (x/double(img->width()) > y/double(img->height()))
      // use width
      sc = x / double(ppm) * double(img->height()) / double(img->width());
    else
      // use height
      sc = y / double(ppm);
  }
  if (sc < 0.2)
    sc = 0.2;
  //
  // new centre of display
  nc.x = pix.x + dx.x / 2;
  nc.y = pix.y + dx.y / 2;
  mousePan(nc, sc);
}
