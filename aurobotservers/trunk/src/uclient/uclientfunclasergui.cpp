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
#ifdef OPENCV2
#include <highgui/highgui_c.h>
#else
#include <opencv/highgui.h>
#endif

#include <ugen4/uimage2.h>
#include <ucam4/ucampwc.h>
#include <ugen4/ucampar.h>
#include <umap4/umanarc.h>
#include <umap4/umanseq.h>

#include "uclientfunclasergui.h"

/**
Position of laser in robot coordinate system */
//const double LASER_POSX = 0.44; // m


UClientFuncLaserGui::UClientFuncLaserGui()
 : UClientFuncLaser()
{
  img = NULL;
  updates = 0;
  //MaxRange = 12.8; // meter
  //StartPos = 3.0;
  MaxRange = 9.5; // meter
  StartPos = 1.7;
  saveImages = false; // do not save all images
  scannerWindowOK = false;
  setPaintVar("all", "true");
  paintBold = false;
  paintPlan = true;
  paintPathAll = true;
  paintPathLinesAll = false;
  paintVisPoly = true;
  serverNamespaceValue = NAMESPACE_MMRD;
  obsts = NULL;
  paintPathHistCnt = 0;
  paintScanHistCnt = 5;
  paintVisPolyCnt = 1;
  paintGridSize = 1.0;
  paintGridOdo = true;
  paintMmr = true;
  sfPool = NULL;
}

//////////////////////////////////////////

UClientFuncLaserGui::~UClientFuncLaserGui()
{
}

/////////////////////////////////////////

void UClientFuncLaserGui::clear()
{
  scanHist.clear();
  scannerWindowOK = false;
  sfPool->clear();
  poseHistCnt = 0;
}

/////////////////////////////////////////

bool UClientFuncLaserGui::paintNewestScan()
{
  if (scanHist.getScansCnt() > 0)
    return doRepaint(scanHist.getNewest());
  else
    return doRepaint(NULL);
}

/////////////////////////////////////////

bool UClientFuncLaserGui::gotNewData(ULaserDataSet * scan)
{
  repaint();
  return true;
}

/////////////////////////////////////////

bool UClientFuncLaserGui::doRepaint(ULaserDataSet * scan)
{
  bool result;
  int i, n, m;
  ULaserData * pd;
  CvPoint pixRob = {0,0}, p1, p2;
  const int w = 800;
  const int h = 600;
  double ppm = double(h) / MaxRange;
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
  const int MSL = 100;
  char s[MSL];
  int pz = 3; // point size (radius in pixels)
  int lw = 1; // line width
  UPose lastPose;
  UPose seenFromPose;
  ULaserPathResult * usedPath = NULL;
  UProbPoly * poly;
  UMatrix4 mLtoR;
  UPosition posr;
  //const int MTL = 30;
  //char text[MTL];
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
  if (MaxRange > 30)
    pz = 1;
  else if (MaxRange > 20)
    pz = 2;
  if (paintBold)
  {
    pz *= 2;
    lw = 2;
  }
  //
  if (img == NULL)
  {
    img = new UImage800();
    if (img != NULL)
    {
      img->setSize(h, w, 3, 8, "BGR");
      img->clear(255);
      printf("Created new image\n");
    }
  }
  if (not scannerWindowOK)
  {
    cvNamedWindow("scan", CV_WINDOW_AUTOSIZE);
    scannerWindowOK = true;
  }
  result = (img != NULL);
  // update new laser measurements
  // laser data comes before other data
  if (result)
  {
    if (saveImages)
      // save before clear
      saveImage();
    //
    img->clear(255);
    //
    // get perspective reference
    if (scan == NULL)
      lastPose = odoPose;
    else
    {
      if (odoPose.t > scan->getScanTime())
        lastPose = odoPose;
      else
      {
        lastPose = scan->getPose();
        odoPose.setPt(lastPose, scan->getScanTime());
      }
    }
    seenFromPose = lastPose;
    //
    if (paintGridOdo)
    {
      paintOdoGrid(paintGridSize, false);
      paintOdoGrid(paintGridSize * 10.0, true);
    }
    // robot position
    pixRob.y = img->height() - roundi((StartPos) * ppm);
    pixRob.x = img->width()/2;
    paintRangeRings(pixRob, *laserPose.getPos());
    // paint statistics data window
    if (scan != NULL)
    {
      if (scan->isStatValid() and paintCurves)
        paintScanStatData(scan, false, true, false);
    }
    //
    //
    if (scan != NULL)
    { // paint history scan data
      n = mini(paintScanHistCnt, scanHist.getScansCnt());
      if ((odoPose.t - scan->getScanTime()) < 0.1)
        m = 1; // new scan is aviailable, so first scan is not history
      else
        m = 0; // all scans are history
      for (i = m; i < n; i++)
      {
        histScan = scanHist.getScan(i);
        if (histScan == NULL)
          break;
        paintHistScan(img, histScan, seenFromPose, &lastPose);
      }
    }
    // paint robot history path
    n = mini(paintPathHistCnt, scanHist.getScansCnt());
    for (i = 1; i < n; i++)
    {
      histScan = scanHist.getScan(i);
      if (histScan == NULL)
        break;
      paintPostHistLine(img, histScan, seenFromPose, &lastPose);
    }
    //printf("UClientFuncLaserGui::gotNewData - has %d points to show\n", scan->getCount());
    // paint newest scan
    if (scan != NULL)
    { // a scan is available, but may be old
      if ((odoPose.t - scan->getScanTime()) < 0.1)
      { // this is a new scan, so paint
        // paint scan positions
        pd = scan->getData();
        mLtoR = scan->getSensorPose()->getRtoMMatrix();
        // robot position
        //cosTilt = cos(scan->getLaserTilt());
        for (i = 0; i < scan->getCount(); i++)
        {
          posr = pd->getPosition(&mLtoR);
          //x = pd->getDistance() * cos(pd->getAngle() * cosTilt); // forward
          //y = pd->getDistance() * sin(pd->getAngle()); // right
          p2.x = pixRob.x - roundi(posr.y * ppm);
          p2.y = pixRob.y - roundi(posr.x * ppm);
          //cvLine(img->cvArr(), p1, p2, cyan, 1, 8, 0);
          if (pd->inZoneA() and pd->inZoneB())
            paintCross(img, p2.x, p2.y, pz+1, redMag, lw);
            //cvCircle(img->cvArr(), p2, pz, redMag, 1, 8, 0);
          else if (pd->inZoneA())
            paintCross(img, p2.x, p2.y, pz+1, orange, lw);
            //cvCircle(img->cvArr(), p2, pz, orange, 1, 8, 0);
          else if (pd->inZoneB())
            //cvCircle(img->cvArr(), p2, pz, yellow, 1, 8, 0);
            paintCross(img, p2.x, p2.y, pz+1, yellow, lw);
          else if (pd->isDazzled())
            paintCross(img, p2.x, p2.y, pz+1, magenta, lw);
          else
            cvCircle(img->cvArr(), p2, pz, green, lw, 8, 0);
          pd++;
        }
      }
    }
    //snprintf(text, MTL, "Updates %5d", updates);
    //cvPutText(img->cvArr(), text, cvPoint(10, 30), &font, black);
  }
  if (paintObst)
  {
    for (n = 0; n < obsts->getGrpsCnt(); n++)
      paintObstGrp(img, obsts->getGroup(n), seenFromPose, n);
  }
  if (paintVisPoly)
  { // paint vision polygon data
    if (freePolyNewest >= 0)
    {
      poly = &freePoly[freePolyNewest];
      poly--;
      n = mini(paintVisPolyCnt, freePolyCnt);
      // paint newest only
      for (i = 1; i < n; i++)
      { // paint first history
        if (poly < freePoly)
          poly = &freePoly[freePolyCnt - 1];
        paintFreePoly(poly, scan->getPose(), true);
        poly--;
      }
      // then paint newest polygon (again)
      poly = &freePoly[freePolyNewest];
      paintFreePoly(poly, scan->getPose(), false);
    }
  }
  if (result)
  {
    paintOdoData(seenFromPose);
    if (paintPlan)
      paintPlannerData(paintVar);
    if (paintGPS and not paintBold)
      paintEkfData();
  }
  // paint interval lines and scan number
  p1.x = img->width()/2 + 30;
  p1.y = 20 + 13;
  if (paintBold and paintPlan)
    p1.y += 20;
  if (paintCurves)
    p1.y += img->height()/5;
  if (result and paintIntervalLines and (scan != NULL))
  { // paint other new stuff
    if (scan->getPisCnt() > 0)
      paintPis(img, scan, seenFromPose, 0);
  }
  if (result and pathsCnt > 0)
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
  }
  if (result)
    paintLineSegments(seenFromPose);
    //
  if (result and paintPathLines)
  {
    paintOdoDest(img, seenFromPose);
  }
  if (scan != NULL)
  { // paint scannumber
    if (result and (paintBold or not paintIntervalLines))
    { // paint just scannumber (not number of intervals found)
      if (not paintPlan)
        p1.x = 10;
    }
    if (not scan->isStatValid() or not paintCurves)
      p1.y = 13;
    snprintf(s, MSL, "#%u", scan->getSerial());
    cvPutText(img->cvArr(), s, p1, &font, black);
  }
  // paint robot
  if (paintMmr)
    paintRobotMmr(pixRob);
  else
    paintRobotSmr(pixRob);
  // paint speed next to robot
  if (paintSpeed)
    paintWpc(&wpcData);
    //
  if (img != NULL)
  { // send to display handler
    cvShowImage("scan", img->cvArr());
    cvWaitKey(7); // parameter is service time in ms
    updates++;
  }
  else
    printf("UClientFuncLaserGui::gotNewData no image handle!\n");
  // debug
  //printf("Update tool %f seconds\n", t1.getTimePassed());
  //t1.Now();
  // debug end
  //
  return result;
}

///////////////////////////////////////

void UClientFuncLaserGui::paintCross(UImage * img, int x, int y, int size, CvScalar col, int lineWidth)
{
  //size /= 2;
  cvLine(img->cvArr(), cvPoint(x-size, y-size),
            cvPoint(x+size, y+size), col, lineWidth, 4, 0);
  cvLine(img->cvArr(), cvPoint(x+size, y-size),
            cvPoint(x-size, y+size), col, lineWidth, 4, 0);
}

///////////////////////////////////////

void UClientFuncLaserGui::paintPose(UImage * img, int x, int y, double h, int size, CvScalar col, int lineWidth)
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

void UClientFuncLaserGui::paintPostHistLine(UImage * img,
                                        ULaserDataSet * scan,
                                        UPose seenFromPose,
                                        UPose * lastPose)
{
  //int i;
  CvPoint pr, p1, p2;
  UPosition posm, posr, pos;
  //ULaserData * pd;
  double ppm;
  CvScalar red = CV_RGB(255, 0, 0);
/*  CvScalar lred = CV_RGB(180, 80, 80);
  CvScalar lgreen = CV_RGB(150, 210, 150);
  CvScalar lyellow = CV_RGB(180, 80, 80);
  CvScalar lmagenta = CV_RGB(155, 0, 155);*/
/*  UPixel pixGray(100, 100, 100);
  UPixel pixGreen(150, 210, 150);*/
  UPose poseScan;
  int pz = 2; // radius of circles
  int lw = 1;
  //double cosLaserTilt;
  //
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
  if (MaxRange > 15.0)
    pz = 1;
  if (paintBold)
  {
    lw = 2;
  }
  //
  //pd = scan->getData();
  poseScan = scan->getPose();
  //cosLaserTilt = cos(scan->getLaserTilt());
  // paint path history line
  // robot pos at this scan
  pos = seenFromPose.getMapToPose(poseScan.getPos());
  p1.y = pr.y - roundi(pos.x * ppm);
  p1.x = pr.x - roundi(pos.y * ppm);
  // robot pos at last scan (newer)
  pos = seenFromPose.getMapToPose(lastPose->getPos());
  p2.y = pr.y - roundi(pos.x * ppm);
  p2.x = pr.x - roundi(pos.y * ppm);
  cvCircle(img->cvArr(), p1, pz, red, lw, 8, 0);
  cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);
  // save last painted pose position
  *lastPose = poseScan;
}

/////////////////////////////////////////////

void UClientFuncLaserGui::paintHistScan(UImage * img,
                                ULaserDataSet * scan,
                                UPose seenFromPose,
                                UPose * lastPose)
{
  int i;
  CvPoint pr, p1; //, p2;
  UPosition posm, posr, pos;
  ULaserData * pd;
  double ppm;
  //CvScalar red = CV_RGB(255, 0, 0);
  CvScalar lred = CV_RGB(180, 80, 80);
  CvScalar lgreen = CV_RGB(150, 210, 150);
  CvScalar lyellow = CV_RGB(180, 80, 80);
  CvScalar lmagenta = CV_RGB(155, 0, 155);
  UPixel pixGray(100, 100, 100);
  UPixel pixGreen(150, 210, 150);
  UPose poseScan;
  int pz = 2; // radius of circles
  int lw = 1;
  //double cosLaserTilt;
  UMatrix4 mLtoR;
  //
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
  if (MaxRange > 15.0)
    pz = 1;
  if (paintBold)
  {
    lw = 2;
  }
  //
  pd = scan->getData();
  poseScan = scan->getPose();
  //cosLaserTilt = cos(scan->getLaserTilt());
  // paint path history line
  // robot pos at this scan
/*  pos = seenFromPose.getMapToPose(poseScan.getPos());
  p1.y = pr.y - roundi(pos.x * ppm);
  p1.x = pr.x - roundi(pos.y * ppm);
  // robot pos at last scan (newer)
  pos = seenFromPose.getMapToPose(lastPose->getPos());
  p2.y = pr.y - roundi(pos.x * ppm);
  p2.x = pr.x - roundi(pos.y * ppm);
  cvCircle(img->cvArr(), p1, pz, red, lw, 8, 0);
  cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);*/
  // save last painted pose position
  //*lastPose = poseScan;
  //
  // make a transformation matrix from laser coordinates to robot coordinates.
  mLtoR = scan->getSensorPose()->getRtoMMatrix();

  for (i = 0; i < scan->getCount(); i++)
  {
    if ((pd->getDistance() < 8.0)) // or (pz > 1))
    {
      //posr = pd->getPosition(laserPos, cosLaserTilt);
      posr = pd->getPosition(&mLtoR);
      posm = poseScan.getPoseToMap(posr);
      pos = seenFromPose.getMapToPose(posm);
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      //cvLine(img->cvArr(), p1, p2, cyan, 1, 8, 0);
      if (pz > 1)
      {
        if (pd->inZoneA())
          cvCircle(img->cvArr(), p1, pz, lred, lw, 8, 0);
        else if (pd->inZoneB())
          cvCircle(img->cvArr(), p1, pz, lyellow, lw, 8, 0);
        else if (pd->isDazzled())
          cvCircle(img->cvArr(), p1, pz, lmagenta, lw, 8, 0);
        else
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

///////////////////////////////////////

bool UClientFuncLaserGui::paintRangeRings(CvPoint robPos, UPosition devPos)
{
  CvPoint p1, pr;
  double ppm;
  CvFont font;
  CvScalar lblue = CV_RGB(200, 200 , 250);
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lblack = CV_RGB(100, 100, 100);
  const int MTL = 30;
  char text[MTL];
  int i;
  bool result;
/*  const double ROBOT_WIDTH = 0.62;
  const double ROBOT_BODY_W = 0.40;
  const double ROBOT_LENGTH = 0.70;
  const double WHEEL_DIAM = 0.35;*/
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

  result = (img != NULL);
  if (result)
  { // paint laser range rings
    ppm = img->height() / MaxRange;
    p1.y = robPos.y - roundi(devPos.x * ppm);
    p1.x = robPos.x - roundi(devPos.y * ppm);
    pr = p1;  // position of laser scanner
    // paint laser position
    cvCircle(img->cvArr(), p1, 3, lblue, 1, 4, 0);
    // paint range rings
    for (i = 1; i <= 8; i++)
    {
       //cvEllipse( CvArr* img, CvPoint center, CvSize axes, double angle,
       //         double start_angle, double end_angle, CvScalar color,
       //         int thickness=1, int line_type=8, int shift=0 );

      cvEllipse(img->cvArr(), p1,
                cvSize(roundi(ppm * i), roundi(ppm * i)),
                0.0, 0.0, 180.0, lblue, 1, 4, 0);
      //cvCircle(img->cvArr(), p1, roundi(ppm * i), lblue, 1, 4, 0);
      if (MaxRange < 11)
      {
        snprintf(text, MTL, "%dm", i);
        cvPutText(img->cvArr(), text, cvPoint(p1.x + roundi(ppm * i)-15, p1.y+15), &font, lblack);
      }
      else if (MaxRange < 25)
      {
        snprintf(text, MTL, "%d", i);
        cvPutText(img->cvArr(), text, cvPoint(p1.x + roundi(ppm * i)-5, p1.y+15), &font, lblack);
      }
    }
/*    p2.x = p1.x;
    p2.y = 0;
    cvLine(img->cvArr(), p1, p2, lblack, 1, 4, 0);*/
    // paint bumper
    // cvEllipse(img->cvArr(), pr,
    //            cvSize(roundi(ROBOT_BODY_W/2.0 * ppm), roundi(ROBOT_BODY_W/2.0 * ppm)),
    //            0.0, 0.0, 180.0, lblack, 2, 4, 0);
/*    // paint robot body
    // bottom left corner (pr=laser scanner pos)
    p1.x = pr.x - roundi(ROBOT_BODY_W/2.0 * ppm);
    p1.y = pr.y + roundi(ROBOT_LENGTH * ppm);
    p2 = p1;
    // top - right
    p2.x += roundi(ROBOT_BODY_W * ppm);
    p2.y -= roundi(ROBOT_LENGTH * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // left wheel
    p1.x -= roundi((ROBOT_WIDTH - ROBOT_BODY_W)/2.0 * ppm);
    p2.x = p1.x + roundi(0.1 * ppm);
    p2.y += roundi((laserPos.x - WHEEL_DIAM/2.0) * ppm);
    p1.y = p2.y + roundi(WHEEL_DIAM * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // right wheel
    p1.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    p2.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);*/
  }
  return result;
}

//////////////////////////////////////////////

bool UClientFuncLaserGui::paintRobotMmr(CvPoint pixRob)
{
  CvPoint p1, p2, pr;
  double ppm;
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
    ppm = img->height() / MaxRange;
    p1.y = pixRob.y - roundi(laserPose.getX() * ppm);
    p1.x = pixRob.x;
    pr = p1;  // position of laser scanner
    // bottom left corner (pr=laser scanner pos)
    p1.x = pr.x - roundi(ROBOT_BODY_W/2.0 * ppm);
    p1.y = pr.y + roundi(ROBOT_LENGTH * ppm);
    p2 = p1;
    // top - right
    p2.x += roundi(ROBOT_BODY_W * ppm);
    p2.y -= roundi(ROBOT_LENGTH * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // left wheel
    p1.x -= roundi((ROBOT_WIDTH - ROBOT_BODY_W)/2.0 * ppm);
    p2.x = p1.x + roundi(0.1 * ppm);
    p2.y += roundi((laserPose.getX() - WHEEL_DIAM/2.0) * ppm);
    p1.y = p2.y + roundi(WHEEL_DIAM * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // right wheel
    p1.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    p2.x += roundi((ROBOT_WIDTH - 0.1) * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
  }
  return result;
}

///////////////////////////////////////////////

bool UClientFuncLaserGui::paintRobotSmr(CvPoint pixRob)
{
  CvPoint p1, p2, pr;
  double ppm;
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
    ppm = img->height() / MaxRange;
    p1.y = pixRob.y - roundi(laserPose.getX() * ppm);
    p1.x = pixRob.x;
    pr = p1;  // position of laser scanner
    // bottom left corner (pr=laser scanner pos)
    p1.x = pr.x - roundi(ROBOT_BODY_W/2.0 * ppm);
    p1.y = pr.y + roundi(ROBOT_LENGTH * ppm);
    p2 = p1;
    // top - right
    p2.x += roundi(ROBOT_BODY_W * ppm);
    p2.y -= roundi(ROBOT_LENGTH * ppm);
    cvRectangle(img->cvArr(), p1, p2, lblack, 2, 8, 0);
    // left wheel
    p1.x -= roundi((ROBOT_WIDTH - ROBOT_BODY_W)/2.0 * ppm);
    p2.x = p1.x + roundi(0.1 * ppm);
    p2.y += roundi((laserPose.getX() - WHEEL_DIAM/2.0) * ppm);
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

bool UClientFuncLaserGui::paintWpc(ULaserWpc * wpc)
{
  CvPoint p1, p2, pr;
  double ppm;
  CvFont font;
  CvScalar red = CV_RGB(255, 0 , 0);
  const int MTL = 30;
  char text[MTL];
  bool result;
  const double ROBOT_WIDTH = 0.62;
  UPlannerValue * pv;
  //
  if (paintBold)
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
              1.0, 1.0, 0.0, 2, 8);
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               1.0, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result and (MaxRange < 35.0))
  { // paint wheel speed
    ppm = img->height() / MaxRange;
    pr.y = img->height() - roundi(ppm * StartPos);
    pr.x = img->width()/2;
    // position of left speed text
    p1.x = pr.x - roundi(ROBOT_WIDTH/2.0 * ppm) - 50;
    p1.y = pr.y - 3;
    // position of right speed text
    p2.x = pr.x + roundi(ROBOT_WIDTH/2.0 * ppm);
    p2.y = pr.y -3;
    //
    if (serverNamespaceValue == NAMESPACE_MMRD)
    {
      pv = planner.findValue("odoSpeed");
      if (pv != NULL)
      {
        if (paintBold)
          p2.y += 13;
        snprintf(text, MTL, "%5.2fm/s", pv->getValue());
        cvPutText(img->cvArr(), text, p2, &font, red);
        p2.y += 13;
        if (paintBold)
          p2.y += 13;
        snprintf(text, MTL, "%5.2fkm/h", pv->getValue() * 3.6);
        cvPutText(img->cvArr(), text, p2, &font, red);
      }
    }
    else
    {
      snprintf(text, MTL, "%5.2f", wpc->motorLeft);
      cvPutText(img->cvArr(), text, p1, &font, red);
      snprintf(text, MTL, "%5.2fm/s", wpc->motorRight);
      cvPutText(img->cvArr(), text, p2, &font, red);
    }
  }
  return result;
}

////////////////////////////////////////////////////

bool UClientFuncLaserGui::paintEkfData()
{
  CvPoint p1;
  CvFont font;
  CvScalar green = CV_RGB(0, 100 , 0);
  const int MSTL = 30;
  char st[MSTL];
  const int MTL = 130;
  char text[MTL];
  bool result;
  double head;
  //
  if (paintBold)
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
                 1.0, 1.0, 0.0, 2, 8);
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              1.0, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result)
  { // EKF position
    p1.x = img->width() / 2 + 5;
    p1.y = img->height() - 18;
    ekfTime.getTimeAsString(st, true);
    snprintf(text, MTL, "EKF %s, %dupd, %dodo",
               st, ekfuUsed, ekfuSkipd);
    cvPutText(img->cvArr(), text, p1, &font, green);
    p1.y += 15;
    // convert heading to compas degrees
    head = 90.0 - ekfPose.h * 180.0 / M_PI;
    if (head < 0.0)
      head += 360.0;
    snprintf(text, MTL, "%.3fE, %.3fN, %.1fdeg (compas)",
               ekfPose.x, ekfPose.y, head);
    cvPutText(img->cvArr(), text, p1, &font, green);
  }
  return result;
}

////////////////////////////////////////////////////

bool UClientFuncLaserGui::paintOdoData(UPose seenFromPose)
{
  CvPoint p1, p2, pr;
  CvFont font;
  CvScalar black = CV_RGB(0, 0 , 0);
  const int MTL = 130;
  char text[MTL];
  const int MSL = 30;
  char t1[MSL];
  char t2[MSL];
  bool result;
  UPose pose;
  ULaserDataSet * scan;
  UPoseTime * pp;
  double ppm;
  UPosition pos;
  int i, n;
  int lw = 1;
  UTime t;
  //
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
  //
  if (paintBold)
  {
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
                 1.0, 1.0, 0.0, 2, 8);
    lw = 2;
  }
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              1.0, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result)
  { // paint wheel speed
    p1.x = 10;
    p1.y = img->height() - 18;
    // of no odo-data use newest scan-position
    pose = odoPose.getPose();
    if (odoPose.t.GetSec() < 1000000)
    { // use pose from last scan
      scan = scanHist.getScan(0);
      if (scan != NULL)
        pose = scan->getPose();
    }
    // print odometer position
    if (odoPose.t.getDecSec() > 1000.0)
    { // odopose is available
      if (paintBold)
        p1.y -= 15;
      snprintf(text, MTL, "odoPos %6.2fx, %6.2fy, %6.1fdeg (mat)",
                pose.x, pose.y, pose.h * 180.0 / M_PI);
      cvPutText(img->cvArr(), text, p1, &font, black);
      // print odometer time
      p1.y += 15;
      if (paintBold)
        p1.y += 13;
    }
    // paint time
    if (odoPose.t.getDecSec() < 1000.0)
    { // no timestamp available - use current time
      t.Now();
      t.getDateString(t1, true);
      t.getTimeAsString(t2, true);
    }
    else
    { // timestamp is available
      odoPose.t.getDateString(t1, true);
      odoPose.t.getTimeAsString(t2, true);
    }
    snprintf(text, MTL, "%s %s", t1, t2);
    cvPutText(img->cvArr(), text, p1, &font, black);
    // paint also odometry history
    p1 = pr; // start at robot position
    n = poseHistNewest;
    for (i = 0; i < poseHistCnt; i++)
    {
      pp = &poseHist[n--];
      if (n < 0)
        n = poseHistCnt - 1;
      p2 = p1;
      pos = seenFromPose.getMapToPose(pp);
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      cvLine(img->cvArr(), p1, p2, black, lw, 8, 0);
      pp++;
    }
  }
  return result;
}

////////////////////////////////////////////////////

bool UClientFuncLaserGui::paintScanStatData(ULaserDataSet * scan,
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
  ULaserData * lp;
  //const double maxVar = 0.02;  // top limit
  const double maxVarLR = 0.2;  // top limit of SD of variance to the left
//  const double maxEdge = 0.4;
//  const double maxCurv = M_PI/2.0;
  const double maxTilt = M_PI/2.0;
  const double maxX = 4.0;
  UPosition pos;
  UPosition posZero(0.0, 0.0, 0.0);
  double cosLaserTilt;
  int txtLineH = 12;
  CvPoint piL, piR;
  //
  if (paintBold)
  {
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
                 1.0, 1.0, 0.0, 1, 8);
    txtLineH = 17;
  }
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              1.0, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result)
  { // get position of varianve curve
    topLeft.y = 13 + 5;
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
      cvPutText(img->cvArr(), "-sdLeft", p1, &font, red);
      p1.y += 12;
/*      cvPutText(img->cvArr(), "-sdLC", p1, &font, brown);
      p1.y += 12;*/
      p2.x = topLeft.x - 37;
      if (paintBold)
        p2.x -= 37;
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
      cvPutText(img->cvArr(), "-tilt", p1, &font, red);
      p1.y += 12;
    }
    if (true)
    {
      cvPutText(img->cvArr(), "-X", p1, &font, black);
      p1.y += 12;
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
        if (lp->inZoneA() or lp->inZoneB())
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

bool UClientFuncLaserGui::paintPlannerData(bool paintAll)
{
  CvPoint p1;
  CvFont font;
  //CvScalar bblue = CV_RGB(0, 0 , 100);
  CvScalar brown = CV_RGB(100, 100 , 0);
  CvScalar red = CV_RGB(255, 0 , 0);
  CvScalar black = CV_RGB(0, 0 , 0);
  CvScalar col;
  bool result;
  UPlannerStopCrit * stopCrit;
  const int MSL = 240;
  char s[MSL];
  int i, n;
  bool roadWidthDo = false;
  bool roadLeftDo = false;
  bool roadRightDo = false;
  bool gpsDistDo = false;
  bool skip;
  UPlannerValue * var;
  int col1, col2, col3, col4w;
  UTime t;
  //
  // <planget planner=true file="mission.txt" usingFile=true
  //  cmd="gotowaypoint odo 10.0 0.0  (get GPS)" isIdle=false/>
  if (paintBold)
    cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
                 1.0, 1.0, 0.0, 2, 8);
  else
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              1.0, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result)
  { // paint planner main status
    p1.x = 10;
    p1.y = 2;
    if (paintBold)
      p1.y += 10;
    // current command line
    p1.y += 13;
    if (planner.cmdLineOverride)
      snprintf(s, MSL, "MAN: %s", planner.cmdLine);
    else
      snprintf(s, MSL, "#%d: %s", planner.cmdLineNum, planner.cmdLine);
    cvPutText(img->cvArr(), s, p1, &font, red);
    //
  }
  if (result and paintAll)
  {
    if (planner.simulated)
    { // print simulation subdir for idenfification on images
      p1.y += 13;
      snprintf(s, MSL, "simulated from: %s", planner.simSubDir);
      cvPutText(img->cvArr(), s, p1, &font, brown);
    }
    // print planner filename
    p1.y += 13;
    snprintf(s, MSL, "mission file: %s", planner.cmdFile);
    cvPutText(img->cvArr(), s, p1, &font, brown);
    // print planner command line (last line first)
    if (strcmp(planner.cmdLineLast, "none") != 0)
    {
      p1.y += 13;
      snprintf(s, MSL, "last: %s", planner.cmdLineLast);
      cvPutText(img->cvArr(), s, p1, &font, brown);
    }
    // stop reason
    if (strcmp(planner.cmdStoppedBy, "none") != 0)
    {
      p1.y += 13;
      snprintf(s, MSL, "Stopped by '%s'", planner.cmdStoppedBy);
      cvPutText(img->cvArr(), s, p1, &font, brown);
    }
    // Speed info
    if (serverNamespaceValue == NAMESPACE_MMRSR2)
    {
      p1.y += 18;
      snprintf(s, MSL, "Speed: %.2fm/s (%.1fkm/h) planned %.2fm/s (%.1fkm/h)",
        wpcData.speedCurrent, wpcData.speedCurrent * 3.6,
        wpcData.speedPlanned, wpcData.speedPlanned * 3.6);
      cvPutText(img->cvArr(), s, p1, &font, black);
      //
      p1.y += 18;
      snprintf(s, MSL, "Speed advice: %.2f",
        wpcData.speedAdvice);
      cvPutText(img->cvArr(), s, p1, &font, black);
    }
    //
    p1.y += 18;
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              0.9, 1.0, 0.0, 1, 8);
    // either display old stop-crits, or some new variables
    if (serverNamespaceValue == NAMESPACE_MMRD)
    {
      var = planner.vars;
      col1 = 10;
      col2 = col1 + 50;
      col3 = col2 + 65;
      col4w = 75;
      for (i = 0; i < planner.varsCnt; i++)
      {
        p1.x = col1;
        cvPutText(img->cvArr(), var->getName(), p1, &font, brown);
        switch (var->getType())
        {
        case VALTYP_D:
          p1.x = col3;
          snprintf(s, MSL, "%8.3f", var->getValue());
          cvPutText(img->cvArr(), s, p1, &font, brown);
          break;
        case VALTYP_DS:
          p1.x = col3;
          snprintf(s, MSL, "%7.2fm/s", var->getValue());
          cvPutText(img->cvArr(), s, p1, &font, brown);
          p1.x += col4w;
          snprintf(s, MSL, "%6.1fkm/h", var->getValue() * 3.6);
          cvPutText(img->cvArr(), s, p1, &font, brown);
          break;
        case VALTYP_DQ:
          p1.x = col3;
          snprintf(s, MSL, "%8.3f", var->getValue());
          cvPutText(img->cvArr(), s, p1, &font, brown);
          p1.x += col4w;
          snprintf(s, MSL, "%8.3fq", var->getQuality());
          cvPutText(img->cvArr(), s, p1, &font, brown);
          p1.x += col4w;
          snprintf(s, MSL, "(%s)", bool2str(var->isValid()));
          cvPutText(img->cvArr(), s, p1, &font, brown);
          break;
        case VALTYP_TIME:
          p1.x = col3;
          t.setTime(var->getValue());
          t.getTimeAsString(s, true);
          cvPutText(img->cvArr(), s, p1, &font, brown);
          break;
        case VALTYP_3D:
          p1.x = col3;
          for (n = 0; n < 3; n++)
          {
            snprintf(s, MSL, "%8.3f", var->getValue(n));
            cvPutText(img->cvArr(), s, p1, &font, brown);
            p1.x += col4w;
          }
          break;
        case VALTYP_POSE:
          p1.x = col3;
          for (n = 0; n < 2; n++)
          {
            snprintf(s, MSL, "%8.3f", var->getValue(n));
            cvPutText(img->cvArr(), s, p1, &font, brown);
            p1.x += col4w;
          }
          snprintf(s, MSL, "%7.1fdeg", var->getValue(2) * 180.0 / M_PI);
          cvPutText(img->cvArr(), s, p1, &font, brown);
          break;
        default:
          break;
        }
        p1.y += 14;
        var++;
      }
    }
    else
    {
      stopCrit = planner.stopCrit;
      for (i = 0; i < planner.stopCritCnt; i++)
      {
        skip = false;
        if (strncasecmp(stopCrit->name, "roadLeft", 8) == 0)
        { // paint only one road width status
          if (not (stopCrit->active or roadLeftDo))
          {
            skip = true;
            roadLeftDo = true;
          }
        }
        if (strncasecmp(stopCrit->name, "roadRight", 8) == 0)
        { // paint only one road width status
          if (not (stopCrit->active or roadRightDo))
          {
            skip = true;
            roadRightDo = true;
          }
        }
        if (strncasecmp(stopCrit->name, "roadWidth", 8) == 0)
        { // paint only one road width status
          if (not (stopCrit->active or roadWidthDo))
          {
            skip = true;
            roadWidthDo = true;
          }
        }
        else if (strncasecmp(stopCrit->name, "gpsDist", 7) == 0)
        {
          if (not (stopCrit->active or gpsDistDo))
          {
            skip = true;
            gpsDistDo = true;
          }
        }

        if (not skip)
        {
          p1.x = 10;
          if (stopCrit->active)
            col = red;
          else
            col = brown;
          cvPutText(img->cvArr(), stopCrit->name, p1, &font, col);
          p1.x = 125;
          cvPutText(img->cvArr(), stopCrit->status, p1, &font, col);
          p1.y += 13;
        }
        stopCrit++;
      }
    }
  }
  return result;
}

////////////////////////////////////////////////////

bool UClientFuncLaserGui::paintPathData(ULaserPathResult * path, int num)
{
  CvPoint pr, p1, p2;
  CvFont font;
  bool result;
  UPosition pos, *ppos;
  double ppm;
  ULineSegment * seg;
  CvScalar red;
  CvScalar green;
  CvScalar rtCol;
  //CvScalar yellow = CV_RGB(150, 150, 0);
  CvScalar blue = CV_RGB(0, 0, 255);
  //CvScalar magenta = CV_RGB(150, 0, 150);
  int i, j, n, redWidth;
  int lw = 1;
  UManSeq * man;
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
  //
  if (path->isPathUsed())
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

  if (path->isPassingAWall())
    // paint red as black when route passes a wall
    red = CV_RGB(0, 0, 0);
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              0.8, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result)
  {
    ppm = img->height() / MaxRange;
    pr.y = img->height() - roundi(ppm * StartPos);
    pr.x = img->width()/2;
    if (path->isPathUsed() and paintRoadLines)
    {
      // left edge (old mmrd stuff)
      if (path->getEdgeLeftValid())
      {
        seg = path->getEdgeLeft();
        pos = odoPose.getMapToPose(seg->pos);
        p1.y = pr.y - roundi(pos.x * ppm);
        p1.x = pr.x - roundi(pos.y * ppm);
        pos = odoPose.getMapToPose(seg->getOtherEnd());
        p2.y = pr.y - roundi(pos.x * ppm);
        p2.x = pr.x - roundi(pos.y * ppm);
        cvLine(img->cvArr(), p1, p2, green, lw);
      }
      // right edge (old mmrd stuff)
      if (path->getEdgeRightValid())
      {
        seg = path->getEdgeRight();
        pos = odoPose.getMapToPose(seg->pos);
        p1.y = pr.y - roundi(pos.x * ppm);
        p1.x = pr.x - roundi(pos.y * ppm);
        pos = odoPose.getMapToPose(seg->getOtherEnd());
        p2.y = pr.y - roundi(pos.x * ppm);
        p2.x = pr.x - roundi(pos.y * ppm);
        cvLine(img->cvArr(), p1, p2, green, lw);
      }
      // top of road line (old mmrd stuff)
      if (path->getEdgeTopValid())
      {
        seg = path->getEdgeTop();
        pos = odoPose.getMapToPose(seg->pos);
        p1.y = pr.y - roundi(pos.x * ppm);
        p1.x = pr.x - roundi(pos.y * ppm);
        pos = odoPose.getMapToPose(seg->getOtherEnd());
        p2.y = pr.y - roundi(pos.x * ppm);
        p2.x = pr.x - roundi(pos.y * ppm);
        cvLine(img->cvArr(), p1, p2, green, lw);
      }
    }
    // passage lines (old mmrd stuff)
    //
    if ((path->isPathUsed() or paintPathLinesAll) and paintIntervalLines)
    {
      n = path->getPassLinesCnt();
/*      if (hist >= 0)
      n = mini(n, hist);*/
      for (i = 0; i < n; i++)
      {
        seg = path->getPassLine(i);
        pos = odoPose.getMapToPose(seg->pos);
        p1.y = pr.y - roundi(pos.x * ppm);
        p1.x = pr.x - roundi(pos.y * ppm);
        pos = odoPose.getMapToPose(seg->getOtherEnd());
        p2.y = pr.y - roundi(pos.x * ppm);
        p2.x = pr.x - roundi(pos.y * ppm);
        if (path->isPassingAWall())
          cvLine(img->cvArr(), p1, p2, red, 2 * lw);
        else
          cvLine(img->cvArr(), p1, p2, blue, 1 * lw);
      }
    }
    // not shown yet
    //
    if (paintPathLines)
    {
      // fount obstacle avoidance route
      // straight lines - old mmrd stuff
      if (false and path->getRouteCnt() > 0)
      {
        p1.y = pr.y;
        p1.x = pr.x;
        ppos = path->getRoute(path->getRouteCnt() - 1);
        for (i = path->getRouteCnt() - 1; i >= 0; i--)
        { // paint from robot to most recent route-point
          // converte to robot perspective
          pos = odoPose.getMapToPose(*ppos);
          // and to pixel coordinates
          p2.y = pr.y - roundi(pos.x * ppm);
          p2.x = pr.x - roundi(pos.y * ppm);
          // paint
          cvLine(img->cvArr(), p1, p2, rtCol, redWidth * lw);
          // advance to next
          p1 = p2;
          ppos--;
        }
      }
      man = path->getManSeq();
      if (man != NULL)
      {
        // debug - changing colours
        if (not path->isPathUsed())
          rtCol = CV_RGB(90, (num % 2) * 90, ((num / 2) % 3) * 90);
        // debug end
        for (i = 0; i < man->getP2PCnt(); i++)
        {
          if (path->isPathUsed() or paintPathAll)
          {
            mpp = man->getP2P(i);
            pv1 = mpp->getStartPoseV();
            pv2 = mpp->getEndPoseV();
            pos = odoPose.getMapToPose(pv1.getPos());
            // and to pixel coordinates
            p1.y = pr.y - roundi(pos.x * ppm);
            p1.x = pr.x - roundi(pos.y * ppm);
            pos = odoPose.getMapToPose(pv2.getPos());
            // and to pixel coordinates
            p2.y = pr.y - roundi(pos.x * ppm);
            p2.x = pr.x - roundi(pos.y * ppm);
            // paint direct (thin) line
            //cvLine(img->cvArr(), p1, p2, rtCol, 1);
            // now paint curved line too
            for (j = 0; j < mpp->getSeqCnt(); j++)
            {
              mm = mpp->getMan(j);
              pv2 = mm->getEndPoseV(pv1);
              switch (mm->getManType())
              {
                case UManoeuvre::MAN_ARC:
                  // convert to arc type manoeuvre
                  mma = (UManArc*)mm;
                  if (mma->getTurnRadius() < 0.0)
                  {
                    mma->setTurnAngle(-mma->getTurnAngle());
                    mma->setTurnRadius(-mma->getTurnRadius());
                  }
                  // if too big, then draw a line, too big is set to r> ~ 200 meter
                  if (mma->getTurnRadius() > (10.0 * MaxRange))
                  { // just a line from start (pv1) to finish (pv2)
                    pos = odoPose.getMapToPose(pv1.getPos());
                    // and to pixel coordinates
                    p1.y = pr.y - roundi(pos.x * ppm);
                    p1.x = pr.x - roundi(pos.y * ppm);
                    pos = odoPose.getMapToPose(pv2.getPos());
                    // and to pixel coordinates
                    p2.y = pr.y - roundi(pos.x * ppm);
                    p2.x = pr.x - roundi(pos.y * ppm);
                    // paint direct line
                    cvLine(img->cvArr(), p1, p2, rtCol, redWidth * lw);
                  }
                  else
                  { // OK, draw as arc
                    // find centre
                    if (mma->getTurnAngle() > 0.0)
                    {
                      pos2.y = mma->getTurnRadius();
                      a1 = (pv1.h - odoPose.h) * (180.0 / M_PI) - 90.0;
                      a2 = a1 + (180.0 / M_PI) * mma->getTurnAngle();
                    }
                    else
                    {
                      pos2.y = -mma->getTurnRadius();
                      a2 = (pv1.h - odoPose.h) * (180.0 / M_PI) + 90.0;
                      a1 = a2 + (180.0 / M_PI) * mma->getTurnAngle();
                    }
                    pos2.x = 0.0;
                    // in real map coordinates
                    pos1 = pv1.getPoseToMap(pos2);
                    // and back to pose coordinates
                    pos = odoPose.getMapToPose(pos1);
                    p1.y = pr.y - roundi(pos.x * ppm);
                    p1.x = pr.x - roundi(pos.y * ppm);
                    // set arc radius
                    sz.width = roundi(mma->getTurnRadius() * ppm);
                    sz.height = sz.width;
                    cvEllipse( img->cvArr(), p1, sz, 90.0,
                              a1, a2, rtCol, redWidth * lw);
                  }
                  break;
                case UManoeuvre::MAN_LINE:
                  pos = odoPose.getMapToPose(pv1.getPos());
                  // and to pixel coordinates
                  p1.y = pr.y - roundi(pos.x * ppm);
                  p1.x = pr.x - roundi(pos.y * ppm);
                  pos = odoPose.getMapToPose(pv2.getPos());
                  // and to pixel coordinates
                  p2.y = pr.y - roundi(pos.x * ppm);
                  p2.x = pr.x - roundi(pos.y * ppm);
                  // paint direct line
                  cvLine(img->cvArr(), p1, p2, rtCol, redWidth * lw);
                  break;
                case UManoeuvre::MAN_STOP:
                  break;
              }
              pv1 = pv2;
            }
          }
        }
      }
      if (true) // paintMidPoseCrosses)
      {
        ppos = path->getRoute(0);
        rtCol.val[0] = rtCol.val[0] / 2;
        rtCol.val[1] = rtCol.val[1] / 2;
        rtCol.val[2] = rtCol.val[2] / 2;
        for (i = 0; i < path->getRouteCnt(); i++)
        { // convert to pose
          pose1.set(ppos->x, ppos->y, ppos->z);
          // convert to robot coordinates
          pose1 = odoPose.getMapToPosePose(&pose1);
                    // and to pixel coordinates
          p1.y = pr.y - roundi(pose1.x * ppm);
          p1.x = pr.x - roundi(pose1.y * ppm);
          // paint cross at position
          if (path->isPathCrashed())
            cvCircle(img->cvArr(), p1, 4, rtCol, 1, 8, 0);
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
    } //////
  }
  return result;
}

////////////////////////////////////////////////////

bool UClientFuncLaserGui::paintOdoGrid(double stepSize, bool bold)
{
  CvPoint p1, p2;
  double ppm;
  //CvFont font;
  CvScalar lyellow = CV_RGB(240, 240 , 180);
  CvScalar lcyan = CV_RGB(180, 240 , 240);
  CvScalar lblue = CV_RGB(180, 180 , 250);
  CvScalar black = CV_RGB(0, 0, 0);
  int i;
  bool result;
  int mr = int(MaxRange * 1.2);
  //
  UPosition posG1, posG2, posR1, posR2;
  CvPoint pr;
  //
  if (bold)
  {
    lyellow = CV_RGB(140, 140 , 80);
    lcyan = CV_RGB(80, 140 , 140);
    lblue = CV_RGB(10, 10 , 150);
    black = CV_RGB(0, 0, 0);
  }
  // set scale and robot position
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
  //
  //cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
  //            1.0, 1.0, 0.0, 1, 8);
  result = (img != NULL);
  if (result)
  { // paint odometry grid
    posG1.set(floor(odoPose.x / stepSize) * stepSize - mr,
              floor(odoPose.y / stepSize) * stepSize - mr, 0.0);
    posG2 = posG1;
    posG2.x = posG1.x + 2.0 * mr;
    for (i = -roundi(double(mr)/stepSize); i < roundi(double(mr)/stepSize); i++)
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
    posG1.set(floor(odoPose.x) - double(mr),
              floor(odoPose.y) - double(mr), 0.0);
    posG2 = posG1;
    posG2.y = posG1.y + 2.0 * double(mr);
    for (i = -roundi(double(mr)/stepSize); i < roundi(double(mr)/stepSize); i++)
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

////////////////////////////////////////////////////

// bool UClientFuncLaserGui::paintCiData()
// {
//   bool result;
//   int i;
//   CvPoint p1, p2, pr, p3, p4;
//   double ppm;
//   CvScalar red = CV_RGB(255, 0, 0);
//   CvScalar black = CV_RGB(0, 0, 0);
//   CvScalar white = CV_RGB(255, 255, 255);
//   CvScalar cyan = CV_RGB(0, 155, 155);
//   CvScalar yellow = CV_RGB(135, 135, 20);
//   CvScalar brown = CV_RGB(100, 100, 0);
//   CvFont font;
//   const int MTL = 60;
//   char text[MTL];
//   UPosition pos;
//   UPosition pos1, pos2;
//   //
//   cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
//               1.0, 1.0, 0.0, 1, 8);
//   //
//   result = (img != NULL);
//   if (result)
//   {
//     ppm = img->height() / MaxRange;
//     pr.y = img->height() - roundi(ppm * StartPos);
//     pr.x = img->width()/2;
//     for (i = 0; i < ciCnt; i++)
//     {
//       p1.y = pr.y - roundi(ciLeft[i].x * ppm);
//       p1.x = pr.x - roundi(ciLeft[i].y * ppm);
//       p2.y = pr.y - roundi(ciRight[i].x * ppm);
//       p2.x = pr.x - roundi(ciRight[i].y * ppm);
//       cvLine(img->cvArr(), p1, p2, red, 2, 8, 0);
//       snprintf(text, MTL, "LI-%d", i);
//       cvPutText(img->cvArr(), text, p1, &font, black);
//       if (ciLeftB[i].x > 0.001)
//       {
//         p3.y = pr.y - roundi(ciLeftB[i].x * ppm);
//         p3.x = pr.x - roundi(ciLeftB[i].y * ppm);
//         p4.y = pr.y - roundi(ciRightB[i].x * ppm);
//         p4.x = pr.x - roundi(ciRightB[i].y * ppm);
//         cvLine(img->cvArr(), p3, p1, red, 1, 4, 0);
//         cvLine(img->cvArr(), p4, p2, red, 1, 4, 0);
//         // snprintf(text, MTL, "LI-%d", i);
//         // cvPutText(img->cvArr(), text, p1, &font, black);
//       }
//     }
//     for (i = 0; i < ciCorrCnt; i++)
//     {
//       pos1 = ciCorrLeft[i];
//       pos2 = ciCorrRight[i];
//       if (ciCorrScan[i] >= 0)
//       { // then it is in odoPose coordinates
//         // and must be converted to robot coordinates
//         pos1 = odoPose.getMapToPose(pos1);
//         pos2 = odoPose.getMapToPose(pos2);
//       }
//       p1.y = pr.y - roundi(pos1.x * ppm);
//       p1.x = pr.x - roundi(pos1.y * ppm);
//       p2.y = pr.y - roundi(pos2.x * ppm);
//       p2.x = pr.x - roundi(pos2.y * ppm);
//       cvLine(img->cvArr(), p1, p2, cyan, 2,8,0);
//       if (ciCorrScan[i] >= 0)
//         snprintf(text, MTL, "cs%d.%d", ciCorrScan[i], ciCorrIdx[i]);
//       else
//         snprintf(text, MTL, "CC-%d", i);
//       cvPutText(img->cvArr(), text, p1, &font, cyan);
//     }
//     for (i = 0; i < wpPosCnt; i++)
//     {
//       pos = odoPose.getMapToPose(wpPos[i]);
//       p1.y = pr.y - roundi(pos.x * ppm);
//       p1.x = pr.x - roundi(pos.y * ppm);
//       cvCircle(img->cvArr(), p1, 3, yellow, 1, 8, 0);
//       snprintf(text, MTL, "wp%d", i);
//       cvPutText(img->cvArr(), text, p1, &font, cyan);
//       // print wp-list
//       if (i < 10)
//       {
//         snprintf(text, MTL, "wp %2d %6.2fx, %6.2fy", i, wpPos[i].x, wpPos[i].y);
//         cvPutText(img->cvArr(), text, cvPoint(10, 30 + 15*i), &font, brown);
//       }
//     }
//     snprintf(text, MTL, "wp count %d:", wpPosCnt);
//     cvPutText(img->cvArr(), text, cvPoint(10, 15), &font, brown);
//     //
//     // clear odometer area
//     /*
//     cvRectangle(img->cvArr(), cvPoint(10, img->height() - 75),
//                               cvPoint(pr.x - 60, img->height()), white,
//                               CV_FILLED, 4, 0);
//     */
//     if (ciCorrCnt > 0)
//     { // then there should be a result position
//       // result is in odo coordinates, convert to robot
//       // pose perspective ...
//       pos = odoPose.getMapToPose(ciResult);
//       // ... and to pixel coordinates
//       p1.y = pr.y - roundi(pos.x * ppm);
//       p1.x = pr.x - roundi(pos.y * ppm);
//       // paint
//       cvCircle(img->cvArr(), p1, 5, cyan, 2, 8, 0);
//       // paint also a line towards goalpoiny
//       pos = odoPose.getMapToPose(ciGoal);
//       // ... and to pixel coordinates
//       p1.y = pr.y - roundi(pos.x * ppm);
//       p1.x = pr.x - roundi(pos.y * ppm);
//       // paint
//       if (pos.x > 0)
//         cvLine(img->cvArr(), pr, p1, red, 1, 8, 0);
//       else
//       {
//         p1.y = pr.y;
//         cvLine(img->cvArr(), pr, p1, red, 1, 8, 0);
//       }
//     }
//     //
//     p1.x = 10;
//     p1.y = img->height() - 48;
//     snprintf(text, MTL, "ciGuide %6.2fx, %6.2fy", ciResult.x, ciResult.y);
//     cvPutText(img->cvArr(), text, p1, &font, yellow);
//     p1.y += 15;
//     snprintf(text, MTL, "ciGoal  %6.2fx, %6.2fy", ciGoal.x, ciGoal.y);
//     cvPutText(img->cvArr(), text, p1, &font, yellow);
//
//     //cvShowImage("scan", img->cvArr());
//     //cvWaitKey(7); // parameter is service time in ms
//   }
//   return result;
// }

void UClientFuncLaserGui::paintWpListData()
{
  bool result;
  int i;
  CvPoint p1, pr;
  double ppm;
  CvScalar cyan = CV_RGB(0, 155, 155);
  CvScalar yellow = CV_RGB(180, 180, 0);
  //CvScalar brown = CV_RGB(100, 100, 0);
  CvFont font;
  const int MTL = 60;
  char text[MTL];
  UPosition pos;
  UPosition pos1, pos2;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              1.0, 1.0, 0.0, 1, 8);
  //
  result = (img != NULL);
  if (result)
  {
    ppm = img->height() / MaxRange;
    pr.y = img->height() - roundi(ppm * StartPos);
    pr.x = img->width()/2;
    //
    for (i = 0; i < wpcData.wpPosCnt; i++)
    {
      pos = odoPose.getMapToPose(wpcData.wpPos[i]);
      p1.y = pr.y - roundi(pos.x * ppm);
      p1.x = pr.x - roundi(pos.y * ppm);
      cvCircle(img->cvArr(), p1, 3, yellow, 1, 8, 0);
      snprintf(text, MTL, "wp%d", i);
      cvPutText(img->cvArr(), text, p1, &font, cyan);
      // print wp-list
/*      if (i < 10)
      {
        snprintf(text, MTL, "wp %2d %6.2fx, %6.2fy", i, wpPos[i].x, wpPos[i].y);
        cvPutText(img->cvArr(), text, cvPoint(10, 30 + 15*i), &font, brown);
      }*/
    }
  }
}

////////////////////////////////////////////////////

void UClientFuncLaserGui::doTimeTick()
{ // do the tisplay events needed to show the image
  const double WAIT_TIME_BEFORE_REPAINT = 0.15; // sec
  //
  if (repaintImage)
  {
    if (repaintImageTime.getTimePassed() > WAIT_TIME_BEFORE_REPAINT)
    {
      paintNewestScan();
      repaintImage = false;
    }
  }
  cvWaitKey(7); // parameter is service time in ms
  /*
  printf(".");
  fflush(stdout);
  */
}

////////////////////////////////////////////////////

bool UClientFuncLaserGui::saveImage()
{
  const int MSL = 300;
  char s[MSL];
  const int MTL = 30;
  char st[MTL];
  bool result = false;
  ULaserDataSet * scan;
  USFData * sfData = NULL;
  UTime t;
  //
  if (img != NULL)
  { // there is an image to save
    scan = scanHist.getScan(0);
    if (sfPool != NULL)
      sfData = sfPool->getScan(0);
    if (scan != NULL)
      snprintf(s, MSL, "%s/Laser-%s-%05u.png",
                  imagePath,
                  scan->getTime().getForFilename(st, true),
                  scan->getSerial());
    else if (sfData != NULL)
      snprintf(s, MSL, "%s/Laser-%s.png",
               imagePath,
               sfData->getScanTime().getForFilename(st, true));
    else
    {
      t.now();
      snprintf(s, MSL, "%s/Laser-%s.png",
               imagePath,
               t.getForFilename(st, true));
    }
    result = cvSaveImage(s, img->cvArr());
    if (result)
    {
      if (verboseMessages)
        printf("Saved image to %s\n", s);
    }
    else
      printf("Could not save to %s\n", s);
  }
  //
  return result;
}

////////////////////////////////////////////////////

void UClientFuncLaserGui::paintPis(UImage * img,
                              ULaserDataSet * scan,
                              UPose seenFromPose,
                              const int cnt)
{
  int i;
  CvPoint p1, p2, pr;
  UPosition pos;
  ULaserPi * pp;
  double ppm;
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
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
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
    if (MaxRange < 20)
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
/*    if (cnt == 0)
    { // current scan - so also text
      p1.x = img->width() - 270;
      p1.y = top + i * 13;
      pos = pp->getLeftPos();
      snprintf(text, MTL, "scan %u pi w=%.2fm top=%.2fy",
        scan->getSerial(),
        pp->getRightPos().dist(pos),
        pp->getTopPos().y);
      cvPutText(img->cvArr(), text, p1, &font, mag);
    }*/
    pp++;
  }
}

////////////////////////////////////////////////////

void UClientFuncLaserGui::doImage(UHighGuiWindowHandle * sourceImg,
                                  char * imgFullName,
                                  UPosition * imPos,
                                  URotation * imRot)
{
  UImage * limg;
  //const char * imgName = "laser/img00619737-00_20050627_163244.760";
  //const char * imgName = "laser/img00666019-00_20050627_195618.521";
//  const char * imgName = "mmr-dyreh-20050629/Raw-20050629_204937.026";
  //const char * imgName = "mmr-dyreh-20050629/Raw_d0_s-1_posUnknown-20050629_203841.845";
  //const char * imgName = "mmr-dyreh-20050629/Raw_d0_s-1_posUnknown-20050629_204400.436";
  //const char * imgName = "mmr-laser-foto-20051207/PICT0013-800x600";
  //const char * imgName = "Philips840CAML12";
  //const char * imgName = "mmr-dyreh-20051010/scan90";
  // Raw_d0_s-1_posUnknown-20050629_204400.436
  const char * imgName = "fotoAndLaser";
  UPosition lpos, rcpos;
  URotation lrot, rcrot;
  ULaserDataSet * data = NULL;
  ULaserData * lpt = NULL;
  UMatrix4 mRob; // convert from laser to robot
  UMatrix4 mCam; // convert from robot to calera
  UMatrix4 mLC; // convert to camera perspective (3D)
  UMatrix4 mCI; // camera inner orientation
  UPosition p1, p2, p3, p4, p5;
  int i, j;
  double r, a;
  //double f = 1050; // philips org linse
  //double f = 673; // philips CAML10
  //double f = 576.0/2.0; // philips CAML012 (320x240 res)
  double f = 637; // minolta wide
  int w, h;
  UPixel * pix;
  bool isOK;
  CvPoint cvp, cvpBL, cvpBR, cvpTL, cvpTR;
  CvPoint cvl;
  CvScalar red = CV_RGB(207, 0, 0);
  CvScalar blue = CV_RGB(0, 0, 207);
  CvScalar blueLight = CV_RGB(100, 100, 255);
  CvScalar white = CV_RGB(255, 255, 255);
  UProbPoly * poly;
  UPose posePoly;
  ULaserPathResult * path;
  ULineSegment * seg;
  const int MSL = 50;
  char s[MSL];
  int lw = 1;
  //
  if (paintBold)
    lw = 2;
  //
  limg = cvw.makeImage("laserProj");
  if (limg != NULL)
  {
    if (sourceImg == NULL)
    {
      if ((imPos != NULL) and (imRot != NULL) and (imgFullName != NULL))
      { // image and image position parameters from command line
        isOK = limg->loadBMP(imgFullName);
        // camera position in robot coordinates
        limg->pos = *imPos;
        limg->rot = *imRot;
      }
      else
      {
        isOK = limg->loadBMP(imagePath, imgName, -1, "");
        // camera position in robot coordinates
        limg->pos.set(0.40, 0.0, 0.86);
        limg->rot.set(0.0,
                      0.40, // 0.23
                      0.0); // 0.70
        if (false)
        { // hjortek�r foto position in robot coordinates
          limg->pos.set(2.4, 2.4, 1.4);
          limg->rot.set(0.0,  0.24, -0.70);
        }
      }
    }
    else
    {
      isOK = limg->copy(sourceImg->getImage());
      // assume camera is at "get visPoly" position
      poly = &freePoly[freePolyNewest];
      posePoly = poly->getPoseOrg();
      posePoly = odoPose.getMapToPosePose(&posePoly);
      limg->pos.set(0.40 + posePoly.x, 0.0 + posePoly.y, 0.86);
      limg->rot.set(0.0,
                    0.40, // 0.23
                    0.0 + posePoly.h); // 0.70
    }
    if (not isOK)
      printf("Image not read\n");
    w = limg->width();
    h = limg->height();
    //
    // laser scanner position on robot
    lpos.set(0.40, 0.0, 0.41);
    lrot.set(0.0, 9.0 * M_PI / 180.0, 0.0);
    // conversion from camera coordinates to image coordinates
    //f *= double(w)/640.0;
    mCI.setSize(3, 4);
    mCI.setRow(0, 0.0, -1.0,   0.0, 0.0); //-limg->width()/2.0);
    mCI.setRow(1, 0.0,  0.0,  -1.0, 0.0); //-limg->height()/2.0);
    mCI.setRow(2, 1.0/f, 0.0, 0.0, 0.0);
    //
//     // front camera (20051010)
//     // camera position in robot coordinates
//     limg->pos.set(-0.2, 0.0, 0.86);
//     limg->rot.set(0.0,
//                   12.0 * M_PI / 180.0,
//                   0.0);
//     // laser scanner position on robot
//     lpos.set(0.0, 0.0, 0.42);
//     lrot.set(0.0, 9.5 * M_PI / 180.0, 0.0);
//     // conversion from camera coordinates to image coordinates
//     f = 1050 * double(w)/640.0;
//     mCI.setSize(3, 4);
//     mCI.setRow(0, 0.0, -1.0,   0.0, 0.0); //-limg->width()/2.0);
//     mCI.setRow(1, 0.0,  0.0,  -1.0, 0.0); //-limg->height()/2.0);
//     mCI.setRow(2, 1.0/f, 0.0, 0.0, 0.0);



    data = scanHist.getNewest();
    mRob = lrot.asMatrix4x4RtoM(lpos);
    mCam = limg->rot.asMatrix4x4MtoR(limg->pos);
    //
    mLC = mCI * mCam * mRob;
    //
    if (data != NULL)
      lpt = data->getData();
    else
      printf("No laserscan data - scanget andall\n");
    if (lpt != NULL and isOK)
    { // get laser position in camera coordinates
      p5 = mCI * mCam * lpos;
      // convert to top-left reference
      p5.x = p5.x / p5.z + double(w) / 2.0;
      p5.y = p5.y / p5.z + double(h) / 2.0;
      p5.z = 1.0;
      cvl.x = roundi(p5.x);
      cvl.y = roundi(p5.y);
      for (i = 0; i < data->getCount(); i++)
      {
        lpt = &data->getData()[i];
        r = lpt->getDistance();
        a = lpt->getAngle();
        // get position seen from laser
        p1.set(r * cos(a), r * sin(a), 0.0);
        if (false)
        {
          // convert to robot perspective
          p2 = mRob * p1;
          // convert to camera perspective
          p3 = mCam * p2;
          // convert to pixel position
          p4 = mCI * p3;
        }
        else
          p4 = mLC * p1;
        // normalize to top-left origin
        p4.x = p4.x / p4.z + double(w) / 2.0;
        p4.y = p4.y / p4.z + double(h) / 2.0;
        p4.z = 1.0;
        //
        if ((fabs(p4.x) < 5000.0) and (fabs(p4.y) < 5000.0))
        {
          cvp.x = roundi(p4.x);
          cvp.y = roundi(p4.y);
          //if ((cvp.x < w) and (cvp.x >= 0) and (cvp.y < h) and (cvp.y >= 0))
          { // paint pixel
            if (false)
            {
              pix = limg->getPixRef(cvp.y, cvp.x);
              pix->set(255, 0, 0);
            }
            else
            {
              cvCircle(limg->cvArr(), cvp, 2, white, 2);
              cvCircle(limg->cvArr(), cvp, 4, red, 2);
            }
            if (i %5 == 0)
              cvLine(limg->cvArr(), cvp, cvl, red, 1, 0, 0);
          }
        }
      }
      mLC = mCI * mCam;
      if (paintIntervalLines)
      {
        // get matrix for conversion from robot to camera perspective
        for (i= 0; i < pathsCnt; i++)
        {
          path = paths[i];
          if (path->isPathUsed() or paintPathLinesAll)
          {
            for (j = 0; j < path->getPassLinesCnt(); j++)
            {
              seg = path->getPassLine(j);
              p1 = odoPose.getMapToPose(seg->pos);
              p2 = odoPose.getMapToPose(seg->getOtherEnd());
              // convert first end to pixel values
              p5 = mLC * p1;
              p5.x = p5.x / p5.z + double(w) / 2.0;
              p5.y = p5.y / p5.z + double(h) / 2.0;
              p5.z = 1.0;
              cvl.x = roundi(p5.x);
              cvl.y = roundi(p5.y);
              // convert the other end
              p5 = mLC * p2;
              p5.x = p5.x / p5.z + double(w) / 2.0;
              p5.y = p5.y / p5.z + double(h) / 2.0;
              p5.z = 1.0;
              cvp.x = roundi(p5.x);
              cvp.y = roundi(p5.y);
              // paint the line
              if (path->isPathUsed())
                cvLine(limg->cvArr(), cvp, cvl, blue, lw, 8, 0);
              else
                cvLine(limg->cvArr(), cvp, cvl, blueLight, lw, 8, 0);
            }
          }
        }
      }
      if (true /* paintRobCamConus */)
      { // robot camera position and rotation
        rcpos.set(0.45,0.0, 0.86);
        rcrot.set(0.0, 0.42, 0.0);
        // camera position in image pixel coordinates
        p5 = mLC * rcpos;
        p5.x = p5.x / p5.z + double(w) / 2.0;
        p5.y = p5.y / p5.z + double(h) / 2.0;
        cvl.x = roundi(p5.x);
        cvl.y = roundi(p5.y);
        // get conus edges position
        p1 = getPixToRobFloor(rcpos, rcrot, 576, 640, 480, 0, 480);
        p5 = mLC * p1;
        p5.x = p5.x / p5.z + double(w) / 2.0;
        p5.y = p5.y / p5.z + double(h) / 2.0;
        cvpBL.x = roundi(p5.x);
        cvpBL.y = roundi(p5.y);
        cvLine(limg->cvArr(), cvpBL, cvl, blue, lw, 8, 0);
        p1 = getPixToRobFloor(rcpos, rcrot, 576, 640, 480, 640, 480);
        p5 = mLC * p1;
        p5.x = p5.x / p5.z + double(w) / 2.0;
        p5.y = p5.y / p5.z + double(h) / 2.0;
        cvpBR.x = roundi(p5.x);
        cvpBR.y = roundi(p5.y);
        cvLine(limg->cvArr(), cvpBR, cvl, blue, lw, 8, 0);
        cvLine(limg->cvArr(), cvpBR, cvpBL, blue, lw, 8, 0);
        p1 = getPixToRobFloor(rcpos, rcrot, 575, 640, 480, 0, 140);
        p5 = mLC * p1;
        p5.x = p5.x / p5.z + double(w) / 2.0;
        p5.y = p5.y / p5.z + double(h) / 2.0;
        cvpTL.x = roundi(p5.x);
        cvpTL.y = roundi(p5.y);
        //cvLine(limg->cvArr(), cvpTL, cvl, blue, lw, 8, 0);
        cvLine(limg->cvArr(), cvpTL, cvpBL, blue, lw, 8, 0);
        p1 = getPixToRobFloor(rcpos, rcrot, 576, 640, 480, 640, 140);
        p5 = mLC * p1;
        p5.x = p5.x / p5.z + double(w) / 2.0;
        p5.y = p5.y / p5.z + double(h) / 2.0;
        cvpTR.x = roundi(p5.x);
        cvpTR.y = roundi(p5.y);
        //cvLine(limg->cvArr(), cvpTR, cvl, blue, lw, 8, 0);
        cvLine(limg->cvArr(), cvpTR, cvpBR, blue, lw, 8, 0);
      }
    }
    limg->imgTime.getForFilename(s, true);
    if (data != NULL)
      isOK = limg->saveBMP(imagePath, imgName, data->getSerial(), s);
    printf("Done\n");
    cvw.showImage(limg, true);
  }
  else
    printf("Could not load camera image '%s'\n", imgName);
}

/////////////////////////////////////////////

UPosition UClientFuncLaserGui::getPixToRobFloor(
    UPosition camPos, URotation camRot,
    double f, int w, int h,
    int x, int y)
{
  UPosition posc, posr, posfr;
  UPosition result;
  UCamPar camPar;
  bool isOK = true;
  ULine line;
  ULine floor(0.0, 0.0, 0.0, 0.0, 0.0, 1.0); // is a plane (point and vector-format)
  UMatrix4 mRT(4,4);
  //
  //set camera parameter structure
  camPar.setCameraParameters(h/2.0, w/2.0, 0.0, 0.0, f, w / 640.0);
  // find vector in this direction (1.0 meter away)
  posc = camPar.getPtoCRob(x, y, 1.0);
  // convert point to robot coordinates
  mRT = camRot.asMatrix4x4RtoM(camPos);
  posr = mRT * posc;
  // make line from camera to point (in robot perspective)
  line.setFromPoints(&camPos, &posr);
  // get point on floor in robot perspective
  result = floor.getPlaneLineCrossing(line, &isOK);
  // result is false if vector is parallel to plane
  if (not isOK)
    printf("UClientFuncLaserGui::getPixToRobFloor: Not a valid position for %dx %dy\n", x, y);
  //
  return result;
}

/////////////////////////////////////////////

void UClientFuncLaserGui::paintFreePoly(UProbPoly * poly,
                                       UPose seenFromPose,
                                       bool historic)
{
  int i;
  CvPoint p1, p2, pr;
  UPosition pos;
  UPosition *po1, *po2;
  bool * poObst;
  double ppm;
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
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
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

bool UClientFuncLaserGui::setPaintVar(const char * name, const char * value)
{
  bool result = true;
  //
  if (strcasecmp(name, "all") == 0)
  {
    paintGPS = true;
    paintVar = true;
    paintCurves = true;
    paintRoadLines = true;
    paintIntervalLines = true;
    paintPathLines = true;
    paintSpeed = true;
    paintPlan = true;
    paintObst = true;
  }
  else if (strcasecmp(name, "raw") == 0)
  {
    paintGPS = true;
    paintVar = false;
    paintCurves = false;
    paintRoadLines = false;
    paintIntervalLines = false;
    paintPathLines = false;
    paintSpeed = true;
  }
/*      printf(" - paint <type> <value>   Type: Bold, Curves, GPS, Road, Segments,\n");
      printf("                          Var, Speed, Path (value=true/false)\n");*/
  else if (strcasecmp(name, "gps") == 0)
    paintGPS = str2bool(value);
  else if (strcasecmp(name, "var") == 0)
    paintVar = str2bool(value);
  else if (strcasecmp(name, "curves") == 0)
    paintCurves = str2bool(value);
  else if (strcasecmp(name, "road") == 0)
    paintRoadLines = str2bool(value);
  else if (strcasecmp(name, "segments") == 0)
    paintIntervalLines = str2bool(value);
  else if (strcasecmp(name, "path") == 0)
    paintPathLines = str2bool(value);
  else if (strcasecmp(name, "speed") == 0)
    paintSpeed = str2bool(value);
  else if (strcasecmp(name, "bold") == 0)
    paintBold = str2bool(value);
  else if (strcasecmp(name, "plan") == 0)
    paintPlan = str2bool(value);
  else if (strcasecmp(name, "obst") == 0)
    paintObst = str2bool(value);
  else if (strcasecmp(name, "pathAll") == 0)
    paintPathAll = str2bool(value);
  else if (strcasecmp(name, "pathLinesAll") == 0)
    paintPathLinesAll = str2bool(value);
  else if (strcasecmp(name, "visPoly") == 0)
    paintVisPoly = str2bool(value);
  else if (strcasecmp(name, "pathHistCnt") == 0)
    paintPathHistCnt = strtol(value, NULL, 0);
  else if (strcasecmp(name, "visPolyCnt") == 0)
    paintVisPolyCnt = strtol(value, NULL, 0);
  else if (strcasecmp(name, "scanHistCnt") == 0)
    paintScanHistCnt = strtol(value, NULL, 0);
  else if (strcasecmp(name, "gridSize") == 0)
    paintGridSize = strtod(value, NULL);
  else if (strcasecmp(name, "odoGrid") == 0)
    paintGridOdo = str2bool(value);
  else if (strcasecmp(name, "mmr") == 0)
    paintMmr = str2bool(value);
  else if (strcasecmp(name, "smr") == 0)
    paintMmr = not str2bool(value);
  else
    result = false;
  // scanHistCnt
  return result;
}

///////////////////////////////////////

// void UClientFuncLaserGui::paintPisInImage(UImage * img)
// {
//   bool isOK = img != NULL;
//   UPose imgPose;
//   UCamPwc pwcDev;
//   UCamMounted cam(&pwcDev);
//   float x,y;
//   Uconfig ini;
//   UProbPoly * ppoly;
//   ULaserPathResult ** pr;
//   int i, j;
//   ULineSegment * seg, sg;
//   CvPoint p1, p2;
//   CvScalar blue = CV_RGB(0, 0, 255);
//   CvScalar red = CV_RGB(255, 0, 0);
//   CvScalar yellow = CV_RGB(255, 255, 0);
//   UPosition posM, posR1, posR2;
//   double v;
//   // prepare camera position
// 
//   //
//   isOK = (img != NULL) and (freePolyNewest >= 0);
//   if (isOK)
//   { // prepare camera position
//     // ini.readConfig("/home/chr/cpp/robcam.conf");
//     ini.readConfig(configFileCam);
//     isOK = cam.loadCamSettings(&ini, "mmrCam0");
//     printf("paintPisInImage: Assuming camera 'mmrCam0' (%s) from %s\n",
//            bool2str(isOK), configFileCam);
//     // get pose of robot, when image was taken
//     ppoly = &freePoly[freePolyNewest];
//     imgPose = ppoly->getPoseOrg();
//   }
//   p1.x = 0;
//   p1.y = 0;
//   if (isOK)
//   { // now get data to paint;
//     pr = paths;
//     for (i = 0; i < pathsCnt; i++)
//     {
//       if ((*pr)->isPathUsed() or paintPathLinesAll)
//       { // paint used path only
//         for (j = 0; j < (*pr)->getPassLinesCnt(); j++)
//         { // get segment to paint
//           seg = (*pr)->getPassLine(j);
//           // paint this line
//           posM = seg->pos; // rightmost position
//           // convert to robor coordinates
//           posR1 = imgPose.getMapToPose(posM);
//           // now the other end the same way
//           posM = seg->getOtherEnd(); // leftmost position
//           posR2 = imgPose.getMapToPose(posM);
//           //test if behind image (range < 1 m
//           if (posR1.x < 1.0)
//           { // make segment shorter
//             v = (1.0 - posR1.x)/(posR2.x - posR1.x) *
//                  seg->length;
//             sg.setFromPoints(posR1, posR2);
//             posR1 = sg.getPositionOnLine(v);
//           }
//           else if (posR2.x < 0)
//           { // make segment shorter
//             v = (1.0 - posR2.x)/(posR1.x - posR2.x) *
//                  seg->length;
//             sg.setFromPoints(posR1, posR2);
//             posR2 = sg.getPositionOnLine(sg.length - v);
//           }
//           // move to pixel values
//           cam.getMtoPix(posR1, true, &x, &y);
//           p1.x = roundi(x);
//           p1.y = roundi(y);
//           // and the other end
//           cam.getMtoPix(posR2, true, &x, &y);
//           p2.x = roundi(x);
//           p2.y = roundi(y);
//           // now paint in image (points outside image is OK)
//           cvLine(img->cvArr(), p1, p2, blue, 1, 8, 0);
//         }
//       }
//       // next path
//       pr++;
//     }
//   }
//   if (isOK)
//   { // paint path line too.
//     if (paintPathAll)
//     {
//       pr = paths;
//       for (i = 0; i < pathsCnt; i++)
//       {
//         if (not (*pr)->isPathUsed())
//         {
//           for (j = 0; j < (*pr)->getRouteCnt(); j++)
//           {
//               p2 = p1;
//               posR2 = posR1;
//               posM = *(*pr)->getRoute(j);
//               posR1 = imgPose.getMapToPose(posM);
//               if (posR1.x < 1.0)
//               { // move point posR1 to a positive value
//                 // of 1.0
//                 sg.setFromPoints(posR1, posR2);
//                 v = (1.0 - posR1.x)/(posR2.x - posR1.x) *
//                     sg.length;
//                 posR1 = sg.getPositionOnLine(v);
//               }
//               // move to pixel values
//               cam.getMtoPix(posR1,
//                 (posR1.x > 2.0), &x, &y);
//               p1.x = roundi(x);
//               p1.y = roundi(y);
//               // paint
//               if (j > 0)
//                   cvLine(img->cvArr(), p1, p2, yellow, 2, 8, 0);
//           }
//         }
//         // advance
//         pr++;
//       }
//     }
//     pr = paths;
//     for (i = 0; i < pathsCnt; i++)
//     { // the real one only
//       if ((*pr)->isPathUsed())
//       {
//         for (j = 0; j < (*pr)->getRouteCnt(); j++)
//         {
//           p2 = p1;
//           posR2 = posR1;
//           posM = *(*pr)->getRoute(j);
//           posR1 = imgPose.getMapToPose(posM);
//           if (posR1.x < 1.0)
//           { // move point posR1 to a positive value
//             // of 1.0
//             sg.setFromPoints(posR1, posR2);
//             v = (1.0 - posR1.x)/(posR2.x - posR1.x) *
//                 sg.length;
//             posR1 = sg.getPositionOnLine(v);
//           }
//           // move to pixel values
//           cam.getMtoPix(posR1, true, &x, &y);
//           p1.x = roundi(x);
//           p1.y = roundi(y);
//           // paint
//           if (j > 0)
//             cvLine(img->cvArr(), p1, p2, red, 2, 8, 0);
//         }
//       }
//       // advance
//       pr++;
//     }
//   }
//   // return and repaint image
// }

///////////////////////////////////////////////////////

void UClientFuncLaserGui::paintObstGrp(UImage * img,
                                       UObstacleGroup *obst,
                                       UPose seenFromPose, int grpIdx)
{
  int i, n;
  CvPoint p1, p2, pr;
  UPosition pos;
  UPosition po1, po2;
  double ppm;
  const CvScalar purpOdd   = CV_RGB(100, 50, 100);
  const CvScalar yellowOdd = CV_RGB(100, 100, 50);
  const CvScalar purp      = CV_RGB(190, 0, 190);
  const CvScalar yellow    = CV_RGB(190, 190, 0);
  //
  CvScalar obsValid;
  CvScalar obsInvalid;
//  CvScalar hist = CV_RGB(150, 100, 100);
  CvScalar * col = &obsValid;
  CvFont font;
  int lw = 3;
  UObstacle * ob;
  //
  if (paintBold)
    lw = 4;
  if (grpIdx % 2 == 0)
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
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
  //

  for (n = 0; n < obst->getObstsCnt(); n++)
  {
    ob = obst->getObstacle(n);
    // paint obstacle
    po1 = ob->getPoint(ob->getPointsCnt() - 1);
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

void UClientFuncLaserGui::paintOdoDest(UImage * img,
                                       UPose seenFromPose )
{
  CvPoint p1, pr;
  UPose pol;
  double ppm;
  CvScalar red   = CV_RGB(200, 0, 0);
//  CvScalar hist = CV_RGB(150, 100, 100);
  CvFont font;
  int lw = 2; // line width
  //int size = 6;
  bool gotAll = true;
  UPose po;
  //
  if (paintBold)
  {
    lw = 4;
    //size=8;
  }
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               1.0, 1.0, 0.0, 1, 8);
  //
  gotAll = planner.findPose("endPoseX", "endPoseY", "endPoseH", &po);
  if (gotAll)
  {
    ppm = img->height() / MaxRange;
    pr.y = img->height() - roundi(ppm * StartPos);
    pr.x = img->width()/2;
    //
    pol = seenFromPose.getMapToPosePose(&po);
    p1.y = pr.y - roundi(pol.x * ppm);
    p1.x = pr.x - roundi(pol.y * ppm);
    paintPose(img, p1.x, p1.y, pol.h, 8, red, lw);
  }
}

//////////////////////////////////////////////////////////

void UClientFuncLaserGui::setObstList(UObstHist * value)
{
  obsts = value;
  if (obsts != NULL)
    addOnEvent(obsts);
}

//////////////////////////////////////////////////////////

void UClientFuncLaserGui::setSfPool(USFPool * pool)
{
  sfPool = pool;
  if (sfPool != NULL)
    addOnEvent(sfPool);
}

//////////////////////////////////////////////////////////


bool UClientFuncLaserGui::onEvent(const char * interface, const char * dataType, void * data)
{
  UPoseTime pt;
  USFData * sf;
  UObstacleGroup * og;
  //
  if (strcmp(dataType, "obst") == 0)
  {
    if (obsts != NULL)
    {
      if (obsts->getGrpsCnt() > 0)
      { // get newest group (first is newest)
        og = obsts->getGroup(0);
        if (og != NULL)
        { // gte pose-time structure for obstacle update
          pt = og->getPoseLast();
          if (pt.t > odoPose.t)
            // add this pose to pose history if newer
            setPose(pt);
        }
      }
    }
  }
  else
  { // assume scanfeature
    sf = sfPool->getScan(0);
    if (sf != NULL)
      if (sf->scanTime > odoPose.t)
      { // add this pose to pose history if newer
        pt.setPt(sf->pose, sf->scanTime);
        setPose(pt);
      }
  }
  // request a repaint
  repaint();
  //
  return false;
}

//////////////////////////////////////////////////////////

void UClientFuncLaserGui::paintLineSegments(UPose seenFromPose)
{
  int i, n, rl;
  CvPoint p1, p2, pr;
  UPosition pos;
  UPosition po1, po2;
  double ppm;
  //const CvScalar purpOdd   = CV_RGB(100, 50, 100);
  const CvScalar yellowOdd = CV_RGB(100, 100, 50);
  const CvScalar purp      = CV_RGB(190, 0, 190);
  const CvScalar yellow    = CV_RGB(190, 190, 0);
  const CvScalar green     = CV_RGB(0, 150, 0);
  const CvScalar red     = CV_RGB(150, 0, 0);
  const CvScalar gray     = CV_RGB(120, 120, 120);
  //const CvScalar black     = CV_RGB(0, 0, 0);
  const CvScalar blue     = CV_RGB(0, 0, 150);
  const CvScalar reddish  = CV_RGB(120, 50, 50);
  USFData * sf;
  ULineSegment * seg;
  const CvScalar * col = &purp;
  int lw = 2;
  int ulw;
  int * segInt;
  bool isRoad, isLine, isPoint;
  //bool showLineOnly;
  const int MAX_TYPES = 4;
  int cnt[MAX_TYPES];
  int typ;
  CvFont font;
  const int MSL = 30;
  char text[MSL];
  const char * segStr;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
               1.0, 1.0, 0.0, 1, 8);
  if (paintBold)
    lw = 4;
  // robot position in image
  ppm = img->height() / MaxRange;
  pr.y = img->height() - roundi(ppm * StartPos);
  pr.x = img->width()/2;
  //
  p1.x = 0;
  p1.y = 0;
  //cnt = mini(sfPool->getScansCnt(), paintScanHistCnt);
  // debug
  //cnt = mini(1, cnt);
  for (i = 0; i < MAX_TYPES; i++)
    cnt[i] = 0;
  // debug end
  for (n = 0; n < sfPool->getScansCnt(); n++)
  {
    sf = sfPool->getScan(n);
    isRoad = false;
    isLine = false;
    isPoint = false;
    //showLineOnly = false;
    // get segment
    if (strcasecmp(sf->getDataType(), "sf") == 0)
    {
      isLine = true;
      col = &purp;
      typ = 1;
    }
    if (strcasecmp(sf->getDataType(), "line") == 0)
    {
      isLine = true;
      col = &purp;
      typ = 1;
    }
    else if (strcasecmp(sf->getDataType(), "pass") == 0)
    {
      col = &green;
      typ = 2;
    }
    else if (strcasecmp(sf->getDataType(), "road") == 0)
    {
      isRoad = true;
      typ = 3;
      //showLineOnly = (cnt[typ] > 5);
    }
    else
    {
      col = &yellow;
      typ = 0;
    }
    if (cnt[typ]++ < paintScanHistCnt)
    { // make newest wider than older entries
      if (sf->isNewest)
        ulw = lw;
      else
        ulw = lw / 2;
      //
      seg = sf->getSegs();
      segInt = sf->getSegInt();
      for (i = 0; i < sf->getSegsCnt(); i++)
      {
        isPoint = (seg->length <= 0.01);
        segStr = sf->segStr[i];
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
        else if (isLine)
        {
          switch (*segInt)
          {
            case 0: col = &red; break;
            case 1: col = &blue; break;
            case 2: col = &purp; break;
            case 3: col = &green; break;
            case 4: col = &yellow; break;
            case 5: col = &yellowOdd; break;
            case 6: col = &gray; break;
            case 7: col = &gray; break;
            default: col= &reddish; break;
          }
        }
        if (not isPoint)
        {
          po1 = seenFromPose.getMapToPose(seg->getOtherEnd());
          p1.y = pr.y - roundi(po1.x * ppm);
          p1.x = pr.x - roundi(po1.y * ppm);
        }
        po2 = seenFromPose.getMapToPose(seg->pos);
        p2.y = pr.y - roundi(po2.x * ppm);
        p2.x = pr.x - roundi(po2.y * ppm);
        if ((p1.x >= 0 or p2.x >=0) and (p1.y >= 0 or p2.y >= 0) and
             (p1.x < (int)img->width() or p2.x < (int)img->width()) and
             (p1.y < (int)img->height() or p2.y < (int)img->height()))
        {
          if (isPoint)
          {
            p1.x = p2.x + 3;
            p1.y = p2.y + 3;
            p2.x -= 3;
            p2.y -= 3;
            cvLine(img->cvArr(), p1, p2, *col, ulw, 8, 0);
            p1.x -= 6;
            p2.x += 6;
            cvLine(img->cvArr(), p1, p2, *col, ulw, 8, 0);
          }
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
        }
        seg++;
        segInt++;
        segStr = sf->segStr[i];
      }
    }
  }
}

/////////////////////////////////////////////////////////

void UClientFuncLaserGui::repaint()
{
  if (not repaintImage)
  {
    repaintImage = true;
    repaintImageTime.Now();
  }
}
