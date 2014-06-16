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

#include <ugen4/uline.h>
//#include "ucamcommon.h"
#include "uimgproj.h"

/////////////////////////////////////////////////////

UImgProj::UImgProj()
{
  mapPath = NULL;
  mapCol = NULL;
  mapCellSize = 0.1;
  robPose.set(0, 0.0, 0.0);
  // start pose (bottom left is (0,0)
  startPose.set(1.0, 1.0, 0.0);
  // camera pos at robot center 1.0 meter up
  camPos.set(0.0, 0.0, 1.0);
  // camera pointing a bit down
  camRot.set(0.0, 12.0 * M_PI / 180.0, 0.0);
  // invalid camera parameters
  camPar.invalidate();
}

/////////////////////////////////////////////////////

UImgProj::~UImgProj()
{  // nothing to regret
}

/////////////////////////////////////////////////////

bool UImgProj::clearColMap()
{
  bool result = false;
  //
  if (mapCol != NULL)
  { // set size
    if (not mapCol->setSize(480, 640, 3, 8, "RGB"))
      result = false;
    mapCol->clear();
  }
  return result;
}

//////////////////////////////////////////////////////

bool UImgProj::clearPathMap()
{
  bool result = false;
  //
  if (mapPath != NULL)
  { // set size
    result = mapPath->setSize(480, 640, 3, 8, "RGB");
    // clear to gray
    mapPath->clear(UPixel::pixRGB(128, 128, 128));
  }
  return result;
}

//////////////////////////////////////////////////////

void UImgProj::paintGrid(UImage * img)
{
  float h0, w0, wg, hg;
  float mpm = 1.0; // meter per marker;
  float ppmeter = mpm / mapCellSize; // pixels per meter
  float ppm = ppmeter; // pixels per marker
  float mv; // marker value
  int h, w;
  CvScalar green = CV_RGB(160, 255, 128);
  CvScalar blue = CV_RGB(150, 0, 128);
  CvFont font;
  const int TS = 20;
  char s[TS];
  char c = 'm'; // grid dimension
  //
  while (ppm < 30)
  {
    mpm *= 2.0;
    ppm = mpm / mapCellSize;
  }
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              0.6, 1.0, 0.0, 1, 8);
  if (img != NULL)
  {  // zero line horizontal
    h0 = mapCol->height() - startPose.y / mapCellSize;
    h = roundi(h0);
    cvLine(img->cvArr(), cvPoint(0, h),
          cvPoint(int(mapCol->width()) - 1, h), blue, 1, 8, 0);
    // zero vertical line
    w0 = startPose.x / mapCellSize;
    w = roundi(w0);
    cvLine(img->cvArr(), cvPoint(w, 0),
          cvPoint(w, int(mapCol->height())-1), blue, 1, 8, 0);
    //
    hg = h0 - ppm;
    mv = mpm;
    while (hg > 0)
    { // paint horizontal lines above zero
      h = roundi(hg);
      cvLine(img->cvArr(), cvPoint(0, h),
          cvPoint(int(mapCol->width()) - 1, h), green, 1, 8, 0);
      hg -= ppm;
      snprintf(s, TS, "%d%c", roundi(mv), c);
      cvPutText(img->cvArr(), s, cvPoint(0, h), &font, blue);
      mv += mpm;
      c = ' ';
    }
    hg = h0 + ppm;
    mv = -mpm;
    while (hg < mapCol->height())
    { // paint horizontal lines below zero
      h = roundi(hg);
      cvLine(img->cvArr(), cvPoint(0, h),
          cvPoint(int(mapCol->width()) - 1, h), green, 1, 8, 0);
      hg += ppm;
      snprintf(s, TS, "%d%c", roundi(mv), c);
      cvPutText(img->cvArr(), s, cvPoint(0, h), &font, blue);
      mv -= mpm;
      c = ' ';
    }
    //
    c = 'm';
    wg = w0 + ppm;
    mv = mpm;
    while (wg < mapCol->width())
    { // paint vertical lines for positive x values
      w = roundi(wg);
      cvLine(img->cvArr(), cvPoint(w, 0),
          cvPoint(w, int(mapCol->height())-1), green, 1, 8, 0);
      wg += ppm;
      if (w > 30)
      { // write distance on x-axis
        snprintf(s, TS, "%d%c", roundi(mv), c);
        cvPutText(img->cvArr(), s, cvPoint(w, int(mapCol->height())-3), &font, blue);
        mv += mpm;
        c = ' ';
      }
    }
    wg = w0 - ppm;
    mv = -mpm;
    while (wg > 0)
    { // paint vertical lines for negative x values
      w = roundi(wg);
      cvLine(img->cvArr(), cvPoint(w, 0),
          cvPoint(w, int(mapCol->height())-1), green, 1, 8, 0);
      wg -= ppm;
      if (w > 30)
      {
        snprintf(s, TS, "%d%c", roundi(mv), c);
        cvPutText(img->cvArr(), s, cvPoint(w, int(mapCol->height())-3), &font, blue);
        mv -= mpm;
        c = ' ';
      }
    }
  }
}

/////////////////////////////////////////////////////

bool UImgProj::paintRobot(UImage * img, int imageNumber)
{
  bool res = (img != NULL);
  UPosition mp;
  CvPoint mpt[4];
  CvScalar rgb = CV_RGB(255, 0, 0);
  const int SL = 10;
  char s[SL];
  //
  if (res)
  { // The robot  is 65 cm long and 45 cm wheel distance
    mp = getRobToMap(UPosition::position(0.0,   0.225, 0.0)); // left wheel
    mpt[0].x = roundi(mp.x / mapCellSize);
    mpt[0].y = img->height() - roundi(mp.y / mapCellSize);
    mp = getRobToMap(UPosition::position(0.0,  -0.225, 0.0)); // right wheel
    mpt[1].x = roundi(mp.x / mapCellSize);
    mpt[1].y = img->height() - roundi(mp.y / mapCellSize);
    mp = getRobToMap(UPosition::position(0.65,  0.0,   0.0)); // front wheel
    mpt[2].x = roundi(mp.x / mapCellSize);
    mpt[2].y = img->height() - roundi(mp.y / mapCellSize);
    mp = getRobToMap(UPosition::position(-0.25,  0.0,   0.0)); // back edge
    mpt[3].x = roundi(mp.x / mapCellSize);
    mpt[3].y = img->height() - roundi(mp.y / mapCellSize);
    // paint robot triangle (arrow)
    cvLine(img->cvArr(), mpt[0], mpt[1], rgb, 1, 8, 0);
    cvLine(img->cvArr(), mpt[1], mpt[2], rgb, 1, 8, 0);
    cvLine(img->cvArr(), mpt[2], mpt[0], rgb, 1, 8, 0);
    cvLine(img->cvArr(), mpt[2], mpt[3], rgb, 1, 8, 0);
    // write image number (sometimes)
    if ((imageNumber % 3) == 1)
    { // paint number by robot position
      snprintf(s, SL, "%d", imageNumber);
      paintText(img, mpt[3].x, mpt[3].y, s);
    }
  }
  //
  return res;
}

//////////////////////////////////////////////////////////////////////

bool UImgProj::paintRobotPath(UImage * img, UPose toPose)
{
  bool res = (img != NULL);
  UPosition mp;
  CvPoint mpt[3];
  CvScalar rgb = CV_RGB(0, 255, 0);
  CvScalar rgbh = CV_RGB(0, 255, 255);
  //
  if (res)
  { // The robot  is 65 cm long and 45 cm wheel distance
    mp = getRobToMap(UPosition::position(0.0, 0.0, 0.0)); // zero
    mpt[0].x = roundi(mp.x / mapCellSize);
    mpt[0].y = img->height() - roundi(mp.y / mapCellSize);
    mp = getRobToMap(UPosition::position(0.5, 0.0, 0.0)); // forward 0.5 meter
    mpt[1].x = roundi(mp.x / mapCellSize);
    mpt[1].y = img->height() - roundi(mp.y / mapCellSize);
    mp = getRobToMap(toPose, UPosition::position(0.0, 0.0, 0.0)); // zero
    mpt[2].x = roundi(mp.x / mapCellSize);
    mpt[2].y = img->height() - roundi(mp.y / mapCellSize);
    // paint lines
    cvLine(img->cvArr(), mpt[0], mpt[1], rgbh, 1, 8, 0);
    cvLine(img->cvArr(), mpt[0], mpt[2], rgb,  1, 8, 0);
  }
  return res;
}

/////////////////////////////////////////////////////

void UImgProj::setCamPos(double x, double y, double z,
          double omega, double phi, double kappa)
{
  camPos.x = x;
  camPos.y = y;
  camPos.z = z;
  camRot.Omega = omega;
  camRot.Phi = phi;
  camRot.Kappa = kappa;
}

/////////////////////////////////////////////////////

void UImgProj::setCamPos(UPosition pos,
              URotation rot)
{
  camPos = pos;
  camRot = rot;
}

/////////////////////////////////////////////////////

UPosition UImgProj::getMapToRob(UPosition mapCoordinate)
{ // from a map position to a position in robot coordinates
  UPosition result;
  UPosition poseRef;
  //
  poseRef = startPose.getMapToPose(mapCoordinate);
  result = robPose.getMapToPose(poseRef);
  //
  return result;
}

/////////////////////////////////////////////////////

UPosition UImgProj::getRobToMap(UPosition robCoordinate)
{ // from a map position to a position in robot coordinates
  UPosition result;
  UPosition poseRef;
  //
  poseRef = robPose.getPoseToMap(robCoordinate);
  result = startPose.getPoseToMap(poseRef);
  //
  return result;
}

/////////////////////////////////////////////////////

UPosition UImgProj::getRobToMap(UPose fromPose, UPosition robCoordinate)
{ // from a map position to a position in robot coordinates
  UPosition result;
  UPosition poseRef;
  //
  poseRef = fromPose.getPoseToMap(robCoordinate);
  result = startPose.getPoseToMap(poseRef);
  //
  return result;
}

/////////////////////////////////////////////////////

UPosition UImgProj::getRobToCam(UPosition robCoordinate)
{ // from a position in robot coordinates to camera coordinates
  UPosition result;
  UMatrix4 mRT(4,4);
  //
  mRT = camRot.asMatrix4x4MtoR(camPos);
  result = mRT * robCoordinate;
  //
  return result;
}

/////////////////////////////////////////////////////

UPosition UImgProj::getCamToRob(UPosition camCoordinate)
{ // from a position in camera coordinates to robot oriented coordinates
  UPosition result;
  UMatrix4 mRT(4,4);
  //
  mRT = camRot.asMatrix4x4RtoM(camPos);
  result = mRT * camCoordinate;
  //
  // debug
  //mRT.print("UImgProj::getCamToRob - mRT");
  //camCoordinate.show("UImgProj::getCamToRob Xcam");
  //result.show("UImgProj::getCamToRob:\n - result = mRT * Xcam");
  // debug end
  //
  return result;
}

/////////////////////////////////////////////////////

bool UImgProj::setImages(UImage * mapColorImage, UImage * mapPathImage)
{
  mapCol = mapColorImage;
  mapPath = mapPathImage;
  return (mapCol != NULL) and (mapPath != NULL);
}

/////////////////////////////////////////////////////

UPosition UImgProj::getPixToMapFloor(
               int w, int h, bool * lookDown)
{
  UPosition posfr;
  UPosition result;
  // get point on floor in robot perspective
  posfr = getPixToRobFloor(w, h, lookDown);
  // get point on floor in map perspective
  result = getRobToMap(posfr);
  //
  return result;
}

//////////////////////////////////////////////////////////

UPosition UImgProj::getPixToRobFloor(
               int w, int h, bool * lookDown)
{
  UPosition posc, posr, posfr;
  UPosition result;
  bool isOK;
  bool isDown;
  ULine line;
  ULine floor(0.0, 0.0, 0.0, 0.0, 0.0, 1.0); // is a plane (point and vector-format)
  // find vector in this direction (1.0 meter away)
  posc = camPar.getPtoCRob(w, h, 1.0);
  // convert point to robot coordinates
  posr = getCamToRob(posc);
  // is camera height larger than 1m point
  isDown = (camPos.z > (posr.z + 0.02));
  // debug
  /*
  if (not isDown)
    printf("UImgProj::getPixToMapFloor: is not pointing down\n");
  */
  // debug end
  // make line from camera to point (in robot perspective)
  line.setFromPoints(&camPos, &posr);
  // get point on floor in robot perspective
  result = floor.getPlaneLineCrossing(line, &isOK);
  // result is false if vector is parallel to plane
  if (lookDown != NULL)
    *lookDown = isDown and isOK;
  //
  return result;
}

//////////////////////////////////////////////////////////

UPosition UImgProj::getPixToRobFloor(
    float w, float h, bool * lookDown)
{
  UPosition posc, posr, posfr;
  UPosition result;
  bool isOK;
  bool isDown;
  ULine line;
  ULine floor(0.0, 0.0, 0.0, 0.0, 0.0, 1.0); // is a plane (point and vector-format)
  // find vector in this direction (1.0 meter away)
  posc = camPar.getPtoCRob(w, h, 1.0);
  // convert point to robot coordinates
  posr = getCamToRob(posc);
  // is camera height larger than 1m point
  isDown = (camPos.z > (posr.z + 0.02));
  // debug
  /*
  if (not isDown)
  printf("UImgProj::getPixToMapFloor: is not pointing down\n");
  */
  // debug end
  // make line from camera to point (in robot perspective)
  line.setFromPoints(&camPos, &posr);
  // get point on floor in robot perspective
  result = floor.getPlaneLineCrossing(line, &isOK);
  // result is false if vector is parallel to plane
  if (lookDown != NULL)
    *lookDown = isDown and isOK;
  //
  return result;
}

//////////////////////////////////////////////////////////


URPos UImgProj::getFloorToPix(UPosition posfl)
{
   URPos result;
   UPosition posr;
   UPosition posc;
   //
   //posfl.show("map floor");
   posr = getMapToRob(posfl);
   //posr.show("Robot");
   posc = getRobToCam(posr);
   //posc.show("Camera");
   result = camPar.getCtoPRob(posc);
   //
   return result;
}

//////////////////////////////////////////////////////////

bool UImgProj::doProject(UImage * imColor, UImage * imPathMask)
{
  bool result;
  unsigned int r,c;
  unsigned int minW, maxW, minH, maxH;
  int w,h;
  double mx, my;
  URPos posi;
  UPosition posmBl, posmBr; // position bottom left/right
  UPosition posfl;
  UPixel * pdc;
  UPixel * pdm;
  UPixel psc;
  UPixel psm;
  //
  result = (imColor != NULL) and (imPathMask != NULL);
  if (result)
    result = camPar.isValid();
  if (result);
  { // get position where bottom left pixel hits floor-plane
    posmBl = getPixToMapFloor(0,
                      imColor->height(), &result);
    posmBl.show("Bottom left");
    // get position where bottol right pixel hits floor plane
    posmBr = getPixToMapFloor(imColor->width(),
                      imColor->height(), &result);
    posmBr.show("Bottom right");
    minW = maxi(0, roundi(mind(posmBl.x, posmBr.x) / mapCellSize));
    maxW = mapCol->width();
    minH = 0;
    maxH = mapCol->height();
  }
  if (result)
  {
    for (r = minH; r < maxH; r++)
    {
      pdc = mapCol->getPixRef(r, minW);
      pdm = mapPath->getPixRef(r, minW);
      for (c = minW; c < maxW; c++)
      { // get real map position
        mx = c * mapCellSize;
        my = (mapCol->height() - r) * mapCellSize;
        posfl.set(mx, my, 0.0);
        posi = getFloorToPix(posfl);
        w = roundi(posi.x);
        h = roundi(posi.y);
        if ((w >= 0) and (w < (int)imColor->width()) and
            (h >= 0) and (h < (int)imColor->height()))
        { // there is image data for this position
          psm = imPathMask->getPix(h, w);
          if (psm.p3 > 250)
          { // pixel is masked - so copy color pixel at
            // nearest position
            psc  = imColor->getPix(h, w);
            *pdc = psc;
            // set path as passable
            //pdm->tone(UPixel::pixRGB(0,0,0), 30);
            //*pdm = psm;
            pdm->tone(psm, 30);
          }
        }
        pdm++;
        pdc++;
      }
    }
  }
  //
  return result;
}


/////////////////////////////////////////////////////

bool UImgProj::doProjectPolygonToMapPix(
                      CvPoint * polygon,
                      int * polygonCnt,
                      bool * isObst)
{
  bool res = (polygon != NULL) and
             (polygonCnt != NULL) and
             (isObst != NULL);
  int i;
  CvPoint * point1;
  CvPoint * point2;
  UPosition pos;
  bool * isObst1;
  bool * isObst2;
  int p2Cnt = 0;
  bool belowHorizon, lastBelow = true;
  //
  if (res)
  { // convert image points to map points
    point1 = polygon;
    point2 = polygon;
    isObst1 = isObst;
    isObst2 = isObst;
    for (i = 0; i < *polygonCnt; i++)
    {
      pos = getPixToMapFloor(point1->x, point1->y, &belowHorizon);
      if (belowHorizon)
      { // use point only if valid (might be abowe horizon)
        point2->x = roundi(pos.x / mapCellSize);
        point2->y = mapCol->height() - roundi(pos.y / mapCellSize);
        point2++;
        p2Cnt++; // count valid points
        *isObst2 = *isObst1 and lastBelow;
        isObst2++;
        lastBelow = true;
      }
      else
        lastBelow = false;
      point1++;
      isObst1++;
    }
    // set the reduced polygon count
    *polygonCnt = p2Cnt;
  }
  //
  return res;
}

//////////////////////////////////////////////////////////////

bool UImgProj::doProjectPolygonToFloorReal(
                      CvPoint * polygon,
                      bool * isObst,
                      int polygonCnt,
                      UProbPoly * destPoly)
/*                      UPosition * polyReal,
                      bool * isObstReal,
                      int * polyRealCnt)*/
{
  bool res = (polygon != NULL) and
             (polygonCnt > 0) and
             (isObst != NULL);
  int i;
  CvPoint * point;
  float p1x, p1y, p2x, p2y;
  UPosition pos;
  URPos posPix;
  //UPosition * posd;
  bool * isObst1;
  //bool * isObst2;
  int p2Cnt = 0;
  bool belowHorizon, lastBelow = true;
  bool isOK;
  const double maxDist = 50.0;
  float minY, dy;
  //
  // get uppermost usable pixel y value
  // to be used a horizon value
  pos.set(maxDist, 0.0, 0.0);
  posPix = getFloorToPix(pos);
  minY = maxi(0, (int)posPix.y) + 1;
  //
  if (res)
  { // convert image points to map points
    point = polygon;
    //posd = polyReal;
    isObst1 = isObst;
    //isObst2 = isObstReal;
    p1x = point->x;
    p1y = point->y;
    // remove radial error
    camPar.getRadialD2U(p1x, p1y, &p1x, &p1y);
    p2x = p1x;
    p2y = p1y;
    lastBelow = (p2y > minY);
    for (i = 0; i < polygonCnt; i++)
    {
      belowHorizon = (p1y > minY);
      if (lastBelow and not belowHorizon)
      { // get position at max-distance instead
        dy = (p1y - p2y);
        p1x = p2x + (p1x - p2x) * (minY - p2y) / dy;
        p1y = minY;
      }
      else if (not lastBelow and belowHorizon)
      { // get position at max-distance instead
        dy = (p1y - p2y);
        p2x = p2x + (p1x - p2x) * (minY - p2y) / dy;
        p2y = minY;
        // add extra position below horizon
        pos = getPixToRobFloor(p2x, p2y, NULL);
        p2Cnt++; // count valid points
        isOK = destPoly->addPos(pos, *isObst1 and lastBelow);
        if (not isOK)
          break;
      }
      if (belowHorizon or lastBelow)
      { // convert to robot flat floor coordinates
        pos = getPixToRobFloor(p1x, p1y, NULL);
        p2Cnt++; // count valid points
        // add to flat-floor polygon
        isOK = destPoly->addPos(pos, *isObst1 and lastBelow);
        if (not isOK)
          break;
      }
      lastBelow = belowHorizon;
      p2x = p1x;
      p2y = p1y;
      point++;
      p1x = point->x;
      p1y = point->y;
      camPar.getRadialD2U(p1x, p1y, &p1x, &p1y);
      isObst1++;
    }
    // set the reduced polygon count
    //*polyRealCnt = p2Cnt;
  }
  //
  return res;
}

//////////////////////////////////////////////////////////////

bool UImgProj::doPolygonMap(UImage * imColor,
                      UImage * imPathMask,
                      CvPoint * polygon,
                      int polygonCnt,
                      bool * isObst)
{
  bool res = (imColor != NULL) and (imPathMask != NULL);
  unsigned int r,c;
  int w, h, i;
  double mx, my;
  UPixel * pdc;
  UPixel * pdm;
  CvPoint * point1;
  CvPoint * point2;
  UPosition pos;
  bool * isObst1;
  bool * isObst2;
  URPos posi;
  UPosition posfl;
  //
  clearPathMap();
  if (res)
    res = (mapCol != NULL) and (mapPath != NULL);
  if (res)
  { // path map need to be cleared, as
    // green color == 0 is used to
    // make the color map.
    clearPathMap();
    // need to be same size too
    res = (mapCol->width() == mapPath->width());
  }
  if (res)
  { // paint passable area inpath map, first as
    // oplygon
    cvFillPoly(mapPath->cvArr(), &polygon, &polygonCnt, 1,
                    CV_RGB(255,0,0), 4, 0);
  }
  //
  if (res)
  { // now paint the edges that are obstacles
    point1 = polygon;
    isObst1 = isObst;
    for (i = 0; i < polygonCnt - 1; i++)
    {
      point2 = point1++;
      isObst2 = isObst1++;
      if (*isObst1 or *isObst2)
        cvLine(mapPath->cvArr(), *point1, *point2, CV_RGB(0,0,0), 2, 8, 0);
    }
  }
  // now make an image
  // where pixels are copied from
  // source image to the color-map image
  if (res)
  {
    for (r = 0; r < mapCol->height(); r++)
    {
      pdc = mapCol->getPixRef(r, 0);
      pdm = mapPath->getPixRef(r, 0);
      for (c = 0; c < mapCol->width(); c++)
      { // get real map position
        if (pdm->p2 == 0)
        {
          mx = c * mapCellSize;
          my = (mapCol->height() - r) * mapCellSize;
          posfl.set(mx, my, 0.0);
          posi = getFloorToPix(posfl);
          w = roundi(posi.x);
          h = roundi(posi.y);
          if ((w >= 0) and (w < (int)imColor->width()) and
              (h >= 0) and (h < (int)imColor->height()))
          { // there is image data for this position
            *pdc  = imColor->getPix(h, w);
            pdc->swapRB();
          }
        }
        pdm++;
        pdc++;
      }
    }
  }
  // debug
  //printf("Row %d fetch\n", r);
  // debug end
  mapCol->saveBMP(imagePath, imColor->name, -1, "Col");
  mapPath->saveBMP(imagePath, imColor->name, -1, "Path");
  return res;
}

///////////////////////////////////////////////////

bool UImgProj::getImageName(
              const char * logImg,
              const char * logPos,
              const char * subDir,
              int imgNum,
              char * bufferImgName,
              const int bufferSize,
              UTime * imgTime,
              int hdrMode,
              int device)
{
  bool result = false;
  const int FN_MAX = 200;
  char s[FN_MAX];
  char fn[FN_MAX];
  FILE * fli;
  FILE * flp;
  char * res = NULL;
  int n, hdr, dev;
  unsigned long ts, tm, serial;
  double x,y,h;
  UTime t1, t2;

  //
  bufferImgName[0] = '\0';
  snprintf(fn, FN_MAX, "%s/%s/%s", imagePath, subDir, logImg);
  fli = fopen(fn, "r");
  snprintf(fn, FN_MAX, "%s/%s/%s", imagePath, subDir, logPos);
  flp = fopen(fn, "r");
  if (fli != NULL)
  { // find filename and image time
    n = 0;
    while (true)
    {
      res = fgets(s, FN_MAX, fli);
      if (res == NULL)
        break;
      n = sscanf(s, "%lu %d %d %lu.%lu %s",
         &serial, &dev, &hdr, &ts, &tm, fn);
      if (n >= 5)
        if ((int(serial) >= imgNum) and
              ((hdr == hdrMode) or (hdrMode == 0)) and
              (device == dev))
        { // set image time
          t1.setTime(ts, tm * 1000);
          // remove ".bmp"
          res = strstr(fn, ".bmp");
          if (res != NULL)
            *res = '\0';
          strncpy(bufferImgName, fn, bufferSize);
          result = true;
          break;
        }
    }
    fclose(fli);
  }
  if (result)
  { // now find robot pose at image time
    result = false;
    if (flp != NULL)
    {
      while (true)
      {
        res = fgets(s, FN_MAX, flp);
        if (res == NULL)
          break;
        n = sscanf(s, "%lu.%lu %lf %lf %lf",
          &ts, &tm, &x, &y, &h);
        if (n == 5)
        {
          t2.setTime(ts, tm);
          if ((t2 - t1) > 0.0)
          {
            robPose.set(x, y, h);
            result = true;
            break;
          }
        }
      }
      fclose(flp);
    }
    if (imgTime != NULL)
      *imgTime = t1;
  }
  return res;
}

//////////////////////////////////////////////////////////////

bool UImgProj::paintPath(const char * filename)
{
  bool res = (filename != NULL);
  FILE * odo = NULL;
  UTime t;
  double x, y, h;
  int n, i;
  unsigned long ts, tm;
  const int MFL = 400;
  char fn[MFL];
  char s[MFL];
  UPose old;
  double d;
  bool stop = false;
  bool first = true;
  //
  if (res)
  {
    snprintf(fn, MFL, "%s/%s", imagePath, filename);
    odo = fopen(fn, "r");
    res = odo != NULL;
  }
  if (res)
  {
    i = 0;
    while (not (feof(odo) or stop))
    {
      if (fgets(s, MFL, odo) == NULL)
        break;
      n = sscanf(s, "%lu.%lu %lf %lf %lf", &ts, &tm, &x, &y, &h);
      if (n == 5)
      {
        t.setTime(ts, tm);
        // debug
        //if (i > 15)
        //  stop = true;
        // debug end
        d = sqrt(sqr(x - old.x) + sqr(y - old.y));
        robPose.set(x, y, h);
        if (first)
        {
          old = robPose;
          first = false;
        }
        if ((d > 0.2) or (absd(limitToPi(h - old.h)) > 0.1) or stop)
        { // moved some distance, so paint
          if (i > 0)
            paintRobotPath(mapCol, old);
          old = robPose;
          i++;
        }
      }
    }
  }
  if (odo != NULL)
    fclose(odo);
  return res;
}


void UImgProj::paintText(UImage * img, int x,int y, const char * text)
{
  CvScalar black = CV_RGB(0, 0, 255);
  CvFont font;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
              1.0, 1.0, 0.0, 1, 8);
  cvPutText(img->cvArr(), text, cvPoint(x, y), &font, black);
}
