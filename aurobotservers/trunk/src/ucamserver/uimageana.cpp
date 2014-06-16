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

//#include "ucamcommon.h"
#include <ugen4/u2dline.h>
#include <math.h>

#include "uimageana.h"

/////////////////////////////////////////////////////////

UPixelLinked::UPixelLinked()
{ // clear
  r = 0;
  c = 0;
  nx = 0;
  px = 0;
}

/////////////////////////////////////////////////////////

bool UPixelLinked::getNext(UPixelLinked * from, int direction, int height, int width)
{
  bool res = true;
  //
  px = (direction + 2) % 4;
  nx = px; // next direction to try
  switch (direction)
  {
    case 0:
        c = from->c + 1;
        r = from->r;
        res = c < width;
        break;
    case 1:
        c = from->c;
        r = from->r + 1;
        res = r < height;
        break;
    case 2:
        c = from->c - 1;
        r = from->r;
        res = c >= 0;
        break;
    case 3:
        c = from->c;
        r = from->r - 1;
        res = r >= 0;
        break;
    default: res = false;
  }
  return res;
}

/////////////////////////////////////////////////////////

UPixelLinked UPixelLinked::operator= (UPixelLinked v)
{
  r = v.r;
  c = v.c;
  nx = v.nx;
  px = v.px;
  return *this;
}


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

UImageAna::UImageAna()
{
  img = NULL;
  storage = NULL;
  floorArea = NULL;
  floorAreaCnt = 0;
  obst = NULL;
  floorOutline = NULL;
  floorOutlineCnt = 0;
  imgPool = NULL;
}

/////////////////////////////////////////////////////////

UImageAna::~UImageAna()
{ // release allocated memory
  if (floorArea != NULL)
    free(floorArea);
  if (obst != NULL)
    free(obst);
  if (floorOutline != NULL)
    free(floorOutline);
}

////////////////////////////////////////////////////////

UPixel fire(double d)
{
  UPixel result(0,0,0);
  if (d < 255.0)
    result.p3 = roundi(d);
  else if (d < 511)
    result.p2 = roundi(d) - 256;
  else if (d < (256 * 3 - 1))
    result.p1 = roundi(d) - 256 * 2;
  else if (d < (256 * 4 - 1))
  {
    result.p2 = 255;
    result.p1 = roundi(d) - 256 * 3;
  }
  else if (d < (256 * 5 - 1))
  {
    result.p2 = 255;
    result.p1 = 255;
    result.p3 = 256 - (roundi(d) - 256 * 4);
  }
  else
    result.set(255, 255, 255);
  return result;
}

/////////////////////////////////////////////////////////

bool UImageAna::getAvgVar(UImage * img,
                unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UMatrix4 * mE, UMatrix4 * mC)
{
  bool result = img != NULL;
  UPixel * p;
  UMatrix4 mP(3, 1, 0.0); // pixel
  UMatrix4 mPt(1, 3, 0.0); // pixel
  UMatrix4 mSQ(3, 3, 0.0); // sqr
  unsigned int r, c, n;
  //
  if (result)
  { // NB! no further range check
    n = 0;
    mE->clear();
    mC->clear();
    for (r = 0; r < h; r++)
    {
      p = &img->getLine(y)[x];
      for (c = 0; c < w; c++)
      {
        mP.set(double(p->p1), double(p->p2), double(p->p3));
        mE->add(&mP);
        n++;
        p++;
      }
    }
    mE->mult(1.0/double(n));
    for (r = 0; r < h; r++)
    {
      p = &img->getLine(y)[x];
      for (c = 0; c < w; c++)
      {
        mP.set(double(p->p1), double(p->p2), double(p->p3));
        mP.sub(mE);
        mPt.transpose(&mP);
        mSQ.mult(&mP, &mPt);
        mC->add(&mSQ);
        p++;
      }
    }
    mC->mult(1.0/double(n));
  }
  return result;
}


/////////////////////////////////////////////////////////

bool UImageAna::findRoad(unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UImage * resi, bool debug, int imgNum, char * imgN)
{
  bool result = img != NULL;
  UPixel * ps;
  UPixel * pd;
  UMatrix4 mP(3, 1, 0.0); // pixel
  UMatrix4 mE(3, 1, 0.0); // mean
  UMatrix4 mC(3, 3, 0.0); // covariance
//  UMatrix4 mE2(3, 1, 0.0); // mean
//  UMatrix4 mC2(3, 3, 0.0); // covariance
  unsigned int r,c;
  double d;
  const int FN_SIZE = 200;
  char s[FN_SIZE];
  //
  if (result)
  { // test result area
    result = (((x + w) < img->width()) and
              ((y + h) < img->height()));
  }
  if (result)
  {
    // blur the image
    resi->copy(img);
    //resi->setROI(20, 100, 280, 100);
    cvSmooth(resi->cvArr(), img->cvArr(), CV_MEDIAN, 7, 7, 0);
    //cvSmooth(resi->cvArr(), img->cvArr(), CV_GAUSSIAN, 5, 5, 0);
    //cvSmooth(resi->cvArr(), img->cvArr(), CV_BLUR, 31, 1, 0);
    //
    // make sample statistics
    getAvgVar(img, x, y, w, h, &mE, &mC);
    // result need to be valid
    result = (resi != NULL);
  }
  if (result)
  { // test rest of image with this
    // value
    result = resi->copyMeta(img, true);
  }
  if (result)
  { // prepare result
    //resi->tone(UPixel::pixRGB(255,255,255), 50);
    resi->setSize(img->height(), img->width(), 3, 8, "BW");
    for (r = 0; r < img->height(); r++)
    { // get compare sample
      //getAvgVar(img, x, r, 11, 1, &mE, &mC);
      ps = img->getLine(r);
      pd = resi->getLine(r);
      for (c = 0; c < img->width() ; c++)
      { // get color distance
        mP.set(double(ps->p1), double(ps->p2), double(ps->p3));
        d = cvMahalanobis( mP.cvArr(), mE.cvArr(), mC.cvArr());
        // get variance distance
        /*
        if ((c > 5) and (c < (img->width() - 5)) and
            (r > 2) and (r < (img->height() - 2)))
        { // is within image
          // getAvgVar(img, x, y, w, h, &mE, &mC);
          getAvgVar(img, c-5, r, 11, 1, &mE2, &mC2);
          dv = sqrt(absd(mC2.get(0,0) - mC.get(0,0)) +
                    absd(mC2.get(1,1) - mC.get(1,1)) +
                    absd(mC2.get(2,2) - mC.get(2,2)));
        }
        else
          dv = 0;
        */
        *pd = fire(d);
        ps++;
        pd++;
      }
    }
  }
  if (result)
  {    // save as well
    // paint sample area in original
    cvRectangle( img->cvArr(), cvPoint(x,y), cvPoint(x+w, y+h),
                  CV_RGB(255,0,0), 1, 8);
    // paint sample area in result
    cvRectangle( resi->cvArr(), cvPoint(x,y), cvPoint(x+w, y+h),
                  CV_RGB(255,0,0), 1, 8);
    //
    snprintf(s, FN_SIZE, "%s/%s-sMed7.bmp", imagePath, imgN);
    if (img->saveBMP(s))
      printf("Saved %s\n", s);
    else
      printf("Save failed for %s\n", s);
    snprintf(s, FN_SIZE, "%s/%s-resMed7.bmp", imagePath, imgN);
    if (resi->saveBMP(s))
      printf("Saved %s\n", s);
    else
      printf("Save failed for %s\n", s);
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool UImageAna::findRoadCroma(unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UImage * resi, bool debug, int imgNum, char * imgN, int pct)
{
  bool result = img != NULL;
  UPixel * ps;
  UPixel * pd;
  UMatrix4 mP(3, 1, 0.0); // pixel
  UMatrix4 mE(3, 1, 0.0); // mean
  UMatrix4 mC(3, 3, 0.0); // covariance
  UMatrix4 mCi(3, 3, 16.0); // covariance
  unsigned int r,c;
  double d;
  unsigned char v1,v2, v3;
  const int FN_SIZE = 200;
  char s[FN_SIZE];
  UPixel * pixs;
  UPixel pix;
  //
  if (result)
  { // test result area
    result = (((x + w) < img->width()) and
              ((y + h) < img->height()));
  }
  if (result)
  { // convert to croma only
    resi->copy(img);
    if (false)
      distFilt(resi, img, // source and destination image
                15, 3,           // width of filter in pixels
                100, 240);        // min and max row to filter
    else
      distFiltH(resi, img, // source and destination image
                15, 3,           // width of filter in pixels
                100, 240, 5);        // min and max row to filter
    snprintf(s, FN_SIZE, "%s/%s-%02dFilt.bmp",
                imagePath, imgN, imgNum);
    img->saveBMP(s);
    pixs = img->getData();
    for (r = 0; r < img->height() * img->width(); r++)
    {
      pix = pixs->asYUV(PIX_PLANES_BGR);
      if (pix.y > 40)
      {
        if (pct > 50)
          pix.y = (pix.y * pct) / 100;
        else
          pix.y = 120 + (pix.y * pct) / 100;
      }
      else
        pix.y = 0;
      *pixs = pix.asBGR(PIX_PLANES_YUV);
      pixs++;
    }
    // blur the image
    //resi->copy(img);
    //resi->setROI(20, 100, 280, 100);
    //cvSmooth(resi->cvArr(), img->cvArr(), CV_MEDIAN, 7, 7, 0);
    //cvSmooth(resi->cvArr(), img->cvArr(), CV_GAUSSIAN, 5, 5, 0);
    //cvSmooth(resi->cvArr(), img->cvArr(), CV_BLUR, 31, 1, 0);
    //
    // make sample statistics
    getAvgVar(img, x, y, w, h, &mE, &mC);
    mE.print("mE");
    mC.add(&mCi); // add e 4 intensity level uncertanty
    mC.print("mC");
    mCi.inverse(&mC);
    // result need to be valid
    result = (resi != NULL);
  }
  if (result)
  { // test rest of image with this
    // value
    result = resi->copyMeta(img, true);
  }
  if (result)
  { // prepare result
    resi->setSize(img->height(), img->width(), 3, 8, "BGR");
    for (r = 0; r < img->height(); r++)
    { // get compare sample
      ps = img->getLine(r);
      pd = resi->getLine(r);
      for (c = 0; c < img->width() ; c++)
      { // get color distance
        mP.set(double(ps->p1), double(ps->p2), double(ps->p3));
        d = cvMahalanobis( mP.cvArr(), mE.cvArr(), mCi.cvArr());
        //*pd = fire(d*5);
        v1 = (unsigned char)mind(255, maxd(0, d * 3));
        if (d*d < 9)
          v2 = 255; // green
        else
          v2 = 0;
        if (d*d < 44) // red
          v3 = 255;
        else
          v3 = 0;
        pd->set(v1,v2,v3);
        ps++;
        pd++;
      }
    }
  }
  if (result)
  {    // save as well
    // paint sample area in original
    cvRectangle( img->cvArr(), cvPoint(x,y), cvPoint(x+w, y+h),
                  CV_RGB(255,0,0), 1, 8);
    // paint sample area in result
    cvRectangle( resi->cvArr(), cvPoint(x,y), cvPoint(x+w, y+h),
                  CV_RGB(255,0,0), 1, 8);
    //
    snprintf(s, FN_SIZE, "%s/%s-%02dsdCromY%d.bmp",
                 imagePath, imgN, imgNum, pct);
    if (img->saveBMP(s))
      printf("Saved %s\n", s);
    else
      printf("Save failed for %s\n", s);
    snprintf(s, FN_SIZE, "%s/%s-%02dresCromY%d.bmp",
                imagePath, imgN, imgNum, pct);
    if (resi->saveBMP(s))
      printf("Saved %s\n", s);
    else
      printf("Save failed for %s\n", s);
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool UImageAna::findRoadFill(unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UImage * resi, bool debug, int camDev,
                char * imgN, int pct, bool lookingLeft)
{
  bool result = (img != NULL);
  UImage * imgDist;
  UImage * imgFilt;
  UImage * imgEdge;
  //UImage * imgFill;
  UImage * imgMask;
  //UMatrix4 mK(3,3);
  UPixel * p;
  UPixel pc;
  unsigned int r, c;
  unsigned char * pp;
  bool tooHigh = false, tooSmall = false;
  int edgeSize = pct;
  int loopCnt = 0;
  bool hopeless = false;
  int lastDirec = 0;
  const int tooDark = 70; // Y value for stay-away
  UTime t1, t2;
  //
  if (pct == 0)
    edgeSize = 22;
  else
    edgeSize = pct;
  //
  if (result)
  { // test result area
    result = (((x + w) < img->width()) and
              ((y + h) < img->height()));
  }
//  imgDist = imgPool.getImage(3, true);
  imgDist = resi;
  imgEdge = imgPool->getImage(3, true);
  imgFilt = imgPool->getImage(4, true);
  //imgFill = imgPool.getImage(5, true);
  imgMask = imgPool->getImage(9, true);
  result = ((imgDist != NULL) and (imgEdge != NULL) and
            (imgFilt != NULL) and (imgMask != NULL));
  if (not result)
    printf("*** UImageAna::findRoadFill: Could not allocate all needed images\n");
  if (result)
  { // median-filter with varying kernel size
    if (false)
      distFilt(img, imgDist, // source and destination image
            17, 3,    // width of filter in pixels - max (at max row) and min (at min row)
            40, 240); // min and max row to filter
    else
      distFiltH(img, imgDist, // source and destination image
            23, 3,    // width of filter in pixels - max (at max row) and min (at min row)
            40, 240, 5); // min and max row to filter
    // save result
    imgDist->saveBMP(imagePath, imgN, camDev, "DistFilt");
    //
    // size result images
    imgEdge->setSize(img->height(), img->width(), 3, 8, "BGR");
    imgFilt->setSize(img->height(), img->width(), 3, 8, "BGR");
    //imgFill->setSize(img->height(), img->width(), 3, 8, "BGR");
    imgMask->setSize(img->height()+2, img->width()+2, 1, 8, "BW");
    //
    // blur the image slightlt
    //cvSmooth(resi->cvArr(), img->cvArr(), CV_MEDIAN, 7, 7, 0);
    if (false)
      cvSmooth(imgDist->cvArr(), imgFilt->cvArr(), CV_GAUSSIAN, 3, 3, 0);
    else
      imgFilt->copy(imgDist);
    //cvSmooth(resi->cvArr(), img->cvArr(), CV_BLUR, 31, 1, 0);
    //
    //imgFilt->saveBMP(imagePath, imgN, imgNum, "Gaus3x3");
    //
    //void cvSobel( const CvArr* src, CvArr* dst,
    //           int xorder, int yorder, int aperture_size=3 );
    //cvSobel( imgFilt->cvArr(), imgEdge->cvArr(), 1, 1, 3);
    //cvFilter2D( const CvArr* src, CvArr* dst, const CvMat* kernel,
    //                     CvPoint anchor=cvPoint(-1,-1));
    //imgFilt->toYUV();
    p = imgFilt->getData();
    for (r = 0; r < imgFilt->height() * imgFilt->width(); r++)
    {
      pc = p->asYUV(PIX_PLANES_BGR);
      if (pc.y > tooDark)
      {
//        if (pc.y > 150)
//          pc.y = 200;
//        else
        pc.y = pc.y/3 + 100;
        // enhance colour saturation
        pc.u = mini(255, maxi(0, (pc.u - 128) * 2 + 128));
        pc.v = mini(255, maxi(0, (pc.v - 128) * 2 + 128));
      }
      else
        // do not drive into dark allyes
        pc.set(0,0,255);
      // next pixel
      *p++ = pc.asRGB(PIX_PLANES_YUV);
    }
    // find edges using a Sobel kernel
    imgFilt->edgeSobel(imgEdge);
    //
    // paint seed area for flood-fill
    // void cvRectangle( CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color,
    //              int thickness=1, int line_type=8, int shift=0 );
    cvRectangle(imgEdge->cvArr(), cvPoint(x,y),
                 cvPoint(x+w, y+h), CV_RGB(0,0,0), -1, 8, 0);
    //
    imgEdge->saveBMP(imagePath, imgN, camDev, "Edge");
    //
    // imgFill->copy(imgEdge);
    //cvFloodFill( CvArr* image, CvPoint seed_point, CvScalar new_val,
    //   CvScalar lo_diff=cvScalarAll(0), CvScalar up_diff=cvScalarAll(0),
    //   CvConnectedComp* comp=NULL, int flags=4, CvArr* mask=NULL );
    loopCnt = 0;
    while (not hopeless)
    {
      imgMask->clear(0);
      //
    // debug timing
    t1.Now();
    // debug end timing
      cvFloodFill( imgEdge->cvArr(), cvPoint(x + w/2,y + h/2),
                    CV_RGB(255,0,0),
                    cvScalarAll(edgeSize), cvScalarAll(edgeSize),
                    NULL, 8 | CV_FLOODFILL_MASK_ONLY, imgMask);
      /*
      cvFloodFill( imgEdge->cvArr(), cvPoint(x+w,y), CV_RGB(255,0,0),
                    cvScalarAll(edgeSize), cvScalarAll(edgeSize),
                    NULL, 4 | CV_FLOODFILL_MASK_ONLY, imgMask);
      cvFloodFill( imgEdge->cvArr(), cvPoint(x+w,y+h), CV_RGB(255,0,0),
                    cvScalarAll(edgeSize), cvScalarAll(edgeSize),
                    NULL, 4 | CV_FLOODFILL_MASK_ONLY, imgMask);
      cvFloodFill( imgEdge->cvArr(), cvPoint(x,y+h), CV_RGB(255,0,0),
                    cvScalarAll(edgeSize), cvScalarAll(edgeSize),
                    NULL, 4 | CV_FLOODFILL_MASK_ONLY, imgMask);
      */
    // debug timing
    t2.Now();
    printf("Flood fill took %f sec\n", t2 - t1);
    // debug end timing
      //
      // enhance contrast
      for (r = 0; r < imgDist->height(); r++)
      {
        p = imgDist->getLine(r);
        pp = (unsigned char *)imgMask->getLine(r);
        pp++;
        for (c = 0; c < imgDist->width(); c++)
        {
          if ((*pp) > 0)
            p->set(255,0,0);
          pp++;
          p++;
        }
      }
      //
      imgDist->saveBMP(imagePath, imgN, camDev, "_Mask");

      // find contours
      if (floorArea == NULL)
      {
        floorArea = (CvPoint *) malloc(MAX_POLYGON_POINTS * sizeof(CvPoint));
        obst = (bool *) malloc(MAX_POLYGON_POINTS * sizeof(bool));
      }
      // debug timing
      t1.Now();
      // debug end timing
      floorAreaCnt = findContour(imgMask, img, floorArea,
                        MAX_POLYGON_POINTS, &tooHigh, &tooSmall, lookingLeft);
      // debug timing
      t2.Now();
      printf("Find contour took %f sec\n", t2 - t1);
      // debug end timing
      //
      // stop if reasonable size
      if ((not tooHigh) and (not tooSmall))
        break;
      else
      { // adjust parameter to get better result
        // debug
        printf("Bad fill tooHigh %s, tooSmall %s, edge = %d\n",
             bool2str(tooHigh), bool2str(tooSmall), edgeSize);
        // debug end
        if (tooHigh)
        {
          edgeSize--;
          lastDirec = -1;
        }
        else
        { // need to increase treshold
          // where last direction were negative
          if (lastDirec < 0)
            break;
          edgeSize++;
          lastDirec = 1;
        }
      }
      if ((edgeSize < 10) or (edgeSize > 45))
        // hopeless - stop
        hopeless = true;
      loopCnt++;
    }
    // debug
    // paint best contour in blue
    cvPolyLine(img->cvArr(), &floorArea, &floorAreaCnt, 1, true,
                         CV_RGB(0,0,255), 1, 4, 0);

    img->saveBMP(imagePath, imgN, camDev, "Floor");
  }
  return not hopeless;
}

/////////////////////////////////////////////////////////////////////

bool UImageAna::findRoadPoly(
                bool debug, int camDev,
                char * imgN, double ballance, double limit,
                double lookingLeft, 
                int x1, int y1, int x2, int y2,
                bool colorCorrect)
{
  bool res = (img != NULL);
  UImage * imgDist;
  UImage * imgC = NULL;
  UImage * imgCroma = NULL;
  //UMatrix4 mK(3,3);
  //UPixel pc;
  UPixel * bgr;
  //int outlineCnt;
  bool tooSmall = true;
  UTime t1, t2;
  int reduc = 1;
  const int MSL = 300;
  char s[MSL];
  int i;
  int maxH;
  // get images from pool
  //imgCroma = imgPool->getImage(2, true); // cromaticity info
  imgC = imgPool->getImage(9, true);    // image for comtour result
  res = ((imgC != NULL) and (img != NULL));
  if (not res)
    printf("*** UImageAna::findRoadFill: Could not allocate all needed images\n");
  if (res)
  { // convert to BGR
    img->toBGR(NULL);
    // median-filter with varying kernel size
    // debug timing
    t1.Now();
    // debug end timing
    if (img->width() == 160)
      reduc = 2;
    //
    if (false)
    {
      imgDist = imgPool->getImage(7, true); // distance median image
      res = (imgDist != NULL);
      if (res)
      {
        distFiltH(img, imgDist, // source and destination image
              15/reduc, 3,    // width of filter in pixels - max (at max row) and min (at min row)
              40/reduc, 240/reduc, 5/reduc); // min and max row to filter
        snprintf(s, MSL, "%s-median", img->name);
        imgDist->setNameAndExt(s, "");
      }
    }
    else
    {
      imgDist = img;
      //snprintf(s, MSL, "%s-raw", img->name);
    }
    // debug timing
    //t2.Now();
    //printf("distFiltH took %f sec\n", t2 - t1);
    // debug end timing
    // set image name - for image monitor
    // size result images
    if (imgC != NULL)
    {
      imgC->copy(img);
      imgC->setNameAndExt("ImgC", "");
      // debug correct images colour
      if (colorCorrect)
      {
        printf("Correctiong for too red image (outdoor white ball)\n");
        bgr = imgC->getLine(0);
        for (i = 0; i < int(imgC->width() * imgC->height()); i++)
        { // compensate for white ballance "outdoor" without IR filter
          bgr->p1 = mini(255, maxi(0, int(bgr->p1 * 2) - 64)); // blue
          bgr->p2 = mini(255, maxi(0, int(bgr->p2 * 2) - 64)); // green
          bgr->p3 = mini(255, maxi(0, int((bgr->p3 * 4) / 3) - 64)); // red
          bgr++;
        }
      }
      // debug end
    }
    if (imgCroma != NULL)
    {
      imgCroma->copyMeta(img, true);
      imgCroma->setNameAndExt("probRoad", "");
    }
    //
    maxH = mini(imgDist->height(), maxi(y1, y2) + 10);
    // find floor area from seed point(s)
    //outlineCnt = MAX_POLYGON_OUTLINE_POINTS;
    res = findContourPolyCroma(imgDist, imgC, imgCroma,
                               &tooSmall, lookingLeft,
                               x1, y1, x2, y2, maxH,
                               ballance, limit);
  }
  if (imgC != NULL)
    imgC->imgUpdated();
  if (res and not tooSmall and (imgC != NULL))
  { // reduce outline - with tollerence at bottom and line for "zero" tollerance
    // paint best contour in blue
    cvPolyLine(imgC->cvArr(), &floorArea, &floorAreaCnt, 1, true,
                         CV_RGB(0, 255, 0), 2, 4, 0);

  }
  return res;
}

//////////////////////////////////////////////

int UImageAna::countContourEdges(CvSeq * contours)
{
  int res = 0;
  CvSeqBlock * block;
  CvSeq * seq;
  //
  if (contours != NULL)
  {
    seq = contours;
    while (true)
    { // try all blocks in seqence list
      block = seq->first;
      while (true)
      { // get count in this contour
        res += block->count;
        // advance to next contour
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        seq = seq->h_next;
    }
  }
  //
  return res;
}

//////////////////////////////////////////////

int UImageAna::countContourLimits(CvSeq * contours, int * left,
         int * right, int * top, int * bottom, bool lookingLeft)
{
  int res = 0;
  CvSeqBlock * block;
  CvSeq * seq;
  CvPoint * point;
  int l = 1000, r = 0, t = 1000, b = 0;
  int i;

  //
  if (contours != NULL)
  {
    seq = contours;
    while (true)
    { // try all blocks in seqence list
      block = seq->first;
      while (true)
      { // get count in this contour
        res += block->count;
        //
        point = (CvPoint *)block->data;
        for (i = 0; i < block->count; i++)
        {
          if (point->x < l)
            l = point->x;
          if (point->x > r)
            r = point->x;
          if (point->y > b)
            b = point->y;
          if (lookingLeft)
          { // top is only valid to the left
            // of the image
            if (point->x < 50)
              if (point->y < t)
                t = point->y;
          }
          else
          {
            if (point->x > 270)
              if (point->y < t)
                t = point->y;
          }
          point++;
        }
        // advance to next contour
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        seq = seq->h_next;
    }
  }
  if (left != NULL)
    *left = l;
  if (right != NULL)
    * right = r;
  if (bottom != NULL)
    *bottom = b;
  if (top != NULL)
    * top = t;
  //
  return res;
}

//////////////////////////////////////////////

int UImageAna::findContour(UImage * imgMask, UImage * dest,
                     CvPoint * flp, int flpMax,
                     bool * tooHigh, bool * tooSmall, bool lookingLeft)
{
  bool isOK;
//  int levels = 3;
  CvSeq * contours = NULL;
  //CvMemStorage * storage = cvCreateMemStorage(0);
  CvPoint * point1 = NULL;
  CvPoint * point2;
  CvSeqBlock * block;
  bool last = false;
  int i, cnt;
  bool * ob; // border point
  CvSeq * seq;
  const int TOO_SMALL_LIMIT = 70; // edge count of area
  const int TOO_HIGH_LIMIT = 3; // pixels from top of image
  int high;
  UTime t1, t2;
  //
  //clearStorage();
  if (storage == NULL)
    storage = cvCreateMemStorage(0);
  //
  isOK = ((img != NULL) and (dest != NULL));
  if (isOK)
  {
    // debug timing
    t1.Now();
    // debug end timing
    cvFindContours( imgMask->cvArr(), storage, &contours, sizeof(CvContour),
                    CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
    // debug timing
    t2.Now();
    printf("cvFindContours took %f sec\n", t2 - t1);
    // debug end timing
    //
    if (tooSmall != NULL)
    {
      i = countContourEdges(contours);
      *tooSmall = (i < TOO_SMALL_LIMIT);
      printf ("Fount contours with %d points\n", i);
    }
    //
    // debug timing
    t1.Now();
    // debug end timing
    // comment this out if you do not want approximation
    contours = cvApproxPoly( contours, sizeof(CvContour), storage,
                            CV_POLY_APPROX_DP, // only method supported
                            2.8,        // accuracy
                            1 );        // 0 = not closed, 1 = closed curve
    // debug timing
    t2.Now();
    printf("cvApproxPoly took %f sec\n", t2 - t1);
    // debug end timing
    //
    if (tooHigh != NULL)
    {
      i = countContourLimits(contours, NULL, NULL, &high, NULL, lookingLeft);
      *tooHigh = (high < TOO_HIGH_LIMIT);
      printf("Path top pixel is %d\n", high);
    }
    //printf("Reduced contours to %d points\n", contours->first->count);
    //
    cnt = 0;
    i = 0;
    seq = contours;
    while (true)
    { // try all sequences in seqence list
      block = seq->first;
      while (true)
      { // find the contour witth the largest point-set
        if (block->count > cnt)
        { // save best
          point1 = (CvPoint *) block->data;
          cnt = block->count;
        }
        i++;
        // advance to next contour
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        seq = seq->h_next;
    }
    // copy the largest contour
    cnt = mini(cnt, flpMax);
    memcpy(flp, point1, cnt * sizeof(CvPoint));
    printf("Saved %d CvPoints (best of %d contours)\n", cnt, i);
    // mark if border-point
    ob = obst;
    point1 = flp;
    for (i = 0; i < cnt; i++)
    { // if point is near edge, then not confirmed obstacle
      *ob++ = not ((point1->x < 13) or (point1->x > int(imgMask->width() - 14)) or
                   (point1->y < 13) or (point1->y > int(imgMask->height() - 14)));
      point1++;
    }
    //
    if (true)
    { // draw largest contour
      cvPolyLine(img->cvArr(), &flp, &cnt, 1, true,
                         CV_RGB(255,0,0), 1, 4, 0);
    }
    else
    {
      block = contours->first;
      while (not last)
      { // paint contour into source image
        point2 = (CvPoint *) block->data;
        for (i = 1; i < block->count; i++)
        {
          point1 = point2++;
          //void cvLine( CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color,
          //       int thickness=1, int line_type=8, int shift=0 )
          cvLine(img->cvArr(), *point1, *point2, CV_RGB(255,0,0), 1, 4, 0);
        }
        // close contour
        point1 = (CvPoint *) block->data;
        cvLine(img->cvArr(), *point1, *point2, CV_RGB(255,0,0), 1, 4, 0);
        //
        block = block->next;
        last = (block == contours->first);
      }
    }
    // point[4]
    //cvDrawContours( imgMask->cvArr(), contours, CV_RGB(255,0,0),
    //                CV_RGB(0,255,0), levels-3, 3, CV_AA );
    // remove data
    cvClearSeq(contours);
    cvClearMemStorage( storage ); // sets the storage as empty (but allocated)
  }
  //
  return cnt;
}


//////////////////////////////////////////

char * UImageAna::getImageName(
                  const char * list, const char * subdir,
                  int n,
                  char * buff, int buffLng)
{ // get name of file n in the list
  const int FN_MAX = 200;
  char s[FN_MAX];
  char fn[FN_MAX];
  FILE * fl;
  char * res = NULL;
  int i;
  //
  buff[0] = '\0';
  snprintf(fn, FN_MAX, "%s/%s/%s", imagePath, subdir, list);
  fl = fopen(fn, "r");
  if (fl != NULL)
  {
    for (i = 0; i < n; i++)
    {
      res = fgets(s, FN_MAX, fl);
      if (res == NULL)
        break;
    }
    if (res != NULL)
    res = strstr(s, ".bmp");
    if (res != NULL)
    {
      res[0] = '\0';
      strncpy(buff, s, buffLng);
      res = buff;
    }
    fclose(fl);
  }
  return res;
}

//////////////////////////////////////////////////////////

int UImageAna::pixCmp(UPixel * p1, UPixel * p2)
{ // p1-
  int a,b;
  a = p1->p1 + p1->p2 + p1->p3;
  b = p2->p1 + p2->p2 + p2->p3;
  return a - b;
}

////////////////////////////////////////////////////////////

UPixel * UImageAna::pixSort(UPixel *** ps, int pixCnt)
{
  UPixel ** p;
//  UPixel * px;
  int i, j;
  // boble sort
  for (i = 1; i < pixCnt; i++)
  {
    for (j = i; j > 0; j--)
    {
      if (pixCmp(*ps[j], *ps[j-1]) < 0)
      { // swap
        p = ps[j];
        ps[j] = ps[j-1];
        ps[j-1] = p;
      }
      else
        break;
    }
  }
  // debug
  /*
  for (i = 0; i < pixCnt; i++)
  {
    px = *ps[i];
    px->print("");
  }
  */
  // debug end
  return *ps[pixCnt >> 1];
}

/////////////////////////////////////////////////////////////////

bool UImageAna::distFilt(UImage * simg, UImage * dimg, // source and destination image
              unsigned int maxW, unsigned int minW,           // width of filter in pixels
              unsigned int minR, unsigned int maxR)           // min and max row to filter
{
  unsigned int r, c;
  unsigned int m, mh;
  UPixel * pixs;
  UPixel * pixd;
  float wpr; // width per pixel
  const unsigned int MAX_PIXS = 40;
  UPixel ** pps[MAX_PIXS];
  UPixel * ps[MAX_PIXS];
  unsigned int pixn;
  //
  wpr = float(maxW - minW)/float(maxR - minR);
  m = minW;
  dimg->copy(simg);
  for (c = 0; c < MAX_PIXS; c++)
    // make medan sort pointers to point af buffer elements
    pps[c] = &ps[c];
  // filter all relevant rows
  for (r = minR; r < maxR; r++)
  { // get mask width
    m = minW + int(wpr * (r - minR));
    if (m % 2 == 0)
      // make sure mask width is odd
      m++;
    // get half mask width
    mh = m >> 1;
    // fill median buffer at start of line
    pixs = simg->getLine(r);
    for (c = 0; c < m; c++)
      ps[c] = pixs++;
    // set pointer to next element to replace
    pixn = 0;
    // get pointer to destination element
    pixd = &dimg->getLine(r)[mh];
    for (c = mh; c < (simg->width() - mh); c++)
    { // save result in destination image
      // of median pixel
      *pixd++ = *pixSort(pps, m);
      // fill next pixel into median buffer
      ps[pixn++] = pixs++;
      // advance pointer to oldest pixel
      if (pixn >= m)
        pixn = 0;
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////

bool UImageAna::distFiltH(UImage * simg, UImage * dimg, // source and destination image
              unsigned int maxW, unsigned int minW,           // width of filter in pixels
              unsigned int minR, unsigned int maxR,           // min and max row to filter
              unsigned int maxMaskHeight)
{
  const unsigned int MAX_MASK_H = 3;
  const unsigned int MAX_PIXS = 80;
  unsigned int r, c, i, sl;
  unsigned int mw, mwh;// mask width and half width
  unsigned int mh;     // mask height
  UPixel * pixs[MAX_MASK_H];
  UPixel * pixd;
  float wpr; // width per pixel
  UPixel ** pps[MAX_PIXS * MAX_MASK_H];
  UPixel * ps[MAX_PIXS * MAX_MASK_H];

  unsigned int pixn;
  //
  wpr = float(maxW - minW)/float(maxR - minR);
  mw = minW;
  dimg->copy(simg);
  for (c = 0; c < MAX_PIXS * MAX_MASK_H; c++)
    // make medan sort pointers to point af buffer elements
    pps[c] = &ps[c];
  // start with smallest mask 1 pixel high
  mh = 1;
  // filter all relevant rows
  for (r = minR; r < maxR; r++)
  { // get mask width
    mw = minW + int(wpr * (r - minR));
    if (mw % 2 == 0)
      // make sure mask width is odd
      mw++;
    // get half mask width
    mwh = mw >> 1;
    // increase mask height to about half mask width
    if (mwh > mh)
      mh = mini(mh + 2, maxMaskHeight);
    // fill median buffer at start of line
    for (i = 0; i < mh; i++)
    { // get filter source line
      sl = r - 1 + i;
      if (sl >= simg->height())
        sl = simg->height()-1;
      pixs[i] = simg->getLine(sl);
      for (c = 0; c < mw; c++)
        ps[c + i * mw] = pixs[i]++;
    }
    // set pointer to next element to replace
    pixn = 0;
    // get pointer to destination element
    pixd = &dimg->getLine(r)[mwh];
    for (c = mwh; c < (simg->width() - mwh); c++)
    { // save result in destination image
      // of median pixel
      *pixd++ = *pixSort(pps, mw * mh);
      // fill next pixel into median buffer
      for (i = 0; i < mh; i++)
        ps[pixn + i * mw] = pixs[i]++;
      // advance pointer to oldest pixel
      pixn++;
      if (pixn >= mw)
        pixn = 0;
    }
  }
  return true;
}

//////////////////////////////////////////////////

bool UImageAna::imageUsable(UImage * img)
{
  bool res = img != NULL;
  unsigned int r,c;
  UPixel px;
  const int step = 10;
  long ys = 0;
  long yss = 0;
  int pixCnt = 0;
  int y;
  //
  if (res)
    res = img->isValid();
  if (res)
  {
    for (r = step/2; r < img->height(); r += step)
    {
      for (c = step/2; c < img->width(); c += step)
      {
        px = img->getPix(r, c);
        if (img->isYUV() or img->isYUV420())
          y = px.y;
        else
          y = (px.p1 + px.p2 + px.p3)/3;
        ys += y;
        yss += y * y;
        pixCnt++;
      }
    }
    // make statistics
    ys /=  pixCnt;
    yss = yss/pixCnt - ys * ys;
  }
  //
  if (res)
    // too bright -- too dark
    res = (ys > 30) and (ys < 210);
  if (res)
    // too uniform -- gray wall etc
    res = yss > 100;
  //
  return res;
}

//////////////////////////////////////////////////////////
/*
bool UImageAna::imageContour(UImage * img, UImage * imgC,
                             unsigned int row, unsigned int col,
                             UPixelLinked * outlineBuffer,
                             int * outlineBufferCnt,
                             int diff, int diffa)
{ // find contour of all pixels in img
  // that are close to seed-point
  // write desition value in imgb and points in buffer
  bool res = (img != NULL);
  UPixelLinked * px1 = outlineBuffer;
  UPixelLinked * px2;
  UPixelLinked * px3;
  int bufCnt = 0;
  UPixel * pix1;
  UPixel * pix2;
  UPixel ref;
  bool finished = false;
  int d, j, i;
  unsigned int w, h;
  bool edge;
  bool isOK;
  UPixel rgb(255,0,0);
  int loops = 0;
  const int MAX_USED = 5000;
  CvPoint used[MAX_USED];
  CvPoint * usedNext = used;
  CvPoint * usedP;
  int usedCnt = 0;
  // debug
  FILE * flog = NULL;
  //flog = fopen("/home/chr/chr/images/median.log", "w");
  // debug end
  //

  if (res)
  {
    w = img->width();
    h = img->height();
    res = ((row < h) and (col < w));
  }
  if (res)
  {
    ref = img->getPix(row, col);
    px1->r = row;
    px1->c = col;
    px1->px = 0;
    px1->nx = 0;
    bufCnt++; // first element is reference
    pix1 = &ref;
  }
  while (res and not finished)
  { // overflow test
    if (bufCnt >= *outlineBufferCnt)
      break;
    // get refernce point
    px1 = &outlineBuffer[bufCnt-1];
    pix1 = img->getPixRef(px1->r, px1->c);
    px2 = px1;
    px2++;
    //px2->px = 0;
    // get the next untested direction
    d = (px1->nx + 1) % 4;
    isOK = false;
    while (d != px1->px)
    {
      px1->nx = d;
      isOK = px2->getNext(px1, d, h, w);
      if (isOK)
      {
        pix2 = img->getPixRef(px2->r, px2->c);
        // edge detect
        edge = absi(pix1->r - pix2->r) > diff;
        if (not edge)
          edge = absi(pix1->g - pix2->g) > diff;
        if (not edge)
          edge = absi(pix1->b - pix2->b) > diff;
        // test also absolute from reference
        if (not edge)
          edge = absi(pix1->r - ref.r) > diffa;
        if (not edge)
          edge = absi(pix1->g - ref.g) > diffa;
        if (not edge)
          edge = absi(pix1->b - ref.b) > diffa;
      }
      // debug
      if (flog != NULL)
        fprintf(flog, "#%3d px1: %3d,%3d px %d nx %d - px2: %3d,%3d px %d nx %d : isOK %5s edge %5s\n",
              bufCnt, px1->r, px1->c, px1->px, px1->nx, px2->r, px2->c, px2->px, px2->nx,
              bool2str(isOK), bool2str(edge));
      // debug end
      if (isOK and not edge)
      { // is it used before
        usedP = usedNext;
        for (j = 1; j < usedCnt; j++)
        { // test unusable points
          usedP--;
          if ((usedP->y == px2->r) and (usedP->x == px2->c))
          { // seen before - may be finished
            isOK = false;
            break;
          }
        }
        // debug
        if ((not isOK) and (flog != NULL))
          fprintf(flog, "unusable (num %d from top)\n", j);
        // debug end
      }
      if (isOK and not edge)
        break;
      d = (d + 1) % 4;
      isOK = false;
    }
    if (isOK and not edge)
    { // is it seen before?
      px3 = px1;
      for (j = bufCnt; j > 2; j--)
      { // test all previous values (except the first)
        px3--;
        if ((px3->r == px2->r) and (px3->c == px2->c))
        { // seen before on main path
          isOK = false;
          break;
        }
      }
      if (not isOK)
      { // add loop to used-and-found-bad-list
        // debug
        if (flog != NULL)
          fprintf(flog, "loop removed %d elements (%d to %d)\n",
                bufCnt - j, bufCnt, j);
        // debug end
        for (i = j; i < bufCnt; i++)
        { // a
          if (usedCnt < MAX_USED)
          {
            usedNext->x = px3->c;
            usedNext->y = px3->r;
            usedNext++;
            usedCnt++;
            px3++;
          }
          else
            // overflow - will exit in not-finished state
            break;
        }
        // tesminate outline at start of loop
        bufCnt = j;
      }
    }
    if (isOK)
    { // continue to next element
      //
      // debug
      imgC->setPix(px2->r, px2->c, rgb);
      // debug end
      //
      // add px2 to list
      bufCnt++;
      // test for finished
      finished = (px2->r == int(row)) and (px2->c == int(col));
    }
    else
    { // px1 leads nowhere, so
      // go back until one can
      // be found.
      bufCnt--;
      // add tested and unusable points to used buffer
      if (usedCnt < MAX_USED)
      {
        usedNext->x = px1->c;
        usedNext->y = px1->r;
        usedNext++;
        usedCnt++;
      }
      else
        // overflow error - exit in not finished state
        break;
      if (flog != NULL)
        fprintf(flog, "back\n");
      // finish if last element
      finished = (bufCnt <= 1);
    }
    //
    loops++;
    if (loops > 120000)
      break;
  }
  // return result value
  *outlineBufferCnt = bufCnt;
  //
  // debug
  printf("Finished %s in %d loops with %d in used list\n",
      bool2str(finished), loops, usedCnt);
  if (flog != NULL)
  {
    fprintf(flog, "Finished %s in %d loops with %d in used list\n",
      bool2str(finished), loops, usedCnt);
    fclose(flog);
  }
  // debug end
  // return result
  return finished;
}
*/
//////////////////////////////////////////////////////////

bool UImageAna::imageContour2(UImage * img, UImage * imgC,
                             unsigned int row, unsigned int col,
                             UPixelLinked * outlineBuffer,
                             int * outlineBufferCnt,
                             double diff, double diffa)
{ // find contour of all pixels in img
  // that are close to seed-point
  // write desition value in imgb and points in buffer.
  // init point may be away from bottom.
  bool res = (img != NULL);
  UPixelLinked * px1 = outlineBuffer;
  UPixelLinked * px2;
  UPixelLinked * px3;
  int bufCnt = 0;
  UPixel * pix1;
  UPixel * pix2;
  UPixel ref;
  bool finished = false;
  int d, j, i;
  unsigned int w, h;
  bool edge = false;
  bool isOK;
  UPixel rgb(255,0,0);
  int loops = 0;
  const int MAX_USED = 5000;
  CvPoint used[MAX_USED];
  CvPoint * usedNext = used;
  CvPoint * usedP;
  int usedCnt = 0;
  int refPixCnt = img->height() - row;
  // debug
  FILE * flog = NULL;
  flog = fopen("/home/chr/chr/images/imageContour2.log", "w");
  // debug end
  //

  if (res)
  {
    w = img->width();
    h = img->height();
    res = ((row < h) and (col < w));
  }
  if (res)
  {
    for (i = 0; i < refPixCnt; i++)
    { // add pixels from border to seed-point
      // (all usable as reference point)
      px1->r = h - i - 1;
      px1->c = col;
      px1->px = 1; // direction of source
      px1->nx = 1; // start cv from this direction
      px1++;
      bufCnt++; // first element is reference
    }
    // back to last pixel
    px1--;
  }
  while (res and not finished)
  { // overflow test
    if (bufCnt >= *outlineBufferCnt)
      break;
    // get refernce point
    px1 = &outlineBuffer[bufCnt-1];
    pix1 = img->getPixRef(px1->r, px1->c);
    // the startline is all usable as absolute ref-colour
    if ((bufCnt <= refPixCnt) and ((int)col == px1->c))
      ref = *pix1;
    px2 = px1;
    px2++;
    //px2->px = 0;
    // get the next untested direction
    d = (px1->nx + 1) % 4;
    // may not go back into seed column
    if ((px1->c == int(col) - 1) and
        (px1->r >= int(row)) and (d < 1))
      d = 1;
    isOK = false;
    while (d != px1->px)
    {
      px1->nx = d;
      isOK = px2->getNext(px1, d, h, w);
      if (isOK)
      {
        pix2 = img->getPixRef(px2->r, px2->c);
        // edge detect
        edge = absi(pix1->p1 - pix2->p1) > diff;
        if (not edge)
          edge = absi(pix1->p2 - pix2->p2) > diff;
        if (not edge)
          edge = absi(pix1->p3 - pix2->p3) > diff;
        // test also absolute from reference
        if (not edge)
          edge = absi(pix1->p1 - ref.p1) > diffa;
        if (not edge)
          edge = absi(pix1->p2 - ref.p2) > diffa;
        if (not edge)
          edge = absi(pix1->p3 - ref.p3) > diffa;
      }
      // debug
      if (flog != NULL)
        fprintf(flog, "#%3d px1: %3d,%3d px %d nx %d - px2: %3d,%3d px %d nx %d : isOK %5s edge %5s\n",
              bufCnt, px1->r, px1->c, px1->px, px1->nx, px2->r, px2->c, px2->px, px2->nx,
              bool2str(isOK), bool2str(edge));
      // debug end
      if (isOK and not edge)
      { // is it used before
        usedP = usedNext;
        for (j = 1; j < usedCnt; j++)
        { // test unusable points
          usedP--;
          if ((usedP->y == px2->r) and (usedP->x == px2->c))
          { // seen before - may be finished
            isOK = false;
            break;
          }
        }
        // debug
        if ((not isOK) and (flog != NULL))
          fprintf(flog, "unusable (num %d from top)\n", j);
        // debug end
      }
      if (isOK and not edge)
        break;
      d = (d + 1) % 4;
      isOK = false;
    }
    if (isOK and not edge)
    { // is it seen before?
      px3 = px1;
      for (j = bufCnt; j > 2; j--)
      { // test all previous values (except the first)
        px3--;
        if ((px3->r == px2->r) and (px3->c == px2->c))
        { // seen before on main path
          // test for finished at or below start point
          if ((px2->r >= int(row)) and (px2->c == int(col)))
          { // found a match in the seed-line - finished
            for (i = 0; i < j; i++)
              *px3-- = *px2;
            finished = true;
            break;
          }
          else
          { // found a loop - mark for deletion
            isOK = false;
            break;
          }
        }
      }
      if (finished)
        break;
      if (not isOK)
      { // add loop to used-and-found-bad-list
        // debug
        if (flog != NULL)
          fprintf(flog, "loop removed %d elements (%d to %d)\n",
                bufCnt - j, bufCnt, j);
        // debug end
        for (i = j; i < bufCnt; i++)
        { // a
          if (usedCnt < MAX_USED)
          {
            usedNext->x = px3->c;
            usedNext->y = px3->r;
            usedNext++;
            usedCnt++;
            px3++;
          }
          else
            // overflow - will exit in not-finished state
            break;
        }
        // tesminate outline at start of loop
        bufCnt = j;
      }
    }
    if (isOK)
    { // continue to next element
      //
      // debug
      imgC->setPix(px2->r, px2->c, rgb);
      // debug end
      //
      // add px2 to list
      bufCnt++;
      // test for finished at or below start point
      finished = (px2->r >= int(row)) and (px2->c == int(col));
    }
    else
    { // px1 leads nowhere, so
      // go back until one can
      // be found.
      bufCnt--;
      // add tested and unusable points to used buffer
      if (usedCnt < MAX_USED)
      {
        usedNext->x = px1->c;
        usedNext->y = px1->r;
        usedNext++;
        usedCnt++;
      }
      else
        // overflow error - exit in not finished state
        break;
      if (flog != NULL)
        fprintf(flog, "back\n");
      // finish if last element
      finished = (bufCnt <= 1);
    }
    //
    loops++;
    if (loops > 120000)
      break;
  }
  // return result value
  *outlineBufferCnt = bufCnt;
  //
  // debug
  printf("Finished %s in %d loops with %d in used list\n",
      bool2str(finished), loops, usedCnt);
  if (flog != NULL)
  {
    fprintf(flog, "Finished %s in %d loops with %d in used list\n",
      bool2str(finished), loops, usedCnt);
    fclose(flog);
  }
  // debug end
  // return result
  return finished;
}

//////////////////////////////////////////////////////////

bool UImageAna::imageContourMaha(UImage * img, UImage * imgC,
                              unsigned int row, unsigned int col,
                              UPixelLinked * outlineBuffer,
                              int * outlineBufferCnt,
                              UMatrix4 * averageSeed, UMatrix4 * covarianceInv,
                              double diff, double diffa)
{ // find contour of all pixels in img
  // that are close to seed-point
  // write desition value in imgb and points in buffer.
  // init point may be away from bottom.
  bool res = (img != NULL);
  UPixelLinked * px1 = outlineBuffer;
  UPixelLinked * px2;
  UPixelLinked * px3;
  int bufCnt = 0;
  UPixel * pix1;
  UPixel * pix2;
  UPixel ref;
  bool finished = false;
  int d, j, i;
  unsigned int w = 0, h = 0;
  bool edge = false;
  bool isOK;
  UPixel rgb(255,0,0);
  int loops = 0;
  const int MAX_USED = 5000;
  CvPoint used[MAX_USED];
  CvPoint * usedNext = used;
  CvPoint * usedP;
  int usedCnt = 0;
  int refPixCnt = img->height() - row;
  UMatrix4 pixVec(1,3);
  double dmaha1, dmaha2, sumPix, sumE;
  // debug
  FILE * flog = NULL;
  flog = fopen("/home/chr/chr/images/imageContour2.log", "w");
  // debug end
  //

  if (res)
  {
    w = img->width();
    h = img->height();
    res = ((row < h) and (col < w));
  }
  if (res)
  {
    for (i = 0; i < refPixCnt; i++)
    { // add pixels from border to seed-point
      // (all usable as reference point)
      px1->r = h - i - 1;
      px1->c = col;
      px1->px = 1; // direction of source
      px1->nx = 1; // start cv from this direction
      px1++;
      bufCnt++; // first element is reference
    }
    // back to last pixel
    px1--;
  }
  px1 = &outlineBuffer[bufCnt-1];
  pix1 = img->getPixRef(px1->r, px1->c);
  sumE = double(pix1->p1 + pix1->p2 + pix1->p3);
  while (res and not finished)
  { // overflow test
    if (bufCnt >= *outlineBufferCnt)
      break;
    // get refernce point
    px1 = &outlineBuffer[bufCnt-1];
    pix1 = img->getPixRef(px1->r, px1->c);
    // the startline is all usable as absolute ref-colour
    if ((bufCnt <= refPixCnt) and ((int)col == px1->c))
      ref = *pix1;
    px2 = px1;
    px2++;
    //px2->px = 0;
    // get the next untested direction
    d = (px1->nx + 1) % 4;
    // may not go back into seed column
    if ((px1->c == int(col) - 1) and
         (px1->r >= int(row)) and (d < 1))
      d = 1;
    isOK = false;
    while (d != px1->px)
    {
      px1->nx = d;
      isOK = px2->getNext(px1, d, h, w);
      if (isOK)
      {
        pix2 = img->getPixRef(px2->r, px2->c);
        dmaha1 = getMahalonobisDist(pix2, averageSeed, covarianceInv);
        // edge detect absolute distance
        edge = (dmaha1 > diffa);
        if (not edge)
        {
          sumPix = double(pix2->p1 + pix2->p2 + pix2->p3);
          pixVec = *averageSeed * (sumPix/sumE);
          dmaha2 = getMahalonobisDist(pix2, &pixVec, covarianceInv);
          edge = (dmaha2 > diff);
        }
      }
      if (isOK and not edge)
      { // is it used before
        usedP = usedNext;
        for (j = 1; j < usedCnt; j++)
        { // test unusable points
          usedP--;
          if ((usedP->y == px2->r) and (usedP->x == px2->c))
          { // seen before - may be finished
            isOK = false;
            break;
          }
        }
      }
      if (isOK and not edge)
        break;
      d = (d + 1) % 4;
      isOK = false;
    }
    if (isOK and not edge)
    { // is it seen before?
      px3 = px1;
      for (j = bufCnt; j > 2; j--)
      { // test all previous values (except the first)
        px3--;
        if ((px3->r == px2->r) and (px3->c == px2->c))
        { // seen before on main path
          // test for finished at or below start point
          if ((px2->r >= int(row)) and (px2->c == int(col)))
          { // found a match in the seed-line - finished
            for (i = 0; i < j; i++)
              *px3-- = *px2;
            finished = true;
            break;
          }
          else
          { // found a loop - mark for deletion
            isOK = false;
            break;
          }
        }
      }
      if (finished)
        break;
      if (not isOK)
      { // add loop to used-and-found-bad-list
        // debug
        if (flog != NULL)
          fprintf(flog, "loop removed %d elements (%d to %d)\n",
                  bufCnt - j, bufCnt, j);
        // debug end
        for (i = j; i < bufCnt; i++)
        { // a
          if (usedCnt < MAX_USED)
          {
            usedNext->x = px3->c;
            usedNext->y = px3->r;
            usedNext++;
            usedCnt++;
            px3++;
          }
          else
            // overflow - will exit in not-finished state
            break;
        }
        // tesminate outline at start of loop
        bufCnt = j;
      }
    }
    if (isOK)
    { // continue to next element
      //
      // debug
      imgC->setPix(px2->r, px2->c, rgb);
      // debug end
      //
      // add px2 to list
      bufCnt++;
      // test for finished at or below start point
      finished = (px2->r >= int(row)) and (px2->c == int(col));
    }
    else
    { // px1 leads nowhere, so
      // go back until one can
      // be found.
      bufCnt--;
      // add tested and unusable points to used buffer
      if (usedCnt < MAX_USED)
      {
        usedNext->x = px1->c;
        usedNext->y = px1->r;
        usedNext++;
        usedCnt++;
      }
      else
      { // overflow error - exit in not finished state
        fprintf(flog, "Overflow error 1\n");
        fprintf(stderr, "UImageAna::imageContourMaha: Overflow error 1\n");
        break;
      }
      if (flog != NULL)
        fprintf(flog, "back\n");
      // finish if last element
      finished = (bufCnt <= 1);
    }
    //
    loops++;
    if (loops > 120000)
    {
      fprintf(flog, "Overflow loop error\n");
      fprintf(stderr, "UImageAna::imageContourMaha: Overflow loop error\n");
      break;
    }
  }
  // return result value
  *outlineBufferCnt = bufCnt;
  //
  // debug
  printf("Finished %s in %d loops with %d in used list\n",
         bool2str(finished), loops, usedCnt);
  if (flog != NULL)
  {
    fprintf(flog, "Finished %s in %d loops with %d in used list\n",
            bool2str(finished), loops, usedCnt);
    fclose(flog);
  }
  // debug end
  // return result
  return finished;
}

///////////////////////////////////////////////////////////////////

double UImageAna::getRoadProbability(int r, int c,
                                 UImage * img, UImage * imgC,
                                 //UPixel * pixE, UPixel * pix1,
                                 UMatrix4 * vECroma, 
                                 UMatrix4 * averageCroma, UMatrix4 * covarCromaInv,
//                                 UMatrix4 * averageRgb, UMatrix4 * covarRgbInv,
                                 double ballance,   // floating criteria
                                 double averageY, bool useShadow)  // absolute criteria
{
  double pro2, pro3;
  double dmaha2;
  double diffH2;
  UPixel * pix2;
  UPixel * p1;
  UPixel * p2;
  UPixel * p3;
  int dr, dc, dd, s, er, ec;
  int s1 = 0, s2 = 0, s3 = 0, s4 = 0, i;
  double sum;
//  double r1, r2, re;
//  double sumPix, crr;
  double result;
  //double h = double(img->height());
  //
  pix2 = img->getPixRef(r, c);
  //dmaha1 = getMahalonobisDistCroma(pix2, averageSeed, covarianceInv);
  //dmaha1 = getMahalonobisDist(pix2, averageRgb, covarRgbInv);
  // edge detect absolute distance
  //dmaha1 = 0.0;
  //dmaha2 = 0.0;
  //diffH1 = 0.0;
  //diffH2 = 0.0;
  //
  //sumPix = double(pix2->p1 + pix2->p2 + pix2->p3);
  //crr = double(pix2->p3) / sumPix;
  // offset average to actual red-level to
  // compensate to colour temperature
  //vECroma->setRC(0, 0, crr * 0.7 + averageCroma->get(0,0) * 0.3);
  dmaha2 = getMahalonobisDistCroma(pix2, vECroma, covarCromaInv, averageY, useShadow);
  diffH2 = 0.01 * (0.15 + 0.85 * 0.5); //(double(r)/h));
  pro2 = 1.0/(1.0 + dmaha2 * diffH2);
  // get also RGB distance
//  dmaha3 = getMahalonobisDist(pix2, averageRgb, covarRgbInv);
  // tighten criteria as going up in image
//  diffH3 = diff * 0.4; //(0.75 + 0.25 *(double(px2->r)/double(h)));
//  pro3 = 1.0; //dmaha3 * diffH3;
  //
  // test for intensity change in either channel
/*  diffH1 = diffa * (0.35 + 0.65 *(double(r)/h));
  dmaha1 = pix2->colorDist(pix1);
  pro1 = 1.0/(1.0 + dmaha1 * diffH1);
  pro1 = 1.0;*/
  //
  // test for intensity change in either channel
/*  diffH4 = diffa;// * (0.85 + 0.15 *(double(px2->r)/double(h)));
  //diffH4 *= 1.3;
  dmaha4 = pix2->colorDist(pixE);
  pro4 = 1.0/(1.0 + dmaha4 * diffH4);*/
  //
  if (r < (int)img->height()/3)
    s = 1;
  else if (r < (int)(img->height()*2)/3)
    s = 2;
  else
    s = 3;
  er = maxi(0, mini(img->height() - 3 * s, r - s));
  ec = maxi(0, mini(img->width() - 3 * s, c - s));
  p1 = img->getPixRef(er, ec);
  p2 = img->getPixRef(er + s, ec);
  p3 = img->getPixRef(er + 2 * s, ec);
  sum = 0;
  for (i = 0; i < s; i++)
  { // sum left column
    s1 = p1->getSum() + 2 * p2->getSum() + p3->getSum();
    // sum right column
    s2 = p1[2*s].getSum() + 2 * p2[2*s].getSum() + p3[2*s].getSum();
    // sum top column
    s3 = p1[0].getSum() + 2 * p1[s].getSum() + p1[2*s].getSum();
    // sum bottom column
    s4 = p3[0].getSum() + 2 * p3[s].getSum() + p3[2*s].getSum();
    sum += 8.0;
    p1++;
    p2++;
    p3++;
  }
  // divide edge
/*  r1 = double(s1) / double (s2);
  r2 = double(s3) / double (s4);
  if (r1 > r2)
    re = maxd(r1, 1.0/r2);
  else
    re = maxd(1.0/r1, r2);
  pro1 = 1.0/(1.0 + (re - 1.0) * 10.0);*/
  // derivative (*2)
  dc = s1 - s2;
  dr = s3 - s4;
  // block sum of derivatives
  dd = abs(dc) + abs(dr);
  pro3 = 1.0/(1.0 + dd/(sum * 15.0));
  //
  //result = (pro1 * pro2 * pro3 * pro4);
  result = pro3 * ballance + pro2 * (1.0 - ballance);
  // debug
/*  pix2 = imgC->getPixRef(r, c);
  pix2->p1 = maxi(0, 255 - int(pro2 * 255));
  pix2->p2 = maxi(0, 255 - int(pro3 * 255));
  pix2->p3 = maxi(0, 255 - int(result * 128));*/
  // debug end
  return result;
}

///////////////////////////////////////////////////////

bool UImageAna::imageContourMahaCroma(UImage * img, UImage * imgC,
                                 unsigned int row, unsigned int col,
                                 int x1, int y1, int x2, int y2, int maxH,
                                 UPixelLinked * outlineBuffer,
                                 int * outlineBufferCnt,
                                 UMatrix4 * averageCroma, UMatrix4 * covarCromaInv,
                                 UMatrix4 * averageRgb, UMatrix4 * covarRgbInv,
                                 double ballance,   // ballance [0 - 1]
                                 double limit)  // absolute limit for road edge
{ // find contour of all pixels in img
  // that are close to seed-point
  // write desition value in imgb and points in buffer.
  // init point may be away from bottom.
  bool res = (img != NULL);
  UPixelLinked * px1 = outlineBuffer;
  UPixelLinked * px2;
  UPixelLinked * px3;
  int bufCnt = 0;
  UPixel * pix1;
  UPixel * pix2;
  UPixel ref;
  UPixel pixE;
  bool finished = false;
  int d, j, i;
  unsigned int w = 0;
  bool edge = false;
//  double edge1, edge2, edge3, edge4; // probability
  bool isOK;
  UPixel rgb(255,0,0);
/*  UPixel rgbe1(0,255,255); // yellow
  UPixel rgbe2(255,0,255); // magenta
  UPixel rgbe3(0,0,255);   // red
  UPixel rgbe4(255,0,0);   // blue*/
  int loops = 0;
  const int MAX_USED = 10000;
  CvPoint used[MAX_USED];
  CvPoint * usedNext = used;
  CvPoint * usedP;
  int usedCnt = 0;
  int refPixCnt = 0;
  UMatrix4 vECroma(1,2);
  UMatrix4 vERgb(1,3);
//  double dmaha1, dmaha2, dmaha3, dmaha4, sumPix;
//  double dmaE1, dmaE2, dmaE3, dmaE4;
//  double crr;
//  double diffH1, diffH2, diffH3, diffH4;
  //const int MSL = 300;
  //char s[MSL];
  //int edgeCroma = 0, edgeRGB = 0, edgeIntens = 0;
//  double * pE;
  int edgeK = 8; // kalman edge gain (in %)
  double proRoad;
  double averageY;
  // debug
  FILE * flog = NULL;
  //flog = fopen("/home/chr/chr/images/imageContour2.log", "w");
  //FILE * elog = NULL; // edge log
  //elog = fopen("/home/chr/chr/images/imageContourEdge.log", "w");
  //snprintf(s, MSL, "%s/%s.txt", imagePath, img->name);
  //elog = fopen(s, "w");
  // debug end
  if (res)
  {
    w = img->width();
    res = ((int(row) < maxH) and (col < w));
    refPixCnt = maxH - row;
  }
  if (res)
  {
    for (i = 0; i < refPixCnt; i++)
    { // add pixels from border to seed-point
      // (all usable as reference point)
      px1->r = maxH - i - 1;
      px1->c = col;
      px1->px = 1; // direction of source
      px1->nx = 1; // start cv from this direction
      px1++;
      bufCnt++; // first element is reference
    }
    // back to last pixel
    px1--;
  }
  px1 = &outlineBuffer[bufCnt-1];
  pix1 = img->getPixRef(px1->r, px1->c);
  // seed average pixel
  pixE = *pix1;
  vECroma = *averageCroma;
  vERgb = *averageRgb;
  averageY = averageRgb->get(0,0) + averageRgb->get(0,1) + averageRgb->get(0,2);
  while (res and not finished)
  { // overflow test
    if (bufCnt >= *outlineBufferCnt)
      break;
    // get refernce point
    px1 = &outlineBuffer[bufCnt-1];
    pix1 = img->getPixRef(px1->r, px1->c);
    // the startline is all usable as absolute ref-colour
    if ((bufCnt <= refPixCnt) and ((int)col == px1->c))
      ref = *pix1;
    px2 = px1;
    px2++;
    //px2->px = 0;
    // get the next untested direction
    d = (px1->nx + 1) % 4;
    // may not go back into seed column
    if ((px1->c == int(col) - 1) and
         (px1->r >= int(row)) and (d < 1))
      d = 1;
    isOK = false;
/*    dmaE1 = 0.0;
    dmaE2 = 0.0;
    dmaE3 = 0.0;*/
    while (d != px1->px)
    {
      px1->nx = d;
      isOK = px2->getNext(px1, d, maxH, w);
      if (isOK)
      {
        pix2 = img->getPixRef(px2->r, px2->c);
        // edge detect absolute distance
        proRoad = getRoadProbability(px2->r, px2->c, 
                                  img, imgC,
                                  //&pixE, pix1, 
                                  &vECroma, 
                                  averageCroma, covarCromaInv, 
                                  //averageRgb, covarRgbInv, 
                                  ballance,
                                  averageY, true); //, diffa);
        edge = proRoad < limit;
        if (edge)
        { // test for beeing inside seed-area
          if ((px2->r >= y1) and (px2->c >= x1) and
              (px2->r <= y2) and (px2->c <= x2))
            // undo edge decition
            edge = false; 
        }
        if (not edge)
        { // test for seed line hit from left (acts as a hard edge)
          if ((px2->r >= int(row)) and (px2->c == int(col)) and (px1->c < int(col)))
            edge = true;
        }
      }
      if (isOK and not edge)
      { // is it used before
        usedP = usedNext;
        for (j = 1; j < usedCnt; j++)
        { // test unusable points
          usedP--;
          if ((usedP->y == px2->r) and (usedP->x == px2->c))
          { // seen before - may be finished
            isOK = false;
            break;
          }
        }
      }
      if (isOK and not edge)
      { // test for seed line hit from left
        if ((px2->r >= int(row)) and (px2->c == int(col)) and (px1->c < int(col)))
          edge = true;
      }
      if (isOK and not edge)
      { // next pixel is found
        pixE.p1 = pixE.p1 + (edgeK * (pix2->p1 - pixE.p1)) / 100;
        pixE.p2 = pixE.p2 + (edgeK * (pix2->p2 - pixE.p2)) / 100;
        pixE.p3 = pixE.p3 + (edgeK * (pix2->p3 - pixE.p3)) / 100;
        if (loops == 8050)
          printf("Loop 8050 -- too much??\n");
        break;
      }
      d = (d + 1) % 4;
      isOK = false;
    }
    if (isOK and not edge)
    { // is it seen before?
      px3 = px1;
      px3 = &outlineBuffer[bufCnt-1];
      for (j = bufCnt; j > 2; j--)
      { // test all previous values (except the first)
        px3--;
        if ((px3->r == px2->r) and (px3->c == px2->c))
        { // seen before on main path
          // test for finished at or below start point if arriced from right side
          if ((px2->r >= int(row)) and (px2->c == int(col)))
          { // found a match in the seed-line - finished
            for (i = 0; i < j; i++)
            {
              outlineBuffer[i] = *px2;
            }
            finished = true;
            break;
          }
          else
          { // found a loop - mark for deletion
            isOK = false;
            break;
          }
        }
      }
      if (finished)
        break;
      if (not isOK)
      { // add loop to used-and-found-bad-list
        // debug
        if (flog != NULL)
          fprintf(flog, "loop removed %d elements (%d to %d)\n",
                  bufCnt - j, bufCnt, j);
        // debug end
        for (i = j; i < bufCnt; i++)
        { // a
          if (usedCnt < MAX_USED)
          {
            usedNext->x = px3->c;
            usedNext->y = px3->r;
            usedNext++;
            usedCnt++;
            px3++;
          }
          else
            // overflow - will exit in not-finished state
            break;
        }
        // tesminate outline at start of loop
        bufCnt = j;
      }
    }
    if (isOK)
    { // continue to next element
      //
      // debug
      //if (imgC != NULL)
      //  imgC->setPix(px2->r, px2->c, rgb);
      // debug end
      //
      // add px2 to list
      bufCnt++;
      // test for finished at or below start point
      finished = (px2->r >= int(row)) and (px2->c == int(col));
    }
    else
    { // px1 leads nowhere, so
      // go back until one can
      // be found.
      bufCnt--;
      // add tested and unusable points to used buffer
      if (usedCnt < MAX_USED)
      {
        usedNext->x = px1->c;
        usedNext->y = px1->r;
        usedNext++;
        usedCnt++;
      }
      else
      { // overflow error - exit in not finished state
        if (flog != NULL)
          fprintf(flog, "Overflow error 1\n");
        fprintf(stderr, "UImageAna::imageContourMaha: Overflow error 1\n");
        break;
      }
      if (flog != NULL)
        fprintf(flog, "back\n");
      // finish if last element
      finished = (bufCnt <= 1);
    }
    //
    loops++;
    if (loops > 60000)
    {
      fprintf(flog, "Overflow loop error\n");
      fprintf(stderr, "UImageAna::imageContourMaha: Overflow loop error\n");
      break;
    }
  }
  // return result value
  *outlineBufferCnt = bufCnt;
  //
  // debug
/*  printf("Finished %s in %d loops with %d in used list. \n"
         "Croma edges=%d. RGB-edges=%d. Intens edges=%d\n",
         bool2str(finished), loops, usedCnt,
         edgeCroma, edgeRGB, edgeIntens);*/
  if (flog != NULL)
  {
    fprintf(flog, "Finished %s in %d loops with %d in used list\n",
            bool2str(finished), loops, usedCnt);
    fclose(flog);
  }
/*  if (elog != NULL)
    fclose(elog);*/
  // debug end
  // return result
  return finished;
}

//////////////////////////////////////////////////////////

void UImageAna::findVertex(const UPixelLinked * end1,
                           const UPixelLinked * end2,
                           const int count,
                           const int startNum,
                           int ** vertexList,
                           int * vertexListCnt,
                           const int listLength,
                           const float toll,
                           int topLine)
{
  U2Dline lin; // from-to line
  int i, j = -1, step;
  int wx, wy; // vector from end1 to testpoint;
  int ux, uy; // vector from end1 to end2
  int cu; // length from end1 to end 2 (squared)
  int cw; // dot product from end1 to testpoint
  const UPixelLinked * next = end1;
  float d2, maxDist = 0; // distance squared and max
  float bx, by; // point on line from end1 to end2
  float redToll;
  // step uses fact that each point is
  // separated by one pixel only
  if (topLine > 0)
  { // tollerance decreases with height in image
    i = mini(end1->r, end2->r) - topLine;
    redToll = toll * sqr(float(i) / float(240 - topLine));
    if (i <= 0)
      redToll = 1.0;
    else if (redToll < 1.0)
      redToll = 1.0;
  }
  else
    redToll = toll;
  step = int(redToll) + 1;
  // calculate start values for line between
  // end 1 and end 2
  ux = end2->c - end1->c;
  uy = end2->r - end1->r;
  cu = ux * ux + uy * uy; // length squared
  i = step;
  while (i < count)
  {
    next = &end1[i];
    wx = next->c - end1->c;
    wy = next->r - end1->r;
    cw = wx * ux + wy * uy;
    // calculate distance from line
    if (cw <= 0)
      // closer to end1 and not on line to end2
      d2 = float(wx * wx + wy * wy);
    else if (cw >= cu)
      // closer to end2 and not on line to end2
      d2 = float(sqri(end2->c - next->c) + sqri(end2->r - next->r));
    else
    { // on line from end1 to end 2
      // calculate closest point on line from dot product cw
      d2 = float(cw) / float(cu);  // use d2 for distance relation
      bx = float(end1->c) + d2 * float(ux);
      by = float(end1->r) + d2 * float(uy);
      d2 = sqr(bx - float(next->c)) + sqr(by - float(next->r));
    }
    if (d2 > maxDist)
    {
      maxDist = d2;
      j = i;
    }
    i = i + step;
  }
  // calculate tollerance from new vertex
  if (j > 0)
  {
    next = &end1[j];
    if (topLine > 0)
    { // reduce tollerance from line number of (potentially) new vertex
      redToll = toll * sqr(float(next->r - topLine) / float(240 - topLine));
      if (next->r <= topLine)
        redToll = 1.0; // use a minimal one above limit
      else if (redToll < 1.0)
        redToll = 1.0;
    }
  }
  if ((maxDist > redToll) and (*vertexListCnt < listLength) and (j > 0))
  { // add new vertex to list
    **vertexList = j + startNum;
    (*vertexList)++; // point to the next index number to use
    (*vertexListCnt)++;
    // find also in the new sub-intervals
    findVertex(end1,  &end1[j], j, startNum, vertexList, vertexListCnt, listLength, toll, topLine);
    findVertex(&end1[j], end2, count - j, startNum + j, vertexList, vertexListCnt, listLength, toll, topLine);
  }
}

////////////////////////////////////////////////////////////////

bool UImageAna::polygonReduce(
                     UPixelLinked * outlineBuffer,
                     int * outlineBufferCnt,
                     double tollerance, // at bottom
                     int topLine)       // "zero" tollerance at this line
{
  int vertexListBuff[MAX_POLYGON_POINTS];
  int * vertexList = vertexListBuff;
  int vertexListCnt = 0;
  int i, j, n;
  //
  if (*outlineBufferCnt > 1)
  {
    findVertex(outlineBuffer,
              &outlineBuffer[*outlineBufferCnt - 2],
              *outlineBufferCnt - 1,
              0,
              &vertexList,
              &vertexListCnt,
              MAX_POLYGON_POINTS,
              sqr(tollerance), topLine);
    // sort index list
    /* // debug
    printf("Idx:");
    for (i = 0; i < vertexListCnt; i++)
      printf(" %3d", vertexListBuff[i]);
    printf("\n"); */
    //printf("Vertex list count after reduce %d\n", vertexListCnt);
    // debug end

    for (i = 1; i < vertexListCnt; i++)
    {  // boble sort, as they is mostly sorted already
      for (j = i; j > 0; j--)
      {
        if (vertexListBuff[j] < vertexListBuff[j-1])
        {
          n = vertexListBuff[j];
          vertexListBuff[j] = vertexListBuff[j-1];
          vertexListBuff[j-1] = n;
        }
        else
          // the rest is OK
          break;
      }
    }
    /* // debug
    printf("Idx:");
    for (i = 0; i < vertexListCnt; i++)
      printf(" %3d", vertexListBuff[i]);
    printf("\n");
    // debug end */
    // leave first point
    for (i = 0; i < vertexListCnt; i++)
    { // put the result into original array
      outlineBuffer[i + 1] = outlineBuffer[vertexListBuff[i]];
    }
    // add also last point (probably same as the first)
    outlineBuffer[vertexListCnt + 1] = outlineBuffer[*outlineBufferCnt-1];
    *outlineBufferCnt = vertexListCnt + 2;
  }
  //
  return (*outlineBufferCnt > 1);
}

///////////////////////////////////////////////////////////

bool UImageAna::polygonReduce2(
    UPixelLinked * outlineBuffer,
                     int * outlineBufferCnt,
                     double tollerance, // at bottom
                     int topLine)       // "zero" tollerance at this line
{
  int i, j, i2;
  int m = *outlineBufferCnt;
  UPixelLinked * p1 = outlineBuffer;
  UPixelLinked * p2;
  double a, a1;
  int dr, dc;
  int dLim = 4;
  double aLim = 10.0;
  //
  for (i = 0; i < m; i++)
  {
    p2 = p1;
    p2++;
    i2 = p2 - outlineBuffer;
    a = 0;
    for (j = 0; j < m-5; j++)
    { // fold back to zero if end is reached
      if (i2 >= m)
        p2 = outlineBuffer;
      // get rea of new triangle
      dr = p2->r - p1->r;
      dc = p2->c - p1->c;
      a1 = 0.5 * double(dr * dc);
      // sum to total area
      a += a1;
      if ((j > 1) and ((dr + dc) < mini(j, dLim)))
      { // close, so test area
        if (a < aLim)
          printf("Found appendix (%dr,%dc) to (%dr,%dc)\n", p1->r, p1->c, p2->r, p2->c);
      }
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////

double UImageAna::polygonArea(CvPoint * poly, int elements)
{
  double res;
  int a = 0;
  int i;
  CvPoint * v1 = &poly[elements-1]; // last point
  CvPoint * v2 = poly; // first point
  //
  for (i = 0; i < elements; i++)
  { // sum of polygons
    a += (v1->x * v2->y) - (v1->y * v2->x);
    v1 = v2;
    v2++;
  }
  res = double(a) / 2.0;
  return res;
}


//////////////////////////////////////////////

int UImageAna::findContourPoly(UImage * imgM, UImage * imgC,
                     bool * tooSmall, double lookingLeft, double pct1, double pct2)
{
  bool res;
  int seedX, seedY;
  double critDyn = pct1;
  double critAbs = pct2;
  float reductionToll = 4.5;
  int   reductionTollZero = 60;
  int i, n, r, c, xmin, xmax;
  UPixel red(0,0,255);
  UPixel green(0,255,0);
  UPixel * pix, *pix2;
  CvPoint * pt;
  bool * ob;
  int obCnt;
  UTime t1, t2;
  double sr, sg, sb;
  UMatrix4  mRGB(3, 3, 0.0);
  UMatrix4  mRGBV(3, 3, 0.0);
  UMatrix4  mRGBVI(3, 3, 0.0);
  UMatrix4  vRGBE(1, 3, 0.0);
  UMatrix4  vRGB1(1, 3, 0.0);
  UMatrix4  vRGB2(1, 3, 0.0);
  UImage * imgMA;

  //
  // allocate UPixelLinked buffer
  if (floorOutline == NULL)
    floorOutline = (UPixelLinked *)
           malloc(MAX_POLYGON_OUTLINE_POINTS * sizeof(UPixelLinked));
  floorOutlineCnt = MAX_POLYGON_OUTLINE_POINTS;
  //
  seedY = imgM->height() - 10;
  seedX = imgM->width()/2 +
            roundi(lookingLeft * double(imgM->width()/2));
  seedX = maxi(0, mini(imgM->width() - 1, seedX));
  xmin = maxi(0, seedX - 10);
  xmax = mini(imgM->width(), seedX + 10);
  // get limit min and max for each relevant pixel
  sr = 0.0;
  sg = 0.0;
  sb = 0.0;
  n = 0;
  for (r = seedY; r < int(img->height()); r++)
  {
    pix = img->getPixRef(r, xmin);
    for (c = xmin; c < xmax; c++)
    {
      n++;
      sr += double(pix->p3);
      sg += double(pix->p2);
      sb += double(pix->p1);
      vRGB1.setRow(0, pix->p3, pix->p2, pix->p1);
      vRGB2 = vRGB1;
      vRGB2.setSize(3,1);
      mRGB = mRGB + (vRGB2 * vRGB1);
      pix++;
    }
  }
  printf("Seed (%i,%i), xmin=%i xmax=%i (n=%i) (sum red = %f)\n",
     seedX, seedY, xmin, xmax, n, sr);
  pix--;
  vRGBE.setRow(0, sr/double(n), sg/double(n), sb/double(n));
  vRGB1 = vRGBE;
  vRGB2 = vRGB1;
  vRGB2.setSize(3,1);
  mRGBV = mRGB * (1.0/double(n)) - (vRGB2 * vRGB1);
  vRGBE.print("Average RGB");
  mRGBV.print("Covarians");
  mRGBVI = mRGBV.inversed();
  // test with center pixel
  //pix = img->getPixRef(seedY, seedX);
/*  pix = img->getPixRef(seedY, seedX);
  printf("pix c(%u,%u,%u) (at %d,%d)", pix->p3, pix->p2, pix->p1, seedX, seedY);*/
/*  vRGB1.setRow(0, pix->r, pix->g, pix->b);
  vRGB1 = vRGB1 - vRGBE;
  vRGB2 = vRGB1;
  vRGB2.setSize(3,1);
  mRGB = vRGB1 * mRGBVI * vRGB2;
  sr = mRGB.get(0,0);*/
//   sr = getMahalonobisDist(pix, &vRGBE, &mRGBVI);
//   printf("Center pixel has distance %f\n", sr);
//   pix = img->getPixRef(5, 5);
//   pix->print("pix 5,5");
//   sr = getMahalonobisDist(pix, &vRGBE, &mRGBVI);
//   printf("( 5x, 5y)    has distance %f\n", sr);

  imgMA = imgPool->getImage(6, true);
  imgMA->copy(img);
  imgMA->setNameAndExt("Mahalanobi", NULL);
  imgMA->imgTime.Now();
  img->setNameAndExt("org", NULL);
  for (r = 0; r < int(img->height()); r++)
  {
    pix = img->getLine(r);
    pix2 = imgMA->getLine(r);
    for (c = 0; c < int(img->width()); c++)
    {
      sr = getMahalonobisDist(pix, &vRGBE, &mRGBVI);
      pix2->p3 = mini(255,roundi(sr *3.0));
      pix2->p2 = mini(255,roundi(sr * 20.0));
      pix2->p1 = mini(255,255 - roundi(sr *5.0));
      pix++;
      pix2++;
    }
  }

  //
  // debug timing
  t1.Now();
  // debug end timing
  if (false)
    res = imageContour2(imgM, imgC,
               seedY, seedX,
               floorOutline, &floorOutlineCnt,
               critDyn, critAbs);
  else
    res = imageContourMaha(imgM, imgC,
                      seedY, seedX,
                      floorOutline, &floorOutlineCnt,
                      &vRGBE, &mRGBVI,
                      critDyn, critAbs);
  // debug timing
  t2.Now();
  printf("imageContour2 took %f secs\n", t2 - t1);
  // debug end timing
  // too small is less than 100 pixels in outline
  *tooSmall = (floorOutlineCnt < 100);
  //
  // debug
  if (false and res and (imgC != NULL))
  {
    // UPixelLinked * px = floorOutline;
    //for (n = 0; n < floorOutlineCnt; n++)
    //{
    //  imgC->setPix(px->r, px->c, red);
    //  px++;
    //}
  }
  // debug end
  //
  if (res)
  { // simplify
    // debug timing
    t1.Now();
    // debug end timing
    res = polygonReduce(floorOutline, &floorOutlineCnt,
                        reductionToll, reductionTollZero);
    // debug timing
    //t2.Now();
    //printf("polygonReduce took %f secs\n", t2 - t1);
    // debug end timing
  }
  if (res)
  { // concert to CvPoint array
    //
    if (floorArea == NULL)
    {
      floorArea = (CvPoint *) malloc(MAX_POLYGON_POINTS * sizeof(CvPoint));
      obst = (bool *) malloc(MAX_POLYGON_POINTS * sizeof(bool));
    }
    //
    pt = floorArea;
    floorAreaCnt = floorOutlineCnt;
    if (floorOutlineCnt > MAX_POLYGON_POINTS)
    { // overflow
      printf("Too many vertexes for CvPoint buffer %d > %d\n",
                         floorOutlineCnt, MAX_POLYGON_POINTS);
      floorAreaCnt = MAX_POLYGON_POINTS;
    }
    // convert to CvPoint and bool[]
    ob = obst;
    obCnt = 0;
    for (i = 0; i < floorAreaCnt; i++)
    { // move to CvPoint format
      pt->x = floorOutline[i].c;
      pt->y = floorOutline[i].r;
      // mark as obstacle position if not in image border area
      *ob = not ((pt->x < 13) or (pt->x > int(imgM->width()) - 14) or
                 (pt->y < 13) or (pt->y > int(imgM->height()) - 14));
      if (*ob)
        obCnt++;
      pt++;
      ob++;
    }
    // debug
    //printf("UImageAna::findContourPoly Found %d CvPoints with %d as obstacles\n",
    //     floorAreaCnt, obCnt);
    // debug end
  }
  // debug
  if (false and res and imgC != NULL)
  {
    pt = floorArea;
    //cvPolyLine(imgC->cvArr(), &pt, &floorAreaCnt, 1, 1, CV_RGB(0,255,0), 1, 8, 0 );
  }
  // debug end
  //
  return res;
}

//////////////////////////////////////////////

int UImageAna::findContourPolyCroma(UImage * imgM, UImage * imgC, UImage * imgCroma,
                               bool * tooSmall, double lookingLeft,
                               int x1, int y1, int x2, int y2, int maxH,
                               double ballance, double limit)
{
  bool res;
  int seedX, seedY;
/*  double critDyn = pct1;
  double critAbs = pct2;*/
  float reductionToll = 4.5;
  int   reductionTollZero = 1;
  int i, n, r, c, xmin, xmax, ymin, ymax;
  int h = 0, w = 0, cx, cy;
  UPixelLinked * px;
  UPixel red(0,0,255);
  UPixel green(0,255,0);
  UPixel * pix, *pix2;
  CvPoint * pt;
  bool * ob;
  int obCnt;
  UTime t1, t2;
  double sr = 0.0, sg = 0.0, sb = 0.0;
  UMatrix4  mCr(2, 2, 0.0);
  UMatrix4  mCrV(2, 2, 0.0);
  UMatrix4  mCrVI(2, 2, 0.0);
  UMatrix4  vCrE(1, 2, 0.0);
  UMatrix4  vCr1(1, 2, 0.0);
  UMatrix4  vCr2(1, 2, 0.0);
  //const int MSA = 3; // Max seed areas
  UMatrix4  mRgb(3, 3, 0.0);
  UMatrix4  mRgbV;
  UMatrix4  mRgbVI(3, 3, 0.0);
  UMatrix4  vRgbE;
  UMatrix4  vRgb1(1, 3, 0.0);
  UMatrix4  vRgb2(1, 3, 0.0);
  UImage * imgMA;
  double crr, crg, crrs, crgs, sum;
  double dcro, drgb;
  CvFont font;
  const int MSL = 100;
  char s1[MSL];
  char s2[MSL];
  char s3[MSL];
  int xw, yw;
//  double tr;
  char * p2;
  const double cromaVarMin = sqr(1.3/256.0); // 1.3 intensity LSB
  //
  // allocate UPixelLinked buffer (just once)
  if (floorOutline == NULL)
    floorOutline = (UPixelLinked *)
        malloc(MAX_POLYGON_OUTLINE_POINTS * sizeof(UPixelLinked));
  floorOutlineCnt = MAX_POLYGON_OUTLINE_POINTS;
  //
  w = imgM->width();
  h = imgM->height();
  // extract seed rectangle and seed point for polygon
  seedY = h - h/10;
  seedX = w/2 + roundi(lookingLeft * double(w/2));
  seedX = maxi(0, mini(w - 1, seedX));
  if (x1 >= 0 and x2 >= 0)
  {
    seedX = (x1 + x2) / 2;
    xmin = mini(mini(x1, x2), w - 1);
    xmax = mini(maxi(x1, x2), w - 1);
  }
  else
  {
    xmin = maxi(0, seedX - w / 10);
    xmax = mini(w - 1, seedX + w / 10);
  }
  if (y1 >= 0 and y2 >= 0)
  {
    ymin = mini(mini(y1, y2), h - 1);
    ymax = mini(maxi(y1, y2), h - 1);
    seedY = ymin;
  }
  else
  {
    ymin = seedY;
    ymax = h - 1;
  }
  // get limit min and max for each relevant pixel
  if (imgC != NULL)
  { // paint seed area
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);
    // paint time in top left corner
    if (strlen(imgM->name) < 25)
    {
      imgM->imgTime.GetDateString(s1, true);
      imgM->imgTime.GetTimeAsString(s2, true);
      snprintf(s3, MSL, "%s %s", s1, s2);
    }
    else
    {
      p2 = strrchr(imgM->name,'/');
      if (p2 != NULL)
      {
        p2++;
        snprintf(s3, MSL, "%s", p2);
        s3[12] = '\0'; // limit size
        strcpy(imgC->name, p2);
      }
      else
        strncpy(s3, imgM->name, MSL);
    }
    cvPutText(imgC->cvArr(), s3, cvPoint(10, 15), &font, CV_RGB(255,0,0));
    // paint seed area
    cvRectangle(imgC->cvArr(), cvPoint(xmin, ymin),
                cvPoint(xmax, ymax),
                CV_RGB(255,0,0), 1, 8, 0);
  }
  //
  // debug end - croma image
  crrs = 0.0;
  crgs = 0.0;
  xw = (xmax - xmin);
  yw = (ymax - ymin);
  n = 0;
//  m = 1;
//  for (m = 0; m < MSA; m++)
//  {
    vRgbE.init(1, 3, 0.0);
    mRgbV.init(3, 3, 0.0);
    for (r = ymin; r <= ymax; r++)
    {
      pix = imgM->getPixRef(r, xmin);
      for (c = 0; c <= xw; c++)
      {
        n++;
        // CROMA, RGB covariance and average
        // intensity total
        sum = double(pix->p1 + pix->p2 + pix->p3);
        // get red value
        crr = double(pix->p3);
        // compensate for shaddows value
        crr += (1.5 * 256.0 - sum) * SHADOW_COMPENSATION_FACTOR;
        // convert to cromaticity
        crr /= sum;
        crg = double(pix->p2) / sum;
        crrs += crr;
        crgs += crg;
        vCr1.setRow(0, crr, crg);
        vCr2 = vCr1;
        vCr2.setSize(2,1);
        mCr = mCr + (vCr2 * vCr1);
        // RGB covariance and average
        sr += double(pix->p3);
        sg += double(pix->p2);
        sb += double(pix->p1);
        vRgb1.setRow(0, pix->p3, pix->p2, pix->p1);
        vRgb2 = vRgb1;
        vRgb2.setSize(3,1);
        mRgb = mRgb + (vRgb2 * vRgb1);
        // debug - paint
        if (false and (imgCroma != NULL))
        {
          cx = maxi(0, mini(511, roundi(crr * w)));
          cy = maxi(0, mini(511, h - roundi(crg * h)));
  /*        cvCircle(imgCroma->cvArr(),
                  cvPoint(roundi(crg * w), h - roundi(crr * h)),
                  2, CV_RGB(pix-R, pix->g, pix->b), 1, 8, 0);*/
          imgCroma->setPix(cy, cx, UPixel::pixRGB(255,0,0));
        }
        // debug end - paint
        pix++;
      }
    }
    // RGB post calculation
    c = xw * yw;
    vRgbE.setRow(0, sr/double(c), sg/double(c), sb/double(c));
//  }
  // find best pair of averages
  
  
  
/*  @hertil
  
  
  sammenlign venstre og h�jre side med center og 
      brug kun midt og den ene af de ydre,
  udregn s� varians og middelv�rdi for de 2/3 der er tilbage og brug dem.
      
      */
      
      
  vRgb1 = vRgbE;
  vRgb2 = vRgb1;
  vRgb2.setSize(3,1);
  mRgbV = mRgb * (1.0/double(c)) - (vRgb2 * vRgb1);
  // debug
  //printf("Seed (%i,%i), xmin=%i xmax=%i ymin=%d ymax=%d (n=%i) (sum red = %f)\n",
  //       seedX, seedY, xmin, xmax, ymin, ymax, n, sr);
  // debug end
  pix--;
  // cromaticity post calculation
  vCrE.setRow(0, crrs/double(n), crgs/double(n), 0.0);
  vCr1 = vCrE;
  vCr2 = vCr1;
  vCr2.setSize(2,1);
  mCrV = mCr * (1.0/double(n)) - (vCr2 * vCr1);
  cromaTrace = mCrV.trace();
  //if (false and (tr < 5e-5))
  //  mCrV = mCrV * (1.0 + (5e-5/tr - 1.0)*0.5);
  // debug
  if (false)
  {
    vCr1.print("vCr1");
    mCr.print("mCr");
    mCrV.print("mCrV");
  }
  printf("Croma SD trace %g\n", sqrt(cromaTrace));
  if (cromaTrace < cromaVarMin)
  {
    mCrV.mult(cromaVarMin/cromaTrace);
    cromaTrace = mCrV.trace();
    printf("Croma SD New   %g\n", sqrt(cromaTrace));
  }
  // debug end
//  vCrE.print("Average CROMA");
//  mCrV.print("Covarians CROMA");
  mCrVI = mCrV.inversed();
  // RGB post calculation
  vRgbE.setRow(0, sr/double(n), sg/double(n), sb/double(n));
  vRgb1 = vRgbE;
  vRgb2 = vRgb1;
  vRgb2.setSize(3,1);
  mRgbV = mRgb * (1.0/double(n)) - (vRgb2 * vRgb1);
//  vRgbE.print("Average RGB");
//  mRgbV.print("Covarians RGB");
  mRgbVI = mRgbV.inversed();
  // test with center pixel
  //pix = img->getPixRef(seedY, seedX);
//  pix = img->getPixRef(seedY, seedX);
//  printf("pix c(%u,%u,%u) (at %d,%d)", pix->p3, pix->p2, pix->p1, seedX, seedY);
/*  vRGB1.setRow(0, pix->r, pix->g, pix->b);
  vRGB1 = vRGB1 - vRGBE;
  vRGB2 = vRGB1;
  vRGB2.setSize(3,1);
  mRGB = vRGB1 * mRGBVI * vRGB2;
  sr = mRGB.get(0,0);*/
//  dcro = getMahalonobisDistCroma(pix, &vCrE, &mCrVI);
//  drgb = getMahalonobisDist(pix, &vRgbE, &mRgbVI);
//  printf("Center pixel has distance CROMA %f, RGB %f\n", dcro, drgb);
//  pix = img->getPixRef(5, 5);
//  pix->print("pix 5,5");
//  dcro = getMahalonobisDistCroma(pix, &vCrE, &mCrVI);
//  drgb = getMahalonobisDist(pix, &vRgbE, &mRgbVI);
//  printf("( 5x, 5y) distance        CROMA %f, RGB %f\n", dcro, drgb);

  if (false)
  { // debug croma/RGB distance - from seed - image
    imgMA = imgPool->getImage(6, true);
    imgMA->copyMeta(img, true);
    imgMA->setNameAndExt("Mahalanobi", NULL);
    imgMA->imgTime.Now();
    img->setNameAndExt("org", NULL);
    for (r = 0; r < int(img->height()); r++)
    {
      pix = imgM->getLine(r);
      pix2 = imgMA->getLine(r);
      for (c = 0; c < int(img->width()); c++)
      { // paint CROMA distance image
        sum = double(pix->p3 + pix->p2 + pix->p1);
        crr = double(pix->p3) / sum;
        vCrE.setRC(0,0, crr);
        dcro = getMahalonobisDistCroma(pix, &vCrE, &mCrVI, 0.0, false);
        //drgb = getMahalonobisDist(pix, &vRgbE, &mRgbVI);
        drgb = dcro;
        pix2->p3 = mini(255,roundi(dcro * 4.0));
        pix2->p2 = mini(255,roundi(drgb * 4.0));
        pix2->p1 = mini(255,255 - roundi(dcro *4.0 + drgb * 4.0));
        pix++;
        pix2++;
      }
    }
  }
  //
  // debug timing
  t1.Now();
  // debug end timing
  res = imageContourMahaCroma(imgM, imgC,
                    seedY, seedX,
                    xmin, ymin, xmax, ymax, maxH,
                    floorOutline, &floorOutlineCnt,
                    &vCrE, &mCrVI,
                    &vRgbE, &mRgbVI,
                    ballance, limit);
  // debug timing (This takes most of the time app. 2/3)
  //t2.Now();
  //printf("imageContourMahaCroma took %f secs\n", t2 - t1);
  // debug end timing
  // debug paint edge image
  if ((imgM != NULL) and (imgC!= NULL) and (imgCroma != NULL))
  {
    paintRoadImage(seedY, seedX,
                   imgM, imgCroma,
                   &vCrE, &mCrVI,
                   &vRgbE, &mRgbVI,
                   ballance, limit
                  );
    cvPutText(imgCroma->cvArr(), s3, cvPoint(10, 15), &font, CV_RGB(255,0,0));
    cvRectangle(imgCroma->cvArr(), cvPoint(xmin, ymin),
                cvPoint(xmax, ymax),
                CV_RGB(255,0,0), 1, 8, 0);
  }
  // too small is less than 100 pixels in outline
  *tooSmall = (floorOutlineCnt < 100);
  //
  // debug
  if (false and res and (imgC != NULL))
  {
    px = floorOutline;
    for (n = 0; n < floorOutlineCnt; n++)
    {
      imgC->setPix(px->r, px->c, red);
      px++;
    }
  }
  // debug endExamples of some of these limitations are described in the next section.

  //
  if (res)
  { // simplify
    // debug timing
    t1.Now();
    // debug end timing
    res = polygonReduce(floorOutline, &floorOutlineCnt,
                        reductionToll, reductionTollZero);
    // debug
    if (false)
      /** @todo try to remove small appendices from the road outline */
      polygonReduce2(floorOutline, &floorOutlineCnt,
                   reductionToll, reductionTollZero);
    // debug end
    // debug timing
    //t2.Now();
    //printf("polygonReduce took %f secs\n", t2 - t1);
    // debug end timing
  }
  if (res)
  { // convert to CvPoint array
    //
    if (floorArea == NULL)
    {
      floorArea = (CvPoint *) malloc(MAX_POLYGON_POINTS * sizeof(CvPoint));
      obst = (bool *) malloc(MAX_POLYGON_POINTS * sizeof(bool));
    }
    //
    pt = floorArea;
    floorAreaCnt = floorOutlineCnt;
    if (floorOutlineCnt > MAX_POLYGON_POINTS)
    { // overflow
      printf("Too many vertexes for CvPoint buffer %d > %d\n",
             floorOutlineCnt, MAX_POLYGON_POINTS);
      floorAreaCnt = MAX_POLYGON_POINTS;
    }
    // convert to CvPoint and bool[]
    ob = obst;
    obCnt = 0;
    for (i = 0; i < floorAreaCnt; i++)
    { // move to CvPoint format
      pt->x = floorOutline[i].c;
      pt->y = floorOutline[i].r;
      // mark as obstacle position if not in image border area
      *ob = not ((pt->x < 13) or (pt->x > int(imgM->width()) - 14) or
                 (pt->y > (maxH - 3)));
      if (*ob)
        obCnt++;
      pt++;
      ob++;
    }
  }
  // debug paint contour in image
  if (false and res and imgC != NULL)
  {
    pt = floorArea;
    cvPolyLine(imgC->cvArr(), &pt, &floorAreaCnt, 1, 1, CV_RGB(0,255,0), 1, 8, 0 );
  }
  // debug end
  //
  return res;
}


///////////////////////////////////////////////////////
// double UImageAna::getRoadProbability(int r, int c,
//                         UImage * img, UImage * imgC,
//                         UPixel * pixE, UPixel * pix1,
//                         UMatrix4 * vECroma, 
//                         UMatrix4 * averageCroma, UMatrix4 * covarCromaInv,
//                         UMatrix4 * averageRgb, UMatrix4 * covarRgbInv,
//                         double diff,   // floating criteria
//                         double diffa)  // absolute criteria

bool UImageAna::paintRoadImage(int seedR, int seedC,
                               UImage * img, UImage * imgC,
                              UMatrix4 * vCroma, UMatrix4 * mCromaInv,
                              UMatrix4 * vRgb, UMatrix4 * mRgbInv,
                              double dyn, double abs
                              )
{
  UMatrix4 vECroma;
  UPixel pixE(128, 128, 128);
  UPixel pix1(128, 128, 128);
  int r, c;
  //
  vECroma.copy(vCroma);
  for (r = 0; r < (int)imgC->height(); r++)
  {
    for (c = 0; c < (int)imgC->width(); c++)
    {
      getRoadProbability(r, c, 
                         img, imgC, 
                         //&pixE, &pix1, 
                         &vECroma, // modified by call
                         vCroma, mCromaInv, // croma
                         //vRgb, mRgbInv, // RGB
                         dyn, 0.0, false); //, abs);
    }
  }
  return true;
}
///////////////////////////////////////////////////////

double UImageAna::getMahalonobisDist(UPixel * pix, UMatrix4 * average, UMatrix4 * covarianceInv)
{
  double result;
  UMatrix4 vRGB1(1,3);
  UMatrix4 vRGB2;
  UMatrix4 mRGB;
  // convert to vector format
  vRGB1.setRow(0, pix->p3, pix->p2, pix->p1);
  // subtract average
  vRGB1 = vRGB1 - *average;
  // make transposed vector as a copy of this
  vRGB2 = vRGB1;
  // simple transpose
  vRGB2.setSize(3,1);
  // calculate mahalonobis distance
  mRGB = vRGB1 * *covarianceInv * vRGB2;
  // extract result
  result = mRGB.get(0,0);
  // return
  return result;
}

//////////////////////////////////////////////////

double UImageAna::getMahalonobisDistCroma(UPixel * pix, UMatrix4 * average, UMatrix4 * covarianceInv,
                                          double averageY, bool useShadow)
{
  double result;
  UMatrix4 vRGB1(1,2);
  UMatrix4 vRGB2;
  UMatrix4 mRGB;
  double crr, crg, sum, r;
  // calculate cromaticity
  sum = double(pix->p1 + pix->p2 + pix->p3);
  crg = double(pix->p2) / sum;
  r = double(pix->p3);
  if (useShadow)
    r += (1.5 - sum) * SHADOW_COMPENSATION_FACTOR;
  crr = r / sum;
  // convert to vector format
  vRGB1.setRow(0, crr, crg);
  // subtract average
  vRGB1 = vRGB1 - *average;
  // make transposed vector as a copy of this
  vRGB2 = vRGB1;
  // simple transpose
  vRGB2.setSize(2,1);
  // calculate mahalonobis distance
  mRGB = vRGB1 * *covarianceInv * vRGB2;
  // extract result
  result = mRGB.get(0,0);
  // return
  return result;
}
