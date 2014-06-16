/***************************************************************************
 *   Copyright (C) 2013 by DTU (Christian Andersen, Andreas Emborg)        *
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

/**
 * Maintains debug image for labyrinth game */
#include <highgui/highgui_c.h>
#include "showimage.h"
#include "fwguppy.h"
#include "labyrinthgame.h"
#include "tiltcontrol.h"
#include "tiltandcraneif.h"


/** image height */
int showImageHeight = 700;
/** debg image width */
int showImageWidth = 1100;
/** is thread running */
bool showImageThreadRunning = false;
/** stop running thread */
bool stopThread;
/** Thread handle for frame read thread. */
pthread_t thShowImage;
/** image buffer */
UImage * imgBufferDebug = NULL;
/**
Mouse move-click event type */
int globalMouseEvent = CV_EVENT_MOUSEMOVE;
/**
Mouse shift-ctrl-alt button combination */
int globalMouseFlag = 0;
//CvPoint globalPixPoint = {0,0};
/**
If a square with left mouse button */
CvPoint globalPixPoint1 = {0,0};
CvPoint globalPixPoint2 = {0,0};
CvPoint globalPixPoint3 = {0,0};
/**
With shift button */
CvPoint globalPixPointS = {0,0};
bool globalMouseDrag = false;
const char * LabyimgName = "labyrinthImage";
int showImageClickRoute = false;
int showImageClickFrame = false;
// from top-left going clockwise
int framePoints[4][2] = {{ 20, 120},
                         {400, 120},
                         {400, 500},
                         { 20, 500}};
int framePointsCnt = 4;
// from start position to end
int routePoints[MRP][2];
int routePointsCnt = 0;
// path in image coordinates
int pathImgPoints[MRP][2];
int pathImgPointsCnt = 0;
// angle of frame top line
float frameAngle;
// ball history (in integers)
const int MHC = 10000;
CvPoint ballHistory[MHC];
int ballHistoryCnt = 0;
// mean value for board in ballance (8-bit D/A units)
///
/**
New openCV on-mouse callback function
\param x,y is new mouse position in pixels
\param flags is mouse flags (shift, drag etc.
\param param is ??? */
void on_mouse(int event, int x, int y, int flags, void* param)
{
/*#define CV_EVENT_MOUSEMOVE      0
#define CV_EVENT_LBUTTONDOWN    1
#define CV_EVENT_RBUTTONDOWN    2
#define CV_EVENT_MBUTTONDOWN    3
#define CV_EVENT_LBUTTONUP      4
#define CV_EVENT_RBUTTONUP      5
#define CV_EVENT_MBUTTONUP      6
#define CV_EVENT_LBUTTONDBLCLK  7
#define CV_EVENT_RBUTTONDBLCLK  8
#define CV_EVENT_MBUTTONDBLCLK  9*/
  // flags defined in openCVHighGui
  // CV_EVENT_FLAG_LBUTTON   1
  // CV_EVENT_FLAG_RBUTTON   2
  // CV_EVENT_FLAG_MBUTTON   4
  // CV_EVENT_FLAG_CTRLKEY   8
  // CV_EVENT_FLAG_SHIFTKEY  16
  // CV_EVENT_FLAG_ALTKEY    32
  switch (event)
  {
    case CV_EVENT_MOUSEMOVE:
      if (globalMouseDrag)
      {
        globalPixPoint3.x = x;
        globalPixPoint3.y = y;
      }
      break;
    case CV_EVENT_LBUTTONDOWN:
      globalPixPoint1.x = x;
      globalPixPoint1.y = y;
      globalPixPoint2 = globalPixPoint1;
      globalPixPoint3 = globalPixPoint1;
      globalPixPointS = globalPixPoint1;
      globalMouseDrag = true;
      break;
      case CV_EVENT_RBUTTONDOWN: break;
      case CV_EVENT_MBUTTONDOWN: break;
    case CV_EVENT_LBUTTONUP:
      if ((flags & CV_EVENT_FLAG_SHIFTKEY) == 0)
      { // not shift
        globalPixPoint2.x = x;
        globalPixPoint2.y = y;
      }
      else
      { // shift
        globalPixPointS.x = x;
        globalPixPointS.y = y;
      }
      globalMouseDrag = false;
      // are we defining a route
      if (showImageClickFrame)
      {
        if (framePointsCnt == 4)
          framePointsCnt = 0;
        framePoints[framePointsCnt][0] = y;
        framePoints[framePointsCnt][1] = x;
        framePointsCnt++;
      }
      else if (showImageClickRoute)
      {
        if (routePointsCnt >= MRP)
          routePointsCnt = 0;
        routePoints[routePointsCnt][0] = y;
        routePoints[routePointsCnt][1] = x;
        routePointsCnt++;
      }
      else
      {
        if (not ballOK)
        {
          ballPosition[0] = x;
          ballPosition[1] = y;
          ballOK = true;
        }
        framePointsCnt = 4;
      }
      break;
    case CV_EVENT_RBUTTONUP: break;
    case CV_EVENT_MBUTTONUP: break;
    case CV_EVENT_LBUTTONDBLCLK: break;
    case CV_EVENT_RBUTTONDBLCLK: break;
    case CV_EVENT_MBUTTONDBLCLK: break;
    default: break;
  }
  globalMouseEvent = event;
  globalMouseFlag = flags;
}

//////////////////////////////////////////////////////////////////

void showDebugImage()
{
  UImage * img = imgBufferDebug;
  void * wHnd;
  if (img != NULL)
  {
    if (img->tryLock())
    {
      wHnd = cvGetWindowHandle(LabyimgName);
      if (wHnd == NULL)
      { // (re)create window
        cvNamedWindow(LabyimgName, CV_WINDOW_AUTOSIZE);
        cvSetMouseCallback(LabyimgName, on_mouse);
      }
      cvShowImage(LabyimgName, img->cvArr());
      img->used = true;
      img->unlock();
    }
  }
}

//////////////////////////////////////////////////////////////////

void removeImage()
{
  void * wHnd;
  //
  wHnd = cvGetWindowHandle(LabyimgName);
  if (wHnd != NULL)
  {
    cvDestroyWindow(LabyimgName);
    cvWaitKey(70); // parameter is service time in ms
  }
}

//////////////////////////////////////////////////////////////////

/**
 * Copy part of source image to destination image and do color conversion
 * \param src is the source image
 * \param top is the first line in source image to copy - format is RGGB
 * \param hgt is number of lines to copy
 * \param dst is destination image */
void  copyUpdated(UImage * src, int top, int hgt, UImage * dst)
{
  UPixel * dp1, *dp2, dp;
  uint8_t * sp1, * sp2; // source pixels
  int w = src->width();
  //
  for (int r = top; r < top + hgt; r += 2)
  {
    sp1 = (uint8_t *) src->getLine(r);     // RG pixel line
    sp2 = (uint8_t *) src->getLine(r + 1); // GB pixel line
    dp1 = dst->getLine(r);
    dp2 = dst->getLine(r + 1);
    for (int c = 0; c < w; c += 2)
    { // make BW image from green pixels - no, sum of all pixels
      int g = (sp1[1] + sp2[0] + sp1[0] + sp2[1]);
      if (g > 128)
      {
        g = 128 + (g - 128)/3;
        if (g > 255)
          g = 255;
      }
      dp.p1 = g;
      dp.p2 = g;
      dp.p3 = g;
      // paint 4 pixels with this value
      for (int i=0; i < 2; i++)
        *dp1++ = dp;
      for (int i=0; i < 2; i++)
        *dp2++ = dp;
      sp1 += 2;
      sp2 += 2;
    }
  }
}

/// /////////////////////////////////////////////////////////////

void showStatus()
{
  CvScalar red = CV_RGB(255, 0, 0);
//  CvScalar redMag = CV_RGB(180, 0, 100);
  CvScalar orange = CV_RGB(180, 100, 0);
//  CvScalar green = CV_RGB(0, 155, 0);
   CvScalar yellow = CV_RGB(175, 175, 0);
//   CvScalar magenta = CV_RGB(155, 0, 155);
   CvScalar cyan = CV_RGB(0, 255, 255);
  CvFont font;
  const int MSL = 100;
  char s[MSL];
  CvPoint p1, p2;
  UImage * img = imgBufferDebug;
  int linespace = 17;
  const int reserverHeight = 220;
  const char * power;
  //
  //cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
  //               1.0, 1.0, 0.0, 1, 8);
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);
  for (int r = 0; r < reserverHeight /*(int)imgFull->height()*/; r++)
  { // clear status area
    int n = img->width() - imgFull->width();
    char * p1 = (char *)img->getLine(r);
    p1 += imgFull->width() * img->getChannels();
    memset(p1, '\0', n * img->getChannels());
  }
  p1.x = imgFull->width() + linespace;
  p1.y = linespace;
  snprintf(s, MSL, "Image  %d", imageNumber);
  cvPutText(img->cvArr(), s, p1, &font, orange);
  p1.x = imgFull->width() + linespace;
  p1.y += linespace;
  snprintf(s, MSL, "Gain    %5d", gainValue);
  cvPutText(img->cvArr(), s, p1, &font, orange);
  p1.y += linespace;
  snprintf(s, MSL, "Shutter %5d", shutterValue);
  cvPutText(img->cvArr(), s, p1, &font, orange);
  p1.y += linespace;
  snprintf(s, MSL, "Top     %5d", frameTop);
  cvPutText(img->cvArr(), s, p1, &font, orange);
  //
  p1.y += linespace;
  snprintf(s, MSL, "Ball    %6.2fr, %6.2fc %s", ballPosition[1] /*row*/ + frameTop, ballPosition[0] /*col*/, bool2str(ballOK));
  cvPutText(img->cvArr(), s, p1, &font, orange);
  p1.y += linespace;
  snprintf(s, MSL, "Ball Q: %5d  B-R %d", ballQuality, ballQualityBlueRed);
  cvPutText(img->cvArr(), s, p1, &font, orange);
//   int r = globalPixPoint2.y & 0xFFFE;
//   int c = globalPixPoint2.x & 0xFFFE;
//   if (r > 0 and c > 0)
//   {
//     p1.y += linespace;
//     if (r > ROWS)
//       r = ROWS - 2;
//     if (c > COLS)
//       c = COLS - 2;
//     unsigned char * pix = (unsigned char *) imgFull->getCharRef(r, c);
//     snprintf(s, MSL, "BG      %3d, %3d  row %3d", pix[0], pix[1], r);
//     cvPutText(img->cvArr(), s, p1, &font, orange);
//     p1.y += linespace;
//     pix += imgFull->width();
//     snprintf(s, MSL, "GR      %3d, %3d  col %3d", pix[0], pix[1], c);
//     cvPutText(img->cvArr(), s, p1, &font, orange);
//   }
  p1.y += linespace;
  snprintf(s, MSL, "framerate %.1f/s", frameRate);
  cvPutText(img->cvArr(), s, p1, &font, orange);
  p1.y += linespace;
  snprintf(s, MSL, "tilt  %.5fr, %.5fc", tiltYangle, tiltXangle);
  cvPutText(img->cvArr(), s, p1, &font, orange);
  p1.y += linespace;
  snprintf(s, MSL, "refPos  %5.1fr, %5.1fc", currentRefY, currentRefX);
  cvPutText(img->cvArr(), s, p1, &font, orange);
  p1.y += linespace;
  snprintf(s, MSL, "ball %d, cranesw %d, switch %d", ballAvailable,
                       craneSwitch, startGameSwitch);
  cvPutText(img->cvArr(), s, p1, &font, yellow);
  p1.y += linespace;
  snprintf(s, MSL, "interface state %s", getControlStateString());
  cvPutText(img->cvArr(), s, p1, &font, yellow);
  p1.y += linespace;
  if (powerOn)
    power = "ON";
  else
    power = "OFF";
  snprintf(s, MSL, "power %s, game %s", power, getGameStateString());
  cvPutText(img->cvArr(), s, p1, &font, yellow);
  //
  if (ballOK)
  {
    p1.x = ballPosition[0];
    p1.y = ballPosition[1] + frameTop;
    cvCircle(img->cvArr(), p1, 10, cyan, 2, 4, 0);
  }
  //
  p1.x = imgFull->width() + 1;
  p1.y = frameTop;
  p2.x = p1.x + 10;
  p2.y = p1.y + frameHeight;
  cvRectangle(img->cvArr(), p1, p2, red, 1);
  //
  p1.x = 0;
  p1.y = 0;
  p2.x = imgFull->width();
  p2.y = imgFull->height();
  cvRectangle(img->cvArr(), p1, p2, yellow, 1);
}

/// //////////////////////////////////////////////////////////////

void showBallArea()
{
//  CvScalar red = CV_RGB(255, 0, 0);
//  CvScalar redMag = CV_RGB(180, 0, 100);
//  CvScalar orange = CV_RGB(180, 100, 0);
//  CvScalar green = CV_RGB(0, 155, 0);
   CvScalar gray = CV_RGB(175, 175, 175);
   CvScalar cyan = CV_RGB(0, 255, 255);
   CvScalar yellow = CV_RGB(175, 175, 0);
  CvFont font;
  const int MSL = 20;
  char s[MSL];
  CvPoint p1, p2, p3;
  UImage * img = imgBufferDebug;
  int linespace = 19;
  int linepitch = 33;
  const int reserverHeight = 220;
  CvScalar pixcol;
  int ballQR = (roundi(ballPosition[1]) + frameTop) & 0xFFFE;
  int ballQC = roundi(ballPosition[0]) & 0xFFFE;
  //
  if (imgFull->tryLock())
  {
    //cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
    //               1.0, 1.0, 0.0, 1, 8);
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                  1.0, 1.0, 0.0, 1, 8);
    // clear area
    for (int r = reserverHeight; r < (int)imgFull->height(); r++)
    { // clear status area
      int n = img->width() - imgFull->width() - 2;
      char * pc = (char *)img->getLine(r);
      pc += (imgFull->width() + 1) * (img->getChannels());
      memset(pc, '\0', n * img->getChannels());
    }
    p2.x = imgFull->width() + 2 + linepitch;
    p2.y = reserverHeight + linespace;
    for (int c = -4; c < 5; c++)
    {
      snprintf(s, MSL, "%3d", ballQC + c);
      cvPutText(img->cvArr(), s, p2, &font, gray);
      p2.x += linepitch;
    }
    for (int r = -4; r < 5; r++)
    { // get pixel pointer to first value to show
      p2.y += linespace;
      p2.x = imgFull->width() + 2;
      // and fat colored line start and end
      p1.y = p2.y - linespace/2 + 2;
      p1.x = p2.x + 8 + linepitch; // start a bit in
      p3.y = p1.y;
      p3.x = p1.x - 12 + 2 * linepitch; // a bit back
      int brow = ballQR + r;
      if (brow < 0)
        brow = 0;
      else if (brow >= (int) imgFull->height())
        brow = imgFull->height() - 1;
      snprintf(s, MSL, "%3d", brow);
      cvPutText(img->cvArr(), s, p2, &font, gray);
      p2.x += linepitch;
      for (int c = -4; c < 5; c++)
      {
        int bcol = ballQC + c;
        if (bcol < 0)
          bcol = 0;
        if (bcol >= (int)imgFull->width())
          bcol = imgFull->width() - 1;
        uint8_t * ps = (uint8_t *) imgFull->getCharRef(brow, bcol);
        uint8_t cl = *ps/2 + 64;
        // make color of fat line
        if (brow % 2 == 0)
          if (bcol % 2 == 0)
            pixcol = CV_RGB(0,0,cl);
          else
            pixcol = CV_RGB(0,cl,0);
        else
          if (bcol % 2 == 0)
            pixcol = CV_RGB(0,cl,0);
          else
            pixcol = CV_RGB(cl,0,0);
        // print fat line
        cvLine(img->cvArr(), p1, p3, pixcol, linespace - 2);
        // printf value on top of fat line
        snprintf(s, MSL, "%3d", *ps);
        cvPutText(img->cvArr(), s, p2, &font, cyan);
        p1.x += linepitch;
        p2.x += linepitch;
        p3.x += linepitch;
      }
    }
    //
    // copy ball patch x 4
    if (ballQR > 10 and ballQC > 14 and
        ballQR < ((int)imgFull->height() - 10) and
        ballQC < ((int)imgFull->width() - 14))
    {
      UPixel * dp1;
      uint8_t * sp1, * sp2; // source pixels
      p1.y += linepitch;
      p1.x = imgFull->width() + 18;
      p2.x = p1.x + 26*12;
      p2.y = p1.y + 22*12;
      //
      for (int r = 0; r < 22; r += 2)
      { // source area upper left
        sp1 = (uint8_t *) imgFull->getCharRef(ballQR + r - 10, ballQC - 14);     // RG pixel line
        sp2 = (uint8_t *) imgFull->getCharRef(ballQR + r + 1 - 10, ballQC - 14); // GB pixel line
        for (int c = 0; c < 26; c += 2)
        { // make BW image from green pixels - no, sum of all pixels
          for (int dr = 0; dr < 12; dr++)
          { // top line blue and green
            dp1 = img->getPixRef(p1.y + r * 12 + dr, p1.x + c * 12);
            for (int dc = 0; dc < 12; dc++)
              (dp1++)->set(sp1[0], 30, 30);
            for (int dc = 0; dc < 12; dc++)
              (dp1++)->set(30, sp1[1] + greenOffset[0], 30);
          }
          for (int dr = 0; dr < 12; dr++)
          { // bottom line green and red
            dp1 = img->getPixRef(p1.y + (r+1) * 12 + dr, p1.x + c * 12);
            for (int dc = 0; dc < 12; dc++)
              (dp1++)->set(30, sp2[0] + greenOffset[1], 30);
            for (int dc = 0; dc < 12; dc++)
              (dp1++)->set(30, 30, sp2[1]);
          }
          // advance to next source quad-pixel
          sp1 += 2;
          sp2 += 2;
        }
      }
      cvRectangle(img->cvArr(), p1, p2, yellow, 1);
      float dr = (ballPosition[1] + frameTop - (ballQR - 10)) * 12.0;
      float dc = (ballPosition[0] - (ballQC - 14)) * 12.0;
      p2.x = p1.x + roundi(dc) + 6;
      p2.y = p1.y + roundi(dr) - 10 + 6;
      p3.x = p2.x;
      p3.y = p2.y + 20;
      cvLine(img->cvArr(), p2, p3, yellow, 1);
      p2.x = p1.x + roundi(dc) - 10 + 6;
      p2.y = p1.y + roundi(dr) + 6;
      p3.x = p2.x + 20;
      p3.y = p2.y;
      cvLine(img->cvArr(), p2, p3, yellow, 1);
    }
    //
    imgFull->unlock();
  }
}

/// ///////////////////////////////////////////////////////////////

void showRouteAndFrame()
{
  CvPoint p1, p2;
  UImage * img = imgBufferDebug;
  CvScalar red = CV_RGB(255, 30, 30);
  CvScalar blue = CV_RGB(30, 30, 255);
  //
  p1.x = framePoints[0][1];
  p1.y = framePoints[0][0];
  for (int i = 1; i < 4; i++)
  {
    p2.x = framePoints[i][1];
    p2.y = framePoints[i][0];
    cvLine(img->cvArr(), p1, p2, red);
    p1 = p2;
  }
  p2.x = framePoints[0][1];
  p2.y = framePoints[0][0];
  cvLine(img->cvArr(), p1, p2, red);
  //
  //
//   p1.x = routePoints[1];
//   p1.y = routePoints[0];
//   for (int i = 2; i < routePointsCnt * 2; i += 2)
//   {
//     p2.x = routePoints[i+1];
//     p2.y = routePoints[i];;
//     cvLine(img->cvArr(), p1, p2, blue);
//     p1 = p2;
//   }
  //
  p1.x = pathImgPoints[0][0];
  p1.y = pathImgPoints[0][1];
  for (int i = 1; i < routePointsCnt; i++)
  {
    p2.x = pathImgPoints[i][0];
    p2.y = pathImgPoints[i][1];
    cvLine(img->cvArr(), p1, p2, blue);
    p1 = p2;
  }
}

/// ///////////////////////////////////////////////////////////////

void showControlLines()
{
  CvPoint p1, p2;
  UImage * img = imgBufferDebug;
  CvScalar red = CV_RGB(255, 70, 70);
//  CvScalar blue = CV_RGB(30, 30, 255);
  float refR, refC;
  // convert reference point to image coordinates
  frameToImage(currentRefX, currentRefY, &refR, &refC);
  p1.x = roundi(refC);
  p1.y = roundi(refR);
  p2.x = ballPosition[0];
  p2.y = ballPosition[1] + frameTop;
  cvLine(img->cvArr(), p1, p2, red, 2);
}

void showBallHistory()
{
  UImage * img = imgBufferDebug;
  CvScalar red = CV_RGB(255, 70, 70);
  for (int bh = 1; bh < ballHistoryCnt; bh++)
  {
    cvLine(img->cvArr(), ballHistory[bh-1], ballHistory[bh], red, 2);
  }
}

/// ///////////////////////////////////////////////////////////////


/**
 * Converts the basic route path to pixel coordinates inside frame */
void convertToFramedRoute()
{
//  int path[][2] = pathImgPoints;
  // find frame angle and position
  double fy = framePoints[0][0]; // row
  double fx = framePoints[0][1]; // column
  double dy = framePoints[1][0] - fy; // row
  double dx = framePoints[1][1] - fx; // column
  frameAngle = atan2(dy, dx);
  int * pp = routePoints[0];
  for (int i = 0; i < routePointsCnt; i++)
  {
    double py = *pp++; // row first
    double px = *pp++; // column
    double x1, y1;
    x1 = px * cos(frameAngle) - py * sin(frameAngle) + fx;
    y1 = px * sin(frameAngle) + py * cos(frameAngle) + fy;
    pathImgPoints[i][0] = roundi(x1);
    pathImgPoints[i][1] = roundi(y1);
  }
  pathImgPointsCnt = routePointsCnt;
}

/// ///////////////////////////////////////////////////////////////

void * run(void * notUsed)
{ // Start thread here and call thread function
  UImage * img;
  showImageThreadRunning = true;
  imgBufferDebug = new UImage();
  UTime tupd;
  int n = 0;
  bool isControlling = false;
  //
  // set ROI to where the ball should be
  // setROIrequest(framePoints[0][0], 60);
  // Convert to framed route
  convertToFramedRoute();
  // copy to control path
  for (int i = 0; i < routePointsCnt; i++)
  { // convert to x=c and y=r notation for tilt controller
    pathPoints[i][0] = routePoints[i][1]; // column
    pathPoints[i][1] = routePoints[i][0]; // row
  }
  pathPointsCnt = routePointsCnt;
  //
  img = imgBufferDebug;
  img->setSize(showImageHeight, showImageWidth, 3, 8, "RGB");
  while (not stopThread)
  {
    if (imgFull->getUpdatedTime() > tupd)
    {
      if (globalDebug >= 2)
        // copy all
        copyUpdated(imgFull, 0, imgFull->height(), img);
      else
        // copy just new part
        copyUpdated(imgFull, frameTop, frameHeight, img);
      tupd = imgFull->getUpdatedTime();
      if (ballOK and not initialImage)// and readyToControl)
      { // save also past positions of ball for display
        bool needed = true;
        if (ballHistoryCnt > 0)
        {
          if ((absi(ballPosition[0] - ballHistory[ballHistoryCnt - 1].x) <= 1) and
              (absi(ballPosition[1] - ballHistory[ballHistoryCnt - 1].y) <= 1))
            needed = false;
        }
        if (needed)
        {
          ballHistory[ballHistoryCnt].x = roundi(ballPosition[0]);
          ballHistory[ballHistoryCnt].y = roundi(ballPosition[1]) + frameTop;
          if (ballHistoryCnt < MHC)
            ballHistoryCnt++;
        }
      }
    }
    // generate debug image
    showRouteAndFrame();
    showBallHistory();
    showStatus();
    if (n++ % 4 == 0)
      showBallArea();
    if (readyToControl)
    {
      if (isControlling)
        showControlLines();
      else
      { // going into (new) control, reset history
        ballHistoryCnt = 0;
        isControlling = true;
      }
    }
    // show debug image on screen
    showDebugImage();
    cvWaitKey(70); // parameter is service time in ms
    Wait(0.03);
  }
  showImageThreadRunning = false;
  removeImage();
  // exit thread
  pthread_exit((void*)NULL);
  return NULL;
}

////////////////////////////////////////////////////

bool startShowImage()
{
  pthread_attr_t  thAttr;
  int i = 0;
  //
  if (not showImageThreadRunning)
  { // start thread
    pthread_attr_init(&thAttr);
    //
    stopThread = false;
    // create socket server thread
    if (pthread_create(&thShowImage, &thAttr, &run, NULL) != 0)
      // report error
      perror("showimage thread");
      // wait for thread to initialize
    while ((not showImageThreadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not showImageThreadRunning)
    { // failed to start
      printf("startShowImage: Failed to start thread - in time (5 sec)\n");
    }
    pthread_attr_destroy(&thAttr);
  }
  return showImageThreadRunning;
}

//////////////////////////////////////////////////////////////

void stopShowImage()
{
  if (showImageThreadRunning)
  {
    stopThread = true;
    if (showImageThreadRunning)
      pthread_join(thShowImage, NULL);
    // debug
    printf("stopShowImage: thread stopped\n");
  }
}


///  ///////////////////////////////////////////////////////////


