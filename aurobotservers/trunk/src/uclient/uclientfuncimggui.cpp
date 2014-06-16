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
#include <ugen4/ufuzzypixel.h>

#include "uclientfuncimggui.h"

/**
Threshold for not showing UV value */
int  globalThresholdMin = 40;
int  globalThresholdMax = 200;
/**
Flag for uvThreshold is changed */
bool globalUvRedisplay = true;
/**
Flag for type of display that is to be performed */
UClientFuncImgGui::ColorDisplayType
         globalColorDisplayType = UClientFuncImgGui::CDT_NONE;
/**
Mouse move-click event type */
int globalMouseEvent = CV_EVENT_MOUSEMOVE;
/**
Mounse shift-ctrl-alt button combination */
int globalMouseFlag = 0;
//CvPoint globalPixPoint = {0,0};
/**
If a square with left mouse button */
CvPoint globalPixPoint1 = {0,0};
CvPoint globalPixPoint2 = {0,0};
/**
With shift button */
CvPoint globalPixPointS = {0,0};


UHighGuiWindowHandle::UHighGuiWindowHandle()
{
  img = NULL;
}

//////////////////////////////////////////////////////

bool UHighGuiWindowHandle::setImage(UImage * toImg)
{
  if (img == NULL)
    img = new UImage800();
  if ((img != NULL) and (img != toImg))
    img->copy(toImg);
  return (img != NULL);
}

//////////////////////////////////////////////////////

UImage * UHighGuiWindowHandle::makeImage(const char * name)
{
  if (img == NULL)
    img = new UImage800();
  if (img != NULL)
  {
    img->setMaxSize43();
    img->clear(128);
    if (name == NULL)
      strncpy(cvWndName, "laserImage", MAX_CV_WINDOW_NAME_SIZE);
    else if (strlen(name) == 0)
      strncpy(cvWndName, "noname", MAX_CV_WINDOW_NAME_SIZE);
    else
      strncpy(cvWndName, name, MAX_CV_WINDOW_NAME_SIZE);
  }
  return img;
}

//////////////////////////////////////////////////////

#if (CV_MAJOR_VERSION >= 1) or (CV_MINOR_VERSION == 97)
/**
New library openCV version 0.97 requires:
"void on_mouse(int event, int x, int y, int flags, void* param)"
Modify this header as needed.  */
void on_mouse(int event, int x, int y, int flags, void* param)
#else
/**
    Old mouse call-back call - openCV version 0.96 */
    void on_mouse(int event, int x, int y, int flags)
#endif
{
  // flags defined in openCVHighGui
  // CV_EVENT_FLAG_LBUTTON   1
  // CV_EVENT_FLAG_RBUTTON   2
  // CV_EVENT_FLAG_MBUTTON   4
  // CV_EVENT_FLAG_CTRLKEY   8
  // CV_EVENT_FLAG_SHIFTKEY  16
  // CV_EVENT_FLAG_ALTKEY    32
  switch (event)
  {
    case CV_EVENT_MOUSEMOVE: break;
    case CV_EVENT_LBUTTONDOWN:
        globalPixPoint1.x = x;
        globalPixPoint1.y = y;
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
      globalUvRedisplay = true;
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
/*  if (event != CV_EVENT_MOUSEMOVE)
    globalUvRedisplay = true;*/
}

///////////////////////////////////////////////////////

bool UHighGuiWindowHandle::showImage(UImage * toImg, bool force)
{
  bool set = true;
  bool result = true;
  void * wHnd;
  //
  set = (img == NULL);
  if (not set)
    set = force or (img->imgTime != toImg->imgTime);
  if (set)
    result = setImage(toImg);
  if (set and result)
  { // show image
    wHnd = cvGetWindowHandle(cvWndName);
    if (wHnd == NULL)
    { // (re)create window
      cvNamedWindow(cvWndName, CV_WINDOW_AUTOSIZE);
      cvSetMouseCallback(cvWndName, on_mouse);
    }
    // create window
    cvShowImage(cvWndName, img->cvArr());
    // allow a little tile to display on screen
    // debug
    //fprintf(stderr, "wait\n");
    // debug end
    cvWaitKey(70); // parameter is service time in ms
    // debug
    //fprintf(stderr, "waited\n");
    // debug end
  }
  return true;
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

UClientFuncImgGui::UClientFuncImgGui()
 : UClientFuncImage()
{
  windowCnt = 0;
  imgUV = NULL;
  imgToMax = false;
  focalLine = -1;
  fuzzyWidth = -1;
  fuzzyClasses = 3;
  fuzzy = NULL;
  fuzzyIter = 0;
  fuzzyLine = -1;
  fuzzyCol = 100;
  imgRefr = NULL;
  saveOnMatchAndShow = true;
  saveOnMatch = false;
}

//////////////////////////////////////////////////////

UClientFuncImgGui::~UClientFuncImgGui()
{
  int i;
  UHighGuiWindowHandle * wnd;
  //
  for (i = 0; i < windowCnt; i++)
  {
    wnd = window[i];
    cvDestroyWindow(wnd->cvWndName);
    delete wnd;
    wnd = NULL;
  }
  if (fuzzy != NULL)
    delete fuzzy;
}

//////////////////////////////////////////////////////

UHighGuiWindowHandle * UClientFuncImgGui::getWndName(char * name, bool mayCreate)
{
  int i;
  UHighGuiWindowHandle * wnd = NULL;
  //
  for (i = 0; i < windowCnt; i++)
  {
    wnd = window[i];
    if (strcmp(wnd->cvWndName, name) == 0)
      break;
    wnd = NULL;
  }
  if ((wnd == NULL) and mayCreate)
  {
    if (windowCnt < MAX_WINDOWS)
    {
      wnd = new UHighGuiWindowHandle();
      window[windowCnt] = wnd;
      strncpy(wnd->cvWndName, name, MAX_CV_WINDOW_NAME_SIZE);
      windowCnt++;
    }
    else
    {
      printf("NO SPACE for more windows (max = %d) -- use clear to free more space\n", MAX_WINDOWS);
    }
  }
  return wnd;
}

//////////////////////////////////////////////////////

void UClientFuncImgGui::gotNewImage(UImage * img, int poolNum, USmlTag * tag)
{
  UHighGuiWindowHandle * wnd;
  UImage800 * imgMax = NULL;
  UImage * imgd = img;
  bool forceDisplay = true;
  const int MFL = MAX_FILENAME_LENGTH;
  char fn[MFL];
  bool showImage = true;
  //
  if (false)
  {
    switch (img->getColorType())
    { //PIX_PLANES_BW , PIX_PLANES_RGB,  PIX_PLANES_BGR,  PIX_PLANES_YUV.
      case PIX_PLANES_BW:  /* printf("BW\n") */ ; img->toBGR(NULL); break;
      case PIX_PLANES_RGB: /* printf("RGB\n") */; img->toBGR(NULL); break;
      case PIX_PLANES_BGR: /* printf("BGR\n") */; break;
      case PIX_PLANES_YUV: /* printf("YUV\n") */; img->toBGR(NULL); break;
      case PIX_PLANES_YUV420: /* printf("YUV\n") */; img->toBGR(NULL); break;
      default: break;
    }
  }
  else
    // convert to BGR format - regardless of source format
    img->toBGR(NULL);

  //
  if (imgToMax)
  {
    imgMax = new UImage800();
    if (imgMax != NULL)
      imgMax->copyToMaxRes(img);
    imgd = imgMax;
  }
  if (saveOnMatch)
  {
    if (strstr(img->name, saveOnMatchStr) != NULL)
    {
      snprintf(fn, MFL, "%s/%s.png", imagePath, img->name);
      cvSaveImage(fn, imgd->cvArr());
      showImage = saveOnMatchAndShow;
    }
  }
  wnd = getWndName(img->name, false);
  if (wnd == NULL and showImage)
  { // Create a window
    wnd = getWndName(img->name, true);
  }
  if (wnd != NULL)
  { // show image and make a copy for analysis
    if (focalLine >= 0)
    { // set line from mouse
      focalLine = globalPixPoint1.y;
      // get pointer to first pixel on used line
      showFocusNumber(img, imgd);
      showLine(img, imgd, focalLine);
    }
    wnd->showImage(imgd, forceDisplay);
    globalUvRedisplay = true;
  }
  if (imgMax != NULL)
    delete imgMax;
}

////////////////////////////////////////////////////

void UClientFuncImgGui::doTimeTick()
{ // do the tisplay events needed to show the image
  // check for UV window
  if (globalUvRedisplay and
     (globalColorDisplayType != CDT_NONE))
  { // redisplay UV window
    showUVImage();
    globalUvRedisplay = false;
  }
  if (imgRefr != NULL)
  {
    gotNewImage(imgRefr, 0, NULL);
    imgRefr = NULL;
  }
  //cvWaitKey(12); // parameter is service time in ms
}

////////////////////////////////////////////////////

void  on_trackbar(int value)
{ // trackbar on UV image
  globalUvRedisplay = true;
}

//////////////////////////////////////////////////////////

void UClientFuncImgGui::showLine(
              UImage * imgs, // source image
              UImage * imgd, // destination image
              int line)    // line to analyze
{
  int i, yt, yb, xl;
  CvPoint p1r, p2r, p1g, p2g, p1b, p2b;
  CvScalar red = CV_RGB(255, 0, 0);
  CvScalar green = CV_RGB(0, 255, 0);
  CvScalar blue = CV_RGB(0, 0, 255);
  CvScalar yellow = CV_RGB(255, 255, 0);
  UPixel * pix;
  int pixCnt;
  const int MBL = 2000;
  UPixel pixBuff[MBL];
  UImage * img;
  //
  if ((imgs != NULL) and (imgd != NULL) and
      (line >= 0) and (line <= int(imgs->height())))
  { // get image bufer with at least this size image
    img = getImageBuffer(0, 480, 640, 3, 8);
    pix = imgs->getPixRef(line, img->width() / 6);
    pixCnt = (imgs->width() * 2) / 3;
    memcpy(pixBuff, pix, sizeof(pix) * pixCnt);
    pix = pixBuff;
    // set height linit for curve
    yt = imgd->height() / 5;
    yb = (imgd->height() * 4) / 5;
    // set left edge
    xl = imgd->width() / 6;
    // set first value
    p1r.x = xl;
    p1g.x = xl;
    p1b.x = xl;
    p1r.y = (pix->p3*(yt-yb))/256 + yb;
    p1g.y = (pix->p2*(yt-yb))/256 + yb;
    p1b.y = (pix->p1*(yt-yb))/256 + yb;
    for (i = 0; i < pixCnt; i++)
    { // advance to next point
      p2r = p1r;
      p2g = p1g;
      p2b = p1b;
      pix++;
      // set pixel curve value
      p1r.x++;
      p1g.x++;
      p1b.x++;
      p1r.y = (pix->p3*(yt-yb))/256 + yb;
      p1g.y = (pix->p2*(yt-yb))/256 + yb;
      p1b.y = (pix->p1*(yt-yb))/256 + yb;
      // paint line
      cvLine(imgd->cvArr(), p2r, p1r, red, 1, 4, 0);
      cvLine(imgd->cvArr(), p2g, p1g, green, 1, 4, 0);
      cvLine(imgd->cvArr(), p2b, p1b, blue, 1, 4, 0);
    }
    // paint used line as intensity curve
    p1r.x = imgd->width() / 6;
    p2r.x = imgd->width() - p1r.x;
    p1r.y = focalLine;
    p2r.y = p1r.y;
    cvLine(imgd->cvArr(), p2r, p1r, yellow, 1, 4, 0);
  }
}

//////////////////////////////////////////////////////////

void UClientFuncImgGui::showFocusNumber(
              UImage * imgs, // source image
              UImage * imgd // destination image
              )
{
  UPixel * pix, * pix2;
  int r, c;
  CvFont font;
  CvScalar red = CV_RGB(255, 0, 0);
  CvScalar green = CV_RGB(0, 255, 0);
  CvScalar blue = CV_RGB(0, 0, 255);
  CvPoint pos;
  int vr, vg, vb;
  int mr = 0, mg = 0, mb = 0;
  const int MSL = 50;
  char s[MSL];
  //
  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
              0.5, 0.5, 0.0, 1, 8);
  //
  for (r = 0; r < int(imgs->height()); r+= 3)
  {
    pix = imgs->getLine(r);
    for (c = 1; c < int(imgs->width()); c++)
    {
      pix2 = pix++;
      vr = absi(pix2->p3 - pix->p3);
      vg = absi(pix2->p2 - pix->p2);
      vb = absi(pix2->p1 - pix->p1);
      if (vr > mr)
        mr = vr;
      if (vg > mg)
        mg = vg;
      if (vb > mb)
        mb = vb;
    }
  }
  pos.x = 10;
  pos.y = int(imgd->height()) - 15;
  snprintf(s, MSL, "%3d", mr);
  cvPutText(imgd->cvArr(), s, pos, &font, red);
  pos.x += 40;
  snprintf(s, MSL, "%3d", mg);
  cvPutText(imgd->cvArr(), s, pos, &font, green);
  pos.x += 40;
  snprintf(s, MSL, "%3d", mb);
  cvPutText(imgd->cvArr(), s, pos, &font, blue);

}

//////////////////////////////////////////////////////////

void UClientFuncImgGui::showUVImage()
{
  UImage * imgsrc = NULL;
  UImage * imgyuv = NULL;
  void * wHnd;
  const char wndUV[] = "UVimg";
  const char wndMin[] = "minY";
  const char wndMax[] = "maxY";
  UPixel pix;
  UPixel * bgr, * yuv;
  UPixel * pcd = NULL;
  UPixel pixBlue(255, 0, 0);
  UPixel pixRed(0, 0, 255);
  UPixel pixBlueD(155, 0, 0);
  UPixel pixRedD(0, 0, 155);
  UPixel * ppix;
  unsigned int  r,c;
  float x = 0, y = 0, d;
  UHighGuiWindowHandle * wh;
  CvScalar red = CV_RGB(255, 0, 0);
  CvScalar blue = CV_RGB(0, 0, 255);
  CvScalar green = CV_RGB(0, 255, 0);
  CvScalar pale = CV_RGB(200, 200, 200);
  CvScalar black = CV_RGB(0, 0, 0);
  CvScalar white = CV_RGB(255, 255, 255);
  CvScalar gridColor;
  CvPoint pos;
  const int MWH = 512;
  CvFont font;
  const int MSL = 40;
  char s[MSL];
  int redFac = 4;
  const int MHC = 256;
  int hist[MHC];
  const int HIST_X = MWH-256-10;
  const int HIST_Y = 10 + 120 + 10 + 100;
  CvPoint p1, p2;
  int hy, hyc = 0, cl, gx, i;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
              0.5, 0.5, 0.0, 1, 8);
  if (imgUV == NULL)
  {
    imgUV = new UImage640();
    imgUV->setSize(MWH, MWH, 3, 8, "BGR");
  }
  if (imgUV != NULL)
  {
    imgUV->clear(255);
    wHnd = cvGetWindowHandle(wndUV);
    if (wHnd == NULL)
    { // vreate UV window
      cvNamedWindow(wndUV, CV_WINDOW_AUTOSIZE);
      cvShowImage(wndUV, imgUV->cvArr());
      cvCreateTrackbar(wndMin, wndUV, &globalThresholdMin, 256, on_trackbar);
      cvCreateTrackbar(wndMax, wndUV, &globalThresholdMax, 256, on_trackbar);
    }
    if (int(imgSourceUV) < windowCnt)
    {
      wh = window[imgSourceUV];
      imgsrc = wh->getImage();
    }
    if (imgsrc != NULL)
    { // zero histogram
      for (c = 0; int(c) < MHC; c++)
        hist[c] = 0;
      // make YUV image copy
      if (fuzzyWidth > 0)
      { // Make an YUV version
        if (imgsrc->width() > 320)
          imgyuv = new UImage640();
        else
          imgyuv = new UImage320();
        imgyuv->copy(imgsrc);
        if (globalColorDisplayType == CDT_YUV)
          imgyuv->toYUV(NULL);
        else if (globalColorDisplayType == CDT_CROMA)
          imgyuv->toCromaBGR(NULL);
        // do fuzzy classify on yuv
        doFuzzyClassify(imgyuv);
      }
      //
      if (globalColorDisplayType == CDT_YUV)
      { // paint coordinates in UV grid
        cvLine(imgUV->cvArr(), cvPoint(MWH/2, MWH/4),
                               cvPoint(MWH/2, MWH*3/4), red, 1,8,0);
        cvLine(imgUV->cvArr(), cvPoint(MWH/4,   MWH/2),
                               cvPoint(MWH*3/4, MWH/2), red, 1,8,0);
        cvPutText(imgUV->cvArr(), "U", cvPoint(MWH*3/4, MWH/2), &font, red);
        cvPutText(imgUV->cvArr(), "V", cvPoint(MWH/2, MWH/4), &font, red);
      }
      else if (globalColorDisplayType == CDT_CROMA)
      {
        snprintf(s, MSL, "Green");
        cvPutText(imgUV->cvArr(), s, cvPoint(25, 15), &font, green);
        snprintf(s, MSL, "Red");
        cvPutText(imgUV->cvArr(), s, cvPoint(MWH-60, MWH-10), &font, red);
        snprintf(s, MSL, "Blue");
        cvPutText(imgUV->cvArr(), s, cvPoint(10, MWH-10), &font, blue);
        cvLine(imgUV->cvArr(), cvPoint(0,0), cvPoint(0,MWH-1), green, 1,8,0);
        cvLine(imgUV->cvArr(), cvPoint(2,0), cvPoint(MWH-1, MWH-3), green, 1,8,0);
        cvLine(imgUV->cvArr(), cvPoint(0,MWH-1), cvPoint(MWH-1,MWH-1), green, 1,8,0);
        for (i = 1; i < 10; i++)
        { // make grid
          gx = i * MWH / 10;
          if (i == 5)
            gridColor = green;
          else
            gridColor = pale;
          cvLine(imgUV->cvArr(), cvPoint(gx, gx), cvPoint(gx,MWH-1), gridColor, 1,8,0);
          cvLine(imgUV->cvArr(), cvPoint(gx, gx), cvPoint(0,gx), gridColor, 1,8,0);
        }
      }
      // get reduction factor for top-left image
      redFac = imgsrc->width()/160;
      // do the task
      for (r = 0; r < imgsrc->height(); r++)
      {
        bgr = imgsrc->getLine(r);
        //
        for (c = 0; c < imgsrc->width(); c++)
        { // paint small copy in top-right corner
          // update histogram
          hy = roundi((bgr->p1 + bgr->p2 + bgr->p3) / 3.0);
          hist[hy]++;
          //
          if ((r % redFac) == 0)
          {
            if (c == 0)
              pcd = &imgUV->getLine(r/redFac + 10)[MWH - 160 - 10];
            if ((c % redFac) == 0)
            { // this pixel should be painted
              ppix = bgr;
              if ((fuzzy != NULL) and (fuzzyWidth > 0))
              { // paint fuzzy class
                yuv = imgyuv->getPixRef(r, c);
                cl = getPixelClass(*yuv, fuzzyClasses);
                // paint class 0 and 1 only
                switch (cl)
                {
                  case 0: ppix = &pixBlueD; break;
                  case 1: ppix = &pixRedD; break;
                  case 2:
                    if (fuzzyClasses > 3)
                      ppix = &pixBlue;
                    else
                      ppix = bgr;
                    break;
                  case 3: ppix = &pixRed; break;
                  // paint ROMA image as small image
                  default: ppix = yuv; break;
                }
              }
              // copy pixel to UV image
              *pcd++ = *ppix;
            }
          }
          if (globalColorDisplayType == CDT_YUV)
          { // use UV coordinates
            pix = bgr->asYUV(imgsrc->getColorType()); //UPixel::RGBtoYUV(bgr->r, bgr->g, bgr->b);
            if ((pix.y > globalThresholdMin) and
                (pix.y < globalThresholdMax))
            {
              pos.x = mini(255, maxi(0, (pix.u - 128) * 1 + 128)) * 2;
              pos.y = (255 - mini(255, maxi(0, (pix.v - 128) * 1 + 128))) * 2;
              pix.setYUVto(180, pix.u, pix.v, PIX_PLANES_BGR);
              pix.tone(imgUV->getPix(pos.y, pos.x), 50/redFac);
              imgUV->setPix(pos.y, pos.x, pix);
              imgUV->setPix(pos.y, pos.x + 1, pix);
              imgUV->setPix(pos.y + 1, pos.x, pix);
              imgUV->setPix(pos.y + 1, pos.x + 1, pix);
            }
          }
          else if (globalColorDisplayType == CDT_CROMA)
          { // convert to cromaticity
            d = float(bgr->p1 + bgr->p2 + bgr->p3);
            if ((d > (globalThresholdMin * 3)) and
                (d < (globalThresholdMax * 3)))
            {
              x = float(bgr->getRed(imgsrc->getColorType())) / d;
              y = float(bgr->getGreen(imgsrc->getColorType())) / d;
              pos.x = mini(MWH - 2, maxi(0, roundi(x * MWH)));
              pos.y = MWH - 2 - mini(MWH - 2, maxi(0, roundi(y * MWH)));
              pix.setRGBto(roundi(x * 256.0), roundi(y * 256.0),
                           roundi((1.0 - x - y) * 256.0), PIX_PLANES_BGR);
              pix.tone(imgUV->getPix(pos.y, pos.x), 40/redFac);
              //
              imgUV->setPix(pos.y, pos.x, pix);
              imgUV->setPix(pos.y, pos.x + 1, pix);
              imgUV->setPix(pos.y + 1, pos.x, pix);
              imgUV->setPix(pos.y + 1, pos.x + 1, pix);
            }
          }
          bgr++;
        }
      }
      // paint circle around mouse location
      pos.x = globalPixPoint2.x / redFac + MWH - 160 - 10;
      pos.y = globalPixPoint2.y / redFac + 10;
      cvCircle(imgUV->cvArr(), pos, 5, white, 3, 8, 0);
      cvCircle(imgUV->cvArr(), pos, 5, red, 1, 8, 0);
      pos.x = mini(imgsrc->width()-1, maxi(0, globalPixPoint2.x));
      pos.y = mini(imgsrc->height() - 1, maxi(0, globalPixPoint2.y));
      bgr = imgsrc->getPixRef(pos.y, pos.x);
      if (bgr != NULL)
      {
        pix = bgr->asYUV(imgsrc->getColorType());
        if (globalColorDisplayType == CDT_YUV)
        {
          pos.x = mini(255, maxi(0, (pix.u - 128) * 1 + 128)) * 2;
          pos.y = (255 - mini(255, maxi(0, (pix.v - 128) * 1 + 128))) * 2;
        }
        else if (globalColorDisplayType == CDT_CROMA)
        {
          d = float(bgr->p1 + bgr->p2 + bgr->p3);
          x = float(bgr->getRed(imgsrc->getColorType())) / d;
          y = float(bgr->getGreen(imgsrc->getColorType())) / d;
          pos.x = mini(MWH - 2, maxi(0, roundi(x * MWH)));
          pos.y = MWH - 2 - mini(MWH - 2, maxi(0, roundi(y * MWH)));
        }
        hyc = roundi((bgr->p1 + bgr->p2 + bgr->p3) / 3.0);
        cvCircle(imgUV->cvArr(), pos, 5, white, 3, 8, 0);
        cvCircle(imgUV->cvArr(), pos, 5, red, 1, 8, 0);
      }
      cvPutText(imgUV->cvArr(), imgsrc->name, cvPoint(140, 20), &font, blue);
      snprintf(s, MSL, "%d < r+g+b < %d",
               globalThresholdMin*3, globalThresholdMax*3);
      cvPutText(imgUV->cvArr(), s, cvPoint(140, 40), &font, blue);
      snprintf(s, MSL, "r,g,b = %d,%d,%d",
               bgr->getRed(imgsrc->getColorType()),
               bgr->getGreen(imgsrc->getColorType()),
               bgr->getBlue(imgsrc->getColorType()));
      cvPutText(imgUV->cvArr(), s, cvPoint(140, 60), &font, red);
      if (globalColorDisplayType == CDT_CROMA)
      {
        snprintf(s, MSL, "red,green = %.3f,%.3f", x, y);
        cvPutText(imgUV->cvArr(), s, cvPoint(140, 80), &font, red);
      }
      else
      {
        snprintf(s, MSL, "y,u,v = %d,%d,%d",
               pix.y, pix.u, pix.v);
        cvPutText(imgUV->cvArr(), s, cvPoint(140, 80), &font, red);
      }
      // mouse position
      snprintf(s, MSL, "at x,y = %d,%d",
               globalPixPoint2.x, globalPixPoint2.y);
      cvPutText(imgUV->cvArr(), s, cvPoint(140, 100), &font, red);
      // fuzzy class
      if (fuzzyWidth > 0)
      {
        snprintf(s, MSL, "class %d", getPixelClass(pix, fuzzyClasses));
        cvPutText(imgUV->cvArr(), s, cvPoint(140, 120), &font, red);
      }
      //
      // paint histogram
      p1.x = HIST_X;
      if (globalColorDisplayType == CDT_YUV)
        p1.y = MWH - 10;
      else
        p1.y = HIST_Y;
      redFac = 0;
      for (c = 0; int(c) < MHC; c++)
        redFac = maxi(redFac, hist[c]);
      redFac /= 100;
      // paint min and max lines in green
      p2.y = p1.y - 100;
      p1.x = HIST_X + globalThresholdMin;
      p2.x = p1.x;
      cvLine(imgUV->cvArr(), p1, p2, green, 1, 4, 0);
      p1.x = HIST_X + globalThresholdMax;
      p2.x = p1.x;
      cvLine(imgUV->cvArr(), p1, p2, green, 1, 4, 0);
      // paint histogram
      p1.x = HIST_X;
      for (c = 0; int(c) < MHC; c++)
      {
        p2.x = p1.x;
        p2.y = p1.y - mini(100, (hist[c] * 1) / redFac);
        if (int(c) == hyc)
          cvLine(imgUV->cvArr(), p1, p2, red, 1, 4, 0);
        else
          cvLine(imgUV->cvArr(), p1, p2, black, 1, 4, 0);
        p1.x++;
      }
      p1.x = HIST_X;
      p2.y = p1.y;
      p1.y -= 100;
      cvRectangle(imgUV->cvArr(), p1, p2, blue, 1, 4, 0);
      // fuzzy
      if ((fuzzyWidth > 1) and (fuzzy != NULL))
      { // paint result of fuzzy
        paintClustPixels(true,
                         imgyuv,
                         imgUV);
        for (cl = 0; cl < fuzzyClasses; cl++)
        {
          paintClustEllipse(true, fuzzy->getV(cl),
                            fuzzy->getF(cl),
                            imgUV, cl);
        }
      }
      cvShowImage(wndUV, imgUV->cvArr());
    }
  }
  if (imgyuv != NULL)
    delete imgyuv;
}

/////////////////////////////////////////////////////////////

void UClientFuncImgGui::paintClustPixels(bool isUV,
                                  UImage * imgsrc,
                                  UImage * imgUV
                                  )
{
  UPixel * yuv;
  UFuzzyPixel pe;
  int cl, r, c;
  UPixel pixBlue(255, 0, 0);
  UPixel pixRed(0, 0, 255);
  UPixel pixBlueD(155, 0, 0);
  UPixel pixRedD(0, 0, 155);
  CvPoint pos = {0,0};;
  float x,y;
  const int MWH = 512;
  //
  for (r = 0; r < int(imgsrc->height()); r ++)
  {
    yuv = imgsrc->getLine(r);
    for (c = 0; c < int(imgsrc->width()); c++)
    {
      cl = getPixelClass(*yuv, fuzzyClasses);
      if ((cl != 2) or (fuzzyClasses > 3))
      { // paint all pixels that belong to cluster 0 and 1
        if (globalColorDisplayType == CDT_YUV)
        { // get position directly from U and V
          pos.x = mini(255, maxi(0, (yuv->u - 128) * 1 + 128)) * 2;
          pos.y = (255 - mini(255, maxi(0, (yuv->v - 128) * 1 + 128))) * 2;
        }
        else if (globalColorDisplayType == CDT_CROMA)
        { // convert to cromaticity
          x = float(yuv->p3) / 256.0;
          y = float(yuv->p2) / 256.0;
          pos.x = mini(MWH - 2, maxi(0, roundi(x * MWH)));
          pos.y = MWH - 2 - mini(MWH - 2, maxi(0, roundi(y * MWH)));
        }
        switch (cl)
        {
          case 0: imgUV->setPix(pos.y, pos.x, pixBlueD); break;
          case 1: imgUV->setPix(pos.y+1, pos.x, pixRedD); break;
          case 2: imgUV->setPix(pos.y, pos.x+1, pixBlue); break;
          case 3: imgUV->setPix(pos.y+1, pos.x+1, pixRed); break;
          default: imgUV->setPix(pos.y+1, pos.x+1, pixRed); break;
        }
      }
      yuv++;
    }
  }
}

////////////////////////////////////////////////////////////

void UClientFuncImgGui::paintClustEllipse(
                                     bool isUV,
                                 UMatrix * mV,
                                 UMatrix * mQ,
                                 UImage * img,
                                 int clust
                                             )
{ // paint error ellipse with center point
  // x,y if within [0-w, 0-h]
  UMatrix4 Q(2,2);
  UMatrix4 E(2,1);
  UMatrix4 eig(2, 1); // eigenvalues
  UMatrix4 eigVec(2,2);// eigenvectors (columns)
  bool isComplex; // matrix has complex eigenvalues
  double p, a, b;
  int r, c;
  // light blue
  CvScalar col = CV_RGB(50, 0, 0);
  //CvScalar red = CV_RGB(100, 100, 255);
  CvPoint pos;
  CvSize sz;
  //
  switch (clust)
  { // paint in light red
    case 0: col = CV_RGB(50, 0, 0); break;
    case 1: col = CV_RGB(0, 0, 50); break;
    case 2: col = CV_RGB(100, 0, 0); break;
    case 3: col = CV_RGB(0, 0, 100); break;
    default: col = CV_RGB(0, 0, 0); break;
  }
  if (isUV or true)
  { // move to reduced matrix/vector.
    for (r = 1; r < int(mV->rows()); r++)
      E.setRC(r-1, 0, mV->get(r,0));
    for (r = 1; r < int(mQ->rows()); r++)
      for (c = 1; c < int(mQ->cols()); c++)
        Q.setRC(r-1, c-1, mQ->get(r,c));
  }
  // get center position
  pos.x = mini(255, maxi(0, (roundi(E.get(0,0)) - 128) * 1 + 128)) * 2;
  pos.y = (255 - mini(255, maxi(0, (roundi(E.get(1,0)) - 128) * 1 + 128))) * 2;
  // get ellipse data
  eig = Q.eig2x2(&isComplex, &eigVec);
  if (fabs(eig.get(1)) < fabs(eig.get(0)))
  { // first eigenvalue is the larger
    p = atan2(eigVec.get(1,0),eigVec.get(0,0));
    a = sqrt(eig.get(0)); // first eigenvalue
    b = sqrt(eig.get(1)); // second eigenvalue
  }
  else
  { // second eigenvalue is the larger
    p = atan2(eigVec.get(1,1),eigVec.get(0,1));
    a = sqrt(eig.get(1));
    b = sqrt(eig.get(0));
  }
  sz.width = roundi(a*3.0);
  sz.height = roundi(b*3.0);
  cvEllipse( img->cvArr(), pos, sz, p * 180.0 / M_PI,
                  0.0, 360.0, col, 1, 4, 0);
}


/////////////////////////////////////////////////////////////

bool UClientFuncImgGui::doFuzzyClassify(UImage * imgsrc)
{
  int r, c, n, sr, sc;
  bool result = false;
  // window elements row and column
  const int MER = 30;
  const int MEC = 30;
  UFuzzyPixel * emw[MER * MEC];
  int emwCnt;
  // number of sample around mouse
  const int MES = 1000;
  UFuzzyPixel * ems1[MES];
  UFuzzyPixel * ems2[MES];
  UFuzzyPixel * ems3[MES];
  UFuzzyPixel * ems4[MES];
  int ems1Cnt = 0;
  int ems2Cnt = 0;
  int ems3Cnt = 0;
  int ems4Cnt = 0;
  int cl;
  //
  UPixel * pix;
  UFuzzyPixel * fz;
  UFuzzyPixel ** pfz;
  //
  if (fuzzy == NULL)
    fuzzy = new UFuzzySplit();
  fuzzy->clear();
  // allocate elements
  // get image pixels (all over)
  sr = imgsrc->height() / (MER + 1);
  sc = imgsrc->width() / (MEC + 1);
  n = 0;
  if (fuzzyClasses > 2)
  { // do also background
    for (r = 0; r < MER; r++)
    {
      for (c = 0; c < MEC; c++)
      {
        pix = imgsrc->getPixRef(r * sr + sr/2, c * sc + sc/2);
        fz = new UFuzzyPixel();
        emw[n] = fz;
        fz->setPixel(pix);
        n++;
      }
    }
  }
  emwCnt = n;
  // get sample pixels around mouse
  ems1Cnt = 0;
  ems2Cnt = 0;
  ems3Cnt = 0;
  ems4Cnt = 0;
  if (fuzzyLine < 0)
  { // use the 2 mouse points
    for (cl = 0; cl < 2; cl++)
    {
      if (sqr(fuzzyWidth * 2 + 1)*2 > MES)
        fuzzyWidth = int(sqrt(MES/2)-1)/2;
      if (cl == 0)
      { // cluster 1
        sr = globalPixPoint2.y - fuzzyWidth;
        sc = globalPixPoint2.x - fuzzyWidth;
      }
      else
      { // cluster 2 (shift-click)
        sr = globalPixPointS.y - fuzzyWidth;
        sc = globalPixPointS.x - fuzzyWidth;
      }
      if (sr < 0)
        sr = 0;
      else if (sr >= (int(imgsrc->height()) - (fuzzyWidth * 2 + 1)))
        sr = imgsrc->height() - (fuzzyWidth * 2 + 2);
      if (sc < 0)
        sc = 0;
      else if (sc >= (int(imgsrc->width()) - (fuzzyWidth * 2 + 1)))
        sc = imgsrc->width() - (fuzzyWidth * 2 + 2);
      for (r = 0; r < 2 * fuzzyWidth + 1; r++)
      {
        pix = imgsrc->getPixRef(sr + r, sc);
        for (c = 0; c < 2 * fuzzyWidth + 1; c++)
        {
          if ((pix->y > 30) and (pix->y < 220))
          {
            fz = new UFuzzyPixel();
            fz->setPixel(pix);
            if (cl == 0)
              ems1[ems1Cnt++] = fz;
            else
              ems2[ems2Cnt++] = fz;
          }
          pix++;
        }
      }
    }
  }
  else
  { // seet from image line
    sr = mini(maxi(1, fuzzyLine), imgsrc->height()-1);
    sc = mini(maxi(0, fuzzyCol) , imgsrc->width());
    for (r = -1; r < 2; r++)
    {
      pix = imgsrc->getLine(sr + r);
      for (c = 0; c < (int)imgsrc->width(); c++)
      {
        if ((pix->y > 30) and (pix->y < 240))
        {
          fz = new UFuzzyPixel();
          fz->setPixel(pix);
          if (c < sc)
          {
            if (fuzzyClasses <=3)
              ems1[ems1Cnt++] = fz;
            else
            { // divide rough into bright and dark groups
              if (pix->y > 100)
                ems1[ems1Cnt++] = fz;
              else
                ems3[ems3Cnt++] = fz;
            }
          }
          else
          {
            if (fuzzyClasses <=3)
              ems2[ems2Cnt++] = fz;
            else
            { // divide road into 2 classes
              if (pix->y > 140)
                ems2[ems2Cnt++] = fz;
              else
                ems4[ems4Cnt++] = fz;
            }
          }
        }
        pix++;
      }
    }
  }
  // any values available?
  printf("Class 0 is based on %d pixels (rough bright)\n", ems1Cnt);
  printf("Class 1 is based on %d pixels (smooth bright)\n", ems2Cnt);
  if (fuzzyClasses == 3)
    printf("Class 2 is based on %d pixels (background)\n", ems3Cnt);
  else
  {
    printf("Class 2 is based on %d pixels (rough dark)\n", ems3Cnt);
    if (ems3Cnt > 0)
      ems3[0]->print("ems3 element 0", 1);
    printf("Class 3 is based on %d pixels (smooth dark)\n", ems4Cnt);
    if (ems4Cnt > 0)
      ems4[0]->print("ems4 element 0", 1);
  }
  //
  if (true) // ((ems1Cnt + ems3Cnt) > 5) and ((ems2Cnt + ems4Cnt) > 10))
  {
    // add elements to fuzzy splitter
    if (fuzzyClasses > 2)
    { // add background class (class 2)
      pfz = emw;
      for (n = 0; n < emwCnt; n++)
        fuzzy->addElement(*pfz++);
      fuzzy->initFromValues(2, 0, emwCnt);
    }
    // ... then sampled values base and shifted
    for (n = 0; n < ems1Cnt; n++)
      fuzzy->addElement(ems1[n]);
    for (n = 0; n < ems2Cnt; n++)
      fuzzy->addElement(ems2[n]);
    for (n = 0; n < ems3Cnt; n++)
      fuzzy->addElement(ems2[n]);
    for (n = 0; n < ems4Cnt; n++)
      fuzzy->addElement(ems2[n]);
    // init class 0 (mouse - no shift)
    if (ems1Cnt > 5)
      fuzzy->initFromValues(0, emwCnt, ems1Cnt);
    // init class 1 (mouse - shift)
    if (ems2Cnt > 5)
      fuzzy->initFromValues(1, emwCnt + ems1Cnt, ems2Cnt);
    if (ems3Cnt > 5)
      fuzzy->initFromValues(2, emwCnt + ems1Cnt + ems2Cnt, ems3Cnt);
    if (ems4Cnt > 5)
      fuzzy->initFromValues(3, emwCnt + ems1Cnt + ems2Cnt + ems3Cnt, ems4Cnt);
    // then find optimal split
    // do the required iterations
    fuzzy->classify(fuzzyClasses, 0.01, fuzzyIter + 1, false);
    // show result count
    fuzzy->countMembers();
    for (n = 0; n < fuzzyClasses; n++)
      printf("Cluster %d has %d members\n",
            n, fuzzy->getMembCount(n));
  }
  else
    printf("Not enough valid values\n");
  // clear invalid UFuzzyElement pointers, but
  // maintain result matrices in UFuzzySplit
  fuzzy->clear();
  // deallocate fuzzy elements
  for (n = 0; n < emwCnt; n++)
    delete emw[n];
  for (n = 0; n < ems1Cnt; n++)
    delete ems1[n];
  for (n = 0; n < ems2Cnt; n++)
    delete ems2[n];
  return result;
}

/////////////////////////////////////////////////////////////

void UClientFuncImgGui::setUVSource(
             const char * captName,
             ColorDisplayType type)
{
  int n;
  UHighGuiWindowHandle * wn = NULL;
  UImage * img;
  //
  for (n = 0; n < windowCnt; n++)
  {
    wn = window[n];
    img = wn->getImage();
    if (img != NULL)
      if (strstr(img->name, captName) != NULL)
        break;
    wn = NULL;
  }
  if (wn == NULL)
    imgSourceUV = 0;
  else
    imgSourceUV = n;
  globalColorDisplayType = type;
}

/////////////////////////////////////////////////////////////

UHighGuiWindowHandle * UClientFuncImgGui::getImageSource(const char * capt)
{
  UHighGuiWindowHandle * result = NULL;
  // set window
  setUVSource(capt, CDT_NONE);
  // return handle
  result = window[imgSourceUV];
  //
  return result;
}

/////////////////////////////////////////////////////////////

bool UClientFuncImgGui::saveImage(const char * capt, const char * noshow)
{
  int n;
  UHighGuiWindowHandle * wn = NULL;
  UImage * img = NULL;
  const int MSL = 300;
  char s[MSL];
  const int MTL = 110;
  char st[MTL];
  char sw[MTL] = "_";
  bool result = false;
  //
  for (n = 0; n < windowCnt; n++)
  {
    wn = window[n];
    img = wn->getImage();
    if (img != NULL)
      if (strstr(img->name, capt) != NULL)
        break;
    wn = NULL;
  }
  if (wn != NULL)
  {
    img = wn->getImage();
    snprintf(sw, MTL, "%s", wn->cvWndName);
  }
  else if (imgUV != NULL)
  {
    img = imgUV;
    if ((int)imgSourceUV < windowCnt)
    {
      snprintf(sw, MTL, "col_%s", window[imgSourceUV]->cvWndName);
      img->imgTime = window[imgSourceUV]->getImage()->imgTime;
    }
  }
  if (img != NULL)
  { // there is an image to save
    if (strlen(sw) < 19)
      // short image name, add date and time
      snprintf(s, MSL, "%s/%s-%s.png", imagePath, sw, img->imgTime.getForFilename(st, true));
    else
      // assume name is unique enough
      snprintf(s, MSL, "%s/%s.png", imagePath, sw);
    if (cvSaveImage(s, img->cvArr()))
    {
      printf("Saved image to %s\n", s);
      result = true;
    }
    else
      printf("Could not save to %s\n", s);
  }
  if (strcasecmp(capt, "false") == 0)
    saveOnMatch = false;
  else
  { // save match string
    saveOnMatch = true;
    // save caption match string for new images
    strncpy(saveOnMatchStr, capt, MAX_MATCH_STR_LNG);
    saveOnMatchStr[MAX_MATCH_STR_LNG-1] = '\0';
    // test if new images that are to be saved should be displayed too
    saveOnMatchAndShow = (strcasecmp(noshow, "noshow") != 0);
  }
  //
  return result;
}

///////////////////////////////////////////////////

UImage * UClientFuncImgGui::getImage(const char * capt)
{
  int n;
  UHighGuiWindowHandle * wn = NULL;
  UImage * img = NULL;
  //
  for (n = 0; n < windowCnt; n++)
  {
    wn = window[n];
    img = wn->getImage();
    if (img != NULL)
      if (strstr(img->name, capt) != NULL)
        break;
    img = NULL;
  }
  return img;
}

///////////////////////////////////////////////////

UPixel UClientFuncImgGui::getCroma(UPixel * src, int sourceFormat)
{
  UPixel bgr;
  UPixel pix;
  int sum;
  //
  if (sourceFormat != PIX_PLANES_BGR)
    bgr = src->asBGR(sourceFormat);
  else
    bgr = *src;
  //
  sum = bgr.p1 + bgr.p2 + bgr.p3;
  pix.p1 = (255 * bgr.p3) / sum; // red
  pix.p2 = (255 * bgr.p2) / sum; // green
  pix.p3 = (255 * bgr.p1) / sum; // blue
  //
  return pix;
}

//////////////////////////////////////////////////////

int UClientFuncImgGui::getPixelClass(UPixel src, int clustCnt)
{
  UFuzzyPixel pe;
  int cl;
  UPixel pix;
  //
  if (globalColorDisplayType == CDT_CROMA)
    // assumes 'src' is in BGR format p1=b, p2=g, p3=r
    pix = getCroma(&src, PIX_PLANES_BGR);
  else if (globalColorDisplayType == CDT_YUV)
    // assumes yuv is in yuv format
    pix = src;
  // make classifier element from pixel
  pe.setPixel(&pix);
  // get class
  cl = fuzzy->updateElement(&pe, clustCnt);
  //
  return cl;
}

////////////////////////////////////////////////////

void UClientFuncImgGui::setFuzzyWidth(int value)
{
  fuzzyWidth = value;
}

////////////////////////////////////////////////////

void UClientFuncImgGui::setFuzzyLine(int row, int col)
{
  fuzzyLine = row;
  fuzzyCol = col;
  globalUvRedisplay = true;
}

///////////////////////////////////////////////////

void UClientFuncImgGui::clear()
{
  int i;
  //
  if (imgUV != NULL)
  {
    delete imgUV;
    imgUV = NULL;
  }
  for (i = 0; i < windowCnt; i++)
  {
    delete window[i];
    window[i] = NULL;
    imgSourceUV = 0;
  }
  windowCnt = 0;
}

