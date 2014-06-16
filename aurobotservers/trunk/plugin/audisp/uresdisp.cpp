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

//#include <SDL.h>
//#include <SDL_gfxPrimitives.h>
#ifdef OPENCV2
#include <highgui/highgui_c.h>
#else
#include <opencv/highgui.h>
#include <pcl-1.6/pcl/impl/point_types.hpp>
#endif
#include <ugen4/uimage.h>
#include <urob4/uimagepool.h>
#include <urob4/uvarcalc.h>
#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
//#include "../../plugin/aulaserif/ureslaserifsf.h"

#include "uresdisp.h"

//#undef USE_HIGHGUI

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

/**
Threshold for not showing UV value */
//int  globalThresholdMin = 40;
//int  globalThresholdMax = 200;
/**
Flag for uvThreshold is changed */
bool globalUvRedisplay = true;
/**
Flag for changing scale on nav display */
bool globalNavRedisplay = false;
/**
Flag for type of display that is to be performed */
//UClientFuncImgGui::ColorDisplayType
//    globalColorDisplayType = UClientFuncImgGui::CDT_NONE;
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
      if ((flags & CV_EVENT_FLAG_CTRLKEY) == 0)
        globalUvRedisplay = true;
      else
        // ctrl-key pressed - is a scale command
        globalNavRedisplay = true;
      globalMouseDrag = false;
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


////////////////////////////////////////////////////

void  on_trackbar(int value)
{ // trackbar on UV image
  globalUvRedisplay = true;
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

UHighGuiWindowHandle::UHighGuiWindowHandle()
{
  img = NULL;
  strncpy(cvWndName, "none", MAX_CV_WINDOW_NAME_SIZE);
}

UHighGuiWindowHandle::~UHighGuiWindowHandle()
{
  removeImage();
  img = NULL;
}

//////////////////////////////////////////////////////

bool UHighGuiWindowHandle::setImage(UImage * toImg, int imgPoolNum)
{
  img = toImg;
  snprintf(cvWndName, MAX_CV_WINDOW_NAME_SIZE, "img%d", imgPoolNum);
  imgNum = imgPoolNum;
  return (img != NULL);
}

///////////////////////////////////////////////////////

void UHighGuiWindowHandle::removeImage()
{
#ifdef USE_HIGHGUI
    void * wHnd;
    //
    wHnd = cvGetWindowHandle(cvWndName);
    if (wHnd != NULL)
    {
      cvDestroyWindow(cvWndName);
      cvWaitKey(70); // parameter is service time in ms
    }
#endif
}

///////////////////////////////////////////////////////

bool UHighGuiWindowHandle::showImage(bool useHghgui)
{
  bool isTime = false;
#ifdef USE_HIGHGUI
    void * wHnd;
    bool isOK = true;
    //
    if (img != NULL)
      isTime = (img->used == 0 and img->width() > 0 and img->height() > 0);
    if (isTime)
    { // show image
      if (img->tryLock())
      {
        UImage * tmp = img;
        if (not img->isBGR())
        {
          // debug
          //img->savePNG("camtest-bggr.png");
          // debug end
          UImage ** imgBuf;
          imgBuf = img->getConvertBuffer();
          if (*imgBuf == NULL)
            *imgBuf = new UImage();
          tmp = *imgBuf;
          isOK = img->toBGR(tmp);
          // debug
  /*        printf("UHighGuiWindowHandle::showImage saving debug images\n");
          tmp->savePNG("camtest-rgb.png");
          img->savePNG("camtest-420.png");*/
          // debug end
        }
        if (isOK)
        {
          if (useHghgui)
          {
            wHnd = cvGetWindowHandle(cvWndName);
            if (wHnd == NULL)
            { // (re)create window
              cvNamedWindow(cvWndName, CV_WINDOW_AUTOSIZE);
              cvSetMouseCallback(cvWndName, on_mouse);
            }
            cvShowImage(cvWndName, tmp->cvArr());
            // allow a little time to display on screen
            // debug
            //fprintf(stderr, "wait\n");
            // debug end
            //cvWaitKey(70); // parameter is service time in ms
            // debug
            //fprintf(stderr, "waited\n");
            // debug end
          }
          img->used = true;
        }
        else
          printf("UHighGuiWindowHandle::showImage: failed to show image %s\n", cvWndName);
  //       if (tmp != img)
  //         delete tmp;
        img->unlock();
      }
    }
#endif
  return isTime;
}

//////////////////////////////////////////////////////

bool UHighGuiWindowHandle::isTime(UTime dueTime)
{
  bool result = false;
  //
  if (img != NULL)
    result = (img->getUpdatedTime() >= dueTime);
  return result;
}


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

void UResDisp::UResDispInit()
{
/*  setResID( getResID());
  resVersion = getResVersion();*/
  createVarSpace(10, 0, 0, "X-display variables", false);
  varRoot = NULL;
  imgPool = NULL;
  poseHist = NULL;
  poseUtm = NULL;
  poseMap = NULL;
  createBaseVar();
  updListCnt = 0;
  imgSourceCROMA = -1;
  imgSourceUV = -1;
  newDataLaser = false;
  newDataNav = false;
  newDataLaserAt.clear();
}

////////////////////////////////////////////

UResDisp::~UResDisp()
{
  stop(true);
  if (imgBuf != NULL)
    delete imgBuf;
}

///////////////////////////////////////////

void UResDisp::createBaseVar()
{
  varRunning = addVar("running", 0.0, "d", "(r) Is display loop (thread) running");
#ifdef USE_HIGHGUI
  varUseHighGUI = addVar("highGUI", 1.0, "d", "(rw) use OPENCV highgui to display images (0=no) - conflicts with gstream");
#else
  varUseHighGUI = addVar("highGUI", 0.0, "d", "(rw) use OPENCV highgui to display images (0=no) - conflicts with gstream");
#endif
  addVar("paintGrid", 0.0, "d", "(rw) Paint the odometry grid");
  addVar("paintSpeed", 0.0, "d", "(rw) Paint robot speed");
  varNavImg = addVar("NavImage", 97.0, "d", "(rw) Image number in image pool for robot nav display");
  varAutoHereNow = addVar("autoHereNow", 1.0, "d", "(rw) Should coordinate systems be aligned at current robot pose");
  varUVCROMAminY = addVar("uvMinY", 40.0, "d", "(rw) Do not show coloeur for intensity below this level [0..255]");
  varUVCROMAmaxY = addVar("uvMaxY", 200.0, "d", "(rw) Do not show coloeur for intensity above this level [0..255]");
  varBold = addVar("bold", 0.0, "d", "(rw) Paint drawn images more bold");
  varRangeRings = addVar("rangeRings", 8.0, "d", "(rw) Number of range rings to paint");
  //
  addMethod("newData", "sd", "Called to trigger repaint of new data. The "
      "string parameter is the data type (pt. img, laser, poly or nav). The number parameter"
          "is an associated number (e.g. image pool number).");
  addMethod("resize", "d", "The image with this image pool number is about to "
                "be resized, and should be removed from screen now (openCV protection issue)");
}

//////////////////////////////////////////////////////

const char * UResDisp::print(const char * preString, char * buff, int buffCnt)
{
  int m = 0;
  char * p1;
  //
  if (buff != NULL)
  {
    p1 = buff;
    snprintf(p1, buffCnt - m, "%s Module to display primarly laser scanner data\n", preString);
  }
  return buff;
}

/////////////////////////////////////////////////////

bool UResDisp::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
  if (poseHist == NULL)
  {
    strncpy(p1, UResPoseHist::getOdoPoseID(), missingTheseCnt);
    n += strlen(p1);
    p1 = &missingThese[n];
    result = false;
  }
  if (varRoot == NULL)
  {
    strncpy(p1, UResVarPool::getResClassID(), missingTheseCnt);
    n += strlen(p1);
    p1 = &missingThese[n];
    result = false;
  }
  if (imgPool == NULL)
  {
    strncpy(p1, UImagePool::getResClassID(), missingTheseCnt);
    n += strlen(p1);
    p1 = &missingThese[n];
    result = false;
  }
  result &= UResVarPool::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

//////////////////////////////////////////////////////////////////

bool UResDisp::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  //
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // delete any local
    if (remove)
      poseHist = NULL;
    else if (poseHist != resource)
      poseHist = (UResPoseHist *)resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getUtmPoseID()))
  { // delete any local
    if (remove)
      poseUtm = NULL;
    else if (poseUtm != resource)
      poseUtm = (UResPoseHist *)resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getMapPoseID()))
  { // delete any local
    if (remove)
      poseMap = NULL;
    else if (poseMap != resource)
      poseMap = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResVarPool::getResID()))
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
  else
    result = false;
  // the navigation painter may need these resources too
  result |= navPaint.setResource(resource, remove);
  //
  return result;
}

////////////////////////////////////////////////////////////////////

bool UResDisp::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  int imgPoolNum;
  // evaluate standard functions
  if ((strcasecmp(name, "newData") == 0) and (strcmp(paramOrder, "sd") == 0))
  {
    if (strcasecmp(strings[0], "img") == 0)
    {
      imgPoolNum = int(pars[0]);
      // add watch of this image
      addImagePoolImg(imgPoolNum);
      if (not newDataImgPool)
      { // need a redisplay of image pool
        newDataImgPoolAt.Now();
        newDataImgPool = true;
      }
    }
    else if (strcasecmp(strings[0], "laser") == 0)
    {
      if (not newDataLaser)
      { // laser data needs redisplay
        newDataLaserAt.Now();
        newDataLaser = true;
      }
    }
    else
    { // all other types of navigation data update
      if (not newDataNav)
      { // navigation data needs redisplay
        newDataNavAt.Now();
        newDataNav = true;
      }
    }
    // set return values
    if (value != NULL)
      *value = 1;
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
    //
  }
  else if ((strcasecmp(name, "resize") == 0) and (strcmp(paramOrder, "d") == 0))
  { // resize pool image (delete image) and wait for update
    imgPoolNum = int(pars[0]);
    dispSync.lock();
    removeImagePoolImg(imgPoolNum);
    dispSync.unlock();
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////

void *startSdlDisp(void *ptr)
{ // called by create_thread
  UResDisp * obj;
  // pointer is an UResSmrCtl instance
  obj = (UResDisp *) ptr;
  // run main loop
  obj->run();
  return NULL;
}

/////////////////////////////////////////////

bool UResDisp::start()
{
  bool result = false;
  pthread_attr_t  thConAttr;
  int err;
  //
  if (not varRunning->getBool())
  {
    stopFlag = false;
    // Starts socket client thread 'runSockClient'
    pthread_attr_init(&thConAttr);
    // create socket client thread
    err = pthread_create(&thDisp, &thConAttr, &startSdlDisp, (void *)this);
    result = (err == 0);
    pthread_attr_destroy(&thConAttr);
  }
  //
  return result;
}

/////////////////////////////////////////////

void UResDisp::stop(bool andWait)
{
  stopFlag = true;
  if (varRunning->getBool())
  {
    if (andWait)
    {
      printf("UResSmrCtl:: stopping ctl loop ...");
      pthread_join(thDisp, NULL);
      printf("[OK]\n");
    }
    else
      Wait(0.05);
  }
}

/////////////////////////////////////////////

void UResDisp::run()
{
//  Uint8* keys;
//  UTime tImg; //, tLaser, tNav;
  const double WAIT_TIME_BEFORE_REPAINT = 0.15;
  bool addedNavImg = false;
  int i = 0;
  int poolImg;
  bool redraw = false;
//  bool moving;
//  CvPoint oldPos;
  //
  varRunning->setValued(true, 0);
  while (not stopFlag)
  { // top in a condition with no data use
    dispSync.lock();
    // Repaint all image-pool images
    doRepaintImages();
    if ((i % 10) == 0)
      test4NewImages();
    // test if other data types is updated
    if (newDataLaser and newDataLaserAt.getTimePassed() > WAIT_TIME_BEFORE_REPAINT)
      redraw = true;
    if (newDataNav and newDataNavAt.getTimePassed() > WAIT_TIME_BEFORE_REPAINT)
      redraw = true;
    if (redraw or globalNavRedisplay)
    {
      if (globalNavRedisplay)
      {
        mouseRescale(globalMouseFlag & CV_EVENT_FLAG_SHIFTKEY);
        globalNavRedisplay = false;
      }
      poolImg = varNavImg->getInt();
      newDataLaser = false;
      newDataNav = false;
      if (varAutoHereNow->getBool())
        navPaint.setRefSystemsHere();
      navPaint.rangeRingCnt = varRangeRings->getInt();
      //
      // set error mode to parent, to not fail completely on paint errors
      // #define CV_ErrModeLeaf 0
      // #define CV_ErrModeParent 1
      // #define CV_ErrModeSilent 2
//      cvSetErrMode(CV_ErrModeParent);
      //
      navPaint.paint(poolImg);
      // test for errors
      int e = cvGetErrStatus();
      if (e != 0)
      { // does not work with the openCV 2.0 wrapper
        printf("UResDisp:navpaint(): openCV error %d\n", e);
      }
      //
      if (not addedNavImg)
      { // add as displayed image
        addImagePoolImg(poolImg);
        addedNavImg = true;
      }
      redraw = false;
    }
/*    else if ((globalMouseEvent & CV_EVENT_MOUSEMOVE) or moving)
    {
      UImage * img = navPaint.getImg();
      CvScalar white = CV_RGB(255, 255, 255);
      CvScalar orange = CV_RGB(100, 40, 0);
      if (img != NULL)
      {
        if (not moving)
          moving = true;
        else
          cvRectangle(img->cvArr(), globalPixPoint1, oldPos, white, 1, 4, 0);
        if (globalMouseEvent & CV_EVENT_MOUSEMOVE)
        {
          oldPos = globalPixPoint3;
          cvRectangle(img->cvArr(), globalPixPoint1, globalPixPoint3, orange, 1, 4, 0);
        }
        else
          moving = false;
        img->used = 0;
      }
    }*/
#ifdef USE_HIGHGUI
    // update to screen too
//     if (varUseHighGUI->getBool())
//       cvWaitKey(15); // parameter is service time in ms
#endif
    dispSync.unlock();
    Wait(0.05);
    i++;
  }
  varRunning->setValued(false, 0);
}

//////////////////////////////////////////////

void UResDisp::doRepaintImages()
{
  int i;
  UHighGuiWindowHandle * ud;
  bool isTime;
  bool resetUVredisp = false;
  //
  ud = updList;
  for (i = 0; i < updListCnt; i++)
  {
    isTime = ud->showImage(varUseHighGUI->getBool());
    if (imgSourceUV == ud->getImageNum())
    { // may be source for UV display
      if (isTime or globalUvRedisplay)
        showUVImage(false);
      resetUVredisp = true;
    }
    if (imgSourceCROMA == ud->getImageNum())
    { // may be source for UV display
      if (isTime or globalUvRedisplay)
        showUVImage(true);
      resetUVredisp = true;
    }
    ud++;
  }
  if (resetUVredisp)
    globalUvRedisplay = false;
}

//////////////////////////////////////////////

void UResDisp::test4NewImages()
{
  int i;
  UImage * img;
  //
  if (imgPool != NULL)
  {
    for (i = 0; i < imgPool->getImageCnt(); i++)
    {
      img = imgPool->getImage(i, false);
      if (img != NULL)
      { // may be source for UV display
        if (img->used == 0)
          addImagePoolImg(i);
      }
    }
  }
}

////////////////////////////////////////////////////

bool UResDisp::loadImgToPool(const char * imgName, const int imgPoolNum)
{
  bool result = imgPool != NULL;
  UImage * img;
  const char * p2;
  //
  img = imgPool->getImage(imgPoolNum, true, 480, 640, 3);
  p2 = &imgName[strlen(imgName) - 4];
  if (strcasecmp(p2, ".png") == 0)
    img->loadPNG(imgName);
  else
    img->loadBMP(imgName);
  img->updated();
  addImagePoolImg(imgPoolNum);
  //
  return result;
}

/////////////////////////////////////////////////


void UResDisp::showUVImage(bool asCROMA)
{
  UImage * imgsrc = NULL;
  UPixel pix;
  UPixel * bgr;
  UPixel * pcd = NULL;
  UPixel pixBlue(255, 0, 0);
  UPixel pixRed(0, 0, 255);
  UPixel pixBlueD(155, 0, 0);
  UPixel pixRedD(0, 0, 155);
  UPixel * ppix;
  unsigned int  r,c;
  float x, y, d;
  CvScalar red = CV_RGB(255, 0, 0);
  CvScalar blue = CV_RGB(0, 0, 255);
  CvScalar green = CV_RGB(0, 255, 0);
  CvScalar black = CV_RGB(0, 0, 0);
  CvScalar white = CV_RGB(255, 255, 255);
  CvPoint pos;
  const int MWH = 512;
  const int MSL = 40;
  char s[MSL];
  int redFac = 4;
  const int MHC = 256;
  int hist[MHC];
  const int HIST_X = MWH-256-10;
  const int HIST_Y = 10 + 120 + 10 + 100;
  CvPoint p1, p2;
  int hy, hyc = 0;
  UImage * imgUV = NULL;
  CvFont font;
  int resultImg;
  bool asBold;
  int lw = 1;
  bool isOn = false;
  //
  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
               0.5, 0.5, 0.0, 1, 8);
  asBold = varBold->getBool();
  if (asBold)
    lw = 2;
  if (imgPool != NULL)
  {
    if (asCROMA)
    { // CROMA image uses pool image 96 (97 is for laser, 98 is temp croma image
      resultImg = 96;
      imgsrc = imgPool->getImage(imgSourceCROMA, false);
    }
    else
    { // UV-image is painted to image pool image 99
      resultImg = 99;
      imgsrc = imgPool->getImage(imgSourceUV, false);
    }
    if (imgsrc != NULL)
    { // get the pool-image with the result
      imgUV = imgPool->getImage(resultImg, false);
      if (imgUV == NULL)
      { // no image yet, so make one
        imgUV = imgPool->getImage(resultImg, true, MWH, MWH, 3, 8);
        imgUV->setSize(MWH, MWH, 3, 8, "BGR");
        // add image to images that are to be updated
        addImagePoolImg(resultImg);
      }
    }
  }
  if (imgUV != NULL)
  {
    globalThresholdMin = varUVCROMAminY->getInt();
    globalThresholdMax = varUVCROMAmaxY->getInt();
    imgUV->clear(255);
/*    wHnd = cvGetWindowHandle(wndUV);
    if (wHnd == NULL)
    { // vreate UV window
      cvNamedWindow(wndUV, CV_WINDOW_AUTOSIZE);
      cvShowImage(wndUV, imgUV->cvArr());
      cvCreateTrackbar(wndMin, wndUV, &globalThresholdMin, 256, on_trackbar);
      cvCreateTrackbar(wndMax, wndUV, &globalThresholdMax, 256, on_trackbar);
    }*/
    if (imgsrc != NULL)
    { // zero histogram
      imgsrc->lock();
      // is image in a usable format
      if (not (imgsrc->isBGR() or imgsrc->isRGB() or imgsrc->isBW() or imgsrc->isYUV()))
      { // a conversion is needed
        if (imgBuf == NULL)
          imgBuf = new UImage();
        imgsrc->toRGB(imgBuf);
        imgsrc->unlock();
        imgsrc = imgBuf;
        imgsrc->lock();
      }
      for (c = 0; int(c) < MHC; c++)
        hist[c] = 0;
      //
      if (not asCROMA)
      { // paint coordinates in UV grid
        imgUV->paintGridAligned(MWH, 0, 20.0, 10);
        // clear grid background for text
        p1.x = 18;
        p1.y = 5;
        p2.x = MWH/2 - 3;
        p2.y = 120;
        cvRectangle(imgUV->cvArr(), p1, p2, white, -1, 4, 0);
        // clear histogram background
        p1.x = HIST_X;
        p1.y = MWH - 10;
        p2.x = p1.x + MHC;
        p2.y = p1.y - 100;
        cvRectangle(imgUV->cvArr(), p1, p2, white, -1, 4, 0);
        //
        cvLine(imgUV->cvArr(), cvPoint(MWH/2, MWH/4),
               cvPoint(MWH/2, MWH*3/4), red, lw,8,0);
        cvLine(imgUV->cvArr(), cvPoint(MWH/8,   MWH/2),
               cvPoint(MWH*7/8, MWH/2), red, lw,8,0);
        cvPutText(imgUV->cvArr(), "U", cvPoint(MWH*7/8+1, MWH/2), &font, red);
        cvPutText(imgUV->cvArr(), "V", cvPoint(MWH/2, MWH/4), &font, red);
      }
      // get reduction factor for top-left image
      redFac = (imgsrc->width()-1)/160 + 1;
      if (redFac < 1)
        redFac = 1;
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
              pcd = &imgUV->getLine(r/redFac + 10)[MWH - 180 - 10];
            if ((c % redFac) == 0)
            { // this pixel should be painted
              ppix = bgr;
              // copy pixel to UV image
              *pcd++ = ppix->asBGR(imgsrc->getColorType());
            }
          }
          if (not asCROMA)
          { // use UV coordinates
            pix = bgr->asYUV(imgsrc->getColorType()); //UPixel::RGBtoYUV(bgr->r, bgr->g, bgr->b);
            isOn = pix.y > globalThresholdMin and
                   pix.y < globalThresholdMax;
            if (isOn)
            {
              pos.x = mini(255, maxi(0, (pix.u - 128) * 1 + 128)) * 2;
              pos.y = (255 - mini(255, maxi(0, (pix.v - 128) * 1 + 128))) * 2;
              pix.setYUVto(180, pix.u, pix.v, PIX_PLANES_BGR);
              pix.tone(imgUV->getPix(pos.y, pos.x), 50/redFac);
            }
          }
          else
          { // convert to cromaticity
            d = float(bgr->p1 + bgr->p2 + bgr->p3);
            isOn = d > (globalThresholdMin * 3) and
                   d < (globalThresholdMax * 3);
            if (isOn)
            {
              x = float(bgr->getRed(imgsrc->getColorType())) / d;
              y = float(bgr->getGreen(imgsrc->getColorType())) / d;
              pos.x = mini(MWH - 2, maxi(0, roundi(x * MWH)));
              pos.y = MWH - 2 - mini(MWH - 2, maxi(0, roundi(y * MWH)));
              pix.setRGBto(roundi(x * 256.0), roundi(y * 256.0),
                           roundi((1.0 - x - y) * 256.0), PIX_PLANES_BGR);
              pix.tone(imgUV->getPix(pos.y, pos.x), 40/redFac);
            }
          }
          //
          if (isOn)
          {
            imgUV->setPix(pos.y, pos.x, pix);
            imgUV->setPix(pos.y, pos.x + 1, pix);
            imgUV->setPix(pos.y + 1, pos.x, pix);
            imgUV->setPix(pos.y + 1, pos.x + 1, pix);
            if (asBold)
            {
              imgUV->setPix(pos.y, pos.x - 1, pix);
              imgUV->setPix(pos.y - 1, pos.x, pix);
              imgUV->setPix(pos.y - 1, pos.x - 1, pix);
              imgUV->setPix(pos.y + 1, pos.x - 1, pix);
              imgUV->setPix(pos.y - 1, pos.x + 1, pix);
            }
          }
          bgr++;
        }
      }
      // paint circle around mouse location
      pos.x = globalPixPoint2.x / redFac + MWH - 180 - 10;
      pos.y = globalPixPoint2.y / redFac + 10;
      cvCircle(imgUV->cvArr(), pos, 5, white, 2+lw, 8, 0);
      cvCircle(imgUV->cvArr(), pos, 5, red, lw, 8, 0);
      pos.x = mini(imgsrc->width()-1, maxi(0, globalPixPoint2.x));
      pos.y = mini(imgsrc->height() - 1, maxi(0, globalPixPoint2.y));
      bgr = imgsrc->getPixRef(pos.y, pos.x);
      if (bgr != NULL)
      {
        pix = bgr->asYUV(imgsrc->getColorType());
        if (not asCROMA)
        {
          pos.x = mini(255, maxi(0, (pix.u - 128) * 1 + 128)) * 2;
          pos.y = (255 - mini(255, maxi(0, (pix.v - 128) * 1 + 128))) * 2;
        }
        else
        {
          d = float(bgr->p1 + bgr->p2 + bgr->p3);
          x = float(bgr->getRed(imgsrc->getColorType())) / d;
          y = float(bgr->getGreen(imgsrc->getColorType())) / d;
          pos.x = mini(MWH - 2, maxi(0, roundi(x * MWH)));
          pos.y = MWH - 2 - mini(MWH - 2, maxi(0, roundi(y * MWH)));
        }
        hyc = roundi((bgr->p1 + bgr->p2 + bgr->p3) / 3.0);
        cvCircle(imgUV->cvArr(), pos, 5, white, 1+lw, 8, 0);
        cvCircle(imgUV->cvArr(), pos, 5, red, lw, 8, 0);
      }
      // text on top
      cvPutText(imgUV->cvArr(), imgsrc->name, cvPoint(22, 20), &font, blue);
      snprintf(s, MSL, "%d < y < %d",
               globalThresholdMin, globalThresholdMax);
      cvPutText(imgUV->cvArr(), s, cvPoint(22, 40), &font, blue);
      snprintf(s, MSL, "r,g,b = %d,%d,%d",
               bgr->getRed(imgsrc->getColorType()),
               bgr->getGreen(imgsrc->getColorType()),
               bgr->getBlue(imgsrc->getColorType()));
      cvPutText(imgUV->cvArr(), s, cvPoint(22, 60), &font, red);
      snprintf(s, MSL, "y,u,v = %d,%d,%d",
               pix.y, pix.u, pix.v);
      cvPutText(imgUV->cvArr(), s, cvPoint(22, 80), &font, red);
      // mouse position
      snprintf(s, MSL, "at x,y = %d,%d",
               globalPixPoint2.x, globalPixPoint2.y);
      cvPutText(imgUV->cvArr(), s, cvPoint(22, 100), &font, red);
      cvPutText(imgUV->cvArr(), "(0,0)", cvPoint(4, MWH-4), &font, red);
      //
      if (asCROMA)
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
      }
      // paint histogram
      p1.x = HIST_X;
      if (not asCROMA)
        p1.y = MWH - 10;
      else
        p1.y = HIST_Y;
      redFac = 0;
      for (c = 0; int(c) < MHC; c++)
        redFac = maxi(redFac, hist[c]);
      redFac /= 100;
      if (redFac < 1)
        redFac = 1;
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

      imgUV->updated();
      imgUV->used=0;
      imgsrc->unlock();
    }
  }
}


void UResDisp::addImagePoolImg(int imgPoolNum)
{
#ifdef USE_HIGHGUI
  if (varUseHighGUI->getBool())
  {
    int i;
    UHighGuiWindowHandle * ud;
    bool found = false;
    //
    ud = updList;
    for (i = 0; i < updListCnt; i++)
    {
      if (ud->getImageNum() == imgPoolNum)
      {
        found = true;
        break;
      }
      ud++;
    }
    if (not found and (updListCnt < UPD_LIST_MAX_CNT))
    {
      ud->setImage(imgPool->getImage(imgPoolNum, false), imgPoolNum);
      updListCnt++;
    }
  }
#endif
}

//////////////////////////////////////////

void UResDisp::removeImagePoolImg(int imgPoolNum)
{
  int i;
  UHighGuiWindowHandle * ud;
  //
  ud = updList;
  for (i = 0; i < updListCnt; i++)
  {
    if (ud->getImageNum() == imgPoolNum)
    {
      ud->removeImage();
      break;
    }
    ud++;
  }
}

////////////////////////////////////////

void UResDisp::paintBold(bool bold)
{
  navPaint.paintBold = bold;
  varBold->setValued(bold);
}

//////////////////////////////////////////////

// void UResDisp::capture(int camIdx)
// {
// #ifdef USE_HIGHGUI
//   CvCapture* capture = NULL;
//   IplImage* frame = NULL;
//   CvSize size;
//   UImage * img4;
//   const int imgnr = 4;
//   int chW = 6, chH = 8; // checkboard inner corners
// //  const int MCA = 60*80;
// //  CvPoint2D32f corners[MCA];
//   int cornersCnt = 0;
// //  int result = 0;
//   //   bool setSize(const unsigned int iHeight, const unsigned int iWidth,
//   // const int channels /* = 3 */, const int depth /* = 8 */,
//   // const char * colorType = NULL);
//   //
//   printf("UResDisp::capture - start (cam index %d)\n", camIdx);
//   capture = cvCaptureFromCAM(camIdx);
//   if (capture == NULL)
//     printf("capture failed\n");
//   else
//   {
//     printf("UResDisp::capture - get frame\n");
//     frame = cvQueryFrame( capture );
//     if (frame == NULL)
//       printf("no frame\n");
//     else
//     {
//       img4 = imgPool->getImage(imgnr, true);
//       size = cvGetSize(frame);
//       if (size.width * size.height > 640 * 480)
//       {
//         size.width = 640;
//         size.height = 480;
//       }
//       printf("Got an image of size %dx%d\n", size.width, size.height);
//       img4->setSize(size.height, size.width, 3, 8, "RGB");
//       memcpy(img4->getData(), frame->imageData, size.width * size.height * 3);
//       addImagePoolImg(imgnr);
//       img4->updated();
//       img4->used=false;
//       //
// /*      result = cvFindChessboardCorners(img4->cvArr(), cvSize(chW, chH),
//                                            //&ponintsTemp[lr][0],
//                                            corners, &cornersCnt,
//                                            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE
//                                            );*/
//       printf("found %d checkerboard corners of expected(%dx%d)\n", cornersCnt, chW, chH);
//     }
//     cvReleaseCapture(&capture);
//   }
//   printf("UResDisp::capture - end\n");
// #endif
// }

///////////////////////////////////////////////////

void UResDisp::setRobot(const char * robname)
{
  if (strcasecmp(robname, "SMR") == 0)
  {
    navPaint.paintRobot = 1;
    varRangeRings->setValued(4.0);
  }
  else if (strcasecmp(robname, "MMR") == 0)
  {
    navPaint.paintRobot = 0;
    varRangeRings->setValued(8.0);
  }
  else if (strcasecmp(robname, "HAKO") == 0)
  {
    navPaint.paintRobot = 2;
    varRangeRings->setValued(8.0);
  }
  else if (strcasecmp(robname, "irobot") == 0)
  {
    navPaint.paintRobot = 3;
    varRangeRings->setValued(16.0);
  }
  else if (strcasecmp(robname, "guidebot") == 0)
  {
    navPaint.paintRobot = 4;
    varRangeRings->setValued(16.0);
  }
}

/////////////////////////////////////////////////

void UResDisp::setRangeRingCnt(int value)
{
  varRangeRings->setValued(value);
  navPaint.rangeRingCnt=value;
}

////////////////////////////////////////////////

void UResDisp::setScale(double value)
{
  navPaint.maxRange = value;
}

/////////////////////////////////////////////////

void UResDisp::setRobotPose(char * value)
{
  const char *p2 = value;
  navPaint.robotPose.x = strtod(p2, (char**)&p2);
  if (p2 != NULL)
  {
    if (*p2 == ',')
      p2++;
    navPaint.robotPose.y = strtod(p2, (char**)&p2);
  }
  if (p2 != NULL)
  {
    if (*p2 == ',')
      p2++;
    navPaint.robotPose.h = strtod(p2, (char**)&p2);
  }
}

///////////////////////////////////////////////

void UResDisp::mouseRescale(bool shiftKey)
{
  CvPoint dx;

// navPaint.printRefSystems();

  dx.x = globalPixPoint2.x - globalPixPoint1.x;
  dx.y = globalPixPoint2.y - globalPixPoint1.y;
  if (absi(dx.x) + absi(dx.y) == 0)
  {
    dx.x = globalPixPointS.x - globalPixPoint1.x;
    dx.y = globalPixPointS.y - globalPixPoint1.y;
  }

//  printf("Mouse rescale new=%dx,%dy; dx=%dx,%dy\n", globalPixPoint1.x, globalPixPoint1.y, dx.x, dx.y);

  if (absi(dx.x) + absi(dx.y) == 0)
  {  // pan to this pose
    if (shiftKey)
      navPaint.mousePan(globalPixPoint1, navPaint.maxRange * 1.5);
    else
      navPaint.mousePan(globalPixPoint1, navPaint.maxRange);
  }
  else
    navPaint.mouseScale(globalPixPoint1, dx);

//printf("UResDisp::mouseScale: maxRange=%.2f robotPose=%.2fx,%.2fy\n", navPaint.maxRange, navPaint.robotPose.x, navPaint.robotPose.y);
//navPaint.printRefSystems();

  setNewDataNav();
}

//////////////////////////////////////////////////

void UResDisp::setUvRedisplay(bool value)
{
  if (imgSourceUV >= 0)
    globalUvRedisplay = value;
}


