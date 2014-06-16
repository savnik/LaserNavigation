/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars
 *   4 apr 2011 Kristian Villen og Linsey? - new color balance
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
#include <legacy/compat.hpp>
#endif

#include <urob4/usmltag.h>

#include "ufuncballkl.h"

//#define UIMGPOOL_BALL_BASE 45

#ifdef LIBRARY_OPEN_NEEDED

UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncBallKL' with your classname, as used in the headerfile */
  return new UFuncBallKL();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncBallKL::~UFuncBallKL()
{ // possibly remove allocated variables here - if needed
  printf("ballfinder unloaded\n");
}


///////////////////////////////////////////////////

bool UFuncBallKL::handleCommand(UServerInMsg * msg, void * extra)
{ // handle command(s) send to this plug-in
  const int MRL = 2000;
  char reply[MRL];
  bool ask4help;
/*  const int MVL = 50;
  char val[MVL];*/
//  bool gotDevice = false;
  int camDevice = -1;
  bool gotImg = false;
  int imgPoolNum = -1;
  UImage * img = (UImage *)extra;
  USmlTag tag;
  UCamPush * cam = NULL;
  bool result;
  bool debug = true; // default is debug on
  bool smrcl = false; // default is <ball ...> reply
  bool gotBlue = false;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", NULL, 0);
  if (not ask4help)
  { // get all other parameters
    //gotDevice = msg->tag.getAttValueInt("device", &camDevice);
    gotImg = msg->tag.getAttValueInt("img", &imgPoolNum);
    msg->tag.getAttValueBool("debug", &debug, true);
    msg->tag.getAttValueBool("smrcl", &smrcl, true);
    msg->tag.getAttValueBool("blue", &gotBlue, true);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("Ballfinder");
    sendText("--- available BALL options\n");
//    sendText("device=X          Use this camera - for position and parameters\n");
    sendText("img=X             Get image from image pool - else take new image\n");
    sendText("blue              Try find a blue ball (def is red)\n");
    sendText("debug=false       Less images and print on server console (def=true)\n");
    sendText("smrcl             Format the reply for MRC (<vision vis1=\"x.x\" vis2=\"y.y\" .../>\n");
    sendText("help              This message\n");
    sendText("---\n");
    sendText("See also: VAR BALL for other parameters and results\n");
    sendHelpDone();
    sendInfo("done");
    result = true;
  }
  else
  { // not help, so - first - get source image
    if (not gotImg)
    {
      imgPoolNum = varDefImg->getInt(0);
      gotImg = imgPoolNum >= 0;
    }
    if (gotImg)
    { // take image from image pool
      img = imgPool->getImage(imgPoolNum, false);
      result = (img != NULL);
      if (result and camDevice < 0)
        //take image device from image
        camDevice = img->camDevice;
      if (result)
      {
        cam = camPool->getCam(camDevice);
        result = (cam != NULL);
      }
    }
    else if (img != NULL)
    { // we have a pushed image in the extra parameter, so just camera is needed
      camDevice = img->camDevice;
      cam = camPool->getCam(camDevice);
      result = (cam != NULL);
    }
    else
    { // get new image from a camera and store in first imagepool number for ball images
      img = imgPool->getImage(varTmpImage->getInt(0), true);
      result = getCamAndRawImage(&cam,        // result camera           out
                                 &img,        // result image            out
                                 &camDevice,  // camera device number    in-out
                                 NULL,        // pushed image (YUV)      in
                                 "", -1);         // camera position name    in
      if (result)
        result = (img != NULL and cam != NULL);
    }
    // camera and image is now available
    // time to kick some ass
    if (result)
    { // there is an image, make the required ball analysis
      result = findBall(cam, img, debug, gotBlue);
      // send the result back to client
      if (smrcl)
      { // format for MRC to be used in smrcl script
        snprintf(reply, MRL, "<vision vis1=\"%d\" vis2=\"%g\" vis3=\"%g\" "
            " vis4=\"%d\"/>\n",
            result, ballPos.x, ballPos.y, ellCnt);
        sendMsg(reply);
      }
      else if (not result)
      { // did not find any balls in image - with a reasonable size
        snprintf(reply, MRL, "<%s cnt=\"%d\"/>\n", msg->tag.getTagName(), ellCnt);
        sendMsg(reply);
      }
      else
      { // send XML open tag with ball count as attribute
        snprintf(reply, MRL, "<%s cnt=\"%d\" diameter=\"%g\" x=\"%.2f\" y=\"%.2f\" z=\"%.2f\"/>\n",
                 msg->tag.getTagName(), ellCnt, varBallSize->getValued(),
                 ballPos.x, ballPos.y, ballPos.z);
        sendMsg(reply);
      }
    }
    else
    {
      snprintf(reply, MRL, "failed, got image %s, got camera %d %s\n",
               bool2str(img != NULL), camDevice, bool2str(cam != NULL));
      sendWarning(reply);
    }
  }
  // return true if the function is handled with a positive result
  return result;
}

////////////////////////////////////////////////////////////

void UFuncBallKL::createBaseVar()
{
  varRedLim      = addVar("redLim", "51", "d",
                       "(r/w) Limit for red above average red [0..255]");
  varRedPos   = addVar("redPos", "2.0 -1.0 0.0", "3d",
                       "(r) Position of ball in coordinates relative to robot");
  varRedTime  = addVar("redTime", 0.0, "d", "(r) time of red ball position");
  varRedCnt   = addVar("redCnt", 0.0, "d", "(r) Number of red balls found in last image");
//  varRedEdgeLR   = addVar("redEdgeLR", "0.0 0.0", "d", "(r) Ball is partially outside image - to the left or right - 0.0 is not");
  varBlueLim      = addVar("blueLim", "51", "d",
                       "(r/w) Limit for blue above average blue [0..255]");
  varBluePos   = addVar("bluePos", "2.0 -1.0 0.0", "3d",
                       "(r) Position of ball in coordinates relative to robot");
  varBlueTime  = addVar("blueTime", 0.0, "d", "(r) time of blue ball position");
  varBlueCnt   = addVar("blueCnt", 0.0, "d", "(r) Number of blue balls found in last image");
//  varBlueEdgeLR   = addVar("blueEdgeLR", "0.0 0.0", "d", "(r) Ball is partially outside image - to the left or right");
  varBallSize = addVar("BallSize", 0.12, "d", "(r/w) Size of the ball (diameter [m])");
  varTopLine = addVar("topLine", 15, "d", "(r/w) is the topmost line that could be a ball on the floor.");
  varMajMinRatio = addVar("majMinRatio", 0.3, "d", "(r/w) minimum ration between major and minor axis of fitted ellipse");
  varMaxSize = addVar("maxSize", 65.0, "d", "(r/w) Is maximum size of ball in pixels (major exis)");
  varMinSize = addVar("minSize", 2.0, "d", "(r/w) Is minimum size of ball in pixels (minor exis)");
  //
  varDefImg = addVar("defImg", 18.0, "d", "(r/w) default image to use as source");
  varTmpImage = addVar("tmpImage", 40.0, "d", "(r/w) first pool image number to use as temporary image");
  varBlackLimit = addVar("blackLimit", 42.0, "d", "(r/w) Black limit where no (true) color is expected [0..255]");
}

//////////////////////////////////////////////

bool UFuncBallKL::findBall(UCamPush * cam, UImage * img, bool debug, bool blue)
{
  bool found;
  UTime t;
  double ballDiam;
  UImage * mask, *dbgImg = NULL, *maskOrg;
  UImagePool * imgPool;
  int imgPoolBase = varTmpImage->getInt(0);
  UPixel gray(127,127,127);
  //
  imgPool = (UImagePool*)getStaticResource("imgPool", true);
  found = (img != NULL);
  if (found)
  { // an image is available, so ...
    // save source image size for border analysis
    imgSizeW = img->width();
    imgSizeH = img->height();
    // get image for result of colour masking
    mask = imgPool->getImage(imgPoolBase + 1, true);
    // mask the colours
    maskColors(img, mask, blue, debug);
    // find balls in the
    if (debug)
    { // make a copy of the mask (as the mask image is destroyed by the
      // 'ballCandidate' function
      maskOrg = imgPool->getImage(imgPoolBase + 2, true);
      maskOrg->copy(mask);
      maskOrg->setName("masked-pixels");
      maskOrg->updated();
      //
      // get debug image for found balls
      dbgImg = imgPool->getImage(imgPoolBase + 3, true);
      // use original image as background
      dbgImg->copy(img);
      dbgImg->setName("result-circles");
      // but a bit faint (50% towards gray)
      dbgImg->tone(gray, 50);
    }
    else
      dbgImg = NULL;
    //
    // filter mask - remove isolated pixels - may not be needed
    filterMask(mask, mask);
    //
    // analyse the masked area and fit to ellipses
    found = findBallCandidates(mask, dbgImg);
  }
  // get time image were captured
  t = img->imgTime;
  if (blue)
  { // set number of found balls
    varBlueCnt->setValued(ellCnt);
    // save image time as detection time for red balls
    varBlueTime->setValued(t.getDecSec());
  }
  else
  { // set number of found balls
    varRedCnt->setValued(ellCnt);
    // save image time as detection time for red balls
    varRedTime->setValued(t.getDecSec());
  }
  // Convert ball position to robot coordinates
  if (found)
  { // get known real size of ball (in meters)
    ballDiam = varBallSize->getValued();
    // find ball position
    found = calculateBallPosition(cam, dbgImg, ballDiam);
  }
  if (dbgImg != NULL)
    // tell the imagepool that the image is updated (may trigger events)
    dbgImg->updated();
  //
  if (found)
  { // the ball position is found too, ...
    // save result into global variables
    if (blue)
    { // save blue values
      varBlueCnt->setValued(ellCnt, 0);
      varBluePos->set3D(&ballPos);
    }
    else
    { // save blue values
      varRedCnt->setValued(ellCnt, 0);
      varRedPos->set3D(&ballPos);
    }
  }
  return found;
}

//////////////////////////////////////////////

void UFuncBallKL::maskColors(UImage * src, UImage * dstImg, bool blue, bool debug)
{
  signed int temp,temp2;
  int avgR, avgB, avgG;
  //bool result = false;
  UPixel * pixSrc, * pixDst;
  unsigned char * pixMask;
  int topLine;
  unsigned char uLim;
  UVariable * var;
  UImage * srcImg = src, *tmpImg=src;
  int imgPoolBase = varTmpImage->getInt(0);
  int pixCnt = 0;
  int blackLimit = varBlackLimit->getInt();
  //const int MAX_ALLOWED_YVALUE = 170;
  //
  // ensure source image is in YUV format
  if (not srcImg->isRGB())
  { // change colour format
    srcImg->toRGB(NULL);
    srcImg->updated();
  }
  if (debug)
  {
    UImage * dbgImg;
    // get debug image for found balls
    dbgImg = imgPool->getImage(imgPoolBase + 6, true);
    // use original image size
    dbgImg->copyMeta(srcImg, true);
    dbgImg->setName("Normalized RGB");
    tmpImg = dbgImg;
  }

	// normalize intensity
  avgR = 0;
  avgG = 0;
  avgB = 0;
  for (int i = 0; i < ((int)srcImg->getHeight()); i++)
  {
    pixSrc = srcImg->getLine(i);
    pixDst = tmpImg->getLine(i);
    for (int j = 0; j < ((int)srcImg->getWidth()); j++)
    {
      temp = pixSrc->p1 + pixSrc->p2 + pixSrc->p3;
      if (temp > blackLimit)
      {
        temp2 = ((int)pixSrc->p1 * 256)/temp;
        avgR += temp2;
        pixDst->p1 = (unsigned char)temp2;
        temp2 = ((int)pixSrc->p2 * 256)/temp;
        avgG += temp2;
        pixDst->p2 = (unsigned char)temp2;
        temp2 = ((int)pixSrc->p3 * 256)/temp;
        avgB += temp2;
        pixDst->p3 = (unsigned char)temp2;
        pixCnt++;
      }
      else
      {
        pixDst->p1 = 0;
        pixDst->p2 = 0;
        pixDst->p3 = 0;
      }
      pixSrc++;
      pixDst++;
    }
  }
  //remove avrage value
  avgR /= pixCnt;
  avgG /= pixCnt;
  avgB /= pixCnt;
  printf("avgR = %i \n\r",avgR);
  printf("avgG = %i \n\r",avgG);
  printf("avgB = %i \n\r",avgB);
  //printf("width = %i \n\r",srcImg->getHeight());
  //printf("height = %i \n\r",srcImg->getWidth());
  //
  srcImg = tmpImg;
  if (debug)
  { // get new temp image
    UImage * dbgImg;
    // get debug image for found balls
    dbgImg = imgPool->getImage(imgPoolBase + 7, true);
    // use original image size
    dbgImg->copyMeta(srcImg, true);
    dbgImg->setName("Average limited");
    tmpImg = dbgImg;
  }
  // limit so that values above average remains
  for (int i = 0; i < ((int)srcImg->getHeight()); i++)
  {
    pixSrc = srcImg->getLine(i);
    pixDst = tmpImg->getLine(i);
    for (int j = 0; j < ((int)srcImg->getWidth()); j++)
    {
//      temp2 = (((int)pixSrc->p1 * 256)/(int)avgR - 256);
      temp2 = pixSrc->p1 - avgR;
      if(temp2 < 0)
        temp2 = 0;
      pixDst->p1 = (unsigned char)temp2;
//      temp2 = (((int)pixSrc->p2 * 256)/(int)avgG - 256);
      temp2 = pixSrc->p2 - avgG;
      if(temp2 < 0)
        temp2 = 0;
      pixDst->p2 = (unsigned char)temp2;
//      temp2 = (((int)pixSrc->p3 * 256)/(int)avgB - 256);
      temp2 = pixSrc->p3 - avgB;
      if(temp2 < 0)
        temp2 = 0;
      pixDst->p3 = (unsigned char)temp2;
      pixSrc++;
      pixDst++;
    }
  }

  srcImg = tmpImg;
//  srcImg->setName("source");
  //
  // get threashold values (in 0..255 range) from global var settings
  if (blue)
    var = varBlueLim;
  else
    var = varRedLim;
  uLim = (unsigned char)var->getInt(0);
  // prepare destination image
  // copy image time and source from original
  dstImg->copyMeta(srcImg, false);
  // set size and colour depth - one plane of 8-bit
  dstImg->setSize(srcImg->getHeight(), srcImg->getWidth(), 1, 8, "gray");
  // and give it a nice name
  dstImg->setName("mask");
  //
  topLine = varTopLine->getInt();
  // apply given colour threashold and put result into image 2
  for (int row = 0; row < (int)srcImg->getHeight(); row++)
  { // get pixel pointer (3-plane) to source image start of row
    pixSrc = srcImg->getLine(row);
    // get pointer to destination pixel (same row)
    pixMask = (unsigned char *) dstImg->getLine(row);
    //
    for (int col = 0; col < (int)srcImg->getWidth(); col++)
    { // plane 1 (p1) is Y, plane 2 (p2) is U and plane 3 (p3) is V
      if (blue)
      {
        if (row > topLine and pixSrc->p3 > uLim)
          // right colou - set to large value (white)r
          *pixMask = 255;
        else
          // not ball colour - so black
          *pixMask = 0;
      }
      else
      {
        if (row > topLine and pixSrc->p1 > uLim)
          // right colou - set to large value (white)r
          *pixMask = 255;
        else
          // not ball colour - so black
          *pixMask = 0;
      }
      // increase to next destination pixel
      pixMask++;
      // increase to next source pixel
      pixSrc++;
    }
  }
  dstImg->updated();
}

//////////////////////////////////////////////////

void UFuncBallKL::filterMask(UImage * src, UImage * dst)
{
  UImagePool * imgPool;
  UImage *img1;
  int iterations = 1;
  //
  imgPool = (UImagePool*)getStaticResource("imgPool", true);
  img1 = imgPool->getImage(varTmpImage->getInt(0) + 4, true);
  img1->copyMeta(src, true);
  if (src != dst)
    dst->copyMeta(src, true);
  //
  cvErode( src->cvArr(), img1->cvArr(), NULL, iterations);
  cvDilate( img1->cvArr(), dst->cvArr(), NULL, iterations);
}

////////////////////////////////////////////////////

bool UFuncBallKL::findBallCandidates(UImage * img, UImage * ellImg)
{
  //bool result = false;
  CvMemStorage* stor;
  CvSeq* cont;
  CvPoint* PointArray;
  CvPoint2D32f* PointArray2D32f;
  int i, count;
  float eMaj, eMin;
  CvPoint center;
  CvSize size;
  float angle;
  int minSize = 8; // minimum pixel size of found colour blocks
  int maxSize;
  double ratio, ratioLim;
  //
  // Using openCV to find structures in the mask image.
  // OpenCV likes dinamic storage allocation, so here is some space allocated
  stor = cvCreateMemStorage(0);
  // create sequence structure for found structures
  cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
  // Find all contours.in image 4
  cvFindContours( img->cvArr(), stor, &cont, sizeof(CvContour),
                  /*CV_RETR_LIST*/ CV_RETR_EXTERNAL
                      , CV_CHAIN_APPROX_NONE, cvPoint(0,0));
  //
  ratioLim = varMajMinRatio->getDouble();
  maxSize = varMaxSize->getInt();
  // Now the found contours will be estiamted as ellipses
  // if they larger than 4x4 pixels
  ellCnt = 0;
  //
  // This cycle draw all contours and approximate it by an ellipses.
  // continue as long as there is contoures in the contour sequence
  for(;cont != NULL;cont = cont->h_next)
  { // find interesting shapes
    count = cont->total; // This is number point in contour
    //CvPoint center;
    //CvSize size;
    //
    // Number point must be more than or equal to 10 (for cvFitEllipse_32f).
    if (count < 10)
      // too small to try an ellipse fit
      continue;
    //
    // Alloc memory for contour point set.
    PointArray = (CvPoint*)malloc( count*sizeof(CvPoint) );
    PointArray2D32f= (CvPoint2D32f*)malloc( count*sizeof(CvPoint2D32f) );
    //
    // Get contour point set.
    cvCvtSeqToArray(cont, PointArray, CV_WHOLE_SEQ);
    //
    // Convert CvPoint set to CvBox2D32f set.
    for(i = 0; i < count; i++)
    {
      PointArray2D32f[i].x = (float)PointArray[i].x;
      PointArray2D32f[i].y = (float)PointArray[i].y;
    }
    //
    // Fits ellipse to current contour.
    cvFitEllipse(PointArray2D32f, count, &ell[ellCnt]);
    // test the esitimated ellipse box for size
    eMaj = fmax(ell[ellCnt].size.width, ell[ellCnt].size.height); // major axis
    eMin = fmin(ell[ellCnt].size.width, ell[ellCnt].size.height); // major axis
    ratio = eMin/eMaj;
    printf("Maj=%.1f min=%.1f ratio=%.3f lim=%.3f\n", eMaj, eMin, ratio, ratioLim);
    if (eMaj < maxSize and eMin > minSize and ratio > ratioLim and eMin > varMinSize->getInt()) // and count > ePixCnt)
    { // an ellipse is found, so save its position and size
      //
      if (ellImg != NULL)
      { // debug - Draw current contour.
        // void cvDrawContours(CvArr*, CvSeq*, CvScalar, CvScalar, int, int, int)
        cvDrawContours(ellImg->cvArr(),cont,CV_RGB(255,255,255),CV_RGB(255,255,255),0,1,8 /*,cvPoint(0,0)*/);
        //
        // Convert ellipse data from float to integer representation.
        center.x = cvRound(ell[ellCnt].center.x);
        center.y = cvRound(ell[ellCnt].center.y);
        size.width = cvRound(ell[ellCnt].size.width * 0.5);
        size.height = cvRound(ell[ellCnt].size.height * 0.5);
        angle = -ell[ellCnt].angle;
        //
        // Draw ellipse.
        cvEllipse(ellImg->cvArr(), center, size,
                  angle, 0, 360,
                  CV_RGB(0,0,255), 1, CV_AA, 0);
      }
      if (ellCnt < MAX_ELL_CNT)
        ellCnt++;
    }
    // Free allocated memory
    free(PointArray);
    free(PointArray2D32f);
  }
  return ellCnt > 0;
}

////////////////////////////////////////////////

bool UFuncBallKL::calculateBallPosition(UCamPush * cam, UImage * ellImg, double ballDiam)
{
  bool result;
  double bestDiameter = 4.0; // in pixels
  int bestIdx = -1;
  CvBox2D32f * bestEll = NULL;
  int i;
  UPosition posCam;   // position of ball in camera coordinates
  double focalLength; // in pixels at current resolution
  double hx, hy;      // centre of image (in pixels)
  UPosRot camPose;
  UMatrix4 mCamPose;  // matrix for conversion to robot coordinates
  CvScalar red = CV_RGB(255, 100, 100);
  CvFont font;
  const int MSL = 50;
  char s[MSL];
  float eMaj, eMin;
  //
  result = (ellCnt > 0) and (cam != NULL);
  if (result)
  { // find best
    if (ellImg != NULL)
      printf("Found %d balls in image\n", ellCnt);
    // find the best (largest) ellipse
    for (i = 0; i < ellCnt; i++)
    {
      eMaj = fmax(ell[i].size.width, ell[i].size.height); // major axis
      if (eMaj > bestDiameter)
      {
        bestEll = &ell[i];
        bestDiameter = eMaj;
        bestIdx = i;
      }
      if (ellImg != NULL)
      { // debug mode
        eMin = fmin(ell[i].size.width, ell[i].size.height); // minor axis
        printf(" - %d size %3.1fmajor %3.1fminor (angle %3.1fdeg) at %3.1fx %3.1fy (pixels)\n", i,
               eMaj, eMin, ell[i].angle, ell[i].center.x, ell[i].center.y);
      }
    }
    result = bestIdx >= 0;
  }
  if (result)
  {// get focal length in pixels from camera
    focalLength = cam->getCamPar()->getFocalLength();
    // calculate distance from camera to ball
    posCam.x = focalLength / bestDiameter * ballDiam;
    // and to the right of the camera axis (opporsite x in pixels coordinates)
    hx = cam->getCamPar()->getHx();
    posCam.y = (hx - bestEll->center.x) / focalLength * posCam.x;
    // and the distance above the camera axis
    hy = cam->getCamPar()->getHy();
    posCam.z = (hy - bestEll->center.y) / focalLength * posCam.x;
    //
    // the position in camera coordinates now need to be converted to
    // robot coordinates, based on the known camera position on the robot
    // this is the 3D position of camera and orientation - pan, tilt and roll
    camPose = cam->getPosRot();
    // these 6 values (x, y, z, Omega, Phi, Kappa) can be expressed as a
    // 4x4 matrix to convert positions in cam coordinates to robot coordinates
    // (the function is intended to get the matrix from robot to map coordinates,
    // but it works equally well from camera to robot coordinates)
    mCamPose = camPose.getRtoMMatrix();
    // now we can get the ball center position in robot coordinates
    ballPos = mCamPose * posCam;
    //
    if (ellImg != NULL)
    { // debug mode
      ballPos.print("Ball position");
      //
      // paint best position in image as a red cross
      cvLine(ellImg->cvArr(),
                  cvPoint((int)bestEll->center.x - 15, (int)bestEll->center.y),
                  cvPoint((int)bestEll->center.x + 15, (int)bestEll->center.y),
                  red, 1, CV_AA, 0);
      cvLine(ellImg->cvArr(),
                  cvPoint((int)bestEll->center.x, (int)bestEll->center.y - 15),
                  cvPoint((int)bestEll->center.x, (int)bestEll->center.y + 15),
                  red, 1, CV_AA, 0);
      cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                 1.0, 1.0, 0.0, 1, 8);
      snprintf(s, MSL, "%.2fm", posCam.x);
      cvPutText(ellImg->cvArr(), s, cvPoint((int)bestEll->center.x + 4, (int)bestEll->center.y - 4), &font, red);
    }
  }
  else if (ellImg != NULL)
  { // debug mode
    printf("No ball found\n");
  }
  return result;
}


