/***************************************************************************
 *   Copyright (C) 2010 by DTU (Christian Andersen)                        *
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

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>
#include <urob4/uimagepool.h>
#include <urob4/uvariable.h>
#include <ucam4/ucampool.h>

#include "ufuncstereo.h"

//#if (CV_MAJOR_VERSION >= 1)

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncStereo' with your classname, as used in the headerfile */
  return new UFuncStereo();
}

#endif

UFuncStereo::~UFuncStereo()
{
  if (mx1 != NULL)
  {
    cvReleaseStereoBMState(&BMState);
    cvReleaseMat( &mx1 );
    cvReleaseMat( &my1 );
    cvReleaseMat( &mx2 );
    cvReleaseMat( &my2 );
    cvReleaseMat( &img1rd );
    cvReleaseMat( &img2rd );
    if (img1 != NULL)
      delete img1;
    if (img2 != NULL)
      delete img2;
  }
}

///////////////////////////////////////////////////

bool UFuncStereo::handleCommand(UServerInMsg * msg, void * extra)
{ // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA("stereo"))
    result = handleStereo(msg);
  else if (msg->tag.isTagA("stereoPush"))
    result = handlePush(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFuncStereo::handleStereo(UServerInMsg * msg)
{
  const int MRL = 500;
  char reply[MRL];
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("STEREO");
    sendText("--- STEREO is an implementation of openCV stereo\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    sendText(            "do                  Update disparity image from source images\n");
    sendText(            "pclFile             make a PCD file from the point cloud\n");
    sendText(            "init                Init should be called if calibration values are changed (not needed at start)\n");
    sendText(            "silent              do not send a reply to this command\n");
    if (varImagesLR != NULL)
    {
      snprintf(reply, MRL, "debug=true|false  Produce debug images (is %s to images from img=%d)\n",
               bool2str(varDebug->getBool(1)), varImageDisp->getInt(1));
      sendText(reply);
    }
    sendText("help       This message\n");
    sendText("----\n");
    sendText("see also: stereoPush for event handling of 3d cloud\n");
    sendText("see also: 'var stereo' for stereo generation parameters\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    bool doLog = false;
    bool aLog = msg->tag.getAttBool("log", &doLog, true);
    bool anUpdate = msg->tag.getAttBool("do", NULL, false);
    bool silent = msg->tag.getAttBool("silent", NULL);
    bool aDebugValue = false;
    bool aDebug = msg->tag.getAttBool("debug", &aDebugValue, true);
    bool anInit = msg->tag.getAttBool("init", NULL);
    //
    // implement command
    if (aLog)
    { // open or close the log
      aLog = logf.openLog(doLog);
      if (doLog)
        snprintf(reply, MRL, "Opened logfile %s (%s)", logf.getLogFileName(), bool2str(aLog));
      else
        snprintf(reply, MRL, "closed logfile %s", logf.getLogFileName());
      // send an information tag back to client
      if (not silent)
        sendInfo(reply);
    }
    if (aDebug)
    {
      if (varDebug->getBool(0) != aDebugValue)
      {
        varDebug->setValued(aDebugValue, 0);
        if (not silent)
        {
          snprintf(reply, MRL, "debug set to %s from image %d", bool2str(aDebugValue),
                  varImageDisp->getInt(3));
          sendInfo(reply);
        }
      }
      else if (not silent)
        sendInfo("debug value is unchanged");
    }
    if (anInit)
    {
      initializeStereo(true);
      if (not silent)
        sendInfo("image rectification reinitialized");
    }
    if (anUpdate)
    { // default update call
      bool isOK = false;
      if (varImagesLR != NULL)
        isOK = processImages();
      if (isOK and cloud3d.p3dCnt > 0 and msg->tag.getAttBool("pclFile", NULL))
      {
        cloud3d.lock();
        cloud3d.makePCLFile(NULL, true, true);
        cloud3d.unlock();
      }
      // format reply as valid XML tag
      if (not silent)
      {
        if (isOK)
        { // OK result
          varUpdateCnt->add(1.0);
          snprintf(reply, MRL, "<stereo isOK=\"%s\" cnt=\"%d\" tod=\"%.3f\"/>\n",
                bool2str(isOK), varUpdateCnt->getInt(0), varTime->getDouble());
          sendMsg(reply);
        }
        else
        { // failed reply
          snprintf(reply, MRL, "Stereo processing failed - %s", whyString);
          sendWarning(reply);
        }
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncStereo::createResources()
{
  // Create global variables - owned by this plug-in.
  // Returns a pointer to the the variable for easy access.
  varUpdateCnt = addVar("updCnt", 0.0, "d", "(r) Number of updates");
  varPoseOnRobot = addVar("poseOnRobot", "0 0 0", "pose", "(rw) pose of left calera on robot");
  varTime = addVar("time", 0.0, "t", "(r) Time at last update");
  varImageSize = addVar("imageSize", "480 752", "d", "(rw) source image size (h,w) - must fit with intrinsic matrix");
  varImagesLR = addVarA("imagesLR", "10 11", "d", "(rw) Image pool source for left and right image");
  varImageDisp = addVarA("imageDisp", "31 32 33 34 35 36 37", "d", "(rw) Image pool for "
          "disparity (16bit), and if debug BW, color, rectified L,R, epipolar, 3D)");
  varDebug = addVarA("Debug", "0", "d", "(rw) Set to 1 to make more debug images");
  varIntrinsicLeft = addVarA("intrinsicLeft", "696   0  364;"
                                              "  0 696  236;"
                                              "  0   0    1", "m",
                             "(rw (init)) Camera intrinsic matrix holding focal length (fx,fy)"
                             " in pixels: [1,1]=fx [2,2]=fy, and headpoint (cx,cy) in [1,3]=cx, [2,3]=cy");
  varDistortionLeft = addVarA("distortionLeft","-0.2035  0.139  0.0  0.0  -0.0159", "m",
                             "(rw (init)) lens distortion: [k1, k2, p1, p2, k3], k is radial and p is tangential values (see openCV manual)");
  varIntrinsicRight = addVarA("intrinsicRight", "696   0  384;"
                                                "  0 696  219;"
                                                "  0   0    1", "m",
                             "(rw (init)) Camera intrinsic matrix holding focal length (fx,fy)"
                             " in pixels: [1,1]=fx [2,2]=fy, and headpoint (cx,cy) in [1,3]=cx, [2,3]=cy");
  varDistortionRight = addVarA("distortionRight","-0.224  0.3158  0.0013  0.0036  -0.337", "m",
                             "(rw (init)) lens distortion: [k1, k2, p1, p2, k3], k is radial and p is tangential values (see openCV manual)");
  varFundamental = addVarA("fundamental", " 7.444e-09    1.926e-07     1.256394e-03;"
                                          "-4.653e-07    1.875e-08    -4.549346e-02;"
                                          "-1.265812e-03 4.559086e-02  1.0", "m",
                                 "(rw (init)) fundamental matrix from left to right image in pixels (includes intrinsic matrix) (see openCV manual)");
  varEssential   = addVarA("essential", "-1.590627e-05 -4.116158e-04 -3.978277e-03;"
                                        " 9.942354e-04 -4.006113e-05  1.395653e-01;"
                                        " 4.203834e-03 -1.395616e-01 -5.824149e-05", "m",
                                 "(rw (init)) matrix from left to right image in camera coordinates (see openCV manual)");
  varLRrotation = addVarA("LRrotation", "  9.999900e-01  1.618254e-03 -4.165675e-03;"
                                        " -1.617024e-03  9.999986e-01  2.985613e-04;"
                                        "  4.166152e-03 -2.918223e-04  9.999913e-01", "m",
                          "(rw (init)) rotation matrix from left to right image in 3D coordinates (see openCV)");
  varLRtranslate = addVarA("LRtranslate", "-1.395682e-01 -3.978188e-03 4.127772e-04", "m",
                           "(rw (init)) translation vector, i.e. movement of right camera to left position (x=right, y=up, z=fwd)");
  varPreFilterSize = addVar("preFilterSize", 13.0, "d", "(rw) prefilter window size NxN for normalization");
  varPreFilterCap = addVar("preFilterCap", 41.0, "d", "(rw) normalize within this range");
  varSADWindowSize = addVar("SADWindowSize", 13.0, "d", "(rw) Sum of abs difference correlation window");
  varMinDisparity = addVar("minDisparity", -16.0, "d", "(rw) Minimum disparity (0 should be OK)");
  varNumberOfDisparities = addVar("numberOfDisparities", 144.0, "d", "(rw) number of disparities tested (from minimum up)");
  varTextureThreshold = addVar("TextureThreshold", 10.0, "d", "(rw) minimum quality for variation in SAD window");
  varUniquenessRatio = addVar("UniquenessRatio", 15.0, "d", "(rw) another post-filter value");
  varTrySmallerWindow = addVar("trySmallerWindow", 0.0, "d", "(rw) If bad correlation, then try a smaller correlation window (0=off)");
  //varSpecleWindowSize = addVar("specleWindowSize", 0.0, "d", "(rw) post-filter window size off=0");
  //varSpecleRange = addVar("specleRange", 0.0, "d", "(rw) some limit for specle filtering");
  varQ = addVarA("Q", "0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0", "m", "(r) x,y,disparity to 3D matrix");
}

////////////////////////////////////////////////////////////////

bool UFuncStereo::processImages()
{
  bool result = false;
  bool isDebug = varDebug->getBool(0);
  //const int MSL = 100;
  //char fns[MSL];
  UImagePool * imgPool = (UImagePool *) getStaticResource("imgPool", false, false);
  //
  // get images
  UImage * img1c = imgPool->getImage(varImagesLR->getInt(0), false);
  UImage * img2c = imgPool->getImage(varImagesLR->getInt(1), false);
  UImage * disp = imgPool->getImage(varImageDisp->getInt(0), true);
  UImage *img1rect = NULL, *img2rect = NULL;
  // array for rectified images set to image pool
  img1rect = imgPool->getImage(varImageDisp->getInt(3), true);
  img2rect = imgPool->getImage(varImageDisp->getInt(4), true);
  // make image the right size
  img1c->lock();
  img2c->lock();
  disp->lock();
  img1rect->lock();
  img2rect->lock();
  //
  if ((int)img1c->width() != varImageSize->getInt(1))
  {
    varImageSize->setInt((int)img1c->getHeight(), 0);
    varImageSize->setInt((int)img1c->getWidth(), 1);
  }
  initializeStereo(false);
  // initialize rectified image buffer
  img1rect->copyMeta(img1c, false);
  img1rect->setSize(imageSize.height, imageSize.width, 1, 8, "gray");
  img2rect->copyMeta(img2c, false);
  img2rect->setSize(imageSize.height, imageSize.width, 1, 8, "gray");
  // sanity check
  bool isOK = img1c != NULL;
  if (not isOK)
    // missing images - or wrong image number
    strncpy(whyString, "missing source images", MWL);
  if (isOK and imageSize.width != (int)img2c->width())
  { // size is not as expected - as calibrated
    isOK = false;
    snprintf(whyString, MWL, "Image width expected=%d but image is %d wide",
             imageSize.width, (int)img2c->width());
  }
  if (isOK and img1c->width() != img2c->width())
  { // size is not equal
    isOK = false;
    snprintf(whyString, MWL, "Image 1 is %d wide but image 2 is %d",
             (int)img1c->width(), (int)img2c->width());
  }
  //
  if (isOK)
  { // struct for 16 bit disparity
    CvMat disp16;
    // convert to BW
    if (img1c->isBW())
    { // use source image as is
      cvRemap( img1c->cvArr(), img1rect->cvArr(), mx1, my1 );
      cvRemap( img2c->cvArr(), img2rect->cvArr(), mx2, my2 );
    }
    else
    {
      img1c->toBW(img1);
      img2c->toBW(img2);
//       cvCvtColor(img1c->cvArr(), img1, CV_BGR2GRAY);
//       cvCvtColor(img2c->cvArr(), img2, CV_BGR2GRAY);
      // rectify images to get alligned epipolar lines
      cvRemap( img1->cvArr(), img1rect->cvArr(), mx1, my1 );
      cvRemap( img2->cvArr(), img2rect->cvArr(), mx2, my2 );
    }
    if (isDebug)
    { // mark images as updated - for possible push commands
      img1rect->updated();
      img2rect->updated();
    }
    // do the stereo matching
    disp->copyMeta(img1c, false);
    disp->setSize(imageSize.height, imageSize.width, 1, 16, "BW16S");
    disp->setName("disparity-16bit");
    cvInitMatHeader(&disp16, imageSize.height,  imageSize.width, CV_16S, disp->getCharRef(0,0));
    //disp16a = cvCreateMat( imageSize.height,  imageSize.width, CV_16S );
    cvFindStereoCorrespondenceBM( img1rect->getIplImage(), img2rect->getIplImage(),
                                  /*disp->cvArr()*/ &disp16, BMState);
    //cvFindStereoCorrespondenceBM( img1r, img2r, disp->cvArr(), BMState);
    disp->updated();
    //
    varTime->setTime(img1c->imgTime);
    if (cloud3d.tryLock())
    { // if unclocable, then ignore do not update 3d-cloud
      CvScalar upx, upy;
      UPixel pix(0,50,50);
      int16_t * d16;
      float p3d[3];
      CvMat point3d = cvMat(1, 1, CV_32FC3, p3d);
      float p3d2[3];
      CvMat point3d2 = cvMat(1, 1, CV_32FC3, p3d2);
      FILE * f3d = NULL;
      UImage * imgS = imgPool->getImage(varImageDisp->getInt(6), true, imageSize.height, imageSize.width, 3, 8);
      int maxDisp = 0, minDisp= 1000;
      UCamPool * camPool;
      if (isDebug)
      { // open files and debug disparity image
        f3d = fopen("3dcloud.txt", "w");
        fprintf(f3d, "x=fwd, y=left, z=up, R, G, B,  orgRow, orgCol, rowRectified, colRectified, disparity\n");
        imgS->setColorType("RGB");
        imgS->setName("3D-cloud");
        imgS->clear();
      }
      cloud3d.clear();
      cloud3d.inPoseCoordinates = true;
      cloud3d.time = img1c->getImageTime();
      cloud3d.serial = img1c->imageNumber;
      camPool = (UCamPool *)getStaticResource("camPool", false);
      if (camPool != NULL)
      { // get conversion matrix from camera device
        // (if available, else result will be in camera coordinates)
        UCamMounted * cam;
        cam = camPool->getCam(img1c->camDevice);
        if (cam != NULL)
          cloud3d.pose = cam->getPosRot();
      }
      for (int i = 0; i < imageSize.height; i+=1)
      {
        d16 = (int16_t *)disp->getCharRef(i, 0);
        int n = 0, m = 0;
        //printf("line %3d:\n", i);
        for (int j = 0 /*imageSize.width/2*/; j < imageSize.width; j+=1 /*32*/)
        { // get original color pixel
          if (d16[j] >= BMState->minDisparity * 16)
          { // a 3D can be calculated
            if (d16[j] > maxDisp)
              maxDisp = d16[j];
            if (d16[j] < minDisp)
              minDisp = d16[j];
            upx = cvGet2D(mx1, i, j);
            upy = cvGet2D(my1, i, j);
            int rx = roundi(upx.val[0]);
            int ry = roundi(upy.val[0]);
            n++;
            p3d[0] = j;
            p3d[1] = i;
            p3d[2] = d16[j]/16.0;
            cvPerspectiveTransform(&point3d, &point3d2, umatQ.cvMat());
            if (rx >= 0 and rx < (int)img1c->width() and
                ry >= 0 and ry < (int)img1c->height())
            {
              pix = img1c->getPix(ry, rx);
              if (img1c->isBGR())
                pix.swapRB();
              if (isDebug)
                imgS->setPix(ry, rx, pix);
            }
            else
              pix.clear();
            //
            if (isDebug)
            { // coordinate system turned so that x is fwd, y is left and z is up
              fprintf(f3d, "%6.3f %6.3f %6.3f   %3d %3d %3d  %5.1f %5.1f   %3d %3d %.2f\n",
                       -p3d2[2], p3d2[0], p3d2[1],
                       (int)pix.p1, (int)pix.p2, (int)pix.p3,
                       upy.val[0], upx.val[0],
                       i, j, d16[j]/16.0);
              m++;
            }
            // add point in robot style coordinates - x is forward, y is left and z is up.
            if (-p3d2[2] > 0 and -p3d2[2] < 20.0)
              cloud3d.add(-p3d2[2], p3d2[0], p3d2[1], rx, ry, (int)pix.getInt(), 100);
          }
        }
        if (isDebug)
        {
          printf("line %d had %3d and %d OK 3d points (mindisp=%d, maxdisp=%d)\n",i, n, m, minDisp, maxDisp);
        }
      }
      cloud3d.unlock();
      // inform push handler that new data is available
      setUpdated(NULL);
      if (isDebug)
      {
        fclose(f3d);
        imgS->updated();
      }
    }
    if (isDebug)
    {
      UImage * vdisp1 = NULL;
      // disparity 8-bit BW image
      UImage * vdisp = imgPool->getImage(varImageDisp->getInt(1), true);
      if (vdisp->lock())
      {
        vdisp->copyMeta(img1c, true);
        vdisp->setSize(imageSize.height, imageSize.width, 1, 8, "BW");
        vdisp->setName("disparity-8-bit");
        int16_t * d16;
        uint8_t * d8;
        // make a BW image with disparity expanded to 256 values
        d16 = (int16_t *)disp->getCharRef(0, 0);
        d8 = (uint8_t *)vdisp->getCharRef(0, 0);
        int d168div = ((BMState->numberOfDisparities + BMState->minDisparity) * 16)/256;
        for (int i = 0; i < imageSize.height * imageSize.width; i+=1)
        {
          if (*d16 <= BMState->numberOfDisparities * 16 and *d16 > 0)
          {
            int v = *d16 / (d168div);
            *d8 = v;
          }
          else
            *d8 = 0;
          d16++;
          d8++;
        }
        // for presentation
        //cvNormalize( disp->cvArr(), vdisp->cvArr(), 0, 256, CV_MINMAX );
        vdisp->updated();
        // disparity in colour
        vdisp1 = imgPool->getImage(varImageDisp->getInt(2), true);
        if (vdisp1->lock())
        {
          vdisp1->setSize(imageSize.height, imageSize.width, 3, 8, "RGB");
          vdisp1->copyMeta(img1c, false);
          vdisp1->setName("disparity-RGB");
          vdisp1->toRaibow(vdisp);
          //snprintf(fns, MSL, "disparity-%06lu.png", vdisp->imageNumber);
          //vdisp1->setName(fns);
          vdisp1->updated();
          vdisp1->unlock();
        }
        //vdisp1->savePNG(fns);
        vdisp->unlock();
      }
    }
    // make double width epipolar line image
    if (isDebug)
    {
      UImage * pair = imgPool->getImage(varImageDisp->getInt(5), true);
      if (pair != NULL)
      {
        if (pair->lock())
        {
          CvMat part;
          pair->copyMeta(img1c, false);
          pair->setName("epipolar-lines");
          // resize debug image if needed
          if (((int)pair->width() != imageSize.width * 2) or (pair->getChannels() < 3))
            pair->setSize(imageSize.height, imageSize.width * 2, 3, 8, "BGR");
          // get part of left image
          cvGetCols(  pair->cvArr(), &part, 0, imageSize.width );
          // put image 1 rectified into left part of pair image
          cvCvtColor( img1rect->cvArr(), &part, CV_GRAY2BGR );
          // get right part of image
          cvGetCols( pair->cvArr(), &part, imageSize.width,
              imageSize.width*2 );
          // put image 2 rectified into right part of pair image
          cvCvtColor( img2rect->cvArr(), &part, CV_GRAY2BGR );
          // paint some epipolar lines across image (in cyan)
          for(int j = 0; j < imageSize.height; j += 16 )
              cvLine( pair->cvArr(), cvPoint(0,j),
                      cvPoint(imageSize.width*2,j),
                      CV_RGB(0,186,((j / 16) % 4) * 64));
          pair->updated();
          pair->unlock();
        }
      }
      // reconstruct
/*      Mat xyz;
      reprojectImageTo3D(disp, xyz, Q, true);
      saveXYZ(point_cloud_filename, xyz);
      printf("\n");*/
    }
    result = true;
  }
  img2rect->unlock();
  img1rect->unlock();
  disp->unlock();
  img2c->unlock();
  img1c->unlock();
  return result;
}

///////////////////////////////////////////////////////////////////

void UFuncStereo::initializeStereo(bool paramChanged)
{
  bool sizeChanged;
  if (BMState == NULL)
  {
    BMState = cvCreateStereoBMState();
    //generate_pseudocolorTable(&pscolor);
    // rectifocation
  }
  sizeChanged = imageSize.height != varImageSize->getInt(0) or
                imageSize.width != varImageSize->getInt(1);
  if (sizeChanged)
  { // many buffers must be resized
    if (mx1 != NULL)
    { // remove images of wrong size
      cvReleaseMat( &mx1 );
      cvReleaseMat( &my1 );
      cvReleaseMat( &mx2 );
      cvReleaseMat( &my2 );
      cvReleaseMat( &img1rd );
      cvReleaseMat( &img2rd );
    }
    imageSize.height = varImageSize->getInt(0);
    imageSize.width = varImageSize->getInt(1);
    // make new buffers
    mx1 = cvCreateMat( imageSize.height,  imageSize.width, CV_32F );
    my1 = cvCreateMat( imageSize.height,  imageSize.width, CV_32F );
    mx2 = cvCreateMat( imageSize.height,  imageSize.width, CV_32F );
    my2 = cvCreateMat( imageSize.height,  imageSize.width, CV_32F );
    // BW image buffer
    img1rd = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
    img2rd = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
    // disparity result
    //disp16 = cvCreateMat( imageSize.height,  imageSize.width, CV_16S );
    // buffer for raw image in BW
    if (img1 == NULL)
      img1 = new UImage640();
    if (img2 == NULL)
      img2 = new UImage640();
  }
  if (sizeChanged or paramChanged)
  { // recalculate rectification matrix
    double * M1 = varIntrinsicLeft->getValuesd();
    double * M2 = varIntrinsicRight->getValuesd();
    // test for scaling of focallength and headpoint
    while (true)
    {  // scale intrinsic matrix if actual image is smaller than calibrated image set
      double hx = M1[2];
      float change = (hx * 2.0 - imageSize.width)/imageSize.width;
      double factor = 0.5;
      if (fabs(change) < 0.25)
        break;
      if (change < 0.0)
        factor = 2.0;
      for (int i = 0; i < 9; i++)
      { //
        M1[i] *= factor;
        M2[i] *= factor;
      }
    }
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    double * D1 = varDistortionLeft->getValuesd();
    double * D2 = varDistortionRight->getValuesd();
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    double * R = varLRrotation->getValuesd();
    CvMat _R = cvMat(3, 3, CV_64F, R );
    double * T = varLRtranslate->getValuesd();
    CvMat _T = cvMat(3, 1, CV_64F, T );
    // and temporary rotation and translation matrices
    double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
    CvMat _R1 = cvMat(3, 3, CV_64F, R1);
    CvMat _R2 = cvMat(3, 3, CV_64F, R2);
    // CALIBRATED (BOUGUET'S METHOD)
    CvMat _P1 = cvMat(3, 4, CV_64F, P1);
    CvMat _P2 = cvMat(3, 4, CV_64F, P2);
    //CvMat matQ = cvMat(4, 4, CV_64F, varQ->getValuesd());
    umatQ.init(4,4, varQ->getValuesd(), varQ->getElementCnt() - 2);
    //
    cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
        &_R, &_T,
        &_R1, &_R2, &_P1, &_P2, umatQ.cvMat(),
        0/*CV_CALIB_ZERO_DISPARITY*/ );
    //isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
    //Precompute maps for cvRemap()
    cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
    cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
  }
  // set stereo parameters every time
  BMState->preFilterSize=varPreFilterSize->getInt(); // 41;
  BMState->preFilterCap=varPreFilterCap->getInt(); // 31;
  BMState->SADWindowSize=varSADWindowSize->getInt(); // 41;
  BMState->minDisparity=varMinDisparity->getInt(); // -64;
  BMState->numberOfDisparities=(varNumberOfDisparities->getInt()/16)*16; // 128;
  if (BMState->numberOfDisparities != varNumberOfDisparities->getInt())
    varNumberOfDisparities->setValued(BMState->numberOfDisparities);
  BMState->textureThreshold=varTextureThreshold->getInt(); // 10;
  BMState->uniquenessRatio=varUniquenessRatio->getInt(); // 15;
  BMState->trySmallerWindows=varTrySmallerWindow->getBool();
  BMState->speckleWindowSize=25;
  BMState->speckleRange=200;
}

///////////////////////////////////////////

void UFuncStereo::callGotNewDataWithObject()
{
  UDataBase * data = &cloud3d;
  gotNewData(data);
}

//////////////////////////////////////////////////////////////////////



//#endif

