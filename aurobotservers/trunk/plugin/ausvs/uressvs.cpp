/**
 * This module is based on stframe.cpp with Copyright 2001 by Kurt Konolige
 * The module used Videre Design SVS library, and integrates this with
 * AURobot servers - Copyright 2008 by DTU
 **
 * For the Kurt Konolige part, the following apply:
 ** The author hereby grants to SRI permission to use this software.
 ** The author also grants to SRI permission to distribute this software
 ** to schools for non-commercial educational use only.
 **
 ** The author hereby grants to other individuals or organizations
 ** permission to use this software for non-commercial
 ** educational use only.  This software may not be distributed to others
 ** except by SRI, under the conditions above.
 **
 ** Other than these cases, no part of this software may be used or
 ** distributed without written permission of the author.
 **
 ** Neither the author nor SRI make any representations about the
 ** suitability of this software for any purpose.  It is provided
 ** "as is" without express or implied warranty.
 **
 ** Kurt Konolige
 ** Senior Computer Scientist
 ** SRI International
 ** 333 Ravenswood Avenue
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@ai.sri.com
 **
 * */

//#ifdef USE_SVS

#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>

#include <ugen4/ucommon.h>
#include <urob4/uvarcalc.h>
#include <ugen4/uimage.h>
#include <urob4/uimgpush.h>

#include "uressvs.h"

/**
 * a svs function */
void debugFn(char *str)
{
  if (str != NULL)
    fprintf(stderr, "SVS: %s\n", str);
  //debugWin->Print(str);

}

/**
 * another svs function */
void debugMessage(char *str, ...)
{
  char buf[256];
  va_list ptr;
  va_start(ptr,str);
  vsprintf(buf,str,ptr);
//  debugFn(buf);
  va_end(ptr);
}

////////////////////////////////////
////////////////////////////////////
////////////////////////////////////

UResSVS::UResSVS()
{ // reference timestamp for image time
  timeRef.now();
  // these first two lines is needed
  // to save the ID and version number
  setResID("svs", 203);
  // set description for global variables owned by this resource (optional)
  setDescription("Stereo camera (svs) interface", false);
  // other local initializations
  createBaseVar();
  // create space for resulting 3D point cloud
  points3D = new USvs3Dpoints();
  //
  strncpy(svsCalib, "svsCalib.ini", MAX_FILENAME_SIZE);
  strncpy(imgSetSubdir, "imgorg/", MAX_FILENAME_SIZE);
  logImg.setLogName(getResID());
  logHoz.setLogName("svs_hoz");
  logFran.setLogName("svs_expo");
  isStreaming = false;
  threadRunning = false;
  //videoObject = NULL; //getVideoObject();
  //if (videoObject == NULL)
  videoObject = getVideoObject();
  processObject = new svsStereoProcess();
  fileObject = NULL; // for replay
  useRealCamera = true;
//  useReplay = false;
  useImgPoolSource = -1;
  serial = 0;
  serialFran = 0;
  verbose = true;
  points3D = NULL;
  newGain = 0.0;
  timeRefAdj = 0.0;
  replayFrameSerial = 0;
}

////////////////////////////////////////////////////////

UResSVS::~UResSVS()
{
  svsAcquireImages * sourceObject;
  //
  if (isStreaming)
  {
    sourceObject = (svsAcquireImages *)videoObject;
    sourceObject->Stop();
    sourceObject->Close();
  }
  if (threadRunning)
    stop(true);
  if (processObject != NULL)
    delete processObject;
  if (videoObject != NULL)
  { // NB! this do not delete all allocaated objects inside?? (says valgrind)
    closeVideoObject();
  }
  if (fileObject != NULL)
    delete fileObject;
  if (points3D != NULL)
    delete points3D;
}

////////////////////////////////////

void UResSVS::createBaseVar()
{ // add global variables for this module
  varMaxDisp = addVar("maxDisp", 48.0, "d", "(r/w) Maximum disparity (determines minimum range)");
  varCorrSize = addVar("corrSize", 9.0, "d", "(r/w) Size of correlation window 5x5 to 21x21 - uneven");
  varFiltTexture = addVar("filtTexture", 4.0, "d", "(r/w) Texture filter (confidence) setting 0 to ???");
  varFiltUnique = addVar("filtUnique", 4.0, "d", "(r/w) Uniqueness filter setting");
  //IMPORT void SetSpeckleSize(int n); // set min disparity region size
  varFiltSprec = addVar("filtSprecSize", 4.0, "d", "(r/w) Disparity sprecle filter remove size (pixels?)");
  //IMPORT void SetSpeckleDiff(int n); // set disparity region neighbor diff
  varFiltDiff = addVar("filtSprecDiff", 30.0, "d", "(r/w) Disparity sprecle filter difference to neighbor");
  varImgLeft = addVar("imgLeft", 31.0, "d", "(r/w) Image number in image pool for left image");
  varImgRight = addVar("imgRight", 32.0, "d", "(r/w) Image number in image pool for right image");
  varImgDisparity = addVar("imgDisparity", 33.0, "d", "(r/w) Image number in image pool for disparity image");
  varWidth = addVar("width", 320.0, "d", "(r/w) Image width for capture and stereo processing (assumes 3/4 image format) - 640 or 320 are valid values");
  varRate = addVar("rate", 3.75, "d", "(r/w) Image framerate 3=3.75, 7=7.5, 10, 15, 30 (set before open)");
  varImgSerial = addVar("imgSerial", 1.0, "d", "(r) Latest image serial number");
  varShutter = addVar("camShutter", -1.0, "d", "(r) camera exposure/shutter setting (0..100, -1 is auto) (set from 'svs shutter')");
  varGain = addVar("camGain", -1.0, "d", "(r) video gain setting (0..100, -1 is auto) (set from 'svs gain')");
  varDelay = addVar("camDelay", 0.1, "d", "(r/w) Delay from shutter close to image timestamp [seconds] (from HW)");
  varDelayReplay = addVar("camDelayReplay", 0.0, "d", "(r/w) Additional delay during replay");
  // exposure control - horizon estimate
  varFranExp = addVar("shutterCtrl", 2.0, "d", "(r/w) Fran Below horizon shutter/exposure control value (0=off, 1=max, 2=mean, 3=log only)");
  varHorizon = addVar("horizon", 100.0, "d", "(r/w) Horizon image line (manual default)");
  varHorizonLow = addVar("horizonLow", 20.0, "d", "(r/w) Horizon lowest estimate (percentage of image height)");
  varHorizonBluePct = addVar("skyBluePct", 0.850, "d", "(r/w) To be sky, the blue intensity must be above this limit, relative to the most bright blue pixel in the image top row.");
  varHorizonEst = addVar("horizonEstimate", -1.0, "d", "(r) Horizon estimate by Fran Auto horizon");
  varHozGreenLimit = addVar("hozGreenLimit", 70.0, "d", "(r/w) sky value limit (for horizon estimate)");
  varFranHoz = addVar("horizonAuto", 1.0, "d", "(r/w) Fran Auto horizon control (1 = auto, 0 = manual");
  varSaturationBins = addVar("saturationBins", 1.0, "d", "(r/w) Number of bins counted as saturated, 1 is just 255, 2 is bin 254 and 255");
  // exposure control - exposure and gain
  varRefIntensity = addVar("refIntensity", 105.0, "d", "(r/w) Fran control reference intensity");
  varRefIntensityBand = addVar("refIntensityBand", 15.0, "d", "(r/w) no-change band each side of ref intensity");
  varRefShutter = addVar("refShutter", 30.0, "d", "(r/w) reference shutter value for gain adjustment");
  varRefShutterIGain = addVar("vgainIGain", 0.03, "d", "(r/w) Shutter I-gain for video gain adjust");
  varRefVideoGainMax = addVar("vgainIMax", 70.0, "d", "(r/w) Max video gain for automatic gain control");
  varFranGain = addVar("franGain", 1.0/6.1621, "d", "(r/w) Fran control gain (delta-bin to delta-exposure");
  varControlDelay = addVar("ctlDelay", 3.0, "d", "(r/w) Delay from shutter/exposure control to image [in images]");
  varMeanIntensity = addVar("imgIntensity", 0.0, "d", "(r) mean intensity below horizon [0..255]");
  varMakeHistogram = addVar("makeHistogram", 1.0, "d", "(r/w) Make intensity histogram in pool image 35");
  // sensor pose
  varPose6d = addVar("pose", 0.0, "6d", "(r/w) Pose (x,y,z,o,p,k) of stereo camera (left camera).");
  //
  //
  addMethod("get3d", "dc", "Function to get the 3d point cloud from the latest stereo calculation. Image serial number must be larger than the value in the 'd' parameter. If 'd' is negative then any result will do. Return value is 'true' (1.0) and (a pointer to) the 3D cloud will be returned as class 'svs3d' (compatible with UImg3Dpoints) at the position pointet to by the 'c parameter. Data is unlocked and may be overwritten by new calculation if not locked while used.");
  addMethod("step", "d", "Replay one or more steps. parameter is number of steps. Returns 1 (true) if successful and 0 (false) if no steps are availabl");
  addMethod("verbose", "d", "Output more messages to console if true. Parameter is false (0.0) or true (1.0)");
}

////////////////////////////////////

bool UResSVS::setResource(UResBase * resource, bool remove)
{
  bool result;
  result  = UServerPush::setResource(resource, remove);
  result &= UResVarPool::setResource(resource, remove);
  return result;
}


//////////////////////////////////////////////////

USvsImageSet * UResSVS::getImageSet(USvsImageSet * pushIsi, bool toImgPool, bool doReplay)
{
  bool isOK = false;
  svsAcquireImages *sourceObject; // source of images
  USvsImageSet * result = pushIsi;
  int h, w;
  UImagePool * imgPool;
  UImage * img;
  const int MFL = MAX_FILENAME_SIZE;
  char fs1[MFL];
  char fs2[MFL];
  int ctlDelay;
  UTime t;
  //
  imgPool = (UImagePool*) getStaticResource("imgPool", true);
  //
  if (imgPool != NULL)
  { // pushed image har priority
    if (result == NULL)
    { // no image, so get one
      // get new imageset from hardware
      if (not threadRunning)
        // used in replay as well as real camera
        start();
      //
      if (useRealCamera)
      { // get the svsVideoImages object from the currently loaded camera interface
        sourceObject = (svsAcquireImages *) videoObject;
        //
        isOK = (sourceObject != NULL);
        if (isOK and not isStreaming)
        {
          isOK = startStreaming();
          if (isOK)
            Wait(2.0);
        }
        //
        if (isOK)
        { // request lens distortion correction (otherwise no stereo correlation)
          isOK = sourceObject->SetRect(true);
          if (not isOK)
            fprintf(stderr, "svsStereo:: Can't set rectification - calibration file read failed\n");
        }
      }
      result = &dataSi;
    }
    //
    if (result != NULL)
    { // get the next frame
      // lock the data structure - inhibits capture (for a while)
      result->lock();
      // must be unlocked by the caller
      if (result->si == NULL)
        // no image so unlock - we have catched the thread before first image
        result->unlock();
      else
      { // image structure is available
        if (toImgPool)
        { // make a copy to image-pool image buffers
          w = varWidth->getInt();
          h = (w * 3) / 4;
          if (result->si->haveColor and result->si->color != NULL)
          {
            img  = imgPool->getImage(varImgLeft->getInt(), true);
            toImagePool(img, h, w, PIX_PLANES_BGRA, result->si->color);
            img->imgTime = result->imageTime;
            img->imageNumber = result->serial;
            img->camDevice = 11; // 11 is left
            strncpy(img->name, "left", MAX_IMG_NAME_SIZE);
            img->updated();
            if (logImg.isLogOpen())
            {
              ctlDelay = maxi(0, mini(MAX_CTRL_DELAY, varControlDelay->getInt()));
              img->imgTime.getForFilename(fs1, true);
              //snprintf(fs2,MFL, "%s/svs_%08lu_%s-C.bmp", imagePath, img->imageNumber, fs1);
              //img->saveBMP(fs2);
              snprintf(fs2,MFL, "%s/svs_%08lu_%s", imagePath, img->imageNumber, fs1);
              // save form library direct
              result->si->SaveToFile(fs2);
              //snprintf(fs2,MFL, "%s/svs_%08lu_%s.ini", imagePath, img->imageNumber, fs1);
              //result->si->SaveParams(fs2);
              // make entry in log
              fprintf(logImg.getF(), "%lu.%06lu svsSet %8lu %3d %3d svs_%08lu_%s\n",
                        img->imgTime.getSec(), img->imgTime.getMicrosec(), img->imageNumber,
                        exposureCtrl[ctlDelay], getGain(), img->imageNumber, fs1);
            }
          }
          if (result->si->haveColorRight and result->si->color_right != NULL)
          {
            t.now();
            img  = imgPool->getImage(varImgRight->getInt(), true);
            toImagePool(img, h, w, PIX_PLANES_BGRA, result->si->color_right);
            img->imgTime = result->imageTime;
            img->imageNumber = result->serial;
            img->camDevice = 12; // 12 is right
            strncpy(img->name, "right", MAX_IMG_NAME_SIZE);
            img->updated();
            if (verbose)
              printf("[OK] %7.3f ms (save to pool R)\n", t.getTimePassed() * 1000.0);
            t.now();
            if (false and logImg.isLogOpen())
            {
              snprintf(fs2,MFL, "%s/svs_%08lu_%s-Q.bmp", imagePath, img->imageNumber, fs1);
              img->saveBMP(fs2);
              // print an entry in text-log
              fprintf(logImg.getF(), "%lu.%06lu imgR %8lu %3d %3d svs_%08lu_%s\n",
                        img->imgTime.getSec(), img->imgTime.getMicrosec(), img->imageNumber,
                        getShutter(), getGain(), img->imageNumber, fs1);
              if (verbose)
              {
                printf("[OK] %7.3f ms (image save R)\n", t.getTimePassed() * 1000.0);
                printf("[OK] %7.3f ms intensity control calculation time\n", exposureCalcTime * 1000.0);
              }
            }
          }
          // try relaod images
          if (false and logImg.isLogOpen())
          {
            snprintf(fs2,MFL, "%s/svs_%08lu_%s", imagePath, img->imageNumber, fs1);
            isOK = result->si->ReadFromFile(fs2);
            if (isOK)
              printf("SVS Images and param read from file %s\n", fs2);
            else
              printf("SVS FAILED reading images and param from file %s\n", fs2);
          }
        }
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////////

bool UResSVS::doDisparityCalculation(USvsImageSet * usi, bool doPoolImages)
{ // processing assumes that image set (usi) is locked during this call
  bool isOK;
  //svsAcquireImages *sourceObject; // source of images
  UTime t;
  int h, w;
  UImagePool * imgPool;
  UImage * img;
  const int MFL = MAX_FILENAME_SIZE;
  char fs1[MFL];
  char fs2[MFL];
  int r, c;
  svs3Dpoint * p3d;
  //svs3Dpoint * p3dBuff;
  //
  // get the svsVideoImages object from the currently loaded camera interface
  //sourceObject = (svsAcquireImages *)videoObject;
  //

  isOK = (usi->si != NULL);
  if (isOK)
  { // allow recalculation with new parameters
    // - too low level
    //if (usi->si->haveDisparity)
    //  usi->si->ReleaseDisparity();
    // set filter variables
    usi->si->SetNDisp(varMaxDisp->getInt());      // maximum disparities
    usi->si->SetCorrsize(varCorrSize->getInt()); // (15) correlation window size
    usi->si->SetLR(false);   // no left-right check, not available
    usi->si->SetThresh(varFiltTexture->getInt()); // 10 texture filter
    usi->si->SetUnique(varFiltUnique->getInt());  // 13 uniqueness filter
    usi->si->SetHoropter(0); // 0 horopter offset - 0-disparity is not infinity
    usi->si->SetSpeckleSize(varFiltSprec->getInt()); // not sure
    usi->si->SetSpeckleDiff(varFiltDiff->getInt());  // not sure
  }
  if (isOK)
  { // ensure we have a stereo processing object
    isOK = (processObject != NULL);
    if (not isOK)
      printf("UResSVS::doDisparityCalculation: No stereo procesing object!\n");
  }
  if (isOK)
  {
    isOK = usi->si != NULL;
    if (not isOK)
      printf("UResSVS::doDisparityCalculation: No imageset to process!\n");
  }
  if (isOK)
  { // Compute the stereo
    if (verbose)
      printf("Dooing stereo ...\n");
    //fflush(stdout);
    t.now();
    usi->si->doConfidence = true;
    processObject->CalcStereo(usi->si);
    if (verbose)
      printf("[OK] %7.3f ms (disparity)\n", t.getTimePassed() * 1000.0);
    //fflush(stdout);
    t.now();
    w = varWidth->getInt();
    h = (w * 3) / 4;
    /*    n = sizeof(svs3Dpoint) * 2;
        n *= w * h;
        p3dBuff = (svs3Dpoint*) malloc(n);*/
    processObject->Calc3D(usi->si, 0, 0, w, h);
    if (verbose)
      printf("[OK] %7.3f ms (conv to 3D)\n", t.getTimePassed() * 1000.0);
    //
    p3d = usi->si->pts3D;
    //
    if (points3D == NULL)
    {
      points3D = new USvs3Dpoints();
    }
    // debug
    /// removed the 3d point cloud generation, as it gives segmentation fault
    /// maybe due to imcompatible libraries?
    // debug end
    if (false and points3D != NULL)
    { /// removed!
      points3D->lock();
      points3D->clear3d(usi->serial, usi->imageTime);
      points3D->pose = varPose6d->get6D();
      for (r = 0; r < h; r++)
      {
        for (c = 0; c < w; c++)
        {
          if (p3d->A > 0 and p3d->Z > 0.0)
          { // a valid point (x is lfet, y is down, z is forward in svs data)
            points3D->add3d(r, c, w, p3d, usi->si->Color());
            // printf("3D %3dr %3dc: %gx %gy %gz %da\n", r, c, p3d->X, p3d->Y, p3d->Z, p3d->A);
          }
          p3d++;
        }
      }
      points3D->unlock();
    }
    //
    isOK = not doPoolImages;
    if (doPoolImages and usi->si->haveDisparity)
    {
      imgPool = (UImagePool*) getStaticResource("imgPool", true);
      if (imgPool != NULL)
      {
        t.now();
        w = varWidth->getInt();
        h = (w * 3) / 4;
        img  = imgPool->getImage(varImgDisparity->getInt(), true);
        toImagePool(img, h, w, PIX_PLANES_BW16S,(unsigned char *) usi->si->disparity);
        img->imgTime = usi->imageTime;
        img->imageNumber = usi->serial;
        img->camDevice = 11; // 11 left - assumed to be reference camera
        strncpy(img->name, "disparity", MAX_IMG_NAME_SIZE);
        img->updated();
        if (verbose)
          printf("[OK] %7.3f ms (save to pool)\n", t.getTimePassed() * 1000.0);
        t.now();
        if (logImg.isLogOpen())
        {
          img->imgTime.getForFilename(fs1, true);
          snprintf(fs2,MFL, "%s/svs_%08lu_%s-D.bmp", imagePath, img->imageNumber, fs1);
          img->saveBMP(fs2);
          fprintf(logImg.getF(), "%lu.%06lu disparity %8lu svs_%08lu_%s-D.bmp\n",
                    img->imgTime.getSec(), img->imgTime.getMicrosec(), img->imageNumber, img->imageNumber, fs1);
        }
        isOK = true;
        if (verbose)
          printf("[OK] %7.3f ms (save disparity image)\n", t.getTimePassed() * 1000.0);
      }
      if (false)
      {
        to3Dfile(usi);
      }
      else if (false)
      { // save using svs library
        snprintf(fs2,MFL, "%s/svs_%08u_%s-D.da1", imagePath, usi->serial, fs1);
        isOK = usi->si->Save3DPointCloud(fs2);
        if (not isOK)
          printf("Failed to save 3D cloud\n");
        snprintf(fs2,MFL, "%s/svs_%08u_%s-D.da2", imagePath, usi->serial, fs1);
        usi->si->Save3DPointArray(fs2);
        if (not isOK)
          printf("Failed to save 3D array\n");
      }
    }
  }

  return isOK;
}

////////////////////////////////////

// void UResSVS::print(const char * preString, char * buff, int buffCnt)
// {
//   snprintf(buff, buffCnt, "%s svs plugin - needs more infor here\n", preString);
// }

/////////////////////////////////////

const char * UResSVS::getImgLogName()
{
  return logImg.getLogFileName();
}

//////////////////////////////////////

const char * UResSVS::getHozLogName()
{
  return logHoz.getLogFileName();
}

//////////////////////////////////////

const char * UResSVS::getExposureLogName()
{
  return logFran.getLogFileName();
}

//////////////////////////////////////

bool UResSVS::startStreaming()
{
  char * name = NULL;
  bool isOK;
  svsAcquireImages *sourceObject; // source of images
  int captureWidth;
  double dt;
  int captureRate;
  //
  // streaming is always from real camere devices, so we need a
  // video object.
  if (videoObject == NULL)
    videoObject = getVideoObject();
  //
  isOK = (videoObject != NULL);
  if (not isOK)
    fprintf(stderr, "UResSVS::startStreaming: Failed to create video object\n");
  //
  if (isOK)
  { // get the svsVideoImages object from the currently loaded camera interface
    sourceObject = (svsAcquireImages *) videoObject;
    // Try to read a parameter file into the system
    // You can change the name here
    // If no file is found, defaults will be used - i.e. no lens correction
    sourceObject->ReadParams((char*) svsCalib);
    //
    // debug
/*    printf("Image time ref posponded by %f sec\n", timeRef.getTimePassed());
    timeRef.now();*/
    // debug end
    // open device (or possibly streming file)
    isOK = sourceObject->Open(name);
    // debug
    dt = timeRef.getTimePassed();
    printf("Time to svs-open is %.4f sec\n", dt);
    // debug end
    if (not isOK)
      fprintf(stderr, "Can't open device or file\n");
    else
      fprintf(stderr, "Opened stereo device\n");
  }
  if (isOK)
  { // devices are now open, but not streaming
    //
    // We can set the frame size and sampling modes here
    captureWidth = varWidth->getInt();
    captureRate = varRate->getInt();
    // Each device has a default mode
    videoObject->SetRate(7);   // frame rate 3(.75), 7(.5), 15, 30
    videoObject->SetFrameDiv(captureRate);                             // Full frame
    videoObject->SetSize(captureWidth,(captureWidth * 3) / 4);  // 320x240 image

    // Set up to get color, if we can, from hardware device
    if (useRealCamera)
      videoObject->SetColor(true, true); // both left and right
  }
  //
  if (isOK)
  { // start streaming mode
    isOK = sourceObject->Start();
    if (not isOK)
      fprintf(stderr, "svsStereo:: Can't start continuous capture\n");
  }
  isStreaming = isOK;
  //
  return isStreaming;
}

/////////////////////////////////////////////////////

void UResSVS::stopStreaming()
{
  svsAcquireImages * sourceObject;
  //
  if (isStreaming)
  {
    sourceObject = (svsAcquireImages *) videoObject;
    isStreaming = false;
    sourceObject->Stop();
    sourceObject->Close();
  }
}

/////////////////////////////////////////////////////

bool UResSVS::toImagePool(UImage * img, int height, int width, int format,
                            const unsigned char * source)
{
  bool isOK = false;
  //UImagePool * imgPool;
  //UImage * img = NULL;
  int n, h, i, j, m;
  UTime imgT;
  //int captureWidth;
  unsigned long * ii, iv;
  short * is, isv;
  UPixel * ip;
  //CvScalar col = CV_RGB(255, 0 , 0);
//  int horizonLine;
  //
  // get pointers to result images
  if (img != NULL)
  {
    //captureWidth = varWidth->getInt();
    imgT.now();
    img->imgTime = imgT;
    img->readDelay = 0.0;
    img->radialErrorRemoved = true;
    img->valid = true;
    img->used = false;
    /*    if (name != NULL)
          strncpy(img->name, name, MAX_IMG_NAME_SIZE);*/
    if (format == PIX_PLANES_BW16S)
    { // disparity image
      n = width * height * 3;
      n = mini(n, img->getBufferSize()) / 3;
      is = (short *) source;
      ip = img->getData();
      for (i = 0; i < n / width; i++)
      {
        m = varMaxDisp->getInt() << 4;
        for (j = 0; j < width; j++)
        {
          isv = *is;
          if (isv < 0)
          {
            ip->p1 = 0;
            ip->p2 = 0;
            ip->p3 = 0;
          }
          else
          {
            // make high disp to red and low to blue
            ip->p1 = maxi(0, 255 -((m - isv) >> 2));
            ip->p2 = mini(255, absi(isv - m/2) / 2);
            ip->p3 = mini(255, 255 -(isv >> 2));
          }
          ip++;
          is++;
        }
      }
      img->setColorType(PIX_PLANES_RGB);
    }
    else if (format == PIX_PLANES_BGRA)
    { // integer pixel size BGR image
      n = width * height * 3;
      n = mini(n, img->getBufferSize()) / 3;
      ii = (unsigned long *) source;
      ip = img->getData();
      for (i = 0; i < n / width; i++)
      {
        for (j = 0; j < width; j++)
        {
          iv = *ii;
          ip->p1 = iv & 0xff;
          ip->p2 = (iv >> 8) & 0xff;
          ip->p3 = (iv >> 16) & 0xff;
          ip++;
          ii++;
        }
      }
      img->setColorType(PIX_PLANES_RGB);
      /*      horizonLine = getLocalValueInt(varHorizon);
            cvLine(img->cvArr(), cvPoint(0, horizonLine),
                   cvPoint(width-1, horizonLine), col, 1, 4, 0);*/
    }
    else
    {
      // number of bytes to copy
      n = (img->getDepth() / 8) * width * height;
      // limit to image buffer size
      n = mini(n , img->getBufferSize());
      memcpy(img->getData(), source, n);
      img->setColorType(format);
    }
    if (not img->setSizeOnly(height, width))
    { // size too big, reduce the height to fit
      h = img->getBufferSize() /(width *(img->getDepth() / 8));
      img->setSizeOnly(h, width);
    }
    //img->toBGR();
//    img->updated();
    isOK = true;
  }
  return isOK;
}

//////////////////////////////////////////////////////

bool UResSVS::to3Dfile(USvsImageSet * usi)
{
  int r, c, w, h, n=0;
  FILE * f3d;
  svs3Dpoint * s3d;
  const int MFL = MAX_FILENAME_SIZE;
  char fs1[MFL];
  char fs2[MFL];
  //
  usi->imageTime.getForFilename(fs1, true);
  snprintf(fs2,MFL, "%s/svs_%08u_%s_D.dat", imagePath, usi->serial, fs1);
  f3d = fopen(fs2, "w");
  if (f3d != NULL)
  {
    w = varWidth->getInt();
    h = (w * 3) / 4;
    s3d = usi->si->pts3D;
    for (r = 0; r < h; r++)
      for (c = 0; c < w; c++)
      {
        if (s3d->A > 0)
          fprintf(f3d, "%d %d %g %g %g %g\n", r, c, s3d->A/16.0, s3d->Z, -s3d->X, -s3d->Y);
        n++;
        s3d++;
      }
    fclose(f3d);
  }
  if (verbose)
    printf("Wrote (%s) %d coordinates to %s\n", bool2str(f3d != NULL), n, fs2);
  return f3d != NULL;
}

////////////////////////////////////////////////////

void UResSVS::run()
{ //
  threadRunning = true;
  int ni = 0, nc = 0;
  //svsAcquireImages * sourceObject;
  UTime tImg, t;
  bool gotData;
  int hozLine; // horizon line in image - to avoid sky-based exposure control
  bool autoHoz;
  //
  // wait to allow init script to finish
  // open logfile
/*  if (not logFran.isLogOpen())
  {
    logFran.openLog("svs_exp");
    saveToExpFile(logFran.getF(), 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0);
  }
  if (not logHoz.isLogOpen())
  {
    logHoz.openLog("svs_horizon");
    saveToHozFile(logHoz.getF(), 0);
  }
  if (not logTime.isLogOpen())
  {
    logTime.openLog("svs_imgDelay");
    logTime.toLog("#imageTime, imageSerial, setDelay, actualDelay, timeOffset, svsTimestamp");
  }*/
  Wait(1.2);
  t.now();
  //sourceObject = /*(svsAcquireImages *)*/videoObject;
  while(not threadStop)
  {
    if (not isStreaming and not replay)
    { // port not open, just wait
      Wait(0.1);
    }
    else
    { // get data from line
      if (dataSi.tryLock())
      {
        serial++;
        /*        if (nc > 0)
                  printf("Image found locked (in %.2f seconds) - now OK\n", t.getTimePassed());*/
        nc = 0;
        //
        if (useRealCamera)
        { // get imageset from real hardware
          gotData = getImageSetFromHardware(&dataSi);
          if (gotData)
          { // debug of exposure control
            //doHorizonEstimate(&dataSi);
            //(&dataSi);
            hozLine = doHorizonEstimateBG(&dataSi, true);
            autoHoz = varFranHoz->getBool();
            if (autoHoz)
              hozLine = varHorizonEst->getInt();
            else
              hozLine = varHorizon->getInt();
            //
            doFranExposureControl(&dataSi, hozLine, true);
            if (logHoz.isLogOpen())
            {
              saveToHozFile(logHoz.getF(), serial);
            }
          }
        }
        else if (replay)
        {
          //
          gotData = getImageSetFromStream(&dataSi, dataSi.serial);
          serial = replayFrameSerial;
          dataSi.serial = serial;
          if (gotData)
          {
            // debug of exposure control
            //hozLine = doHorizonEstimate(&dataSi);
            //hozLine = doHorizonEstimate3(&dataSi);
            hozLine = doHorizonEstimateBG(&dataSi, true);
            autoHoz = varFranHoz->getBool();
            if (autoHoz)
              hozLine = varHorizonEst->getInt();
            else
              hozLine = varHorizon->getInt();
            //
            doFranExposureControl(&dataSi, hozLine, true);
            if (logHoz.isLogOpen())
            {
              saveToHozFile(logHoz.getF(), serial);
            }
          }
        }
        else if (useImgPoolSource >= 0)
        { // not implemented
          gotData = getImageSetFromImgPool(&dataSi);
        }
        else
        {
          gotData = false;
        }
        //
        if (gotData)
        {
          // a new image is available
          setUpdated("");
          // update with imageset serial number
          dataSi.serial = serial;
          varImgSerial->setValued(serial, 0);
        }
        dataSi.unlock();
        // wait a bit to let others get and lock image
        Wait(0.01);
      }
      else
      { // wait for processing to finish using the latest image
        ni++;
        if (nc == 0)
          //printf("Image found locked\n");
          t.now();
        else if ((nc % 1000) == 0)
        {
          printf("Image found locked (in %.2f seconds)\n", t.getTimePassed());
          if (nc >= 2000)
            // force unlock - to be able to continue
            dataSi.unlock();
        }
        Wait(0.01);
        nc++;
      }
    }
  }
  threadRunning = false;
}

///////////////////////////////////////////////////

bool UResSVS::getImageSetFromHardware(USvsImageSet * ssi)
{
  svsAcquireImages * sourceObject;
  UTime tImg, t;
  bool result;
  double imgDelay;
  // const double MIN_IMAGE_TIMEDELAY = 0.05; // seconds
  double transmisDelay;
  const double allowedJitter = 0.05;
  const int MSL = 200;
  char s[MSL];
  double acqt;
  //double sinceRef;
  //
  sourceObject = /*(svsAcquireImages *)*/videoObject;
  ssi->si = sourceObject->GetImage(500); // 500 ms timeout
  // tell push queue that a new image is available
  result = (ssi->si != NULL);
  if (result)
  { // set image time
    t.now();
    transmisDelay = varDelay->getValued();
    acqt = sourceObject->acqTime / 1000.0;
    //sinceRef = t - timeRef;
    tImg = timeRef + acqt + timeRefAdj;
    imgDelay = t - tImg;
    if (logTime.isLogOpen())
      fprintf(logTime.getF(), "%lu.%06lu %6lu %8.5f %8.5f %8.5f %8.5f %6d\n",
              tImg.getSec(), tImg.getMicrosec(), serial,
                          transmisDelay, imgDelay,
                          imgDelay - transmisDelay, timeRefAdj,
                          sourceObject->acqTime);
    if (logImg.isLogOpen())
    {
      snprintf(s, MSL, "%lu.%06lu + %.4f = %lu.%06lu %.5f",
                 timeRef.getSec(), timeRef.getMicrosec(),
                 sourceObject->acqTime / 1000.0,
                 tImg.getSec(), tImg.getMicrosec(), imgDelay);
      logImg.toLog(s);
    }
    if (imgDelay < transmisDelay - allowedJitter)
    { // getting ahead of real time - delay image by the surplus value
      snprintf(s, MSL, "SVS:: Image %6lu is early (%7.4fs) adjust value %.4fs",
               serial, imgDelay - transmisDelay, timeRefAdj);
      printf("%s\n", s);
      //if (timeRefAdj < 2.0)
      timeRefAdj += imgDelay - transmisDelay;
      if (logImg.isLogOpen())
      {
        logImg.toLog(s);
      }
    }
    else if (imgDelay > transmisDelay + allowedJitter)
    { // is the delay too big - too much jitter - adjust timestamp reference time
      snprintf(s, MSL, "SVS:: Image %6lu is late  (%7.4fs) adjust value %.4fs",
               serial, imgDelay - transmisDelay, timeRefAdj);
      printf("%s\n", s);
      //if (timeRefAdj > -2.0)
      timeRefAdj += imgDelay - transmisDelay;
      if (logImg.isLogOpen())
      {
        logImg.toLog(s);
      }
    }
    else
    {
/*      snprintf(s, MSL, "SVS:: Image %6lu is OK     (%7.4fs) adjust value %.3fs", serial, imgDelay - transmisDelay, timeRefAdj);
      printf("%s\n", s);*/
      timeRefAdj += (imgDelay - transmisDelay) * 0.02;
    }
    ssi->imageTime = tImg;
  }
  return result;
}

///////////////////////////////////////

bool UResSVS::getImageSetFromStream(USvsImageSet * ssi, unsigned int lastSerial)
{
//  UTime tImg, t;
  bool result = false;
  /*  const int MFL = MAX_FILENAME_SIZE;
    char fnIni[MFL];*/
  if (replayFrameSerial > lastSerial)
  {
    result = (strlen(replayImgBaseName) > 3);
    if (result)
    {
      if (fileObject == NULL)
      { // create a stereo image data structure
        fileObject = new svsFileImages();
      }
      result = fileObject->Open(replayImgBaseName);
      if (result)
      {
        ssi->si = fileObject->GetImage(100);
        //ssi->serial = replayFrameSerial;
        ssi->imageTime = replayTimeNow;
        result = (ssi->si != NULL);
        if (result)
          result = ssi->si->color != NULL and ssi->si->color_right != NULL;
      }
    }
    if (verbose and result)
    { //
      printf("got image set from file %s\n", replayImgBaseName);
    }
    if (not result)
    { //
      printf("** Failed to get image set from file %s\n", replayImgBaseName);
    }
  }
  return result;
}

/////////////////////////////////////////

bool UResSVS::getImageSetFromImgPool(USvsImageSet * ssi)
{
  printf("@todo - implement image source form image pool *************\n");
  ssi->si = NULL;
  return false;
}

///////////////////////////////////////////////////

void * threadRunSvs(void * obj)
{ // call the hadling function in provided object
  UResSVS * ce = (UResSVS *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UResSVS::start()
{
  bool result = true;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandle, &thAttr,
                                &threadRunSvs,(void *) this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////

void UResSVS::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

////////////////////////////////////////////////////

void UResSVS::callGotNewDataWithObject()
{
  //dataSi.lock();
  // start the push handling
  /*  if (dataSi.si == NULL)
    { // no data ready - let the function request
      dataSi.unlock();
      gotNewData(NULL);
    else
    { // use this data
      gotNewData(&dataSi);
      // finished with the data structure, so release.
      dataSi.unlock();
    }*/
  // let function acquire its own data - equally good compared to this approach
  gotNewData(NULL);
}

/////////////////////////////////////////////////////

void UResSVS::setShutter(int value)
{
  svsVideoImages * sourceObject;
  int exp;
  // get the svsVideoImages object from the currently loaded camera interface
  sourceObject = (svsVideoImages *) videoObject;
  //
  if (sourceObject != NULL)
  {
    exp = mini(100, maxi(-1, value));
    varShutter->setValued(double(exp), 0);
    if (sourceObject->autoexposure and exp == -1)
      sourceObject->SetExposure(sourceObject->exposure, sourceObject->gain, true, sourceObject->use_autogain);
    else if (sourceObject->manualexposure)
    {
      sourceObject->SetExposure(exp, sourceObject->gain, false, sourceObject->use_autogain);
    }
    else
    { // could not set exposure - not supported by camera structure
      printf("UResSVS::setShutter: could not set exposure - not supported by camera structure\n");
    }
  }
  else
    printf("UResSVS::setShutter: could not get camera structure\n");
}

/////////////////////////////////////////////////////

int UResSVS::getShutter()
{
  svsVideoImages * sourceObject;
  int exp = -1;
  //
  sourceObject = (svsVideoImages *) videoObject;
  if (sourceObject != NULL)
    exp = sourceObject->exposure;
  return exp;
}

/////////////////////////////////////////////////////

bool UResSVS::isShutterAuto()
{
  svsVideoImages * sourceObject;
  bool expAuto = true;
  //
  sourceObject = (svsVideoImages *) videoObject;
  if (sourceObject != NULL)
    expAuto = sourceObject->use_autoexposure;
  return expAuto;
}

/////////////////////////////////////////////////////

void UResSVS::setGain(int value)
{
  svsVideoImages * sourceObject;
  int gain;
  // get the svsVideoImages object from the currently loaded camera interface
  sourceObject = (svsVideoImages *) videoObject;
  //
  if (sourceObject != NULL)
  {
    gain = mini(100, maxi(-1, value));
    varGain->setValued(double(gain), 0);
    if (sourceObject->autogain and gain == -1)
      sourceObject->SetExposure(sourceObject->exposure, sourceObject->gain, sourceObject->use_autoexposure, true);
    else if (sourceObject->manualexposure)
    {
      sourceObject->SetExposure(sourceObject->exposure, gain, sourceObject->use_autoexposure, false);
    }
    else
    { // could not set exposure - not supported by camera structure
      printf("UResSVS::setGain: could not set gain - not supported by camera structure\n");
    }
  }
  else
    printf("UResSVS::setGain: could not get camera structure\n");
}

/////////////////////////////////////////////////////

int UResSVS::getGain()
{
  svsVideoImages * sourceObject;
  int gain = -1;
  //
  sourceObject = (svsVideoImages *)videoObject;
  if (sourceObject != NULL)
    gain = sourceObject->gain;
  return gain;
}


/////////////////////////////////////////////////////

bool UResSVS::isGainAuto()
{
  svsVideoImages * sourceObject;
  bool gainAuto = true;
  //
  sourceObject = (svsVideoImages *)videoObject;
  if (sourceObject != NULL)
    gainAuto = sourceObject->use_autogain;
  return gainAuto;
}

/////////////////////////////////////////////////////

int UResSVS::doFranExposureControl(USvsImageSet * ssi, const int hozLine, bool debugImage)
{
  int hozLineEst, hozLineDef;
  int iw, ih;
  unsigned long * ii, *i0, iv;
  int i, j;
  int p1, p2, p3, pg;
  int binT = 0; // pixel count total
  const int M = 1; // take every M pixel only
  int mode, mm;
  int modeRef, modeRefBand;
  int binSky = 0;
  int saturated; // intensity levels for   saturated pixels
  //const int saturated = 10; // intensity levels for
  double binSkyN;
  double errGain; // gain
  double errLinear;          // linear proportional error
  double errSaturated; // extra error value in saturated region
  double modeValid;
  int controlMode;
  int newControl = 0;
  int delay;
  int autoControl;
  int meanSum, meanN, mean;
  int gain;
  UTime t;
  bool mkeHist;
  double videoGainIgain;
  int refShutter;
  int videoGainMax;
  // debug
  UImage * img = NULL;
  UImagePool * imgPool;
  CvScalar blue = CV_RGB(255, 0 , 0);
  CvScalar red = CV_RGB(0, 0 , 255);
  CvScalar mag = CV_RGB(175, 0 , 175);
  CvScalar black = CV_RGB(0, 0 , 0);
  CvFont font;
  const int MSL = 80;
  char s[MSL];
  // debug
  UImgPush * imgp;
  // det ser ud til at der sker noget herfra til
  // enden af denne if (img->updated())
  UServerPushQueue * imgpq;
  // debug end
  //
  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,
               0.5, 0.5, 0.0, 1, 8);
  // debug end
  // image size
  iw = varWidth->getInt();
  if (iw > 640)
    iw = 640;
  ih = (iw * 3) / 4;
  //
  mkeHist = varMakeHistogram->getBool() and debugImage;
  saturated = varSaturationBins->getInt();
  // debug
  if (mkeHist)
  {
    imgPool = (UImagePool *)getStaticResource("imgPool", false);
    if (imgPool != NULL)
    {
      img  = imgPool->getImage(35, true, ih, iw);
      toImagePool(img, ih, iw, PIX_PLANES_BGRA, ssi->si->color);
      img->imgTime = ssi->imageTime;
      img->imageNumber = ssi->serial;
      img->camDevice = 11; // 11 is left
      strncpy(img->name, "left-exposure", MAX_IMG_NAME_SIZE);
      snprintf(s, MSL, "#%u", ssi->serial);
      cvPutText(img->cvArr(), s, cvPoint(240, 20), &font, red);
      //img->updated();
    }
  }
  // debug
  if (img != NULL)
  {
    imgp = (UImgPush*)img;
    imgpq = imgp->getPushQueue();
    if (imgpq->getPushCmdCnt() > 12 or imgpq->getPushCmdCnt() < 0)
    {
      printf("UResSVS::doFranExposureControl: bad push cnt %d\n", imgp->getPushQueue()->getPushCmdCnt());
    }
  }
  // debug end
  //
  t.now();
  // estimate horizon line - or get default value
  //  hozLine = doHorizonEstimate(ssi);
  hozLineDef = varHorizon->getInt();
  hozLineEst = varHorizonEst->getInt();
  //
  // debug
  if (img != NULL)
  {
    // paint horizon lines
    if (hozLine == hozLineDef)
    {
      cvLine(img->cvArr(), cvPoint(58, hozLine),
               cvPoint(iw-1, hozLine), red, 1, 4, 0);
      cvPutText(img->cvArr(), "def horizon", cvPoint(220, hozLine-2), &font, red);
    }
    else
    {
      cvLine(img->cvArr(), cvPoint(10, hozLineEst),
               cvPoint(iw-8, hozLineEst), blue, 1, 4, 0);
      cvPutText(img->cvArr(), "horizon", cvPoint(10, hozLineEst-2), &font, black);
    }
  }
  // debug end
  // get control flag
  autoControl = varFranExp->getInt();
  //
  if (autoControl > 0 and ssi->si != NULL)
  {
    // Image intensity detector, i.e. current mode
    // reset bin counters
    for (i = 0; i < 256; i++)
      bin[i] = 0;
    // use left image (only)
    i0 = ssi->si->Color();
    for (i = hozLine; i < ih; i+=M)
    { // line number
      // get highest intensity in row here
      ii = &i0[i * iw];
      for (j = 0; j < iw; j+=M)
      {
        iv = *ii;
        p1 = iv & 0xff;
        p2 = (iv >> 8) & 0xff;
        p3 = (iv >> 16) & 0xff;
        // get gray value (using matlab constants)
        pg = (p1 * 2989 + p2 * 5870 + p3 * 1141) / 10000;
        // put into bin counter
        bin[pg]++;
        if (pg >= (256 - saturated))
          // over exposed
          binSky++;
        ii += M;
        binT++;
      }
    }
    // part of distribution may be in the saturated bins
    mm = 10;     // histogram maximum value
    mode = 0;    // mode (top value)
    meanSum = 0; // for mean calculation
    //meanLim = mini(1000, meanLim); // limit cutoff
    // avoid ends if less than 50% is in end bin
    // use maximum in non saturated area
    //lowBin = 0;
    //highBin = 0;
    for (i = 0; i < 256; i++)
    {
      if (bin[i] > mm)
      { // get maximum too -
        if (i <= 256 - saturated)
        {
          mode = i;
          mm = bin[i];
        }
      }
      meanN += bin[i];
      meanSum += i * bin[i];
    }
    if (meanN == 0)
      meanN = 1;
    mean = meanSum / binT;
    //
    // measured intensity is now in mode and mean
    //
    // mode error amplitude
    modeRef = varRefIntensity->getInt();
    modeRefBand = varRefIntensityBand->getInt();
    if (autoControl == 2)
      // use mean
      modeValid = mean;
    else
      // use Fran method
      modeValid = mode;
    // error gain
    errGain = varFranGain->getValued();
    //
    // amount in saturated and dark bin normalized
    binSkyN = binSky / double(binT);
    //
    // get control mode and non-linear values
    errSaturated = 0.0;
    if (modeValid < modeRef - modeRefBand)
      // use linear model only
      controlMode = 2;
    else if (modeValid > 150 and binSkyN > 0.05)
    { // way too bright
      errSaturated = -((((((2564.0  * binSkyN - 4925.0) * binSkyN +
          3465.0) * binSkyN - 976.7) * binSkyN +
          52.61) * binSkyN + 27.29)  * binSkyN + 1.066);
/*      errSaturated = -((((((6.9e4  * binSkyN - 7.7e4) * binSkyN +
                            3.2e4) * binSkyN - 5.3e3) * binSkyN +
                            197.1) * binSkyN + 45.3)  * binSkyN + 1.084);*/
/*      errSaturated = -((((((1.26e5  * binSkyN - 1.718e5) * binSkyN +
          9.23e4) * binSkyN - 2.42e4) * binSkyN +
          3.16e3) * binSkyN - 163.4) * binSkyN + 3.733);*/
/*      errSaturated = -(1.26e5 * pow(binSkyN, 6) - 1.718e5 * pow(binSkyN, 5) +
                         9.23e4 * pow(binSkyN, 4) - 2.42e4 * pow(binSkyN, 3) +
                         3.16e3 * sqr(binSkyN) - 163.4 * binSkyN + 3.733);*/
      // limit to max value - 45% saturated
      //errSaturated = fmax(-60.0, errSaturated);
      controlMode = 5;
    }
    else if (modeValid > modeRef + modeRefBand)
      // linear area
      controlMode = 4;
    else
      // acceptable area - no change
      controlMode = 3;
    //
    // delay the control signal
    for (i = MAX_CTRL_DELAY - 1; i > 0; i--)
      exposureCtrl[i] = exposureCtrl[i-1];
    // add current exposure control value
    exposureCtrl[0] = varShutter->getInt();
    // integrate error
    if (isReplay())
      delay = 0;
    else
      delay = varControlDelay->getInt();
    errLinear = (modeRef - modeValid) * errGain;
    newControl = exposureCtrl[delay] + roundi(errLinear + errSaturated);
    newControl = mini(100, maxi(0, newControl));
    //
    exposureCalcTime = t.getTimePassed();
    gain = varGain->getInt();
    // do gain control
    videoGainIgain = varRefShutterIGain->getValued();
    refShutter = varRefShutter->getInt();
    videoGainMax = varRefVideoGainMax->getInt();
    //
    newGain += videoGainIgain * double(newControl - refShutter);
    // limit integration
    newGain = fmin(double(videoGainMax), fmax(0.0, newGain));
    // time to implement gain only?
    if (controlMode == 3 and fabs(newGain - gain) > 10.0)
      // implement, if the gain change gets too far from ideal value
      controlMode = -3;
    //
    if (img != NULL)
    {
      paintHistogramH(img, bin, 256, mm, ih-18, 70, iw - 262, 1);
      cvPutText(img->cvArr(), "intensity below horizon", cvPoint(iw - 260, ih-5), &font, red);
      // paint lines for mode control
      i = iw - 257 + modeRef;
      cvLine(img->cvArr(), cvPoint(i+modeRefBand, ih-17), cvPoint(i+modeRefBand, ih-18-15), mag, 2);
      cvLine(img->cvArr(), cvPoint(i-modeRefBand, ih-17), cvPoint(i-modeRefBand, ih-18-15), mag, 2);
//      i = iw - 257 + 256 - saturated;
//      cvLine(img->cvArr(), cvPoint(i+5,   ih-17), cvPoint(i+5,   ih-18-15), mag, 2);
//      cvLine(img->cvArr(), cvPoint(i, ih-17), cvPoint(i, ih-18-15), mag, 2);
      // paint mode and mean values
      i = iw - 257;
//      cvLine(img->cvArr(), cvPoint(i+mode, ih-18-70), cvPoint(i+mode, ih-18-55), mag, 3);
      cvLine(img->cvArr(), cvPoint(i+mean, ih-18-70), cvPoint(i+mean, ih-18-60), red, 3);
      // write also current shutter and gain
      if (controlMode == 3)
        snprintf(s, MSL, "shut=%d, gain=%d no change", exposureCtrl[delay], gain);
      else
        snprintf(s, MSL, "shut=%d->%d, gain=%d->%d", exposureCtrl[delay], newControl, gain, roundi(newGain));
      cvPutText(img->cvArr(), s, cvPoint(iw - 260, ih-18-75), &font, red);
      snprintf(s, MSL, "bin255:%3.0f%%", binSkyN*100.0);
      cvPutText(img->cvArr(), s, cvPoint(iw - 110, ih-75+10), &font, red);
      img->updated();
    }
    //
    if (false)
    {
      if (verbose)
        printf("img %lu avgIntensity %d\n", serial, mean);
    }
    varMeanIntensity->setValued(mean, 0);
    // debug end
    if (not isReplay())
    { // do not set controls in replay mode
      if (autoControl < 3 and(serial - serialFran > 4))
      { // control allowed, is it needed?
        if (controlMode != 3)
        { // implement
          if (newControl != exposureCtrl[0])
          { // not same as before and not too close to reference value
            setShutter(newControl);
            serialFran = serial;
          }
          if (round(newGain) != gain)
          { // need gain adjustment too
            setGain(roundi(newGain));
            serialFran = serial;
          }
        }
      }
    }
    // write to log
    if (logFran.isLogOpen())
    {
      //
      saveToExpFile(logFran.getF(), serial, exposureCtrl[delay], newControl, newGain,
                    errLinear, errSaturated, binSkyN, controlMode);
    }
    //
    // debug
/*    if (img != NULL)
    {
      paintHistogramV(img, binh, ih/M, 256.0, 0, 60, 0, M);
      cvPutText(img->cvArr(), "max I", cvPoint(2, 22), &font, red);
      img->updated();
    }*/
    // debug end
  }
  return newControl;
}

////////////////////////////////////////////////////

void UResSVS::setGainAndShutterVar()
{
  svsVideoImages * sourceObject;
  //
  sourceObject = (svsVideoImages *) videoObject;
  if (sourceObject != NULL)
  {
    if (sourceObject->use_autoexposure)
      varShutter->setValued(-1.0, 0);
    else
      varShutter->setValued(sourceObject->exposure, 0);
    if (sourceObject->use_autogain)
      varGain->setValued(-1.0, 0);
    else
      varGain->setValued(sourceObject->gain, 0);
  }
}

//////////////////////////////////////////////////////

// int UResSVS::doHorizonEstimate(USvsImageSet * ssi)
// {
//   int hozLine = -1;
//   int w, h;
//   unsigned long * ii, *i0, iv;
//   int i, j;
//   int p1, p2, p3, pg;
//   const int M = 2; // take every M pixel only
//   int * binp, *bin1, *bin2;
//   int maxDif = 0;
//   int maxDifRow = -1;
//   int maxVal = 0;
//   //
//   if (ssi->si != NULL)
//   {
//     // last line
//     hozLine = varHorizon->getInt();
//     // Image intensity detector, i.e. current mode
//     // get image size
//     w = varWidth->getInt();
//     if (w > 640)
//       w = 640;
//     if (w < 0)
//       w = 0;
//     h = (w * 3) / 4;
//     // reset bin counters
//     for (i = 0; i < h/M; i++)
//       binh[i] = 0;
//     // use left image (only)
//     i0 = ssi->si->Color();
//     if (hozLine < 0 or hozLine >= h)
//       hozLine = (h * 2) / 3;
//     for (i = 0; i < h; i+=M)
//     {
//       // line number
//       // get highest intensity in row here
//       binp = &binh[i/M];
//       ii = &i0[i * w];
//       for (j = 0; j < w; j+=M)
//       {
//         iv = *ii;
//         p1 = iv & 0xff;
//         p2 = (iv >> 8) & 0xff;
//         p3 = (iv >> 16) & 0xff;
//         pg = (p1 + p2 * 2 + p3) / 4;
//         if (pg > *binp)
//           // save highest intensity in this row
//           *binp = pg;
//         ii += M;
//       }
//       // get the highest intensity overall
//       if (*binp > maxVal)
//         maxVal = *binp;
//     }
//   }
//   // is it worth while?
//   if (maxVal > 210 and  binh[h/M-1] < 230)
//   {
//     // get line with largest difference
//     bin1 = binh;
//     bin2 = &binh[5/M];
//     for (i = 5; i < h; i+=M)
//     {
//       // find area with largest difference
//       if ((*bin1 - *bin2) > maxDif)
//       {
//         maxDif = *bin1 - *bin2;
//         maxDifRow = i;
//       }
//       bin1++;
//       bin2++;
//     }
//     hozLine = maxDifRow;
//   }
//   else
//   {
//     hozLine = varHorizon->getInt();
//   }
//   varHorizonEst->setValued(hozLine, 0);
//   return hozLine;
// }

//////////////////////////////////////////////////////

int UResSVS::doHorizonEstimateBG(USvsImageSet * ssi, bool debugImage)
{
  int hozLine;
  int w, h;
  unsigned long * ii, *i0, iv;
  int i, j;
  int p1, p2, p3, pbm, hLow;
  const int M = 1; // take every M pixel only
  bool finished;
  UImagePool * imgPool;
  UImage * img = NULL, *imgO;
  UPixel * pi = NULL;
  bool isSky;
  double blueSkyPct; // part of brightest blue to exceed range [0..1]
  //
  //
  hozLine = 0;
  if (ssi->si != NULL)
  { // get image dimension
    w = varWidth->getInt();
    if (w > 640)
      w = 640;
    if (w < 0)
      w = 0;
    h = (w * 3) / 4;
    //
    if (debugImage)
    {
      imgPool = (UImagePool*) getStaticResource("imgPool", true);
      if (imgPool == NULL)
      {
        printf("UResSVS::doHorizonEstimateBG - no imagePool available\n");
      }
      else
      { // assumes original image is here
        imgO  = imgPool->getImage(varImgLeft->getInt(), true);
        img  = imgPool->getImage(83, true);
        img->copyMeta(imgO, false);
        img->setName("sky-area");
        img->setSize(h/M, w/M, 3, 8, "RGB");
        // set image size
        img->setSize(h/M, w/M, 3, 8);
      }
    }
    // get pointer to first pixel (in 32 bit integer format)
    // use left image (only)
    i0 = ssi->si->Color();
    // find most intensive blue in first line of image
    pbm = 1;
    for (j = 0; j < w; j+=M)
    { // find max blue in line 0
      iv = *i0;
      p3 = (iv >> 16) & 0xff;   // blue
      if (p3 > pbm)
        pbm = p3;
      ii += M;
      if (p3 == 255)
        break;
    }
    // reduce to 85% of blue value as treshold
    blueSkyPct = varHorizonBluePct->getValued();
    pbm = roundi(pbm * blueSkyPct);
    //
    // now test for the extend of the sky
    hLow = h - roundi(h * varHorizonLow->getValued() / 100.0);
    // ignoring 20% at bottom
    for (i = 1 /*h/5*/; i < hLow; i+=M)
    { // get highest intensity in row here
      ii = &i0[i * w];
      finished = true;
      if (debugImage and img!= NULL)
        pi = img->getLine(i/M);
      for (j = 0; j < w; j+=M)
      {
        iv = *ii;
        p1 = iv & 0xff;           // red
        p2 = (iv >> 8) & 0xff;    // green
        p3 = (iv >> 16) & 0xff;   // blue
        isSky = (p3 >= p2 and p3 > pbm);
        if (isSky and finished)
        { // there is a blue (non-green) high intensity pixel
          finished=false;
          // continue for debug image, else
          if (not debugImage)
            break;
        }
        if (debugImage and img!= NULL)
        {
          if (isSky)
            pi->set(255, 0, 0);
          else
            pi->set(p3, p2, p1);
          pi++;
        }
        ii += M;
      }
      if (finished and hozLine == 0)
      { // no more horizon
        hozLine = i;
        if (not debugImage)
          break;
      }
    }
    // set estimated line as found
    if (hozLine == 0)
      hozLine = i;
    if (debugImage and img!= NULL)
      img->updated();
    varHorizonEst->setValued(hozLine, 0);
  }
  return hozLine;
}

///////////////////////////////////////////////////

// int UResSVS::doHorizonEstimate3(USvsImageSet * ssi)
// {
//   int hozLine = -1;
//   int w, h;
//   unsigned long * ii, *i0, iv;
//   int i, j, c, cM;
//   int p1, p2, p3;
//   const int M = 2; // take every M pixel only
//   int * binp;
//   bool finished = false;
//   UImagePool * imgPool;
//   UImage *imgS, *imgH, *imgO;
//   UPixel *piS, *piH;
//   bool isSky;
//   int limit = 180;
//   int lum;
//   int bluIch, sum;
//   const int MIW = 800;
//   bool lA[MIW], lB[MIW];
//   bool *ls, *l1, *l2;
//   int lI[MIW];
//   int lJ[MIW];
//   int *is, *i1, *i2;
//   int mode = 0;
//   int lowLimit;
//   //
//   imgPool = (UImagePool*) getStaticResource("imgPool", true);
//   imgS  = imgPool->getImage(80, true);
//   imgH  = imgPool->getImage(81, true);
//   imgO  = imgPool->getImage(varImgLeft->getInt(), true);
//   //
//   if (ssi->si != NULL)
//   { // last line
//     limit = varHozLimit->getInt();
//     hozLine = varHorizon->getInt();
//     // Image intensity detector, i.e. current mode
//     // get image size
//     w = varWidth->getInt();
//     if (w > 640)
//       w = 640;
//     if (w < 0)
//       w = 0;
//     h = (w * 3) / 4;
//     lowLimit = (h*4)/5;
//     // debug result images
//     imgH->copyMeta(imgO, false);
//     imgH->setName("sky-area");
//     imgH->setSize(h/M, w/M, 3, 8, "RGB");
//     imgS->copyMeta(imgO, false);
//     imgS->setName("sky-level");
//     imgS->setSize(h/M, w/M, 3, 8, "RGB");
//     // reset bin counters
//     for (i = 0; i < h/M; i++)
//       binh[i] = 0;
//     cM = w/M;
//     for (i = 0; i < cM; i++)
//     { // reset 2 lines with intensity and sky-flag
//       lA[i] = true;
//       lB[i] = true;
//       lI[i] = 0;
//       lJ[i] = 0;
//     }
//     // assign pointers to line buffers
//     l1 = lA;
//     l2 = lB;
//     i1 = lI;
//     i2 = lJ;
//     // use left image (only)
//     i0 = ssi->si->Color();
//     hozLine = -1;
//     for (i = 0; i < h; i+=M)
//     { // line number
//       // get highest intensity in row here
//       binp = &binh[i/M];
//       ii = &i0[i * w];
//       finished = true;
//       // prepare result image line pointers
//       piS = imgS->getLine(i/M);
//       piH = imgH->getLine(i/M);
//       for (j = 0; j < w; j+=M)
//       {
//         c = j/M;
//         iv = *ii;
//         p1 = iv & 0xff;             // red
//         p2 = (iv >> 8) & 0xff;      // green
//         p3 = (iv >> 16) & 0xff;     // blue
//         lum = (p1 + p2 + p3);
//         if (mode == 1)
//         {
//           bluIch = p3 - 128;
//         }
//         else
//         {
//           bluIch = (p3*3*128)/(lum + 1) - 128;
//           bluIch *= 8;
//         }
//         // intensity
//         lum /= 3;
//         // sum of intensity, bluich and isSky limit
//         if (mode == 1)
//           sum = ((h - i - h/3) * 100) / h + bluIch;
//         else
//           sum = (lum - 128) +((h - i - h/2) *128) / h + bluIch;
//         //
//         // if getting darker, then less likely to be the sky
//         i1[c] = lum;
//         sum -= (i2[c] - lum) * 4;
//         piS->set(mini(255, maxi(0, sum + 128)), 0, 0);
//         //
//         // test
//         isSky = sum > limit;
//         if (isSky)
//         { // test if connected with other sky
//           if (c > 0 and c <(cM - 1))
//             // all lines except edges
//             isSky = isSky and(l1[c] or l1[c+1] or l1[c-1] or l2[c-1]);
//           else if (c == 0)
//             // left edge
//             isSky = isSky and(l1[c] or l1[c+1]);
//           else
//             // right edge
//             isSky = isSky and(l1[c] or l1[c-1] or l2[c-1]);
//         }
//         //
//         l2[c] = isSky;
//         if (isSky and finished)
//         { // there is a blue (non-green) pixel
//           finished=false;
//         }
//         if (isSky)
//           piH->set(127, 127, 255);
//         else
//           piH->set(p1, p2, p3);
//         ii += M;
//         piS++;
//         piH++;
//       }
//       // swap isSky line
//       ls = l1;
//       l1 = l2;
//       l2 = ls;
//       // swap sky level line
//       is = i1;
//       i1 = i2;
//       i2 = is;
//       if (finished)
//       { // no more horizon
//         // a full line of not blueich
//         if (hozLine < 0)
//         {
//           hozLine = i;
//           if (hozLine > lowLimit)
//             // not less than 20% of image
//             hozLine = varHorizon->getInt();
//         }
//         if (not isReplay())
//           // in real life we stop here
//           // the rest is wast of time
//           break;
//       }
//     }
//   }
//   if (isReplay())
//     printf("Found horizon line at %d (limit=%d)\n", hozLine, limit);
//   imgS->updated();
//   imgH->updated();
//   varHorizonEst->setValued(hozLine, 0);
//   return hozLine;
// }

/////////////////////////////////////////////////////////

bool UResSVS::replayStep(int steps)
{
  bool result = true;
  int i;
  //
  if (steps == 0)
  {
    // first image can not be zero-stepped
    if (dataSi.serial > 0)
      dataSi.serial--;
  }
  else
  {
    for (i = 0; i < steps; i++)
    {
      result = replayStep();
      if (not result)
        break;
    }
    if (result)
    {
      // alert other resources of the advance in replay time
      replayAdvanceTime(replayTimeNow);
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////////////

bool UResSVS::replayStep()
{
  bool result = true;
  UTime t;
  int n;
  unsigned int u1,u2;
  unsigned int imgSer;
  const int MFL = MAX_FILENAME_SIZE;
  char fbn[MFL];
  int shutter, gain;
  double transmisDelayExtra;
  char * p1;
  //
  if ((replayFile != NULL) and replay)
  {
    t = replayTimeNext; // t.setTimeTod(replayLine);
    if (t.valid)
    {
      // this is a legal line
      // 1216815931.315272 svsSet    11179  43   0 svs_00011179_20080723_142531.315
      // 1216815931.582272 svsSet    11181  43   0 svs_00011181_20080723_142531.582
      //
      n = sscanf(replayLine, "%u.%u svsSet %u %d %d %s",
                   &u1, &u2, &imgSer, &shutter, &gain,
                   fbn);
      result = (n >= 6);
      if (result)
      { // all values read - speed is optional
        // implement - i.e. read source images
        snprintf(replayImgBaseName, MFL, "%s/%s%s", replayPath, imgSetSubdir, fbn);
        replayTimeNow = t;
        replayFrameSerial = imgSer;
        varShutter->setValued(shutter, 0);
        varGain->setValued(gain, 0);
      }
    }
    do
    { // read until next valid line
      p1 = fgets(replayLine, MAX_LOG_LINE_LENGTH, replayFile);
      // timestamp is first entry - read
      if (p1 != NULL)
        t.setTimeTod(replayLine);
      // increase line counter
      replayLogLine++;
      if (t.valid)
        // make sure it is a svsSet line
        t.valid = (strstr(replayLine, " svsSet ") != NULL);
      //
    } while (not t.valid and not feof(replayFile));
    //
    if (t.valid)
    { // set and adjust replay time
      transmisDelayExtra = varDelayReplay->getValued();
      replayTimeNext = t - transmisDelayExtra;
    }
    if (feof(replayFile))
    {
      fclose(replayFile);
      replayFile = NULL;
      replayPath[0] = '\0';
      //if (verbose)
      fprintf(stderr, "No more valid replay data (after %d lines)\n", replayLogLine);
    }
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////

bool UResSVS::setReplay(bool value)
{
  bool result = true;
  const int MFL = 500;
  char fn[MFL];
  //
  if (value != replay)
  { // change state
    replay = value;
    useRealCamera = not replay;
    if (not threadRunning)
      // thread is needed to do push processing etc.
      start();
    if (not replay and (replayFile != NULL))
    { // stop replay and clear pose history
      fclose(replayFile);
      replayFile = NULL;
    }
    if (replay)
    { // start replay
      replayLogLine = 0;
      replayTimeNext.valid = false;
      replayFile = fopen(getReplayFileName(fn, MFL), "r");
      result = (replayFile != NULL);
      if (not result)
        //if (verbose)
        fprintf(stderr, "Replay file not found '%s'\n", fn);
      if (result)
      { // read first line
        result = replayStep();
        if (not result)
          fprintf(stderr, "no valid replay lines in file '%s'\n", fn);
      }
    }
  }
  return result;
}

////////////////////////////////////////////////////////////

bool UResSVS::methodCall(const char * name, const char * paramOrder,
                           char ** strings, const double * pars,
                           double * value,
                           UDataBase ** returnStruct,
                           int * returnStructCnt)
{
  bool result = true;
  double resVal = 0.0; // no valid data
  double ser;
  bool rv;
  // evaluate standard functions
  if ((strcasecmp(name, "get3d") == 0) and(strcmp(paramOrder, "dc") == 0))
  { // get serial number of old data
    ser = pars[0];
    if (points3D != NULL)
    { // data structure is available
      if ((ser < 0.0 or points3D->serial >(unsigned int) ser) and
           returnStruct != NULL)
      { // data is usable
        *returnStruct = points3D;
        resVal = 1.0; // data is valid
      }
    }
    *value = resVal; // validity of data
  }
  if ((strcasecmp(name, "step") == 0) and(strcmp(paramOrder, "d") == 0))
  { // get serial number of old data
    ser = pars[0];
    rv = replayStep(int(ser));
    *value = rv; // validity of data
  }
  if ((strcasecmp(name, "verbose") == 0) and(strcmp(paramOrder, "d") == 0))
  { // get serial number of old data
    verbose = (int(pars[0]) != 0);
    *value = 1.0; // always valid
  }
  else
    // function not found
    result = false;
  return result;
}

/////////////////////////////////////////////////////

void UResSVS::setSensorPose(UPosRot * newPose)
{
  varPose6d->set6D(newPose);
}

/////////////////////////////////////////////////////

UPosRot UResSVS::getSensorPose()
{
  return varPose6d->get6D();
}

/////////////////////////////////////////////////////

void UResSVS::paintHistogramH(UImage * img, const int bin[], const int binCnt,
                                double binMax, int baseline, int height, int left, int colWidth)
{
  int i;
  CvScalar blue = CV_RGB(255, 0 , 0);
  CvScalar red = CV_RGB(0, 0 , 255);
  //CvScalar mag = CV_RGB(155, 0 , 155);
  CvPoint p1, p2;
  double scale = height / binMax;
  int hgt;
  //
  if (img != NULL)
  {
    int w = img->getWidth();
    //h = img->getHeight();
    //
    p1.x = left;
    p1.y = baseline;
    // make first double width
    p2.x = left + colWidth*2;
    for (i = 0; i < binCnt; i++)
    {
      hgt = roundi(bin[i] * scale);
      if (hgt > height)
        hgt = height;
      p2.y = baseline - hgt;
      //void cvRectangle(CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color,
      //                  int thickness=1, int line_type=8, int shift=0);
      cvRectangle(img->cvArr(), p1, p2, blue);
      p1.x = p2.x;
      p2.x += colWidth;
      // stop if out of image
      if (p1.x >= w)
        break;
    }
    // make last double width
    cvRectangle(img->cvArr(), p1, p2, blue);
    p1.x = left - 1;
    p1.y = baseline;
    p2.x = p1.x + colWidth * binCnt + 4;
    p2.y = p1.y - height - 1;
    cvRectangle(img->cvArr(), p1, p2, red);
  }
}

///////////////////////////////////////////////////

void UResSVS::paintHistogramV(UImage * img, const int bin[], const int binCnt,
                                double binMax, int baseline, int height, int top, int elemWidth)
{
  int i;
  CvScalar blue = CV_RGB(255, 0 , 0);
  CvScalar red = CV_RGB(0, 0 , 255);
  //CvScalar mag = CV_RGB(155, 0 , 155);
  CvPoint p1, p2;
  double scale = height / binMax;
  int hgt;
  //
  if (img != NULL)
  {
    //w = img->getWidth();
    int h = img->getHeight();
    //
    p1.y = top;
    p1.x = baseline;
    p2.y = p1.y + elemWidth;
    for (i = 0; i < binCnt; i++)
    {
      hgt = roundi(bin[i] * scale);
      if (hgt > height)
        hgt = height;
      p2.x = baseline + hgt;
      //void cvRectangle(CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color,
      //                  int thickness=1, int line_type=8, int shift=0);
      cvRectangle(img->cvArr(), p1, p2, blue);
      p1.y = p2.y;
      p2.y += elemWidth;
      // stop if out of image
      if (p1.y >= h)
        break;
    }
    p1.y = top - 1;
    p1.x = baseline;
    p2.y = p1.y + elemWidth * binCnt + 1;
    p2.x = p1.x + height + 1;
    cvRectangle(img->cvArr(), p1, p2, red);
  }
}

////////////////////////////////////

// bool UResSVS::doFranCode(int doJust)
// {
//   const char * baseDir = "/home/chr/chr/results/hako20080821-2/fran/imgs/";
//   const int MIC = 25;
//   const char * imgs[MIC] = {"1", "2", "3", "4", "5", "6", "7",
//                             "8", "9", "10", "11", "12", "13", "14", "15",
//                             "svs_00001356_20080821_133454.985",
//                             "svs_00005750_20080821_134440.801",
//                             "svs_00001364_20080821_133456.052",
//                             "svs_00011677_20080821_134658.137",
//                             "svs_00003563_20080821_133949.187",
//                             "svs_00012038_20080821_135839.131",
//                             "svs_00003586_20080821_133952.254",
//                             "svs_00013275_20080821_140124.051",
//                             "svs_00004403_20080821_134141.178",
//                             "svs_00013892_20080821_140246.310"
//                            };
//
//   //
//   const int MFL = MAX_FILENAME_LENGTH;
//   char fn[MFL], cmd[MFL];
//   FILE * fhl;
//   int i;
//   int hozLineJCA;
//   int hozLineBG;
//   int hozLineOld;
//   int meanIntens;
//   //
//   snprintf(fn, MFL, "%s/franHozLine.log", dataPath);
//   fhl = fopen(fn, "w");
//   if (fhl != NULL)
//   {
//     fprintf(fhl, "# horizon line for selected files\n");
//     fprintf(fhl, "# limit for JCA mode %d\n", varHozLimit->getInt());
//     fprintf(fhl, "Number, old, JCA, B>R, mean intensity, imageName\n");
//     dataSi.lock();
//     for (i = maxi(0, doJust); i < MIC; i++)
//     {
//       snprintf(replayImgBaseName, MFL, "%s/%s", baseDir, imgs[i]);
//       replayFrameSerial = i+1;
//       if (not getImageSetFromStream(&dataSi, i))
//       {
//         snprintf(cmd, MFL, "mv %s/%sC.bmp %s/%s-C.bmp\n", baseDir, imgs[i], baseDir, imgs[i]);
//         system(cmd);
//         snprintf(cmd, MFL, "mv %s/%sQ.bmp %s/%s-Q.bmp\n", baseDir, imgs[i], baseDir, imgs[i]);
//         system(cmd);
//         snprintf(cmd, MFL, "mv %s/%sL.bmp %s/%s-L.bmp\n", baseDir, imgs[i], baseDir, imgs[i]);
//         system(cmd);
//         snprintf(cmd, MFL, "mv %s/%sR.bmp %s/%s-R.bmp\n", baseDir, imgs[i], baseDir, imgs[i]);
//         system(cmd);
//       }
//       else
//       {
// //        hozLineJCA = doHorizonEstimate3(&dataSi);
//         hozLineBG = doHorizonEstimateBG(&dataSi, true);
// //        hozLineOld = doHorizonEstimate(&dataSi);
//         doFranExposureControl(&dataSi, hozLineBG, true);
//         meanIntens = varMeanIntensity->getInt();
//         fprintf(fhl, "%i, %d, %d, %d, %d, %s\n", i, hozLineOld, hozLineJCA, hozLineBG, meanIntens, imgs[i]);
//       }
//       if (doJust >= 0)
//         break;
//     }
//     dataSi.unlock();
//     printf("Saved result to %s\n", fn);
//     fclose(fhl);
//   }
//   return true;
// }

////////////////////////////////////////////////////////

void UResSVS::saveToHozFile(FILE * logfile, unsigned int serial)
{
//  int hozLineJCA;
  int hozLineBG;
//  int hozLineOld;
  int meanIntens;
  int h, hLow, w;
  //
  if (logfile != NULL)
  {
    w = varWidth->getInt();
    if (w > 640)
      w = 640;
    if (w < 0)
      w = 0;
    h = (w * 3) / 4;
    hLow = h - roundi(h * varHorizonLow->getValued() / 100.0);

    if (serial == 0)
    {
      fprintf(logfile, "# hozGreenLimit   %d\n", varHozGreenLimit->getInt());
      fprintf(logfile, "# horizon         %d\n", varHorizon->getInt());
      fprintf(logfile, "# horizonLowLimit %d%% = line %d\n", varHorizonLow->getInt(), hLow);
      fprintf(logfile, "# refIntensity    %d\n", varRefIntensity->getInt());
      fprintf(logfile, "# image size      %dx%d\n", h, w);
      fprintf(logfile, "time  imageserial  horizon lowLimit  meanIntensity  videoGain, Shutter\n");
    }
    else
    {
      //hozLine = varHorizonEst->getInt();
//      hozLineJCA = doHorizonEstimate3(&dataSi);
      hozLineBG = doHorizonEstimateBG(&dataSi, true);
//      hozLineOld = doHorizonEstimate(&dataSi);
//      varHorizonEst->setValued(hozLine, 0);
      //doFranExposureControl(&dataSi, hozLineJCA);
      dataSi.unlock();
      meanIntens = varMeanIntensity->getInt();
      fprintf(logfile, "%lu.%06lu  %6u  %3d %3d %4d %4d %4d\n",
                dataSi.imageTime.getSec(), dataSi.imageTime.getMicrosec(),
                serial, hozLineBG, hLow,
                meanIntens, varGain->getInt(), varShutter->getInt());
    }
  }
}

////////////////////////////////////////////////////////////

void UResSVS::saveToExpFile(FILE * logfile, unsigned int serial,
                              const int oldShutter,
                              const int newControl,
                              const double newGain,
                              const double errLinear,
                              const double errSaturated,
                           const double bin255, const int controlMode)
{
  int meanIntens;
  int refIntens, refIntensBand;
  int hozLine;
//  int shutter;
  int gain;
  //int * binp;
  //
  if (logfile != NULL)
  {

    if (serial == 0)
    {
      fprintf(logfile, "# refIntensity       %d\n", varRefIntensity->getInt());
      fprintf(logfile, "# meanToExposureGain %d\n", varFranGain->getInt());
      fprintf(logfile, "# shutterRef         %d\n", varRefShutter->getInt());
      fprintf(logfile, "# videoGain-I-Gain   %d\n", varRefShutterIGain->getInt());
      fprintf(logfile, "# videoGainMax       %d\n", varRefVideoGainMax->getInt());
      fprintf(logfile, "time serial horizon oldshutter use!=3 newShutter oldvideogain newVideoGain "
          "refIntensity refIntensityBand intensity ctlErrorLin bin255%% ctlErrSat\n");
    }
    else
    {
      hozLine = varHorizonEst->getInt();
      meanIntens = varMeanIntensity->getInt();
      refIntens = varRefIntensity->getInt();
      refIntensBand = varRefIntensityBand->getInt();
      //shutter = varShutter->getInt();
      gain = varGain->getInt();

      fprintf(logFran.getF(), "%lu.%06lu %6u "
                              "%3d   %3d %2d %3d   %3d %4.1f   "
                              "%3d %3d %3d   %4.1f %5.1f %4.1f",
                dataSi.imageTime.getSec(), dataSi.imageTime.getMicrosec(),
                serial,
                hozLine,  oldShutter, controlMode, newControl, gain, newGain,
                refIntens, refIntensBand, meanIntens, errLinear, bin255*100.0, errSaturated);
      //binp = bin;
/*      for (i = 0; i < 256; i++)
      {
        // print intensity distribution
        fprintf(logFran.getF(), " %3d", *binp++);
      }*/
      fprintf(logFran.getF(), "\n");
    }
  }
}


void UResSVS::createResources()
{ // start streaming - if cameras are available
  if (useRealCamera)
  {
    printf("Starting stereo camera\n");
    getImageSetFromHardware(&dataSi);
  }
  printf("Starting stereo camera thread\n");
  start();
}
//#endif
