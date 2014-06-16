/***************************************************************************
 *   Copyright (C) 2010 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *
 * $Rev: 257 $
 * $Id: ufunckinect.cpp 257 2013-10-06 12:12:56Z jcan $
 *
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
 *
 *
 * This file includes part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 ***************************************************************************/

/* edited by Kristian Villien 11/1-12
added variable for depth correction
added depth correction in new image pool
when using depth correction 8 bit image is using this pool
*/

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>
#include <urob4/uimagepool.h>
#include <ucam4/ucammount.h>
#include <ucam4/ucampool.h>

//PCL
// #define USE_KINECT
// #define USE_PCL
#ifdef USE_PCL
#define BOOST_SYMBOL_VISIBLE
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <upcpitem.h>
#endif


#include "ufunckinect.h"

#include <pthread.h>

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncKinect' with your classname, as used in the headerfile */
  return new UFuncKinect();
}

#endif

const int MAX_KINECT_OBJ = 3;
UFuncKinect * kinectObj[MAX_KINECT_OBJ] = {NULL, NULL, NULL};

#ifdef USE_KINECT
/**
C-callback function for color image */
void rgb_cb0(freenect_device *dev, void *rgb, uint32_t timestamp)
{
  if (kinectObj[0] != NULL)
    kinectObj[0]->callback_rgb(dev, (uint8_t *) rgb, timestamp);
}
/**
C-callback function for color image */
void depth_cb0(freenect_device *dev, void /*freenect_depth */ * depth, uint32_t timestamp)
{
  if (kinectObj[0] != NULL)
    kinectObj[0]->callback_depth(dev, (uint16_t *)depth, timestamp);
}
/**
C-callback function for color image */
void rgb_cb1(freenect_device *dev, void *rgb, uint32_t timestamp)
{
  if (kinectObj[1] != NULL)
    kinectObj[1]->callback_rgb(dev, (uint8_t *) rgb, timestamp);
}
/**
C-callback function for color image */
void depth_cb1(freenect_device *dev, void /*freenect_depth */ * depth, uint32_t timestamp)
{
  if (kinectObj[1] != NULL)
    kinectObj[1]->callback_depth(dev, (uint16_t *)depth, timestamp);
}
/**
C-callback function for color image */
void rgb_cb2(freenect_device *dev, void *rgb, uint32_t timestamp)
{
  if (kinectObj[2] != NULL)
    kinectObj[2]->callback_rgb(dev, (uint8_t *) rgb, timestamp);
}
/**
C-callback function for color image */
void depth_cb2(freenect_device *dev, void /*freenect_depth */ * depth, uint32_t timestamp)
{
  if (kinectObj[2] != NULL)
    kinectObj[2]->callback_depth(dev, (uint16_t *)depth, timestamp);
}
#endif

///////////////////////////////////////////////////


UFuncKinect::~UFuncKinect()
{
  stop(true);
#ifdef USE_KINECT
  if (varInitialized->getBool())
  { // close device
    openKinect(false);
    libusb_exit((libusb_context *) usbCtx);
  }
#endif
  logImage.closeLogging();
  logAcc.closeLog();
  logf.closeLog();
  if (cloud3d != NULL)
    delete cloud3d;
  if (imgCbuf != NULL)
    delete imgCbuf;
}

/////////////////////////////////////////////////

void UFuncKinect::init()
{
  //initialize RGB depth gamma
  for (int i = 0; i < 2048; i++)
  {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }
  //initialize depth convertion
// values found at http://stackoverflow.com/questions/8663301/microsoft-kinect-sdk-depth-calibration
  float pixel;
  int i;
  for (i = 0; i < 1091; i++) //1090 is just above 10 meters, calculation screw up outside this range
                             //valid kinect range is ~4m
  {
    pixel = (float) i;
    pixel = 123.6*tan(pixel/2842.5+1.1863); // depth in mm, note depth means Z position, not direct depth
    t_depth[i] = (uint16_t) round(pixel);
  }
  for (; i < 2048; i++) // when outside range, set to 0
    t_depth[i] = 0;//-1;

  //initialize position scaling with focal length of 595 pixels

  threadRunning = false;
  threadStop = false;
  oldFocalLength = 0;
  oldLed = -1;
  oldCamDevC = -1;
  oldCamDevD = -1;
#ifdef USE_KINECT
  usbCtx = NULL;
#endif
  logDepthImage = NULL;
  logVideoImage = NULL;
  frameCntDepth = 0;
  frameCntRgb = 0;
  obj3dcloud = NULL;
  imgCbuf = NULL;
}

///////////////////////////////////////////////

void UFuncKinect::initXYTable(int focalLength)
{
  if (focalLength > 0)
  {
    const double cx_d = 640/2; // centre point
    const double cy_d = 480/2;
    printf("UFuncKinect::initXYTable:: making x-y- table based on focal length=%d\n", focalLength);
    for (int i = 0; i < 640; i++)
      t_posX[i] = float( (((double)i)-cx_d)/double(focalLength));
    for (int i = 0; i < 480; i++)
      t_posY[i] = float( (((double)i)-cy_d)/double(focalLength));
  }
  oldFocalLength = focalLength;
}

///////////////////////////////////////////////////

bool UFuncKinect::handleCommand(UServerInMsg * msg, void * extra)
{ // message is unhandled
  bool result = false;
  //
  if (getCmdIndex() == 0)
    result = handleKinect(msg);
  else if (getCmdIndex() == 1)
    result = handlePush(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFuncKinect::handleKinect(UServerInMsg * msg)
{
  const int MRL = 500;
  char reply[MRL];
  char r[MRL];
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0) or msg->tag.getAttCnt() == 0)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(aliasName);
    snprintf(reply, MRL, "--- %s is a plugin to XBOX 360 camera (3d and color)\n", aliasName);
    sendText(reply);
    snprintf(reply, MRL, "open=true|false     Start or stop camera (open=%s, device thread OK=%s)\n",
             bool2str(varIsOpen->getBool(1)), bool2str(lastLoopTime.getTimePassed() < 4.0));
    sendText(reply);
    sendText(            "silent              do not generate a reply\n");
    if (varImagesC3D != NULL)
    {
      snprintf(reply, MRL, "debug=true|false    Produce debug image (is %s for color-depth img=%d)\n",
               bool2str(varImagesC3D->getBool(0)), varImagesC3D->getInt(1));
      sendText(reply);
    }
    snprintf(reply, MRL, "log=true|false      action log (open=%s) to %s \n", bool2str(logf.isOpen()), logf.getLogFileName());
    sendText(reply);
    snprintf(reply, MRL, "logImage=N          logging of every N (N=%d) images to %s\n", varImageLogN->getInt(), logImage.getLogFileName());
    sendText(reply);
    snprintf(reply, MRL, "replay=true|false   replay (replay=%s) from %s\n", bool2str(isReplay()), getReplayFileName(r, MRL));
    sendText(reply);
    snprintf(reply, MRL, "step = N            replay N steps from replay file\n");
    sendText(reply);
    snprintf(reply, MRL, "logAcc=N            logging of every N (N=%d) acc value to %s \n", varAccLogN->getInt(), logAcc.getLogFileName());
    sendText(reply);
    snprintf(reply, MRL, "pcdFile[=filename]  make PCD file (clip first lines for MATLAB)\n");
    sendText(reply);
#ifdef USE_PCL
    sendText(            "updatePCP           Update point cloud pool from kinect images (assumed rectified)\n");
#else
    sendText(            "NB! Some commands are not functional, as the PCL library were not used at compile time\n");
#endif
    sendText("NB! High resolution IR works only if opened in normal mode first, and then not with depth image on\n");
    sendText("help       This message\n");
    sendText("----\n");
    snprintf(reply, MRL, "see also: '%sPush' for event handling of 3d cloud\n", aliasName);
    sendText(reply);
    snprintf(reply, MRL, "see also: 'var %s' for parameters\n", aliasName);
    sendText(reply);
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    bool doLog = false;
    bool doOpen = false;
    bool aLog = msg->tag.getAttBool("log", &doLog, true);
    bool anOpen = msg->tag.getAttBool("open", &doOpen, true);
    bool silent = msg->tag.getAttBool("silent", NULL);
    bool aDebugValue = false;
    bool aDebug = msg->tag.getAttBool("debug", &aDebugValue, true);
    int aLogAccValue = 0;
    bool aLogAcc = msg->tag.getAttInteger("logAcc", &aLogAccValue, true);
    int aLogImageValue = 0;
    bool aLogImage = msg->tag.getAttInteger("logImage", &aLogImageValue, true);
    //
    // implement command options
    // replay
    bool valueBool;
    if (msg->tag.getAttBool("replay", &valueBool, true))
      setReplay(valueBool);
    if (msg->tag.getAttBool("silent", &valueBool, true))
      varSilent->setBool(valueBool);
    // replay step
    int valueInt;
    if (msg->tag.getAttInteger("step", &valueInt, 1))
      replayStep(valueInt);
    if (msg->tag.getAttValue("updatePcp", r, MRL))
    {
      updateCloud(str2bool2(r, true));
    }
    if (msg->tag.getAttValue("pcdFile", r, MRL))
    {
      bool isOK = makePCD(r, false);
      if (not silent)
      {
        if (isOK)
          sendInfo("done");
        else
          sendWarning("failed to make 3Dfile");
      }
    }
    //
    if (aLog)
    { // open or close the log
      logf.openLog(doLog);
      if (logf.isOpen())
        snprintf(reply, MRL, "Opened logfile %s", logf.getLogFileName());
      else if (doLog == false)
        snprintf(reply, MRL, "Closed logfile %s", logf.getLogFileName());
      else
        snprintf(reply, MRL, "Failed to open logfile %s", logf.getLogFileName());
      // send an information tag back to client
      if (not silent)
        sendInfo(reply);
    }
    if (aLogImage)
    { // open or close the image log
      varImageLogN->setValued(aLogImageValue);
      Wait(0.1);
      if (logImage.isOpen() and aLogImageValue > 0)
        snprintf(reply, MRL, "Image logfile is open: %s", logImage.getLogFileName());
      else if (aLogImageValue == 0)
        snprintf(reply, MRL, "Image logfile is closed: %s", logImage.getLogFileName());
      else if (aLogImageValue > 0)
        snprintf(reply, MRL, "Image logfile %s failed to open (is camera closed?)", logImage.getLogFileName());
      else
        snprintf(reply, MRL, "Image logfile %s failed to close", logImage.getLogFileName());
      // send an information tag back to client
      if (not silent)
        sendInfo(reply);
    }
    if (aLogAcc)
    { // open or close the acc log
      varAccLogN->setValued(aLogAccValue);
      Wait(0.1);
      if (logAcc.isOpen())
        snprintf(reply, MRL, "Acc logfile is open: %s", logAcc.getLogFileName());
      else
        snprintf(reply, MRL, "Acc logfile is closed: %s", logAcc.getLogFileName());
      // send an information tag back to client
      if (not silent)
        sendInfo(reply);
    }
    if (aDebug)
    {
      //varDebug->setValued(aDebugValue);
//       if (varUseDepth8bit->getInt() == -1)
//         varUseDepth8bit->setInt(1, 0, true);
      varUseDepth->setBool(true, 0, true);
      varUseDepthColor->setBool(true, 0, true);
      if (not silent)
      {
        snprintf(reply, MRL, "debug set to %s - (img=%d is color, img=%d is depth, img=%d is colored-depth img=%d is 8-bit BW image)", bool2str(aDebugValue),
                varImagesC3D->getInt(0), varImagesC3D->getInt(1), varImagesC3D->getInt(2), varImagesC3D->getInt(2));
        sendInfo(reply);
      }
    }
    if (anOpen)
    { // default update call
      bool isOpen = varIsOpen->getBool(1);
      varIsOpen->setBool(doOpen);
      if (not silent)
      {
        snprintf(reply, MRL, "camera open=%s, set to open=%s (threadOK=%s)",
                  bool2str(isOpen), bool2str(varIsOpen->getBool(0)), bool2str(lastLoopTime.getTimePassed() < 4.0));
        sendInfo(reply);
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncKinect::createResources()
{
  aliasname = getAliasName();
  // Create global variables - owned by this plug-in.
  varSilent = addVar("silent", 1.0, "d", "(rw) show less messages to console");
  varKinectNumber = addVar("kinectNumber", 0.0, "d", "(r/w) kinect number to be used on next open");
  varUpdateCnt = addVar("updCnt", 0.0, "d", "(r) Number of updates");
  varFramerateDesired = addVar("desiredFramerate", 2.0, "d", "(rw) desired framerate (frames/sec) for both depth and color");
  varFramerate = addVar("measuredFramerate", "0 0", "d", "(r) [color, depth] measured framerate (frames/sec)");
  varResolution = addVar("desiredResolution", 1.0, "d", "(rw) desired color image resolution 1=medium (640x480), 2 = (1280x1024))");
  varImagesC3D = addVarA("imagesC3D", "18 19 20 22", "d", "(rw) Image pool for [0] color, [1] depth (16bit), [2] depth (RGB)image,"
                    " [3] corrected depth");
  varCamDeviceNum = addVarA("camDeviceNum", "18 19", "d", "(r/w) index to camera info - [color, depth]");
  varVideoFmt = addVar("videoFmt", 0.0, "d", "(rw) is IR (dot) video format [0=bayer, 1=IR-8, 2=RGB, 3=YUV422]");
  varDepthFmt = addVar("depthFmt", 4.0, "d", "(rw) is IR (dot) depth format [0=11bit, 1=10bit, 2=8-bit, 3=mm, 4=registred]");
  varUseColor = addVar("useVideo", "1 0", "d", "(rw) [0]=1 enable video (color or IR), [1]=1 is in use");
  varUseDepth = addVar("useDepth", "1 0", "d", "(rw) [0]=1 request depth (at next open), [1]=1 is in use");
 // varUseDepth8bit = addVar("useDepth8bit", 1.0, "d", "(rw) make 8-bit BW image from depth -1 = no image, 0-8 subtracts 512 and shift 0-8 bits");
  varUseDepthColor = addVar("useDepthColor", 0.0, "d", "(rw) make RGB pseudo color image from 10 or 11 bit depth 0=do not make");
  varIsOpen = addVarA("open", "0 0", "d", "(rw) [0]: set to 0 to close set to 1 to open, [1]: 0:isclosed 1:isopen");
  varInitialized = addVar("initialized", 0.0, "d", "(r) is the camera initialized");
  varTiltDeg = addVar("tiltDeg", 0.0, "d", "(rw) the current motor control tilt angle [degrees]");
  varTiltUse = addVar("tiltUse", 0.0, "d", "(rw) should tilt control be attempted (0=not used, 1=used)");
  varLed = addVar("led", 1.0, "d", "(r/w) LED value: OFF 0, GREEN 1, RED 2, YELLOW 3, B-YELLOW 4, B-GREEN 5, B-RED_YELLOW = 6, B-RED_GREEN = 7");
  varAcc = addVarA("acc", "0 0 0", "3d", "(r) the current accelerometer reading in N/kg (x fwd,y left,z up)");
  varOrient = addVarA("orient", "0 0 0", "3d", "(r) the current kinect orientation based on acc values [roll (cv), tilt (down), (yaw (bad))]");
  varTime = addVar("time", 0.0, "t", "(r) Time at last update");
  varAccInterval = addVar("accInterval", 0.7, "d", "(r/w) wait getting acc-readings at least this time [sec]");
  varAccRate = addVar("accRate", 0.0, "d", "(r) rate of acc measurements (per second)");
  varAccLogN = addVar("accLogN", 0.0, "d", "(rw) log acceleration evert N measurements (0 = close log)");
  varImageLogN = addVar("imageLogN", 0.0, "d", "(rw) log images (color and depth) every Nth imageset (0 = close log)");
  varReplayLine = addVarA("replay", "0 0", "d", "(r) [0]: replay status 1=on, [1] current replay line");
  varReductionFactor = addVar("reductionFactor", 2.0, "d", "(rw) reduce number of points in the produced point clouds 1=all, 2=every other, 3= ...");
//  varMakeObj3Dcloud = addVar("makeObj3Dcloud", 0.0, "d", "(rw) make obj3d cloud (for obj3d plugin)");
//  varMakePCLFile = addVarA("MakePCLFile", "0", "d", "set to save valid points in a PCL file");
//  varMake3DFile = addVarA("Make3dFile", "0", "d", "set to save valid points in a 3d text file");
#ifdef USE_PCL
  varUseCorrectedDepth = addVar("UseCorrectedDepth", "0", "d", "Set to one for correction the kinect depth. Depth image is then in mm.");
  varMaxUsedDepth = addVarA("MaxUsedDepth", "40", "d", "The maximum accepted depth [m] when working with PointCloudLib. Kinect datasheet notes 4.5 m as maximum valid distance.");
  //getting pointcloud
  addMethod("GetPointCloud", "dd",  "Get data in a PCL pointcloud structure points (cloud type(0=xyz,1=xxyzrgb),max points(-1=no limit))");
#endif
  addMethod("get3d", "",  "Get (img3d) pointcloud structure with all valid points (not PCL)");

  //
  { // set log file names
    const int MSL = 100;
    char s[MSL];
    logf.setLogName(getAliasName());
    snprintf(s, MSL, "%sImage", getAliasName());
    logImage.setLogName(s);
    replaySetBaseFileName(s);
    snprintf(s, MSL, "%sAcc", getAliasName());
    logAcc.setLogName(s);
  }
  // start read thread
  start();
}
//////////////////////////////////////////////////////

bool UFuncKinect::methodCall(const char * name, const char * paramOrder, char ** strings, const double * pars,
                      double * value, UDataBase ** returnStruct, int * returnStructCnt)
{ // inter-plugin method call using simple string and double arrays as parameters
 bool result = true;
 if ((strcasecmp(name, "GetPointCloud") == 0) and ((strcmp(paramOrder, "dd") == 0) or (strcmp(paramOrder, "ddd") == 0)))
 {
   int redFac = varReductionFactor->getInt();
   if (strlen(paramOrder) == 3)
     redFac = roundi(pars[2]);
   *returnStructCnt = GetXYZCoordinates((void *) returnStruct, pars[1],pars[0], redFac);
   if (returnStructCnt != 0)
     *value = 1.0;
   else
     *value = 0.0;
 }
 else
   result = false;
 return result;
}

////////////////////////////////////////////////

bool UFuncKinect::methodCallV(const char * name, const char * paramOrder,
                                          UVariable * params[],
                                          UDataBase ** returnStruct,
                                          int * returnStructCnt)
{ // inter plug-in method call using UVariables as parameter and UDataBase as result type
  // ordinary parameters can be used both ways
 bool result = true;
 if ((strcasecmp(name, "get3d") == 0) and (strlen(paramOrder) == 0))
 {
   if (*returnStructCnt > 0 and returnStruct != NULL and cloud3d != NULL)
   {
     *returnStruct = cloud3d;
     *returnStructCnt = 1;
   }
   else
     *returnStructCnt = 0;
 }
 else
   result = false;
 return result;
}

///////////////////////////////////////////

void UFuncKinect::callGotNewDataWithObject()
{
  UDataBase * data = cloud3d;
  bool make;
  make = (cloud3d == NULL);
  if (not make)
  { // maybe data is OK as is
    UTime imgTime = varTime->getTime();
    make = (imgTime - cloud3d->time) > 0.001;
  }
  if (make)
  { // cloud data don't exist or is too old
    // (re)generate point cloud
    if (makeObj3Dcloud())
      data = cloud3d;
    else
      // failed (probably no source images)
      data = NULL;
  }
  gotNewData(data);
}

#ifdef USE_KINECT

bool UFuncKinect::openKinect(bool doOpen)
{
  bool isOK;
  int err;
  frameLogCnt = 0;
  //
  // printf("UFuncKinect::openKinect entry\n");
  logf.toLog("Open/close call");
  if (varIsOpen->getBool(1) != doOpen)
  {
    bool doCloseDevice = not doOpen;
    // printf("UFuncKinect::openKinect trying to lock\n");
    logf.toLog("Open call - try lock");
    lock();
    if (doOpen)
    { // open the camera
      // printf("UFuncKinect::openKinect trying to open\n");
      logf.toLog("Try open");
      varIsOpen->setBool(0, 1);
      varUseColor->setBool(0, 1);
      varUseDepth->setBool(0, 1);
      // set callback object pointer
      int kn = varKinectNumber->getInt();
      if (kn < MAX_KINECT_OBJ and kn >= 0)
        kinectObj[kn] = this;
      //
      if (not varInitialized->getBool())
      {
        err = libusb_init(&usbCtx);
        isOK = err == 0;
        if (isOK)
        {
          err = freenect_init(&f_ctx,  usbCtx);
          isOK = err == 0;
        }
        if (isOK)
          // stop debug messages
          libusb_set_debug(usbCtx, 0);
        if (not isOK)
        {
          printf("freenect_init() failed (err = %d)\n", err);
          logf.toLog("freenect_init()", err, "failed - err number");
        }
        //printf("UFuncKinect::openKinect init done %s\n", bool2str(isOK));
        logf.toLog("freenect_init()", err, "is OK");
        varInitialized->setValued(isOK);
      }
      if (varInitialized->getBool())
      { // try open camera
        err = freenect_open_device(f_ctx, &f_dev, varKinectNumber->getInt());
        isOK = err == 0;
        if (not isOK)
        {
          printf("Could not open kinect device %d (err = %d)\n", varKinectNumber->getInt(), err);
          logf.toLog("freenect_open_device()", varKinectNumber->getInt(), err, "failed - err number");
        }
        varIsOpen->setBool(isOK, 1);
        //printf("UFuncKinect::openKinect open done %s\n", bool2str(isOK));
        logf.toLog("freenect_open_device()", varKinectNumber->getInt(), err, "is OK");
        //
        if (isOK)
        {
          printf("UFuncKinect::openKinect setting callbacks %s\n", bool2str(isOK));
          logf.toLog("UFuncKinect::openKinect setting callbacks");
          //
          // depth mode and callback
          //   typedef enum {
          //           FREENECT_DEPTH_11BIT        = 0, /**< 11 bit depth information in one uint16_t/pixel */
          //           FREENECT_DEPTH_10BIT        = 1, /**< 10 bit depth information in one uint16_t/pixel */
          //           FREENECT_DEPTH_11BIT_PACKED = 2, /**< 11 bit packed depth information */
          //           FREENECT_DEPTH_10BIT_PACKED = 3, /**< 10 bit packed depth information */
          //           FREENECT_DEPTH_REGISTERED   = 4, /**< processed depth data in mm, aligned to 640x480 RGB */
          //           FREENECT_DEPTH_MM           = 5, /**< depth to each pixel in mm, but left unaligned to RGB image */
          //           FREENECT_DEPTH_DUMMY        = 2147483647, /**< Dummy value to force enum to be 32 bits wide */
          //   } freenect_depth_format;
          freenect_depth_format dFmt;
          switch (varDepthFmt->getInt())
          {
            case 1: // 10 bit dybde
            case 2: // 8-bit dybde
              dFmt = FREENECT_DEPTH_10BIT_PACKED;
              break;
            case 3: // FREENECT_DEPTH_MM; requires 11BIT
            case 4: // FREENECT_DEPTH_REGISTERED; requires 11 bit - and result is in mm
              dFmt = FREENECT_DEPTH_11BIT_PACKED;
              freenect_init_registration(f_dev);
              break;
            default:
              dFmt = FREENECT_DEPTH_11BIT_PACKED;
              if (varDepthFmt->getInt() != 0)
                varDepthFmt->setInt(0);
              break;
          }
          depthFmt = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, dFmt);
          //fmt = freenect_get_depth_mode(1);
          freenect_set_depth_mode(f_dev, depthFmt);
          switch (kn) {
            case 0: freenect_set_depth_callback(f_dev, depth_cb0); break;
            case 1: freenect_set_depth_callback(f_dev, depth_cb1); break;
            case 2: freenect_set_depth_callback(f_dev, depth_cb2); break;
            default: break;
          }
          //freenect_set_depth_format(f_dev, FREENECT_DEPTH_11BIT);
          //
          // set video format
//         FREENECT_VIDEO_RGB             = 0, /**< Decompressed RGB mode (demosaicing done by libfreenect) */
//         FREENECT_VIDEO_BAYER           = 1, /**< Bayer compressed mode (raw information from camera) */
//         FREENECT_VIDEO_IR_8BIT         = 2, /**< 8-bit IR mode  */
//         FREENECT_VIDEO_IR_10BIT        = 3, /**< 10-bit IR mode */
//         FREENECT_VIDEO_IR_10BIT_PACKED = 4, /**< 10-bit packed IR mode */
//         FREENECT_VIDEO_YUV_RGB         = 5, /**< YUV RGB mode */
//         FREENECT_VIDEO_YUV_RAW         = 6, /**< YUV Raw mode */
          freenect_resolution res;
          switch (varResolution->getInt())
          {
            case 2: res = FREENECT_RESOLUTION_HIGH; break;
            default:
              res = FREENECT_RESOLUTION_MEDIUM;
              if (varResolution->getInt() != 1)
                varResolution->setInt(1);
              break;
          }
          freenect_video_format vFmt;
          switch (varVideoFmt->getInt())
          {
            case 1: vFmt = FREENECT_VIDEO_IR_8BIT; break;
            case 2: vFmt = FREENECT_VIDEO_RGB; break;
            case 3: vFmt = FREENECT_VIDEO_YUV_RAW; break;
            default:
              vFmt = FREENECT_VIDEO_BAYER;
              if (varVideoFmt->getInt() != 0)
                varVideoFmt->setInt(0);
              break;
          }
          videoFmt = freenect_find_video_mode(res, vFmt);
          freenect_set_video_mode(f_dev, videoFmt);
          switch (kn) {
            case 0: freenect_set_video_callback(f_dev, rgb_cb0); break;
            case 1: freenect_set_video_callback(f_dev, rgb_cb1); break;
            case 2: freenect_set_video_callback(f_dev, rgb_cb2); break;
            default: break;
          }
          //
          printf("UFuncKinect::openKinect callbacks set %s\n", bool2str(isOK));
          logf.toLog("UFuncKinect::openKinect setting callbacks", err, "isOK");
          //
          err = 0;
          if (varUseDepth->getBool(0))
          {
            if (vFmt == FREENECT_VIDEO_IR_8BIT and res == FREENECT_RESOLUTION_HIGH)
            {
              printf("**** NB! High resolution IR video prohibits depth\n");
              varUseDepth->setBool(false, 1, true);
            }
            else
            {
              err = freenect_start_depth(f_dev);
              logf.toLog("UFuncKinect::openKinect depth started", err, "err number (0=OK)");
              printf("UFuncKinect::openKinect depth started err=%d number (0=OK)\n", err);
              varUseDepth->setBool(err == 0, 1, true);
              isOK = (err == 0);
              if (frameCntRgb > frameCntDepth)
                frameCntDepth = frameCntRgb;
              frameRateDepthTime.now();
            }
          }
          if (varUseColor->getBool(0))
          {
            err = freenect_start_video(f_dev);
            logf.toLog("UFuncKinect::openKinect video started", err, "err number (0=OK)");
            printf("UFuncKinect::openKinect video started err=%d number (0=OK)\n", err);
            varUseColor->setBool(err == 0, 1, true);
            isOK &= (err == 0);
            if (frameCntDepth > frameCntRgb)
              frameCntRgb = frameCntDepth;
            frameRateRgbTime.now();
          }
          isOK = (err == 0);
          printf("UFuncKinect::openKinect started depth (%s) and camera (%s)\n",
                 bool2str(varUseDepth->getBool(1)), bool2str(varUseColor->getBool(1)));
          if (varTiltUse->getBool())
          {
            freenect_set_tilt_degs(f_dev,varTiltDeg->getInt());
            //varTiltUse->setBool(1, 1);
          }
          // set led to "recording"
          varLed->setValued(LED_RED);
          //freenect_set_led(f_dev,LED_RED);
          varIsOpen->setBool(err == 0, 1);
        }
      }
      else
        doCloseDevice = true;
    }
    if (doCloseDevice)
    { // close the camera
      //printf("UFuncKinect: shutting down streams...\n");
      logf.toLog("UFuncKinect::openKinect closing");
      varIsOpen->setBool(false, 1);
      // turn off LED
      freenect_set_led(f_dev, (freenect_led_options) 0);
      oldLed = 0;
      //
      freenect_stop_depth(f_dev);
      freenect_stop_video(f_dev);
      //
      freenect_close_device(f_dev);
      freenect_shutdown(f_ctx);
      varInitialized->setValued(false);
      varUseDepth->setBool(0, 1);
      varUseColor->setBool(0, 1);
      //varTiltUse->setBool(0, 1);
      logf.toLog("UFuncKinect::openKinect closed and shut");
    }
    unlock();
    //printf("UFuncKinect::openKinect finished\n");
  }
  return varIsOpen->getBool(1) == doOpen;
}

/////////////////////////////////////////////////////

void UFuncKinect::callback_depth(freenect_device *dev, uint16_t *depth, uint32_t timestamp)
{
  UTime t;
  t.now();
  // decode and use image
  UImagePool * imgPool = (UImagePool *)getStaticResource("imgPool", false, false);
  if (imgPool != NULL)
  { // raw depth image
    UImage * depthBW = imgPool->getImage(varImagesC3D->getInt(1), true, 0, 0, 1, 16);
    if (depthBW != NULL)
    {
      int validBits = 11;
      if (depthBW->tryLock())
      {
        depthBW->setName("depth");
        // depth time is set, as the first usb-packet is arrived in the freenect driver
        depthBW->imgTime.SetTime(getDevUse(dev)->depthFrameTime);
        depthBW->imageNumber = frameCntDepth;
        depthBW->camDevice = varCamDeviceNum->getInt(1);
        depthBW->used = false;
        // convert from raw data to 16-bit BW image
        // convert_packed11_to_16bit(uint8_t *sec, uint16_t *dst, int n)
        if (depthFmt.depth_format == FREENECT_DEPTH_11BIT_PACKED)
        {
          depthBW->setSize(depthFmt.height, depthFmt.width, 1, 16, "BW16S");
          switch (varDepthFmt->getInt())
          {
            case 0: // 11 BIT in 16 bit image
              convert_packed11_to_16bit((uint8_t *) depth, (uint16_t*)depthBW->getData(), 640*480);
              depthBW->setName("depth11bit");
              validBits = 11;
              break;
            case 3: // mm in 16 bit image
              freenect_apply_depth_to_mm(dev, (uint8_t *) depth, (uint16_t*)depthBW->getData());
              depthBW->setName("depth_mm");
              validBits = 16;
              break;
            case 4: // registred in 16 bit image
              freenect_apply_registration(dev, (uint8_t *) depth, (uint16_t*)depthBW->getData());
              depthBW->setName("depth_reg");
              validBits = 16;
              break;
            default:
              break;
          }
        }
        else
        { // only other option is 10bit packed
          if (varDepthFmt->getInt() == 1)
          {
            depthBW->setSize(depthFmt.height, depthFmt.width, 1, 16, "BW16S");
            convert_packed_to_16bit((uint8_t *)depth, (uint16_t*)depthBW->getData(), 10, 640*480);
            depthBW->setName("depth10bit");
            validBits = 10;
          }
          else
          {
            depthBW->setSize(depthFmt.height, depthFmt.width, 1, 8, "BW");
            convert_packed_to_8bit((uint8_t *)depth, (uint8_t*)depthBW->getData(), 10, 640*480);
            depthBW->setName("depth8bit");
            validBits = 8;
          }
        };
        //memmove(depthBW->getData(), depth, depthFmt.bytes);
        logDepthImage = depthBW;
        depthBW->updated();
        // there may be need for processed depth images (to get fake color and 8-bit BW)
        processDepthImages(depthBW, validBits);
        // release
        depthBW->unlock();
      }
    }
  }
  //
  varUpdateCnt->add(1.0);  varUpdateCnt->add(1.0);
  frameCntDepth++;
  const int FRAMERATE_IMGS = 10;
  if (frameCntDepth % FRAMERATE_IMGS == 0)
  {
    //int dti = timestamp - frameRateVStamp;
    //frameRateVStamp = timestamp;
    double d = frameRateDepthTime.getTimePassed();
    // debug
    //printf("%d frames took %.3f sec and %d kinect timestamp units (now %u)\n", FRAMERATE_IMGS, d, dti, timestamp);
    // debug
    if (d > 1e-6)
    { // time has passed
      varFramerate->setValued(roundi(FRAMERATE_IMGS / d * 100.0)/100.0, 1, true);
      frameRateDepthTime.now();
    }
  }
}

///////////////////////////////////////////////////////////////////////////

void UFuncKinect::callback_rgb(freenect_device *dev, uint8_t *rgb, uint32_t timestamp)
{
  UTime t;
  t.Now();
  // use the image
  UImagePool * imgPool = (UImagePool *)getStaticResource("imgPool", false, false);
  UImage * rgbImg = NULL;
  //
  if (imgPool != NULL)
    rgbImg = imgPool->getImage(varImagesC3D->getInt(0), true, videoFmt.height, videoFmt.width, 1, 8);
  if (rgbImg != NULL and videoFmt.height > 0 and videoFmt.width > 0)
  {
    if (rgbImg->tryLock())
    {
      rgbImg->imageNumber = frameCntRgb;
      // video frame time is set, as the first usb-packet is arrived in the freenect driver
      rgbImg->imgTime.SetTime(getDevUse(dev)->videoFrameTime);
      unsigned char * pix; // = rgbImg->getUCharRef(0, 0);
      switch ((int) videoFmt.video_format)
      {
        case FREENECT_VIDEO_IR_8BIT:
          // IR image
          rgbImg->setSize(videoFmt.height, videoFmt.width, 1, 8, "BW");
          pix = rgbImg->getUCharRef(0, 0);
          memcpy(pix, rgb, videoFmt.bytes);
          rgbImg->setName("kinect IR8");
          break;
        case FREENECT_VIDEO_BAYER:
          // color image (GRBG)
          // changed colot type to BAYER
          rgbImg->setSize(videoFmt.height, videoFmt.width, 1, 8, "GRBG");
          pix = rgbImg->getUCharRef(0, 0);
          memcpy(pix, rgb, videoFmt.bytes);
          rgbImg->setName("kinect GRBG");
          break;
        case FREENECT_VIDEO_RGB:
          // color image (RGB)
          rgbImg->setSize(videoFmt.height, videoFmt.width, 3, 8, "RGB");
          pix = rgbImg->getUCharRef(0, 0);
          memcpy(pix, rgb, videoFmt.bytes);
          rgbImg->setName("kinect RGB");
          break;
        case FREENECT_VIDEO_YUV_RAW:
          // color image (RGB)
          rgbImg->setSize(videoFmt.height, videoFmt.width, 2, 8, "YUV422");
          pix = rgbImg->getUCharRef(0, 0);
          memcpy(pix, rgb, videoFmt.bytes);
          rgbImg->setName("kinect YUV422");
          break;
        default:
          if (rgbImg->setSizeOnly(videoFmt.height, videoFmt.width))
          {
            pix = rgbImg->getUCharRef(0, 0);
            memcpy(pix, rgb, videoFmt.bytes);
          }
          rgbImg->setName("kinect unknown format");
          break;
      }
      rgbImg->camDevice = varCamDeviceNum->getInt(0);
      logVideoImage = rgbImg;
      rgbImg->updated();
      rgbImg->unlock();
    }
  }
  varUpdateCnt->add(1.0);
  frameCntRgb++;
  const int FRAMERATE_IMGS = 10;
  if (frameCntRgb % FRAMERATE_IMGS == 0)
  {
    //int dti = timestamp - frameRateVStamp;
    //frameRateVStamp = timestamp;
    double d = frameRateRgbTime.getTimePassed();
    // debug
    //printf("%d frames took %.3f sec and %d kinect timestamp units (now %u)\n", FRAMERATE_IMGS, d, dti, timestamp);
    // debug
    if (d > 1e-6)
    { // time has passed
      varFramerate->setValued(roundi(FRAMERATE_IMGS / d * 100.0)/100.0, 0);
      frameRateRgbTime.now();
    }
  }
  setUpdated(NULL);
}

#endif

///////////////////////////////////////////////////

void * startUFuncKinectThread(void * obj)
{ // call the hadling function in provided object
  UFuncKinect * ce = (UFuncKinect *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UFuncKinect::start()
{
  int err = 0;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    err = (pthread_create(&threadHandle, &thAttr,
              &startUFuncKinectThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return err;
}

///////////////////////////////////////////////////

void UFuncKinect::stop(bool andWait)
{
  if (threadRunning and not threadStop)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////


void UFuncKinect::run()
{
  int err = 0;
  int errCnt = 0;//, accCnt = 0;
  UPosition acc;
#ifdef USE_KINECT
  int  accCnt = 0;
  int16_t accOld[3] = {0,0,0};
  int frameRateLow = 0;
#endif
  UTime t, t2, tFr;
  URotation orient;
  // double accTot;
  if (threadRunning)
    // prevent nested calls;
    return;
  threadRunning = true;
  t.now();
  tFr.Now();
  bool timeToLogColor = false;
  int lastImageNumber = 0, lastDepthNumber = 0;
  while (not threadStop)
  { // get framerate and statistics struct from freenect driver
#ifdef USE_KINECT
    int devIdx = maxi(0, mini(KINECT_MAX_DEVICE_USE, varKinectNumber->getInt(0)));
    deviceUseInfo * useInfo = &devUse[devIdx];
#endif
    lastLoopTime.now();
    if (varIsOpen->getBool(1))
    { // is open already
      lock();
#ifdef USE_KINECT
      if (true)
      {
        freenect_raw_tilt_state* state = NULL;
        int useEvery;
        double dfr = fmax(varFramerateDesired->getDouble(), 1e-3);
        if (varResolution->getInt() == 1)
          useEvery = maxi(1, roundi(30.0 / dfr));
        else
          useEvery = maxi(1, roundi(10.0 / dfr));
        //
        if (useEvery > 2)
          useInfo->syncVideoFromDepth = varUseDepth->getBool();
        else
          useInfo->syncVideoFromDepth = 0;
        if (useEvery != useInfo->useDepthFrame or useInfo->syncVideoFromDepth < 0)
        { // always let depth control video frame rate, to get matched pairs
          if (useEvery < useInfo->useDepthFrame)
            // going faster, ensure next available frame is used
            useInfo->useDepthCnt = useEvery;
          useInfo->useDepthFrame = useEvery;
          if (useInfo->syncVideoFromDepth)
            useInfo->useVideoFrame = 10000; // large value, should be triggered by depth
          else
            useInfo->useVideoFrame = useEvery; // video is free running compared to depth
          printf("aukinect::run: Using every %d set of images\n", useEvery);
        }
        double dt = t.getTimePassed();
        double ai = varAccInterval->getValued();
        if (dt > ai or ((useInfo->depthIdle == 1 and useInfo->videoIdle == 1 and dt > ai/2.0)))
        { // prefer to get acc values in image idle time
          freenect_update_tilt_state(f_dev);
          state = freenect_get_tilt_state(f_dev);
          if (state->accelerometer_x != accOld[0] or
              state->accelerometer_y != accOld[1] or
              state->accelerometer_z != accOld[2])
          { // new acc measurement
            t.Now();
            accOld[0] = state->accelerometer_x;
            accOld[1] = state->accelerometer_y;
            accOld[2] = state->accelerometer_z;
            double ax, ay, az;
            freenect_get_mks_accel(state, &ax, &ay, &az);
            acc.set(-az, -ax, -ay);
            varAcc->set3D(&acc, &t);
            accCnt++;
            varTime->setTime(t);
            // calculate new orientation based on acc values
            //accTot = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
            //varAcc->setDouble(accTot, 3);
            orient.Omega = atan2(-acc.y, -acc.z);
            orient.Phi = atan2(acc.x, -acc.z);
            // kappa (yaw) can not be calculated safely
            orient.Kappa = atan2(-acc.y, acc.x);
//             orient.Omega = acos(acc.x / accTot) - M_PI / 2.0;
//             orient.Phi   = acos(acc.z / accTot) - M_PI / 2.0;
//             orient.Kappa = acos(acc.y / accTot);
            varOrient->setRot(&orient, &t);
            //
            if (logAcc.isOpen() and (accCnt % varAccLogN->getInt() == 0))
            { // log acc value
              fprintf(logAcc.getF(), "%lu.%06lu %.4f %.4f %.4f  %.6f %.6f\n",
                      t.GetSec(), t.GetMicrosec(), acc.x, acc.y, acc.z, orient.Omega, orient.Phi);
            }
            if (accCnt >= 20)
            {
              dt = t - t2;
              t2 = t;
              varAccRate->setValued(roundi(accCnt/dt*100.0)/100.0);
              accCnt = 0;
            }
          }
        }
        // test new tilt command
        if (oldTilt != varTiltDeg->getDouble() and varTiltUse->getBool())
        { // new desired tilt value
          const int MSL = 50;
          char s[MSL];
          //
          if (state != NULL)
          {
            oldTilt = varTiltDeg->getDouble();
            freenect_set_tilt_degs(f_dev, oldTilt);
            snprintf(s, MSL, "tilt %6.2f deg set value (measured %g deg)", varTiltDeg->getDouble(), freenect_get_tilt_degs(state));
            logf.toLog(s);
            logImage.toLog(s);
          }
        }
      }
      // test new LED command
      if (oldLed != varLed->getInt())
      {
        freenect_set_led(f_dev, (freenect_led_options) varLed->getInt());
        oldLed = varLed->getInt();
        logf.toLog("set LED to ", oldLed, "(0..7: off,green,red,yellow,blink-Y,blink-G,blink-RY)");
      }
      err = freenect_process_events(f_ctx);
      if (err < 0)
        errCnt++;
#endif
      unlock();
#ifdef USE_KINECT
      if (tFr.getTimePassed() * varFramerateDesired->getDouble() > 15.0)
      { // more than 15 frame intervals has passed, so test result
        if (varFramerate->getDouble() < varFramerateDesired->getDouble() * 0.4 and varIsOpen->getBool())
          frameRateLow++;
        else
          frameRateLow = 0;
        if (frameRateLow > 5)
        { // close and reopen the camera
          const int MSL = 200;
          char ms[MSL];
          openKinect(false);
          snprintf(ms, MSL, "KINECT::run(): closed device, too low framerate %.2f, desired=%.2f", varFramerate->getDouble(), varFramerateDesired->getDouble());
          logf.toLog(ms);
          printf("%s\n", ms);
          Wait(0.5);
          openKinect(true);
          snprintf(ms, MSL, "KINECT::run(): reopened device");
          logf.toLog(ms);
          printf("%s\n", ms);
          frameRateLow = 0;
        }
        tFr.Now();
      }
      if (not varIsOpen->getBool(0))
        // close kinect requested
        openKinect(false);
#endif
    }
    else
    { // update interval times when camera is closed
#ifdef USE_KINECT
      if (varIsOpen->getBool(0))
      { // requested to open kinect, but is not open
        // reset statistics
        // depth stream
        useInfo->depthFrameTime = t.getTimeval();
        useInfo->depthIdle = 0;
        useInfo->useDepthCnt = 0;
        useInfo->useDepthFrame = 0;
        // video stream
        useInfo->videoFrameTime = t.getTimeval();
        useInfo->videoIdle = 0;
        useInfo->useVideoCnt = 0;
        useInfo->useVideoFrame = 0;
        if (not openKinect(true))
          // failed to open - do not retry
          varIsOpen->setValued(0, 0);
      }
#endif
      Wait(0.05);
      t.now();
      tFr.now();
    }
    if (oldCamDevC != varCamDeviceNum->getInt(0))
    { // is camera device changed - so update new device
      UCamPool * camPool = (UCamPool*) getStaticResource("camPool", false);
      if (camPool != NULL)
      {
        int n = varCamDeviceNum->getInt(0);
        UCamPush * cam = camPool->getCam(n);
        const int MSL = 30;
        char s[MSL];
        snprintf(s, MSL, "%sColor", aliasName);
        if (cam == NULL)
        { // new cam is unknown
          UCamPush * camOld = camPool->getCam(oldCamDevD);
          if (camOld != NULL and oldCamDevC >= 0)
            camOld->setMountName("unused");
          cam = camPool->makeDevice(n, s);
        }
        if (cam != NULL)
        {
          cam->setMountName(s);
          cam->setTypeName("KinectColor");
          cam->getDev()->setDevice(640, 480, varFramerateDesired->getInt());
        }
        oldCamDevC = n;
      }
    }
    if (oldCamDevD != varCamDeviceNum->getInt(1))
    { // is depth camera device changed - so update new device
      UCamPool * camPool = (UCamPool*) getStaticResource("camPool", false);
      if (camPool != NULL)
      { // camera pool available
        int n = varCamDeviceNum->getInt(1);
        UCamPush * cam = camPool->getCam(n);
        const int MSL = 30;
        char s[MSL];
        snprintf(s, MSL, "%sDepth", aliasName);
        if (cam == NULL)
        { // new cam is unknown
          UCamPush * camOld = camPool->getCam(oldCamDevD);
          if (camOld != NULL and oldCamDevD >= 0)
            camOld->setMountName("unused");
          cam = camPool->makeDevice(n, NULL);
        }
        if (cam != NULL)
        {
          cam->setMountName(s);
          cam->setTypeName("KinectDepth");
          cam->getDev()->setDevice(640, 480, varFramerateDesired->getInt());
        }
        oldCamDevD = n;
      }
    }
    // test for open/close of logfiles - image log
    logFileTest();
    // should images be logged
    if (logImage.isLogOpen())
    {
      int logn = varImageLogN->getInt();
      if (logDepthImage != NULL)
      { // is may be time to log depth image
        if ((int)logDepthImage->imageNumber - lastDepthNumber >= logn)
        {
          lastDepthNumber = logDepthImage->imageNumber;
          // save image and make an entry in image log
          logImageToFile(logDepthImage);
          //  printf("Logged depth image %d\n", frameLogCnt);
          timeToLogColor = true;
        }
      }
      // is it time to log color image
      if (logVideoImage != NULL)
      { // save image and make an entry in image log
        if (timeToLogColor and logn > 1)
          // force logging of color - triggered by depth
          lastImageNumber = 0;
        if ((int)logVideoImage->imageNumber - lastImageNumber >= logn)
        {
          lastImageNumber = logVideoImage->imageNumber;
          logImageToFile(logVideoImage);
          //printf("Logged video image %d\n", frameLogCnt);
          timeToLogColor = false;
        }
      }
    }
    //
    if (errCnt > 10)
    {
      printf("UFuncKinect::run: %d freenect_process_events errors (lase err=%d) - closing device\n", errCnt, err);
      errCnt = 0;
    }
  }
  threadRunning = false;
}

/////////////////////////////////////////////////////////

void UFuncKinect::logFileTest()
{
  if (logImage.isOpen() and varImageLogN->getInt() == 0)
  {
    logImage.openLog(false);
    logf.toLog("closed imagelog");
  }
  else if (not logImage.isOpen() and varImageLogN->getInt() > 0)
  {
    if (not logImage.openLog(true))
    { // error opening logfile
      printf("******** failed to open %s (setting imageLogN=0)\n", logImage.getLogFileName());
      logf.toLog("failed to open ", logImage.getLogFileName());
      varImageLogN->setValued(0.0);
    }
    else
      logf.toLog("opened imagelog", logImage.getLogFileName());
  }
  // test for logfiles - acc log
  if (logAcc.isOpen() and varAccLogN->getInt() == 0)
    logAcc.openLog(false);
  else if (not logAcc.isOpen() and varAccLogN->getInt() > 0)
  {
    if (not logAcc.openLog(true))
    { // error opening logfile
      printf("******** failed to open %s (setting accLogN=0)\n", logAcc.getLogFileName());
      logf.toLog("failed to open ", logAcc.getLogFileName());
      varAccLogN->setValued(0.0);
    }
    else
      logf.toLog("opened acc log", logAcc.getLogFileName());
  }
}

/////////////////////////////////////////////////////////////

bool UFuncKinect::logImageToFile(UImage * imgToLog)
{
  bool result = false;
  if (varImageLogN->getInt() > 0)
  {
    UCamPool * cams = (UCamPool * ) getStaticResource("campool", false, false);
    if (cams != NULL)
    { // we have a camera
      UCamMounted * cam = cams->getCam(imgToLog->camDevice);
      if (cam == NULL)
      {
        const int MCSL = 20;
        char cs[MCSL];
        snprintf(cs, MCSL, "kinectDev%d", imgToLog->camDevice);
        cams->makeDevice(imgToLog->camDevice, cs);
        cam = cams->getCam(imgToLog->camDevice);
      }
      if (cam != NULL)
      {
        imgToLog->lock();
        logImage.logImage(imgToLog, cam);
        imgToLog->unlock();
        result = true;
      }
    }
  }
  return result;
}

////////////////////////////////////////

bool UFuncKinect::decodeReplayLine(char* line)
{
  varReplayLine->add(1.0, 1);
//   1310653790.415352 36 480 640 1050.00 19 0.000 0.000 0.000 0.0000 0.0000 0.0000 BW16S00000036-cam19-20110714_162950.415.png
//   1310653790.437549 36 480 640 1050.00 18 0.000 0.000 0.000 0.0000 0.0000 0.0000 GRBG00000036-cam18-20110714_162950.437.png
//   1310653793.619357 42 480 640 1050.00 19 0.000 0.000 0.000 0.0000 0.0000 0.0000 BW16S00000042-cam19-20110714_162953.619.png
//   1310653793.637544 42 480 640 1050.00 18 0.000 0.000 0.000 0.0000 0.0000 0.0000 GRBG00000042-cam18-20110714_162953.637.png
  int imgSerial = 0;
  int camDevNum = 0;
  char * p1 = line;
  int state = 0;
  UTime t;
  char * filename = NULL;
  UImagePool * iPool;
  UImage * img;
  while (*p1 >= ' ' and state <= 12)
  {
    switch (state)
    {
      case 0:
        t.setTimeTod(p1);
        p1 = strchr(line, ' ');
        break;
      case 1: // image serial
        imgSerial = strtol(p1, &p1, 10); break;
      case 5: // camera device number
        camDevNum = strtol(p1, &p1, 10); break;
      case 12:
        while (isspace(*p1))
          p1++;
        if (*p1 > ' ')
        {
          filename = p1;
          // remove white space and linefeed at end
          p1 = &filename[strlen(filename) - 1];
          while (*p1 <= ' ')
            *p1-- = '\0';
        }
        break;
      default: // unused numbers
        strtof(p1, &p1);
        break;
    }
    state++;
  }
  if (filename != NULL)
  {
    iPool = (UImagePool *) getStaticResource("imgPool", false, true);
    if (iPool != NULL)
    {
      const int MFL = 500;
      char ffname[MFL];
      bool isOK;
      //
      img = iPool->getImage(camDevNum, true);
      isOK = img != NULL;
      if (isOK)
      {
        img->lock();
        snprintf(ffname, MFL, "%s/%s", replayPath, filename);
        isOK = img->loadPNG(ffname);
      }
      if (isOK)
      {
        img->imageNumber = imgSerial;
        img->imgTime = t;
        img->camDevice = camDevNum;
        img->setName(filename);
        img->updated();
        if (camDevNum == varImagesC3D->getInt(1))
          // this is a depth image - may need extra processing
          processDepthImages(img, 11);
      }
      if (img != NULL)
        img->unlock();
    }
  }
  return true;
}

/////////////////////////////////////////

void UFuncKinect::updateReplayStatus()
{
  varReplayLine->setBool(isReplay(), 0);
}

////////////////////////////////////////

bool UFuncKinect::processDepthImages(UImage * img, int validBits)
{
  UImagePool * iPool;
  bool processDepth = varUseDepthColor->getBool();
  //

  if (processDepth)
  { // make 3D cloud
    iPool = (UImagePool *) getStaticResource("imgPool", false, true);
    if (iPool != NULL)
    {
      int h = img->getHeight();
      int w = img->getWidth();
//       // make corrected depth image and update
//       UImage * cDepthImg = iPool->getImage(varImagesC3D->getInt(3), true, h, w);
//       if (cDepthImg != NULL and varMakePCLFile->getBool())
//       {
//         if (varDepthFmt->getInt() >= 3)
//           cDepthImg = img;
//         else if (cDepthImg->tryLock())
//         {
//           cDepthImg->copyMeta(img, false);
//           cDepthImg->setSize(h, w, 1, 16, "BW16S");
//           cDepthImg->setName("Corrected depth [mm]");
//           cDepthImg->used = false;
//           uint16_t * pix = (uint16_t *) cDepthImg->getData();
//           uint16_t * src = (uint16_t *) img->getData();
//           for (int i = 0; i < h * w; i++)
//           { // correct the
//             // values found at http://stackoverflow.com/questions/8663301/microsoft-kinect-sdk-depth-calibration
//             *pix = t_depth[*src];
//             if(*pix > lims)
//               *pix = 0;
//             src++; pix++;
//           }
//           cDepthImg->updated();
//           cDepthImg->unlock();
//           pix = (uint16_t *) cDepthImg->getData();
//           src = (uint16_t *) img->getData();
//           //printf("[%d],[%d]\n",pix[h/2*w+w/2],src[h/2*w+w/2]);
//           //if depth image is successfull check if PCL file is requested
//         }
//         if (varMakePCLFile->getBool())
//           MakePCLFile(cDepthImg);
//       }
//       // make obj3d point cloud
//       if (varMakeObj3Dcloud->getBool())
//       {
//         UImage * imgC = iPool->getImage(varImagesC3D->getInt(0), false);
//         UImage * imgCrgb = imgC;
//         if (imgC != NULL)
//         {
//           imgC->lock();
//           if (not imgC->isRGB())
//           { // need RGB for coloured point cloud
//             if (imgCbuf == NULL)
//               imgCbuf = new UImage800();
//             imgC->toRGB(imgCbuf);
//             imgCrgb = imgCbuf;
//           }
//           if ((int)imgC->getBufferSize() >= w * h * 3)
//           {
//             if (cloud3d == NULL)
//               cloud3d = new UImg3Dpoints();
//             if (cloud3d->tryLock())
//             {
//               FILE * obj3df = NULL;
//               if (varMake3DFile->getBool())
//               {
//                 const int MNL = 100;
//                 char filename[MNL];
//                 snprintf(filename, MNL, "Kin3D_%06lu.txt", img->imageNumber);
//                 obj3df = fopen(filename, "w");
//                 if (obj3df != NULL)
//                 {
//                   fprintf(obj3df, "# 3d cloud from kinect, image %lu, at %lu.%06lu\n",
//                           img->imageNumber,
//                           img->imgTime.getSec(), img->imgTime.getMicrosec());
//                   fprintf(obj3df, "# format: \n#x(fwd) y(lft) z(hgt) row col  R   G   B\n");
//                 }
//               }
//               cloud3d->clear();
//               cloud3d->serial = img->imageNumber;
//               cloud3d->time = img->imgTime;
//               cloud3d->inPoseCoordinates = true;
//               UCamPool * camPool = (UCamPool *)getStaticResource("camPool", false);
//               if (camPool != NULL)
//               { // get conversion matrix from camera device
//                 // (if available, else result will be in camera coordinates)
//                 UCamMounted * cam;
//                 cam = camPool->getCam(img->camDevice);
//                 if (cam != NULL)
//                   cloud3d->pose = cam->getPosRot();
//               }
//               uint16_t* srcd = (uint16_t*) img->getData();
//               UPixel * srcc = imgCrgb->getData();
//               bool isInmm = varDepthFmt->getInt() >= 3;
//               float d;
//               for (int y = 0; y < h; y++)
//                 for (int x = 0; x < w; x++)
//                 {
//                   if (isInmm)
//                     // is in mm already - use as is
//                     d = *srcd/1000.0;
//                   else if(*srcd > 0 and *srcd < 2047)
//                     // convert using depth table
//                     d = t_depth[*srcd]/1000.0;
//                   else
//                     d = 0;
//                   if (d > 0.01)
//                   {
//                     double py, pz;
//                     py = -d * t_posX[x];
//                     pz = -d * t_posY[y];
//                     // and change to robot frame type - x=fwd, y=left and z=up
//                     cloud3d->add(d, py, pz, y, x, srcc->getInt(), 1.0);
//                     if (obj3df != NULL)
//                       fprintf(obj3df, "%6.3f %6.3f %6.3f  %3d %3d  %3d %3d %3d\n", d, py, pz, y, x,
//                               srcc->p1, srcc->p2, srcc->p3);
//                   }
//                   srcd++;
//                   srcc++;
//                 }
//               if (varMake3DFile->getBool())
//               {
//                 cloud3d->makePCLFile();
//                 varMake3DFile->setBool(false, 0);
//               }
//               cloud3d->unlock();
//               if (obj3df != NULL)
//                 fclose(obj3df);
//             }
//           }
//           imgC->unlock();
//         }
//       }

      // make color depth image
      UImage * depthImg = iPool->getImage(varImagesC3D->getInt(2), true, h, w);
      if (depthImg != NULL and varUseDepthColor->getBool() and img->getDepth() == 16)
      { // only convert if 16 source image
        if (depthImg->tryLock())
        {
          int pval;
          depthImg->copyMeta(img, false);
          depthImg->setColorType(PIX_PLANES_RGB);
          depthImg->setName("kinect RGB depth");
          UPixel * pix = depthImg->getData();
          uint16_t * src = (uint16_t *) img->getData();
          for (int i = 0; i < h * w; i++)
          { // get a good gamma value and convert to color
            if (validBits == 16)
            {
              pval = mini(uint16_t(*src), 0x1fff);
              pval = pval >> 2;
            }
            else
              pval = t_gamma[mini(uint16_t(*src) & 0x7ff, 0x7ff)];
            int lb = pval & 0xff;
            switch (pval>>8) {
              case 0: pix->set(     255, 255-lb, 255-lb);  break;
              case 1: pix->set(     255,     lb,      0);  break;
              case 2: pix->set(255 - lb,    255,      0);  break;
              case 3: pix->set(       0,    255,     lb);  break;
              case 4: pix->set(       0, 255-lb,    255);  break;
              case 5: pix->set(      lb,      0,    255);  break;
              case 6: pix->set(     255,      0, 255-lb);  break;
              default:pix->set(  255-lb,      0,      0);  break;
            }
            src++;
            pix++;
          }
          depthImg->updated();
          depthImg->unlock();
        }
      }
// make an 8-bit BW image of depth, used corrected debth is possible, "varUseDepth8bit" should be updated then
//       int bw8bit = varUseDepth8bit->getInt();
//       if (bw8bit >= 0)
//       { // make an 8-bit BW image of depth
//         UImage * depthBW2 = iPool->getImage(varImagesC3D->getInt(3), true);
//         if (depthBW2 != NULL)
//         {
//           if (depthBW2->tryLock())
//           {
//             depthBW2->copyMeta(img, false);
//             depthBW2->setSize(h, w, 1, 8, "BW");
//             depthBW2->setName("depth-8bit");
//             depthBW2->used = false;
//             uint16_t * ps;
// #ifdef USE_PCL
//             if(varUseCorrectedDepth->getBool())
//             {
//               if (cDepthImg->tryLock())
//                 ps = (uint16_t *) cDepthImg->getData();
//               else
//                 ps = (uint16_t *) img->getData();
//             }
//             else
// #endif
//               ps = (uint16_t *) img->getData();
//             unsigned char * pd = depthBW2->getUCharRef(0, 0);
//             for (int p = 0; p < h * w; p++)
//               *pd++ = mini(((*ps++ - 512) >> bw8bit) , 0xff);
//             depthBW2->updated();
//             depthBW2->unlock();
// #ifdef USE_PCL
//             if(varUseCorrectedDepth->getBool())
//               cDepthImg->unlock();
// #endif
//           }
//         }
//      }
      if (not varUseColor->getBool())
        // color is after depth, so color should
        // trigger events, if both are enabled
        setUpdated(NULL);
    }
  }
  return true;
}
//////////////////////////////////////////////////////////////////////

// int UFuncKinect::MakePCLFile(UImage * img) //makes a single PCD file in
// {
//   const int MNL = 100;
//   char FileName[MNL];
//   int i,x,y;
//   long NumEl;
//   int h = img->getHeight();
//   int w = img->getWidth();
//   uint16_t * src = (uint16_t *) img->getData();
//   // make file
//   FILE* PCLFile;
//   i = snprintf(FileName, MNL, "KinPCL%ld.pcd",img->imageNumber);
//   FileName[i] = 0;
//   printf("%s\n",FileName);
//   PCLFile = fopen(FileName,"w");
//   printf("writing to file(%ld)\n",(long)PCLFile);
//   int lims = (int)(varMaxUsedDepth->getDouble(0)*1000);
//   if(PCLFile != NULL)
//   {
//     //count number for valid elements
//     NumEl = 0;
//     for (y=0;y<h;y++)
//       for (x=0;x<w;x++)
//       {
//         if((*src > 0) && (*src < lims))
//           NumEl++;
//         src++;
//       }
//
// //file format found at http://pointclouds.org/documentation/tutorials/pcd_file_format.php#pcd-file-format
// /*
// # .PCD v.7 - Point Cloud Data file format
// VERSION .7
// FIELDS x y z rgb
// SIZE 4 4 4 4
// TYPE F F F F
// COUNT 1 1 1 1
// WIDTH 213
// HEIGHT 1
// VIEWPOINT 0 0 0 1 0 0 0
// POINTS 213
// DATA ascii
// 0.93773 0.33763 0 4.2108e+06
// ...
// */
//    fprintf(PCLFile,"# .PCD v.7 - Point Cloud Data file format\n");
//    fprintf(PCLFile,"VERSION .7\n");
//    fprintf(PCLFile,"FIELDS x y z\n");
//    fprintf(PCLFile,"SIZE 4 4 4\n");
//    fprintf(PCLFile,"TYPE F F F\n");
//    fprintf(PCLFile,"COUNT 1 1 1\n");
//    fprintf(PCLFile,"WIDTH %ld\n",NumEl);
//    fprintf(PCLFile,"HEIGHT %d\n",1);
//    fprintf(PCLFile,"VIEWPOINT 0 0 0 1 0 0 0\n");
//    fprintf(PCLFile,"POINTS %ld\n",NumEl);
//    fprintf(PCLFile,"DATA ascii\n");
// // white all data points
//    src = (uint16_t *) img->getData();
//     for (y=0;y<h;y++)
//       for (x=0;x<w;x++)
//       {
//         if(*src > 0)
//           fprintf(PCLFile,"%f %f %f\n",(float)(*src) * t_posX[x]/1000,(float)(*src) * t_posY[y]/1000,(float)*src/1000);
//         src++;
//       }
//    fclose(PCLFile);
//     printf("writing done\n");
//   }
//
//   //varMakePCLFile->setInt(0, 0, true); //clear make file flag
//   if(PCLFile != NULL)
//     return 1;
//   else
//     return 0;
// }
////////////////////////////////////////////////////////////////////////

void UFuncKinect::updateCloud(bool color)
{
#ifdef USE_PCL
  UPcpItem * pcp;
  int pntCnt;
  UImage * imgD;
  const char * pcpName = "kinect";
  pcp = getPointCloudXYZ(pcpName, color);
  UImagePool * iPool;
  iPool = (UImagePool *) getStaticResource("imgPool", false, true);
  imgD = iPool->getImage(varImagesC3D->getInt(1), false);
  if (pcp != NULL and iPool != NULL and imgD != NULL)
  {
    pcp->lock();
    UCamPool * camPool = (UCamPool*) getStaticResource("camPool", false);
    UCamMounted * cam = NULL;
    if (camPool != NULL)
    {
      int n = varCamDeviceNum->getInt(0);
      cam = camPool->getCam(n);
    }
    if (varUseColor->getBool())
      pntCnt = GetXYZCoordinates(pcp->xyzrgbp, 64000, 1, varReductionFactor->getInt());
    else
      pntCnt = GetXYZCoordinates(pcp->xyzp, 64000, 0, varReductionFactor->getInt());
    // debug
    if (false and pntCnt != pcp->getPointsCnt())
      printf("UFuncKinect::updateCloud: Error en creating point cloud "
             "(made %d points, but cloud has %d! (image found locked)\n",
             pntCnt, pcp->getPointsCnt());
    // debug end
    pcp->sensorTime = imgD->getImageTime();
    if (cam != NULL)
    {
      pcp->relPose = cam->getPosRot();
      pcp->relPoseUse = true;
    }
    pcp->cooSys = 0;
    pcp->updateTime.now();
    pcp->shapeType = UPcpItem::cloud;
    if (not setPointCloudUpdated(pcpName))
      printf("UFuncKinect::updateCloud: failed to tell PCP that a point cloud (%s) is updated\n", pcpName);
    pcp->unlock();
  }
#endif
}

///////////////////////////////////////////////////////////////////

int UFuncKinect::GetXYZCoordinates(void *pointCloud, int Max,int type, int redFac)
// returns all valid points as a long list of XYZ
// 4 options:
// 1: XYZ & corrected depth ok
// 2: XYZ & corrected depth not ok
// 1: XYZRGB & corrected depth ok
// 2: XYZRGB & corrected depth not ok
{

  int k = 0; // pointcloud index
#ifdef USE_PCL
  bool result = false;
  int y = 0; // Y counter
  int x = 0; // X counter
  int dLims = (int)(varMaxUsedDepth->getDouble(0)*1000);
  if (Max == -1)
    Max = 640*480;
  // limit reduction factor to sane value
  if (redFac < 1)
    redFac = 1;
  if (redFac > 32)
    redFac = 32;
  if (not varSilent->getBool())
    printf("UFuncKinect::GetXYZCoordinates:: distance limit:%d mm, reduction factor %d\n",dLims, redFac);
  UImagePool * iPool;
  iPool = (UImagePool *) getStaticResource("imgPool", false, true);
  result = (iPool != NULL);
  if (result)
  {
    result = pointCloud != NULL;
    if (not result)
      printf("UFuncKinect::GetXYZCoordinates: no destination point cloud provided\n");
  }
  if (result)
  {
    UCamPool * camPool = (UCamPool*) getStaticResource("camPool", false);
    UCamMounted * cam = NULL;
    if (camPool != NULL)
      cam = camPool->getCam(varCamDeviceNum->getInt(0));
    if (cam != NULL)
    { // make sure table for 3D conversion is updated
      int fl = cam->getCamPar()->getFocalLength();
      if (fl < 10)
        fl = 595;
      if (fl != oldFocalLength)
        initXYTable(fl);
    }
    else if (oldFocalLength == 0)
      // use some appropriate size
      initXYTable(595);
    if(type == 0) // if XYZ cloud
    {
      //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_o (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud;// = &cloud_o;
      //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (reinterpret_cast<pcl::PointCloud<pcl::PointXYZ> *>(pointCloud)); // make a cloud that is pointing to the link
      cloud = reinterpret_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr *>(pointCloud);
      //
      UImage * cDepthImg = iPool->getImage(varImagesC3D->getInt(1), true, 480, 640);
      //
      //
      if (cDepthImg != NULL)
      {
        if (cDepthImg->tryLock())
        {
          int h = cDepthImg->getHeight();
          int w = cDepthImg->getWidth();
          std::vector <pcl::PointXYZ, Eigen::aligned_allocator_indirection <pcl::PointXYZ> >::iterator pnt;
          (*cloud)->width  = h*w;
          (*cloud)->height = 1;
          (*cloud)->points.resize ((*cloud)->width / redFac * (*cloud)->height / redFac);
          uint16_t * pix;
          //run through all pixels
          pnt = (*cloud)->points.begin();
          for(y = 0; y < h; y += redFac)
          {
            pix = (uint16_t *) cDepthImg->getLine(y);
            for(x = 0; x < w && k < Max; x += redFac)
            {
                // if the pixel value is valid
                if((*pix > 5) && (*pix < dLims))
                {
                  float d = *pix/1000.0;
                  pnt->y = -d * t_posX[x];
                  pnt->z = -d * t_posY[y];
                  pnt->x =  d;
                  k ++;
                  pnt++;
                }
                pix+=redFac;
            }
          }
          //if (not varSilent->getBool())
            printf("UFuncKinect::GetXYZCoordinates: saved %d points of %d (%dx%d)\n", k, w*h, h, w);
          cDepthImg->unlock();
          (*cloud)->width  = k;
          (*cloud)->points.resize (k);
        } // if try lock
        else if (not varSilent->getBool())
        {
          printf("UFuncKinect::GetXYZCoordinates: found depth image locked (serial %lu) ignoring request\n", cDepthImg->imageNumber);
        }
      }// if cdept!=0
    }
    if(type == 1) // if XYZrgb cloud
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr *cloud;// = &cloud_o;
      cloud = reinterpret_cast<pcl::PointCloud<pcl::PointXYZRGB>::Ptr *>(pointCloud);
      UImage * depthImg = iPool->getImage(varImagesC3D->getInt(1), false);
      UImage * RGBImg = iPool->getImage(varImagesC3D->getInt(0), false);
      if (RGBImg != NULL and depthImg != NULL)
      {
//         printf("UFuncKinect::GetXYZCoordinate1 img=%lud img=%lx w=%ud h=%d channels=%ud depth=%d colfmt=%s\n",
//                    depthImg->imageNumber, (unsigned long)depthImg, depthImg->width(), depthImg->height(), depthImg->getChannels(),
//                    depthImg->getDepth(), depthImg->getColorTypeString());
        RGBImg->lock();
        RGBImg->toRGB();
        if (depthImg->tryLock())
        {
          int h = depthImg->getHeight();
          int w = depthImg->getWidth();
          uint16_t * pix;
          UPixel * cpix;
          std::vector <pcl::PointXYZRGB, Eigen::aligned_allocator_indirection <pcl::PointXYZRGB> >::iterator pnt;
          (*cloud)->width  = h*w;
          (*cloud)->height = 1;
          (*cloud)->points.resize ((*cloud)->width / redFac * (*cloud)->height / redFac);
          //run through all pixels
          pnt = (*cloud)->points.begin();
          for(y = 0;y < h;y += redFac)
          {
            pix = (uint16_t *) depthImg->getLine(y);
            cpix = RGBImg->getLine(y);
            for(x = 0;x < w;x += redFac)
            {
                // if the pixel value is valid
                if(*pix > 305 && *pix < dLims)
                {
                  float d = *pix/1000.0;
                  pnt->y = -d * t_posX[x];
                  pnt->z = -d * t_posY[y];
                  pnt->x =  d;
                  pnt->rgba = (uint32_t(cpix->p1) << 16) + (uint32_t(cpix->p2) << 8) + cpix->p3;
                  k ++;
                  pnt++;
                }
                pix += redFac;
                cpix += redFac;
            }
          }
          depthImg->unlock();
          (*cloud)->width  = k;
          (*cloud)->points.resize ((*cloud)->width * (*cloud)->height);
        }// if try locvk
        RGBImg->unlock();
      }// if =0
    }
    else
    {
      //return error;
    }
  }// if ipool
#else
  printf("PCL library support not enabled - set USE_PCL to 1.4 (if version 1.4) and recompile aurs\n");
#endif
  return k;
}

//////////////////////////////////////////////////////////////

bool UFuncKinect::makePCD(const char * name, bool andRowCol)
{
  bool result;
  result = makeObj3Dcloud();
  if (result)
  {
    cloud3d->lock();
    cloud3d->makePCLFile(name, true, andRowCol);
    cloud3d->unlock();
  }
  return result;
}

//////////////////////////////////////////////////////////////

bool UFuncKinect::makeObj3Dcloud()
{
  UImagePool * iPool = (UImagePool *)getStaticResource("imgPool", false, false);
  UImage * imgC = iPool->getImage(varImagesC3D->getInt(0), false);
  UImage * imgD = iPool->getImage(varImagesC3D->getInt(1), false);
  UImage * imgCrgb = imgC;
  bool result = imgC != NULL and imgD != NULL;
  if (result)
  {
    UCamPool * camPool = (UCamPool*) getStaticResource("camPool", false);
    UCamMounted * cam = NULL;
    if (camPool != NULL)
      cam = camPool->getCam(varCamDeviceNum->getInt(0));
    if (cam != NULL)
    { // make sure table for 3D conversion is updated
      int fl = cam->getCamPar()->getFocalLength();
      if (fl < 10)
        fl = 595;
      if (fl != oldFocalLength)
        initXYTable(fl);
    }
    else if (oldFocalLength == 0)
      // use some appropriate size
      initXYTable(595);
    //
    imgD->lock();
    imgC->lock();
    int w = imgD->getWidth();
    int h = imgD->getHeight();
    int redFac = varReductionFactor->getInt();
    if (not imgC->isRGB())
    { // need RGB for coloured point cloud
      if (imgCbuf == NULL)
        imgCbuf = new UImage800();
      imgC->toRGB(imgCbuf);
      imgCrgb = imgCbuf;
    }
    // is conversion valid
    result = (int)imgCrgb->getBufferSize() >= w * h * 3;
    if (result)
    { // we have depth in imgD and color (in RGB) in imgCrgb
      // now make a 3D point cloud (not a PCL point cloud)
      if (cloud3d == NULL)
        cloud3d = new UImg3Dpoints();
      result = cloud3d->tryLock();
      if (result)
      { // cloud is available and locked
        cloud3d->clear();
        cloud3d->serial = imgD->imageNumber;
        cloud3d->time = imgD->imgTime;
        cloud3d->inPoseCoordinates = true;
        UCamPool * camPool = (UCamPool *)getStaticResource("camPool", false);
        if (camPool != NULL)
        { // get conversion matrix from camera device
          // (if available, else result will be in camera coordinates)
          UCamMounted * cam;
          cam = camPool->getCam(imgD->camDevice);
          if (cam != NULL)
            cloud3d->pose = cam->getPosRot();
        }
        uint16_t* srcd;
        UPixel * srcc;
        bool isInmm = varDepthFmt->getInt() >= 3;
        float d;
        for (int y = 0; y < h; y+= redFac)
        {
          srcd = (uint16_t *) imgD->getLine(y);
          srcc = imgCrgb->getLine(y);
          for (int x = 0; x < w; x+= redFac)
          {
            if (isInmm)
              // is in mm already - use as is
              d = *srcd/1000.0;
            else if(*srcd > 0 and *srcd < 2047)
              // convert using depth table
              d = t_depth[*srcd]/1000.0;
            else
              d = 0;
            if (d > 0.01)
            {
              double py, pz;
              py = -d * t_posX[x];
              pz = -d * t_posY[y];
              // and change to robot frame type - x=fwd, y=left and z=up
              cloud3d->add(d, py, pz, y, x, srcc->getInt(), 1.0);
            }
            srcd += redFac;
            srcc += redFac;
          }
        }
        cloud3d->unlock();
      }
    }
    imgC->unlock();
    imgD->unlock();
  }
  return result;
}

//////////////////////////////////////////////////////

#ifdef USE_PCL

UPcpItem * UFuncKinect::getPointCloudXYZ(const char * name, bool andRGB)
{
  const int MNP = 2;
  UVariable varName;
  UVariable varType;
  UVariable * params[MNP] = {&varName, &varType};
  UDataBase * cloudItem[1] = {NULL};
  int retCnt = 1;
  bool isOK;
  UPcpItem * result = NULL;
  //
  varName.setValues(name, 0, true);
  if (andRGB)
    varType.setValued(UPcpItem::PointXyzRgb);
  else
    varType.setValued(UPcpItem::PointXyz);
  isOK = callGlobalV("pcp.getCloud", "sd", params, cloudItem, &retCnt);
  if (isOK and cloudItem[0] != NULL)
  {
    if (cloudItem[0]->isA("pcpitem"))
      result = (UPcpItem*) cloudItem[0];
  }
  return result;
}

#endif

/////////////////////////////////////////////////////////////////

bool UFuncKinect::setPointCloudUpdated(const char * name)
{
  const int MNP = 1;
  UVariable varName;
  UVariable * params[MNP] = {&varName};
  int retCnt = 0;
  bool isOK;
  //
  varName.setValues(name, 0, true);
  isOK = callGlobalV("pcp.updated", "s", params, NULL, &retCnt);
  return isOK;
}

