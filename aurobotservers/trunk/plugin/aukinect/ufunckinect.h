/***************************************************************************
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
 *
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/

#ifndef UFUNC_AUKINECT_H
#define UFUNC_AUKINECT_H

#include <stdint.h>
#include <libusb.h>

#include <ugen4/uimg3dpoint.h>
#include <urob4/ufuncplugbase.h>
#include <ucam4/uimagelog.h>


// #define USE_KINECT
// #define USE_PCL
#ifdef USE_KINECT
#include <libusb.h>
#include <libfreenect.h>
#include <registration.h>
#include <ucam4/uimagelog.h>
#endif

class UPcpItem;
/**
 * This is a simple example plugin, that manipulates global variables, both of its own, and those of other modules.
@author Christian Andersen
*/
class UFuncKinect : public UFuncPlugBasePush
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncKinect) followed by
  // a descriptive extension for this specific plug-in
public:
  /**
  Constructor */
  UFuncKinect()
  { // command list and version text
    setCommand("kinect kinectPush", "kinect", "Plug-in to get data from XBox 360 RGB and 3D camera");
    init();
  }
  /**
  Destructor */
  ~UFuncKinect();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  * Function to implement a var-pool method call.
  * \param name is the name of the called function
  * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
  * \param strings is an array of string pointers for the string type parameters (may be NULL if not used)
  * \param doubles is an array with double typed parameters (may be NULL if not used)
  * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true 2.0 for implicit stop if a controll call.
  * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
  * \param returnStructCnt is the number of objects in the returnStruct buffer
  * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCall(const char * name, const char * paramOrder, char ** strings, const double * pars,
                      double * value, UDataBase ** returnStruct, int * returnStructCnt);
  /**
  * Function to implement an inter plug-in method call using UVariables as parameter (and return value).
  * Structures of the base type UDatabase (but not UVariables) may be returned in the returnStruct pointer array.
  * The returnStructCnt should be set to returned count, if values are returned.
  * \param name is the name of the called function
  * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
  * \param params is an array of UVariable pointers, one pointer for each character in paramOrder. NB! params are not declared as constant, and
  *         may be used to return values.
  * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
  * \param returnStructCnt is the number of objects in the returnStruct buffer
  * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCallV(const char * name, const char * paramOrder, UVariable * params[],
                           UDataBase ** returnStruct, int * returnStructCnt);
  /**
  Called from push implementor to get the push object.
  should call to 'gotNewData(object)' with the available event object.
  if no object is available (anymore), then call with a NULL pointer. */
  virtual void callGotNewDataWithObject();
  //
  /**
  Decode this replay line, that is assumed to be valid - has got a timestamp.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
  virtual bool decodeReplayLine(char * line);
  /**
  is called when it is time to update replay status (after a step in some replay file) */
  virtual void updateReplayStatus();


private:
  /**
  Initialize other variables */
  void init();
  /**
   * Init table for fast calculation of 3D position from depth image
   * \param focalLength is the estimated focal length for the color camera in pixel.
   * This assumes, that the depth image is rectified to match the color image. */
  void initXYTable(int focalLength);
  /**
  produce fake disparity images - if requested.
  \param img is the true 16-bit depth image to process
  \param validBits is number of valid bits in img
  \returns true if successful - i.e. source images available*/
  bool processDepthImages(UImage * img, int validBits);
  /**
  Handle stereo-push commands */
  bool handleKinect(UServerInMsg * msg);
  /// start read thread
  bool start();
  /** stop read thread
  \param andWait waits until thread is terminated, when false, then the call returns
  when the stop flag is set (i.e. immidiately). */
  void stop(bool andWait);
  /**
   * Tell point cloud pool (PCP) that a point cloud owned by pcp, is updated
   * \param name is name of updated point cloud.
   * \returns true if call was successful */
  bool setPointCloudUpdated(const char * name);

#ifdef USE_KINECT
  /**
   * Get named point cloud from point cloud pool
   * \param name is the name of the cloud in the point cloud pool (PCP)
   * \param andRGB if true, then the cloud is pcl::PointXYZRGB type else it is pcl::PointXYZ.
   * \returns a pointer to a UPcpItems structure with pointer to the point cloud */
  UPcpItem * getPointCloudXYZ(const char * name, bool andRGB);

  /**
  Open or close camera stream */
  bool openKinect(bool doOpen);

public:
  /**
  Callback function for depth image  */
  //void callback_depth(freenect_device *dev, freenect_depth *depth, uint32_t timestamp);
  void callback_depth(freenect_device *dev, uint16_t *depth, uint32_t timestamp);
  /**
  Callback function for color (RGB) image */
  void callback_rgb(freenect_device *dev, uint8_t *rgb, uint32_t timestamp);
#endif
public:
  /**
  Run receive thread */
  void run();
  // returns the 3D point cloud as a long list of variables
  /**
   * get the imageset as a point cloud
   * \param[out] pointCloud the point cloud to change
   * \param Max is the maximum distance (in mm or depth value) to add
   * \param type is 0 for XYZ and 1 for XYZRGB format
   * \param redFac is an optional reduction factor - to reduce number of points. 1= all points, 2=every other, 3=every third ...
   * \returns number of points added */
  int GetXYZCoordinates(void *pointCloud, int Max,int type, int redFac);
  /**
   * Make PCL file from depth image without using PCL libraries */
  // int MakePCLFile(UImage * img);
  /**
   * Update point-cloud in point cloud pool (PCP), then name of the cloud is KINECT.
   * \param color if true, then an XYZRGB cloud is maintained, else just a XYZ cloud. */
  void updateCloud(bool color);

  /**
   * Make 3D object from depth and color images - for colors to match, the format most be "rectified"
   * \returns true if both a depth and color image is available (in imagesC3D[0] and imagesC3D[1]).*/
  bool makeObj3Dcloud();
  /**
   * Make 3D object, either in PCD format for PCD viewer or in the old 3D format used by obj3d plugin.
   * \param name is an optional filename for the generated file (NB same nale for both file formats).
   * \param andRowCol adds row and column from source image to the file
   * \returns true if both a depth and color image is available (in imagesC3D[0] and imagesC3D[1]).*/
  bool makePCD(const char * name, bool andRowCol);
private:
  /**
   * Test for open or close logfiles using global variable */
  void logFileTest();
  /**
   * Save image to imagelog and save the image itself in the imagePath.
   * if camera device do not exist, then a default device is created..
   * \param imgToLog is the image to be logged. */
  bool logImageToFile(UImage * imgToLog);

private:

  static const int MWL = 100;
  /**
  Short explanation to why processing failed */
  char whyString[MWL];
  /**
  Pointers to "own" global variables. */
  UVariable * varUpdateCnt;
//  UVariable * varPoseOnRobot;
  UVariable * varTime;
  /**
  processing parameters */
  UVariable * varSilent;
  UVariable * varImagesC3D;
  UVariable * varCamDeviceNum;
  UVariable * varIsOpen;
  UVariable * varFramerateDesired;
  UVariable * varFramerate;
  UVariable * varResolution;
  UVariable * varVideoFmt;
  UVariable * varDepthFmt;
  UVariable * varTiltDeg;
  UVariable * varTiltUse;
  UVariable * varLed;
  UVariable * varAcc;
  UVariable * varOrient;
  UVariable * varAccRate;
  UVariable * varAccLogN;
  UVariable * varImageLogN;
  UVariable * varAccInterval;
  UVariable * varUseColor;
  UVariable * varUseDepth;
  UVariable * varUseCorrectedDepth;
  UVariable * varKinectNumber;
//  UVariable * varUseDepth8bit;
  UVariable * varUseDepthColor;
  UVariable * varReplayLine;
//  UVariable * varMakePCLFile;
//  UVariable * varMake3DFile;
  UVariable * varMaxUsedDepth;
  UVariable * varReductionFactor;
//  UVariable * varMakeObj3Dcloud;

  int oldLed;
  double oldTilt;
  int oldCamDevC, oldCamDevD;
  const char * aliasname;
  /// is kinect initialized
  UVariable * varInitialized;
  /**
  General Logfile */
  ULLogFile logf;
  /**
  Logfile for acceleration measurement*/
  ULLogFile logAcc;
  /**
  Logfile */
  UImageLog logImage;
  /// pointer to depth image to log
  UImage * logDepthImage;
  /// pointer to video (color or IR) image to log
  UImage * logVideoImage;
  /**
  Array with 3d points */
  UImg3Dpoints * cloud3d;
  /**
   * Kinect loop keep-alive time */
  UTime lastLoopTime;

#ifdef USE_KINECT
  // kinect specific typed variables
  /**
  freenect handles */
  freenect_context *f_ctx;
  /// freenect device handle
  freenect_device *f_dev;
  /// libusb context handle
  /*freenect_usb_context*/ libusb_context * usbCtx;
  /// video format structure
  freenect_frame_mode depthFmt;
  /// video format structure
  freenect_frame_mode videoFmt;
#endif

  /// number of accepter depth frames
  int frameCntDepth;
  /// number of accepted video frames (rgb or ir)
  int frameCntRgb;
  /// frame count for logging
  int frameLogCnt;
  /// variable for frame rate calculation
  UTime frameRateDepthTime;
  /// variable for frame rate calculation
  UTime frameRateRgbTime;
  /**
  Gamma correction table from 11-bit depth - for better display */
  uint16_t t_gamma[2048];
  /**
  Depth correction table for kinect */
  uint16_t t_depth[2048];
  /**
  X-Position correction table for kinect */
  float t_posX[640];
  /**
  Y-Position correction table for kinect */
  float t_posY[480];
  /// last used focal length for rectified image
  int oldFocalLength;
  /// obj3d data structure
  UImg3Dpoints * obj3dcloud;
  /// image buffer for color in 3d cloud
  UImage * imgCbuf;
  /// thread runnung flag
  bool threadRunning;
  /// stop thread flag
  bool threadStop;
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
};


#endif

