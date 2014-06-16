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

/*! \file ucamdevguppy.h
    \brief firewire camera device integration using dc1394 library version 22 (or newer)
    \author Christian Andersen <jca@elektro.dtu.dk>

    Is used by camera pool (ucampool.h) in the ucam4 library.
    Uses a thread to keep the image queue empty.
*/
#ifndef UCAMDEVGUPPY_H
#define UCAMDEVGUPPY_H

//#define USE_GUPPY
#ifdef USE_GUPPY
#include <libraw1394/raw1394.h>
#include <dc1394/dc1394.h>
#endif

#include "ucamdevbase.h"
#define FIRST_IEEE1394_DEVICE_NUMBER 10

class UVariable;

/**
Camera device type that controls one IEEE1394 DCI camera
Device number is from 10 to 19 - defined in URob4/UCamPool

	@author Christian Andersen <jca@elektro.dtu.dk>
*/
class UCamDevGuppy : public UCamDevBase
{
public:
  /**
  Constructor */
  UCamDevGuppy();
  /**
  Constructor */
  ~UCamDevGuppy();

#ifdef USE_GUPPY
    /**
  Set image push image buffer - if one buffer is needed only.
  \param img is the image buffer - typically from image pool. */
  virtual void setPushBuffer(UImage * imageBuffer);

  /**
  Get number of available IEEE1394 devices */
  int getIeee1394PortCnt();
  /**
  Set camera node info */
  bool setCamDeviceNode(int camNum);
  /**
  Get number of cameras as reported by DC1394 */
  int getDc1394CamCnt();
  /**
  Get basic camera info */
  void getCamInfo();
  /**
  Get full camera feature set */
  bool getCamFeatures();
  /**
  \brief Set format 7 - p.t. in full frame mode.
  ROI mode to be added later.
  */
  bool setFormat7();
  /**
  Set format other than format 7 */
  bool setFormat0mode5();
  /**
  Capture an image after iso-streaming is started */
  bool getSingleImage(UImage * destination);
  /**
  Start iso transmission */
  bool startIsoTransmission();
  /**
  Stop iso transmission */
  bool stopIsoTransmission();
  /**
  Setup dma image capture */
  bool setDMAcapture();
  /**
  Set gain value.
  A negative value (-1) sets automatic gain.
  A positive value in legal range is video gain in
  camera. 0 is low gain (but not zero).
  Sets desired values as global vars, and assumes camera thread implements.
  \Returns true if set successful - i.e. the function is supported by camera.*/
  virtual bool setGain(int agc);
  /**
  Set actual gain value (in the range -1 (auto) or manual 0..0xFFFF).
  \returns existing vaue is returned if 'probe' is false,
  if 'probe' is true, then the camera is asked
  for the most recent value. if this is
  not possible, then en error is reported and last
  value returned.
  If dataValid is set, then this is set false on error */
  virtual int getGain(bool probe, bool * dataValid = NULL, bool * isOnAuto = NULL);
  /**
  Set shutter value.
  A negative value (-1) sets automatic gain.
  A positive value in range 0..0xFFFF is shutter setting in
  camera. 0 is short gain (but not zero).
  \Returns true if set successful - i.e. the function is supported by camera.*/
  virtual bool setShutter(int value);
  /**
  Set actual gain value.
  \returns existing vaue is returned if 'probe' is false,
  if 'probe' is true, then the camera is asked
  for the most recent value. if this is
  not possible, then en error is reported and last
  value returned.
  If dataValid is set, then this is set false on error */
  virtual int getShutter(bool probe, bool * dataValid = NULL, bool * isOnAuto = NULL);
  /**
  Test if souch device exist and can be opened.
  Returns true if device can be opened (a device exists). */
  virtual bool deviceExist();
  /**
  Open devise to get default camera parameters.
  Returns true iff successful. */
  virtual bool openDeviceDefault();
  /**
  Open devise to get default camera parameters.
  Returns true iff successful. */
  virtual bool openDevice()
  { return openDeviceDefault(); };
  /**
  close if the device is open */
  virtual void closeDevice();
  /**
   * close and release all resources */
  void closeDeviceRelease();
  /**
  Get new image - the newest available to the provided image buffer.
  if 'image' == NULL, then get and discard an image.
  \returns true if an image could be captured. */
  virtual bool getImageSnapshot(UImage * image);
  /**
  Get current frame rate setting */
  virtual int getFrameRate();
  /**
  implement these
  settings and leave the device open. */
  virtual bool setDevice(const int width, const int height,
                         const int framesPerSec);
  /**
  Stop the frame read thread.
  If 'wait' == true, then the thread are joined prior to return. */
  void stop(bool andWait);
  /**
  Star the read thread.
  Waits until the thread has had time to start and initialize.
  \returns true if the thread is started. */
  bool start();
  /**
  Thread that read frames off the camera device driver queue to allow camPush functions to be
  executed.
  If no users of the frames, they are just discarded.
  If single snapshots are needed, these are serviced by this thread too.
  Should not be called directly, are called implicitly,
  when camera is opened.*/
  void run();
  /**
  \brief Set white balance correction values.
  \param mode can have one of the following values:
  4: PWC_WB_AUTO,
  3: PWC_WB_MANUAL,
  2: PWC_WB_INDOOR,
  1: PWC_WB_OUTDOOR,
  0: PWC_WB_FL (flurocent).
  \param red and
  \param blue is used in MANUAL mode only
  and in a range from 0 to 1022, where 511 is
  assumed to be neutral (equal to gain in green channel)
  \Returns true if successful.  */
  bool setWhiteBalance(int mode, int red, int blue);
  /**
  @brief Get white balance values
  @param probe the the camera is asked, otherwise previous value
  is returned unchecked.
  If one or more of the following pointers are non-zero
  the integer value is returned.
  @returns true if probed succesfull or probe=false. */
  bool getWhiteBalance(bool probe, int * red = NULL, int * blue = NULL, int * mode = NULL);
/**
  Set external trigger
  \param value set to external trigger if true, else internal (free run) trigger.
  \param supported is set true if external trigger is supported by device.
  \returns true if value is set as specified. */
  virtual bool setExternalTrigger(bool value, bool * supported);
/**
  Get external trigger support
  \param value set to true if external trigger is on, false if free running.
  \param supported is set true if external trigger is supported by device.
  \returns true if calls were successful. */
  virtual bool getExternalTrigger(bool * value, bool * supported);
  /**
  Get exposure target value (used in auto shutter mode only)
  \param supported is set true if external trigger is supported by device.
  \param minValue is set to the minimum value - if supported, else unchanged.
  \param maxValue is set to the maximum value - if supported, else unchanged.
  \returns the actual value of exposure target - returns -1 if not supported. */
  //virtual int getExposureTarget(bool * supported, int * minValue, int * maxValue);
  /**
  Set exposure target value (used in auto shutter mode only)
  \param value the value to set - is truncated to within allowed range.
  \returns true if set. */
  //virtual bool setExposureTarget(int value);
  /**
  Toggle the control 2 output (pin 2 on Guppy) on and off, to trigger a new set of images, that
  is if pin 2 is connected to pin 4 on this and the other camera(s) to be triggered.
  Does nothing if not connected and if external trigger is not available.
  \returns true if calls were successfull. */
  virtual bool makeTriggerPulse();
  /**
  This function is called in server thread time and expects that the function
  provides the most recent image, and calls gotNewImage(UImage*) with this
  image as a parameter. This is then used as a parameter to the push call. */
  inline virtual void callGotNewDataWithObject();

protected:
  /**
  Get the bayer pattern from the camera.
  Result is one of the PIX_PLANEX_BGGR integers. */
  int getBayerPattern();
  /**
  Get one of the feature sets stored in 'features', holding
  requested data. */
//  dc1394_feature_info * getFeature(unsigned int featureId);
  /**
  Convert a frames per second (integer) value to the possible
  framerate constants defined in mode 5 ieee1394 settings, i.e.
  FRAMERATE_1_875 = 32 for 1.875 frames / sec. */
  int fps2mode5framerate(int fps);
  /**
  Convert the bit-position framerate to interger framerate in frames per second (rounded to closest integer) */
  int framerate2fps(int framerate);
  /**
  Print camera infor to console */
  void print_mode_info(dc1394camera_t *camera , uint32_t mode);
  /**
  Print format name to console */
  void print_format(uint32_t format);
  /**
  Release BW left over form crash or similar */
  void release_iso_and_bw();
  /**
  Greate locally maintained variables */
  void createVars();
  /**
  Update global variables and change current settings in camera if global variables has changed. */
  void updateVars();
  /**
  Test if there is a notisable difference in var-values and known camera settings */
  bool isVarUpdated();
  /**
  Set shutter value to camera. if value is -1 then camera is set to auto.
  Value should be within supported range.
  \returns true if set. */
  bool setShutterRaw(int value);
  /**
  Get shutter values from camera and set fShutter structure as well as global variable 'shutter'
  \returns true if data is fresh from camera. */
  bool getShutterRaw();
  /**
  Set gain value and assumes that noone else mangles with camera or global variables
  A negative value (-1) sets automatic gain.
  \Returns true if set successful - i.e. the function is supported by camera.*/
  bool setGainRaw(int agc);
  /**
  Gets raw information from camera and sets global variables.
  Assumes noone else fiddles with camera nor the gain global var
  \returns true if call to camera is successfull */
  bool getGainRaw();
  /**
  Set exposure target to this value - must be within allowed range - see global vars for device.
  \returns true if set. */
  bool setExposureTargetRaw(int value);
  /**
  Get exposure target value (used in auto shutter mode only) from camera
  \returns true on fresh data. */
  bool getExposureTargetRaw();
  /**
  Get white ballance settings from camera
  \returns true if fresh from camera */
  bool getWhiteBalanceRaw();
  /**
  Set white ballance to camera device using these values.
  \param mode is 3 for manual and 4 for auto (auto is one-shot only)
  \param red is manual red gain value - must be within range (see global vars for device)
  \param blue is manual blue gain value - must be within range (see global vars for device)
  \returns true if set. */
  bool setWhiteBalanceRaw(int mode, int red, int blue);
  /**
  Set external trigger on or off.
  \param value is value to set off=free running trigger based on available bandwidth.
  on is trigger using external pin (may be connected to trigger function
  \returns true if set */
  bool setExternalTriggerRaw(bool value);
  /**
  Get fresh trugger data from camera. Sets fTrigger structure and global variable for device.
  \returns true if fresh. */
  bool getExternalTriggerRaw();


protected:
  /**
  Node for this camera */
  dc1394_t * devPlatform;
  /**
  Handle for accessing the camera */
  dc1394camera_t * camHandle;
  /**
  format mode (should be format 7 (DC1394_VIDEO_MODE_FORMAT7_0)) */
  dc1394video_mode_t selected_mode;
  /**
  ROI settings */
  unsigned int roiWidth;
  unsigned int roiHeight;
  unsigned int roiTop;
  unsigned int roiLeft;
  /** Terminate the frame read thread */
  bool stopFrameRead;
  /** Flag that is set by the read thread on entry
  and reset by the thread on exit */
  bool threadRunning;
  /**
  Thread handle for frame read thread. */
  pthread_t thRead;
  /**
  Image pointer for single image capture */
  UImage * captureImage;
  /**
  Last time an image was received - for frame-rate calculation */
  UTime tLastFrame;
  /**
  Capture lock to be unlocked (posted) when
  a snapshot image is needed.
  Should be posted by the thread needing the image, and will be
  accepted (locked by a tryPost() or tryLock()) by the frame read thread.*/
  ULock captureDo;
  /**
  Capture finished post, posted by the read thread and the thread
  needing the image may wait on this lock. */
  ULock captureDone;
  /// feature info
  /// gain feature
  dc1394feature_info_t fGain;
  /// shutter
  dc1394feature_info_t fShutter;
  /// white ballance
  dc1394feature_info_t fWhite;
  /// exposure - target value for auto gain/shutter
  dc1394feature_info_t fExposure;
  /// external trigger settings
  dc1394feature_info_t fTrigger;
  /// external trigger delay
  dc1394feature_info_t fTriggerDelay;
  /**
  Flag indicating that settings are changed and should be implemented now */
  //bool settingsChanged;
  /**
  Globally available variables
  Image number */
  UVariable * varImageNumber;
  // for framerate calculation
  int lastImageNumber;
  /// external trigger
  UVariable * varExternalTrig;
  /// gain setting - [value, min, max, auto]
  UVariable * varGain;
  /// shutter setting [value, min, max, auto]
  UVariable * varShutter;
  /// exposure [value, min, max]
  UVariable * varExposure;
  /// white balance [auto, min, max, red, blue]
  UVariable * varWhite;
  /// is camera open
  UVariable * varOpen;
  /// framerate (calculated)
  UVariable * varFramerate;
  /// desired ISO BW
  UVariable * varISObw;
  /// desired packet size (each timeslot - see code for details
  /// or http://damien.douxchamps.net/ieee1394/libdc1394/faq/#How_do_I_set_the_frame_rate)
  UVariable * varPacketSize;
  /// implemented isoBW;
  int isoBW;
  /// should settings be updated
  UVariable * varSettingsChanged;
#endif
};

#endif
