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
#ifndef UCAMDEVIEEE1394_H
#define UCAMDEVIEEE1394_H

#ifdef USE_IEEE1394
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#endif

#include "ucamdevbase.h"
#define FIRST_IEEE1394_DEVICE_NUMBER 10

/**
Camera device type that controls one IEEE1394 DCI camera
Device number is from 10 to 19 - defined in URob4/UCamPool

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UCamDevIeee1394 : public UCamDevBase
{
public:
  /**
  Constructor */
  UCamDevIeee1394();
  /**
  Constructor */
  ~UCamDevIeee1394();
#ifdef USE_IEEE1394
  /**
  Set image push image buffer - if one buffer is needed only.
  \param img is the image buffer - typically from image pool. */
  virtual void setPushBuffer(UImage * imageBuffer);

  /**
  Get number of available IEEE1394 devices */
  int getIeee1394PortCnt();
  /**
  Set camera node info */
  bool setCamDeviceNode(int portNum, int camNum);
  /**
  Get number of cameras */
  int getIeee1394CamCnt(const int port);
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
  A positive value in range 0..0xFFFF is video gain in
  camera. 0 is low gain (but not zero).
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

protected:
  /**
  Get the bayer pattern from the camera.
  Result is one of the PIX_PLANEX_BGGR integers. */
  int getBayerPattern();
  /**
  Get one of the feature sets stored in 'features', holding
  requested data. */
  dc1394_feature_info * getFeature(unsigned int featureId);
  /**
  Convert a frames per second (integer) value to the possible
  framerate constants defined in mode 5 ieee1394 settings, i.e.
  FRAMERATE_1_875 = 32 for 1.875 frames / sec. */
  int fps2mode5framerate(int fps);
  /**
  Convert the bit-position framerate to interger framerate in frames per second (rounded to closest integer) */
  int framerate2fps(int framerate);

protected:
  /**
  Node for this camera */
  dc1394_cameracapture camera;
  /**
  Handle for accessing the camera */
  raw1394handle_t camHandle;
  /**
  Camera info */
  dc1394_camerainfo info;
  /**
  camera features */
  dc1394_feature_set features;
  /**
  ROI settings */
  unsigned int roiWidth;
  unsigned int roiHeight;
  unsigned int roiTop;
  unsigned int roiLeft;
  /**
  Flag for disposing DMA buffers */
  bool usingDMA;
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
  Capture lock to be unlocked (posted) when
  a snapshot image is needed.
  Should be posted by the thread needing the image, and will be
  accepted (locked by a tryPost() or tryLock()) by the frame read thread.*/
  ULock captureDo;
  /**
  Capture finished post, posted by the read thread and the thread
  needing the image may wait on this lock. */
  ULock captureDone;
#endif
};

#endif
