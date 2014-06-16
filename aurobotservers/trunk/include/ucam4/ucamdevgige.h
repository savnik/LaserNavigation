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
#ifndef UCAMDEV_GIGE_H
#define UCAMDEV_GIGE_H

#define _LINUX
#define _x86

#include <gige/PvApi.h>
#include <gige/ImageLib.h>

#include "ucamdevbase.h"

#define FRAMESCOUNT 15
#define MAX_GIGE_DEVS 4
#define FIRST_GIGE_DEVICE_NUMBER 14

/// GigE camera data structures
struct tCamera
{

    unsigned long   UID;
#if defined USE_GIGE
    tPvHandle       Handle;
    tPvFrame        Frames[FRAMESCOUNT];
//    tPvUint32       Counter;
#endif
    char            Filename[20];
};

/**
Camera device type that controls GigE camera
Device number is from 15 to 19 - defined in URob4/UCamPool

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UCamDevGigE : public UCamDevBase
{
public:
  /**
  Constructor */
  UCamDevGigE();
  /**
  Constructor */
  ~UCamDevGigE();

  /**
  Set camera node info */
  bool setCamDeviceNode(int portNum, int camNum);
  /**
  Get basic camera info */
  void getCamInfo();
  /**
  Get full camera feature set */
  bool getCamFeatures();
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
  If camera is available initialize to a state where it can be opened right away.
  \Returns true if device can be opened (a device exists). */
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
    Get white balance values
    if 'probe' the the camera is asked, otherwise previous value
    is returned unchecked.
    If one or more of the following pointers are non-zero
    the integer value is returned.
   */
  virtual bool getWhiteBalance(bool probe, int * red = NULL, int * blue = NULL, int * mode = NULL);
  /**
  Call back function when frame is finished */
  void  frameDoneCB(tPvFrame* pFrame);
  /**
  callback called when the camera is plugged/unplugged */
  void  cameraEventCB(void* Context,
                             tPvInterface Interface,
                             tPvLinkEvent Event,
                             unsigned long UniqueId);
  /**
  Copy this frame to the destination image.
  \param destination is the image-pool image for the frame.
  \param frame is the frame from the camera.
  \returns true if image is copied to destination,and false if source or destination is invalid. */
  bool frameToImage(UImage * destination, tPvFrame * frame);

protected:
  /**
  Get the bayer pattern from the camera.
  Result is one of the PIX_PLANEX_BGGR integers. */
  int getBayerPattern();
  /**
  Get index of gige cameras */
  int getGigEIdx()
  {
    int n;
    n = getDeviceNumber() - FIRST_GIGE_DEVICE_NUMBER;
    n = mini(MAX_GIGE_DEVS - 1, maxi(0, n));
    return n;
  };
  /** setup and start streaming */
  bool CameraStart(tCamera* Camera);
  /**
  reset timestamp - and time reference */
  void resetTimestamp();
  
protected:

  /// GigE camera info
  tCamera Camera;
  /**
  ROI settings */
  unsigned int roiWidth;
  unsigned int roiHeight;
  unsigned int roiTop;
  unsigned int roiLeft;
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
  /**
  White balance mode */
  bool whitebalAuto;
  /**
  White balance red */
  int whitebalRed;
  /**
  White ballance blue */
  int whitebalBlue;
  /**
  Timestamp frequency */
  long unsigned int timestampFrequency;
  /**
  Time stamp reference */
  UTime timestampRefTime;
  /**
  Format of last received frame */
  int frameFormat;
};

#endif
