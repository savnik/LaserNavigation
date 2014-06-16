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
#ifndef UCOMCAM_H
#define UCOMCAM_H

#include <ugen4/u3d.h>
#include <urob4/userverqueue.h>

#include "ucammount.h"

// name length for video device channel name - e.g. "television" or "video"
#define MAX_CHANNEL_NAME_LENGTH 32

/**
Structure to hold camera settings information and communicate this on a binary packed serial device.

@author Christian Andersen
*/
class UComCam
{
public:
  /**
  Constructor
  clears all vaid flags. */
  UComCam();
  /**
  Destructor */
  virtual ~UComCam();
  /**
  Pack camera settings to string.
  Packs all valid values into the string and
  returns the number of bytes used.
  The format do not set byte at buff[2], as
  this is reserved to message serial number
  and must be set by communication part.
  if toCamServer the type is MSG_CAM_SET_CAMERA_STATE
  else it is information from camera server
  and the message type is set to MSG_CAM_CAMERA_STATE. */
/*  unsigned int pack(unsigned char * buff,
                      unsigned int buffLng,
                      bool toCamServer,
                      unsigned int msgSerial);*/
  /**
  Unpack the information from the buffer
  if the valid flags in the message for
  the individual data subjects are not valid,
  then the related valid flag is not changed.
  That is these may be left valid with value
  from before the call.
  If however it is importent to find the changes, then
  clear all the Valid flags first.
  Returns true if a buffer is provided and
  the camera name has a valid length (if valid) */
  bool unpack(unsigned char * buff);
  /**
  Peek the device number without unpacking the whole message.
  Returns -1 if device number is invalid, else
  a device number in range 0 to 31 is returned. */
  //static int peekDevice(unsigned char * buff);
  /**
  Clear all the valid flags. */
  virtual void clear();
  /**
  Set values from camera.
  if probe is true, then some values
  are requested from camera. */
  bool setFromCam(UCamDevBase * dev, bool probe,
                         bool clientInCtrl);
  /**
  Set camera from valid values.
  Camera is assumed to be the right device and
  in-control is assumed to be checked before the call.
  If the settings are to be maintained the
  'openValid' and openValue should both be true.
  Returns true if data is assumed implemented */
  bool setCamDevice(UCamDevBase * dev);
  /**
  Print all states to console */
  void print(const char * prestring);
  /**
  Get white ballance mode as string */
  const char * getWhiteModeStr(int mode);
  /**
  Returns saturation mode as string */
  const char * getSaturationModeStr(int mode);
public:
  /**
  Camera device open/closed is valis or not */
  bool openValid;
  /** Camera device is open/closed */
  bool openValue;
  /**
  Video device number - usually means directly the '/dev/video0'
  device number series.
  - is info valid. */
  bool deviceValid;
  /**
  Video device number - usually means directly the '/dev/video0'
  device number series (0..32). */
  int deviceValue;
  /**
  Is client in control of camera - or take control
  - is info valid */
  bool inCtrlValid;
  /** Is client in control of camera - or take control */
  bool inCtrlValue;
  /**
  Frame size from camera - is info valid */
  bool sizeValid;
  /** Frame size from camera - width in pixels */
  unsigned int sizeWidth;
  /** Frame size from camera - height in pixels */
  unsigned int sizeHeight;
  /**
  Framerate in images per second
  - is info valid */
  bool fpsValid;
  /** Framerate in images per second from camera. */
  int fpsValue;
  /**
  Image number setting (serial number for used images)
  is info valid. */
  bool imageNumberValid;
  /**
  Image number setting (serial number for used images)
  A long positive number increased for every
  used image frame, and placed in image meta data. */
  unsigned long imageNumber;
  /**
  Camera position relative to robot center
  - is info valid. */
  //bool posValid;
  /**
  Camera position relative to robot center. in meter
  x is left, y is up and z is behind - robot center. */
  //UPosition posValue;
  /**
  Camera position relative to robot center
  - is info valid. */
  //bool rotValid;
  /**
  Camera rotation relative to robot. in radians
  Omagea is rotation about x,
  Phi is rotation about y and Kappa is rotation about z. */
  //UPosition rotValud;
  /**
  Camera gain control-
  is value valid. */
  bool gainValid;
  /**
  Camera gain value:
  Manual gain values are in range from  to 0xFFFF.
  Only some (typically 6 bit) of the MSB bits are
  actually implemented in camera. */
  unsigned int gainValue;
  /** Camera gain is set to automatic */
  bool gainAutomatic;
  /**
  Shutter control - is value valid. */
  bool shutterValid;
  /**
  Shutter control value in range 0 to 0xFFFF, where the
  most significant (typically 5 bits) are implemented only. */
  unsigned int shutterValue;
  /**
  Is shutter set to automatic. NB! if automatic, then no values are
  available for actual setting. */
  bool shutterAutomatic;
  /**
  Video gain contour control - is info valid */
  bool contourValid;
  /**
  Video gain contour - lead filter.
  Enhances edges in horizontal lines. Value is from 0 to 0xFFFF, but
  only some of the MSBs are implemented.
  The value 0 means no enhancement. */
  unsigned int contourValue;
  /**
  Camera LED (on/off control) is info valid. */
  bool ledValid;
  /**
  Camera led value - true if on. */
  bool ledOn;
  /**
  Compression level used in image transfer. Is info valid. */
  bool compressionValid;
  /**
  Compression is needed in some image sizes and is optional
  in others. This value is the preferred cmpression if
  bandwidth allows. */
  unsigned int compressionValue;
  /**
  Gamma correction - is info valid. */
  bool videoValid;
  /**
  Video Gamma correction value can change the the number of intensity
  bits used in the dark part of the image. A preliminary analysis
  suggests that a value of 0 means linear relation from intensity
  to coding in the 8-bit image. a higher value (up to 0xFFFF) makes
  the intensity coding non-linear compressing the bright parts and
  expanding the darker part. A value of 0x8000 is the default
  setting (TV-style gamma correction */
  unsigned int gammaValue;
  /**
  Video brightness adjustment in camera.
  Normal value is 0x8000.
  Range is [0 .. 0xFFF0] a returned value of 0xFFFF probably
  meand 'unsupported feature'. */
  unsigned int brightnessValue;
  /**
  Video contrast adjustment in camera.
  Normal value is 0x8000.
  Range is [0 .. 0xFFF0] a returned value of 0xFFFF probably
  meand 'unsupported feature'. */
  unsigned int contrastValue;
  /**
  Video colour sturation adjustment in camera.
  Normal value is 0x8000.
  Range is [0 .. 0xFFF0] a returned value of 0xFFFF probably
  meand 'unsupported feature'. */
  unsigned int colourValue;
  /**
  White ballance - is info valid. */
  bool whiteBalValid;
  /**
  White ballance can be set to automatic, a few stanrd settings and
  a manual value, the following values may be used:.
  PWC_WB_AUTO: Automatic
  PWC_WB_MANUAL: Manual
  PWC_WB_INDOOR: Indoor setting using high temperature wolfram light
  PWC_WB_OUTDOOR: An outdoor setting.
  PWC_WB_FL: Fluorcent lightning
  The values are defined in <pwc-ioctl.h>
  (in /usr/src/linux/drivers/usb(/media)   */
  unsigned int whiteBalMode;
  /**
  The white ballance gain can in some modes be moditored and can
  be set in manual mode. The values are gain in the red and blue channel
  (compared to the green).
  The range is 0 to 0x7FFF - but only a few of the MSBs are impelmented.
  A value of 0x4000 is assumed to be neutral (gain == 1.0), a higher
  value increases the gain. */
  unsigned int whiteBalGainRed, whiteBalGainBlue;
  /**
  Video/tuner data channel, may be any number,
  but most likely not supported (webcams) or 4 channels (TV-card)
  TV-card: 0 = TV, 2 = video, 1,3=unknown. */
  int channelNumber;
  /**
  Channel name */
  char channelName[MAX_CHANNEL_NAME_LENGTH + 1];
  /**
  Is data channel numnber vaid */
  bool channelNumberValid;
  /**
  Camera name valid */
  bool nameValid;
  /**
  Camera name */
  char name[UCamDevBase::MAX_CAM_DEV_NAME_LENGTH + 1];
  /**
  High dynamic range mode - e.g. takes 3 images and combine as a set.
  is hdrSetting valid. */
  bool hdrValid;
  /**
  High Dynamic range mode is number of images with increasing
  shutter and gain settings, plus one with closed shutter to get
  a black level image.
  p.t. only 3 images are supported, i.e. ine image with current
  gain and shutter setting, one with increased values and one with
  closed shutter and nominal gain (dark level).
  NB! the mode requires that gain and shutter is set to manual or
  is in no-saturation mode.
  0 means hdr is OFF and 3 that it is on. Other values are
  equal to off. */
  unsigned int hdrMode;
  /**
  No saturation mode is a computer controlled adaptive gain
  where every 3rd image are analyzed for intensity and
  the brightest pixel is attempted to be at a desired
  intensity level (Y = 220). - is setting valid. */
  bool saturationCtrlValid;
  /**
  Saturation mode setting.
  Value 0 is no saturation control.
  Value > 0 is one of the modes selected by UCamDevice. */
  unsigned int saturationCtrlMode;
  /**
  Is images to be captured at regular intervals, then these values
  should be valid */
  bool streamValid;
  /**
  Do capture at regular intervals. */
  bool streamOn;
  /**
  Capture with this framerate (if possible).
  Positive values are frames per second,
  negative values are seconds per frame.
  (so -1 and 1 is the same ( 1 image per second). */
  int streamFps;
  /**
  Command at every new image - valid? */
  bool cmdValid;
  /**
  Is command to be executed at every new image? */
  bool cmdUse;
  /**
  Command to be executed */
  //char cmdCmd[MAX_CMD_LENGTH];
};


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
Extension to camera parameters class, packing and unpacking
to and from the XML-like format SML. */
class UComCamSml : public UComCam
{
public:
  /**
  Pack the parameters requested from this camera to the provided buffer
  zero terminate the buffer.
  Returns false if not enough space, or camera device is invalid. */
  bool pack(UCamMounted * cam, int client, char * message, int messageSize);
  /**
  Unpach the parameter part of this message and set values
  and flacks accordingly.
  Returns true if ask for help only */
  bool unpack(UServerInMsg * msg);
  /**
  Set values from actual camera */
  bool setFromCam(UCamMounted * cam, bool probe, int client);
  /**
  Set camera device from stored values */
  bool setCamDevice(UCamMounted * cam);
  /**
  Clear all values to not valid. */
  virtual void clear();

public:
  /**
  Name of the mounted device, e.g. "left" or "cameraLeft",
  as specified in the config file. */
  char posName[MAX_MOUNT_NAME_SIZE];
  /**
  is device mount name valid */
  bool posNameValid;
  /**
  Position of camera on robot values are valid */
  bool relPosValid;
  /** new position data */
  bool relPosXValid;
  /** new position data */
  bool relPosYValid;
  /** new position data */
  bool relPosZValid;
  /**
  Position relative to robot */
  UPosition relPos;
  /**
  Rotation of camera on robot values are valid */
  bool relRotValid;
  /** new rotation (orientation) data */
  bool relRotOValid;
  /** new rotation (orientation) data */
  bool relRotPValid;
  /** new rotation (orientation) data */
  bool relRotKValid;
  /**
  Rotation of camera relative to robot */
  URotation relRot;
  /**
  Camera parameters valid */
  bool camParValid;
  /**
  Camera parameter focal length */
  float camParFocalLength;
  /**
  Radial error parameter K1 */
  float camParRadialK1;
  /**
  Radial error parameter K2 */
  float camParRadialK2;
  /**
  Headpoint x */
  float camParHeadX;
  /**
  Headpoint y */
  float camParHeadY;
  /**
  pan-tilt relative position command */
  bool panRelValid;
  /**
  pan tilt relative value */
  int panRelValue;
  /**
  pan-tilt relative position command */
  bool tiltRelValid;
  /**
  pan tilt relative value */
  int tiltRelValue;
  /**
  pan-tilt position command / status */
  bool panPosValid;
  /**
  pan tilt relative value */
  int panPosValue;
  /**
  pan-tilt position command / status */
  bool tiltPosValid;
  /**
  pan tilt relative value */
  int tiltPosValue;
  /**
  pan-tilt status request or status */
  bool panTiltValid;
  /**
  pan-tilt min pan value */
  int panMinValue;
  /**
  pan-tilt max pan value */
  int panMaxValue;
  /**
  pan-tilt min tilt value */
  int tiltMinValue;
  /**
  pan-tilt max tilt value */
  int tiltMaxValue;
  /**
  pan-tilt support */
  bool panTiltSupportValue;
  /**
  Reset pan-tilt function */
  bool panTiltHome;
  /**
  Not used parameters, when unpacking a message.
  The list can be used in a reply to requested */
  char notUsedPars[MAX_MESSAGE_LENGTH_TO_CAM];
};

#endif
