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

#ifndef UCAMDEVICE_H
#define UCAMDEVICE_H

#include <videodev.h>

#include <pwc-ioctl.h>

#include <ugen4/uimage2.h>

// debug flag -- prints a bit more when opening the camera
//#define CAM_VERBOSE
/**
Max image size, used for size reference in
pixel size calculation, and should not be changed from
640x480 - also used for by radial error calculation matrix. */
#define MAX_IMAGE_WIDTH 640
#define MAX_IMAGE_HEIGHT 480
/**
Size of buffer for raw images from camera, that is timestamped and ready to use */
#define RAW_IMAGEBUFFER_SIZE 5
/** name length from camera itself. */
#define MAX_CAMERA_NAME_LENGTH 32
/** default camera frame rate
    valid values are 5, 10, 15, 30 */
#define CAMERA_FRAME_RATE 5
/**
Number of frames in a Hgh Dynamic Range set
of images + 1.
Set number 0 is actual setting in camera.
Setting 1 ... is for frames with increasing intensity */
#define MAX_HDR_FRAMES 4


/**Class related to open, close, setting parameters and reading timestamped images from (Philips style) camera devices.
  *@author Christian Andersen
  */
class UCamDevice
{
public:
  /**
  Constructor */
  UCamDevice();
  /**
  Destructor */
  virtual ~UCamDevice();
  /**
  Set video device number.
  returns true if a valid number (0..20). Does not test camera
  to see if camera can be opened. (use getCameraName for that).
  If new device and camera is open already, then it is closed.
  if new device then camera info is invalidated.
  The new device number will be used next time
  the camera is opened. */
  bool setDeviceNumber(int dev);
  /**
  Get camera file descriptor (should only be used
  for testing, as object open and close this file descrioptor
  at its own descression */
  inline int getCamFd()
  { return cam_fd; };
  /**
  Get image width in pixels from last captured image. */
  inline unsigned int getWidth() {return frameWidth;};
  /**
  Get camera pixel height from last captured image */
  inline unsigned int getHeight() {return frameHeight;};
  /**
  Get current device number. */
  inline int  getDeviceNumber() {return devNum;};
  /**
  Get image serial number */
  inline unsigned long getImageNumber() { return imageNumber;};
  /**
  Set image serial number */
  inline void setImageNumber(unsigned long serial) { imageNumber = serial;};
  /**
  Get current frame rate setting */
  inline int getFrameRate() {return frameRate;};
  /**
  Is camera open in streaming mode. */
  inline bool isCameraOpen() {return cameraOpen;};
  /**
  If camera has been open and device number is unchanged,
  then the camera information is regarded valid. */
  inline bool isInfoValid() {return devInfoValid;};
  /**
  Is the vodeo device valid - is it possible to open.
  Returns true if a valid device */
  inline bool isValidDevice()
    { return (cam_fd >= 0); };
  /**
  Get camera name. If camera is not open, then camera is opened
  with default parameters and name (and other settings) are
  obtained.
  Closes camera after value is obtained.
  Returns NULL if no camera is available. */
  char * getCameraName();
  /**
  Set camera name. */
  inline void setCamName(const char * newName)
  { // space for 32 characters in camera name
    strncpy(vcap.name, newName, 32);
  };
  /**
  Read frames off the camera device driver queue, and timestamp the images.
  Should not be called directly, are called implicitly,
  when camera is opened.*/
  void readFramesThread(void);
  /**
  Open and close devise to get default camera parameters.
  Returns 0 iff sucessfull. */
  bool openDeviceDefault(bool readThreadNormal = true);
  /**
  Open device and set size and framerate as set in frameHeight,
  frameWidth and frameRate variables.
  If 'readThreadNormal' = false, no read thread is started.
  and image capturing and timestamp is disabled.
  (captureImage will fail) */
  bool openDevice(bool readThreadNormal = true);
  /**
  implement these
  settings and leave the device open. */
  bool openAndSetDevice(const int width, const int height,
                                const int frameRate,
                                bool readThreadNormal = true);
  /**
  Set default size and framerate if camera is closed.
  If camera is open, then change size and framerate as specified.
  Returns true if size is saved/implemented, if camera is
  closed no range check check is performed. */
  bool setSizeAndFramerate(const int width, const int height,
                        const int framerate);
  /**
  Global function to close the connection to the
  video device.
  Returns true if successful. */
  bool setDeviceClosed();
  /**
  Set image size and speed as last requested, and leave camera open
  Returns 0 if no error. */
  //bool setToRequestedSizeandSpeed();
  /**
  Get a new image to this URawImage object.
  Current camera settings are used for size etc.
  'image' must be able to hold image size.
  Camera is opended if not already, and is left open
  at default streaming speed.
  Returns true if new image is available in 'image'. */
  //bool getImageSnapshot(URawImage * image);
  /**
  Manual control with camera intensity gain and shutter speed.
  NB! This routine does not work - it just sets shutter and AGC
  to automatic. Just kept for future enhancements.
  Returns true if no error occured. */
  bool setShutter(int shut);
  /**
  Get actual shutter value.
  if 'probe' is false, then existing vaue is returned.
  if 'probe' is true, then the camera is asked
  for the most recent value. if this is
  not possible, then en error is reported and last
  value returned.
  This value can not be read from camera, so
  last commanded value is returned.
  if not available then *dataValid is set to false. */
  int getShutter(bool probe = false);
  /**
  Set noise reduction value in camera.
  A value of 0 is no filter and 3 is maximum filter.
  Returns true if set successful - i.e. camera could be locked and
  the function is supported by camera.*/
  bool setNoiseRed(int noiseReducion);
  /**
  Get actual noise reduction value.
  if 'probe' is false, then existing vaue is returned.
  if 'probe' is true, then the camera is asked
  for the most recent value. if this is
  not possible, then en error is reported and last
  value returned. */
  int getNoiseRed(bool probe);
  /**
  Set gain value.
  A negative value (-1) sets automatic gain.
  A positive value in range 0..0xFFFF is video gain in
  camera. 0 is low gain (but not zero).
  Returns true if set successful - i.e. camera could be locked and
  the function is supported by camera.*/
  bool setGain(int agc);
  /**
  Get actual gain value.
  if 'probe' is false, then existing vaue is returned.
  if 'probe' is true, then the camera is asked
  for the most recent value. if this is
  not possible, then en error is reported and last
  value returned.
  If dataValid is set, then this is set false on error */
  int getGain(bool probe, bool * dataValid = NULL);
  /**
  Set white balance correction values.
  The mode can be one of the following values:
  PWC_WB_AUTO,
  PWC_WB_MANUAL,
  PWC_WB_INDOOR,
  PWC_WB_OUTDOOR,
  PWC_WB_FL (flurocent).
  Red and blue gain is used in MANUAL mode only
  and in a range from 0 to 0x7FFF, where 0x4000 are
  assumed to be neutral (equal to gain in green channel)
  Returns true if successful.  */
  bool setWhiteBalance(int mode, int red, int blue);
  /**
  Get white balance values
  if 'probe' the the camera is asked, otherwise previous value
  is returned unchecked.
  If one or more of the following pointers are non-zero
  the integer value is returned.
  */
  bool getWhiteBalance(bool probe, int * red = NULL, int * blue = NULL, int * mode = NULL);
  /**
  Convert a WB mode to a descriptive string.
  'strFuff most be a buffer of at least 10 bytes.
  Returns a pointer to the buffer, where the mode-string is filled. */
  static char * getWBModeAsString(int mode, char * strBuff);
  /**
  Convert white ballance mode to integer. The modes
  are 'auto' indoor' 'outdoor' 'flurocent' and 'manual'.
  Returns the mode, and auto, if mode is not recognized. */
  int getWBModeFromString(const char * mode);
  /**
  Set led speed
  time is in miliseconds.
  (on,off):
  (0,0) = OFF,
  (100,0) = ON,
  (200,800) = 1 sec blink with 20% on.
  Returns true if set.   */
  bool setLed(int onTime, int offTime);
  /**
  Get setting or last setting */
  bool getLed(bool probe, int * onTime, int * offTime);
  /**
  Set led on */
  inline void setLedOn(bool toOn)
  {
    if (toOn)
      setLed(100,0);
    else
      setLed(0,0);
  };
  /**
  Is led on - sometime */
  bool isLedOn(bool probe);
  /**
  Set led blink proportional to framerate.
  That is on in 200 ms and off in 5/frameRate seconds. */
  inline void setLedFrameRate() { setLed(200,5000/frameRate);};
  /**
  Set the contour function in camera.
  This is an edge sharpener to compensate for lack
  of bandwidth (horizontal lines only.
  The range is 0 to 0xFFFF.
  The value 0x0 is no contour correction.
  If set negative (e.g. -1) the camera selects an appropriate
  value.
  Returns true if command issued. */
  bool setContour(int value = 0x1000);
  /**
  Get contour value.
  if 'probe' then a value from camera requested is loaded.
  if this fails, an error is issued.
  The function returnes the retreived value (if requested and available)
  otherwise it returns the last fetched value.
  NB! if contour is set to automatic the returned value
  is not related to the implemented value. */
  int getContour(bool probe);
  /**
  Set compression preference. This tells the camera the preferred compression
  ratio. If framerate and image size is too high for the selected
  compression ratio, then a higher compression is used.
  0 = no compression
  1 = small
  2 = more
  3 = highest compression.
  resolution 640x480 always uses some compression (USB1.1)
  Compression is visible as blocks of 4x4 pixels get a bit too similar.
  Returns true if setting is send to camera. */
  bool setCompPref(int value);
  /**
  Get current setting of compression preference.
  0 = no compression
  3 = highest compression.
  If 'probe' = true the a fresh value is requested from
  camera, otherwise tha last obtained value is returned.
  If a requested value is unavailable, then en error is issued. */
  int getCompPref(bool probe);
  /**
  Set gamma correction value.
  The default setting is 0x8000 (32768),
  Value 0 is assumed to be linear. So gamma > 1.0 is possible only. */
  bool setGamma(int gamma);
  /**
  Get the current gamma correction value.
  The value is an integer in 16 bit range, where 0x8000
  is default. */
  int getGamma(bool probe);
  /**
  Set contrast in camera - normal value is 0x8000 and
  range is 0..0xFFF0 */
  bool setContrast(int contrast);
  /**
  Set brightness in camera - normal value is 0x8000 and
  range is 0..0xFFF0 */
  bool setBrightness(int brightness);
  /**
  Set colour saturation in camera - normal value is 0x8000 and
  range is 0..0xFFF0 */
  bool setColour(int colour);
  /**
  Get contrast in camera - a value of 0xFFFF probably
  means 'not supported'.
  This value is probed together with brightness, colour and gamma,
  so probe for one of these only. */
  int getContrast(bool probe);
  /**
  Get brightness in camera - a value of 0xFFFF probably
  means 'not supported'.
  These values are probed together: Contrast, brightness, colour and gamma,
  so probe for one of them only. */
  int getBrightness(bool probe);
  /**
  Get colour saturation in camera - a value of 0xFFFF probably
  means 'not supported'.
  These values are probed together: Contrast, brightness, colour and gamma,
  so probe for one of them only. */
  int getColour(bool probe);
  /**
  Set all 4 video control values in one call - normal value is 0x8000 and
  range is 0..0xFFF0. */
  bool setVideoCap(int brightness, int contrast, int gamma, int colour);
  /**
  Get the framemode for High Dynamic range image capture.
  Mode = 0 is off, i.e. normal camera mode.
  Mode = 3 is where an image set of 3 images are taken, one
  with normal gain and shutter setting, one with longer exposure time
  (and possibly gain) and one with closed shutter (dark reference).
  Other modes are not supported.
  Mode 3 requires that image shutter and gain is set to manual or
  to 'no-Sturation' control mode */
  inline int getHdrMode() { return hdrMode;};
  /**
  Set High Dynamic range mode.
  See getHdrMode() for explanation. */
  void setHdrMode(int mode);
  /**
  Is saturation prohibit mode set to automatic. */
  inline unsigned int getStaurationCtrlMode()
       { return saturationControl[0];};
  /**
  Set saturation prohibit mode set to automatic. */
  inline void setSaturationCtrlMode(unsigned int mode)
       { saturationControl[0] = mode;};
  /**
  Get a set of HDR images converted to
  UImage format.
  First image (image1) is with closed shutter - black reference.
  Second image (image2) is with no (minor) saturation (dark).
  Third image (image3) is for shaddow analysis (bright) */
  //bool getHdrImageSnapshot(UImage * image[], int imCnt);
  /**
  Wait a frame is a finction, that calls Wait for
  the time it takes to get a frame. If a parameter is provided
  it waits a fraction of a frametime. */
  void waitAFrame(float frames = 1.0);
  /**
  Called when a new image is available.
  NB! this cunction will be called by the read-image thread,
  so do not process much here.
  The image is not locked, and may be made invalid by the
  another thread. */
//  virtual void gotNewImage(URawImage * raw);
  virtual void gotNewImage(UImage * rgb);
  /**
  Set data source channel on a source device, where it is supported.
  Returns true if successful.
  On successful return the name of the channel is available by
  getDataChannelName(). */
  bool setDataChannel(int channel);
  /**
  Get current data channel setting.
  Returns -1 if channel setting is not supported. */
  int getDataChannel();
  /**
  Get current data channel name - e.g. Television or s-video on a
  TV card.
  Channel is set using setDataChannel(). */
  inline char * getDataChannelName()
  { return vch.name; };

  /**
  Get pixel size relative to 640x480. */
  inline double getPixelSize()
  {return MAX_IMAGE_WIDTH / double(frameWidth);};
  /**
  Test if souch device exist and can be opened.
  Returns true if device can be opened (a device exists). */
  bool deviceExist();

protected:
  /**
  Start read hread.
  Should be done, when images are neded only */
  bool startReadThread();
  /**
  Stop read thread - should be done on exit only */
  void stopReadThread(bool andWait);
  /**
  Open device and get default settings. */
  bool protOpenDevice(bool vcapOnly = false);
  /**
  Assume that the camera is locked and open and implement
  these device settings.
  Returns true if camera can be controlled and is open.
  New camera settings are in frameHeight, frameWidth and frameRate.
  valid is true if camera can be controled.  */
  bool protSetDeviceSizeAndSpeed(unsigned int toWidth,
                            unsigned int toHeight,
                            int toFramesPerSec);
  /**
  Open device and set resolution to 'width' and 'height'.
  If with and height is both 0, then default resolution is
  maintained. <br>
  NB! Bandwith may limits the number
      of simultanious open devices in high resolution <br>
  Returns true if all is OK and as requested, in this case
  all parameters of the camera device is filled. e.g. vwin with
  height and width.
  Returns false if any error occured or the image could not be
  taken with the required resolution. */
  bool protOpenDevice(const unsigned int height,
                 const unsigned int width,
                 const int framesPerSec,
                 bool readThreadNormal);
  /**
  Close device streem */
  bool protCloseDevice();
  /**
  Get a buffer with a locked image from the queue the read thread
  has processed.
  The image must be unlocked by a call to unlock() after
  use, and should not be kept locked, as this will reduce the
  number of available buffers for the read thread.
  Returns NULL if no images were available within a reasonable time. */
  bool protGetLockedNewImage(/*URawImage ** praw,*/ UImage ** prgb);
  /**
  With camera locked, get and lock images
  in an hdr image set.
  Will attempt to return images where the 'get' parameter is true.
  Returns true if requested images are found and locked OK. */
  //bool protGetLockedNewImage(URawImage * rawres[], bool get[], int imCnt);
  /**
  Opens camera if closed, and waits until first image
  is ready. To be used on request for new image. */
  //bool protTryOpenBeforeNewImage();
  /**
  Semaphore for locking, when reading or writing to camera device */
  void lockInitCam();
  /**
  Semaphore for locking, when reading or writing to camera device */
  bool lockCam();
  /**
  Semaphore for locking, when reading or writing to camera device */
  bool tryLockCam();
  /**
  Semaphore for locking, when reading or writing to camera device */
  void unlockCam();
  //
  /**
  Set the parameters derived from resolution, i.e.
  conversion matrix, radial error valeues etc.
  but at this level just the resolution factor. */
  virtual void imageSizeChanged(double iResFactor);
  /**
  Device is changed, there may be a need to load
  new parameters for camera constants etc. */
  virtual void imageDeviceChanged(int newDevice);
  /**
  Error messages that may be generated from class */
  virtual void info(const char * s, const int type, const int num)
  { // should probably be overwritten by decendent classes
    printf("(%d,%d): %s\n", type, num, s);
  };
  /**
  Adjust gain and shutter setting from image to avoid saturation.
  Mode is the way to do it */
//  void doSaturationControl(URawImage * rawImg,
//              unsigned int * modes, int * eVal);
  /**
  Common function to report return errors for
  ioctl functions - pricate Philips calls only */
  void ioctlError(int err, const char * prestring);
  /**
  Get basic device information.
  Returns true if all is good */
  bool getDeviceInfo();

protected:
  /**
  Is device info valid */
  bool devInfoValid;
  //
  // device related properties
  /**
  Device number - translates to /dev/video0, -1, -2 ... */
  int devNum;
  /**
  Device, when camera were last sucessfully opened.
  is initiated to -1. */
  int devNumLast;
  /**
  Camera is open - straming images */
  bool cameraOpen;
  /**
  Flag for simulated device - should be used to avoid unneeded errors */
  bool simulated;
  /**
  Buffer, where images get read to for timestamping */
//  URawImage * raw[RAW_IMAGEBUFFER_SIZE];
  /**
  Buffer, where images get read to for timestamping */
  UImage * rgb[RAW_IMAGEBUFFER_SIZE];
  /**
  Size of images read from device (expected size) */
  unsigned int frameHeight;
  /**
  Size of images read from device (expected size) */
  unsigned int frameWidth;
//  unsigned int reqHeight; // requested image height
//  unsigned int reqWidth;  // requested image width
//  int reqFrameRate;       // requested frame rate
  /**
  Framerate from camera */
  int frameRate;
  // resFactor (pixSize) is sed when new image is captured (by newImage())
  // double pixSize; // 1=640x480, 2=320x240, 4=160x120
  //
  /**
  Serial number for next used image.
  images not used are not counted - see imCnt. */
  unsigned long imageNumber;
  /** frame read thread variables */
  int nextFrame;
  /** Terminate the frame read thread */
  bool stopFrameRead;
  /** Flag that is set by the read thread on entry
      and reset by the thread on exit */
  bool threadRunning;
  /**
  Thread handle for frame read thread. */
  pthread_t thRead;
  /**
  Images read by thread since last size change */
  unsigned int imCnt;
  //
  /** lock, when talking to camera driver */
  pthread_mutex_t mLock; // pthread_mutex_lock
  // camera info
  int cam_fd; // file descriptor
  //bool isPhilips;
  /**
  Noise reduction in camera, this can be set to
  a value from 0 to 3, with 0 being no reduction. */
  int vnoiseReduc;
  /**
  Compression preference range [0-3].
  0= no compression, ... 3=higest compression. */
  int vcompressionPref;
  /**
  Contour value to sharpen edges in image */
  int vcontour;
  /**
  Video gain.
  A negative value marks an automatic gain set by camera
  A positive gain is a manually set gain */
  int vgain; // video gain at camera NB
  /**
  Set when sending data to camera, to enabel active control
  without limiting to camera resolution.
  Used by saturation control. */
  int vgainSet[MAX_HDR_FRAMES];
  /**
  Shutter speed useable range is about 20000 to 65535.
  A negative value sets the shutter to automatic.
  Automatic is usualy a long exposure time and a low video gain. */
  int vshutter;
  /**
  Set when sending data to camera, to enabel active control
  without limiting to camera resolution.
  Used by saturation control. */
  int vshutterSet[MAX_HDR_FRAMES];
  /**
  Saturation avoidance control. If true
  the camera attempts to control gain and shutter to avoid video
  saturation. Every 3rd read frame are tested for saturation
  (every 3rd pixel only) and if max intensity is not (y_max !=220),
  then shutter or gain or both are adjusted.
  Item 0 is fot non-HDR use, ther rest is
  mode for different shutter-gain setting modes */
  unsigned int saturationControl[MAX_HDR_FRAMES];
  /**
  High Dynamic Range image sequence mode.
  If mode = 3, then a set of 3 images are taken with the following
  settings:
  image 1: Normal gain (as set manually or by saturation control)
  image 2: Longer shutter time and possibly higher gain.
  image 3: Shutter closed as a black reference image. */
  int hdrMode;
  /**
  Number of frames in a High Dynamic range set.
  Usually 3 frames are used, one for shaddows, one for sunshine and
  one for dark reference. */
  int hdrFrames;
  /**
  White ballance, with:
  int mode = 'PWC_WB_' + AUTO, MANUAL, INDOOR, OUTDOOR, FL (flurocent).
  int manual_red, manual_blue; gain in range 0..0xFFFF, green is assumed to be 0xF000.
  int read_red, read_blue: actual gain in auto mode. */
  struct pwc_whitebalance vwhite;
  /**
  Image size and framerate */
  struct video_window vwin;
  /**
  Video capabilities - name, max-size etc */
  struct video_capability vcap;
  /**
  Camera picture settings, this include
  brightness, color, contrast, whiteness (gamma), depth and palette.
  is a Video4Linux feature. This is the last value
  read from the camera. A value of 0xFFFF indicates a non-supported feature. */
  struct video_picture vpic;
  /**
  Is values in vpic valid (not valid after a set) */
  bool vpicValid;
  /**
  Camera picture settings, this include
  brightness, color, contrast, whiteness (gamma), depth and palette.
  is a Video4Linux feature.
  This value is the last value send to the camera. */
  struct video_picture vpicSet;
  /**
  Structure with allowed channel information */
  struct video_channel vch;
  /**
  Structure with shared memory buffers capabilities. */
  struct video_mbuf vmbuf;
  /**
  Some cameras has a led attached, and some can be controlled.
  this structure holds the on and off timing values for the led. */
  struct pwc_leds vled;
  /**
  Pointer to start of shared memory with image buffers.
  Should be set to NULL if invalid (or closed) */
  char * vsharedmem;
private:
  unsigned int oldWidth;
};

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////




#endif
