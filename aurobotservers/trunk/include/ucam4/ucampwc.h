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
#ifndef UCAMPWC_H
#define UCAMPWC_H

#include "ucamdevbase.h"
#include <videodev.h>
#include <pwc-ioctl.h>

/**
Implement camera device using the PWC USB camera driver

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UCamPwc : public UCamDevBase
{
  public:
  /**
  Constructor */
  UCamPwc();
  /**
  Destructor */
  virtual ~UCamPwc();
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
  Read frames off the camera device driver queue, and timestamp the images.
  Should not be called directly, are called implicitly,
  when camera is opened.*/
  void readFramesThread(void);
  /**
  Open and close devise to get default camera parameters.
  Returns 0 iff sucessfull. */
  virtual bool openDeviceDefault();
  /**
  Open device and set size and framerate as set in frameHeight,
  frameWidth and frameRate variables.
  If 'readThreadNormal' = false, no read thread is started.
  and image capturing and timestamp is disabled.
  (captureImage will fail) */
  virtual bool openDevice();
  /**
  implement these
  settings and leave the device open. */
  virtual bool setDevice(const int width, const int height,
                     const int newFrameRate);
  /**
  Global function to close the connection to the
  video device.
  Returns true if successful. */
  virtual void closeDevice();
  /**
  Manual control with camera intensity gain and shutter speed.
  NB! This routine does not work - it just sets shutter and AGC
  to automatic. Just kept for future enhancements.
  Returns true if no error occured. */
  virtual bool setShutter(int shut);
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
  virtual int getShutter(bool probe = false, bool * dataValid = NULL, bool * isOnAuto = NULL);
  /**
  Set noise reduction value in camera.
  A value of 0 is no filter and 3 is maximum filter.
  Returns true if set successful - i.e. camera could be locked and
  the function is supported by camera.*/
//  virtual bool setNoiseRed(int noiseReducion);
  /**
  Get actual noise reduction value.
  if 'probe' is false, then existing vaue is returned.
  if 'probe' is true, then the camera is asked
  for the most recent value. if this is
  not possible, then en error is reported and last
  value returned. */
//  virtual int getNoiseRed(bool probe);
  /**
  Set gain value.
  A negative value (-1) sets automatic gain.
  A positive value in range 0..0xFFFF is video gain in
  camera. 0 is low gain (but not zero).
  Returns true if set successful - i.e. camera could be locked and
  the function is supported by camera.*/
  virtual bool setGain(int agc);
  /**
  Get actual gain value.
  if 'probe' is false, then existing vaue is returned.
  if 'probe' is true, then the camera is asked
  for the most recent value. if this is
  not possible, then en error is reported and last
  value returned.
  If dataValid is set, then this is set false on error */
  virtual int getGain(bool probe, bool * dataValid = NULL, bool * isOnAuto = NULL);
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
  and in a range from 0 to 0x7FFF, where 0x4000 are
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
  virtual bool setContour(int value = 0x1000);
  /**
    Get contour value.
    if 'probe' then a value from camera requested is loaded.
    if this fails, an error is issued.
    The function returnes the retreived value (if requested and available)
    otherwise it returns the last fetched value.
    NB! if contour is set to automatic the returned value
    is not related to the implemented value. */
  virtual int getContour(bool probe);
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
  virtual bool setCompPref(int value);
  /**
    Get current setting of compression preference.
    0 = no compression
    3 = highest compression.
    If 'probe' = true the a fresh value is requested from
    camera, otherwise tha last obtained value is returned.
    If a requested value is unavailable, then en error is issued. */
  virtual int getCompPref(bool probe);
  /**
    Set gamma correction value.
    The default setting is 0x8000 (32768),
    Value 0 is assumed to be linear. So gamma > 1.0 is possible only. */
  virtual bool setGamma(int gamma);
  /**
    Get the current gamma correction value.
    The value is an integer in 16 bit range, where 0x8000
    is default. */
  virtual int getGamma(bool probe);
  /**
    Set contrast in camera - normal value is 0x8000 and
    range is 0..0xFFF0 */
  virtual bool setContrast(int contrast);
  /**
    Set brightness in camera - normal value is 0x8000 and
    range is 0..0xFFF0 */
  virtual bool setBrightness(int brightness);
  /**
    Set colour saturation in camera - normal value is 0x8000 and
    range is 0..0xFFF0 */
  virtual bool setColour(int colour);
  /**
    Get contrast in camera - a value of 0xFFFF probably
    means 'not supported'.
    This value is probed together with brightness, colour and gamma,
    so probe for one of these only. */
  virtual int getContrast(bool probe);
  /**
    Get brightness in camera - a value of 0xFFFF probably
    means 'not supported'.
    These values are probed together: Contrast, brightness, colour and gamma,
    so probe for one of them only. */
  virtual int getBrightness(bool probe);
  /**
    Get colour saturation in camera - a value of 0xFFFF probably
    means 'not supported'.
    These values are probed together: Contrast, brightness, colour and gamma,
    so probe for one of them only. */
  virtual int getColour(bool probe);
  /**
    Set all 4 video control values in one call - normal value is 0x8000 and
    range is 0..0xFFF0. */
  virtual bool setVideoCap(int brightness, int contrast, int gamma, int colour);
  /**
    Wait a frame is a function, that calls Wait for
    the time it takes to get a frame. If a parameter is provided
    it waits a fraction of a frametime. */
  virtual void waitAFrame(float frames = 1.0);
  /**
  Set data source channel on a source device, where it is supported.
  Returns true if successful.
  On successful return the name of the channel is available by
  getDataChannelName(). */
  virtual bool setDataChannel(int channel);
  /**
    Get current data channel setting.
    Returns -1 if channel setting is not supported. */
  virtual int getDataChannel();
  /**
  Get current data channel name - e.g. Television or s-video on a
  TV card.
  Channel is set using setDataChannel(). */
  virtual const char * getDataChannelName()
  { return vch.name; };
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
    bool protSetDeviceSizeAndSpeed(int toWidth,
                                   int toHeight,
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
    bool protOpenDevice(const int height,
                        const int width,
                        const int framesPerSec);
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
    bool protGetLockedNewImage(UImage ** prgb);
  /**
    Error messages that may be generated from class */
    void info(const char * s, const int type, const int num)
    { // should probably be overwritten by decendent classes
      printf("(%d,%d): %s\n", type, num, s);
    };
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
  /** Terminate the frame read thread */
  bool stopFrameRead;
  /** Flag that is set by the read thread on entry
  and reset by the thread on exit */
  bool threadRunning;
  /**
  Thread handle for frame read thread. */
  pthread_t thRead;
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
  int oldWidth; /// used while changing image size - somehow?
};


#endif
