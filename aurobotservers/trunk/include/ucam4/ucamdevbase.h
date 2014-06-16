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
#ifndef UCAMDEVBASE_H
#define UCAMDEVBASE_H

#include <ugen4/uimage.h>
#include <ugen4/ulock.h>
#include <urob4/uvariable.h>

// these defines correspond to the PWC defines of the same names - i.e. PWC_WB_AUTO is also defined as 4
#define AU_WB_INDOOR     0
#define AU_WB_OUTDOOR    1
#define AU_WB_FL         2
#define AU_WB_MANUAL     3
#define AU_WB_AUTO       4


class UCamBase;
class UVarPool;

/**
The base class for camera device interface classes

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UCamDevBase : public ULock
{
  public:
  /**
    Constructor */
    UCamDevBase();
  /**
    Destructor */
    virtual ~UCamDevBase();
  /**
    Basic camera types that are supported.*/
    typedef enum camTypes
    {
      CAM_DEV_REPLAY, /// FILE is used for a device to load data from a file.
      CAM_DEV_PWC, /// PWC is a device based on the Philips chipset.
      CAM_DEV_IEEE1394, /// IEEE1394 is a firewire camera.
      CAM_DEV_GRAPPER, /// FRAMEGRAPPER is not a very supported device type.
      CAM_DEV_GIGE
    } USupportedCamTypes;
  /**
    Is the camera of type PWC

    This determines a number of possibe additional options, and
    allow pointer cast to the more specific driver class - not nice but saved time to rewrite
    e.g. the pan-tilt settings.
    @param camType is one of the supported camera types - such as PWC or IEEE1394.
    @return true if camera is of this base-type - i.e. can be safely typecast to a more specific class. */
    virtual bool isThisA(USupportedCamTypes thisType)
    { return camType == thisType;};
  /**
  Det device number - may translate into /dev/videoX if a PWC or other video device */
  int getDeviceNumber()
  { return devNum; };
  /**
  Set image push image buffer - if one buffer is needed only.
  \param img is the image buffer - typically from image pool. */
  virtual void setPushBuffer(UImage * imageBuffer)
  {  // use depends on camera type
  };
  /**
  Set device number for this device.

  NB! this should be set only when the camera is closed
  as no check exist to see if it chenges while the camera is open. */
  void setDeviceNumber(int deviceNum)
  { devNum = deviceNum; };
  /**
  Get image width in pixels from last captured image. */
  inline unsigned int getWidth() {return frameWidth;};
  /**
  Get camera pixel height from last captured image */
  inline unsigned int getHeight() {return frameHeight;};
  /**
  Get image serial number */
  inline unsigned long getImageNumber() { return imageNumber;};
  /**
  Set image serial number */
  inline void setImageNumber(unsigned long serial) { imageNumber = serial;};
  /**
  Get current frame rate setting */
  virtual inline int getFrameRate()
  { return frameRate; };
  /**
  Is camera open in streaming mode. */
    inline bool isCameraOpen()
    {return cameraOpen;};
  /**
  Get camera name - old method name - use getName(). */
  inline char * getCameraName() {return camName; };
  /**
  Get camera name. */
  inline char * getName() {return camName; };
  /**
    Set camera name. */
    void setTypeName(const char * newName);
  /**
    Open devise to get default camera parameters.
    \Returns true if sucessfull. */
    virtual bool openDeviceDefault()
    { return false; };
  /**
    Open device and set size and framerate as set in frameHeight,
    frameWidth and frameRate variables.
    If 'readThreadNormal' = false, no read thread is started.
    and image capturing and timestamp is disabled.
    (captureImage will fail) */
    virtual bool openDevice()
    { return false; };
  /**
    close if the device is open */
    virtual void closeDevice()
    { };
  /**
    implement these
    settings and leave the device open. */
    virtual bool setDevice(const int width, const int height,
                           const int framesPerSec);
  /**
    Manual control with camera intensity gain and shutter speed.
    NB! This routine does not work - it just sets shutter and AGC
    to automatic. Just kept for future enhancements.
    \param shut is either -1 for auto or a 16bit value range [0..65537]
    \Returns true if no error occured. */
    virtual bool setShutter(int shut)
    { return false; };
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
    virtual int getShutter(bool probe = false, bool * dataValid = NULL, bool * isOnAuto = NULL)
    { return vshutter;};
  /**
    Set gain value.
    A negative value (-1) sets automatic gain.
    A positive value in range 0..0xFFFF is video gain in
    camera. 0 is low gain (but not zero).
    \Returns true if set successful - i.e. camera could be locked and
    the function is supported by camera.*/
    virtual bool setGain(int agc)
    { return false; };
  /**
    Get actual gain value.
    if 'probe' is false, then existing vaue is returned.
    if 'probe' is true, then the camera is asked
    for the most recent value. if this is
    not possible, then en error is reported and last
    value returned.
    If dataValid is set, then this is set false on error */
    virtual int getGain(bool probe, bool * dataValid = NULL, bool * isOnAuto = NULL)
    {return vgain; };
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
    \Returns true if successful.  */
    virtual bool setWhiteBalance(int mode, int red, int blue)
    { return false;};
  /**
    Get white balance values
    if 'probe' the the camera is asked, otherwise previous value
    is returned unchecked.
    If one or more of the following pointers are non-zero
    the integer value is returned.
   */
    virtual bool getWhiteBalance(bool probe, int * red = NULL, int * blue = NULL, int * mode = NULL)
    { return false; };
  /**
    Convert a WB mode to a descriptive string.
    'strFuff most be a buffer of at least 10 bytes.
    \Returns a pointer to the buffer, where the mode-string is filled. */
    static char * getWBModeAsString(int mode, char * strBuff);
  /**
    Convert white ballance mode to integer. The modes
    are 'auto' indoor' 'outdoor' 'flurocent' and 'manual'.
    \Returns the mode, and auto, if mode is not recognized. */
    int getWBModeFromString(const char * mode);
  /**
    Set led speed
    time is in miliseconds.
  (on,off):
    (0,0) = OFF,
    (100,0) = ON,
    (200,800) = 1 sec blink with 20% on.
    Returns true if set.   */
    //bool setLed(int onTime, int offTime);
  /**
    Get setting or last setting */
    //bool getLed(bool probe, int * onTime, int * offTime);
  /**
    Set led on */
/*    inline void setLedOn(bool toOn)
    {
      if (toOn)
        setLed(100,0);
      else
        setLed(0,0);
    };*/
  /**
    Is led on - sometime */
//    bool isLedOn(bool probe);
  /**
    Set led blink proportional to framerate.
    That is on in 200 ms and off in 5/frameRate seconds. */
  //  inline void setLedFrameRate() { setLed(200,5000/frameRate);};
  /**
    Set the contour function in camera.
    This is an edge sharpener to compensate for lack
    of bandwidth (horizontal lines only.
    The range is 0 to 0xFFFF.
    The value 0x0 is no contour correction.
    If set negative (e.g. -1) the camera selects an appropriate
    value.
    \Returns true if command issued. */
    virtual bool setContour(int value = 0x1000)
    { vcontour = value; return false; };
  /**
    Get contour value.
    if 'probe' then a value from camera requested is loaded.
    if this fails, an error is issued.
    The function returnes the retreived value (if requested and available)
    otherwise it returns the last fetched value.
    NB! if contour is set to automatic the returned value
    is not related to the implemented value. */
    virtual int getContour(bool probe)
    { return vcontour; };
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
    virtual bool setCompPref(int value)
    {  vcompressionPref = value;
       return false; };
  /**
    Get current setting of compression preference.
    0 = no compression
    3 = highest compression.
    If 'probe' = true the a fresh value is requested from
    camera, otherwise tha last obtained value is returned.
    If a requested value is unavailable, then en error is issued. */
    virtual int getCompPref(bool probe)
    { return vcompressionPref; };
    /**
    Set gamma correction value.
    The default setting is 0x8000 (32768),
    Value 0 is assumed to be linear. So gamma > 1.0 is possible only. */
    virtual bool setGamma(int gamma)
    { vgamma = gamma; return false;};
  /**
    Get the current gamma correction value.
    The value is an integer in 16 bit range, where 0x8000
    is default. */
    virtual int getGamma(bool probe)
    { return vgamma; };
  /**
    Set contrast in camera - normal value is 0x8000 and
    range is 0..0xFFF0 */
    virtual bool setContrast(int contrast)
    { vcontrast = contrast; return false; };
  /**
    Set brightness in camera - normal value is 0x8000 and
    range is 0..0xFFF0 */
    virtual bool setBrightness(int brightness)
    { vbrightness = brightness; return false; };
  /**
    Set colour saturation in camera - normal value is 0x8000 and
    range is 0..0xFFF0 */
    virtual bool setColour(int colour)
    { vcolour = colour; return false; };
  /**
    Get contrast in camera - a value of 0xFFFF probably
    means 'not supported'.
    This value is probed together with brightness, colour and gamma,
    so probe for one of these only. */
    virtual int getContrast(bool probe)
    { return vcontrast; };
  /**
    Get brightness in camera - a value of 0xFFFF probably
    means 'not supported'.
    These values are probed together: Contrast, brightness, colour and gamma,
    so probe for one of them only. */
    virtual int getBrightness(bool probe)
    { return vbrightness; };
  /**
    Get colour saturation in camera - a value of 0xFFFF probably
    means 'not supported'.
    These values are probed together: Contrast, brightness, colour and gamma,
    so probe for one of them only. */
    virtual int getColour(bool probe)
    { return vcolour; };
  /**
    Set all 4 video control values in one call - normal value is 0x8000 and
    range is 0..0xFFF0. */
    virtual bool setVideoCap(int brightness, int contrast, int gamma, int colour)
    { return false; };
/**
  Set external trigger
  \param value set to external trigger if true, else internal (free run) trigger.
  \param supported is set true if external trigger is supported by device.
  \returns true if value is set as specified. */
  virtual bool setExternalTrigger(bool value, bool * supported)
  {
    if (supported != NULL)
      *supported = false;
    return false;
  };
/**
  Get external trigger support flag
  \param supported is set true if external trigger is supported by device.
  \returns true if external trigger support call is successful. */
  virtual bool getExternalTrigger(bool * supported)
  {
    if (supported != NULL)
      *supported = false;
    return false;
  };
  /**
  Trigger a new set of images, if this is allowed by the camera.
  Does nothing if not connected and if external trigger is not available.
  \returns true if calls were successfull. */
  virtual bool makeTriggerPulse()
  { return false; };
  /**
  Called when a new image is available.
  NB! this function should not be called in camera thread, but by the main server thread,
  \param image is the new image in any colour format (and may be locked for driver purposes) */
  void gotNewImage(UImage * rgb);
  /**
  Got new image - this is a call to inform server thread that an new-image
  event has occured for this camera device. - sets a flag and requests the image when main thread is ready. */
  void imgUpdated();
  /**
    Set data source channel on a source device, where it is supported.
    Returns true if successful.
    On successful return the name of the channel is available by
    getDataChannelName(). */
    virtual bool setDataChannel(int channel)
    { return false; };
  /**
    Get current data channel setting.
    Returns -1 if channel setting is not supported. */
    virtual int getDataChannel()
    { return 0; };
  /**
    Get current data channel name - e.g. Television or s-video on a
    TV card.
    Channel is set using setDataChannel(). */
    virtual const char * getDataChannelName()
    { return NULL; };
  /**
  Get pixel size relative to 640x480. */
  inline double getPixelSize()
  { 
    if (frameWidth > 1.0)
      return MAX_IMAGE_WIDTH / double(frameWidth);
    else 
      return 1.0;
  };
  /**
  Test if souch device exist and can be opened.
  Returns true if device can be opened (a device exists). */
  virtual bool deviceExist()
  { return false; };
  /**
  Set the parameters derived from resolution, i.e.
  conversion matrix, radial error valeues etc.
  but at this level just the resolution factor. */
  void imageSizeChanged(double iResFactor);
  /**
  Get the latest new image from the image buffer

  lock it so that the user may copy the content if needed, the
  user will then need to unlock the read buffer after use
  (or the image buffer is lost for ever, and will drain
  the tmage read capabilities. */
  bool getLockedNewImage(UImage ** raw);
  /**
  Get device handle for external manipulations and access */
  int getCamFd()
  { return cam_fd; };
  /**
  Set reference to camera, so that camera cam be informed when a
  new image has arrived */
  void setCam(UCamBase * camRef)
  { cam = camRef; };
  /**
  Get new image - the newest available.
  if 'image' == NULL, then get and discard an image.
  \returns true if an image could be captured. */
  virtual bool getImageSnapshot(UImage * image);
  /**
  Request to test push commands for need of new data.
  If new data is needed then 'gotNewImage()' should be called.
  \returns true if data is needed. */
  bool needNewPushData();
  /**
  Set camera as initialized */
  inline void setInitialized(bool value)
  { initialized = value; };
  /**
  Set pointer to established var-pool structure for locally maintained variables */
  void setVarPool(UVarPool * vpd)
  {
    vars = vpd;
    createVars();
  };
  /**
  Create locally maintained variables - if any */
  virtual void createVars();
  /**
  Called from push structure to get push object
  and do a call to 'gotNewData(object)'.
  Should be overwritten by push object holder. */
  inline virtual void callGotNewDataWithObject()
  { // dummy call, as if no data were available
    gotNewImage(NULL);
  }
  /**
  Set value of log-variable - if true, then all used images are logged. */
  void setLog(bool value);
  /**
  Is logging set to true */
  inline bool isLog()
  {
    if (varLog != NULL)
      return varLog->getBool();
    else
      return false;
  }
  /**
  Set value of replay-variable - if true, then image may be set from replay. */
  void setReplay(bool value);
  /**
  Is logging set to true */
  inline bool isReplay()
  {
    if (varReplay != NULL)
      return varReplay->getBool();
    else
      return false;
  }
  
public:
  /**
  Max image size, used for size reference in
  pixel size calculation, and should not be changed from
  640x480 - also used for by radial error calculation matrix. */
  static const int MAX_IMAGE_WIDTH = 640;

public:
  /**
  Max length of camera name */
  static const int MAX_CAM_DEV_NAME_LENGTH = 100;

protected:
  int cam_fd; /// file descriptor for camera device
  /**
  Device number - translates to /dev/video0, -1, -2 ... if video device or something else if it is a IEEE1394 device */
  int devNum;
  /**
  Camera is open - straming images */
  bool cameraOpen;
  /**
  Size of images read from device (expected size) */
  int frameHeight;
  /**
  Size of images read from device (expected size) */
  int frameWidth;
  /**
  Framerate from camera (frames/sec (rounded) */
  int frameRate;
  /**
  Serial number for next used image.
  images not used are not counted - see imCnt. */
  unsigned long imageNumber;
  /**
  Camera name - as obtained from device */
  char camName[MAX_CAM_DEV_NAME_LENGTH];
  /**
  Video gain.
  A negative value marks an automatic gain set by camera
  A positive gain is a manually set gain
  Value is in range 0..0xffff and -1 is automatic */
  int vgain;
  /**
  Shutter speed useable range is about 20000 to 65535.
  A negative value sets the shutter to automatic.
  Automatic is usualy a long exposure time and a low video gain.
  Value is in range 0..0xffff and -1 is automatic */
  int vshutter;
  /**
  Compression preference range [0-3].
  0= no compression, ... 3=higest compression. */
  int vcompressionPref;
  /**
  Contour value to sharpen edges in image
  Value is in range 0..0xffff and -1 is automatic */
  int vcontour;
  /**
  Image brightness setting, if supported by camera
  Value is in range 0..0xffff and -1 is automatic */
  int vbrightness;
  /**
  Image contrast setting - if supported by camera
  Value is in range 0..0xffff and -1 is automatic */
  int vcontrast;
  /**
  Image colour saturation setting - if supported by camera
  Value is in range 0..0xffff and -1 is automatic */
  int vcolour;
  /**
  Image gamma setting - if supported by camera.
  Value is in range 0..0xffff and -1 is automatic */
  int vgamma;
  /**
  Pointer to the camera using this device and thus need
  information on events in this camera device. */
  UCamBase * cam;
  /**
  Image count since last camera device were last opened */
  int imCnt;
  /**
  size of image buffer for use by the read thread. */
  static const int RAW_IMAGEBUFFER_MAX_CNT = 5;
  /**
  Image buffer for images read by read thread */
  UImage * imgBuff[RAW_IMAGEBUFFER_MAX_CNT];
  /**
  Next image buffer to use */
  int imgBuffNext;
  /**
  Camera type */
  USupportedCamTypes camType;
  /**
  Camera is initialized */
  bool initialized;
  /**
  Pointer to established structure for device deopendent variables */
  UVarPool * vars;
  /// Variable for log flag
  UVariable * varLog;
  /// Variable for replay
  UVariable * varReplay;
  /// camera name from camera
  UVariable * varCamName;
};


#endif
