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
#include "ucampwc.h"

#include <asm/errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
//#include "/usr/src/linux/drivers/usb/pwc-ioctl.h"
//#include "/usr/src/linux/drivers/usb/pwc-uncompress.h"
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
//#include "ucamcommon.h"
#include "ucamdevice.h"


#define einfo    0
#define ewarning 1
#define eerror   2
#define edebug   3

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

void * runPwcCamThread(void * camobj)
{ // Start thread here and call thread function
  UCamPwc * cam;
  //
  cam = (UCamPwc *) camobj;
  cam->readFramesThread();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/////////////////////////////////////////////////

UCamPwc::UCamPwc()
{
  cameraOpen = false;
  devInfoValid = false;
  oldWidth = 0; // to detect size change
  vpicValid = false;
  //
  threadRunning = false;
  stopFrameRead = true;
  // newtral settings
  vpicSet.whiteness = 0x8000; // gamma
  vpicSet.contrast = 0x8000;
  vpicSet.brightness = 0x8000;
  vpicSet.colour = 0x8000;
  vpicSet.hue = 0x8000;
  vpic = vpicSet;
  vpicValid = false;
  //
  vch.name[0] = '\0';
  vch.channel = 0;
  vsharedmem = NULL;
  camType = CAM_DEV_PWC;
  frameHeight = 240;
  frameWidth = 320;
}

/////////////////////////////////////////////////

UCamPwc::~UCamPwc()
{ // stop thread if running
  stopReadThread(true);
  // ensure camera connection is closed
  closeDevice();
  //printf("PWC cam disposed\n");
}


///////////////////////////////////////////////////////

void UCamPwc::readFramesThread(void)
{
  int err = 0;
  int i, got;
//  URawImage * rawimg;
  UImage * rgbimg;
  UTime t1;
  int errCnt = 0;
  struct video_mmap vmmap;
  //
  // clear buffers
  for (i = 0; i < RAW_IMAGEBUFFER_SIZE; i++)
  {
    if (imgBuff[i] == NULL)
    {
      imgBuff[i] = new UImage800();
    }
    imgBuff[i]->camDevice = devNum;
    imgBuff[i]->setSize(frameHeight, frameWidth, 3, 8, "RGB");
    imgBuff[i]->valid = false;
  }
  vmmap.frame = 0;
  vmmap.format = VIDEO_PALETTE_RGB24;
  vmmap.width  = getWidth();
  vmmap.height = getHeight();
  // image read count
  imCnt = 0;
  imgBuffNext = 0;
  if (err == 0)
  { // buffers available
    threadRunning = true;
    //
    // debug
    //printf("UCamPwc::readFramesThread -- thread started\n");
    // debug end
    printf("UCamPwc:: found %s\n", getCameraName());
    //
    // tap images from device
    while (not stopFrameRead)
    { // read one frame
      //rawimg = raw[imgBuffNext];
      rgbimg = imgBuff[imgBuffNext];
      i = 0;
      while (not rgbimg->tryLock())
      {
        if (stopFrameRead)
        { // dont continue
          err = -1;
          break;
        }
        // try next frame - someone is reading the last
        imgBuffNext = (imgBuffNext + 1) % RAW_IMAGEBUFFER_SIZE;
        //rawimg = raw[imgBuffNext];
        rgbimg = imgBuff[imgBuffNext];
        i++;
        if (i >= RAW_IMAGEBUFFER_SIZE)
        { // all frames are locked - should not be
          printf("*** UCameraDevice::readFramesThread: all frames locked!\n");
          i = 0;
          err = -1;
          break;
        }
        if (i > 1)
          printf("*** UCameraDevice::readFramesThread: image buffer locked\n");
      }
      if (err == 0)
      { // image is locked
        // mark as invalid, while fetching and while camera closed
        rgbimg->valid = false;
        // fetch image
        err = -1;
        if (cameraOpen)
        {
          if (tryLock())
          { // camera is locked during read operation
            // but may just have been closed
            if (cameraOpen)
            { // ensure frame butter is right size
              if (vpic.palette == VIDEO_PALETTE_YUV420P)
                  // image size has changed - redirect pointers
                rgbimg->setSize(frameHeight, frameWidth, 3, 8, "YUV420");
              else if (vpic.palette == VIDEO_PALETTE_RGB24)
                  // image size has changed - redirect pointers
                rgbimg->setSize(frameHeight, frameWidth, 3, 8, "RGB");
              //
              if (vsharedmem == NULL)
              { // try a read
                // time the read-time from driver
                //rgbimg->stat.clear();
                t1.Now();
                got = read(cam_fd, rgbimg->getYline(0), rgbimg->getDataSize());
                if ((got < 0) and (errCnt < 10))
                {
                  perror("UCamPwc::readFramesThread:read(...)");
                  errCnt++;
                }
                if (got == int(rgbimg->getDataSize()))
                { //printf("Read %d bytes\n", got);
                  rgbimg->imgTime.Now();
                  rgbimg->readDelay = rgbimg->imgTime.GetDecSec(t1);
                  rgbimg->valid = true;
                  rgbimg->imageNumber = imageNumber++;
                // if less than 10 ms, then taken from old image buffer
                // if more than 250 ms (< 5 frames per second) then just opened
                // camera - this must be tested by the client if need be
                  err = 0;
                }
              }
              else
              { // start capture using buffer capture
                vmmap.frame++;
                if (vmmap.frame >= (unsigned int)vmbuf.frames)
                  vmmap.frame = 0;
                err = ioctl(getCamFd(), VIDIOCMCAPTURE, &vmmap);
                if ((err != 0) and (errCnt < 10))
                {
                  perror("UCamPwc::readFramesThread:VIDIOCMCAPTURE");
                  errCnt++;
                }
                // set destination image size
                rgbimg->setSizeOnly(frameHeight, frameWidth);
                t1.Now();
                // wait until image is ready
                err = ioctl(getCamFd(), VIDIOCSYNC, &vmmap);
                if ((err != 0) and (errCnt < 10))
                {
                  perror("UCamPwc::readFramesThread:VIDIOCSYNC");
                  errCnt++;
                  if (errCnt > 5)
                  { // switch back to read method
                    munmap(vsharedmem, vmbuf.size);
                    vsharedmem = NULL;
                  }
                }
                if ((err == 0) and (vsharedmem != NULL))
                { //printf("Read %d bytes\n", got);
                  memmove(rgbimg->getData(),
                          vsharedmem + vmbuf.offsets[vmmap.frame],
                          rgbimg->getDataSize());
                  rgbimg->imgTime.Now();
                  rgbimg->readDelay = rgbimg->imgTime.GetDecSec(t1);
                  rgbimg->valid = true;
                  rgbimg->imageNumber = imageNumber++;
                // if less than 10 ms, then taken from old image buffer
                // if more than 250 ms (< 5 frames per second) then just opened
                // camera - this must be tested by the client if need be
                  err = 0;
                }
              }
            }
            //
            if (err == 0)
              imCnt++;
            // release lock
            unlock();
          }
        }
        //
        if (err != 0)
        { // camera is closed or locked - configuration change
          // wait a while
          Wait(0.150);
        }
        //
        if (err == 0)
        { // need for statistics?
          imgBuffNext = (imgBuffNext + 1) % RAW_IMAGEBUFFER_SIZE;
        }
        rgbimg->unlock();
        // send trigger for users of every image
        if (rgbimg->valid)
          gotNewImage(rgbimg);
        // wait a bit for others to lock the camera
        Wait(0.0011);
      }
      err = 0;
    }
  }
  threadRunning = false;
  //
  //invalidate remaining frames
  for (i = 0; i < RAW_IMAGEBUFFER_SIZE; i++)
  {
    if (imgBuff[i] != NULL)
      imgBuff[i]->valid = false;
  }
  //
  // debug
  printf("UCamPwc::readFramesThread -- thread stopped\n");
  // debug end
  //
}

///////////////////////////////////////////////////////

// bool UCamPwc::setSizeAndFramerate(
//                       const int width,
//                       const int height,
//                       const int framerate)
// {
//   bool result = false;
//   if (cameraOpen)
//     result = openAndSetDevice(width, height, framerate);
//   else
//   {
//     if (lock())
//     { // just set size and leave to openDevice() to implement
//       frameWidth = width;
//       frameHeight = height;
//       frameRate = framerate;
//       imageSizeChanged(MAX_IMAGE_WIDTH/float(width));
//       unlock();
//       result = true;
//     }
//   }
//   return result;
// }

///////////////////////////////////////////////////////

bool UCamPwc::protCloseDevice()
{
  bool result = true;
  int err;
  //
  if (cameraOpen)
  { // stop the frame read thread
    cameraOpen = false;
    // unmap shared memory for framegrabber devices
    if (vsharedmem != NULL)
    {
      err = munmap(vsharedmem, vmbuf.size);
      if (err != 0)
        perror("munmap on close");
      vsharedmem = NULL;
    }
    // close device
    result =(close(cam_fd) == 0);
    if (not result)
      perror(vcap.name);
    // mark as closed
    cam_fd = -1;
    imCnt = 0;
    // debug
    // printf("UCamPwc::protCloseDevice: device=%d closed %s\n", devNum, bool2str(result));
    // debug end
  }
  return result;
}

///////////////////////////////////////////////////////

void UCamPwc::closeDevice()
{
  lock();
  protCloseDevice();
  unlock();
}

/////////////////////////////////////////////////////////

bool UCamPwc::setShutter(int shut)
{ // test features for the camera control
  bool result;
  int err = 0;
  //
  result = cameraOpen and lock();
  if (result)
  {
    vshutter = shut;
    err = ioctl(cam_fd, VIDIOCPWCSSHUTTER, &vshutter);
    result = (err == 0);
    unlock();
    if (not result)
      ioctlError(err, "setShutter");
  }
  //
  return (err == 0);
}

////////////////////////////////////////////////////////

int UCamPwc::getShutter(bool probe, bool * dataValid, bool * isOnAuto)
{ // get actual shutter value
  /* can not ask for shutter value
  int err;
  int got;
  //
  if (probe and lock())
  { // ask camera
  err = ioctl(cam_fd, VIDIOCPWCGSHUTTER, &got);
  if (err == 0)
  shutter = got;
  unlock();
  if (err != 0)
  ioctlError(-err, "getShutter");
}
  */
  if (isOnAuto != NULL)
    *isOnAuto = (vsharedmem < 0);
  if (dataValid != NULL)
    *dataValid = true;
  return vshutter;
}

////////////////////////////////////////////////////////

//VIDIOCPWCSDYNNOISE / VIDIOCPWCGDYNNOISE

// bool UCamPwc::setNoiseRed(int noiseReduction)
// { // test features for the camera control
//   bool result;
//   int err;
//   //
//   result = cameraOpen and lock();
//   if (result)
//   {
//     vnoiseReduc = noiseReduction;
//     if (cam_fd >= 0)
//     {
//       err = ioctl(cam_fd, VIDIOCPWCSDYNNOISE, &vnoiseReduc);
//       result = (err == 0);
//       if (result)
//         // ask camera for actual value
//         err = ioctl(cam_fd, VIDIOCPWCGDYNNOISE, &vnoiseReduc);
//       else
//         ioctlError(err, "setNoiseRed");
//     }
//     unlock();
//   }
//   //
//   return result;
// }
//
// ////////////////////////////////////////////////////////
//
// int UCamPwc::getNoiseRed(bool probe)
// { // get gain value
//   int err = 0;
//   int got;
//   //
//   if (probe and not simulated)
//     if (lock())
//   { // ask camera
//     if (not simulated)
//     {
//       err = ioctl(cam_fd, VIDIOCPWCGDYNNOISE, &got);
//       if (err == 0)
//         vnoiseReduc = got;
//     }
//     unlock();
//     if (err != 0)
//       ioctlError(-err, "getNoiseRed");
//   }
//   return vnoiseReduc;
// }

////////////////////////////////////////////////////////

bool UCamPwc::setGain(int agc)
{ // test features for the camera control
  bool result;
  int err;
  //
  result = cameraOpen and (cam_fd >= 0);
  if (result)
  {
    lock();
    vgain = agc;
    err = ioctl(cam_fd, VIDIOCPWCSAGC, &vgain);
    result = (err == 0);
    if (result)
      // ask camera
      err = ioctl(cam_fd, VIDIOCPWCGAGC, &vgain);
    else
      ioctlError(err, "setGain");
    unlock();
  }
  //
  return result;
}

////////////////////////////////////////////////////////

int UCamPwc::getGain(bool probe, bool * dataValid, bool * isOnAuto)
{ // get gain value
  int err = 0;
  int got = -1;
  //
  if (probe)
  {
    lock();
    // ask camera
    err = ioctl(cam_fd, VIDIOCPWCGAGC, &got);
    if (err == 0)
      vgain = got;
    unlock();
    if (err != 0)
      ioctlError(err, "getGain");
  }
  if (dataValid != NULL)
    *dataValid = (err == 0);
  if (isOnAuto != NULL)
    *isOnAuto = (got < 0);
  return vgain;
}

////////////////////////////////////////////////////////

bool UCamPwc::getWhiteBalance(bool probe, int * red, int * blue, int * mode)
{ // get red and blue gain, and mode (4=auto, 3=manual)
  bool result = true;
  int err;
  //
  if (probe)
  {
    lock();
    // ask camera
    err = ioctl(cam_fd, VIDIOCPWCGAWB, &vwhite);
    result = (err == 0);
    unlock();
    if (not result)
      ioctlError(-err, "getWhite");
  }
  if (red != NULL)
  {
    if (vwhite.mode == PWC_WB_MANUAL)
      *red = vwhite.manual_red;
    else
      *red = vwhite.read_red;
  }
  if (blue != NULL)
  {
    if (vwhite.mode == PWC_WB_MANUAL)
      *blue = vwhite.manual_blue;
    else
      *blue = vwhite.read_blue;
  }
  if (mode != NULL)
    *mode = vwhite.mode;
  return result;
}

/////////////////////////////////////////////////////////

bool UCamPwc::setWhiteBalance(int mode, int red, int blue)
{ // set mode (4=auto, 3=manual), red and blue gain
  bool result = true;
  int err;
  //
  result = cameraOpen and cam_fd >= 0;
  if (result)
  {
    result =  tryLock();
    if (result)
    { // set values
      // mode 0=indoor, 1=outdoor, 2=flurocent, 3=manual
      vwhite.mode = mode;
      vwhite.manual_red = red;
      vwhite.manual_blue = blue;
      //
      err = ioctl(cam_fd, VIDIOCPWCSAWB, &vwhite);
      result = (err == 0);
      if (result)
        // get resulting whiteballabce values from camera
        err = ioctl(cam_fd, VIDIOCPWCGAWB, &vwhite);
      else
        ioctlError(err, "setWhite");
      unlock();
    }
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamPwc::isLedOn(bool probe)
{
  if (probe)
    getLed(probe, NULL, NULL);
  return (vled.led_on > 0);
}

///////////////////////////////////////////////////////

bool UCamPwc::setLed(int onTime, int offTime)
{
  bool result = true;
  /*
  int err;
  //
  result =  lock();
  if (result)
  { // set values
  vled.led_on = onTime;
  vled.led_off = offTime;
  //
  err = ioctl(cam_fd, VIDIOCPWCSLED, &vled);
  result = (err == 0);
  unlock();
  if (not result)
  ioctlError(err, "setLed");
}
  */
  return result;
}

///////////////////////////////////////////////////////

bool UCamPwc::setDataChannel(int channel)
{
  bool result = true;
  int err;
  //
  lock();
  if (result)
  {
    vch.channel = channel;
    err = ioctl(getCamFd(),  VIDIOCGCHAN, &vch);
    if (err != 0)
    {
      perror("UCamPwc::setChannel: VIDIOCGCHAN");
      vch.channel = -1;
    }
    if (err == 0)
    { // set to requested channel
      // vch.channel = 0;
      err = ioctl(getCamFd(),  VIDIOCSCHAN, &vch);
      if (err != 0)
        perror("UCamPwc::setChannel: VIDIOCGCHAN");
    }
    unlock();
    result = (err == 0);
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamPwc::getDataChannel()
{
  return vch.channel;
}

////////////////////////////////////////////////////////

bool UCamPwc::getLed(bool probe, int * onTime, int * offTime)
{
  bool result = true;
  /*
  int err;
  //
  if (probe)
  if (lock())
  { // get values
  err = ioctl(cam_fd, VIDIOCPWCGLED, &vled);
  result = (err == 0);
  unlock();
  if (not result)
  ioctlError(err, "getLed");
}
  if (result)
  {
  if (onTime != NULL)
  *onTime = vled.led_on;
  if (offTime != NULL)
  *offTime = vled.led_off;
}
  */
  return result;
}

////////////////////////////////////////////////////////

bool UCamPwc::setContour(int value /*= 0x3000*/)
{
  bool result = true;
  int err = 0;
  //
  result =  cam_fd >= 0;
  if (result)
  { // set values
    lock();
    vcontour = value;
    err = ioctl(cam_fd, VIDIOCPWCSCONTOUR, &vcontour);
    result = (err == 0);
    unlock();
    if (result)
      // ask camera
      err = ioctl(cam_fd, VIDIOCPWCGCONTOUR, &vcontour);
    else
      ioctlError(err, "setContour");
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamPwc::getContour(bool probe)
{
  int err;
  int val;
  //
  if (probe and (cam_fd >= 0))
  {
    lock();
    // get values
    err = ioctl(cam_fd, VIDIOCPWCGCONTOUR, &val);
    unlock();
    if (err == 0)
      vcontour = val;
    else
      ioctlError(err, "getContour");
  }
  return vcontour;
}

////////////////////////////////////////////////////////

bool UCamPwc::setCompPref(int value /*= 0x0*/)
{
  bool result = true;
  int err = 0;
  //
  result = (cam_fd >= 0);
  if (result)
  { // set values
    vcompressionPref = value;
    err = ioctl(cam_fd, VIDIOCPWCSCQUAL, &vcompressionPref);
    result = (err == 0);
    unlock();
    if (result)
      err = ioctl(cam_fd, VIDIOCPWCGCQUAL, &vcompressionPref);
    else
      ioctlError(err, "setCompression");
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamPwc::getCompPref(bool probe)
{
  int err;
  int val;
  //
  if (probe)
  {
    lock();
    // get values
    err = ioctl(cam_fd, VIDIOCPWCGCQUAL, &val);
    unlock();
    if (err == 0)
      vcompressionPref = val;
    else
      ioctlError(err, "getCompression");
  }
  return vcompressionPref;
}

////////////////////////////////////////////////////////

bool UCamPwc::setGamma(int gamma)
{
  bool result = true;
  int err = 0;
  //
  result =  lock() and (cam_fd >= 0);
  if (result)
  { // set values
    /*
    vpic.whiteness = gamma;
    vpic.contrast = 0x8000;
    vpic.brightness = 0x8000;
    vpic.colour = 0x8000; */
    vpicSet.whiteness = gamma;
    err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
    result = (err == 0);
    unlock();
    if (not result)
      ioctlError(err, "setVideoGamma");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamPwc::setContrast(int contrast)
{
  bool result = true;
  int err = 0;
  //
  result =  (cam_fd >= 0);
  if (result)
  { // set values
    lock();
    vpicSet.contrast = contrast;
    //vpicSet.brightness = 0x8000;
    //vpicSet.colour = 0x8000;
    err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
    result = (err == 0);
    unlock();
    if (not result)
      ioctlError(err, "setVideoContrast");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamPwc::setBrightness(int brightness)
{
  bool result = true;
  int err = 0;
  //
  result =  (cam_fd >= 0);
  if (result)
  { // set values
    lock();
    vpicSet.brightness = brightness;
    //vpicSet.colour = 0x8000;
    err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
    result = (err == 0);
    unlock();
    if (not result)
      ioctlError(err, "setVideoBrightness");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamPwc::setColour(int colour)
{
  bool result = true;
  int err;
  //
  result =  (cam_fd >= 0);
  if (result)
  { // set values
    lock();
    // vpicSet.brightness = brightness;
    vpicSet.colour = colour;
    err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
    result = (err == 0);
    unlock();
    if (not result)
      ioctlError(err, "setVideoColour");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamPwc::getBrightness(bool probe)
{
  int err;
  //
  if (probe or ((not vpicValid) and (cam_fd >= 0)))
  {
    lock();
    // get values
    err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
    unlock();
    if (err != 0)
      ioctlError(err, "getVideoBrightness");
    else
      vpicValid = true;
  }
  return vpic.brightness;
}

////////////////////////////////////////////////////////

int UCamPwc::getContrast(bool probe)
{
  int err;
  //
  if (probe or (not vpicValid and (cam_fd >= 0)))
  { // get values
    lock();
    err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
    unlock();
    if (err != 0)
      ioctlError(err, "getVideoContrast");
    else
      vpicValid = true;
  }
  return vpic.contrast;
}

////////////////////////////////////////////////////////

int UCamPwc::getColour(bool probe)
{
  int err;
  //
  if ((probe or not vpicValid) and (cam_fd >= 0))
  {
    lock();
    // get values
    err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
    unlock();
    if (err != 0)
      ioctlError(err, "getVideoColour");
    else
      vpicValid = true;
  }
  return vpic.colour;
}

////////////////////////////////////////////////////////

bool UCamPwc::setVideoCap(int brightness, int contrast, int gamma, int colour)
{
  bool result = true;
  int err;
  //
  result = (cam_fd >= 0);
  if (result)
  { // set values
    lock();
    vpicSet.whiteness = gamma;
    vpicSet.contrast = contrast;
    vpicSet.brightness = brightness;
    vpicSet.colour = colour;
    vpicSet.hue = colour;
    err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
    result = (err == 0);
    unlock();
    if (not result)
      ioctlError(err, "setVideoCap");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamPwc::getGamma(bool probe)
{
  int err;
  //
  if ((probe or not vpicValid) and (cam_fd >= 0))
  {
    lock();
    // get values
    err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
    unlock();
    if (err != 0)
      ioctlError(err, "getVideoGamma");
    else
      vpicValid = true;
  }
  return vpic.whiteness;
}

////////////////////////////////////////////////////////

void UCamPwc::ioctlError(int err, const char * prestring)
{
  const int MSL = 200;
  char s[MSL];
  int n;
  //
  n = -EFAULT;
  if (err == n)
  {
    snprintf(s, MSL,
             "IOCTL failed for 'UCamPwc::%s' (faulty parameter or not supported?)", prestring);
    perror(s);
  }
  else if (err == 0)
    ;
  else
  {
    snprintf(s, MSL,
             "IOCTL failed (%d) for %s (simulated or not supported?)'", err, prestring);
    perror(s);
  }
}

///////////////////////////////////////////////////////////

// bool UCamPwc::protGetLockedNewImage(UImage ** prgb)
// { // finds the most recent bufer with a timestamped image
//   // and returns it locked.
//   // opens the camera is closed.
//   bool isOK;
//   int bufNum;
//   bool validAndLocked = false;
//   int i = 0; //, j, n;
//   //
//   // if device is not open, or open in a wrong mode
//   // (re)open in right mode
//   isOK = cameraOpen;
//   // open camera
//   if (not isOK)
//     isOK = protOpenDevice(false);
//   // start read thread -- if not running already
//   if (isOK and not threadRunning)
//     isOK = startReadThread();
//   // test for available images (image thread got first image)
//   if (isOK and (imCnt == 0))
//   { // wait for first image
//     i = 0;
//     while ((imCnt == 0) and (i < 40))
//     { // no image is read yet, size is just changed and
//       // this can take time (up to 2 seconds).
//       Wait(0.05);
//       i++;
//     }
//     isOK = imCnt > 0;
//   }
//   // debug
//   // printf("Get new image cameraOpen=%s thread=%s imageCnt=%d\n",
//   //        bool2str(cameraOpen), bool2str(threadRunning), imCnt);
//   // debug end
//   // get an image
//   if (isOK)
//   { // open and in right mode, now get image
//     i = 0;
//     do
//     { // get buffer pointer just released by read thread
//       bufNum = (imgBuffNext - 1 + RAW_IMAGEBUFFER_SIZE)
//           % RAW_IMAGEBUFFER_SIZE;
//       // get latest frame pointer
//       //*praw = raw[bufNum];
//       *prgb = rgb[bufNum];
//       validAndLocked = false;
//       if ((*prgb)->tryLock())
//       { // lock frame to avoid reuse by read thread
// /*        if (vpic.palette == VIDEO_PALETTE_YUV420P)
//         validAndLocked = (*praw)->valid;
//         else*/
//         validAndLocked = (*prgb)->valid;
//         if (not validAndLocked)
//         { // image not available or already used
//           (*prgb)->unlock(); // release
// //          n = 0; // count valid frames
// /*          for (j = 0; j < RAW_IMAGEBUFFER_SIZE; j++)
//           {
//           if (((vpic.palette == VIDEO_PALETTE_YUV420P)
//           and (raw[j]->valid))
//           or
//           ((vpic.palette == VIDEO_PALETTE_RGB24)
//           and (raw[j]->valid)))
//           n++; // count number of usable frames
//         }*/
// /*          if ((i % 200) == 199) // should not take more than 2 s (5 frames/sec)
//           {
//           printf("*** UCamPwc:: "
//           "did not get image after %d ms (%d of %d buffers valid)\n",
//           i, n, RAW_IMAGEBUFFER_SIZE);
//         }*/
//           Wait(0.02); // max 3 waits at 30 frames / sec
//         }
//       }
//       i += 20; // add wait time in miliseconds
//       // if not valid, then wait for up to 2 seconds
//     } while (i < 2000 and not validAndLocked);
//     //
//     // image is always in YUV format with
//     // 4:1:1 relation between Y, U and V (YUV 4:2:0 Planer)
//     if (validAndLocked)
//     { // image is valid and locked
//       //imageNumber++;
//       //result->imageNumber = imageNumber;
//     }
//     else
//     {
//       //praw = NULL;
//       prgb = NULL;
//       info("UCamPwc:: did not get an image", ewarning, 0);
//     }
//   }
//   return validAndLocked;
// }

/////////////////////////////////////////////////////

bool UCamPwc::protOpenDevice(bool vcapOnly /*= false*/)
{ // open device and get default parameters
  bool result = true;
  const int MAX_DEV_NAME_LENGTH = 50;
  char device[MAX_DEV_NAME_LENGTH]; // device file name
  //
  // debug
  // printf("UCamPwc::protOpenDevice (vcap only = %s)\n", bool2str(vcapOnly));
  // debug end
  //
  result = not cameraOpen;
  if (result)
  { // old parameters are invalidated
    devInfoValid = false;
    //
    // make video device name
    snprintf(device, MAX_DEV_NAME_LENGTH, "/dev/video%d", devNum);
    // try opening device
    cam_fd = open(device, O_RDONLY);
    result = (cam_fd >= 0);
    if (not result)
      perror(device);
    // debug
    // printf("UCamPwc::protOpenDevice: device=%d opened %s\n", devNum, bool2str(result));
    // debug end
  }
  if (result)
  { // device is open
    cameraOpen = true;
    if (vcapOnly)
    {
      // debug
      //printf("Camera %d open OK\n", devNum);
      // get device capabilities
      result = (ioctl(cam_fd, VIDIOCGCAP, &vcap) == 0);
    }
    else
      result = getDeviceInfo();
    if (not result)
      // failed to get cam data - so not a valid device - close
      cameraOpen = false;
  }
  return result;
}

//////////////////////////////////////////////////////////////////////

bool UCamPwc::deviceExist()
{ // open device and get default parameters
  bool result;
  const int MNL = 50;
  char device[MNL]; // device file name
  //
  lock();
  result = (cam_fd >= 0);
  if (not result)
  { // camera not open, so may not exist
    // make video device name
    snprintf(device, MNL, "/dev/video%d", devNum);
    // try opening device
    cam_fd = open(device, O_RDONLY);
    result = (cam_fd >= 0);
    if (result)
    {  // leave device closed
      result =(close(cam_fd) == 0);
      if (not result)
        perror(vcap.name);
    }
    cam_fd = -1;
  }
  unlock();
  return result;
}

//////////////////////////////////////////////////////////////////////

bool UCamPwc::getDeviceInfo()
{
  bool result = cameraOpen;
  //
  // debug
  // printf("UCamPwc::getDeviceInfo ...\n");
  // debug end
  if (result)
  { // device is open
    cameraOpen = true;
    // get device capabilities
    result = (ioctl(cam_fd, VIDIOCGCAP, &vcap) == 0);
    if (result)
      strncpy(camName, vcap.name, MAX_CAM_DEV_NAME_LENGTH);
    else
      perror("IOCTRL vcap failed");
  }
  if (result)
  { // get device picture capabilities
    //vpic.depth = 24;
    //vpic.palette = VIDEO_PALETTE_RGB24;
    result = (ioctl(cam_fd, VIDIOCGPICT, &vpic) == 0);
    if (result)
    {
      vbrightness = vpic.brightness;
      vcontrast = vpic.contrast;
      vcolour = vpic.colour;
    }
    else
      perror("IOCTL VIDIOCGPICT");
  }
  if (result)
  {
    if (vpic.palette != VIDEO_PALETTE_YUV420P)
    { // not in webcam format - try RGB24
      vpic.depth = 24;
      vpic.palette = VIDEO_PALETTE_RGB24;
      result = (ioctl(cam_fd, VIDIOCSPICT, &vpic) == 0);
      // these are the formats supported p.t.
      if (not result)
        perror("getDeviceInfo::RGB24:VIDIOCSPICT");
    }
  }
  if (result)
  {
    if ((vpic.palette == VIDEO_PALETTE_YUV420P) or
         (vpic.palette == VIDEO_PALETTE_RGB24))
    { // palette is always (or should be) YUV 4:2:0 Planer in 24 bit color
      // or 24 bit RGB

      //printf("Video Palette: YUV 4:2:0 Planer\n");
      //printf("Palette %d \n",vpic.palette);
      //printf("Depth: %d \n",vpic.depth);
      //
      //  get present (default) resolution
      if (ioctl(cam_fd,VIDIOCGWIN, &vwin) != 0)
        perror("IOCTRL VIDIOCGWIN");
      else
      {
        // set size from camera
        frameHeight = vwin.height;
        frameWidth = vwin.width;
        frameRate = (vwin.flags & PWC_FPS_FRMASK) >> PWC_FPS_SHIFT;
        //
        if (frameWidth != oldWidth)
        { // camera parameters has changed
          // set new values
          oldWidth = frameWidth;
          imageSizeChanged(MAX_IMAGE_WIDTH / float(frameWidth));
        }
        // default parameters are OK now
        devInfoValid = true;
      }
    }
  }
  if (result)
  {  // set (current) channel configuration
    if (ioctl(getCamFd(),  VIDIOCGCHAN, &vch) != 0)
      perror("IOCTRL VIDIOCGCHAN");
    // do not care if it fails (not supported)
  }
  //
  /** @todo shared memory temporarily omitted */
  if (false and result)
  { // shared memory buffers
    if (ioctl(getCamFd(), VIDIOCGMBUF, &vmbuf) < 0)
      perror("VIDIOCGMBUF");
    else
    { //get memmap data
      vsharedmem = (char *)mmap(0, vmbuf.size,
                    PROT_READ /*|PROT_WRITE*/,
                    MAP_SHARED,
                    getCamFd(),
                             0);
//      if (int(vsharedmem) == -1)
//      {
//        perror("UCamPwc::protOpenDevice:memm1");
//        vsharedmem = (char *)mmap(0, vmbuf.size,
//                    PROT_READ /*|PROT_WRITE*/,
//                    MAP_SHARED,
//                    getCamFd(),
//                    0);
//      }
      if (vsharedmem == NULL)
      {
        perror("UCamPwc::protOpenDevice:mem2");
        vsharedmem = NULL;
      }
    }
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UCamPwc::setDevice(const int width, int const height,
                                  const int newFrameRate)
{
  bool result = false;
  //
  if (lock())
  {
    if (newFrameRate > 0)
      frameRate = newFrameRate;
    result = protOpenDevice(height, width, frameRate);
    unlock();
  }
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamPwc::protSetDeviceSizeAndSpeed(int toWidth,
                                        int toHeight,int toFramesPerSec)
{ // NB! default values MUST be in frameWidth, frameHeight and frameRate.
  // and camera must be open
  bool result = false;
  bool change = false;
  unsigned int fps = 5;
  char s[MAX_VINFO_SIZE];
  //
  result = devInfoValid and cameraOpen;
  if (result)
  { // get current resolution
    result = (ioctl(cam_fd,VIDIOCGWIN, &vwin) == 0);
  }
  if (result)
  { // extract current settings
    fps = (vwin.flags & PWC_FPS_FRMASK) >> PWC_FPS_SHIFT;
    frameRate = fps;
    frameHeight = vwin.height;
    frameWidth = vwin.width;
  }
  else
  { // failed to get current setting, set size anyhow - debug measure
    frameHeight = toHeight;
    frameWidth = toWidth;
  }  
  // test for need of change
  if (result)
  {
    change = ((toWidth != frameWidth) and (toWidth > 0)) or
        ((toHeight != frameHeight)  and (toHeight > 0)) or
        ((toFramesPerSec != frameRate) and (toFramesPerSec > 0));
  }
  if (change)
  { // set desired size to reqWidth, reqHeight,
    if (toWidth > 0)
      vwin.width = toWidth;
    if (toHeight > 0)
      vwin.height = toHeight;
    //
    // Framerate is upper 16 bits of 32 bit flag value
    // used values bit 22..16 (lower 7 bits).
    if (toFramesPerSec >= 0)
      fps = toFramesPerSec; // with this camera (740k) 5fps is lowest legal value
    fps = fps << PWC_FPS_SHIFT;
    fps = fps | (vwin.flags & ~PWC_FPS_FRMASK);
    if (vpic.palette == VIDEO_PALETTE_YUV420P)
      // philips webcam
      vwin.flags = fps;
    else
      vwin.flags = 0;
    // set image size
    result = (ioctl(cam_fd,VIDIOCSWIN, &vwin) == 0);
    // debug
    result = true;
    // debug end
    // test set resolution
    if (result)
    { // test to se if required resolution was accepted
      result = (ioctl(cam_fd,VIDIOCGWIN, &vwin) == 0);
      // get framerate
      if (vpic.palette == VIDEO_PALETTE_YUV420P)
      { // philips webcam
        fps = (vwin.flags & PWC_FPS_FRMASK) >> PWC_FPS_SHIFT;
        if ((int(fps) != toFramesPerSec) and (toFramesPerSec >= 0))
        { // framerate not as requested
          snprintf(s, MAX_VINFO_SIZE,
                   "Device %d frame rate not %d but %d frames per sec.",
                   devNum, toFramesPerSec, fps);
          info(s, einfo, 0);
        }
      }
      if (((int(vwin.width) != toWidth) and (toWidth > 0)) or
            ((int(vwin.height) != toHeight) and (toHeight > 0)))
      { // size not as requested
        snprintf(s, MAX_VINFO_SIZE,
                 "Device %d frame size not %dx%d but %dx%d.",
                 devNum, toWidth, toHeight, vwin.width, vwin.height);
        info(s, einfo, 0);
      }
      // set new values from camera
      frameRate = fps;
      frameHeight = vwin.height;
      frameWidth = vwin.width;
      if (frameWidth != oldWidth)
      { // camera parameters has changed
        // set new values
        oldWidth = frameWidth;
        imageSizeChanged(MAX_IMAGE_WIDTH / float(frameWidth));
      }
      // camera information is valid
      devInfoValid = true;
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamPwc::openDeviceDefault()
{ // open device acceptiong default parameters
  bool result = false;
  //
  if (lock())
  {
    result = protOpenDevice(0, 0, -1);
    unlock();
  }
  //
  return result;
}


///////////////////////////////////////////////////////

bool UCamPwc::openDevice()
{ // open device acceptiong default parameters
  bool result = false;
  //
  if (lock())
  {
    result = protOpenDevice(frameHeight, frameWidth, frameRate);
    unlock();
  }
  //
  return result;
}

///////////////////////////////////////////////////////


bool UCamPwc::protOpenDevice(const int height,
                             const int width,
                             const int framesPerSec)
{ // open device in required resolution
  bool result = true;
  /*  pthread_attr_t  thAttr;*/
//  UTime t1, t2;
/*
#ifdef CAM_VERBOSE
  char s[MAX_VINFO_SIZE];
#endif
*/
  // debug
//  t1.Now();
  // debug end
  // open device and get default parameters
  if (not cameraOpen)
    result = protOpenDevice();
  // is specific frame size requested - try to opbtain that
  if (result)
    if (((height > 0) and (width > 0)) or (framesPerSec >= 0))
  { // if a height is specified the enforce this
    result = protSetDeviceSizeAndSpeed(width, height, framesPerSec);
  }
  //
  if (result and not threadRunning)
  { // start read thread if not running already.
    stopFrameRead = false;
    startReadThread();
  }
  return result;
}

//////////////////////////////////////////////////////////////

bool UCamPwc::startReadThread()
{
  pthread_attr_t  thAttr;
  int i = 0;
  //
  if (not threadRunning)
  { // start thread to read and timestamp images
    pthread_attr_init(&thAttr);
    //
    stopFrameRead = false;
      // create socket server thread
    if (pthread_create(&thRead, &thAttr, &runPwcCamThread, (void *)this) != 0)
      // report error
      perror(vcap.name);
      // wait for thread to initialize
    while ((not threadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not threadRunning)
    { // failed to start
      stopFrameRead = true;
      printf("Failed to start read thread for device %d : %s\n", devNum, vcap.name);
    }
    pthread_attr_destroy(&thAttr);
  }
  return threadRunning;
}

//////////////////////////////////////////////////////////////

void UCamPwc::stopReadThread(bool andWait)
{
  if (threadRunning)
  {
    stopFrameRead = true;
    if (andWait and threadRunning)
      pthread_join(thRead, NULL);
    // debug
    printf("read thread stopped\n");
  }
}

////////////////////////////////////////////////////////////////////

void UCamPwc::waitAFrame(float frames)
{
  float time;
  //
  if (frameRate > 0)
    time = frames/frameRate;
  else
    time = 0.1;
  //
  Wait(time);
}





