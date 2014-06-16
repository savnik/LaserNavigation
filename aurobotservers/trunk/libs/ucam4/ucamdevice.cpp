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

void * runCamThread(void * camobj)
{ // Start thread here and call thread function
  UCamDevice * cam;
  //
  cam = (UCamDevice *) camobj;
  cam->readFramesThread();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/////////////////////////////////////////////////

UCamDevice::UCamDevice()
{
  int i;
  // devise settings
  //printf("UCamDevice::UCamDevice ...\n");
  //
  devNum = 0;
  cam_fd = -1;
  cameraOpen = false;
  devInfoValid = false;
  frameHeight = 240;
  frameWidth = 320;
  simulated = false;
  oldWidth = 0; // to detect size change
  vpicValid = false;
  // allocate raw image buffers
//  for (i = 0; i < RAW_IMAGEBUFFER_SIZE; i++)
//    raw[i] = new URawImage();
  for (i = 0; i < RAW_IMAGEBUFFER_SIZE; i++)
    rgb[i] = new UImage800();
  // read frames mode.
  for (i = 0; i < MAX_HDR_FRAMES; i++)
  {
    saturationControl[i] = 0;
    vshutterSet[i] = 0x8000;
    vgainSet[i] = 0x1000;
  }
  hdrMode = 0;
  hdrFrames = 0;
  // default camera frame rate
  frameRate = CAMERA_FRAME_RATE;
  //
  threadRunning = false;
  stopFrameRead = true;
  // talk to camera lock
  lockInitCam();
  // image read count set to zero
  imCnt = 0;
  //
  devNumLast = -1; // last succesfully opened device
  //printf("UCamDevice::UCamDevice Initialized\n");
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
}

/////////////////////////////////////////////////

UCamDevice::~UCamDevice()
{ // stop thread if running
  int i;
  // stop thread if running
  stopReadThread(true);
  // ensure camera connection is closed
  setDeviceClosed();
  // deallocate image memory
/*  for (i = 0; i < RAW_IMAGEBUFFER_SIZE; i++)
    if (raw[i] != NULL)
      delete raw[i];*/
  for (i = 0; i < RAW_IMAGEBUFFER_SIZE; i++)
    if (rgb[i] != NULL)
      delete rgb[i];
}


///////////////////////////////////////////////////////

void UCamDevice::readFramesThread(void)
{
  int err = 0;
  int i, got;
//  URawImage * rawimg;
  UImage * rgbimg;
  UTime t1;
  int errCnt = 0;
/*  int hdrCSet = 0;
  int hdrSetOld = 0;
  int hdrYOld1 = 255; // old average intensity last
  int hdrYOld2 = 255; // old average intensity 2 back*/
//  FILE * fm = NULL;
//  const int FN_SIZE = 200;
//  char fn[FN_SIZE];
//  char tfn[20];
  // debug
//  int eVal = 0; // error value in HDR control
  // debug end
  struct video_mmap vmmap;
  //
/*  if (false)
  {
    t1.Now();
    snprintf(fn, FN_SIZE, "%s/intensDev%d_%s.m", "/mnt/ram/results",
                  devNum, t1.getForFilename(tfn, true));
    fm = fopen(fn, "w");
    printf("Writing to logfile FILE * = %x\n", (unsigned int)fm);
    if (fm != NULL)
    {
      fprintf(fm, "%% intensity for device %d\n", devNum);
      fprintf(fm, "%% [Image Number, Average Y, actual control set, "
                  "eValue, gain2, shut2, gain3, shut 3];\n");
      fprintf(fm, "F%d1=[0, 0, 0, 0, 0, 0, 0, 0];\n", devNum);
      fprintf(fm, "F%d2=[0, 0, 0, 0, 0, 0, 0, 0];\n", devNum);
      fprintf(fm, "F%d3=[0, 0, 0, 0, 0, 0, 0, 0];\n", devNum);
      fflush(fm);
      printf("flushed logfile\n");
    }
  }*/
  // clear buffers
/*  for (i = 0; i< RAW_IMAGEBUFFER_SIZE; i++)
  {
    if (raw[i] == NULL)
    {
      err = -1;
    }
    raw[i]->clear();
    raw[i]->camDevice = devNum;
    raw[i]->valid = false;
  }*/
  // clear buffers
  for (i = 0; i< RAW_IMAGEBUFFER_SIZE; i++)
  {
    if (rgb[i] == NULL)
    {
      err = -1;
    }
    rgb[i]->camDevice = devNum;
    rgb[i]->setSize(frameHeight, frameWidth, 3, 8, "RGB");
    rgb[i]->valid = false;
  }
  vmmap.frame = 0;
  vmmap.format = VIDEO_PALETTE_RGB24;
  vmmap.width  = getWidth();
  vmmap.height = getHeight();
  // image read count
  imCnt = 0;
  nextFrame = 0;
  if (err == 0)
  { // buffers available
    threadRunning = true;
    //
    // debug
    //printf("UCamDevice::readFramesThread -- thread started\n");
    // debug end
    printf("UCamDevice:: found %s\n", getCameraName());
    //
    // tap images from device
    while (not stopFrameRead)
    { // read one frame
      //rawimg = raw[nextFrame];
      rgbimg = rgb[nextFrame];
      i = 0;
      while (not rgbimg->tryLock())
      {
        if (stopFrameRead)
        { // dont continue
          err = -1;
          break;
        }
        // try next frame - someone is reading the last
        nextFrame = (nextFrame + 1) % RAW_IMAGEBUFFER_SIZE;
        //rawimg = raw[nextFrame];
        rgbimg = rgb[nextFrame];
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
          if (tryLockCam())
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
                  perror("UCamDevice::readFramesThread:read(...)");
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
                  perror("UCamDevice::readFramesThread:VIDIOCMCAPTURE");
                  errCnt++;
                }
                // set destination image size
                rgbimg->setSizeOnly(frameHeight, frameWidth);
                t1.Now();
                // wait until image is ready
                err = ioctl(getCamFd(), VIDIOCSYNC, &vmmap);
                if ((err != 0) and (errCnt < 10))
                {
                  perror("UCamDevice::readFramesThread:VIDIOCSYNC");
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
            unlockCam();
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
          /*if (hdrMode > 0)
          { // set HDR camera control to next type
            hdrCSet++;
            if (hdrCSet > hdrFrames)
              hdrCSet = 2;
            //Wait(0.020);
            t1.Now();
            setShutter(vshutterSet[hdrCSet]);
            got = (vgainSet[2] + vgainSet[3])/2;
            if (getGain(false) != got)
              setGain(got);
            // debug log settings
            if (fm != NULL)
              fprintf(fm, "Setting to mode %d at %s to gain 04x%04x, shut 04x%04x\n",
                          hdrCSet, t1.getForFilename(tfn, true),
                          got, vshutterSet[hdrCSet]);
            // evaluate the received image
            if (rawimg->stat.evaluateStatistics(rawimg))
            { // statistics are valid
              // Find out wich number this image is
              // either 1 = closed shutter low Y
              //        2 = no saturation  medium Y
              //        3 = shaddow is OK  high Y
              //if ((rawimg->stat.getAvgY() < hdrYOld1) and (rawimg->stat.getAvgY() < hdrYOld2))
              if (rawimg->stat.getAvgY() < hdrYOld1)
              {
                hdrSetOld = 2;
                if (fm != NULL)
                  fprintf(fm, "%% found black image with Y=%d, (old1 = %d, yold2 = %d)\n",
                           rawimg->stat.getAvgY(), hdrYOld1, hdrYOld2);
              }
              else
              { // not black, so increase
                hdrSetOld++;
                if (hdrSetOld >3)
                  hdrSetOld = 2;
              }
              // save old average Y
              hdrYOld2 = hdrYOld1;
              hdrYOld1 = rawimg->stat.getAvgY();
              // debug print
              if (fm != NULL)
                fprintf(fm, "Got image with Y = %d at %s -> set %d\n",
                          rawimg->stat.getAvgY(),
                          rawimg->imgTime.getForFilename(tfn, true),
                          hdrSetOld);
              // debug end
            }
            else
            { // no need for statistics in this mode
              hdrSetOld = 0;
              printf("Should not be here!!!\n");
            }
            // save set number in image meta
            rawimg->hdrSet = hdrSetOld;
            //
          }
          else
            hdrCSet = 0;
          // software saturation control
          if ((hdrMode > 0) or
              (saturationControl[0] > 0) and ((imCnt % 3) == 0))
          { // Calcultae new shutter and gain values
            if (rawimg->valid)
            { // need valid image to do control
              if (not rawimg->stat.isValid())
              rawimg->stat.evaluateStatistics(rawimg);
              //
              doSaturationControl(rawimg, saturationControl, &eVal);
            }
          }
          //
          // debug print
          if (hdrMode > 0)
          {
            if (fm != NULL)
            {
              / *
              fprintf(fm, "%% readFrames: set %d avg %3d, buff %2d, ctrl set %d,"
                  " e: %4d -> (agc,shut) (0x%04x,0x%04x), (0x%04x,0x%04x) %s\n",
                rawimg->hdrSet, rawimg->stat.getAvgY(),
                nextFrame,
                hdrCSet, eVal,
                vgainSet[2], vshutterSet[2],
                vgainSet[3], vshutterSet[3], t1.getForFilename(tfn, true));
              * /
              //snprintf(fn, FN_SIZE, "/mnt/ram/images/raw%06d_%s_%d_%d.bmp",
              //                imCnt, tfn,  rawimg->hdrSet, rawimg->stat.getAvgY());
              //rawimg->saveBMP(fn);
              / *
              fprintf(fm, "F%d%d = [ F%d%d ; %d, %d, %d, %d, %d, %d, %d, %d, %d, %d];\n",
                devNum, rawimg->hdrSet, devNum, rawimg->hdrSet,
                imCnt, rawimg->stat.getAvgY(),
                hdrCSet, eVal,
                vgainSet[1], vshutterSet[1],
                vgainSet[2], vshutterSet[2],
                vgainSet[3], vshutterSet[3]);

      }
      } * /
          // debug end*/
          //
          nextFrame = (nextFrame + 1) % RAW_IMAGEBUFFER_SIZE;
        }
        rgbimg->unlock();
        // send trigger for users of every image
        if (rgbimg->valid);
          gotNewImage(rgbimg);
/*        if (rgbimg->valid);
          gotNewImage(rgbimg);*/
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
    if (rgb[i] != NULL)
      rgb[i]->valid = false;
  }
  //
  // debug
  printf("UCamDevice::readFramesThread -- thread stopped\n");
  // debug end
  //
/*  if (fm != NULL)
    fclose(fm);*/
}

////////////////////////////////////////////////////////

// void UCamDevice::doSaturationControl(URawImage * rawImg,
//                            unsigned int * modes, int * eVal)
// {
//   const int gShutterPos = 35;
//   const int gShutterNeg = 35;
//   const int gGainPos = 15;
//   const int gGainNeg = 35;
//   //const int gUseGainMin = 0x4000;
//   const int gUseGainMax = 0xf000;
//   const int mode2Ref = 220; // target for yMax (maximum value)
//   const int mode3Ref = 70; // (not saturated)/(saturated) relation
//   const int mode4Ref = 150; // average intensity overall
//   const int mode5Ref = 120;  // average for not saturated part
//   // target gain as function of shutter value (4 MSB)
//   // used below gUseGainMax only
//   int gNorm[16] = {    0,      0,      0,      0,
//                     0x50,  0x100,  0x200,  0x300,
//                    0x500,  0x650,  0x800,  0xa00,
//                    0xd00, 0x1000, 0x1300, 0x2000};
//   int gNormal;
//   const int giGainFade = 0x8;
//   int e, eg, n;
//   int set = rawImg->hdrSet;
//   // get current settings for this image
//   int shut = vshutterSet[set];
//   int gain = vgainSet[set];
//   int mode = modes[set];
//   //
//   // mode dependent part
//   switch (mode)
//   {
//     case 3: // saturation ctrl - no saturation
//       e = mode2Ref - rawImg->stat.getMaxY();
//       e = mini(60, maxi(-60, e));
//       break;
//     case 4: // tarhget is 1% in the saturation group
//       e = -(mode3Ref - rawImg->stat.getLowHighRel());
//       // limit result
//       e = mini(60, maxi(-60, e/6));
//       break;
//     case 5: // control on overall average
//       e = mode4Ref - rawImg->stat.getAvgY();
//       break;
//     case 6: // control on overall average
//       e = mode5Ref - rawImg->stat.getAvgLowY();
//       break;
//     default:
//       e = 0;
//       break;
//   }
//   if (eVal != NULL)
//     *eVal = e;
//   // debug
//   //printf("Dev %d, Sat mode %d: stat: high %3d (%3d), low %3d (%3d), E{y} %d, e=%d\n",
//   //    devNum, mode, yHighSum, yHighCnt, yLowSum, yLowCnt, yE, e);
//   // debug end
//   // debug
//   /*
//   printf("Sat dev%d max %04d (%4d), e=%5d, shut=%04x (%04x), gn=%4x, eg=%5d, gain=%04x (%04x)\n",
//           devNum, yMax,yHighCnt, e, vshutterSet, getShutter(true),
//           gNormal, eg, vgainSet, getGain(true));
//   */
//   // debug end
//   // implement
//   if (mode == 2)
//   { // closed shutter
//     gain = 0x0;
//     shut = 0x0;
//   }
//   else if (mode > 2)
//   { // mode 1 is display result only
//     if (shut > gUseGainMax)
//     { // use both shutter and gain
//         if (e > 0)
//         { // too dark
//           shut += gShutterPos/2 * e;
//           gain += gGainPos * e;
//         }
//         else
//         { // too bright
//           shut += gShutterNeg/2 * e;
//           shut += gGainNeg * e;
//         }
//         shut = mini(0xfff0, maxi(0x3800, shut));
//     }
//     else
//     { // use shutter only and
//       // normalize gain
//       if (e > 0)
//         shut += gShutterPos * e;
//       else
//         shut += gShutterNeg * e;
//       shut = mini(0xfff0, maxi(0x3800, shut));
//       n = shut >> 12;
//       gNormal = gNorm[n];
//       // normalize gain k2 * ( k1 - shutter) = target gain
//       //gNormal = (-3*(26000 - vshutterSet))/4;
//       //gNormal = mini(0xfff0, maxi(1, gNormal));
//       eg = gNormal - gain;
//       gain += (eg / giGainFade);
//     }
//     gain = mini(0xc000, maxi(1, gain));
//   }
//   if (set > 0)
//   {
//     if (false and (gain > 0x9000))
//     { // try to avoid mode, where
//       // intensity is from gain alone
//       gain = 0x4000;
//       if (shut > 0xff00)
//         shut = 0xf000;
//     }
//     // save last control value
//     vgainSet[set] = gain;
//     vshutterSet[set] = shut;
//   }
// }


///////////////////////////////////////////////////////

bool UCamDevice::setDeviceNumber(int dev)
{
  bool result = false;
  //

  if (dev >= 0)
  {
    if (dev != devNum)
    {
      if (cameraOpen)
      { // close camera before changing device number
        lockCam();
        protCloseDevice();
        devInfoValid = false;
        unlockCam();
      }
      devNum = dev;
      // set camera info validity to false
      devInfoValid = false;
    }
    result = true;
  }
  //
  return result;
}

///////////////////////////////////////

char * UCamDevice::getCameraName()
{ // return name from camera
  char * result = vcap.name;
  //
  if ((not devInfoValid) and (not simulated))
  { // name not valid (never opened before)
    // try open device with default settings
    if (lockCam())
    { // this open-close just gets default value
      // and nothing more
      protOpenDevice(true);
      if (not cameraOpen)
        result = NULL;
      else
        protCloseDevice();
      unlockCam();
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamDevice::setSizeAndFramerate(
                         const int width,
                         const int height,
                         const int framerate)
{
  bool result = false;
  if (cameraOpen)
    result = openAndSetDevice(width, height, framerate);
  else
  {
    if (lockCam())
    { // just set size and leave to openDevice() to implement
      frameWidth = width;
      frameHeight = height;
      frameRate = framerate;
      imageSizeChanged(MAX_IMAGE_WIDTH/float(width));
      unlockCam();
      result = true;
    }
  }
  return result;
}

///////////////////////////////////////////////////////

bool UCamDevice::protCloseDevice()
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
    // printf("UCamDevice::protCloseDevice: device=%d closed %s\n", devNum, bool2str(result));
    // debug end
  }
  return result;
}

///////////////////////////////////////////////////////

bool UCamDevice::setDeviceClosed()
{
  bool result = true;
  //
  if (lockCam());
  {
    result = protCloseDevice();
    unlockCam();
  }
  //
  return result;
}

/////////////////////////////////////////////////////////

bool UCamDevice::setShutter(int shut)
{ // test features for the camera control
  bool result;
  int err = 0;
  //
  result = cameraOpen and lockCam();
  if (result)
  {
    vshutter = shut;
    vshutterSet[0] = shut;
    if (not simulated)
    {
      err = ioctl(cam_fd, VIDIOCPWCSSHUTTER, &vshutter);
      result = (err == 0);
    }
    unlockCam();
    if (not result)
      ioctlError(err, "setShutter");
  }
  //
  return (err == 0);
}

////////////////////////////////////////////////////////

int UCamDevice::getShutter(bool probe)
{ // get actual shutter value
  /* can not ask for shutter value
  int err;
  int got;
  //
  if (probe and lockCam())
  { // ask camera
    err = ioctl(cam_fd, VIDIOCPWCGSHUTTER, &got);
    if (err == 0)
      shutter = got;
    unlockCam();
    if (err != 0)
      ioctlError(-err, "getShutter");
  }
  */
  return vshutter;
}

////////////////////////////////////////////////////////

//VIDIOCPWCSDYNNOISE / VIDIOCPWCGDYNNOISE

bool UCamDevice::setNoiseRed(int noiseReduction)
{ // test features for the camera control
  bool result;
  int err;
  //
  result = cameraOpen and lockCam();
  if (result)
  {
    vnoiseReduc = noiseReduction;
    if (not simulated)
    {
      err = ioctl(cam_fd, VIDIOCPWCSDYNNOISE, &vnoiseReduc);
      result = (err == 0);
      if (result)
        // ask camera for actual value
        err = ioctl(cam_fd, VIDIOCPWCGDYNNOISE, &vnoiseReduc);
      else
        ioctlError(err, "setNoiseRed");
    }
    unlockCam();
  }
  //
  return result;
}

////////////////////////////////////////////////////////

int UCamDevice::getNoiseRed(bool probe)
{ // get gain value
  int err = 0;
  int got;
  //
  if (probe and not simulated)
    if (lockCam())
    { // ask camera
      if (not simulated)
      {
        err = ioctl(cam_fd, VIDIOCPWCGDYNNOISE, &got);
        if (err == 0)
          vnoiseReduc = got;
      }
      unlockCam();
      if (err != 0)
        ioctlError(-err, "getNoiseRed");
    }
  return vnoiseReduc;
}

////////////////////////////////////////////////////////

bool UCamDevice::setGain(int agc)
{ // test features for the camera control
  bool result;
  int err;
  //
  result = cameraOpen;
  if (result)
    result = lockCam();
  if (result)
  {
    vgain = agc;
    vgainSet[0] = agc;
    if (not simulated)
    {
      err = ioctl(cam_fd, VIDIOCPWCSAGC, &vgain);
      result = (err == 0);
      if (result)
        // ask camera
        err = ioctl(cam_fd, VIDIOCPWCGAGC, &vgain);
      else
        ioctlError(err, "setGain");
    }
    unlockCam();
  }
  //
  return result;
}

////////////////////////////////////////////////////////

int UCamDevice::getGain(bool probe, bool * dataValid)
{ // get gain value
  int err = 0;
  int got;
  //
  if (probe)
    if (lockCam())
    { // ask camera
      err = ioctl(cam_fd, VIDIOCPWCGAGC, &got);
      if (err == 0)
        vgain = got;
      unlockCam();
      if (err != 0)
        ioctlError(err, "getGain");
    }
  if (dataValid != NULL)
    *dataValid = (err == 0);
  return vgain;
}

////////////////////////////////////////////////////////

bool UCamDevice::getWhiteBalance(bool probe, int * red, int * blue, int * mode)
{ // get red and blue gain
  bool result = true;
  int err;
  //
  if (probe)
    if (lockCam())
    { // ask camera
      err = ioctl(cam_fd, VIDIOCPWCGAWB, &vwhite);
      result = (err == 0);
      unlockCam();
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

char * UCamDevice::getWBModeAsString(int mode, char * strBuff)
{
  switch (mode)
  {
    case PWC_WB_AUTO:
      sprintf(strBuff, "auto     ");
      break;
    case PWC_WB_MANUAL:
      sprintf(strBuff, "manual   ");
      break;
    case PWC_WB_INDOOR:
      sprintf(strBuff, "indoor   ");
      break;
    case PWC_WB_OUTDOOR:
      sprintf(strBuff, "outdoor  ");
      break;
    case PWC_WB_FL:
      sprintf(strBuff, "flurocent");
      break;
    default:
      sprintf(strBuff, "unknown  ");
      break;
  }
  return strBuff;
}

////////////////////////////////////////////////////////

int UCamDevice::getWBModeFromString(const char * mode)
{
  int result;
  //
  if (strncasecmp(mode, "auto", 4) == 0)
    result = PWC_WB_AUTO;
  else if (strncasecmp(mode, "manual", 4) == 0)
    result = PWC_WB_MANUAL;
  else if (strncasecmp(mode, "indoor", 4) == 0)
    result = PWC_WB_INDOOR;
  else if (strncasecmp(mode, "outdoor", 4) == 0)
    result = PWC_WB_OUTDOOR;
  else if (strncasecmp(mode, "flurocent", 4) == 0)
    result = PWC_WB_FL;
  else
    result = PWC_WB_AUTO;
  //
  return result;
}

////////////////////////////////////////////////////////

bool UCamDevice::setWhiteBalance(int mode, int red, int blue)
{ // set mode, red and blue gain
  bool result = true;
  int err;
  //
  result =  lockCam();
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
    unlockCam();
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamDevice::isLedOn(bool probe)
{
  if (probe)
    getLed(probe, NULL, NULL);
  return (vled.led_on > 0);
}

///////////////////////////////////////////////////////

bool UCamDevice::setLed(int onTime, int offTime)
{
  bool result = true;
  /*
  int err;
  //
  result =  lockCam();
  if (result)
  { // set values
    vled.led_on = onTime;
    vled.led_off = offTime;
    //
    err = ioctl(cam_fd, VIDIOCPWCSLED, &vled);
    result = (err == 0);
    unlockCam();
    if (not result)
  ioctlError(err, "setLed");
  }
  */
  return result;
}

///////////////////////////////////////////////////////

bool UCamDevice::setDataChannel(int channel)
{
  bool result = true;
  int err;
  //
  lockCam();
  if (result)
  {
    vch.channel = channel;
    err = ioctl(getCamFd(),  VIDIOCGCHAN, &vch);
    if (err != 0)
    {
      perror("UCamDevice::setChannel: VIDIOCGCHAN");
      vch.channel = -1;
    }
    if (err == 0)
    { // set to requested channel
      // vch.channel = 0;
      err = ioctl(getCamFd(),  VIDIOCSCHAN, &vch);
      if (err != 0)
        perror("UCamDevice::setChannel: VIDIOCGCHAN");
    }
    unlockCam();
    result = (err == 0);
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamDevice::getDataChannel()
{
  return vch.channel;
}

////////////////////////////////////////////////////////

bool UCamDevice::getLed(bool probe, int * onTime, int * offTime)
{
  bool result = true;
  /*
  int err;
  //
  if (probe)
    if (lockCam())
    { // get values
      err = ioctl(cam_fd, VIDIOCPWCGLED, &vled);
      result = (err == 0);
      unlockCam();
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

bool UCamDevice::setContour(int value /*= 0x3000*/)
{
  bool result = true;
  int err = 0;
  //
  result =  lockCam();
  if (result)
  { // set values
    vcontour = value;
    if (not simulated)
    {
      err = ioctl(cam_fd, VIDIOCPWCSCONTOUR, &vcontour);
      result = (err == 0);
    }
    unlockCam();
    if (result)
      // ask camera
      err = ioctl(cam_fd, VIDIOCPWCGCONTOUR, &vcontour);
    else
      ioctlError(err, "setContour");
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamDevice::getContour(bool probe)
{
  int err;
  int val;
  //
  if (probe and not simulated)
    if (lockCam())
    { // get values
      err = ioctl(cam_fd, VIDIOCPWCGCONTOUR, &val);
      unlockCam();
      if (err == 0)
        vcontour = val;
      else
        ioctlError(err, "getContour");
    }
  return vcontour;
}

////////////////////////////////////////////////////////

bool UCamDevice::setCompPref(int value /*= 0x0*/)
{
  bool result = true;
  int err = 0;
  //
  result =  lockCam();
  if (result)
  { // set values
    vcompressionPref = value;
    err = ioctl(cam_fd, VIDIOCPWCSCQUAL, &vcompressionPref);
    result = (err == 0);
    unlockCam();
    if (result)
      err = ioctl(cam_fd, VIDIOCPWCGCQUAL, &vcompressionPref);
    else
      ioctlError(err, "setCompression");
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamDevice::getCompPref(bool probe)
{
  int err;
  int val;
  //
  if (probe)
    if (lockCam())
    { // get values
      err = ioctl(cam_fd, VIDIOCPWCGCQUAL, &val);
      unlockCam();
      if (err == 0)
        vcompressionPref = val;
      else
        ioctlError(err, "getCompression");
    }
  return vcompressionPref;
}

////////////////////////////////////////////////////////

bool UCamDevice::setGamma(int gamma)
{
  bool result = true;
  int err = 0;
  //
  result =  lockCam();
  if (result)
  { // set values
    /*
    vpic.whiteness = gamma;
    vpic.contrast = 0x8000;
    vpic.brightness = 0x8000;
    vpic.colour = 0x8000; */
    vpicSet.whiteness = gamma;
    if (not simulated)
    {
      err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
      result = (err == 0);
    }
    unlockCam();
    if (not result)
      ioctlError(err, "setVideoGamma");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamDevice::setContrast(int contrast)
{
  bool result = true;
  int err = 0;
  //
  result =  lockCam();
  if (result)
  { // set values
    vpicSet.contrast = contrast;
    //vpicSet.brightness = 0x8000;
    //vpicSet.colour = 0x8000;
    if (not simulated)
    {
      err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
      result = (err == 0);
    }
    unlockCam();
    if (not result)
      ioctlError(err, "setVideoContrast");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamDevice::setBrightness(int brightness)
{
  bool result = true;
  int err = 0;
  //
  result =  lockCam();
  if (result)
  { // set values
    vpicSet.brightness = brightness;
    //vpicSet.colour = 0x8000;
    if (not simulated)
    {
      err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
      result = (err == 0);
    }
    unlockCam();
    if (not result)
      ioctlError(err, "setVideoBrightness");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

bool UCamDevice::setColour(int colour)
{
  bool result = true;
  int err;
  //
  result =  lockCam();
  if (result)
  { // set values
    // vpicSet.brightness = brightness;
    vpicSet.colour = colour;
    err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
    result = (err == 0);
    unlockCam();
    if (not result)
      ioctlError(err, "setVideoColour");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamDevice::getBrightness(bool probe)
{
  int err;
  //
  if (probe or not vpicValid)
    if (lockCam())
    { // get values
      err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
      unlockCam();
      if (err != 0)
        ioctlError(err, "getVideoBrightness");
      else
        vpicValid = true;
    }
  return vpic.brightness;
}

////////////////////////////////////////////////////////

int UCamDevice::getContrast(bool probe)
{
  int err;
  //
  if (probe or not vpicValid)
    if (lockCam())
    { // get values
      err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
      unlockCam();
      if (err != 0)
        ioctlError(err, "getVideoContrast");
      else
        vpicValid = true;
    }
  return vpic.contrast;
}

////////////////////////////////////////////////////////

int UCamDevice::getColour(bool probe)
{
  int err;
  //
  if ((probe or not vpicValid) and not simulated)
    if (lockCam())
    { // get values
      err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
      unlockCam();
      if (err != 0)
        ioctlError(err, "getVideoColour");
      else
        vpicValid = true;
    }
    return vpic.colour;
}

////////////////////////////////////////////////////////

bool UCamDevice::setVideoCap(int brightness, int contrast, int gamma, int colour)
{
  bool result = true;
  int err;
  //
  result =  lockCam();
  if (result)
  { // set values
    vpicSet.whiteness = gamma;
    vpicSet.contrast = contrast;
    vpicSet.brightness = brightness;
    vpicSet.colour = colour;
    vpicSet.hue = colour;
    err = ioctl(cam_fd, VIDIOCSPICT, &vpicSet);
    result = (err == 0);
    unlockCam();
    if (not result)
      ioctlError(err, "setVideoCap");
    vpicValid = false;
  }
  return result;
}

////////////////////////////////////////////////////////

int UCamDevice::getGamma(bool probe)
{
  int err;
  //
  if (probe or not vpicValid)
    if (lockCam())
    { // get values
      err = ioctl(cam_fd, VIDIOCGPICT, &vpic);
      unlockCam();
      if (err != 0)
        ioctlError(err, "getVideoGamma");
      else
        vpicValid = true;
    }
  return vpic.whiteness;
}

////////////////////////////////////////////////////////

void UCamDevice::ioctlError(int err, const char * prestring)
{
  const int MSL = 200;
  char s[MSL];
  int n;
  //
  n = -EFAULT;
  if (err == n)
  {
    snprintf(s, MSL,
             "IOCTL failed for 'UCamDevice::%s' (faulty parameter or not supported?)", prestring);
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

bool UCamDevice::protGetLockedNewImage(UImage ** prgb)
{ // finds the most recent bufer with a timestamped image
  // and returns it locked.
  // opens the camera is closed.
  bool isOK;
  int bufNum;
  bool validAndLocked = false;
  int i = 0; //, j, n;
  //
  // if device is not open, or open in a wrong mode
  // (re)open in right mode
  isOK = cameraOpen;
  // open camera
  if (not isOK)
    isOK = protOpenDevice(false);
  // start read thread -- if not running already
  if (isOK and not threadRunning)
    isOK = startReadThread();
  // test for available images (image thread got first image)
  if (isOK and (imCnt == 0))
  { // wait for first image
    i = 0;
    while ((imCnt == 0) and (i < 40))
    { // no image is read yet, size is just changed and
      // this can take time (up to 2 seconds).
      Wait(0.05);
      i++;
    }
    isOK = imCnt > 0;
  }
  // debug
  // printf("Get new image cameraOpen=%s thread=%s imageCnt=%d\n",
  //        bool2str(cameraOpen), bool2str(threadRunning), imCnt);
  // debug end
  // get an image
  if (isOK)
  { // open and in right mode, now get image
    i = 0;
    do
    { // get buffer pointer just released by read thread
      bufNum = (nextFrame - 1 + RAW_IMAGEBUFFER_SIZE)
                % RAW_IMAGEBUFFER_SIZE;
      // get latest frame pointer
      //*praw = raw[bufNum];
      *prgb = rgb[bufNum];
      validAndLocked = false;
      if ((*prgb)->tryLock())
      { // lock frame to avoid reuse by read thread
/*        if (vpic.palette == VIDEO_PALETTE_YUV420P)
          validAndLocked = (*praw)->valid;
        else*/
        validAndLocked = (*prgb)->valid;
        if (not validAndLocked)
        { // image not available or already used
          (*prgb)->unlock(); // release
//          n = 0; // count valid frames
/*          for (j = 0; j < RAW_IMAGEBUFFER_SIZE; j++)
          {
            if (((vpic.palette == VIDEO_PALETTE_YUV420P)
                  and (raw[j]->valid))
                  or
                ((vpic.palette == VIDEO_PALETTE_RGB24)
                  and (raw[j]->valid)))
              n++; // count number of usable frames
          }*/
/*          if ((i % 200) == 199) // should not take more than 2 s (5 frames/sec)
          {
            printf("*** UCamDevice:: "
                 "did not get image after %d ms (%d of %d buffers valid)\n",
                 i, n, RAW_IMAGEBUFFER_SIZE);
          }*/
          Wait(0.02); // max 3 waits at 30 frames / sec
        }
      }
      i += 20; // add wait time in miliseconds
      // if not valid, then wait for up to 2 seconds
    } while (i < 2000 and not validAndLocked);
    //
    // image is always in YUV format with
    // 4:1:1 relation between Y, U and V (YUV 4:2:0 Planer)
    if (validAndLocked)
    { // image is valid and locked
      //imageNumber++;
      //result->imageNumber = imageNumber;
    }
    else
    {
      //praw = NULL;
      prgb = NULL;
      info("UCamDevice:: did not get an image", ewarning, 0);
    }
  }
  return validAndLocked;
}

///////////////////////////////////////////////

void UCamDevice::lockInitCam()
{ // init to default (fast) mutes
  // with no error check (not much)
  pthread_mutex_init(&mLock, NULL);
}

////////////////////////////////////////////////

bool UCamDevice::lockCam()
{
  int err;
  bool result;
  //
  err = pthread_mutex_lock(&mLock);
  result = (err == 0);
  //
  return result;
}

////////////////////////////////////////////////

bool UCamDevice::tryLockCam()
{
  int err;
  bool result = false;

  //
  err = pthread_mutex_trylock(&mLock);
  result = (err == 0);
  //
  return result;
}

////////////////////////////////////////////////

void UCamDevice::unlockCam()
{
  pthread_mutex_unlock(&mLock);
}

/////////////////////////////////////////////////////

bool UCamDevice::protOpenDevice(bool vcapOnly /*= false*/)
{ // open device and get default parameters
  bool result = true;
  const int MAX_DEV_NAME_LENGTH = 50;
  char device[MAX_DEV_NAME_LENGTH]; // device file name
  //
  // debug
  // printf("UCamDevice::protOpenDevice (vcap only = %s)\n", bool2str(vcapOnly));
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
    // printf("UCamDevice::protOpenDevice: device=%d opened %s\n", devNum, bool2str(result));
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
  }
  return result;
}

//////////////////////////////////////////////////////////////////////

bool UCamDevice::deviceExist()
{ // open device and get default parameters
  bool result;
  const int MNL = 50;
  char device[MNL]; // device file name
  //
  lockCam();
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
  unlockCam();
  return result;
}

//////////////////////////////////////////////////////////////////////

bool UCamDevice::getDeviceInfo()
{
  bool result = cameraOpen;
  //
  // debug
  // printf("UCamDevice::getDeviceInfo ...\n");
  // debug end
  if (result)
  { // device is open
    cameraOpen = true;
    // debug
    //printf("Camera %d open OK\n", devNum);
    // get device capabilities
    result = (ioctl(cam_fd, VIDIOCGCAP, &vcap) == 0);
    if (not result)
      perror("IOCTRL vcap failed");
  }
  if (result)
  { // get device picture capabilities
    //vpic.depth = 24;
    //vpic.palette = VIDEO_PALETTE_RGB24;
    result = (ioctl(cam_fd, VIDIOCGPICT, &vpic) == 0);
    if (not result)
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
        if (devNum != devNumLast)
        { // device number has changed, there may be a need
          // to load new version of parameters
          imageDeviceChanged(devNum);
          devNumLast = devNum;
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
//        perror("UCamDevice::protOpenDevice:memm1");
//        vsharedmem = (char *)mmap(0, vmbuf.size,
//                    PROT_READ /*|PROT_WRITE*/,
//                    MAP_SHARED,
//                    getCamFd(),
//                    0);
//      }
      if (int(vsharedmem) == -1)
      {
        perror("UCamDevice::protOpenDevice:mem2");
        vsharedmem = NULL;
      }
    }
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UCamDevice::openAndSetDevice(const int width, int const height,
                                  const int newFrameRate,
                                  bool readThreadNormal /* = true */)
{
  bool result = false;
  //
  if(lockCam())
  {
    if (newFrameRate > 0)
      frameRate = newFrameRate;
    //if (not isCameraOpen())
    result = protOpenDevice(height, width, frameRate, readThreadNormal);
    //else
    //  result = protSetDeviceSizeAndSpeed(width, height, frameRate);
    unlockCam();
  }
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamDevice::protSetDeviceSizeAndSpeed(unsigned int toWidth,
                              unsigned int toHeight,int toFramesPerSec)
{ // NB! default values MUST be in frameWidth, frameHeight and frameRate.
  // and camera must be open
  bool result = false;
  bool change = false;
  unsigned int fps = 5;
  char s[MAX_VINFO_SIZE];
  //
  result = devInfoValid and cameraOpen;
  if (result)
    // get current resolution
    result = (ioctl(cam_fd,VIDIOCGWIN, &vwin) == 0);
  if (result)
  { // extract current settings
    fps = (vwin.flags & PWC_FPS_FRMASK) >> PWC_FPS_SHIFT;
    frameRate = fps;
    frameHeight = vwin.height;
    frameWidth = vwin.width;
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
      if (((vwin.width != toWidth) and (toWidth > 0)) or
          ((vwin.height != toHeight) and (toHeight > 0)))
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

bool UCamDevice::openDeviceDefault(bool readThreadNormal /* = true */)
{ // open device acceptiong default parameters
  bool result = false;
  //
  if (lockCam())
  {
    result = protOpenDevice(0, 0, -1, readThreadNormal);
    unlockCam();
  }
  //
  return result;
}


///////////////////////////////////////////////////////

bool UCamDevice::openDevice(bool readThreadNormal /* = true */)
{ // open device acceptiong default parameters
  bool result = false;
  //
  if (lockCam())
  {
    result = protOpenDevice(frameHeight, frameWidth, frameRate, readThreadNormal);
    unlockCam();
  }
  //
  return result;
}

///////////////////////////////////////////////////////


bool UCamDevice::protOpenDevice(const unsigned int height,
                        const unsigned int width,
                        const int framesPerSec,
                        bool readThreadNormal)
{ // open device in required resolution
  bool result = true;
/*  pthread_attr_t  thAttr;*/
  UTime t1, t2;
/*
#ifdef CAM_VERBOSE
  char s[MAX_VINFO_SIZE];
#endif
*/
  //
  //printf("UCamDevice::protOpenDevice start\n");
  t1.Now();
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
/*  if (cameraOpen and frameReadStopped and readThreadNormal)
  { // start thread to read and timestamp images
    pthread_attr_init(&thAttr);
    //
    stopFrameRead = false;
    // create socket server thread
    result = (pthread_create(&thRead, &thAttr, &runCamThread, (void *)this) == 0);
    // wait for thread to initialize
    Wait(0.05);
  }*/
  //
  //printf("UCamDevice::protOpenDevice end\n");
  return result;
}

//////////////////////////////////////////////////////////////

bool UCamDevice::startReadThread()
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
    if (pthread_create(&thRead, &thAttr, &runCamThread, (void *)this) != 0)
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

void UCamDevice::stopReadThread(bool andWait)
{
  if (threadRunning)
  {
    stopFrameRead = true;
    if (andWait and threadRunning)
      pthread_join(thRead, NULL);
  }
}

//////////////////////////////////////////////////////////////

void UCamDevice::imageSizeChanged(double /*iResFactor*/)
{ // is probably uverwritten at a higher level
  // to change matrices etc.
  // pixSize = iResFactor;
}

void UCamDevice::imageDeviceChanged(int /*newDevice*/)
{ // called when camera is opened with a new
  // device number
}

////////////////////////////////////////////////////////////////////

void UCamDevice::setHdrMode(int mode)
{
  //printf("UCamDevice::setHdrMode HDR mode %d a\n", mode);
  if (MAX_HDR_FRAMES <= 3)
  {
    printf("*** UCamDevice::setHdrMode MAX_HDR_FRAMES must be > 3\n");
    mode = 0;
  }
  switch (mode)
  {  // prepare tye hdr mode shift
    case 0:
      hdrFrames = 0;
      hdrMode = 0;
      break;
    case 1:
    case 2:
    case 3:
      //printf("UCamDevice::setHdrMode HDR mode %d b\n", mode);
      hdrFrames = 3;
      //printf("UCamDevice::setHdrMode HDR mode %d c\n", mode);
      saturationControl[1] = 2; // shutter shut
      saturationControl[2] = 4; // ~1% in saturation
      saturationControl[3] = 6; // average of low part ~90
      hdrMode = mode;
      // debug
      /*
      printf("UCamDevice::setHdrMode %d frames modes: %d (%d,%d,%d)\n",
                 hdrFrames,
                 hdrMode,
                 saturationControl[1],
                 saturationControl[2],
                 saturationControl[3]);
      */
      // debug end
      break;
    default:
      hdrFrames = 0;
      hdrMode = 0;
      printf("UCamDevice::setHdrMode unknown HDR mode %d)\n",
                 hdrMode);
      break;
  }
}

////////////////////////////////////////////////////////////////////

void UCamDevice::waitAFrame(float frames)
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


void UCamDevice::gotNewImage(UImage * rgb)
{
  // no use at this level
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////





