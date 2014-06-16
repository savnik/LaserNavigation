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

#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

#include <ugen4/uimage2.h>
#include <urob4/uvarpool.h>

#include "ucamdevguppy.h"
//#define USE_GUPPY
#ifdef USE_GUPPY
#include <dc1394/control.h>
#include <dc1394/types.h>
#endif

UCamDevGuppy::UCamDevGuppy()
 : UCamDevBase()
{
#ifdef USE_GUPPY
  //
  camHandle = NULL;
  devPlatform = NULL;
  camType = CAM_DEV_IEEE1394;
  stopFrameRead = true;
  threadRunning = false;
  fGain.id = (dc1394feature_t) 0;
  fShutter.id = (dc1394feature_t) 0;
  fWhite.id = (dc1394feature_t) 0;
  fExposure.id = (dc1394feature_t) 0;
  fTrigger.id = (dc1394feature_t) 0;
  fTriggerDelay.id = (dc1394feature_t) 0;
  varImageNumber = NULL;
  varExternalTrig = NULL;
  varGain = NULL;
  varShutter = NULL;
  varExposure = NULL;
  varWhite = NULL;
  varOpen = NULL;
  varSettingsChanged = NULL;
  varISObw = NULL;
  varPacketSize = NULL;
  isoBW = -1;
#endif
}

///////////////////////////////////////////

UCamDevGuppy::~UCamDevGuppy()
{
#ifdef USE_GUPPY
  // stop read thread
  stop(true);
  closeDevice();
  if (camHandle != NULL)
  {
    dc1394_capture_stop(camHandle);
    dc1394_camera_free(camHandle);
    printf("UCamDevGuppy::~UCamDevGuppy closed camera handle\n");
    camHandle = NULL;
  }
  if (devPlatform != NULL)
  {
    dc1394_free(devPlatform);
    devPlatform = NULL;
    //printf("UCamDevGuppy::~UCamDevGuppy closed device handle\n");
  }
#endif
}

#ifdef USE_GUPPY

///////////////////////////////////////////

int UCamDevGuppy::getIeee1394PortCnt()
{
  const int MAX_IEEE1394_PORTS = 10;
  struct raw1394_portinfo ports[MAX_IEEE1394_PORTS];
  int portCnt = 0;
  raw1394handle_t handle;
  //
  handle = raw1394_new_handle();
  if (handle==NULL)
  {
    fprintf( stderr, "Unable to aquire a raw1394 handle - no camera or no rights?\n");
/*        "Please check \n"
            "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
            "  - if you have read/write access to /dev/raw1394\n\n");*/
  }
  /* get the number of ports (cards) and write info (up to 10) into 'ports' */
  if (handle != NULL)
  {
    portCnt = raw1394_get_port_info(handle, ports, MAX_IEEE1394_PORTS);
    raw1394_destroy_handle(handle);
  }
  /* look across all ports for cameras */
  return portCnt;
}

///////////////////////////////////////////

int UCamDevGuppy::getDc1394CamCnt()
{
  int camCnt = 0;
  dc1394camera_list_t * list = NULL;
  dc1394error_t err = DC1394_SUCCESS;
  //
  if (devPlatform == NULL)
  {
    devPlatform = dc1394_new ();
    if (devPlatform == NULL)
    {
      printf("UCamDevGuppy::getDc1394CamCnt failed to get access to /dev/raw1394\n");
    }
  }
  //
  if (devPlatform != NULL)
  {
    err = dc1394_camera_enumerate(devPlatform, &list);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::getDc1394CamCnt failed to enumerate cameras\n");
      // DC1394_ERR_RTN(err,"Failed to enumerate cameras");
  }
  if (devPlatform != NULL and err == DC1394_SUCCESS)
  {
/*    printf("UCamDevGuppy::getDc1394CamCnt found %d cameras\n", list->num);
    for (int i = 0; i < (int)list->num; i++)
      printf("  - %d ID: %llx\n", i, list->ids[i].guid);*/
    if (list->num == 0)
    {
        dc1394_log_error("No cameras found");
        return 1;
    }
    camCnt = list->num;
  }
  if (list != NULL)
    dc1394_camera_free_list(list);
  return camCnt;
}


////////////////////////////////////////////

bool UCamDevGuppy::setCamDeviceNode(int camNum)
{ // this is to test that the camera exist and is working
  // and is not called after camera detect is finished (except for >> camsget all)
  bool result = false;
  //dc1394_t * d;
  dc1394camera_list_t * list;
  dc1394error_t err = DC1394_SUCCESS;
  dc1394video_modes_t modes;
//  dc1394featureset_t allFeatures;
  //
  if (devPlatform == NULL)
  {
    devPlatform = dc1394_new ();
    if (devPlatform == NULL)
      printf("UCamDevGuppy::setCamDeviceNode failed to access /dev/fw0\n");
    cameraOpen = false;
  }
  if (cameraOpen)
    closeDevice();
  //
  if (devPlatform != NULL)
    err = dc1394_camera_enumerate(devPlatform, &list);
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::setCamDeviceNode failed to enumerate cameras\n");
    // DC1394_ERR_RTN(err,"Failed to enumerate cameras");
  if (err == DC1394_SUCCESS and list->num > 0)
  { // get handle to camera bus (port)
    camHandle = dc1394_camera_new(devPlatform, list->ids[camNum].guid);
    if (camHandle == NULL)
      printf("UCamDevGuppy::setCamDeviceNode failed to initialize camera %d with GUID=%llx\n", camNum, list->ids[camNum].guid);
    if (camHandle != NULL)
    { // handle is OK,
      // debug
      //dc1394_camera_print_info(camHandle, stdout);
      //dc1394_feature_get_all(camHandle, &allFeatures);
      //dc1394_feature_print_all(&allFeatures, stdout);
      // debug end
      // printf("Device is a %s %s\n", camHandle->vendor, camHandle->model);
      snprintf(camName, MAX_CAM_DEV_NAME_LENGTH, "%s %llx", camHandle->model, camHandle->guid);
      // release any existing iso channels
      release_iso_and_bw();
      // get camera get mode to use
      err = dc1394_video_get_supported_modes(camHandle, &modes);
      if (err == DC1394_SUCCESS)
      {
/*        for (int i=0; i < (int)modes.num; i++)
        {
          selected_mode = modes.modes[i];
          print_mode_info(camHandle, selected_mode);
        }*/
        selected_mode = modes.modes[modes.num - 1];
        print_mode_info(camHandle, selected_mode);
        result = true;
      }
//       dc1394_format7_set_packet_size()
//       Arguments:
//       dc1394camera_t  :       *camera :       A pointer to an initialized camera structure
//       dc1394video_mode_t      :       mode    :       The format_7 video mode
//       uint_t  :       packet_bytes    :       the number of bytes per packet
//       Returns:   dc1394error_t (error code)
//       // set trigger mode
// max size is determined by iso_speed:
// The IEEE 1394 specification imposes the following limits on the payload size of isochronous packets:
// Speed   Cycle period    Max. packet size        Max. bandwidth per ISO channel
// [Mb/s]  [µs]    [B]     [MB/s]  [MiB/s]
// S100    125     1024    8.192   7.8125
// S200    125     2048    16.384  15.6250   --- used for guppy to allow 2cameras on one s400 connection
// S400    125     4096    32.768  31.2500
// S800    125     8192    65.536  62.5000
// S1600   125     16384   131.072 125.0000
// S3200   125     32768   262.144 250.0000
//       if (varPacketSize != NULL)
//         // try this after camera is detected only - else just use default (about 400)
//         err = dc1394_format7_set_packet_size(camHandle, selected_mode, varPacketSize->getInt());
      //
      err = dc1394_external_trigger_set_mode(camHandle, DC1394_TRIGGER_MODE_0);
      if (err != DC1394_SUCCESS)
        printf("Could not set trigger mode\n");
//      err = dc1394_feature_set_power(camHandle, DC1394_FEATURE_TRIGGER, DC1394_ON);
      err = dc1394_feature_set_power(camHandle, DC1394_FEATURE_TRIGGER, DC1394_OFF);
      if (err != DC1394_SUCCESS)
        printf("Could not set trigger on-off\n");
      err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_EXPOSURE, 100);
      if (err != DC1394_SUCCESS)
        printf("Could not set exposure\n");
      err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_BRIGHTNESS, 200);
      if (err != DC1394_SUCCESS)
        printf("Could not set brightness\n");
//        err=dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_200);
//       if (err != DC1394_SUCCESS)
//       {
//         printf("UCamDevGuppy::setCamDeviceNode: Could not set iso speed DC1394_ISO_SPEED_200\n");
//       }
/*      else
        printf("UCamDevGuppy::setCamDeviceNode: set iso speed to DC1394_ISO_SPEED_100\n");*/
      result &= (err == DC1394_SUCCESS);

      //dc1394_feature_get_all(camHandle, &allFeatures);
      //dc1394_feature_print_all(&allFeatures, stdout);
    }
  }
  dc1394_camera_free_list(list);
  //dc1394_free(d);
  return result;
}

////////////////////////////////////////////////////////////////

void UCamDevGuppy::release_iso_and_bw()
{
  uint32_t val;
  dc1394error_t err = DC1394_SUCCESS;

  err = dc1394_video_get_bandwidth_usage(camHandle, &val);
  if (err == DC1394_SUCCESS)
  {
    err = dc1394_iso_release_bandwidth(camHandle, val);
    //printf("UCamDevGuppy::release_iso_and_bw: released (%s) %d bytes/sec? of bandwidth for device %d\n",
    //       bool2str(err == DC1394_SUCCESS), val, devNum);
  }

  err = dc1394_video_get_iso_channel(camHandle, &val);
  if (err == DC1394_SUCCESS)
  {
    printf(" - A failure to free iso channel is OK\n");
    err = dc1394_iso_release_channel(camHandle, val);
    //printf("UCamDevGuppy::release_iso_and_bw: released (%s) iso channel (value %d) for device %d\n",
    //       bool2str(err == DC1394_SUCCESS), val, devNum);
  }
}

////////////////////////////////////////////

void UCamDevGuppy::print_format(uint32_t format)
{
#define print_case(A) case A: printf(#A ""); break;

    switch( format ) {
        print_case(DC1394_VIDEO_MODE_160x120_YUV444);
        print_case(DC1394_VIDEO_MODE_320x240_YUV422);
        print_case(DC1394_VIDEO_MODE_640x480_YUV411);
        print_case(DC1394_VIDEO_MODE_640x480_YUV422);
        print_case(DC1394_VIDEO_MODE_640x480_RGB8);
        print_case(DC1394_VIDEO_MODE_640x480_MONO8);
        print_case(DC1394_VIDEO_MODE_640x480_MONO16);
        print_case(DC1394_VIDEO_MODE_800x600_YUV422);
        print_case(DC1394_VIDEO_MODE_800x600_RGB8);
        print_case(DC1394_VIDEO_MODE_800x600_MONO8);
        print_case(DC1394_VIDEO_MODE_1024x768_YUV422);
        print_case(DC1394_VIDEO_MODE_1024x768_RGB8);
        print_case(DC1394_VIDEO_MODE_1024x768_MONO8);
        print_case(DC1394_VIDEO_MODE_800x600_MONO16);
        print_case(DC1394_VIDEO_MODE_1024x768_MONO16);
        print_case(DC1394_VIDEO_MODE_1280x960_YUV422);
        print_case(DC1394_VIDEO_MODE_1280x960_RGB8);
        print_case(DC1394_VIDEO_MODE_1280x960_MONO8);
        print_case(DC1394_VIDEO_MODE_1600x1200_YUV422);
        print_case(DC1394_VIDEO_MODE_1600x1200_RGB8);
        print_case(DC1394_VIDEO_MODE_1600x1200_MONO8);
        print_case(DC1394_VIDEO_MODE_1280x960_MONO16);
        print_case(DC1394_VIDEO_MODE_1600x1200_MONO16);
        print_case(DC1394_VIDEO_MODE_FORMAT7_0);
        print_case(DC1394_VIDEO_MODE_FORMAT7_1);
        print_case(DC1394_VIDEO_MODE_FORMAT7_2);
        print_case(DC1394_VIDEO_MODE_FORMAT7_3);
        print_case(DC1394_VIDEO_MODE_FORMAT7_4);
        print_case(DC1394_VIDEO_MODE_FORMAT7_5);
        print_case(DC1394_VIDEO_MODE_FORMAT7_6);
        print_case(DC1394_VIDEO_MODE_FORMAT7_7);

    default:
        dc1394_log_error("Unknown format\n");
        break;
    }

}

////////////////////////////////////////////////////

void UCamDevGuppy::print_mode_info( dc1394camera_t *camera , uint32_t mode )
{
  int j;
  dc1394video_mode_t vmode = (dc1394video_mode_t)mode;

/*  printf("Mode: ");
  print_format(mode);
  printf("\n");*/
  if (vmode < DC1394_VIDEO_MODE_FORMAT7_MIN)
  { // format 7 has no framerate?
    dc1394framerates_t framerates;
    dc1394error_t err;
    err=dc1394_video_get_supported_framerates(camera, vmode, &framerates);
    if (err != 0)
      printf("UCamDevGuppy::print_mode_info failed to get framerate\n");
    else
    {
      printf("Frame Rates:\n");
      for( j = 0; j < (int)framerates.num; j++ )
      {
        dc1394framerate_t rate = framerates.framerates[j];
        float f_rate;
        dc1394_framerate_as_float(rate,&f_rate);
        printf("  [%d] rate = %f\n",j,f_rate );
      }
    }
  }
}



////////////////////////////////////////////////////////

bool UCamDevGuppy::getSingleImage(UImage * destination)
{
  bool result;
  int bytes, n;
  dc1394error_t err;
  UImage * img = destination;
  unsigned int width, height, depth;
  dc1394video_frame_t *frame=NULL;
  uint64_t ts, tu;
  UTime t;
  //
  /*-----------------------------------------------------------------------
   *  capture one frame
   *-----------------------------------------------------------------------*/
  //err = dc1394_capture_dequeue(camHandle, DC1394_CAPTURE_POLICY_WAIT, &frame);
  err = dc1394_capture_dequeue(camHandle, DC1394_CAPTURE_POLICY_POLL, &frame);
  result = err == DC1394_SUCCESS and frame != NULL;
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::getSingleImage: Could not poll for an image frame\n");

  if (result)
  { // frame time
    ts = frame->timestamp / 1000000;
    tu = frame->timestamp - ts * 1000000;
    t.setTime(ts, tu);
    // debug
/*    if (fTrigger.is_on == DC1394_ON and fTrigger.available == DC1394_TRUE)
    {
      t.getTimeAsString(s, true);
      printf("device %d got frame at frame time %s  %lu.%06lu serial %lu\n", devNum, s,
            t.getSec(), t.getMicrosec(), imageNumber);
    }*/
    // debug end
    //
    if (img != NULL and img->tryLock())
    {
      width = frame->size[0];
      height = frame->size[1];
      depth = frame->data_depth; // mono8 => 8 bit depth
      // frame->color_coding == DC1394_COLOR_CODING_MONO8
      // frame->color_filter == DC1394_COLOR_CODING_RGGB
      // frame->little_endian == DC1394_FALSE
      //dc1394_get_image_size_from_video_mode(camHandle, selected_mode, &width, &height);
      img->setSize(height, width, 1, depth, "BGGR");
      n = height * width;
      n = frame->image_bytes;
      if (varPacketSize != NULL)
        varPacketSize->setInt(frame->packet_size);
      bytes = img->getBufferSize();
      if (n <= bytes)
      {
        memcpy(img->getData(), frame->image, n);
      }
      img->camDevice = getDeviceNumber();
      img->cam = NULL;
      img->imgTime = t;
      // debug
/*      img->imgTime.getTimeAsString(s, true);
      printf("frame time %s  %lu.%06lu serial %lu\n", s,
            img->imgTime.getSec(), img->imgTime.getMicrosec(), imageNumber);*/
      // debug end
      //
      img->imageNumber = imageNumber;
      img->valid = true;
      while (((int)img->getHeight() > frameHeight) and (frameHeight > 25))
      { // reduce size
        img->toHalf();
      }
      img->unlock();
    }
    else
    { // just increase image number
      //imageNumber++;
      Wait(0.0011);
    }
    // put buffer back in camera queue
    err = dc1394_capture_enqueue(camHandle, frame);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::getSingleImage: could not return image buffer (dev %d)\n", devNum);
  }
  return result;
}

/////////////////////////////////////////////////////////////


bool UCamDevGuppy::startIsoTransmission()
{
  dc1394error_t err;
  /*-----------------------------------------------------------------------
    *  setup capture
    *-----------------------------------------------------------------------*/
  //printf("Starting video-mode, capture and video transmission (dev %d)\n", devNum);
  if (varISObw != NULL)
  {
    if (isoBW != varISObw->getInt())
    {
      isoBW = varISObw->getInt();
      if (not (isoBW == 100 or isoBW == 200 or isoBW==400 or isoBW == 800 or isoBW == 1600 or isoBW == 3200))
      {
        if (isoBW < 150)
          isoBW = 100;
        else if (isoBW < 300)
          isoBW = 200;
        else if (isoBW < 600)
          isoBW = 400;
        else if (isoBW < 1200)
          isoBW = 800;
        else if (isoBW < 2400)
          isoBW = 1600;
        else
          isoBW = 3200;
      }
      varISObw->setInt(isoBW);
      switch (isoBW)
      {
        case 100: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_100); break;
        case 200: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_200); break;
        case 400: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_400); break;
        case 800: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_800); break;
        case 1600: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_1600); break;
        case 3200: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_3200); break;
      }
    }
  }
  // max size is determined by iso_speed:
// The IEEE 1394 specification imposes the following limits on the payload size of isochronous packets:
// Speed, Cycle, Max. packet size, Max. bandwidth per ISO channel
// [Mb/s]  [µs]    [B]     [MB/s]  [MiB/s]
// S100    125     1024    8.192   7.8125
// S200    125     2048    16.384  15.6250   --- used s200 for guppy to allow 2cameras on one s400 connection
// S400    125     4096    32.768  31.2500
// S800    125     8192    65.536  62.5000
// S1600   125     16384   131.072 125.0000
// S3200   125     32768   262.144 250.0000
  if (varPacketSize != NULL)
  {
    err = dc1394_format7_set_packet_size(camHandle, selected_mode, varPacketSize->getInt());
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::startIsoTransmission: Could not set packet size to %d\n", varPacketSize->getInt());
  }
  err = dc1394_video_set_mode(camHandle, selected_mode);
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::startIsoTransmission: Could not set video mode FORMAT_7_0\n");


/*  if (err == DC1394_SUCCESS)
  {
    err = dc1394_video_set_framerate(camHandle, DC1394_FRAMERATE_7_5);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::startIsoTransmission: Could not set framerate to DC1394_FRAMERATE_7_5\n");
  }*/
  if (err == DC1394_SUCCESS)
  {
    err = dc1394_capture_setup(camHandle, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err != DC1394_SUCCESS and err != DC1394_CAPTURE_IS_RUNNING)
    {
      printf("UCamDevGuppy::startIsoTransmission: Could not setup camera\n");
      // DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");
    }
  }
  if (err == DC1394_SUCCESS)
  {
    /*-----------------------------------------------------------------------
      *  have the camera start sending us data
      *-----------------------------------------------------------------------*/
    err = dc1394_video_set_transmission(camHandle, DC1394_ON);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::startIsoTransmission: Could not start camera iso transmission\n");
    cameraOpen = (err == DC1394_SUCCESS);
  }
  else if (err == DC1394_CAPTURE_IS_RUNNING)
    cameraOpen = true;
  return cameraOpen;
}

//////////////////////////////////////////////////////////////

bool UCamDevGuppy::stopIsoTransmission()
{
  dc1394error_t err;
  bool result = true;
  //
  if (cameraOpen)
  {
    lock();
    cameraOpen = false;
    if (varOpen != NULL)
      varOpen->setBool(false, 0, false);
    // wait for any last image to be captured
    Wait(0.2);
    /*Step 6: Stop sending data*/
    dc1394_capture_stop(camHandle);
    err = dc1394_video_set_transmission(camHandle,DC1394_OFF);
    result = err == DC1394_SUCCESS;
    if (not result)
      printf("UCamDevGuppy::stopIsoTransmission: could not stop the camera\n");
    unlock();
  }
  else
    printf("UCamDevGuppy::stopIsoTransmission(): camera is closed already\n");

  return result;
}

///////////////////////////////////////////////////////////

bool UCamDevGuppy::setGain(int agc)
{
  vars->lock();
  if (agc < 0)
    varGain->setValued(1, 3, false);
  else
  {
    varGain->setInt(0, 3, false);
    varGain->setInt(agc, 0, false);
  }
  varSettingsChanged->setBool(true, 0, false);
  vars->unlock();
  return isCameraOpen();
}

/////////////////////////////////////////////////////////

bool UCamDevGuppy::setGainRaw(int agc)
{
//   int v;
//   dc1394_feature_info * feature;
  dc1394error_t err;
  //
  if (agc == -1)
  {
    if (fGain.id == 0)
      getGainRaw();
    if (fGain.current_mode != DC1394_FEATURE_MODE_AUTO)
    {
      err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
      if (err != DC1394_SUCCESS)
        printf("UCamDevGuppy::setGain: failed to set gain to auto\n");
      err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_GAIN, &fGain.current_mode);
    }
    else
      err = DC1394_SUCCESS;
  }
  else
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_GAIN, agc);
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::setGain: to %d failed\n", agc);
   return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////

int UCamDevGuppy::getGain(bool probe, bool * dataValid, bool * isOnAuto)
{
  int result = 0;
  bool valid = true;
  bool locked = vars != NULL;
  //
  if (locked)
    vars->lock();
  if (probe)
  {
    valid = getGainRaw();
    // debug
    // dc1394_feature_print(&fGain, stdout);
    // debug end
  }
  else if (varGain != NULL)
    valid = varGain->getBool(4);
  result = fGain.value;
  if (dataValid != NULL)
    *dataValid = valid;
  if (isOnAuto != NULL)
    *isOnAuto = fGain.current_mode == DC1394_FEATURE_MODE_AUTO;
  if (locked)
    vars->unlock();
  return result;
}

//////////////////////////////////////

bool UCamDevGuppy::getGainRaw()
{
  dc1394error_t err;
  //
  fGain.id = DC1394_FEATURE_GAIN;
  err = dc1394_feature_get(camHandle, &fGain);
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::getGain: Failed to get gain feature info\n");
  //
  if (varGain != NULL)
  {
    varGain->setInt(fGain.value, 0);
    varGain->setInt(fGain.min, 1);
    varGain->setInt(fGain.max, 2);
    varGain->setBool(fGain.current_mode == DC1394_FEATURE_MODE_AUTO, 3);
    varGain->setBool(err == DC1394_SUCCESS, 4, true);
  }
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////

bool UCamDevGuppy::setShutter(int value)
{
  vars->lock();
  if (value == -1)
    varShutter->setBool(true, 3);
  else
  {
    varShutter->setBool(false, 3);
    varShutter->setInt(value, 0);
  }
  varSettingsChanged->setBool(true);
  vars->unlock();
  return isCameraOpen();
}

/////////////////////////////////////////////////

bool UCamDevGuppy::setShutterRaw(int value)
{
  dc1394error_t err;
  if (value == -1)
  {
    err = DC1394_FAILURE;
    if (fShutter.id == 0)
      getShutterRaw();
    if (fShutter.current_mode != DC1394_FEATURE_MODE_AUTO)
    {
      err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
      err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_SHUTTER, &fShutter.current_mode);
    }
  }
  else
  {
    if (fShutter.current_mode == DC1394_FEATURE_MODE_AUTO)
    {
      err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
      if (err != DC1394_SUCCESS)
        printf("UCamDevGuppy::setShutter: failed to change to manuel mode\n");
    }
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_SHUTTER, value);
    err = dc1394_feature_get_value(camHandle, DC1394_FEATURE_SHUTTER, &fShutter.value);
  }
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::setGain: to %d failed\n", value);
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////

int UCamDevGuppy::getShutter(bool probe, bool * dataValid, bool * isOnAuto)
{
  int result = 0;
  bool valid = true;
  bool locked = vars != NULL;
  if (locked)
    vars->lock();
  if (probe)
  {
    valid = getShutterRaw();
    // debug
    //dc1394_feature_print(&fShutter, stdout);
    // debug end
  }
  else if (varShutter != NULL)
    valid = varShutter->getBool(4);
  result = fShutter.value;
  if (dataValid != NULL)
    *dataValid = valid;
  if (isOnAuto != NULL)
    *isOnAuto = fShutter.current_mode == DC1394_FEATURE_MODE_AUTO;
  if (locked)
    vars->unlock();
  return result;
}

/////////////////////////////////////////////////

bool UCamDevGuppy::getShutterRaw()
{
  dc1394error_t err;
  //
  fShutter.id = DC1394_FEATURE_SHUTTER;
  err = dc1394_feature_get(camHandle, &fShutter);
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::getGain: Failed to get gain feature info\n");
  //
  if (varShutter != NULL)
  {
    varShutter->setInt(fShutter.value, 0);
    varShutter->setInt(fShutter.min, 1);
    varShutter->setInt(fShutter.max, 2);
    varShutter->setBool(fShutter.current_mode == DC1394_FEATURE_MODE_AUTO, 3);
    varShutter->setBool(err == DC1394_SUCCESS, 4, true);
  }
  return err == DC1394_SUCCESS;
}


////////////////////////////////////////////////////////

bool UCamDevGuppy::getWhiteBalance(bool probe, int * red, int * blue, int * mode)
{ // get red and blue gain, and mode (4=auto, 3=manual)
  bool result;
  bool locked = vars != NULL;
  //
  if (locked)
    vars->lock();
  if (probe)
  {
    result = getWhiteBalanceRaw();
    // debug
    //dc1394_feature_print(&fWhite, stdout);
    // debug end
  }
  else if (varWhite != NULL)
    result = varWhite->getInt(5);
  else
    result = true;
  if (red != NULL)
    *red = fWhite.RV_value;
  if (blue != NULL)
    *blue = fWhite.BU_value;
  if (mode != NULL)
  { // mode is always manual, but may be set on a one-shot basis
    if (varWhite != NULL)
      *mode = varWhite->getInt(0);
    else
      *mode = 3;
  }
  if (locked)
    vars->unlock();
  return result;
}

////////////////////////////////////////////////////////

bool UCamDevGuppy::getWhiteBalanceRaw()
{ // get red and blue gain, and mode (4=auto, 3=manual)
  dc1394error_t err;
  //
  fWhite.id = DC1394_FEATURE_WHITE_BALANCE;
  err = dc1394_feature_get(camHandle, &fWhite);
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::getGain: Failed to get gain feature info\n");
  //
  if (varWhite != NULL)
  {
    varWhite->setInt(3, 0);
    varWhite->setInt(fWhite.min, 1);
    varWhite->setInt(fWhite.max, 2);
    varWhite->setInt(fWhite.RV_value, 3);
    varWhite->setInt(fWhite.BU_value, 4);
    varWhite->setBool(err == DC1394_SUCCESS, 5, true);
  }
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////////////

bool UCamDevGuppy::setWhiteBalance(int mode, int red, int blue)
{
  vars->lock();
  varWhite->setInt(mode, 0);
  if (mode == 3)
  { // manuel setting, so red and blue values are valid
    varWhite->setInt(red, 3);
    varWhite->setInt(blue, 4);
  }
  varSettingsChanged->setBool(true);
  vars->unlock();
  return true;
}

/////////////////////////////////////////////////////////

bool UCamDevGuppy::setWhiteBalanceRaw(int mode, int red, int blue)
{ // set mode (4=auto, 3=manual), red and blue gain
  dc1394error_t err;
  printf("Setting whitebal to mode=%d, red=%d blue=%d\n", mode, red, blue);
  if (mode != 3)
  {
    err = DC1394_FAILURE;
    if (fWhite.id == 0)
      getWhiteBalanceRaw();
    err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::setWhite: failed to set white-bal one-push auto\n");
    // return mode to manual - values will be set at next polling
    varWhite->setInt(3, 0);
    //err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_WHITE_BALANCE, &fWhite.current_mode);
  }
  else
  { // manuel mode
    err = dc1394_feature_whitebalance_set_value(camHandle, blue, red);
    err = dc1394_feature_whitebalance_get_value(camHandle, &fWhite.BU_value, &fWhite.RV_value);
  }
  if (err != DC1394_SUCCESS)
    printf("UCamDevGuppy::setWhite balance: to mode=%d (3=manual, 4=auto), red=%d, blue=%d failed\n", mode, red, blue);
  return err == DC1394_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////

bool UCamDevGuppy::deviceExist()
{ // open device and get default parameters
  bool result;
  //
  int d, c, j;
  //
  result = camHandle != NULL;
  if (not result)
  { // not already in action, so test interface card for a device
    lock();
    d = 10;
    c = getDc1394CamCnt();
    //printf(" - port %d has %d camera node(s)\n", i, c);
    // test if camera device number is on this interface card
    if ((d + c) <= getDeviceNumber())
    { // no - continue with next interface card
      d += c;
    }
    else if (d <= getDeviceNumber())
    {
      // the camera is on this card
      for (j = 0; j < c; j++)
      {
        if (d == getDeviceNumber())
        { // camera is found
          result = setCamDeviceNode(j);
          if (result)
            printf("Found device %d as a '%s'\n", d, getName());
          break;
        }
        d++;
      }
    }
    unlock();
  }
  return result;
}

////////////////////////////////////////////////////

bool UCamDevGuppy::openDeviceDefault()
{
  bool result;
  unsigned int width, height;
  //
  if (camHandle == NULL)
    deviceExist();
  //
  result = camHandle != NULL;
  if (frameRate <= 0)
    frameRate = 100;
  if (result)
  {
    if (varOpen != NULL)
      varOpen->setBool(true, 0);
    result = startIsoTransmission();
    if (result and not threadRunning)
      start();
    if (result and frameHeight < 0)
    { // no size is selected, so use as is.
      dc1394_get_image_size_from_video_mode(camHandle, selected_mode, &width, &height);
      if (width > 0)
      { // set current image size
        frameHeight = (int)height;
        frameWidth = (int)width;
      }
    }
  }
  return result;
}

////////////////////////////////////////////////////

void UCamDevGuppy::closeDevice()
{
  if (cameraOpen)
  { // stop iso_transmission
    stopIsoTransmission();
    closeDeviceRelease();
  }
}

void UCamDevGuppy::closeDeviceRelease()
{
  if (camHandle != NULL)
  {
    dc1394_capture_stop(camHandle);
    dc1394_camera_free(camHandle);
    printf("UCamDevGuppy::~UCamDevGuppy closed camera handle\n");
    camHandle = NULL;
  }
  if (devPlatform != NULL)
  {
    dc1394_free(devPlatform);
    devPlatform = NULL;
    //printf("UCamDevGuppy::~UCamDevGuppy closed device handle\n");
  }
}

////////////////////////////////////////////////////

bool UCamDevGuppy::getImageSnapshot(UImage * image)
{
  int i;
  bool result;
  //
  if (not isCameraOpen())
  { // open on data request
    openDevice();
  }
  // request a snapshot to this image buffer
  if (image != NULL)
    // image pointer may be NULL, just to trigger a snapshot event
    image->valid=false;
  // set destination pointer
  captureImage = image;

  // post the read thread to do the capture
  captureDo.post();
  // wait for the read thread to have finished
  for (i = 0; i < 200; i++)
  { // wait for read thread to post in captureDone
    if (captureDone.tryWait())
      break;
    Wait(0.01);
  }
  if (false and (i >= 200))
    printf("UCamDevGuppy::getImageSnapshot: Gave up "
        "waiting for image (device %d)\n", devNum);
  // remove buffer pointer - just to avoid misuse
  captureImage = NULL;
  // did we get an image
  result = image != NULL;
  if (result)
    result = image->valid;
  //
  return result;
}

///////////////////////////////////////////////////////////

int UCamDevGuppy::getFrameRate()
{
//  unsigned int fr;
//  if (dc1394_get_video_framerate(camHandle, camera.node, &fr) == DC1394_SUCCESS)
//    frameRate = framerate2fps(fr);
  return frameRate;
}

//////////////////////////////////////////////////////////

int UCamDevGuppy::framerate2fps(int framerate)
{
  int fps;
  switch(framerate)
  {
    case DC1394_FRAMERATE_1_875: fps = 2; break;
    case DC1394_FRAMERATE_3_75: fps = 4; break;
    case DC1394_FRAMERATE_7_5: fps = 7; break;
    case DC1394_FRAMERATE_15: fps = 15; break;
    case DC1394_FRAMERATE_30: fps = 30; break;
    case DC1394_FRAMERATE_60: fps = 60; break;
    case DC1394_FRAMERATE_120: fps = 120; break;
    case DC1394_FRAMERATE_240: fps = 240; break;
    default: fps = 10; break;
  }
  return fps;
}

///////////////////////////////////////////////////////////

int UCamDevGuppy::fps2mode5framerate(int fps)
{
  int fr;
  if (fps < 3)
    fr = DC1394_FRAMERATE_1_875;
  else if (fps <= 4)
    fr = DC1394_FRAMERATE_3_75;
  else if (fps <= 9)
    fr = DC1394_FRAMERATE_7_5;
  else if (fps <= 15)
    fr = DC1394_FRAMERATE_15;
  else if (fps <= 30)
    fr = DC1394_FRAMERATE_30;
  else if (fps <= 60)
    fr = DC1394_FRAMERATE_60;
  else if (fps <= 120)
    fr = DC1394_FRAMERATE_120;
  else
    fr = DC1394_FRAMERATE_240;
  return fr;
}

///////////////////////////////////////////////////////////

bool UCamDevGuppy::setDevice(const int width, const int height,
               const int framesPerSec)
{
  bool result = false;
  //int frNy, frCurr;
  //
  frameWidth = width;
  frameHeight = height;
//   frNy = fps2mode5framerate(framesPerSec);
//   frCurr = fps2mode5framerate(frameRate);
  if (not cameraOpen)
    openDeviceDefault();
/*  if (frNy != frCurr)
  { // change framerate
    if (dc1394_set_video_framerate(camHandle, camera.node, frNy) != DC1394_SUCCESS)
      getFrameRate();
    else
      frameRate = framesPerSec;
  }
  else*/
    result = true;
  imageSizeChanged(MAX_IMAGE_WIDTH / float(frameWidth));
  return result;
}

////////////////////////////////////////////////////

void * runIeee1394CamThread(void * camobj)
{ // Start thread here and call thread function
  UCamDevGuppy * camDev;
  //
  camDev = (UCamDevGuppy *) camobj;
  camDev->run();
  pthread_exit((void*)NULL);
  return NULL;
}

////////////////////////////////////////////////////

bool UCamDevGuppy::start()
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
    if (pthread_create(&thRead, &thAttr, &runIeee1394CamThread, (void *)this) != 0)
      // report error
      perror(camName);
      // wait for thread to initialize
    while ((not threadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not threadRunning)
    { // failed to start
      stopFrameRead = true;
      printf("Failed to start read thread for device %d : %s\n", devNum, getCameraName());
    }
    pthread_attr_destroy(&thAttr);
  }
  return threadRunning;
}

//////////////////////////////////////////////////////////////

void UCamDevGuppy::stop(bool andWait)
{
  if (threadRunning)
  {
    stopFrameRead = true;
    if (andWait and threadRunning)
      pthread_join(thRead, NULL);
    // debug
    printf("UCamDevGuppy::stop: read thread %d stopped\n", devNum);
  }
}

////////////////////////////////////////////////////////////

void UCamDevGuppy::run()
{
  UImage * img = NULL;
  bool needSingle;
  bool need4Push;
  bool gotOK = false;
  UTime t;
  int i = 0, j = 0;
  int failCnt = 0;
//  bool reFetchNewValues = false;
/*  const int MSL = 50;
  char s[MSL];*/
  //
  if (imgBuff[0] == NULL)
  { // make push buffer (if needed)
    imgBuff[0] = new UImage800();
    imgBuff[1] = imgBuff[0]; // to avoid destruction conflict
    //printf("Guppy device allocated local push buffer\n");
  }
  // initialize push buffer
  imgBuff[0]->camDevice = devNum;
  imgBuff[0]->setSize(480, 750, 1, 8, "RGGB");
  imgBuff[0]->valid = false;
  //
  threadRunning = true;
  //
  if (not stopFrameRead)
  {  // empy the capture post flag
    captureDo.tryWait();
    // and the done flag
    captureDone.tryWait();
  }
  // wait for camera to be initialized
  while (not initialized and not stopFrameRead)
  {
    Wait(0.1);
  }
  t.now();
  //
  // main image read loop
  while (not stopFrameRead)
  {
    if (isCameraOpen())
    { // try if a wait would succeed
      needSingle = captureDo.tryWait();
      // test push stack for commands in need of images
      need4Push = needNewPushData();
      if (needSingle)
      { // a posted request for data is accepted
        img = captureImage;
        // debug
//        printf("Image serial %lu, need single image, img=%x\n",
//               imageNumber, (unsigned int)img);
        // debug end
      }
      else if (need4Push)
      { // use push buffer
        img = imgBuff[0];
        // debug
/*        printf("Image serial %lu, need push image, img=%x\n",
               imageNumber, (unsigned int)img);*/
        // debug end
      }
      else
        // noone needs the data, so just empty the buffer
        img = NULL;
      //
      while (j < 40)
      {
        gotOK = getSingleImage(img);
        if (need4Push and gotOK)
        {
          img->updated();
          // imform event handler that a new image is ready
          imgUpdated();
        }
        if (needSingle and gotOK)
        {
          captureDone.post();
        }
        if (gotOK)
        {
          double dt = tLastFrame.getTimePassed();
          if (dt > 5.0)
          {
            double fr = (imageNumber - lastImageNumber) / dt;
            frameRate = roundi(fr);
            varFramerate->setValued(fr, 0, false);
            tLastFrame.now();
            lastImageNumber = imageNumber;
          }
          failCnt = 0;
          i = frameRate;
          j = 0;
          varImageNumber->setInt(imageNumber++, 0);
        }
        else
        {
          j++;
          //if (j == 40)
          //  printf("Camera device %d waited > 2 sec with no images from camera\n", devNum);
          Wait(0.01);
        }
        if (gotOK or img == NULL)
          break;
      }
      // debug
      if (j >= 40)
        j = 0;
      // debug end
      if (img != NULL and not gotOK)
      { // failed to get image - report
        bool extTrig;
        failCnt++;
        extTrig = fTrigger.is_on == DC1394_ON and fTrigger.available == DC1394_TRUE;
        if (not extTrig)
        { // on external trigger - assume some wait time (not an error)
          if (failCnt > i)
          {
            printf("Failed to get image %d images (device=%d)\n",
                  failCnt, devNum);
            i += abs(frameRate);
          }
          if (failCnt > (4 * abs(frameRate)))
          {
            printf("closing device %d\n", devNum);
            closeDevice();
            failCnt = 0;
            i = frameRate;
          }
        }
      }
      // regular poll of settings to see if values in var has changed
      if (t.getTimePassed() > 1.1 and (not varSettingsChanged->getBool())) //  or reFetchNewValues))
      {
/*        if (reFetchNewValues)
        {
          varSettingsChanged->setBool(true);
          reFetchNewValues = false;
        }
        else*/
        { // test settings
          varSettingsChanged->setBool(isVarUpdated());
          // refetch to get actual values after a settings change
//          reFetchNewValues = varSettingsChanged->getBool();
        }
      }
      // update as needed
      if (varSettingsChanged->getBool())
      { // time to query (and implement) settings
        varSettingsChanged->setBool(false);
        updateVars();
      }
    }
    else
    {
      Wait(0.1);
    }
  }
  threadRunning = false;
}

///////////////////////////////////////////////

bool UCamDevGuppy::setExternalTrigger(bool value, bool * supported)
{
  vars->lock();
  varExternalTrig->setBool(value, 0);
  varSettingsChanged->setBool(true);
  vars->unlock();
  return true;
}

///////////////////////////////////////////////

bool UCamDevGuppy::setExternalTriggerRaw(bool value)
{
  dc1394error_t err = DC1394_FAILURE;
  uint32_t   flag;
  uint64_t IO_OUTP_CTRL2 = 0x1000324;
  //
  fTrigger.id = DC1394_FEATURE_TRIGGER;
  fTriggerDelay.id = DC1394_FEATURE_TRIGGER_DELAY;
  if (camHandle != NULL)
  {
    //err = dc1394_feature_get(camHandle, &fTriggerDelay);
    //if (err != DC1394_SUCCESS)
    //  printf("UCamDevGuppy::setExternalTrigger: Failed to get trigger delay feature info\n");
    //dc1394_feature_print(&fTriggerDelay, stdout);
    err = dc1394_feature_get(camHandle, &fTrigger);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::setExternalTrigger: Failed to get trigger feature info\n");
    //dc1394_feature_print(&fTrigger, stdout);
  }
  if (err == DC1394_SUCCESS and fTrigger.available)
  {
    if (value != (fTrigger.is_on == DC1394_ON))
    { // need to set trigger
      if (value)
        dc1394_external_trigger_set_power(camHandle, DC1394_ON);
      else
        dc1394_external_trigger_set_power(camHandle, DC1394_OFF);
      // get new state and set global variables
      getExternalTriggerRaw();
    }
    if (value)
    { // set to external trigger - set also output 2 to allow it to control trigger from software
      // through pin 2 in the 8-pin plug (should be connected to pin 4 on this and other
      // connected cameras.
      //
      // get current state of IO_CONTROL 2
      err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
      // debug
      // printf("UCamDevGuppy::setExternalTrigger: is  ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
      // debug end
      // set also output 2 to be follow pin state
      flag = 0x80000000; // mode is on      bit [0]
      //flag += 0x1 << 24; // invert output - bit [7]
      flag += 0x01 << 16; // follow pin state - bit [11-15]
      //flag += 0x1; // pin state = 1  bit [31]
      err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
      // debug
      // printf("UCamDevGuppy::setExternalTrigger: to  ioCtrl (%llx) value is %x\n\n", IO_OUTP_CTRL2, flag);
      // debug end
      //err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
      // debug
      //printf("UCamDevGuppy::setExternalTrigger: now ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
      // debug end
    }
  }
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////

bool UCamDevGuppy::getExternalTrigger(bool * value, bool * supported)
{
  bool result;
  bool locked = vars != NULL;
  //
  if (locked)
    vars->lock();
  result = getExposureTargetRaw();
  if (supported != NULL)
    *supported = result and fTrigger.available == DC1394_TRUE;
  if (value != NULL and result)
    *value = (fTrigger.is_on == DC1394_ON);
  if (locked)
    vars->unlock();
  return result;
}

/////////////////////////////////////////////

bool UCamDevGuppy::getExternalTriggerRaw()
{
  dc1394error_t err = DC1394_FAILURE;
  //
  if (camHandle != NULL)
  {
    fTrigger.id = DC1394_FEATURE_TRIGGER;
    err = dc1394_feature_get(camHandle, &fTrigger);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::getExternalTrigger: Failed to get trigger feature info\n");
    //dc1394_feature_print(&fTrigger, stdout);
  }
  if (varExternalTrig != NULL)
  {
    varExternalTrig->setBool(fTrigger.is_on == DC1394_ON, 0);
    varExternalTrig->setBool(err == DC1394_SUCCESS, 1);
  }
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////

bool UCamDevGuppy::makeTriggerPulse()
{
  uint32_t   flag;
  uint64_t IO_OUTP_CTRL2 = 0x1000324;
  dc1394error_t err = DC1394_FAILURE;
  //
  if (camHandle != NULL and fTrigger.available == DC1394_TRUE)
  { // make shure noone else has an io-control function in action
    vars->lock();
    err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
    //printf("UCamDevGuppy::makeTriggerPulse: was  ioCtrl (0x%llx) value is 0x%x\n", IO_OUTP_CTRL2, flag);
    // debug end
    // set pin state (LSB bit, i.e. bit [31]) to 1
    flag |= 0x1;
    //printf("UCamDevGuppy::makeTriggerPulse: to   ioCtrl (0x%llx) value is 0x%x\n", IO_OUTP_CTRL2, flag);
    err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
    //Wait(0.02);
    //flag ^= 0x1;
    //err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
    //printf("UCamDevGuppy::makeTriggerPulse: got  ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
    // and set it back to zero
    flag &= 0xfffffffe;
    err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
    // debug
    //printf("UCamDevGuppy::makeTriggerPulse: back ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
    // debug end
    //Wait(0.02);
    //err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
    // debug
    //printf("UCamDevGuppy::makeTriggerPulse: now  ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);*/
    //printf("\n");
    vars->unlock();
  }
  return err == DC1394_SUCCESS;
}


///////////////////////////////////////////////

// int UCamDevGuppy::getExposureTargetRaw(bool * supported, int * minValue, int * maxValue)
// {
//   int result = -1;
//   if (supported != NULL)
//     *supported = fExposure.available;
//   if (minValue != NULL)
//     *minValue = fExposure.min;
//   if (maxValue != NULL)
//     *maxValue = fExposure.max;
//   result = fExposure.value;
// }

bool UCamDevGuppy::getExposureTargetRaw()
{
  dc1394error_t err = DC1394_FAILURE;
  //
  fExposure.id = DC1394_FEATURE_EXPOSURE;
  if (camHandle != NULL)
  {
    err = dc1394_feature_get(camHandle, &fExposure);
    if (err != DC1394_SUCCESS)
      printf("UCamDevGuppy::getExposure: Failed to get exposure feature info\n");
  }
  if (varExposure != NULL)
  {
    varExposure->setInt(fExposure.value, 0);
    varExposure->setInt(fExposure.min, 1);
    varExposure->setInt(fExposure.max, 2);
    varExposure->setBool(err == DC1394_SUCCESS, 3, true);
  }
  return err == DC1394_SUCCESS;
}

///////////////////////////////////////////////

// bool UCamDevGuppy::setExposureTarget(int value)
// {
//   vars->lock();
//   varExposure->setValued(value, 0);
//   vars->unlock();
//   return true;
// }

//////////////////////////////////////////////

bool UCamDevGuppy::setExposureTargetRaw(int value)
{
  bool supported = false;
  dc1394error_t err = DC1394_FAILURE;
  //
  supported = getExposureTargetRaw();
  if (supported)
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_EXPOSURE, (uint32_t)mini(fExposure.max, maxi(fExposure.min, value)));
  return err == DC1394_SUCCESS;
}

////////////////////////////////////////////////

void UCamDevGuppy::createVars()
{
  UCamDevBase::createVars();
  if (vars != NULL)
  {
    varOpen = vars->addVar("open", double(cameraOpen), "d", "(rw) is camera open (0 = closed, 1 = open)");
    varImageNumber = vars->addVar("imageNumber", imageNumber, "d", "(ro) Next image number to use (number of images received from camera)");
    varExposure = vars->addVarA("exposure", "-1 0 100", "d", "(rw) Exposure target in auto mode (value, min, max, valid)");
    varGain = vars->addVarA("gain", "-1 0 100 0", "d", "(rw) Video gain setting for camera (value, min, max, auto [0=man/1=auto], valid)");
    varShutter = vars->addVarA("shutter", "-1 0 100 0", "d", "(rw) Shutter value setting for camera (value, min, max, auto [0=man/1=auto], valid)");
    varWhite = vars->addVarA("white", "-1 0 100 0 0", "d", "(rw) White ballance (mode [3=man,4=auto], min, max, red, blue, valid)");
    varExternalTrig = vars->addVarA("externalTrigger", "0 0", "d", "(rw) internal free run; 1=external trigger mode (mode, valid)");
    varFramerate = vars->addVar("framerate", 0.0, "d", "(r) actually generated framerate [frames/sec]");
    varISObw = vars->addVar("isoBW", 200.0, "d", "(rw) set the allocated BW for the camera 100, 200 or 400 (bus has 400Mbps in total, so 2 cams need to set both to 200 (or less))");
    varPacketSize = vars->addVar("packetsize", 1024, "d", "(rw) packed size each timeslot (on next open), 1024 is max in iso100 bw");
    varSettingsChanged = vars->addVar("settingsUpdate", 1.0, "d", "(r/w) if set to 1 request settings update, changed to 0 when updated");
  }
}

////////////////////////////////////////////////

void UCamDevGuppy::updateVars()
{
  int u;
  //
  if (vars != NULL and vars->tryLock())
  { // update only if noone else has locked the global vars vars
    // gain
    u = varGain->getInt(0);
    if (u < 0)
      getGainRaw();
    else if ((int)fGain.value != u or ((fGain.current_mode == DC1394_FEATURE_MODE_AUTO) != varGain->getBool(3)))
    {
      if (varGain->getBool(3))
        u = -1;
      setGainRaw(u);
    }
    else
      getGainRaw();
    // shutter
    u = varShutter->getInt(0);
    if (u < 0)
      getShutterRaw();
    else if ((int)fShutter.value != u or ((fShutter.current_mode == DC1394_FEATURE_MODE_AUTO) != varShutter->getBool(3)))
    {
      if (varShutter->getBool(3))
        u = -1;
      setShutterRaw(u);
    }
    else
      getShutterRaw();
    // exposure
    u = varExposure->getInt(0);
    if (u < 0)
      getExposureTargetRaw();
    else if ((int)fExposure.value != u)
      setExposureTargetRaw(u);
    else
      getExposureTargetRaw();
    // white balance
    u = varWhite->getInt(0);
    if (u < 0)
      getWhiteBalanceRaw();
    else if (u != 3)
    { // set automatic white bal (one-shot)
      setWhiteBalanceRaw(4, varWhite->getInt(3), varWhite->getInt(4));
    }
    else if ((int)fWhite.RV_value != varWhite->getInt(3) or (int)fWhite.BU_value != varWhite->getInt(4) )
      // manuel mode
      setWhiteBalanceRaw(3, varWhite->getInt(3), varWhite->getInt(4));
    else
      // just update values
      getWhiteBalanceRaw();
    // trigger
    u = varExternalTrig->getInt(0);
    if (u < 0)
      // never set - ask camera
      getExternalTriggerRaw();
    else if (varExternalTrig->getBool(0) != (fTrigger.is_on == DC1394_ON))
      // modified
      setExternalTriggerRaw(varExternalTrig->getBool(0));
    else
      // regular update
      getExternalTriggerRaw();
    //
    if (not varOpen->getBool())
      closeDevice();
    //
    vars->unlock();
  }
}

////////////////////////////////////////////////

bool UCamDevGuppy::isVarUpdated()
{
  bool result = false;
  //
  if (vars != NULL and vars->tryLock())
  { // gain
    result = ((fGain.current_mode == DC1394_FEATURE_MODE_AUTO) != varGain->getBool(3));
    if (not result and fGain.current_mode != DC1394_FEATURE_MODE_AUTO)
      result = varGain->getInt(0) != (int)fGain.value;
    // shutter
    if (not result)
    {
      result = (fShutter.current_mode == DC1394_FEATURE_MODE_AUTO) != varShutter->getBool(3);
      if (not result and fShutter.current_mode != DC1394_FEATURE_MODE_AUTO)
        result = varShutter->getInt(0) != (int)fShutter.value;
    }
    // exposure
    if (not result)
      result = varExposure->getInt(0) != (int)fExposure.value;
    // white balance
/*    if (not result)
      result = (int)fWhite.RV_value != varWhite->getInt(3) or (int)fWhite.BU_value != varWhite->getInt(4);*/
    // trigger
    if (not result)
      result = varExternalTrig->getBool(0) != (fTrigger.is_on == DC1394_ON);
    //
    vars->unlock();
  }
  return result;
}

/////////////////////////////////////////

void UCamDevGuppy::callGotNewDataWithObject()
{ // dummy call, as if no data were available
  UImage * img = imgBuff[0];
  //
  if (img != NULL)
  { // call then push implementation with the actual image
    img->lock();
    gotNewImage(img);
    img->unlock();
  }
}



////////////////////////////////////

void UCamDevGuppy::setPushBuffer(UImage * imageBuffer)
{
  // replace (or set) image buffer
  if (imageBuffer == NULL)
    // make shure to stop before buffer is removed
    stop(true);
  //
  imgBuff[0] = imageBuffer;
  if (imgBuff[1] != NULL)
  { // temporary buiffer were created - remove
    delete imgBuff[1];
    imgBuff[1] = NULL;
    // printf("push buffer replaced for device %d\n", getDeviceNumber());
  }
/*  else
    printf("push buffer set for device %d\n", getDeviceNumber());*/
}

#endif

