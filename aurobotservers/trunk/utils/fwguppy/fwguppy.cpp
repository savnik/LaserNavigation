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

#include "fwguppy.h"
//#define USE_GUPPY
#include <dc1394/control.h>
#include <dc1394/types.h>

  //

int main(int argc, char ** argv)
{
  bool isOK;
  // init
  //camType = CAM_DEV_IEEE1394;
  stopFrameRead = true;
  threadRunning = false;
  fGain.id = (dc1394feature_t) 0;
  fShutter.id = (dc1394feature_t) 0;
  fWhite.id = (dc1394feature_t) 0;
  fExposure.id = (dc1394feature_t) 0;
  fTrigger.id = (dc1394feature_t) 0;
  fTriggerDelay.id = (dc1394feature_t) 0;
  imageNumber = 0;
  lastImageNumber = 0;
  //
  camHandle = NULL;
  devPlatform = NULL;
  // find device and set camHandle and devPlatform (for FW device 0)
  setCamDeviceNode(0);
  // open data channel and set BW and packet size
  isoBW = 400;
  packetSize = 1600;
  isOK = openDevice(0);
  if (not isOK)
    printf("Failed to open device\n");
  // camera is now open - start read thread.
  getShutterRaw();
  getGainRaw();
  printf("shutter range %d-%d, gain range %d-%d\n", fShutter.min, fShutter.max, fGain.min, fGain.max);
//  setGainRaw(-1); // -1 is auto
//  setShutterRaw(32);
  // work is done in thread, so just wait
  while (isOK and imageNumber < 100000)
  { // print status
    saveSample = true;
    Wait(5);
  }
  // stop read thread
  printf("closing device\n");
  if (isOK)
  { // stop read thread 
    stop(true);
    // close iso channel
    closeDevice();
  }
  if (camHandle != NULL)
  { // release device structures
    dc1394_capture_stop(camHandle);
    dc1394_camera_free(camHandle);
    printf("~UCamDevGuppy closed camera handle\n");
    camHandle = NULL;
  }
  if (devPlatform != NULL)
  {
    dc1394_free(devPlatform);
    devPlatform = NULL;
    //printf("~UCamDevGuppy closed device handle\n");
  }
  return 0;
}


///////////////////////////////////////////

int getIeee1394PortCnt()
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

int getDc1394CamCnt()
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
      printf("getDc1394CamCnt failed to get access to /dev/raw1394\n");
    }
  }
  //
  if (devPlatform != NULL)
  {
    err = dc1394_camera_enumerate(devPlatform, &list);
    if (err != DC1394_SUCCESS)
      printf("getDc1394CamCnt failed to enumerate cameras\n");
      // DC1394_ERR_RTN(err,"Failed to enumerate cameras");
  }
  if (devPlatform != NULL and err == DC1394_SUCCESS)
  {
/*    printf("getDc1394CamCnt found %d cameras\n", list->num);
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

bool setCamDeviceNode(int camNum)
{ // this is to test that the camera exist and is working
  // and is not called after camera detect is finished (except for >> camsget all)
  bool result = false;
  //dc1394_t * d;
  dc1394camera_list_t * list;
  dc1394error_t err = DC1394_SUCCESS;
//   dc1394video_modes_t modes;
//  dc1394featureset_t allFeatures;
  int c = getDc1394CamCnt();
    //printf(" - port %d has %d camera node(s)\n", i, c);
    // test if camera device number is on this interface card
  result = c > camNum;
  if (not result)
    printf("No camera number %d (found only %d cameras\n", camNum, c);
  if (result)
  {
    if (devPlatform == NULL)
    {
      devPlatform = dc1394_new();
      if (devPlatform == NULL)
        printf("setCamDeviceNode failed to access /dev/fw0\n");
      cameraOpen = false;
    }
    //
    if (devPlatform != NULL)
      err = dc1394_camera_enumerate(devPlatform, &list);
    result &= (err == DC1394_SUCCESS);
    if (err != DC1394_SUCCESS)
      printf("setCamDeviceNode failed to enumerate cameras\n");
  }
    // DC1394_ERR_RTN(err,"Failed to enumerate cameras");
  if (result and list->num > 0)
  { // get handle to camera bus (port)
    camHandle = dc1394_camera_new(devPlatform, list->ids[camNum].guid);
    if (camHandle == NULL)
      printf("setCamDeviceNode failed to initialize camera %d with GUID=%llx\n", camNum, list->ids[camNum].guid);
    if (camHandle != NULL)
    { // handle is OK,
      // debug
      //dc1394_camera_print_info(camHandle, stdout);
      //dc1394_feature_get_all(camHandle, &allFeatures);
      //dc1394_feature_print_all(&allFeatures, stdout);
      // debug end
      // printf("Device is a %s %s\n", camHandle->vendor, camHandle->model);
      snprintf(camName, MAZ_CAM_NAME_CNT, "%s %llx", camHandle->model, camHandle->guid);
      // release any existing iso channels 
      release_iso_and_bw();
      // get camera get mode to use
      err = dc1394_video_get_supported_modes(camHandle, &modes);
      if (err == DC1394_SUCCESS)
      {
     /* for (int i=0; i < (int)modes.num; i++)
        {
          selected_mode = modes.modes[i];
          print_mode_info(camHandle, selected_mode);
        }*/
        selected_mode = modes.modes[modes.num - 1];
        print_mode_info(camHandle, selected_mode);
        result = true;
      }
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

void release_iso_and_bw()
{
  uint32_t val;
  dc1394error_t err = DC1394_SUCCESS;

  err = dc1394_video_get_bandwidth_usage(camHandle, &val);
  if (err == DC1394_SUCCESS)
  {
    err = dc1394_iso_release_bandwidth(camHandle, val);
    //printf("release_iso_and_bw: released (%s) %d bytes/sec? of bandwidth for device %d\n",
    //       bool2str(err == DC1394_SUCCESS), val, devNum);
  }

  err = dc1394_video_get_iso_channel(camHandle, &val);
  if (err == DC1394_SUCCESS)
  {
    printf(" - A failure to free iso channel is OK\n");
    err = dc1394_iso_release_channel(camHandle, val);
    //printf("release_iso_and_bw: released (%s) iso channel (value %d) for device %d\n",
    //       bool2str(err == DC1394_SUCCESS), val, devNum);
  }
}

////////////////////////////////////////////

void print_format(uint32_t format)
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

void print_mode_info( dc1394camera_t *camera , uint32_t mode )
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
      printf("print_mode_info failed to get framerate\n");
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

bool getSingleImage(UImage * destination)
{
  bool result;
  int bytes, n;
  dc1394error_t err;
  UImage * img = destination;
  unsigned int depth;
  dc1394video_frame_t *frame=NULL;
  uint64_t ts, tu;
  UTime t;
  //uint64_t bCnt;
  //

//   err = dc1394_format7_get_total_bytes(camHandle, selected_mode, &bCnt);
//   if (err != DC1394_SUCCESS)
//     printf("failed to get image size in bytes\n");
  //
  err = dc1394_capture_dequeue(camHandle, DC1394_CAPTURE_POLICY_WAIT, &frame);
  //err = dc1394_capture_dequeue(camHandle, DC1394_CAPTURE_POLICY_POLL, &frame);
  result = err == DC1394_SUCCESS and frame != NULL;
  if (err != DC1394_SUCCESS)
    printf("getSingleImage: Could not dequeue (or poll for) an image frame\n");

  if (result)
  { // frame time
    ts = frame->timestamp / 1000000;
    tu = frame->timestamp - ts * 1000000;
    t.setTime(ts, tu);
    //
    if (img != NULL and img->tryLock())
    {
//       frameWidth = w; // frame->size[0];
//       frameHeight = h; // frame->size[1];
      depth = frame->data_depth; // mono8 => 8 bit depth
      // frame->color_coding == DC1394_COLOR_CODING_MONO8
      // frame->color_filter == DC1394_COLOR_CODING_RGGB
      // frame->little_endian == DC1394_FALSE
      //dc1394_get_image_size_from_video_mode(camHandle, selected_mode, &width, &height);
      img->setSize(frameHeight, frameWidth, 1, depth, "BGGR");
      n = frameHeight * frameWidth * depth / 8;
      //n = frame->image_bytes;
      //packetSize = frame->packet_size;
      //
      if (imageNumber % 50 == 0)
        printf("got image at %lu.%06lu size %dx%d, from %dx,%dy, depth=%d (pkg-size=%d)\n", 
             t.getSec(), t.getMicrosec(), 
             frameHeight, frameWidth, frame->position[0], frame->position[1], 
             depth, frame->packet_size);
      //
      bytes = img->getBufferSize();
      if (n <= bytes)
      {
        memcpy(img->getData(), frame->image, n);
      }
      img->camDevice = 10;
      img->cam = NULL;
      img->imgTime = t;
      img->imageNumber = ++imageNumber;
      img->valid = true;
      img->unlock();
    }
    else
    { // no image to get - wait a bit and retry
      Wait(0.0011);
    }
    // put buffer back in camera queue
    err = dc1394_capture_enqueue(camHandle, frame);
    if (err != DC1394_SUCCESS)
      printf("getSingleImage: could not return image buffer\n");
  }
  return result;
}

/////////////////////////////////////////////////////////////


bool startIsoTransmission(int isoBw, int packetSize)
{
  dc1394error_t err;
  /*-----------------------------------------------------------------------
    *  setup capture
    *-----------------------------------------------------------------------*/
  //printf("Starting video-mode, capture and video transmission (dev %d)\n", devNum);
  switch (isoBW)
  {
    case 100: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_100); break;
    case 200: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_200); break;
    case 400: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_400); break;
    case 800: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_800); break;
    case 1600: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_1600); break;
    case 3200: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_3200); break;
  }
  if (err != DC1394_SUCCESS)
    printf("startIsoTransmission: Could not set ISO BW (to%d)\n", isoBW);
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
  err = dc1394_format7_set_packet_size(camHandle, selected_mode, packetSize);
  if (err != DC1394_SUCCESS)
    printf("startIsoTransmission: Could not set packet size to %d\n", packetSize);
  //
  err = dc1394_video_set_mode(camHandle, selected_mode);
  if (err != DC1394_SUCCESS)
    printf("startIsoTransmission: Could not set video mode FORMAT_7_0\n");

  if (err == DC1394_SUCCESS)
  {
    err = dc1394_capture_setup(camHandle, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err != DC1394_SUCCESS and err != DC1394_CAPTURE_IS_RUNNING)
    {
      printf("startIsoTransmission: Could not setup camera\n");
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
      printf("startIsoTransmission: Could not start camera iso transmission\n");
    cameraOpen = (err == DC1394_SUCCESS);
  }
  else if (err == DC1394_CAPTURE_IS_RUNNING)
    cameraOpen = true;
  return cameraOpen;
}

//////////////////////////////////////////////////////////////

bool stopIsoTransmission()
{
  dc1394error_t err;
  bool result = true;
  //
  if (cameraOpen)
  {
    cameraOpen = false;
    // wait for any last image to be captured
    Wait(0.2);
    /*Step 6: Stop sending data*/
    dc1394_capture_stop(camHandle);
    err = dc1394_video_set_transmission(camHandle,DC1394_OFF);
    result = err == DC1394_SUCCESS;
    if (not result)
      printf("stopIsoTransmission: could not stop the camera\n");
  }
  else
    printf("stopIsoTransmission(): camera is closed already\n");

  return result;
}

///////////////////////////////////////////////////////////

bool setGainRaw(int agc)
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
        printf("setGain: failed to set gain to auto\n");
      err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_GAIN, &fGain.current_mode);
    }
    else
      err = DC1394_SUCCESS;
  }
  else
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_GAIN, agc);
  if (err != DC1394_SUCCESS)
    printf("setGain: to %d failed\n", agc);
   return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////


bool getGainRaw()
{
  dc1394error_t err;
  //
  fGain.id = DC1394_FEATURE_GAIN;
  err = dc1394_feature_get(camHandle, &fGain);
  if (err != DC1394_SUCCESS)
    printf("getGain: Failed to get gain feature info\n");
  //
//   fGain.value;
//   fGain.min;
//   fGain.max;
//   fGain.current_mode == DC1394_FEATURE_MODE_AUTO;
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////

bool setShutterRaw(int value)
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
        printf("setShutter: failed to change to manuel mode\n");
    }
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_SHUTTER, value);
    err = dc1394_feature_get_value(camHandle, DC1394_FEATURE_SHUTTER, &fShutter.value);
  }
  if (err != DC1394_SUCCESS)
    printf("setGain: to %d failed\n", value);
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////

bool getShutterRaw()
{
  dc1394error_t err;
  //
  fShutter.id = DC1394_FEATURE_SHUTTER;
  err = dc1394_feature_get(camHandle, &fShutter);
  if (err != DC1394_SUCCESS)
    printf("getGain: Failed to get gain feature info\n");
  //
// fShutter.value;
// fShutter.min;
// fShutter.max;
// fShutter.current_mode == DC1394_FEATURE_MODE_AUTO;
  return err == DC1394_SUCCESS;
}


////////////////////////////////////////////////////////


bool getWhiteBalanceRaw()
{ // get red and blue gain, and mode (4=auto, 3=manual)
  dc1394error_t err;
  //
  fWhite.id = DC1394_FEATURE_WHITE_BALANCE;
  err = dc1394_feature_get(camHandle, &fWhite);
  if (err != DC1394_SUCCESS)
    printf("getGain: Failed to get gain feature info\n");
  //
// fWhite.min;
// fWhite.max;
// fWhite.RV_value;
// fWhite.BU_value;
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////////////

bool setWhiteBalanceRaw(int mode, int red, int blue)
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
      printf("setWhite: failed to set white-bal one-push auto\n");
    //err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_WHITE_BALANCE, &fWhite.current_mode);
  }
  else
  { // manuel mode
    err = dc1394_feature_whitebalance_set_value(camHandle, blue, red);
    err = dc1394_feature_whitebalance_get_value(camHandle, &fWhite.BU_value, &fWhite.RV_value);
  }
  if (err != DC1394_SUCCESS)
    printf("setWhite balance: to mode=%d (3=manual, 4=auto), red=%d, blue=%d failed\n", mode, red, blue);
  return err == DC1394_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////

// bool deviceExist(int j)
// { // open device and get default parameters
//   bool result;
//   int c;
//   //
//   result = camHandle != NULL;
//   if (not result)
//   { // not already in action, so test interface card for a device
//     c = getDc1394CamCnt();
//     //printf(" - port %d has %d camera node(s)\n", i, c);
//     // test if camera device number is on this interface card
//     if (c >= j)
//     {
//       result = setCamDeviceNode(j);
//       if (result)
//         printf("Found device %d as a '%s'\n", j, camName);
//     }
//   }
//   return result;
// }

////////////////////////////////////////////////////

bool openDevice(int idx)
{
  bool result;
  unsigned int width, height;
  //
  result = camHandle != NULL;
  if (frameRate <= 0)
    frameRate = 100;
  if (result)
  {
    result = startIsoTransmission(isoBW, packetSize);
    if (result and not threadRunning)
      start();
    if (result and frameHeight <= 0)
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

void closeDevice()
{
  if (cameraOpen)
  { // stop iso_transmission
    stopIsoTransmission();
    closeDeviceRelease();
  }
}

void closeDeviceRelease()
{
  if (camHandle != NULL)
  {
    dc1394_capture_stop(camHandle);
    dc1394_camera_free(camHandle);
    printf("~UCamDevGuppy closed camera handle\n");
    camHandle = NULL;
  }
  if (devPlatform != NULL)
  {
    dc1394_free(devPlatform);
    devPlatform = NULL;
    //printf("~UCamDevGuppy closed device handle\n");
  }
}

////////////////////////////////////////////////////

void * runIeee1394CamThread(void * camobj)
{ // Start thread here and call thread function
  run();
  pthread_exit((void*)NULL);
  return NULL;
}

////////////////////////////////////////////////////

bool start()
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
    if (pthread_create(&thRead, &thAttr, &runIeee1394CamThread, NULL) != 0)
      // report error
      perror(camName);
      // wait for thread to initialize
    while ((not threadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not threadRunning)
    { // failed to start
      stopFrameRead = true;
      printf("Failed to start read thread\n");
    }
    pthread_attr_destroy(&thAttr);
  }
  return threadRunning;
}

//////////////////////////////////////////////////////////////

void stop(bool andWait)
{
  if (threadRunning)
  {
    stopFrameRead = true;
    if (andWait and threadRunning)
      pthread_join(thRead, NULL);
    // debug
    printf("stop: read thread stopped\n");
  }
}

////////////////////////////////////////////////////////////

void run()
{
  UImage * img = NULL;
  bool gotOK = false;
  UTime t;
  const int MSL = 300;
  char s[MSL], s2[MSL];
  int hgt = 15, top = 0;
  const int maxFrameHeight = 480;
  //
  if (imgBuff == NULL)
  { // make push buffer (if needed)
    imgBuff = new UImage800();
  }
  // initialize push buffer
  imgBuff->setSize(480, 750, 1, 8, "RGGB");
  imgBuff->valid = false;
  //
  threadRunning = true;
  //
  t.now();
  //
  img = imgBuff;
  // main image read loop
  while (not stopFrameRead)
  {
    if (cameraOpen)
    { // test push stack for commands in need of images
      gotOK = getSingleImage(img);
      if (gotOK)
      {
        img->updated();
        //
        // printf("got image at %lu.%06lu of size %dx%d\n", img->imgTime.getSec(), img->imgTime.getMicrosec(), img->height(), img->width());
        //
        double dt = tLastFrame.getTimePassed();
        //
        if (saveSample)
        {
          img->imgTime.getForFilename(s2);
          snprintf(s, MSL, "imgfwguppy%06d_%s.png", imageNumber, s2);
          img->savePNG(s);
          saveSample = false;
          //
          // change ROI
          dc1394error_t err = DC1394_SUCCESS;
          printf("Image number %d\n", imageNumber);
          hgt *=2;
          if (hgt + top < maxFrameHeight and camHandle != NULL)
          {
            err = dc1394_format7_set_roi(camHandle, selected_mode,  DC1394_COLOR_CODING_RAW8,
                      packetSize,
                      0 /*left*/, top /*top*/, frameWidth /* width*/, hgt /*height*/ );
            if (err != DC1394_SUCCESS)
              printf("failed to set ROI to %dx, %dy, %dw, %dh\n", 0, top, frameWidth, hgt); 
            else
              printf("set ROI to %dx, %dy, %dw, %dh\n", 0, top, frameWidth, hgt); 
            
            uint32_t w, h;
            err = dc1394_format7_get_image_size(camHandle, selected_mode, &w, &h);
            if (err != DC1394_SUCCESS)
              printf("failed to get image size in HxW\n");
            else
            {
              frameHeight = h;
              frameWidth = w;
            }
          }
          else
          {
            // reached max height
            top += 20;
            hgt = 10;
          }
          if (top > maxFrameHeight - 100)
            top = 0;
        }
        //
        if (dt > 5.0)
        {
          frameRate = (imageNumber - lastImageNumber) / dt;
          tLastFrame.now();
          lastImageNumber = imageNumber;
          printf("Framerate = %.2f Hz\n", frameRate);
        }
      }
      else
      {  // failed no image available yet
        Wait(0.001);
      }
    }
    else
    { // camera not open
      printf("camera not open\n");
      Wait(0.1);
    }
  }
  threadRunning = false;
}

///////////////////////////////////////////////


bool setExternalTriggerRaw(bool value)
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
    //  printf("setExternalTrigger: Failed to get trigger delay feature info\n");
    //dc1394_feature_print(&fTriggerDelay, stdout);
    err = dc1394_feature_get(camHandle, &fTrigger);
    if (err != DC1394_SUCCESS)
      printf("setExternalTrigger: Failed to get trigger feature info\n");
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
      // printf("setExternalTrigger: is  ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
      // debug end
      // set also output 2 to be follow pin state
      flag = 0x80000000; // mode is on      bit [0]
      //flag += 0x1 << 24; // invert output - bit [7]
      flag += 0x01 << 16; // follow pin state - bit [11-15]
      //flag += 0x1; // pin state = 1  bit [31]
      err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
      // debug
      // printf("setExternalTrigger: to  ioCtrl (%llx) value is %x\n\n", IO_OUTP_CTRL2, flag);
      // debug end
      //err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
      // debug
      //printf("setExternalTrigger: now ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
      // debug end
    }
  }
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////

bool getExternalTriggerRaw()
{
  dc1394error_t err = DC1394_FAILURE;
  //
  if (camHandle != NULL)
  {
    fTrigger.id = DC1394_FEATURE_TRIGGER;
    err = dc1394_feature_get(camHandle, &fTrigger);
    if (err != DC1394_SUCCESS)
      printf("getExternalTrigger: Failed to get trigger feature info\n");
    //dc1394_feature_print(&fTrigger, stdout);
  }
  //  fTrigger.is_on == DC1394_ON;
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////

bool makeTriggerPulse()
{
  uint32_t   flag;
  uint64_t IO_OUTP_CTRL2 = 0x1000324;
  dc1394error_t err = DC1394_FAILURE;
  //
  if (camHandle != NULL and fTrigger.available == DC1394_TRUE)
  { // make shure noone else has an io-control function in action
    err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
    //printf("makeTriggerPulse: was  ioCtrl (0x%llx) value is 0x%x\n", IO_OUTP_CTRL2, flag);
    // set pin state (LSB bit, i.e. bit [31]) to 1
    flag |= 0x1;
    //printf("makeTriggerPulse: to   ioCtrl (0x%llx) value is 0x%x\n", IO_OUTP_CTRL2, flag);
    err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
    //printf("makeTriggerPulse: got  ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
    // and set it back to zero
    flag &= 0xfffffffe;
    err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
  }
  return err == DC1394_SUCCESS;
}


///////////////////////////////////////////////

bool getExposureTargetRaw()
{
  dc1394error_t err = DC1394_FAILURE;
  //
  fExposure.id = DC1394_FEATURE_EXPOSURE;
  if (camHandle != NULL)
  {
    err = dc1394_feature_get(camHandle, &fExposure);
    if (err != DC1394_SUCCESS)
      printf("getExposure: Failed to get exposure feature info\n");
  }
//     fExposure.value;
//     fExposure.min;
//     fExposure.max;
  return err == DC1394_SUCCESS;
}

///////////////////////////////////////////////

bool setExposureTargetRaw(int value)
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



