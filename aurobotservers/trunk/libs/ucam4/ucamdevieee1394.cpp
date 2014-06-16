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

#include <ugen4/uimage2.h>

#include "ucamdevieee1394.h"


UCamDevIeee1394::UCamDevIeee1394()
 : UCamDevBase()
{
#ifdef USE_IEEE1394
  //
  camHandle = NULL;
  usingDMA = false;
  camType = CAM_DEV_IEEE1394;
  stopFrameRead = true;
  threadRunning = false;
#endif
}

///////////////////////////////////////////

UCamDevIeee1394::~UCamDevIeee1394()
{
#ifdef USE_IEEE1394
  closeDevice();
  if (camHandle != NULL)
  {
    if (usingDMA)
      dc1394_dma_release_camera(camHandle,&camera);
    else
      dc1394_release_camera(camHandle,&camera);
    dc1394_destroy_handle(camHandle);
  }
  // stop read thread
  stop(true);
#endif
}

#ifdef USE_IEEE1394

///////////////////////////////////////////

int UCamDevIeee1394::getIeee1394PortCnt()
{
  const int MAX_IEEE1394_PORTS = 10;
  struct raw1394_portinfo ports[MAX_IEEE1394_PORTS];
  int portCnt = 0;
  raw1394handle_t handle;
  //
  handle = raw1394_new_handle();
  if (handle==NULL)
  {
    fprintf( stderr, "Unable to aquire a raw1394 handle - no camera?\n");
/*        "Please check \n"
            "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
            "  - if you have read/write access to /dev/raw1394\n\n");*/
  }
  /* get the number of ports (cards) and write info (up to 10) into 'ports' */
  if (handle != NULL)
  {
    portCnt = raw1394_get_port_info(handle, ports, MAX_IEEE1394_PORTS);
    dc1394_destroy_handle(handle);
  }
  /* look across all ports for cameras */
  return portCnt;
}

///////////////////////////////////////////

int UCamDevIeee1394::getIeee1394CamCnt(const int port)
{
  int camCnt = 0;
  raw1394handle_t handle;
  nodeid_t * camera_nodes = NULL;
  //
  handle = dc1394_create_handle(port);
  if (handle == NULL)
    fprintf( stderr, "Unable to aquire a raw1394 handle for interface card (port) %i\n", port);
  else
    camera_nodes = dc1394_get_camera_nodes(handle, &camCnt, 0);
  dc1394_destroy_handle(handle);
  return camCnt;
}

////////////////////////////////////////////

bool UCamDevIeee1394::setCamDeviceNode(int portNum, int camNum)
{
  nodeid_t * camera_nodes = NULL;
  bool result = false;
  int camCnt;
  const int MNL = MAX_CAM_DEV_NAME_LENGTH;
  // gte handle to camera bus (port)
  camHandle = dc1394_create_handle(portNum);
  if (camHandle != NULL)
  { // handle is OK, get camera info on this port,
    // now get node array
    camera_nodes = dc1394_get_camera_nodes(camHandle, &camCnt, 0);
    if (camNum < camCnt)
    { // reset camera
      dc1394_init_camera(camHandle, camera_nodes[camNum]);
      // get data on requested node
      if (dc1394_get_camera_info(camHandle, camera_nodes[camNum], &info) == DC1394_SUCCESS)
      { // print camera info - or part of it
        dc1394_print_camera_info(&info);
        // save camera name
        snprintf(camName, MNL, "%s, %s", info.vendor, info.model);
        // prepare camera capture structure
        camera.node = camera_nodes[camNum];
        // all is fine so far
        result = true;
      }
      else
        printf("IEEE1394 Device %d on port %d is not a valid camera\n", camNum, portNum);
    }
    else
      printf("IEEE1394 Device %d not found, just %d devices on port %d\n",
             camNum, camCnt, portNum);
  }
  else
    printf("IEEE1394 failed to create andle to port %d\n", portNum);
  return result;
}

////////////////////////////////////////////

void UCamDevIeee1394::getCamInfo()
{
  dc1394_camerainfo info;
  const int MNL = MAX_CAM_DEV_NAME_LENGTH;
  dc1394_feature_info feature_temp;
  dc1394_feature_info feature_expose;
  dc1394_feature_info feature_sharp;
  dc1394_feature_info feature_white;
  dc1394_feature_info feature_hue;
  dc1394_feature_info feature_saturation;
  dc1394_feature_info feature_gamma;
  dc1394_feature_info feature_brightness;
  dc1394_feature_info feature_shut;
  dc1394_feature_info feature_trigger;
  dc1394_feature_info feature_trig_delay;
  dc1394_feature_info feature_size;
  dc1394_feature_info feature_fps;
  bool isOK;
  int red, blue;
  //
  /* use the first camera found */
  if (camHandle != NULL)
  {
    if (dc1394_get_camera_info(camHandle, camera.node, &info) == DC1394_SUCCESS)
    {
      dc1394_print_camera_info(&info);
      snprintf(camName, MNL, "%s, %s", info.vendor, info.model);
    }
  }

  feature_expose.feature_id = FEATURE_EXPOSURE;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_expose);
  if (isOK)
  {
    dc1394_print_feature(&feature_expose);
    if ((feature_expose.available == DC1394_TRUE) and
         feature_expose.readout_capable == DC1394_TRUE)
    {
      vshutter = feature_expose.value;
    }
  }
  //
  feature_sharp.feature_id = FEATURE_SHARPNESS;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_sharp);
  if (isOK)
  {
    dc1394_print_feature(&feature_sharp);
    if (feature_sharp.available and feature_sharp.readout_capable)
    {
      vcontour = feature_sharp.value;
    }
  }
  //
  feature_brightness.feature_id = FEATURE_BRIGHTNESS;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_brightness);
  if (isOK)
  {
    dc1394_print_feature(&feature_brightness);
    if (feature_brightness.available and feature_brightness.readout_capable)
    {
      vbrightness = feature_brightness.value;
    }
  }
  //
  feature_white.feature_id = FEATURE_WHITE_BALANCE;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_white);
  if (isOK)
  {
    dc1394_print_feature(&feature_white);
    if (feature_white.available and feature_white.readout_capable)
    {
      red = feature_white.RV_value;
      blue = feature_white.BU_value;
    }
  }
  //
  feature_hue.feature_id = FEATURE_HUE;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_hue);
  if (isOK)
  {
    dc1394_print_feature(&feature_hue);
  }
  //
  feature_saturation.feature_id = FEATURE_SATURATION;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_saturation);
  if (isOK)
  {
    dc1394_print_feature(&feature_saturation);
    if (feature_saturation.available and feature_saturation.readout_capable)
      vcolour = feature_saturation.value;
  }
  //
  feature_gamma.feature_id = FEATURE_GAMMA;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                       &feature_gamma);
  if (isOK)
  {
    dc1394_print_feature(&feature_gamma);
    if (feature_gamma.available and feature_gamma.readout_capable)
      vgamma = feature_gamma.value;
  }
  //
  //
  feature_shut.feature_id = FEATURE_SHUTTER;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_shut);
  if (isOK)
  {
    dc1394_print_feature(&feature_shut);
    if (feature_shut.available and feature_shut.readout_capable)
      vshutter = feature_shut.value;
  }
  //
/*  feature.feature_id = FEATURE_GAIN;
  feature.feature_id = FEATURE_IRIS;
  feature.feature_id = FEATURE_FOCUS;*/
  //
  feature_temp.feature_id = FEATURE_TEMPERATURE;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_temp);
  if (isOK)
  {
    dc1394_print_feature(&feature_temp);
  }
  //
  feature_trigger.feature_id = FEATURE_TRIGGER;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_trigger);
  if (isOK)
  {
    dc1394_print_feature(&feature_trigger);
  }
  //
  feature_trig_delay.feature_id = FEATURE_TRIGGER_DELAY;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_trig_delay);
  if (isOK)
  {
    dc1394_print_feature(&feature_trig_delay);
  }
  /*feature.feature_id = FEATURE_WHITE_SHADING;
  feature.feature_id = FEATURE_FRAME_RATE;
  feature.feature_id = FEATURE_ZOOM;
  feature.feature_id = FEATURE_PAN;
  feature.feature_id = FEATURE_TILT;
  feature.feature_id = FEATURE_OPTICAL_FILTER; */
  feature_size.feature_id = FEATURE_CAPTURE_SIZE;
  //feature.feature_id = FEATURE_CAPTURE_QUALITY;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_size);
  if (isOK)
  {
    dc1394_print_feature(&feature_size);
  }
  //
  feature_fps.feature_id = FEATURE_FRAME_RATE;
  isOK = dc1394_get_camera_feature(camHandle, camera.node,
                                   &feature_fps);
  if (isOK)
  {
    dc1394_print_feature(&feature_fps);
  }
}

///////////////////////////////////////////////////

dc1394_feature_info * UCamDevIeee1394::getFeature(unsigned int featureId)
{
  dc1394_feature_info * feature;
  int i;
  //
  feature = features.feature;
  for (i = 0; i < 22; i++)
  {
    if (feature->feature_id == featureId)
      break;
    feature++;
  }
  return feature;
}

//////////////////////////////////////////////////

bool UCamDevIeee1394::setFormat7()
{
  unsigned int channel;
  unsigned int mode;
  unsigned int speed;
  bool result = false;
  unsigned int bytes_per_packet;
  // FORMAT 7
  if (true)
  {
    channel = 0;
    speed = 0;
    mode = 0;
    // get current channel and speed
    dc1394_get_iso_channel_and_speed(camHandle,camera.node, &channel,&speed);
    mode = MODE_FORMAT7_0;
    // get video mode
    dc1394_get_video_mode(camHandle,camera.node, &mode);
    // query current mode
    dc1394_query_format7_byte_per_packet(camHandle,camera.node, mode,
                                         &bytes_per_packet);
    // set new options
    mode = MODE_FORMAT7_0;
    speed =SPEED_400;
    //bytes_per_packet = QUERY_FROM_CAMERA; //1000;
/*    roiTop = QUERY_FROM_CAMERA;
    roiLeft = QUERY_FROM_CAMERA;
    roiWidth = QUERY_FROM_CAMERA;
    roiHeight = QUERY_FROM_CAMERA;*/
    //
    //bytes_per_packet = 1000;
    //bytes_per_packet = USE_MAX_AVAIL;
    roiTop = 0;
    roiLeft = 0;
    roiWidth = 640;
    roiHeight = 480;
    //roiWidth = QUERY_FROM_CAMERA;
    //roiHeight = QUERY_FROM_CAMERA;

    result = (dc1394_setup_format7_capture(
                               camHandle,
                               camera.node,
                               channel, /* channel */
                               mode , /*mode */
                               speed,
                               bytes_per_packet ,
                               roiLeft ,/*area of interest start column */
                               roiTop, /*area of interest start row */
                               roiWidth,/* area of interest width */
                               roiHeight, /* area of interest height */
                               &camera /* dc1394_cameracapture type pointer */
                                    ) == DC1394_SUCCESS);
    if (not result)
    {
      printf("Failed to set format 7\n");
    }
    else
      printf("Finished setting format-7\n");
  }
/*  if (result)
  {
    if (dc1394_start_iso_transmission(camHandle,camera.node) !=DC1394_SUCCESS)
    {
      fprintf( stderr, "unable to start camera iso transmission\n");
      result = false;
    }
    else
    {
      printf("Started iso-transmission in format-7 (0)\n");
    }
  }*/
  usingDMA = false;

  return result;
}

////////////////////////////////////////////////////////

bool UCamDevIeee1394::getSingleImage(UImage * destination)
{
  bool result;
  int j;
  int bytes, n;
  UImage * img = destination;
  //const int MSL = 50;
  //char s[MSL];
  //
  if (usingDMA)
  { // DMA capture
    result = dc1394_dma_single_capture(&camera) == DC1394_SUCCESS;
    if (not result)
    {
      fprintf(stderr, "dma1394; DMA, can't capture a single frame.\n");
      result = false;
    }
  }
  else
  {
    //  dc1394_single_capture(handle,&camera)!=DC1394_SUCCESS)
    result = dc1394_single_capture(camHandle ,&camera) == DC1394_SUCCESS;
    if (not result)
    {
      fprintf( stderr, "unable to capture a frame (not using DMA)\n");
    }
  }

  if (result)
  {
    // debug
/*    printf("Captured image from %s (size %dx%d)\n", getName(), camera.frame_height, camera.frame_width);
    printf("pixels 1: ");
    for (j = 0; j < 20; j++)
      printf("%3d ", ((const unsigned char *)camera.capture_buffer)[j]);
    printf("\n");
    printf("pixels 2: ");
    for (j = 0; j < 20; j++)
      printf("%3d ", ((const unsigned char *)camera.capture_buffer)[camera.frame_width + j]);
    printf("\n");
    printf("pixels 3: ");
    for (j = 0; j < 20; j++)
      printf("%3d ", ((const unsigned char *)camera.capture_buffer)[camera.frame_width*2 + j]);
    printf("\n");*/
    // debug end;
    if (img != NULL)
    {
      img->setSize(camera.frame_height, camera.frame_width, 1, 8, "BGGR");
      j = getBayerPattern();
      if (j >= 0)
        // camera knows the Bayer type to tell
        img->setColorType(j);
      if (usingDMA)
        n = camera.dma_frame_size;
      else
        n = camera.frame_height * camera.frame_width;
      bytes = img->getBufferSize();
      if (n <= bytes)
      {
        memcpy(img->getData(), camera.capture_buffer, n);
      }
      img->camDevice = getDeviceNumber();
      img->cam = NULL;
      img->imgTime.setTimeU(camera.filltime.tv_sec, camera.filltime.tv_usec);
      // debug
      //img->imgTime.getTimeAsString(s, true);
      //printf("frame time %s  %lu.%06lu serial %lu\n", s,
      //       img->imgTime.getSec(), img->imgTime.getMicrosec(), imageNumber);
      // debug end
      //
      img->imageNumber = imageNumber++;
      img->valid = true;
      while (((int)img->getWidth() > frameWidth) and (frameWidth > 25))
      { // reduce size
        img->toHalf();
      }

    }
    else
    { // just increase image number
      imageNumber++;
    }
  }

  if (usingDMA and result)
  { // we need to dispose the DMA buffer
    // printf("Releasing the capture buffer.\n");
    dc1394_dma_done_with_buffer(&camera); /*important step */
  }
  return result;
}

/////////////////////////////////////////////////////////////

bool UCamDevIeee1394::setFormat0mode5()
{
  unsigned int channel = 0;
  unsigned int mode = MODE_640x480_MONO;
  unsigned int format = FORMAT_VGA_NONCOMPRESSED;
  unsigned int speed = SPEED_400;
  bool result = true;
  unsigned int frame_rate = FRAMERATE_30; //FRAMERATE_7_5;
//  unsigned int bytes_per_packet;
  quadlet_t quadlet;
  //
  if (camHandle != NULL)
  {
    dc1394_query_supported_formats(camHandle,camera.node,&quadlet);
    if (quadlet & (1<<31))
    { // check for VGA format
      dc1394_query_supported_modes(camHandle,camera.node,FORMAT_VGA_NONCOMPRESSED,&quadlet);
      if (quadlet & (1<< (31-5))) {
        fprintf(stderr,"Using format0/mode5\n");
        format=FORMAT_VGA_NONCOMPRESSED;
        mode=MODE_640x480_MONO;
      }
    }
    else
    {
      fprintf(stderr,"Your camera does not seem to support an 8-bit grayscale format.\nAborting...\n");
      result = false;
    }

    /*-----------------------------------------------------------------------
    *  setup capture
    *-----------------------------------------------------------------------*/
    if (result)
    {
      if (dc1394_setup_capture(camHandle,camera.node,
          channel, /* channel */
          format,
          mode,
          speed,
          frame_rate,
          &camera) != DC1394_SUCCESS)
      {
        fprintf( stderr,"unable to setup camera-\n"
            "check line %d of %s to make sure\n"
                "that the video mode,framerate and format are\n"
                "supported by your camera\n",
                __LINE__,__FILE__);
        result = false;
      }
    }
  }
  usingDMA = false;
  return result;
}

///////////////////////////////////////////////////////////

bool UCamDevIeee1394::getCamFeatures()
{
  bool result;
  //
  result = (dc1394_get_camera_feature_set(camHandle, camera.node,&features) ==DC1394_SUCCESS);
  if (not result)
  {
    fprintf( stderr, "unable to get feature set\n");
  }
  else
  {
    //dc1394_print_feature_set(&features);
  }
  return result;
}

////////////////////////////////////////////////////////////////

//#include <dc1394/dc1394_control.h>
//#include <dc1394/dc1394_conversions.h>
//#include <dc1394/dc1394_utils.h>
//#include <dc1394/dc1394_register.h>

bool UCamDevIeee1394::startIsoTransmission()
{
  bool result;
  //dc1394switch_t pwr;
  //
  if (true)
  { // may not be needed
    result = dc1394_set_trigger_mode(camHandle,camera.node,TRIGGER_MODE_0) == DC1394_SUCCESS;
    //
    result = dc1394_start_iso_transmission(camHandle,camera.node)  == DC1394_SUCCESS;
    if (not result)
    {
      fprintf( stderr, "unable to start camera iso transmission\n");
    }
  }
  else
  {
    //dc1394error_t dc1394_video_set_transmission(dc1394camera_t *camera, dc1394switch_t pwr);
    //dc1394error_t dc1394_video_get_transmission(camera.node, &pwr);

  }
  return result;
}

//////////////////////////////////////////////////////////////

bool UCamDevIeee1394::stopIsoTransmission()
{
  bool result = false;
  //
  if (cameraOpen)
  {
    lock();
    cameraOpen = false;
    // wait for any last image to be captured
    Wait(0.2);
    /*Step 6: Stop sending data*/
    /*==================================================*/
    if (usingDMA)
    {
      if (dc1394_dma_unlisten(camHandle, &camera) != DC1394_SUCCESS)
      {
        fprintf(stderr, "UCamDevIeee1394::stopIsoTransmission(): Can't unlisten iso DMA channel!\n");
        result = false;
      }
    }

    result = dc1394_stop_iso_transmission(camHandle,camera.node) == DC1394_SUCCESS;
    if (not result)
    {
      printf("UCamDevIeee1394::stopIsoTransmission(): couldn't stop the camera?\n");
    }
    unlock();
  }
  else
    printf("UCamDevIeee1394::stopIsoTransmission(): camera is closed already\n");

  return result;
}

///////////////////////////////////////////////////////////


bool UCamDevIeee1394::setDMAcapture()
{
  unsigned int channel = 1;
  unsigned int speed = SPEED_400;
  unsigned int format = FORMAT_VGA_NONCOMPRESSED;
  unsigned int mode = MODE_640x480_MONO;
  unsigned int framerate = FRAMERATE_30;
  bool result = true;
/*  unsigned int mode = MODE_640x480_MONO;
  unsigned int format = FORMAT_VGA_NONCOMPRESSED;
  unsigned int speed = SPEED_400;
  bool result = true;
  unsigned int frame_rate = FRAMERATE_30; //FRAMERATE_7_5;*/
  /*Step 3: Setup Capture*/
  /*=====================================================================*/
  /*Using camera functions to get the params by querying them*/
  // debug
  //printf("INFO FOR DEBUG: num_dma_buffers: %d \n", camera.num_dma_buffers);
  // debug end
  dc1394_get_iso_channel_and_speed(camHandle, camera.node, &channel, &speed); /*get channel and speed*/
  //dc1394_get_video_format(camHandle, camera.node, &format); /*get format*/
  //dc1394_get_video_framerate(camHandle, camera.node, &framerate); /*get framerate*/
  //dc1394_get_video_mode(camHandle, camera.node, &mode); /*get mode*/
  // debug
/*  printf("dc1394: Got parameters from the camera.\n"
      "=======================================\n"
      "Channel:   %d\n"
      "Speed:     %d\n"
      "Format:    %d\n"
      "Framerate: %d\n"
      "Mode:      %d\n", channel, speed, format, framerate, mode);*/
  //if (framerate <
  // debug end
  if (frameRate < 30)
  { // less than 30 fps wont work (camera stops)
    framerate = FRAMERATE_30;
    frameRate = 30;
  }
  else
  { // convert to bit-position (allowed values)
    framerate = fps2mode5framerate(frameRate);
    // and back to frames per second
    frameRate = framerate2fps(framerate);
  }
  //
  camera.num_dma_buffers = 8; /* set the dma buffers */
  camera.drop_frames = 1; /* set the number of drop frames */
  camera.dma_device_file = NULL;
  if (dc1394_dma_setup_capture(camHandle,
      camera.node,
      channel,
      format,
      mode,
      speed,
      framerate,
      camera.num_dma_buffers,
      camera.drop_frames,
      camera.dma_device_file,
      &camera) !=DC1394_SUCCESS)
  {
      fprintf(stderr, "dma1394: Unable to setup camera.\n"
          "Check line %d of %s to ensure that the options set are supported by your camera.\n", __LINE__, __FILE__);
      result = false;
  }
  else
  {
    //printf("dma1394: Capture has been setup.\n");
  }
  if (result)
  { //Set Trigger Mode -- Generally not required thus I will comment it out.
    if (dc1394_set_trigger_mode(camHandle, camera.node, TRIGGER_MODE_0) != DC1394_SUCCESS)
    {
      fprintf(stderr, "dma1394: Unable to set the camera trigger mode. Refer to line %d in %s.\n", __LINE__, __FILE__);
      result = false;
    }
    else
    {
      //printf("dma1394: Successfully set trigger mode.\n");
    }
  }
  if (result)
  { /*Step 4: Start sending data */
    /*=======================================================*/
    if (dc1394_start_iso_transmission(camHandle, camera.node) != DC1394_SUCCESS)
    {
      fprintf(stderr, "dma1394: Unable to start the data transmission.\n");
      result = false;
    }
    else
    {
      usingDMA = true;
      cameraOpen = true;
      printf("dma1394: Success.  Data Transmission started (device %d).\n", getDeviceNumber());
    }
  }
  return result;
}

/////////////////////////////////////////////////

bool UCamDevIeee1394::setGain(int agc)
{
  int v;
  dc1394_feature_info * feature;
  bool result;
  //
  if (agc == -1)
    result = dc1394_auto_on_off(camHandle, camera.node, FEATURE_GAIN, 1) == DC1394_SUCCESS;
  else
  {
    feature = getFeature(FEATURE_GAIN);
    if (feature->auto_active == DC1394_TRUE)
    {
      result = dc1394_auto_on_off(camHandle, camera.node, FEATURE_GAIN, 0) == DC1394_SUCCESS;
      if (not result)
        printf("Failed to turn off automatic gain\n");
    }
    // convert from scale as if 16 bit integer to usefull range
    v = (agc * feature->max) >> 16;
    if (v < feature->min)
      v = feature->min;
    result = dc1394_set_gain(camHandle, camera.node, v) == DC1394_SUCCESS;
  }
  getGain(true, NULL);
  return result;
}

/////////////////////////////////////////////////

int UCamDevIeee1394::getGain(bool probe, bool * dataValid, bool * isOnAuto)
{
  bool isOK;
  int result = 0;
  dc1394_feature_info * feature;
  //
  feature = getFeature(FEATURE_GAIN);
  isOK = ((camHandle != NULL) and (feature->max > 0));
  if (isOK)
  { // camera and data is available
    if (probe)
      isOK = dc1394_get_camera_feature(camHandle, camera.node, feature);
    if (feature->readout_capable == DC1394_FALSE)
      result = -1;
    else
      // scale for 0xFFFF range
      result = (feature->value << 16) / feature->max;
    if (isOnAuto != NULL)
      *isOnAuto = (feature->auto_active == DC1394_TRUE);
  }
  if (dataValid != NULL)
    *dataValid = isOK;
  return result;
}

/////////////////////////////////////////////////

bool UCamDevIeee1394::setShutter(int value)
{
  int v;
  dc1394_feature_info * feature;
  bool result;
  //
  if (value == -1)
    result = dc1394_auto_on_off(camHandle, camera.node, FEATURE_SHUTTER, 1) == DC1394_SUCCESS;
  else
  {
    feature = getFeature(FEATURE_SHUTTER);
    if (feature->auto_active == DC1394_TRUE)
    {
      result = dc1394_auto_on_off(camHandle, camera.node, FEATURE_SHUTTER, 0) == DC1394_SUCCESS;
      if (not result)
        printf("Failed to turn off automatic shutter\n");
    }
    // convert from scale as if 16 bit integer to usefull range
    v = (value * feature->max) >> 16;
    if (v < feature->min)
      v = feature->min;
    result = dc1394_set_shutter(camHandle, camera.node, v) == DC1394_SUCCESS;
  }
  // get new value from camera
  getShutter(true, NULL);
  return result;
}

/////////////////////////////////////////////////

int UCamDevIeee1394::getShutter(bool probe, bool * dataValid, bool * isOnAuto)
{
  bool isOK;
  int result = 0;
  dc1394_feature_info * feature;
  //
  feature = getFeature(FEATURE_SHUTTER);
  // feature->max should always be > 0 if available from camera
  isOK = ((camHandle != NULL) and (feature->max > 0));
  if (isOK)
  { // camera and feature is available
    if (probe)
      // ask camera (for fresh data)
      isOK = dc1394_get_camera_feature(camHandle, camera.node, feature);
    if (feature->readout_capable == DC1394_FALSE)
      result = -1;
    else
      // scale for 0xFFFF range
      result = (feature->value << 16) / feature->max;
    if (isOnAuto != NULL)
      *isOnAuto = (feature->auto_active == DC1394_TRUE);
  }
  if (dataValid != NULL)
    *dataValid = isOK;
  return result;
}

//////////////////////////////////////////////////

int UCamDevIeee1394::getBayerPattern()
{
  int result = 0;
  //bayer_pattern_t pattern = BAYER_PATTERN_BGGR;
  int err;
  quadlet_t qValue = 0;
  //
  err = GetCameraControlRegister( camHandle,
                          camera.node,
                          0x1040,/* Bayer Tile Mapping register */
                          &qValue );
  if (err == 0)
  {
    switch( qValue )
    {
      case 0x42474752:/* BGGR */
        //pattern = BAYER_PATTERN_BGGR;
        result = PIX_PLANES_BGGR;
        break;
      case 0x47524247:/* GRBG */
        //pattern = BAYER_PATTERN_GRBG;
        result = PIX_PLANES_GRBG;
        break;
      case 0x52474742: /* RGGB */
        //pattern = BAYER_PATTERN_RGGB;
        result = PIX_PLANES_RGGB;
        break;
      case 0x47425247:/* GBRG */
        //pattern = BAYER_PATTERN_GBRG;
        result = PIX_PLANES_GBRG;
        break;
      case 0x59595959:/* YYYY = BW */
        fprintf( stderr, "Camera is black and white\n" );
        result = PIX_PLANES_BW;
        break;
      default:
        fprintf(stderr,
                "Camera BAYER_TILE_MAPPING register has an unexpected value:\n"
                    "\t0x%x\n", qValue );
    }
  }
  else
    // assume one of the Bayer patterns
    result = -PIX_PLANES_RGGB;

   return result;
}

//////////////////////////////////////////

// int UCamDevIeee1394::getBayerPattern()
// {
//   int result = 0;
//   //bayer_pattern_t pattern = BAYER_PATTERN_BGGR;
//   int err;
//   const int MBL = 5;
//   char bayer[MBL];;
//   //
//   err = GetCameraControlRegister( camHandle,
//                                   camera.node,
//                                   0x1040,/* Bayer Tile Mapping register */
//                                   (quadlet_t *) bayer );
//   if (err == 0)
//   {
//     bayer[4] = '\0';
//     printf("UCamDevIeee1394::getBayerPattern: Bayer order is %s\n", bayer);
//     result = PIX_PLANES_BGGR;
//   }
//   else
//     // assume one of the Bayer patterns
//     result = -PIX_PLANES_RGGB;
//
//   return result;
// }


bool UCamDevIeee1394::deviceExist()
{ // open device and get default parameters
  bool result;
  //
  int d, n, i, c, j;
  //
  result = camHandle != NULL;
  if (not result)
  { // not already in action, so test interface card for a device
    lock();
    n = getIeee1394PortCnt();
    d = 10;
    //printf("Found %d IEEE1394 interface cards (ports) in search for device %d\n", n, getDeviceNumber());
    for (i =0; i < n; i++)
    {
      c = getIeee1394CamCnt(i);
      //printf(" - port %d has %d camera node(s)\n", i, c);
      // test if camera device number is on this interface card
      if ((d + c) <= getDeviceNumber())
      { // no - continue with next interface card
        d += c;
        continue;
      }
      else if (d > getDeviceNumber())
        // not found
        break;

      // the camera is on this card
      for (j = 0; j < c; j++)
      {
        if (d == getDeviceNumber())
        { // camera is found
          setCamDeviceNode(i, j);
          //dev1394->getCamInfo();
          getCamFeatures();
          printf("Found device %d as a '%s'\n", d, getName());
          result = true;
          // finished
          break;
        }
        d++;
      }
      if (result)
        break;
    }
  }
  unlock();

  //
/*  lock();
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
  unlock();*/
  return result;
}

////////////////////////////////////////////////////

bool UCamDevIeee1394::openDeviceDefault()
{
  bool result;
  //
  result = camHandle != NULL;
  if (result)
  {
    result = setDMAcapture();
    if (not threadRunning)
      start();
      if (frameHeight < 0)
        frameHeight = 480;
      if (frameWidth < 0)
        frameWidth = 640;
  }
  return result;
}

////////////////////////////////////////////////////

void UCamDevIeee1394::closeDevice()
{
  if (cameraOpen)
  { // stop iso_transmission
    stopIsoTransmission();
  }
}

////////////////////////////////////////////////////

bool UCamDevIeee1394::getImageSnapshot(UImage * image)
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
    printf("UCamDevIeee1394::getImageSnapshot: Gave up "
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

int UCamDevIeee1394::getFrameRate()
{
  unsigned int fr;
  if (dc1394_get_video_framerate(camHandle, camera.node, &fr) == DC1394_SUCCESS)
    frameRate = framerate2fps(fr);
  return frameRate;
}

//////////////////////////////////////////////////////////

int UCamDevIeee1394::framerate2fps(int framerate)
{
  int fps;
  switch(framerate)
  {
    case FRAMERATE_1_875: fps = 2; break;
    case FRAMERATE_3_75: fps = 4; break;
    case FRAMERATE_7_5: fps = 7; break;
    case FRAMERATE_15: fps = 15; break;
    case FRAMERATE_30: fps = 30; break;
    case FRAMERATE_60: fps = 60; break;
    case FRAMERATE_120: fps = 120; break;
    case FRAMERATE_240: fps = 240; break;
    default: fps = 10; break;
  }
  return fps;
}

///////////////////////////////////////////////////////////

int UCamDevIeee1394::fps2mode5framerate(int fps)
{
  int fr;
  if (fps < 3)
    fr = FRAMERATE_1_875;
  else if (fps <= 4)
    fr = FRAMERATE_3_75;
  else if (fps <= 9)
    fr = FRAMERATE_7_5;
  else if (fps <= 15)
    fr = FRAMERATE_15;
  else if (fps <= 30)
    fr = FRAMERATE_30;
  else if (fps <= 60)
    fr = FRAMERATE_60;
  else if (fps <= 120)
    fr = FRAMERATE_120;
  else
    fr = FRAMERATE_240;
  return fr;
}

///////////////////////////////////////////////////////////

bool UCamDevIeee1394::setDevice(const int width, const int height,
               const int framesPerSec)
{
  bool result = false;
  int frNy, frCurr;
  //
  frameWidth = width;
  frameHeight = height;
  frNy = fps2mode5framerate(framesPerSec);
  frCurr = fps2mode5framerate(frameRate);
  if (not cameraOpen)
    openDeviceDefault();
  if (frNy != frCurr)
  { // change framerate
    if (dc1394_set_video_framerate(camHandle, camera.node, frNy) != DC1394_SUCCESS)
      getFrameRate();
    else
      frameRate = framesPerSec;
  }
  else
    result = true;
  imageSizeChanged(MAX_IMAGE_WIDTH / float(frameWidth));
  return result;
}

////////////////////////////////////////////////////

void * runIeee1394CamThread(void * camobj)
{ // Start thread here and call thread function
  UCamDevIeee1394 * camDev;
  //
  camDev = (UCamDevIeee1394 *) camobj;
  camDev->run();
  pthread_exit((void*)NULL);
  return NULL;
}

////////////////////////////////////////////////////

bool UCamDevIeee1394::start()
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

void UCamDevIeee1394::stop(bool andWait)
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

////////////////////////////////////////////////////////////

void UCamDevIeee1394::run()
{
  UImage * img = NULL;
  bool needSingle;
  bool need4Push;
  bool gotOK;
  UTime t;
  int i = 0;
  int failCnt = 0;
/*  const int MSL = 50;
  char s[MSL];*/
  //
  if (imgBuff[0] == NULL)
  { // make push buffer (if needed)
    imgBuff[0] = new UImage800();
    imgBuff[1] = imgBuff[0];
    printf("camera made its own push buffer image - should be from image pool\n");
  }
  // initialize push buffer
  imgBuff[0]->camDevice = devNum;
  imgBuff[0]->setSize(480, 640, 3, 8, "RGB");
  imgBuff[0]->valid = false;
  //
  threadRunning = true;
  // wait for camera to be initialized
  while (not initialized and not stopFrameRead)
    Wait(0.1);
  if (not stopFrameRead)
  {  // empy the capture post flag
    captureDo.tryWait();
    // and the done flag
    captureDone.tryWait();
  }
  // debug timing
  t.now();
  // debug timing end
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
        // printf("Image serial %lu, need single image, img=%x\n",
        //       imageNumber, (unsigned int)img);
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
      gotOK = getSingleImage(img);
      if (need4Push and gotOK)
      {
        img->updated();
        gotNewImage(img);
      }
      if (needSingle)
      {
        captureDone.post();
      }
      // debug
      if (false and (imageNumber % 120 == 0))
      {
        printf("UCamDevIeee1394::run: timePassed %f sec - now %lu.%06lu - serial %lu\n",
              t.getTimePassed(), t.getSec(), t.getMicrosec(), imageNumber);
      }
      t.now();
      // debug end
      if (gotOK)
      {
        failCnt = 0;
        i = frameRate;
      }
      else
      { // failed to get image - report
        failCnt++;
        if (failCnt > i)
        {
          printf("Failed to get image %d images - about to close device=%d\n",
                 i, devNum);
          i += frameRate;
        }
        if (failCnt > (4 * frameRate))
        {
          printf("closing device %d\n", devNum);
          closeDevice();
          failCnt = 0;
          i = frameRate;
        }
      }
    }
    else
    {
      Wait(0.1);
    }
  }
  threadRunning = false;
}


void UCamDevIeee1394::setPushBuffer(UImage * imageBuffer)
{
  if (imageBuffer == NULL)
    // no buffer available - so stop camera
    stop(true);
  lock();
  // replace (or set) image buffer
  imgBuff[0] = imageBuffer;
  if (imgBuff[1] != NULL)
  { // temporary buiffer were created - remove
    delete imgBuff[1];
    imgBuff[1] = NULL;
  }
  unlock();
}

#endif

