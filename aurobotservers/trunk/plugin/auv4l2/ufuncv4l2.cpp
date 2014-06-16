/***************************************************************************
 *   Copyright (C) 2011 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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

/// @todo - not in a working state

#include <ugen4/ucommon.h>
#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>
#include <urob4/uimagepool.h>
#include "defs.h"
//#include <libavcodec/avcodec.h>
#include "ufuncv4l2.h"
#include "v4l2uvc.h"
#include "ms_time.h"
#include "colorspaces.h"

#define NB_BUFFER 4


#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncv4l2' with your classname, as used in the headerfile */
  return new UFuncv4l2();
}

#endif


void UFuncv4l2::init()
{
  varDevice = NULL;
  varOpen = NULL;
  varTime = NULL;
  varCnt = NULL;
  varErrCnt = NULL;
  varErrMsg = NULL;
  varUpdateRate = NULL;
  varDataSample = NULL;
  varPoolImage = NULL;
  poolImageNum = 11;
  poolImage = NULL;
  threadRunning = false;
  threadStop = false;
  initGlobals(&global);
  strncpy(videoDevice, "/dev/video0", MAX_DEV_NAME_CNT);
  global.videodevice = videoDevice;

  // must be called before using avcodec lib
  //avcodec_init();

  // register all the codecs (you can also register only the codec
  //you wish to have smaller code
  //avcodec_register_all();
  isOpen = false;
}

///////////////////////////////////////////////////

bool UFuncv4l2::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 500;
  char reply[MRL];
  bool silent = false;
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("v4l2");
    sendText("--- v4l2 is an interface plug-in for camera interface\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    snprintf(reply, MRL, "dev='device'        The v4l2 video device, is %s\n", global.videodevice);
    sendText(reply);
    snprintf(reply, MRL, "mode='shortname'    Video mode from camera, is %s\n", global.mode);
    sendText(reply);
    snprintf(reply, MRL, "w=width             Camera image width, is %d\n", global.width);
    sendText(reply);
    snprintf(reply, MRL, "h=height            Camera image height, is %d\n", global.height);
    sendText(reply);
    snprintf(reply, MRL, "open[=false]        Open camera (open=%s)\n", bool2str(varIsOpen->getBool()));
    sendText(reply);
    sendText("help       This message\n");
    sendText(            "silent              Make less print to console\n");
    sendText("----\nsee also 'var imu' for status and opter options\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    bool doLog, doOpen;
    msg->tag.getAttBool("silent", &silent, true);
    if (msg->tag.getAttValue("dev", videoDevice, MAX_DEV_NAME_CNT))
    {
      global.videodevice = videoDevice;
      snprintf(reply, MRL, "Set device name to %s", videoDevice);
      sendInfo(reply);
    }
    if (msg->tag.getAttValue("mode", global.mode, 5))
    {
      char gm[6];
      global.format = get_PixFormat(global.mode);
      if (get_PixMode(global.format, gm) == 1)
        snprintf(reply, MRL, "Set mode to %s (%d)", global.mode, global.format);
      else
      {
        snprintf(reply, MRL, "Unknown %s mode, set to %s (%d)", global.mode, gm, global.format);
        strncpy(global.mode, gm, 6);
      }
      sendInfo(reply);
    }
    if (msg->tag.getAttInteger("w", &global.width, 320))
    {
      sendInfo("done, set at next open");
    }
    if (msg->tag.getAttInteger("h", &global.height, 240))
    {
      sendInfo("done, set at next open");
    }
    if (msg->tag.getAttBool("open", &doOpen, true))
    {
      //global.videodevice = "/dev/video0";
      //global.width = 640;
      //global.height = 480;
      global.vidFPath[1] = imagePath;
      global.imgFPath[1] = imagePath;
      global.profile_FPath[1] = dataPath;
      global.vidFPath[0] = aliasName;
      global.imgFPath[0] = (char*)"noname.jpg";
      global.debug = not silent;
      snprintf(confFileName, MAX_FILENAME_CNT, "%s.gpfl", aliasName);
      global.profile_FPath[0] = confFileName;
      //
      if (initDevice())
      {
        int n;
        global.video_buff_size = (global.fps) / (global.fps_num); // enough for 1 sec
        n = sizeof(VidBuff) * global.video_buff_size;
        global.videoBuff = (VidBuff *) realloc(global.videoBuff, n);
        memset(global.videoBuff, 0, n);
        videoIn.capVid = true;
        isOpen = true;
        if (not silent)
          sendInfo("opened");
      }
      else
        sendWarning("failed to open device");
      //enum_devices();
    }
    //
    // implement command
    if (msg->tag.getAttBool("log", &doLog, true))
    { // open or close the log
      bool isOK = logf.openLog(doLog);
      if (doLog)
        snprintf(reply, MRL, "Opened logfile %s (%s)", logf.getLogFileName(), bool2str(isOK));
      else
        snprintf(reply, MRL, "closed logfile %s", logf.getLogFileName());
      // send an information tag back to client
      sendInfo(reply);
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncv4l2::createResources()
{
  varTime = addVar("time", 0.0, "t", "(r) Time of last update");
  varCnt = addVar("cnt", "1", "d", "(r) Update count");
  varDevice = addVar("device", "/dev/ttyUSBS0", "s", "(rw) device name - not used");
  varCamDev = addVar("camDevNum", 11, "d", "(rw) camera device number (into camera pool)");
  varOpen = addVar("open", 0.0, "d", "(rw) is device to be open or closed");
  varIsOpen = addVar("isopen", 0.0, "d", "(r) is device open (0=closed, 1=open (no data), 2=receiving)");
  varErrCnt = addVar("errCnt", 0.0, "d", "(r) number of format errors");
  varErrMsg = addVar("errMsg", "no error", "s", "(r/w) last error message");
  varUpdateRate = addVar("updaterate", 0.0, "d", "(r) Update rate in Hz");
  varDataSample = addVar("dataSample", "none", "s", "(r) sample of inputdata every 10 sec");
  varPoolImage = addVar("poolImage", "4 5", "d", "(r/w) image pool number to use (unpacked, packed (jpeg)");
  varImageFormat = addVar("colorFormat", 5, "d", "(r/w) desired unpacked color format "
                                   "(0=BW, 1=RGB, 2=RGB half, 3=YUV, 4=YUV half"
                                   ", 5=YUV422 (default))");
// set as time stamped variable
//  varAcc->setHasTime(30.0, 30*100, false);
  // start the receive thread
  start();
}

////////////////////////////////////////////////////////////////

/*
Output format by Sparkfun's "9DOF Razor IMU" (rev 1.0 demo software)
!ANG:-0.48,0.37,23.70,AN:388,384,381,-4,-1,257,346,-191,744
!ANG:-0.47,0.37,23.76,AN:388,384,380,-4,-2,257,346,-191,744
!ANG:-0.51,0.36,23.83,AN:388,384,381,-6,-2,260,346,-191,744
!ANG:-0.52,0.41,23.89,AN:388,384,380,-2,0,258,342,-186,735
!ANG:-0.50,0.35,23.94,AN:388,384,381,-3,-1,258,342,-186,735
}*/

///////////////////////////////////////////////////////////////

void * startUFuncv4l2Thread(void * obj)
{ // call the hadling function in provided object
  UFuncv4l2 * ce = (UFuncv4l2 *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UFuncv4l2::start()
{
  bool result = true;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandle, &thAttr,
              &startUFuncv4l2Thread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////

void UFuncv4l2::stop(bool andWait)
{
  if (threadRunning and not threadStop)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////

void UFuncv4l2::run()
{

    //struct JPEG_ENCODER_STRUCTURE *jpg_data=NULL;
    //
    pthread_mutex_lock(&videoIn.mutex);
        videoIn.IOfinished=false;
    pthread_mutex_unlock(&videoIn.mutex);
    gboolean failed = false;
    //buffers to be processed (video and audio)
    int frame_size = 0;
    VidBuff *proc_buff = NULL;
    UImagePool * imgpool = NULL;
    //
    frame_size = global.height * global.width * 2;
    proc_buff = (VidBuff *) malloc(sizeof(VidBuff));
    proc_buff->frame = (BYTE *) malloc(sizeof(BYTE) * frame_size);

    if(!failed)
    {
      int err;
      threadRunning = true;
      while(not threadStop)
      {
        if (isOpen)
        {   //process video
          int jbuffCnt = 0;
          unsigned char * jbuff = NULL;
          if (varPoolImage->getInt(1) != jpegImageNum or jpegImage == NULL)
          {
            jpegImageNum = varPoolImage->getInt(1);
            imgpool = (UImagePool *) getStaticResource("imgpool", false, false);
            if (imgpool != NULL)
            {
              jpegImage = imgpool->getImage(jpegImageNum, true);
              jpegImage->setName("v4l2-jpeg");
              jpegImage->camDevice = varCamDev->getInt();
            }
          }
          if (jpegImage != NULL)
          { // jpeg image is desired - get buffer
            if (jpegImage->tryLock())
              jbuff = jpegImage->getUCharRef(0, 0);
            else
              jbuff = NULL;
            if (jbuff != NULL)
              jbuffCnt = jpegImage->getBufferSize();
          }
          err = uvcGrab(&videoIn, global.format, global.width, global.height,
                        &global.fps, &global.fps_num, jbuff, &jbuffCnt);
          // if OK, then image is in videoIn.mem[videoIn.buf.index] and has size bytesUsed
          // and moved to videoIn.tmpbuffer, and
          // converted to yuyv format into videoIn.framebuffer - (for no reason?)
          if (jbuff != NULL and err != 0)
            jpegImage->unlock();
          if (not err)
          { // det her skal vi vist ikke bruge
            if (jpegImage != NULL)
            { // jpeg image is available
              if (jbuffCnt > 0)
              {
                jpegImage->imageNumber = global.image_inc;
                jpegImage->imgTime.SetTime(videoIn.timestamp);
                //jpegImage->getIplImage()->size = jbuffCnt;
                jpegImage->source = varCamDev->getInt();
                jpegImage->updated();
              }
              jpegImage->unlock();
            }
            // process_video (proc_buff, &(jpg_data));
            global.image_inc++;
            videoIn.capImage = true;
            if (varPoolImage->getInt(0) != poolImageNum or poolImage == NULL)
            {
              poolImageNum = varPoolImage->getInt(0);
              imgpool = (UImagePool *) getStaticResource("imgpool", false, false);
              if (imgpool != NULL)
              {
                poolImage = imgpool->getImage(poolImageNum, true, global.height, global.width);
                poolImage->setName("v4l2-image");
              }
            }
            if (poolImage != NULL)
            { // there is a destination, so update
              if (poolImage->tryLock())
              { // image is clocked - copy from framebuffer
                poolImage->imgTime.SetTime(videoIn.timestamp);
                poolImage->camDevice = varCamDev->getInt();
                poolImage->imageNumber = global.image_inc;
                int n = *videoIn.buff_length;
                if (n <= (int)poolImage->getBufferSize())
                {
                  switch (varImageFormat->getInt())
                  {
                    case 0: // BW
                      poolImage->yuv422toBW(videoIn.framebuffer, global.height, global.width);
                      break;
                    case 1: // RGB full resolution
                      poolImage->yuv422toRGB(videoIn.framebuffer, global.height, global.width);
                      break;
                    case 2: // bgr in full resolution
                      poolImage->yuv422toRGBhalf(videoIn.framebuffer, global.height, global.width);
                      break;
                    case 3: // yuv in full resolution
                      poolImage->yuv422toYUV(videoIn.framebuffer, global.height, global.width);
                      break;
                    case 4: // yuv in half resolution
                      poolImage->yuv422toYUVhalf(videoIn.framebuffer, global.height, global.width);
                      break;
                    case 5: // yuv422
                      poolImage->setSize(global.height, global.width, 2, 8, "YUV422");
                      memcpy(poolImage->getData(), videoIn.framebuffer, n);
                      break;
                    case 6: // jpeg
                      poolImage->setSize(global.height, global.width, 1, 8, "JPEG");
                      memcpy(poolImage->getData(), videoIn.framebuffer, n);
                      break;
                    default:
                      poolImage->setSize(global.height, global.width, 2, 8, "YUV22");
                      memcpy(poolImage->getData(), videoIn.framebuffer, n);
                      break;
                  }
                }
                poolImage->updated();
                poolImage->unlock();
              }
            }
          }
        }
        else
          // wait a while for the device to open
          Wait(0.1);
      }
      /*free proc buffer*/
      free(proc_buff->frame);
      free(proc_buff);
/*        if(aud_proc_buff)
      {
          g_free(aud_proc_buff->frame);
          g_free(aud_proc_buff);
      }
      if (global.Sound_enable) close_audio_effects (aud_eff);*/
  }
  if(global.debug)
    printf("IO thread finished...OK\n");
  pthread_mutex_lock(&videoIn.mutex);
    videoIn.IOfinished = true;
  pthread_mutex_unlock(&videoIn.mutex);
}

////////////////////////////////////////////////////////

bool UFuncv4l2::initDevice()
{
  int ret;
  const int MRL = 500;
  char reply[MRL] = "";
  //
  if ( ( ret=init_videoIn(&videoIn, &global) ) != 0)
  {
    fprintf(stderr, "Init video returned %i\n",ret);
    switch (ret)
    {
        case VDIN_DEVICE_ERR://can't open device
            snprintf(reply, MRL, "auv4l2::initDevice: error: Unable to open device");
            break;


        case VDIN_DYNCTRL_ERR: //uvc extension controls error - EACCES (needs root user)
            snprintf(reply, MRL, "auv4l2::initDevice: error: UVC Extension controls");
            break;

        case VDIN_UNKNOWN_ERR: //unknown error (treat as invalid format)
        case VDIN_FORMAT_ERR://invalid format
        case VDIN_RESOL_ERR: //invalid resolution
            printf("trying minimum setup ...\n");
            if (videoIn.listFormats->numb_formats > 0) //check for supported formats
            {
                VidFormats *listVidFormats;
                videoIn.listFormats->current_format = 0; //get the first supported format
                global.format = videoIn.listFormats->listVidFormats[0].format;
                if(get_PixMode(global.format, global.mode) < 0)
                    fprintf(stderr, "IMPOSSIBLE: format has no supported mode !?\n");
                listVidFormats = &videoIn.listFormats->listVidFormats[0];
                global.width = listVidFormats->listVidCap[0].width;
                global.height = listVidFormats->listVidCap[0].height;
                if (listVidFormats->listVidCap[0].framerate_num != NULL)
                    global.fps_num = listVidFormats->listVidCap[0].framerate_num[0];
                if (listVidFormats->listVidCap[0].framerate_denom != NULL)
                    global.fps = listVidFormats->listVidCap[0].framerate_denom[0];
            }
            else
            {
                fprintf(stderr, "ERROR: Can't set video stream. No supported format found\nExiting...\n");
            }

            //try again with new format
            ret = init_videoIn (&videoIn, &global);

            if ((ret == VDIN_QUERYBUF_ERR) && (global.cap_meth != videoIn.cap_meth))
            {
                //mmap not supported ? try again with read method
                fprintf(stderr, "mmap failed trying read method...");
                global.cap_meth = videoIn.cap_meth;
                ret = init_videoIn(&videoIn, &global);
                if (ret == VDIN_OK)
                    fprintf(stderr, "OK\n");
                else
                    fprintf(stderr, "FAILED\n");
            }

            if (ret < 0)
            {
                fprintf(stderr, "ERROR: Minimum Setup Failed.\n Exiting...\n");
            }

            break;

        case VDIN_QUERYBUF_ERR:
            if (global.cap_meth != videoIn.cap_meth)
            {
                //mmap not supported ? try again with read method
                fprintf(stderr, "mmap failed trying read method...");
                global.cap_meth = videoIn.cap_meth;
                ret = init_videoIn (&videoIn, &global);
                if (ret == VDIN_OK)
                    fprintf(stderr, "OK\n");
                else
                {
                    fprintf(stderr, "FAILED\n");
                    //return to default method(mmap)
                    global.cap_meth = IO_MMAP;
                    fprintf(stderr, "ERROR: Minimum Setup Failed.\n Exiting...\n");
                }
            }
            break;

        case VDIN_QUERYCAP_ERR:
            snprintf(reply, MRL, "auv4l2::initDevice: Couldn't query device capabilities");
            break;
        case VDIN_READ_ERR:
            snprintf(reply, MRL, "auv4l2::initDevice: error: Read method error. "
                "Please try mmap instead.");
            break;

        case VDIN_REQBUFS_ERR:/*unable to allocate dequeue buffers or mem*/
        case VDIN_ALLOC_ERR:
        case VDIN_FBALLOC_ERR:
        default:
            snprintf(reply, MRL, "auv4l2::initDevice: Unable to allocate Buffers");
            break;
    }
  }
  return ret == VDIN_OK;
}

//////////////////////////////////////

bool UFuncv4l2::process_video(VidBuff *proc_buff,
                struct JPEG_ENCODER_STRUCTURE **jpeg_struct)
{
/*    struct GLOBAL *global = all_data->global;
    struct vdIn *videoIn = all_data->videoIn;
    struct paRecordData *pdata = all_data->pdata;*/
   // int64_t audio_drift = 0;

//     if (global.Sound_enable) {
//         pthread_mutex_lock(pdata->mutex);
//             audio_drift = pdata->ts_drift;
//         pthread_mutex_unlock(pdata->mutex);
//         max_drift = 1000000000 / global.fps;   /* one frame */
//     }

    pthread_mutex_lock(&videoIn.mutex);
        gboolean capVid = videoIn.capVid;
    pthread_mutex_unlock(&videoIn.mutex);

    gboolean finish = false;

    pthread_mutex_lock(&global.mutex);
    gboolean used = global.videoBuff[global.r_ind].used;
    pthread_mutex_unlock(&global.mutex);
    if (used)
    {
        pthread_mutex_lock(&global.mutex);
            // read video Frame
            proc_buff->bytes_used = global.videoBuff[global.r_ind].bytes_used;
            memcpy(proc_buff->frame, global.videoBuff[global.r_ind].frame, proc_buff->bytes_used);
            proc_buff->time_stamp = global.videoBuff[global.r_ind].time_stamp;
            global.videoBuff[global.r_ind].used = false;
            /*signals an empty slot in the video buffer*/
            //g_cond_broadcast(global.IO_cond);

            NEXT_IND(global.r_ind,global.video_buff_size);
            //audio_drift -= global.av_drift;
        pthread_mutex_unlock(&global.mutex);

        /* fprintf(stderr, "audio drift = %lli ms\n", audio_drift / 1000000); */
        /*process video Frame*/
    }
    else
    {
        if (capVid)
        {
            /*video buffer underrun            */
            /*wait for next frame (sleep 10 ms)*/
            sleep_ms(10);
        }
        else
        {
            finish = true; /*all frames processed and no longer capturing so finish*/
        }
    }
    return finish;
}



