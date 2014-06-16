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

#include <gstreamer-0.10/gst/gst.h>
#include <gstreamer-0.10/gst/gstelementfactory.h>
#include "uimgpush.h"

// pointer for indirect callback function (gstreamer)
//UImgPush * plugin;

// constructor
UImgPush::UImgPush(const unsigned int height, const unsigned int width, int channels, int depth)
{ // make initial image buffer
  resize(height, width, channels, depth);
  varPoolParent = NULL;
}

// destructor
UImgPush::~UImgPush()
{
  stop(true);
}

// image update is complete
void UImgPush::imgUpdated()
{ // note that image is updated, for later
  bool resized = false;
  // test of push commands
  setUpdated("");
  // and set the update time - if others are polling
  // for potential updates
  UImage::imgUpdated();
  if (varPoolParent != NULL)
  {
    if (varSize->getInt(0) != (int)height() or
      varSize->getInt(1) != (int)width() or
      varSize->getInt(2) != (int)getChannels() or
      varSize->getInt(3) != (int)getDepth() or
      varSize->getInt(4) != (int)getColorType())
    {
      varSize->setInt(height(), 0);
      varSize->setInt(width(), 1);
      varSize->setInt(getChannels(), 2);
      varSize->setInt(getDepth(), 3);
      varSize->setInt(getColorType(), 4);
      resized=true;
    }
    if (resized)
    {
      stop(true);
      stopShow(true);
    }
    if (varStreaming->getBool(0))
    { // start streaming thread
      if (not threadRunning)
        start();
      while (not threadRunning)
        Wait(0.01);
    }
    else if (threadRunning)
      stop(true);
    if (varShow->getBool(0))
    { // start streaming thread
      if (not threadShowRunning)
        startShow();
      while (not threadShowRunning)
        Wait(0.01);
    }
    else if (threadShowRunning)
      stopShow(true);
    //
    varSerial->setInt(imageNumber);
    varTime->setTime(imgTime);
  }
}

////////////////////////////////////////////////////////////////////

void UImgPush::createLocalVariables(UVarPool * parent)
{
  varPoolParent = parent;
  varSize = parent->addVarA("size", "1 1 3 8 0", "d", "(r) size of image (row x col x channels x depth x colorType)");
  varSerial = parent->addVar("serial", 0.0, "d", "(r) serial number");
  varTime = parent->addVar("time", 0.0, "d", "(r) time of exposure");
  varShow = parent->addVarA("show", "0 0 0", "d", "(r/w) show image on x-display: 0: should show, 1: is showed 2:update count");
  varStreaming = parent->addVarA("streaming", "0 0 0", "d", "(r/w) is streaming active: 0: should stream, 1: is streamning 2:stream count");
  varDesiredUpdateRate = parent->addVar("maxFrameRate", 4.0, "d", "(r/w) stream at this maximum frame rate (images/sec)");
  varTcpPort = parent->addVar("tcpport", 0.0, "d", "(r/w) stream to this port");
  //varUdpClients = parent->addVarA("clients", "localhost:23000", "s", "(rw) Comma separated list of host:port pairs with UDP destinations");
}

///////////////////////////////////////////////////////////////////

bool UImgPush::setTcpPort(int port)
{
  // stop streaming - if running
  stop(true);
  varTcpPort->setInt(port);
  if (varStreaming->getBool())
    // restart streaming
    start();
  return true;
}

/////////////////////////////////////////////////////////////
/// the rest is to maintain an image stream

/// support function to start thread (using pthread)
void * startThread(void * obj);
/**
 * some sort of status monitoring */
// static gboolean
// bus_call (GstBus     *bus,
//          GstMessage *msg,
//          gpointer    data);
/// callback function, every time a buffer has arrived
// static  GstPadProbeReturn
// cb_have_data (GstPad          *pad,
//               GstPadProbeInfo *info,
//               gpointer         user_data);


static void cb_handoff (GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad,
                        gpointer  user_data);



  /**
   * Start stream processing thread */
bool UImgPush::start()
{
  int err = 0;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    err = (pthread_create(&threadHandle, &thAttr,
              &startThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return err;
}

///////////////////////////////////////////////////

/** stop read thread
  \param andWait waits until thread is terminated, when false, then the call returns
  when the stop flag is set (i.e. immidiately). */
void UImgPush::stop(bool andWait)
{
  if (threadRunning and not threadStop)
  { // stop and join thread
    threadStop = true;
    Wait(0.001);
    if (g_main_loop_is_running(gloop))
      g_main_loop_quit(gloop);
    pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////



// void UImgPush::busCall (GstBus  * bus,
//           GstMessage * msg,
//           gpointer    data)
// {
//   GMainLoop *loop = (GMainLoop *)data;
//   //
//   switch (GST_MESSAGE_TYPE (msg))
//   {
//     case GST_MESSAGE_EOS:
//       g_print ("End-of-stream\n");
//       g_main_loop_quit (loop);
//       break;
//     case GST_MESSAGE_ERROR:
//     {
//       gchar *debug = NULL;
//       GError *err = NULL;
//       //
//       gst_message_parse_error (msg, &err, &debug);
//       g_print ("Error: %s\n", err->message);
//       g_error_free (err);
//       //
//       if (debug)
//       {
//         g_print ("Debug details: %s\n", debug);
//         g_free (debug);
//       }
//       //
//       g_main_loop_quit (loop);
//       break;
//     }
//     default:
//       varImgCnt->add(1, 0);
//       break;
//   }
// }


/* *
 * stream thread using fake source - image-pool to stream
 * assembles processing and starts gstreamer main thread,
 * \returns when main loop is stopped, or an error occured. */
#define USE_MJPEG_TCP

void UImgPush::run()
{
  GstStateChangeReturn ret;
  GstElement *pipeline; //, *filesrc, *decoder, *filter, *sink;
  GstElement *fakesrc;
#ifdef USE_X264_UPD
  GstElement *genc;
  GstElement *rtppay;
  GstElement *udpsink;
#endif
#ifdef USE_MJPEG_TCP
   GstElement *mjpeg;
   GstElement *multipartmux;
   GstElement *tcpsink;
#endif
  GstElement *ffmpegcolorspace;
  GstElement *filter;
  GstBus *bus;
  //guint watch_id;
  bool isOK = true;
  //
  if (threadRunning)
    // prevent nested calls;
    return;
  threadRunning = true;
  /* initialization gstreamer */
//   gst_init (&s_argc, &s_argv);

  //
  gloop = g_main_loop_new (NULL, FALSE);
  /* create elements */
  pipeline = gst_pipeline_new ("my_pipeline");

  /* watch for messages on the pipeline's bus (note that this will only
   * work like this when a GLib main loop is running) */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
//  watch_id = gst_bus_add_watch (bus, bus_call, gloop);
  gst_object_unref (bus);
  // video capabilities
  // fake source from image pool
  fakesrc = gst_element_factory_make ("fakesrc", "fakesrc");
  g_object_set(G_OBJECT (fakesrc), "filltype", 1 /*1=nothing*/ ,
               "signal-handoffs", TRUE,
               "sizemax", getDataSize(),
               "sizetype", 2,
               "can-activate-push", TRUE,
               "can-activate-pull", FALSE,
               NULL); // sizetype 2 is FAKE_SRC_SIZETYPE_FIXED as sizemax
  // make filter for fake source
  filter = gst_element_factory_make ("capsfilter", "filter");
  /* setup of fake src */
  if (isBW() or isBayer())
  {
    g_object_set(G_OBJECT (filter), "caps",
        gst_caps_new_simple ("video/x-raw-gray",
            "width", G_TYPE_INT, width(),
            "height", G_TYPE_INT, height(),
            "framerate", GST_TYPE_FRACTION, 30, 1,
            "filltype", G_TYPE_INT, 1,
            "bpp", G_TYPE_INT, getChannels() * getDepth(),
            "depth", G_TYPE_INT, getChannels() * getDepth(),
            "endianness", G_TYPE_INT, 4321,
            "red_mask", G_TYPE_INT, 16711680,
            "green_mask", G_TYPE_INT, 65280,
            "blue_mask", G_TYPE_INT, 0xff,
            NULL),
        NULL);
  }
  else if (isRGB())
  {
    g_object_set(G_OBJECT (filter), "caps",
        gst_caps_new_simple ("video/x-raw-rgb",
            "width", G_TYPE_INT, width(),
            "height", G_TYPE_INT, height(),
            "framerate", GST_TYPE_FRACTION, 30, 1,
            "filltype", G_TYPE_INT, 1,
            "bpp", G_TYPE_INT, getChannels() * getDepth(),
            "depth", G_TYPE_INT, getChannels() * getDepth(),
            "endianness", G_TYPE_INT, 4321,
            "red_mask", G_TYPE_INT, 16711680,
            "green_mask", G_TYPE_INT, 65280,
            "blue_mask", G_TYPE_INT, 0xff,
            NULL),
        NULL);
  }
  else // assume BGR
  {
    g_object_set(G_OBJECT (filter), "caps",
        gst_caps_new_simple ("video/x-raw-rgb",
            "width", G_TYPE_INT, width(),
            "height", G_TYPE_INT, height(),
            "framerate", GST_TYPE_FRACTION, 30, 1,
            "filltype", G_TYPE_INT, 1,
            "bpp", G_TYPE_INT, getChannels() * getDepth(),
            "depth", G_TYPE_INT, getChannels() * getDepth(),
            "endianness", G_TYPE_INT, 4321,
            "blue_mask", G_TYPE_INT, 16711680,
            "green_mask", G_TYPE_INT, 65280,
            "red_mask", G_TYPE_INT, 0xff,
            NULL),
        NULL);
  }
  //
  ffmpegcolorspace = gst_element_factory_make("ffmpegcolorspace",  "ffmpegcolorspace");
  // x264enc pass=qual quantizer=20 tune=zerolatency
#ifdef USE_X264_UPD
  genc = gst_element_factory_make("x264enc", "server_x264enc");
  g_object_set(G_OBJECT(genc)/*, "pass", 1*/ /*GST_X264_ENC_PASS_QUAL*/, "quantizer", 20, "tune", 7 /* GST_X264_ENC_TUNE_ZEROLATENCY*/,  NULL);
  //
  rtppay = gst_element_factory_make("rtph264pay", "server_rtph264pay");
  //
   udpsink = gst_element_factory_make("udpsink", "server_udpsink");
//   g_object_set(G_OBJECT(udpsink), "host", "127.0.0.1",  "port", varUdpPort->getInt(),  NULL);
   g_object_set(G_OBJECT(udpsink), "clients", varUdpClients->getString(), "host", "127.0.0.1",  "port", varUdpPort->getInt(), NULL);
  gst_bin_add_many (GST_BIN (pipeline), fakesrc, filter, ffmpegcolorspace, genc, rtppay, udpsink, NULL);
#endif
  //
#ifdef USE_MJPEG_TCP
  mjpeg = gst_element_factory_make("jpegenc", "mjepeg_encoder");
  multipartmux = gst_element_factory_make("multipartmux", "multipart");
  tcpsink = gst_element_factory_make("tcpserversink", "server_tcpsink");
  g_object_set(G_OBJECT(tcpsink), "port", varTcpPort->getInt(),  NULL);
  gst_bin_add_many (GST_BIN (pipeline), fakesrc, filter, ffmpegcolorspace, mjpeg, multipartmux, tcpsink, NULL);
#endif
  //gst_bin_add_many (GST_BIN (pipeline), fakesrc, filter, ffmpegcolorspace, genc, rtppay, tcpsink, NULL);
  //
  if ( !gst_element_link( fakesrc, filter) )
  {
    isOK = false;
    g_warning("Failed to link elements fakeSrc and filter");
  }
  if ( !gst_element_link( filter, ffmpegcolorspace) )
  {
    isOK = false;
    g_warning("Failed to link elements filter and ffmpegcolorspace");
  }

#ifdef USE_X264_UPD
  if (isOK and !gst_element_link(ffmpegcolorspace,genc) )
  {
    isOK = false;
    g_warning("Failed to link elements ffmpegcolorspace and x264enc");
  }
  if (isOK and !gst_element_link(genc, rtppay)) // framerate2) )
  {
    isOK = false;
    g_warning("Failed to link elements genc and rtpPay"); // framerate2");
  }
  if (isOK and !gst_element_link(rtppay, udpsink) )
  {
    isOK = false;
    g_warning("Failed to link elements rtph264pay and udpsink");
  }
#endif
#ifdef USE_MJPEG_TCP
  if (isOK and !gst_element_link(ffmpegcolorspace, mjpeg) )
  {
    isOK = false;
    g_warning("Failed to link elements ffmpegcolorspace and mjpeg");
  }
  if (isOK and !gst_element_link(mjpeg, multipartmux)) // framerate2) )
  {
    isOK = false;
    g_warning("Failed to link elements mjpeg and multipartmux"); // framerate2");
  }
  if (isOK and !gst_element_link(multipartmux, tcpsink) )
  {
    isOK = false;
    g_warning("Failed to link elements multipartmux and tcpsink");
  }
#endif
  /* setup fake source */
  g_signal_connect(fakesrc, "handoff", G_CALLBACK (cb_handoff), this);
  /* run */
  if (isOK)
    ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  else
    ret = GST_STATE_CHANGE_FAILURE;
  //
  if (ret == GST_STATE_CHANGE_FAILURE)
  {
    GstMessage *msg;
    g_print ("Failed to start up pipeline!\n");
    /* check if there is an error message with details on the bus */
    msg = gst_bus_poll (bus, GST_MESSAGE_ERROR, 0);
    if (msg) {
      GError *err = NULL;

      gst_message_parse_error (msg, &err, NULL);
      g_print ("ERROR: %s\n", err->message);
      g_error_free (err);
      gst_message_unref (msg);
    }
  }
  else if (not threadStop)
  { // everithing OK
    // run streamer
    varStreaming->setBool(true, 1);
    g_main_loop_run (gloop);
    varStreaming->setBool(false, 1);
  }
  // stream ended
  varStreaming->setBool(false, 0);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
  //g_source_remove (watch_id);
  g_main_loop_unref (gloop);
  gloop = NULL;
  threadRunning = false;
}


/// ///////////////////////////////////////////////////////////////
/* *
 * Fill fake source buffer with new data from pool image */
void UImgPush::fillBuffer(GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad)
{
  UTime t;
  t.now();
  while ( not threadStop and (t.getTimePassed() < 1.0) and
         ((lastUpdateTime == imgUpdateTime) or
          (varStreaming->getBool(0) == false) or
          (varDesiredUpdateRate->getDouble(0) * (imgUpdateTime - lastUpdateTime) < 1.0)))
  { // not time for update or no update or not streaming right now
    Wait(0.01);
  }
  if (tryLock())
  { // copy image to streamer buffer
    lastUpdateTime = imgUpdateTime;
    int n = mini(GST_BUFFER_SIZE (buffer) , getDataSize());
    if (n > 0)
      memcpy(GST_BUFFER_DATA (buffer), getData(), n);
    varStreaming->add(1, 2);
    unlock();
  }
}
  //

///////////////////////////////////////////////////////////////////////////////


void * startThread(void * obj)
{ // call the hadling function in provided object
  UImgPush * plugin = (UImgPush *) obj;
  if (plugin != NULL)
  {
    plugin = (UImgPush *)obj;
    plugin->run();
    pthread_exit((void*)NULL);
  }
  else
    fprintf(stderr, "UImgPush::startThread failed to get image object, so failed to start streaming thread\n");
  return NULL;
}

/////////////////////////////////////////////////////////////////

// static gboolean
// bus_call (GstBus     *bus,
//           GstMessage *msg,
//           gpointer    data)
// {
//   plugin->busCall(bus, msg, data);
//   return TRUE;
// }

/////////////////////////////////////////////////////////////////

static void cb_handoff (GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad,
                        gpointer  user_data)
{
//  static gboolean white = FALSE;
  UImgPush * cls = (UImgPush *) user_data;
  /* this makes the image black/white */
//   memset (GST_BUFFER_DATA (buffer), white ? 0xff : 0x0,
//   GST_BUFFER_SIZE (buffer));
//   white = !white;
  if (cls != NULL)
    cls->fillBuffer(fakesrc, buffer, pad);
  else
    fprintf(stderr, "UImgPush::cb_handoff failed to get pointer to image object\n");
}

/// /////////////////////////////////////////////////////////////////
/// /////////////////////////////////////////////////////////////////
/// /////////////////////////////////////////////////////////////////

/// show image useing gstreamer
/// support function to start thread (using pthread)

void * startShowThread(void * obj);
/**
 * some sort of status monitoring */
// static gboolean
// bus_call (GstBus     *bus,
//          GstMessage *msg,
//          gpointer    data);
/// callback function, every time a buffer has arrived
// static  GstPadProbeReturn
// cb_have_data (GstPad          *pad,
//               GstPadProbeInfo *info,
//               gpointer         user_data);


static void cb_handoff_show (GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad,
                        gpointer  user_data);



  /**
   * Start stream processing thread */
bool UImgPush::startShow()
{
  int err = 0;
  pthread_attr_t  thAttr;
  //
  if (not threadShowRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadShowStop = false;
    // create socket server thread
    err = (pthread_create(&threadShowHandle, &thAttr,
              &startShowThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return err;
}

///////////////////////////////////////////////////

/** stop read thread
  \param andWait waits until thread is terminated, when false, then the call returns
  when the stop flag is set (i.e. immidiately). */
void UImgPush::stopShow(bool andWait)
{
  if (threadShowRunning and not threadShowStop)
  { // stop and join thread
    threadShowStop = true;
    Wait(0.001);
    if (g_main_loop_is_running(gloop_show))
      g_main_loop_quit(gloop_show);
    pthread_join(threadShowHandle, NULL);
  }
}

/////////////////////////////////////////////////////

/* *
 * stream thread using fake source - image-pool to stream
 * assembles processing and starts gstreamer main thread,
 * \returns when main loop is stopped, or an error occured. */
void UImgPush::runShow()
{
  GstStateChangeReturn ret;
  GstElement *pipeline; //, *filesrc, *decoder, *filter, *sink;
  GstElement *fakesrc;
  GstElement *videosink;
  GstElement *ffmpegcolorspace;
  GstElement *filter;
  GstBus *bus;
  //guint watch_id;
  bool isOK = true;
  //
  if (threadShowRunning)
    // prevent nested calls;
    return;
  threadShowRunning = true;
  /* initialization gstreamer */
//   gst_init (&s_argc, &s_argv);

  //
  gloop_show = g_main_loop_new (NULL, FALSE);
  /* create elements */
  pipeline = gst_pipeline_new ("my_pipeline");

  /* watch for messages on the pipeline's bus (note that this will only
   * work like this when a GLib main loop is running) */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
//  watch_id = gst_bus_add_watch (bus, bus_call, gloop);
  // video capabilities
  // fake source from image pool
  fakesrc = gst_element_factory_make ("fakesrc", "fakesrc");
  unsigned int dataSize;
  if (isBW() or isRGB())
    dataSize = getDataSize();
  else
    // convert to RGB
    dataSize = getWidth() * getHeight() * 3;
  g_object_set(G_OBJECT (fakesrc), "filltype", 1 /*1=nothing*/ ,
               "signal-handoffs", TRUE,
               "sizemax", dataSize,
               "sizetype", 2,
               "can-activate-push", TRUE,
               "can-activate-pull", FALSE,
               NULL); // sizetype 2 is FAKE_SRC_SIZETYPE_FIXED as sizemax
  // make filter for fake source
  filter = gst_element_factory_make ("capsfilter", "filter");
  /* setup of fake src */
  if (isBW())
  {
    g_object_set(G_OBJECT (filter), "caps",
        gst_caps_new_simple ("video/x-raw-gray",
            "width", G_TYPE_INT, width(),
            "height", G_TYPE_INT, height(),
            "framerate", GST_TYPE_FRACTION, 30, 1,
            "filltype", G_TYPE_INT, 1,
            "bpp", G_TYPE_INT, getChannels() * getDepth(),
            "depth", G_TYPE_INT, getChannels() * getDepth(),
            "endianness", G_TYPE_INT, 4321,
            NULL),
        NULL);
  }
  else if (isRGB())
  {
    g_object_set(G_OBJECT (filter), "caps",
        gst_caps_new_simple ("video/x-raw-rgb",
            "width", G_TYPE_INT, width(),
            "height", G_TYPE_INT, height(),
            "framerate", GST_TYPE_FRACTION, 30, 1,
            "filltype", G_TYPE_INT, 1,
            "bpp", G_TYPE_INT, getChannels() * getDepth(),
            "depth", G_TYPE_INT, getChannels() * getDepth(),
            "endianness", G_TYPE_INT, 4321,
            "red_mask", G_TYPE_INT, 16711680,
            "green_mask", G_TYPE_INT, 65280,
            "blue_mask", G_TYPE_INT, 0xff,
            NULL),
        NULL);
  }
  else // convert to BGR
  { // set size and other items as if in RGB format
    // will be converted when buffer is updated
    g_object_set(G_OBJECT (filter), "caps",
        gst_caps_new_simple ("video/x-raw-rgb",
            "width", G_TYPE_INT, width(),
            "height", G_TYPE_INT, height(),
            "framerate", GST_TYPE_FRACTION, 30, 1,
            "filltype", G_TYPE_INT, 1,
            "bpp", G_TYPE_INT, 24, // getChannels() * getDepth(),
            "depth", G_TYPE_INT, 24, // getChannels() * getDepth(),
            "endianness", G_TYPE_INT, 4321,
            "blue_mask", G_TYPE_INT, 16711680,
            "green_mask", G_TYPE_INT, 65280,
            "red_mask", G_TYPE_INT, 0xff,
            NULL),
        NULL);
  }
  //
  ffmpegcolorspace = gst_element_factory_make("ffmpegcolorspace",  "ffmpegcolorspace");
  videosink = gst_element_factory_make("xvimagesink",  "videosink");
  //
  gst_bin_add_many (GST_BIN (pipeline), fakesrc, filter, ffmpegcolorspace, videosink, NULL);
  //
  if ( !gst_element_link( fakesrc, filter) )
  {
    isOK = false;
    g_warning("UImgPush::runShow Failed to link elements fakeSrc and filter");
  }
  if ( !gst_element_link( filter, ffmpegcolorspace) )
  {
    isOK = false;
    g_warning("UImgPush::runShow Failed to link elements filter and ffmpegcolorspace");
  }
  if (isOK and !gst_element_link(ffmpegcolorspace,videosink) )
  {
    isOK = false;
    g_warning("UImgPush::runShow Failed to link elements ffmpegcolorspace and x264enc");
  }
  /* setup fake source */
  g_signal_connect(fakesrc, "handoff", G_CALLBACK (cb_handoff_show), this);
  /* run */
  if (isOK)
    ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  else
    ret = GST_STATE_CHANGE_FAILURE;
  //
  if (ret == GST_STATE_CHANGE_FAILURE)
  {
    GstMessage *msg;
    g_print ("UImgPush::runShow: Failed to start up pipeline to show image!\n");
    /* check if there is an error message with details on the bus */
    msg = gst_bus_poll (bus, GST_MESSAGE_ERROR, 0);
    if (msg) {
      GError *err = NULL;

      gst_message_parse_error (msg, &err, NULL);
      g_print ("UImgPush::runShow: ERROR: %s\n", err->message);
      g_error_free (err);
      gst_message_unref (msg);
    }
  }
  else if (not threadShowStop)
  { // everithing OK
    // run streamer
    varShow->setBool(true, 1);
    g_main_loop_run (gloop_show);
  }
  // show ended
  varShow->setBool(false, 1);
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
  //g_source_remove (watch_id);
  g_main_loop_unref (gloop_show);
  gloop_show = NULL;
  threadShowRunning = false;
}


/// ///////////////////////////////////////////////////////////////
/* *
 * Fill fake source buffer with new data from pool image */
void UImgPush::fillShowBuffer(GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad)
{
  UTime t;
  t.now();
  while ( not threadShowStop and (t.getTimePassed() < 1.0) and
         ((lastShowTime == imgUpdateTime) or
          (varShow->getBool(0) == false) or
          (varDesiredUpdateRate->getDouble(0) * (imgUpdateTime - lastUpdateTime) < 4.0)))
  { // not time for update or no update or not streaming right now
    Wait(0.01);
  }
  if (tryLock())
  { // copy image to streamer buffer
    lastShowTime = imgUpdateTime;
    UImage * tmp = this;
    if (tmp->isBayer() or tmp->isYUV() or tmp->isYUV420() or tmp->isYUV422() or (tmp->getDepth() > 8))
    { // not a useable color format
      UImage ** imgBuf;
      imgBuf = getConvertBuffer();
      if (*imgBuf == NULL)
        *imgBuf = new UImage();
      tmp = *imgBuf;
      toBGR(tmp);
    }
    int g = GST_BUFFER_SIZE (buffer);
    int n = tmp->getDataSize();
    n = mini(g, n);
    if (n > 0)
      memcpy(GST_BUFFER_DATA (buffer), tmp->getData(), n);
    varShow->add(1, 2);
    unlock();
  }
}
  //

///////////////////////////////////////////////////////////////////////////////


void * startShowThread(void * obj)
{ // call the hadling function in provided object
  UImgPush * plugin = (UImgPush *) obj;
  if (plugin != NULL)
  {
    plugin = (UImgPush *)obj;
    plugin->runShow();
    pthread_exit((void*)NULL);
  }
  else
    fprintf(stderr, "UImgPush::startShowThread failed to get image object, so failed to start streaming thread\n");
  return NULL;
}

/////////////////////////////////////////////////////////////////

static void cb_handoff_show (GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad,
                        gpointer  user_data)
{
//  static gboolean white = FALSE;
  UImgPush * cls = (UImgPush *) user_data;
  /* this makes the image black/white */
//   memset (GST_BUFFER_DATA (buffer), white ? 0xff : 0x0,
//   GST_BUFFER_SIZE (buffer));
//   white = !white;
  if (cls != NULL)
    cls->fillShowBuffer(fakesrc, buffer, pad);
  else
    fprintf(stderr, "UImgPush::cb_handoff_show failed to get pointer to image object\n");
}


