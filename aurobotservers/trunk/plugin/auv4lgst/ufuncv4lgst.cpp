/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#ifdef OPENCV2
#include <legacy/compat.hpp>
#endif

//#include <gstreamer-1.0/gst/gst.h>
#include <gstreamer-0.10/gst/gst.h>
//#include <gst/gst.h>
#include <gstreamer-0.10/gst/gstelementfactory.h>

#include <urob4/usmltag.h>
#include <cstdlib>
#include <ucam4/ufunctioncambase.h>
#include <urob4/uimgpush.h>


/// support function to start thread (using pthread)
void * startGstThread(void * obj);
/**
 * some sort of status monitoring */
static gboolean
bus_call (GstBus     *bus,
          GstMessage *msg,
          gpointer    data);
/// callback function, every time a buffer has arrived
// static  GstPadProbeReturn
// cb_have_data (GstPad          *pad,
//               GstPadProbeInfo *info,
//               gpointer         user_data);


static void cb_handoff (GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad,
                        gpointer  user_data);

class UFuncV4lGst;
UFuncV4lGst * uFuncV4lGst = NULL;


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

/**
Experimental plugin to use gstreamer to import/export image series.
- is of no use yet (march 2013)
@author Christian Andersen
*/
class UFuncV4lGst : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncV4lGst()
  {
    setCommand("v4lgst", "v4lgst", "video streamer (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    // createBaseVar();
    // initialize local variables
    threadRunning = false;
    threadStop = false;
    threadHandle = -1;
    gloop = NULL;
  };
  /**
  Destructor - to delete the resource (etc) when finished */
  virtual ~UFuncV4lGst()
  { // possibly remove allocated variables here - if needed
  }
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources()
  {
    createBaseVar();
  }
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  { // handle command(s) send to this plug-in
    const int MRL = 2000;
    char reply[MRL];
    bool ask4help;
    const int MVL = 50;
    char val[MVL];
    USmlTag tag;
    bool result;
    bool debug = true; // default is debug on
    // check for parameters - one parameter is tested for - 'help'
    // the help value is ignored, e.g. if help="bark", then
    // the value "bark" will be in the 'helpValue' string.
    ask4help = msg->tag.getAttValue("help", val, MVL);
    if (not ask4help)
    { // get all other parameters
      msg->tag.getAttValueBool("debug", &debug, true);
    }
    // ask4help = false, if no 'help' option were available.
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart("v4lgst");
      sendText("--- available V4LGST options\n");
      sendText("Gstreamer camera device device\n");
      sendText("open[=false]      (re)open [or close] video stream\n");
      sendText("help              This message\n");
      sendText("see also: 'VAR v4lgst' for parameters\n");
      sendHelpDone();
      sendInfo("done");
      result = true;
    }
    else
    { // resource is available, so make a reply
      bool openRequest = false;
      msg->tag.getAttBool("open", &openRequest, true);
      varStream->setInt(openRequest, 0);
      if (openRequest)
      { // stop - if already running
        stop(true);
        start();
        snprintf(reply, MRL, "trying to start stream from %s to image %d", varDevice->getString(0), varImgNum->getInt(0));
        sendInfo(reply);
      }
      else
      { // stop thread
        if (threadRunning)
        { // stop streamer
          if (g_main_loop_is_running(gloop))
            stop(true);
          sendInfo("stopped streaming");
        }
        else
          sendWarning("stream is not running");
      }
      snprintf(reply, MRL, "Not finished yet");
      sendWarning(reply);
    }
    // return true if the function is handled with a positive result
    return result;
  }

protected:

  /**
  Make the variables that will be available to other plugins */
  void createBaseVar()
  {
    varDevice = addVarA("v4ldevice", "/dev/video0", "s", "(r/w) video source device");
    varStream = addVarA("stream", "0 0 0", "d", "(rw) streaming 0:start=1, 1:is streaming, 2:frame counter");
    varSize = addVarA("size", "480 640 3 8", "d", "(rw) Image size (height x width, channels, depth)");
    varFramerate = addVarA("framerate", "30 1 2 1", "d", "(rw) camera framerate (num denom), used framerate (num, denom)");
    varFormat = addVarA("format", "rgb", "s", "(rw) video format - limited support");
    varImgNum = addVar("img", 0.0, "d", "(rw) destination image number");
    varCamNum = addVar("cam", 0.0, "d", "(rw) Camera number index");
    varBusCnt = addVar("busErrCnt", 0.0, "d", "(r) bus call count");
    varBusErr = addVarA("busErr", "(none)", "s", "(r) last error message from stream bus");
  }


///////////////////////////////////////////////////
private:

  /**
   * Start stream processing thread */
bool start()
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
              &startGstThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return err;
}

///////////////////////////////////////////////////

/** stop read thread
  \param andWait waits until thread is terminated, when false, then the call returns
  when the stop flag is set (i.e. immidiately). */
void stop(bool andWait)
{
  if (threadRunning and not threadStop)
  { // stop and join thread
    threadStop = true;
    if (g_main_loop_is_running(gloop))
      g_main_loop_quit(gloop);
    if (andWait)
      pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////



public:

void busCall (GstBus  * bus,
          GstMessage * msg,
          gpointer    data)
{
  GMainLoop *loop = (GMainLoop *)data;
  //
  switch (GST_MESSAGE_TYPE (msg))
  {
    case GST_MESSAGE_EOS:
      g_print ("End-of-stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR:
    {
      gchar *debug = NULL;
      GError *err = NULL;
      //
      gst_message_parse_error (msg, &err, &debug);
      varBusErr->setString(err->message, 0, true);
      g_print ("Error: %s\n", err->message);
      g_error_free (err);
      //
      if (debug)
      {
        g_print ("Debug details: %s\n", debug);
        g_free (debug);
      }  
      //
      g_main_loop_quit (loop);
      break;
    }
    default:
      varBusCnt->add(1, 0);
      break;
  }
}

/**
 * stream thread
 * assembles processing and starts gstreamer main thread,
 * This part works, use client script to test.
 * \returns when main loop is stopped, or an error occured. */
void run()
{
  GstStateChangeReturn ret;
  GstElement *pipeline;
  GstElement *fakesink;
//  GstElement *ffmpegcolorspace;
  GstElement *framerate;
  GstElement *filter;
  GstElement *v4l2src;
  GstBus *bus;
  guint watch_id;
  GstCaps *caps;
  GstCaps *filtercaps;
  bool isOK = true;
  const char * videoFormat;
  //
  if (threadRunning)
    // prevent nested calls;
    return;
  threadRunning = true;
  /* initialization gstreamer */
  gst_init (&s_argc, &s_argv);
  //
  gloop = g_main_loop_new (NULL, FALSE);
  /* create elements */
  pipeline = gst_pipeline_new ("my_pipeline");
  /* watch for messages on the pipeline's bus (note that this will only
   * work like this when a GLib main loop is running) */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  watch_id = gst_bus_add_watch (bus, bus_call, gloop);
  gst_object_unref (bus);
  // video capabilities
  /// framerate from camera
  if (strcasecmp(varFormat->getString(), "rgb") == 0)
    videoFormat = "video/x-raw-rgb";
  else if (strcasecmp(varFormat->getString(), "bayer") == 0)
    videoFormat = "video/x-raw-bayer";
  else if (strcasecmp(varFormat->getString(), "yuv") == 0)
    videoFormat = "video/x-raw-yuv";
  else if (strcasecmp(varFormat->getString(), "gray") == 0)
    videoFormat = "video/x-raw-gray";
  else 
    videoFormat = "video/x-raw-rgb";
  //
  caps = gst_caps_new_simple (videoFormat,
            //"format", G_TYPE_STRING, "RGB",
            "width", G_TYPE_INT, varSize->getInt(1),
            "height", G_TYPE_INT, varSize->getInt(0),
            "framerate", GST_TYPE_FRACTION, varFramerate->getInt(0), varFramerate->getInt(1),
            NULL);
  /// framerate in communication
  // raw source from live camera
  v4l2src  = gst_element_factory_make ("v4l2src", "my_camera");
  g_object_set(G_OBJECT(v4l2src), "device", "/dev/video0", NULL);

  // to get rather slow frame rates
  framerate = gst_element_factory_make("videorate",  "videorate");
//  framerate2 = gst_element_factory_make("videorate",  "videorate2");
  filter = gst_element_factory_make ("capsfilter", "filter");
  /// framerate in encoder
  filtercaps = gst_caps_new_simple ("video/x-raw-rgb",
            "framerate", GST_TYPE_FRACTION, varFramerate->getInt(2), varFramerate->getInt(3),
            NULL);
  //
  fakesink = gst_element_factory_make("fakesink", "fakesink");
  g_object_set(G_OBJECT(fakesink), "signal-handoffs", true,  NULL);

  //   ffmpegcolorspace = gst_element_factory_make("ffmpegcolorspace",  "ffmpegcolorspace");
//   // x264enc pass=qual quantizer=20 tune=zerolatency
//   genc = gst_element_factory_make("x264enc", "server_x264enc");
//   g_object_set(G_OBJECT(genc)/*, "pass", 1*/ /*GST_X264_ENC_PASS_QUAL*/, "quantizer", 20, "tune", 7 /* GST_X264_ENC_TUNE_ZEROLATENCY*/,  NULL);
//   //
//   // rtph264pay ! udpsink host=127.0.0.1 port=1234
//   rtppay = gst_element_factory_make("rtph264pay", "server_rtph264pay");
//   //
//   udpsink = gst_element_factory_make("udpsink", "server_udpsink");
//   g_object_set(G_OBJECT(udpsink), "host", "127.0.0.1",  "port", varUdpPort->getInt(),  NULL);
  //
  gst_bin_add_many (GST_BIN (pipeline), v4l2src, framerate, filter, fakesink, NULL); // ffmpegcolorspace, genc, rtppay, udpsink, NULL);
  //
  if ( !gst_element_link_filtered( v4l2src, framerate, caps) )
  {
    isOK = false;
    g_warning("Failed to link elements v4l2src and framerate");
  }
  if ( !gst_element_link_filtered( framerate, filter, filtercaps) )
  {
    isOK = false;
    g_warning("Failed to link elements framerate and filter");
  }
  if ( !gst_element_link( filter, fakesink) )
  {
    isOK = false;
    g_warning("Failed to link elements filter and fakesink");
  }
  /* setup fake sink use */
  g_signal_connect(fakesink, "handoff", G_CALLBACK (cb_handoff), NULL);
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
    // set destination image to right size
    UImage * img = imgPool->getImage(varImgNum->getInt(), true);
    if (img->tryLock())
    {
      if ((int)img->height() != varSize->getInt(0) or (int)img->width() != varSize->getInt(1) or
          (int)img->getChannels() != varSize->getInt(2) or (int)img->getDepth() != varSize->getInt(3))
      {
        img->resize(varSize->getInt(0), varSize->getInt(1), varSize->getInt(2), varSize->getInt(3));
        if ((int)img->height() != varSize->getInt(0) or (int)img->width() != varSize->getInt(1) or
            (int)img->getChannels() != varSize->getInt(2) or (int)img->getDepth() != varSize->getInt(3))
          printf("UFuncV4lGst::run: failed to set image size\n");
      }
      img->unlock();
    }
    // run streamer
    varStream->setBool(true, 1);
    g_main_loop_run (gloop);
  }
  // stream ended
  varStream->setBool(false, 1);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
  g_source_remove (watch_id);
  g_main_loop_unref (gloop);
  gloop = NULL;
  threadRunning = false;
}

/**
 * stream thread using fake source - image-pool to stream
 * assembles processing and starts gstreamer main thread,
 * \returns when main loop is stopped, or an error occured. */
// void runFakeSrc()
// {
//   GstStateChangeReturn ret;
//   GstElement *pipeline; //, *filesrc, *decoder, *filter, *sink;
//   GstElement *fakesrc;
//   GstElement *genc;
//   GstElement *rtppay;
//   GstElement *udpsink;
//   GstElement *ffmpegcolorspace;
//   GstElement *filter;
//   GstBus *bus;
//   guint watch_id;
//   bool isOK = true;
//   //
//   if (threadRunning)
//     // prevent nested calls;
//     return;
//   threadRunning = true;
//   /* initialization gstreamer */
//   gst_init (&s_argc, &s_argv);
// 
//   //
//   gloop = g_main_loop_new (NULL, FALSE);
//   /* create elements */
//   pipeline = gst_pipeline_new ("my_pipeline");
// 
//   /* watch for messages on the pipeline's bus (note that this will only
//    * work like this when a GLib main loop is running) */
//   bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
//   watch_id = gst_bus_add_watch (bus, bus_call, gloop);
//   gst_object_unref (bus);
//   // video capabilities
//   // fake source from image pool
//   fakesrc = gst_element_factory_make ("fakesrc", "fakesrc");
//   g_object_set(G_OBJECT (fakesrc), "filltype", 1 /*1=nothing*/ ,
//                "signal-handoffs", TRUE,
//                "sizemax", 640 * 480  *3,
//                "sizetype", 2,
//                "can-activate-push", TRUE,
//                "can-activate-pull", FALSE,
//                NULL); // sizetype 2 is FAKE_SRC_SIZETYPE_FIXED as sizemax
//   // make filter for fake source
//   filter = gst_element_factory_make ("capsfilter", "filter");
//   /* setup of fake src */
//   g_object_set(G_OBJECT (filter), "caps",
//        gst_caps_new_simple ("video/x-raw-rgb",
//           "width", G_TYPE_INT, 640,
//           "height", G_TYPE_INT, 480,
//           "framerate", GST_TYPE_FRACTION, 30, 1,
//           "filltype", G_TYPE_INT, 1,
//           "bpp", G_TYPE_INT, 24,
//           "depth", G_TYPE_INT, 24,
//           "endianness", G_TYPE_INT, 4321,
//           "red_mask", G_TYPE_INT, 16711680,
//           "green_mask", G_TYPE_INT, 65280,
//           "blue_mask", G_TYPE_INT, 0xff,
//           NULL),
//        NULL);
//   //
//   ffmpegcolorspace = gst_element_factory_make("ffmpegcolorspace",  "ffmpegcolorspace");
//   // x264enc pass=qual quantizer=20 tune=zerolatency
//   genc = gst_element_factory_make("x264enc", "server_x264enc");
//   g_object_set(G_OBJECT(genc)/*, "pass", 1*/ /*GST_X264_ENC_PASS_QUAL*/, "quantizer", 20, "tune", 7 /* GST_X264_ENC_TUNE_ZEROLATENCY*/,  NULL);
//   //
//   rtppay = gst_element_factory_make("rtph264pay", "server_rtph264pay");
//   //
//   udpsink = gst_element_factory_make("udpsink", "server_udpsink");
//   g_object_set(G_OBJECT(udpsink), "host", "127.0.0.1",  "port", varUdpPort->getInt(),  NULL);
//   //
//   gst_bin_add_many (GST_BIN (pipeline), fakesrc, filter, ffmpegcolorspace, genc, rtppay, udpsink, NULL);
//   //
//   if ( !gst_element_link( fakesrc, filter) )
//   {
//     isOK = false;
//     g_warning("Failed to link elements fakeSrc and filter");
//   }
//   if ( !gst_element_link( filter, ffmpegcolorspace) )
//   {
//     isOK = false;
//     g_warning("Failed to link elements filter and ffmpegcolorspace");
//   }
//   if (isOK and !gst_element_link(ffmpegcolorspace,genc) )
//   {
//     isOK = false;
//     g_warning("Failed to link elements ffmpegcolorspace and x264enc");
//   }
//   if (isOK and !gst_element_link(genc, rtppay)) // framerate2) )
//   {
//     isOK = false;
//     g_warning("Failed to link elements genc and rtpPay"); // framerate2");
//   }
//   if (isOK and !gst_element_link(rtppay, udpsink) )
//   {
//     isOK = false;
//     g_warning("Failed to link elements rtph264pay and udpsink");
//   }
//   /* setup fake source */
//   g_signal_connect(fakesrc, "handoff", G_CALLBACK (cb_handoff), NULL);
//   /* run */
//   if (isOK)
//     ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
//   else
//     ret = GST_STATE_CHANGE_FAILURE;
//   //
//   if (ret == GST_STATE_CHANGE_FAILURE)
//   {
//     GstMessage *msg;
//     g_print ("Failed to start up pipeline!\n");
//     /* check if there is an error message with details on the bus */
//     msg = gst_bus_poll (bus, GST_MESSAGE_ERROR, 0);
//     if (msg) {
//       GError *err = NULL;
// 
//       gst_message_parse_error (msg, &err, NULL);
//       g_print ("ERROR: %s\n", err->message);
//       g_error_free (err);
//       gst_message_unref (msg);
//     }
//   }
//   else if (not threadStop)
//   { // everithing OK
//     // run streamer
//     varStreaming->setBool(true, 0);
//     g_main_loop_run (gloop);
//   }
//   // stream ended
//   varStreaming->setBool(false, 0);
//   gst_element_set_state (pipeline, GST_STATE_NULL);
//   gst_object_unref (pipeline);
//   g_source_remove (watch_id);
//   g_main_loop_unref (gloop);
//   gloop = NULL;
//   threadRunning = false;
// }


/// ///////////////////////////////////////////////////////////////
/**
 * Fill fake source buffer with new data from pool image */
// void fillBuffer(GstElement *fakesrc,
//                         GstBuffer *buffer,
//                         GstPad    *pad)
//{
//   UImagePool * ipool = (UImagePool *)getStaticResource("imgpool", false, false);
//   UImage * img;
//   double dt;
//   if (imgPool != NULL)
//   {
//     img = ipool->getImage(22, true, 480, 640, 3, 8);
//     while (imageNumber == img->imageNumber and not threadStop)
//     {
//       Wait(0.01);
//     }
//     if (img->tryLock())
//     {
//       imageNumber = img->imageNumber;
// //      if (img->imageNumber % 30 == 0)
//       {
//         if (GST_BUFFER_SIZE (buffer) >= img->getDataSize())
//         {
//           memcpy(GST_BUFFER_DATA (buffer), img->getData(), img->getDataSize());
//         }
//         dt = tPush.getTimePassed();
//         tPush.now();
//         printf("-- dt=%.4fs\n", dt);
//       }
//       img->unlock();
//     }
//   }
//}
/**
 * Fill fake source buffer with new data from pool image */
void useBuffer(GstElement *fakesink,
                        GstBuffer *buffer,
                        GstPad    *pad)
{
  UImage * img;
  if (imgPool != NULL)
  {
    img = imgPool->getImage(varImgNum->getInt(), true);
    if (img->tryLock())
    {
      if ((int)img->height() != varSize->getInt(0) or (int)img->width() != varSize->getInt(1) or
          (int)img->getChannels() != varSize->getInt(2) or (int)img->getDepth() != varSize->getInt(3))
      { // image size has changed - stop
        printf("UFuncV4lGst::useBuffer: stopping camera, as image size has changed\n");
        stop(false);
      }
      else
      { // all OK, so copy image
        int n = mini(GST_BUFFER_SIZE (buffer), img->getDataSize());
        if (n > 0)
        {
          memcpy(img->getData(), GST_BUFFER_DATA (buffer), n);
          img->camDevice = varCamNum->getInt();
          img->imageNumber++;
          img->imgTime.now();
          img->used = 0;
          img->valid = true;
          img->updated();
          varStream->add(1, 2);
        }
      }
      img->unlock();
    }
  }
}
  //
private:
  /// is thread running - set by thread
  bool threadRunning;
  /// is thread to stop - set by main function
  bool threadStop;
  /**
  Thread handle for processing thread. */
  pthread_t threadHandle;
  /// size of source image in pixels
  int imgSizeH;
  /// size of source image in pixels
  int imgSizeW;
  /// device string variable
  UVariable *  varDevice;
  /// is port open
  UVariable *  varStream;
  /// Image size (height x width)
  UVariable * varSize;
  /// camera framerate (num denom), used framerate (num, denom)
  UVariable * varFramerate;
  /// video format string - limited support
  UVariable * varFormat;
  /// destination image number
  UVariable * varImgNum;
  /// camera index number
  UVariable * varCamNum;
  /// streamer bus call count
  UVariable * varBusCnt;
  /// streamer bus error message
  UVariable * varBusErr;

  /// timestamp
  UTime tPush;
  /// last image number
  uint32_t imageNumber;
/// streamer variables:
  GMainLoop * gloop;
  
};

///////////////////////////////////////////////////////////////////////////////

UFuncV4lGst * plugin;


void * startGstThread(void * obj)
{ // call the hadling function in provided object
  //UFuncV4lGst * ce = (UFuncV4lGst *)obj;
  plugin = (UFuncV4lGst *)obj;
  plugin->run();
  pthread_exit((void*)NULL);
  return NULL;
}

/////////////////////////////////////////////////////////////////

static gboolean
bus_call (GstBus     *bus,
          GstMessage *msg,
          gpointer    data)
{
  plugin->busCall(bus, msg, data);
  return TRUE;
}

/////////////////////////////////////////////////////////////////

static void cb_handoff (GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad,
                        gpointer  user_data)
{
//  static gboolean white = FALSE;
  UFuncV4lGst * cls = uFuncV4lGst; // (UFuncV4lGst *) user_data;
  /* this makes the image black/white */
//   memset (GST_BUFFER_DATA (buffer), white ? 0xff : 0x0,
//   GST_BUFFER_SIZE (buffer));
//   white = !white;
  if (cls != NULL)
    cls->useBuffer(fakesrc, buffer, pad);
}


/////////////////////////////////////////////////////////////////

// static GstPadProbeReturn
// cb_have_data (GstPad          *pad,
//               GstPadProbeInfo *info,
//               gpointer         user_data)
// {
//   gint x, y;
//   GstMapInfo map;
//   guint16 *ptr, t;
//   GstBuffer *buffer;
// 
//   buffer = GST_PAD_PROBE_INFO_BUFFER (info);
// 
//   buffer = gst_buffer_make_writable (buffer);
// 
//   gst_buffer_map (buffer, &map, GST_MAP_WRITE);
// 
//   ptr = (guint16 *) map.data;
//   /* invert data */
//   for (y = 0; y < 288; y++) {
//     for (x = 0; x < 384 / 2; x++) {
//       t = ptr[384 - 1 - x];
//       ptr[384 - 1 - x] = ptr[x];
//       ptr[x] = t;
//     }
//     ptr += 384;
//   }
//   gst_buffer_unmap (buffer, &map);
// 
//   GST_PAD_PROBE_INFO_DATA (info) = buffer;
// 
//   return GST_PAD_PROBE_OK;
// }

///////////////////////////////////////////////////////////////////////////////

#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncV4lGst' with your classname, as used in the headerfile */
  uFuncV4lGst = new UFuncV4lGst();
  return uFuncV4lGst;
}
#endif

