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
//#include <gstreamer-0.10/gst/gstfakesink.h>

#include <urob4/usmltag.h>
#include <cstdlib>
#include <urob4/ufuncplugbase.h>
#include <urob4/uimagepool.h>


/// support function to start thread (using pthread)
void * startThread(void * obj);
/**
 * some sort of status monitoring */
// static gboolean
//bus_call (GstBus     *bus,
//          GstMessage *msg,
//          gpointer    data);
/// callback function, every time a buffer has arrived
// static int // GstPadProbeReturn
// cb_have_data (GstElement * fakesink,
//               GstBuffer    * buffer,
//               GstPad      * pad,
//               gpointer    * user_data);

static void
fakesink_handoff (GstElement *fakesink,
            GstBuffer  *buffer,
            GstPad     *pad,
            gpointer    user_data);

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

/**
Example plugin to find balls in camera image
@author Christian Andersen
*/
class UFuncV4lCam : public UFuncPlugBasePush
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncV4lCam()
  {
    setCommand("v4lcam v4lcamPush", "v4lcam", "video stream to pool image (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    createBaseVar();
    // initialize local variables
    threadRunning = false;
    threadStop = false;
    threadHandle = -1;
    gloop = NULL;
  };
  /**
  Destructor - to delete the resource (etc) when finished */
  virtual ~UFuncV4lCam()
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
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \param extra pointer that may contained pushed data structure (not used here)
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  bool handleCommand(UServerInMsg * msg, void * extra)
  { // message is unhandled
    bool result = false;
    //
    if (getCmdIndex() == 0)
      result = handleV4lCommand(msg);
    else if (getCmdIndex() == 1)
      result = handlePush(msg);
    else
      sendDebug(msg, "Command not handled (by me)");
    return result;
  }

  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleV4lCommand(UServerInMsg * msg)
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
      sendHelpStart("v4l2pool");
      sendText("--- available V4LGST options\n");
      sendText("Test plugin for gstreamer device\n");
      sendText("open[=false]      open [or close] video stream\n");
      sendText("silent[=false]    Reduce reply and console print to a minimum\n");
      sendText("help              This message\n");
      sendText("see also: 'VAR v4lgst' for parameters\n");
      sendHelpDone();
      sendInfo("done");
      result = true;
    }
    else
    { // resource is available, so make a reply
      bool openRequest = false;
      msg->tag.getAttBool("silet", &silent, true);
      msg->tag.getAttBool("open", &openRequest, true);
      if (openRequest)
      {
        if (threadRunning)
          sendWarning("is open already - done nothing");
        else
        {
          start();
          if (not silent)
          {
            snprintf(reply, MRL, "trying to start stream from %s", varDevice->getString(0));
            sendInfo(reply);
          }
        }
      }
      else
      { // stop thread
        if (threadRunning)
        { // stop streamer
          if (g_main_loop_is_running(gloop))
            stop(true);
          if (not silent)
            sendInfo("stopped streaming");
        }
        else if (not silent)
          sendWarning("stream is not running");
      }
      if (not silent)
      {
        snprintf(reply, MRL, "Not finished yet");
        sendWarning(reply);
      }
    }
    // return true if the function is handled with a positive result
    return result;
  }

protected:

  /**
  Make the variables that will be available to other plugins */
  void createBaseVar()
  {
    varDevice = addVarA("v4ldevice", "/dev/video1", "s", "(r/w) video source device");
    varStreaming = addVar("open", "0 0", "d", "(r) is devise open (streaming) [should-open, is-open] 0=false.");
    varImgCnt = addVar("serial", 0.0, "d", "(r) image serial number (count since open)");
    /// destination image
    varImgPoolImg = addVar("poolImg", 1, "d", "(r/w) destination imagepool number");
    varPose = addVar("pose", "0 0 0 0 0 0", "6d", "(r/w) camera pose (on robot)");
    varImgSize = addVar("imgSize", "896 1600", "d", "(r/w) image size (rows x columns) - must match a supported image size - see $ lsusb -D /dev/bus/usb/00*/*");
    varImgFormat = addVar("format", "rgb", "s", "(r/w) image format - rgb, yuv, bw or bayer - may be supported");
    varGetFramerate = addVar("framerate", "1 1", "d", "(r/w) desired frame rate fraction to image-pool [numerator denominator] integers");
    varReqFramerate = addVar("camFramerate", "10 1", "d", "(r/w) camera frame rate fraction [numerator denominator] integers - must be supported bu camera");
    varGotFrameRate = addVar("gotFramerate", 0.0, "d", "(r) measured frame rate");
  }


///////////////////////////////////////////////////

bool setResource(UResBase * resource, bool remove)
{
  bool result = true;

  if (resource->isA(UImagePool::getResClassID()))
  { // ressource may change
    lock();
    if (remove)
      imgpool = NULL;
    else if (imgpool != (UImagePool*) resource)
      imgpool = (UImagePool*) resource;
    else 
      result = false;
    unlock();
  }
  else
    result = UResBase::setResource(resource, remove);
  return result;
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
  varStreaming->setBool(true, 0);
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
void stop(bool andWait)
{
  if (threadRunning and not threadStop)
  { // stop and join thread
    threadStop = true;
    if (g_main_loop_is_running(gloop))
      g_main_loop_quit(gloop);
    pthread_join(threadHandle, NULL);
    printf("closed camera %s\n", getAliasName());
  }
}

/////////////////////////////////////////////////////

public:
  
/**
 * stream thread
 * assembles processing and starts gstreamer main thread,
 * \returns when main loop is stopped, or an error occured. */
void run()
{
  GstStateChangeReturn ret;
  GstElement *pipeline; //, *filesrc, *decoder, *filter, *sink;
  GstElement *v4l2src, *framerate, *ffmpegcolorspace, *fakesink; //, *queue1;
  //GstElement *convert1, *convert2, *resample;
  //GMainLoop *loop;
  //GstBus *bus;
  //guint watch_id;
  GstCaps *caps, *filtercaps;
  bool isOK = true;
  const int MFL = 30;
  char imgFmt[MFL];
  int n, d;
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
  // video capabilities
  snprintf(imgFmt, MFL, "video/x-raw-%s", varImgFormat->getString());
  n = varReqFramerate->getInt(0);
  d = varReqFramerate->getInt(1);
  caps = gst_caps_new_simple (imgFmt,
            //"format", G_TYPE_STRING, "Y42B",
            "width", G_TYPE_INT, varImgSize->getInt(1),
            "height", G_TYPE_INT, varImgSize->getInt(0),
            "framerate", GST_TYPE_FRACTION, n, d,
            NULL);
  v4l2src  = gst_element_factory_make ("v4l2src", "my_camera");
  g_object_set(G_OBJECT(v4l2src), "device", varDevice->getString(), NULL);
  // to get rather slow frame rates
  framerate = gst_element_factory_make("videorate",  "videorate");
  n = varGetFramerate->getInt(0);
  d = varGetFramerate->getInt(1);
  filtercaps = gst_caps_new_simple ("video/x-raw-rgb",
            //"format", G_TYPE_STRING, "",
            "width", G_TYPE_INT, varImgSize->getInt(1),
            "height", G_TYPE_INT, varImgSize->getInt(0),
            "framerate", GST_TYPE_FRACTION, n, d,
            NULL);
  fakesink = gst_element_factory_make("fakesink",  "fakesink");
  //
  ffmpegcolorspace = gst_element_factory_make("ffmpegcolorspace",  "ffmpegcolorspace");
  //
  gst_bin_add_many (GST_BIN (pipeline), v4l2src, framerate, ffmpegcolorspace, fakesink, NULL);
  //
  if ( !gst_element_link_filtered( v4l2src, framerate, caps) )
  {
    isOK = false;
    g_warning("Failed to link elements v4l2src and framerate");
  }
  if ( !gst_element_link_filtered( framerate, ffmpegcolorspace, filtercaps) )
  {
    isOK = false;
    g_warning("Failed to link elements framerate and ffmpegcolorspace");
  }
  if (isOK and !gst_element_link(ffmpegcolorspace, fakesink) )
  {
    isOK = false;
    g_warning("Failed to link elements ffmpegcolorspace and fakesink");
  }
//   if ( !gst_element_link_filtered( framerate, fakesink, filtercaps) )
//   {
//     isOK = false;
//     g_warning("Failed to link elements framerate and fakesink");
//   }
  if (isOK)
  { // put a trap on framerate input
    g_object_set (G_OBJECT (fakesink),
                  "signal-handoffs", TRUE, NULL);
//                   "sizemax", 384 * 288 * 2,
//                   "sizetype", 2, NULL);
    g_signal_connect (fakesink, "handoff", G_CALLBACK (fakesink_handoff), this);  }
  /* run */
  if (isOK)
    ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  else
    ret = GST_STATE_CHANGE_FAILURE;
  //
  if (ret == GST_STATE_CHANGE_FAILURE) {
//    GstMessage *msg;

    g_print ("Failed to start up pipeline!\n");

    /* check if there is an error message with details on the bus */
//     msg = gst_bus_poll (bus, GST_MESSAGE_ERROR, 0);
//     if (msg) {
//       GError *err = NULL;
// 
//       gst_message_parse_error (msg, &err, NULL);
//       g_print ("ERROR: %s\n", err->message);
//       g_error_free (err);
//       gst_message_unref (msg);
//     }
  }
  else if (not threadStop) 
  { // everithing OK
    // run streamer
    varStreaming->setBool(true, 1);
    g_main_loop_run (gloop);
    varStreaming->setBool(false, 1);
    varStreaming->setBool(false, 0);
  }
  // stream ended
  varStreaming->setBool(false, 0);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
//  g_source_remove (watch_id);
  g_main_loop_unref (gloop);
  gloop = NULL;
  threadRunning = false;
}

///////////////////////////////////////////

/**
 * Call back function from gstreamer sink pad.
 * \param imgdata is pointer to the start of the image buffer.
 * \param imgdataCnt is the size of the buffer in bytes. */
void gotNewImage(unsigned char * imgdata, int imgdataCnt)
{ // got new image - of the agreed size
  //UImagePool * imgpool = (UImagePool *)getStaticResource("imgpool", false, false);
  if (imgpool != NULL)
  {
    UImage * img = imgpool->getImage(varImgPoolImg->getInt(), true);
    img->setSize(varImgSize->getInt(0), varImgSize->getInt(1), 3, 8, varImgFormat->getString());
    if ((int)img->getDataSize() == imgdataCnt)
    { // format is OK
      img->lock();
      img->imgTime.now();
      img->imageNumber = varImgCnt->getInt();
      memcpy(img->getData(), imgdata, imgdataCnt);
      img->updated();
      varImgCnt->add(1.0);
      varGotFrameRate->setDouble(1.0/lastFrame.getTimePassed());
      lastFrame.now();
      img->unlock();
      // inform push stack, that a new image is available
      setUpdated(NULL);
    }
    else
    {
      printf("%s: Image size do not match! source %d destination pool image %d (format %s)\n", aliasName, imgdataCnt, img->getDataSize(), varImgFormat->getString());
    }
  }
}

///////////////////////////////////////////

//
private:
  /// is thread running - set by thread
  bool threadRunning;
  /// is thread to stop - set by main function
  bool threadStop;
  UImagePool * imgpool;
  /**
  Thread handle for processing thread. */
  pthread_t threadHandle;
  /// device string variable
  UVariable *  varDevice;
  /// is port open
  UVariable *  varStreaming;
  /// is port open
  UVariable *  varImgCnt;
  /// is port open
  UVariable *  varGotFrameRate;
  /// image pool image number reference
  UVariable * varImgPoolImg;
  /// image size - for next open 
  UVariable * varImgSize;
  /// image format - not supported yet
  UVariable * varImgFormat;
  /// requested framerate
  UVariable * varGetFramerate;
  /// requested framerate
  UVariable * varReqFramerate;
  /// camera device number
  UVariable * varPose;

  /// framerate
  UTime lastFrame;
/// streamer variables:
  GMainLoop * gloop;
  
};

///////////////////////////////////////////////////////////////////////////////

void * startThread(void * obj)
{ // call the hadling function in provided object
  UFuncV4lCam * plugin = (UFuncV4lCam *)obj;
  plugin->run();
  pthread_exit((void*)NULL);
  return NULL;
}


///////////////////////////////////////////////////////////////////////////////

static void
fakesink_handoff (GstElement *fakesink,
            GstBuffer  *buffer,
            GstPad     *pad,
            gpointer    user_data)
{
  UFuncV4lCam * obj = (UFuncV4lCam *)user_data;
  unsigned char * imgdata = GST_BUFFER_DATA (buffer);
  int imgdataCnt = GST_BUFFER_SIZE (buffer);
  printf("fakesink_handoff: got buffer with %d bytes\n", imgdataCnt);
  obj->gotNewImage(imgdata, imgdataCnt);
}


#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncV4lCam' with your classname, as used in the headerfile */
  return new UFuncV4lCam();
}
#endif

