#include <iostream>
#include <gst/gst.h>
#include <gst/gstelementfactory.h>

void testStream();
void testServer();


static gboolean
bus_call (GstBus     *bus,
          GstMessage *msg,
          gpointer    data);

/**
 * gstream test */


gint main (gint   argc,
      gchar *argv[])
{
  /* initialization */
  gst_init (&argc, &argv);
  // testStream();
  testServer();
  return 0;
}


/**
 * some sort of status monitoring */
static gboolean
bus_call (GstBus     *bus,
          GstMessage *msg,
          gpointer    data)
{
  GMainLoop *loop = (GMainLoop *)data;

  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End-of-stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR: {
      gchar *debug = NULL;
      GError *err = NULL;

      gst_message_parse_error (msg, &err, &debug);

      g_print ("Error: %s\n", err->message);
      g_error_free (err);

      if (debug) {
        g_print ("Debug details: %s\n", debug);
        g_free (debug);
      }

      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }

  return TRUE;
}


////////////////////////////////////////////////
///////////////////////////////////////////////
/**
 * Test streaming from v4l video camera */

void testStream()
{
  GstStateChangeReturn ret;
  GstElement *pipeline; //, *filesrc, *decoder, *filter, *sink;
  GstElement *v4l2src, *xvsink, *ffmpegcolorspace; //, *queue1;
  //GstElement *convert1, *convert2, *resample;
  GMainLoop *loop;
  GstBus *bus;
  guint watch_id;
  GstCaps *caps;

  loop = g_main_loop_new (NULL, FALSE);
//   if (argc != 2) {
//     g_print ("Usage: %s <mp3 filename>\n", argv[0]);
//     return 01;
//   }

  /* create elements */
  pipeline = gst_pipeline_new ("my_pipeline");

  /* watch for messages on the pipeline's bus (note that this will only
   * work like this when a GLib main loop is running) */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  // Creat some caps stuff
  caps = gst_caps_new_simple ("video/x-raw-yuv","width", G_TYPE_INT,  
640, "height", G_TYPE_INT, 480, "framerate", GST_TYPE_FRACTION, 10, 1,  
NULL);
//   filesrc  = gst_element_factory_make ("filesrc", "my_filesource");
//   decoder  = gst_element_factory_make ("mad", "my_decoder"); 
  v4l2src  = gst_element_factory_make ("v4l2src", "my_camera"); 
  ffmpegcolorspace = gst_element_factory_make("ffmpegcolorspace",  "ffmpegcolorspace");
//   g_object_set(G_OBJECT(vsrc), "location", "fifo.avi", NULL);
//   queue1 = gst_element_factory_make("queue", "queue1");
  xvsink = gst_element_factory_make("xvimagesink", "x-show");
  /* putting an audioconvert element here to convert the output of the
   * decoder into a format that my_filter can handle (we are assuming it
   * will handle any sample rate here though) */
  //convert1 = gst_element_factory_make ("audioconvert", "audioconvert1");

  /* use "identity" here for a filter that does nothing */
  //filter   = gst_element_factory_make ("my_filter", "my_filter");

  /* there should always be audioconvert and audioresample elements before
   * the audio sink, since the capabilities of the audio sink usually vary
   * depending on the environment (output used, sound card, driver etc.) */
//   convert2 = gst_element_factory_make ("audioconvert", "audioconvert2");
//   resample = gst_element_factory_make ("audioresample", "audioresample");
//   sink     = gst_element_factory_make ("pulsesink", "audiosink");
// 
//   if (!sink || !decoder) {
//     g_print ("Decoder or output could not be found - check your install\n");
//     return -1;
//   } else if (!convert1 || !convert2 || !resample) {
//     g_print ("Could not create audioconvert or audioresample element, "
//              "check your installation\n");
//     return -1;
//   } else if (!filter) {
//     g_print ("Your self-written filter could not be found. Make sure it "
//              "is installed correctly in $(libdir)/gstreamer-1.0/ or "
//              "~/.gstreamer-1.0/plugins/ and that gst-inspect-1.0 lists it. "
//              "If it doesn't, check with 'GST_DEBUG=*:2 gst-inspect-1.0' for "
//              "the reason why it is not being loaded.");
//     return -1;
//   }
// 
//   g_object_set (G_OBJECT (filesrc), "location", argv[1], NULL);

//   gst_bin_add_many (GST_BIN (pipeline), filesrc, decoder, convert1, filter,
//                     convert2, resample, sink, NULL);
  gst_bin_add_many (GST_BIN (pipeline), v4l2src, ffmpegcolorspace, xvsink, NULL);
  if ( !gst_element_link_filtered( v4l2src, ffmpegcolorspace, caps) )
  {
    g_warning("Failed to link elements v4l2src and ffmpegcolorspace");
  }
  if ( !gst_element_link(ffmpegcolorspace,xvsink) )
  {
    g_warning("Failed to link elements ffmpegcolorspace and vqueue");
  }
  /* link everything together */
//   if (!gst_element_link_many (filesrc, decoder, convert1, filter, convert2,
//                               resample, sink, NULL)) {
//     g_print ("Failed to link one or more elements!\n");
//     return -1;
//   }
//   if (!gst_element_link_many (v4l2src, xvsink, NULL)) 
//   {
//     g_print ("Failed to link one or more elements!\n");
//     return -1;
//   }

  /* run */
  ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
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
  else
  { // everithing OK
    // run streamer
    g_main_loop_run (loop);
    // stream ended
    /* clean up */
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (pipeline);
    g_source_remove (watch_id);
    g_main_loop_unref (loop);
  }
}


////////////////////////////////////////////////
///////////////////////////////////////////////
/**
 * Test v4l video server */

void testServer()
{
  GstStateChangeReturn ret;
  GstElement *pipeline; //, *filesrc, *decoder, *filter, *sink;
  GstElement *v4l2src, *x264enc, *rtph264pay, *udpsink, *ffmpegcolorspace; //, *queue1;
  //GstElement *convert1, *convert2, *resample;
  GMainLoop *loop;
  GstBus *bus;
  guint watch_id;
  GstCaps *caps;
  //
  loop = g_main_loop_new (NULL, FALSE);
  /* create elements */
  pipeline = gst_pipeline_new ("my_pipeline");

  /* watch for messages on the pipeline's bus (note that this will only
   * work like this when a GLib main loop is running) */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);
  // video capabilities
  caps = gst_caps_new_simple ("video/x-raw-yuv","width", G_TYPE_INT,  
            // "format", G_TYPE_STRING, "RGB16",
            640, "height", G_TYPE_INT, 480,
            "framerate", GST_TYPE_FRACTION, 10, 1,
            NULL);
  v4l2src  = gst_element_factory_make ("v4l2src", "my_camera"); 
  g_object_set(G_OBJECT(v4l2src), "device", "/dev/video0", NULL);
  // may be needed
  ffmpegcolorspace = gst_element_factory_make("ffmpegcolorspace",  "ffmpegcolorspace");
  // x264enc pass=qual quantizer=20 tune=zerolatency
  x264enc = gst_element_factory_make("x264enc", "server_x264enc");
  g_object_set(G_OBJECT(x264enc)/*, "pass", 1*/ /*GST_X264_ENC_PASS_QUAL*/, "quantizer", 20, "tune", 7 /* GST_X264_ENC_TUNE_ZEROLATENCY*/,  NULL);
  //
  // rtph264pay ! udpsink host=127.0.0.1 port=1234
  rtph264pay = gst_element_factory_make("rtph264pay", "server_rtph264pay");
  //
  udpsink = gst_element_factory_make("udpsink", "server_udpsink");
  g_object_set(G_OBJECT(udpsink), "host", "127.0.0.1",  "port", 2500,  NULL);
  //
  gst_bin_add_many (GST_BIN (pipeline), v4l2src, ffmpegcolorspace, x264enc, rtph264pay, udpsink, NULL);
  //
  if ( !gst_element_link_filtered( v4l2src, ffmpegcolorspace, caps) )
  {
    g_warning("Failed to link elements v4l2src and ffmpegcolorspace");
  }
  if ( !gst_element_link(ffmpegcolorspace,x264enc) )
  {
    g_warning("Failed to link elements ffmpegcolorspace and x264enc");
  }
  if ( !gst_element_link(x264enc, rtph264pay) )
  {
    g_warning("Failed to link elements x264enc and rtph264pay");
  }
  if ( !gst_element_link(rtph264pay, udpsink) )
  {
    g_warning("Failed to link elements rtph264pay and udpsink");
  }
  /* run */
  ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
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
  else
  { // everithing OK
    // run streamer
    g_main_loop_run (loop);
    // stream ended
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (pipeline);
    g_source_remove (watch_id);
    g_main_loop_unref (loop);
  }
}


/**
 * her er nok noget */

// static GMainLoop *loop;
// 
// static void
// cb_need_data (GstElement *appsrc,
//               guint       unused_size,
//               gpointer    user_data)
// {
//   static gboolean white = FALSE;
//   static GstClockTime timestamp = 0;
//   GstBuffer *buffer;
//   guint size;
//   GstFlowReturn ret;
// 
//   size = 385 * 288 * 2;
// 
//   buffer = gst_buffer_new_allocate (NULL, size, NULL);
// 
//   /* this makes the image black/white */
//   gst_buffer_memset (buffer, 0, white ? 0xff : 0x0, size);
//   
//   white = !white;
// 
//   GST_BUFFER_PTS (buffer) = timestamp;
//   GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 2);
// 
//   timestamp += GST_BUFFER_DURATION (buffer);
// 
//   g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
// 
//   if (ret != GST_FLOW_OK) {
//     /* something wrong, stop pushing */
//     g_main_loop_quit (loop);
//   }
// }
// 
// gint
// main (gint   argc,
//       gchar *argv[])
// {
//   GstElement *pipeline, *appsrc, *conv, *videosink;
// 
//   /* init GStreamer */
//   gst_init (&argc, &argv);
//   loop = g_main_loop_new (NULL, FALSE);
// 
//   /* setup pipeline */
//   pipeline = gst_pipeline_new ("pipeline");
//   appsrc = gst_element_factory_make ("appsrc", "source");
//   conv = gst_element_factory_make ("videoconvert", "conv");
//   videosink = gst_element_factory_make ("xvimagesink", "videosink");
// 
//   /* setup */
//   g_object_set (G_OBJECT (appsrc), "caps",
//                 gst_caps_new_simple ("video/x-raw",
//                                      "format", G_TYPE_STRING, "RGB16",
//                                      "width", G_TYPE_INT, 384,
//                                      "height", G_TYPE_INT, 288,
//                                      "framerate", GST_TYPE_FRACTION, 0, 1,
//                                      NULL), NULL);
//   gst_bin_add_many (GST_BIN (pipeline), appsrc, conv, videosink, NULL);
//   gst_element_link_many (appsrc, conv, videosink, NULL);
// 
//   /* setup appsrc */
//   g_object_set (G_OBJECT (appsrc),
//                 "stream-type", 0,
//                 "format", GST_FORMAT_TIME, NULL);
//   g_signal_connect (appsrc, "need-data", G_CALLBACK (cb_need_data), NULL);
// 
//   /* play */
//   gst_element_set_state (pipeline, GST_STATE_PLAYING);
//   g_main_loop_run (loop);
// 
//   /* clean up */
//   gst_element_set_state (pipeline, GST_STATE_NULL);
//   gst_object_unref (GST_OBJECT (pipeline));
//   g_main_loop_unref (loop);
// 
//   return 0;
//   }

