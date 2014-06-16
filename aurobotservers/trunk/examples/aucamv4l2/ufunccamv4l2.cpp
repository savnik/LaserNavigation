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

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>

#include <gst/gst.h>
#include "ufunccamv4l2.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncCamV4l2' with your classname, as used in the headerfile */
  return new UFuncCamV4l2();
}

#endif

///////////////////////////////////////////////////

bool UFuncCamV4l2::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 500;
  char reply[MRL];
  bool aLog, doLog;
  bool aReset;
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("v4l2");
    sendText("--- MINIVAR is a demonstration plug-in that counts number of calls and note the odometry pose and sets a few matrices\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    sendText(            "update              Update variables (default behaviour)\n");
    sendText(            "reset               Reset variables\n");
    sendText("help       This message\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    aLog = msg->tag.getAttBool("log", &doLog, true);
    aReset = msg->tag.getAttBool("reset", NULL, false);
    bool anUpdate = msg->tag.getAttBool("update", NULL, false);
    //
    // implement command
    if (aLog)
    { // open or close the log
      aLog = logf.openLog(doLog);
      if (doLog)
        snprintf(reply, MRL, "Opened logfile %s (%s)", logf.getLogFileName(), bool2str(aLog));
      else
        snprintf(reply, MRL, "closed logfile %s", logf.getLogFileName());
      // send an information tag back to client
      sendInfo(reply);
    }
    if (aReset)
    {
      resetVars();
      sendInfo("Values reset");
    }
    if (anUpdate)
    { // default update call
      update();
      // format reply as valid XML tag
      snprintf(reply, MRL, "<%s cnt=\"%d\" tod=\"%.3f\"/>\n", msg->tag.getTagName(), 
			   varUpdateCnt->getInt(0), varTime->getDouble());
      // send reply to client
      sendMsg(reply);
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncCamV4l2::createResources()
{
  // Create global variables - owned by this plug-in.
  // Returns a pointer to the the variable for easy access.
  varLogName = addVar("logname", "exvar", "s", "(r) name of the logfile - is placed in dataPath");
  varUpdateCnt = addVar("updCnt", 0.0, "d", "(r/w) Number of updates");
  varPose = addVar("pose", "0 0 0", "pose", "(r) Odometry pose at last update");
  varTime = addVar("time", 0.0, "t", "(r) Time at last update");
  varAUMatrix = addVar("mau", "1 2 3; 2 1 2; 3 2 1", "m", "(r/w) AU Matrix 3x3");
  varCVMatrix = addVar("mcv", "1 2 3; 2 1 2; 3 2 1", "m", "(r/w) CV Matrix 3x3");
}

////////////////////////////////////////////////////////////////

void UFuncCamV4l2::resetVars()
{
  UVarPool * vp;
  //
  // set CV matrix to a identity 3x3 matrix
  varCVMatrix->setValued("0 0 0; 0 0 0; 0 0 0", 0, true);
  // set AU matrix to a 3x3 matrix
  varAUMatrix->setValued("0 0 0; 0 0 0; 0 0 0", 0, true);
  // set update count
  varUpdateCnt->setInt(0);
  // set time (to now)
  varTime->setTimeNow();
  // reset pose [x, y, h]
  varPose->setValued("0 0 0", 0, true);
  //
  varLogName->setValues(logf.getLogName(), 0, true);
  //
  // reset trip counter B
  vp = getVarPool();
  vp->setGlobalVar("odoPose.tripB", 0.0, false);
}

////////////////////////////////////////////////////////////////

int nn = 0;

static gboolean
bus_call (GstBus     *bus,
          GstMessage *msg,
          gpointer    data)
{
  GMainLoop *loop = (GMainLoop *) data;

  switch (GST_MESSAGE_TYPE (msg)) {

    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;

    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;

      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);

      g_printerr ("Error: %s\n", error->message);
      g_error_free (error);

      g_main_loop_quit (loop);
      break;
    }
    default:
	  nn++;
	  if (nn >= 100)
        g_main_loop_quit (loop);
      break;
  }
  printf(".");
  return TRUE;
}


static void
on_pad_added (GstElement *element,
              GstPad     *pad,
              gpointer    data)
{
  GstPad *sinkpad;
  GstElement *decoder = (GstElement *) data;

  /* We can now link this pad with the vorbis-decoder sink pad */
  g_print ("Dynamic pad created, linking demuxer/decoder\n");

  sinkpad = gst_element_get_static_pad (decoder, "sink");

  gst_pad_link (pad, sinkpad);

  gst_object_unref (sinkpad);
}


void UFuncCamV4l2::update()
{
  GMainLoop *loop;

  GstElement *pipeline, *source, *muxer, /* *decoder, *conv,*/ *sink;
  GstBus *bus;
  bool result = true;
  /* Initialisation */
  gst_init (NULL, NULL);
  loop = g_main_loop_new (NULL, FALSE);

  /* Create gstreamer elements */
  pipeline = gst_pipeline_new ("audio-player");
  source   = gst_element_factory_make ("v4l2src",       "video-source");
//  source   = gst_element_factory_make ("filesrc",       "file-source");
  muxer  = gst_element_factory_make ("mpeg2enc",      "mpeg-muxer");
//  demuxer  = gst_element_factory_make ("oggdemux",      "ogg-demuxer");
//  decoder  = gst_element_factory_make ("vorbisdec",     "vorbis-decoder");
//  conv     = gst_element_factory_make ("audioconvert",  "converter");
//  sink     = gst_element_factory_make ("autoaudiosink", "audio-output");
  sink     = gst_element_factory_make ("filesink", "file-output");

  if (!pipeline || !source || !muxer /*|| !decoder || !conv*/ || !sink) {
    g_printerr ("One element could not be created. Exiting.\n");
	result = false;
  }
  if (result)
  { /* Set up the pipeline */
    /* we set the input filename to the source element */
	//g_object_set (G_OBJECT (source), "location", "/dev/video0", NULL);
	g_object_set (G_OBJECT (source), "device", "/dev/video0", NULL);
	g_object_set (G_OBJECT (sink), "location", "testvideo.m1v", NULL);

	/* we add a message handler */
	bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
	gst_bus_add_watch (bus, bus_call, loop);
	gst_object_unref (bus);

	/* we add all elements into the pipeline */
	/* file-source | ogg-demuxer | vorbis-decoder | converter | alsa-output */
	gst_bin_add_many (GST_BIN (pipeline),
					  source, muxer, /*decoder, conv,*/ sink, NULL);

	/* we link the elements together */
	/* file-source -> ogg-demuxer ~> vorbis-decoder -> converter -> alsa-output */
	gst_element_link (source, muxer);
	gst_element_link (muxer, sink);
//	gst_element_link_many (decoder, conv, sink, NULL);
//	g_signal_connect (source, "pad-added", G_CALLBACK (on_pad_added), muxer);

	/* note that the demuxer will be linked to the decoder dynamically.
	  The reason is that Ogg may contain various streams (for example
	  audio and video). The source pad(s) will be created at run time,
	  by the demuxer when it detects the amount and nature of streams.
	  Therefore we connect a callback function which will be executed
	  when the "pad-added" is emitted.*/


	/* Set the pipeline to "playing" state*/
	g_print ("Now playing: %s\n", "a sound");
	gst_element_set_state (pipeline, GST_STATE_PLAYING);


	/* Iterate */
	g_print ("Running...\n");
	g_main_loop_run (loop);


	/* Out of the main loop, clean up nicely */
	g_print ("Returned, stopping playback\n");
	gst_element_set_state (pipeline, GST_STATE_NULL);

	g_print ("Deleting pipeline\n");
	gst_object_unref (GST_OBJECT (pipeline));
  }
}
