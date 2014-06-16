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
#ifndef UIMGPUSH_H
#define UIMGPUSH_H

#include <gstreamer-0.10/gst/gst.h>
#include <ugen4/uimage2.h>
#include "ucmdexe.h"

/**
Image with push capabilities
@author Christian Andersen
*/
class UImgPush : public UImage, public UServerPush
{
public:
  /**
  Constructor */
  UImgPush(const unsigned int height = 480, const unsigned int width = 640, int channels = 3, int depth = 8);
  /**
  Destructor */
  virtual ~UImgPush();
  /**
  Add a camera push command.
  A camera push command is triggered when a
  new image is available. */
  inline int addPoolPushCommand(UServerInMsg * msg)
  { return addPushCommand(msg); };
  /**
  Image is updated now, note this in client handler */
  virtual void imgUpdated();
  /**
   * create local variables
   * Should be called no more than once, and not if not to create global variables with status
   * \param parent parent structure in global variable tree.
   *  */
  void createLocalVariables(UVarPool * parent);
  /**
   * Set TCP port number for streaming - disconnecting ant clients */
  bool setTcpPort( int port);
  /**
   * Set TCP port number for streaming - disconnecting ant clients */
  int getTcpPort()
  {
    return varTcpPort->getInt();
  }
  /**
   * start streaming, that is set the flag for stream start, but
     actual start requires that image is updated. */
  void startStreaming(bool doStart)
  {
    varStreaming->setBool(doStart, 0);
    imgUpdated();
  }
  /**
   * start streaming, that is set the flag for stream start, but
     actual start requires that image is updated. */
  void startShowing(bool doStart)
  {
    varShow->setBool(doStart, 0);
    imgUpdated();
  }
  
protected:
  /**
  Called from push structure to get push object
  followed by a call to 'gotNewData(object)'.
  Returns false if no push commands are valid. */
  inline virtual void callGotNewDataWithObject()
  { 
    gotNewData(this); 
    //printf("UImgPush::callGotNewDataWithObject pending push cmds=%d\n", getPushQueue()->getPushCmdCnt());
  };
  /** stop read thread
    \param andWait waits until thread is terminated, when false, then the call returns
    when the stop flag is set (i.e. immidiately). */
  /**
   * Start stream processing thread */
  bool start();
  void stop(bool andWait);
  /**
   * Start gstream show thread */
  bool startShow();
  void stopShow(bool andWait);

  
public:
  /**
  * stream thread using fake source - image-pool to stream
  * assembles processing and starts gstreamer main thread,
  * \returns when main loop is stopped, or an error occurred. */
  void run();
  /**
  * image show thread using fake source - image-pool to x-window
  * assembles processing and starts gstreamer main thread,
  * \returns when main loop is stopped, or an error occurred. */
  void runShow();
  /**
  * Fill fake source buffer with new data from pool image for streaming */
  void fillBuffer(GstElement *fakesrc,
                          GstBuffer *buffer,
                          GstPad    *pad);
  /**
  * Fill fake source buffer with new data from pool image - for direct show */
  void fillShowBuffer(GstElement *fakesrc,
                        GstBuffer *buffer,
                        GstPad    *pad);
  /**
   * Status change in stream */
//   void busCall (GstBus  * bus,
//           GstMessage * msg,
//           gpointer    data);

private:
  /// link to global variable tree
  UVarPool * varPoolParent;
  /// size of image
  UVariable * varSize;
  /// image serial number
  UVariable * varSerial;
  /// image serial number
  UVariable * varTime;
  /// (r/w) is streaming active
  UVariable * varShow;
  /// (r/w) stream at this update interval
  UVariable * varDesiredUpdateRate;
  /// (r/w) is streaming active
  UVariable * varStreaming;
  /// streaming port
  UVariable * varTcpPort;
  /// UDP streaming clients
  //UVariable * varUdpClients;

/// for image stream
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
  /// is port open
  UVariable *  varFrameRate;
  /// last image number
  UTime lastUpdateTime;
  /// streamer loop - to be able to stop:
  GMainLoop * gloop;  
  /// is thread running - set by thread
  bool threadShowRunning;
  /// is thread to stop - set by main function
  bool threadShowStop;
  /**
  Thread handle for processing thread. */
  pthread_t threadShowHandle;
  /// streamer loop - to be able to stop:
  GMainLoop * gloop_show;
  /// last image number
  UTime lastShowTime;
};

#endif
