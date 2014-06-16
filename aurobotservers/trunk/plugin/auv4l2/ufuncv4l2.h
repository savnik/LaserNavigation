/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/
#ifndef UFUNC_IMU_H
#define UFUNC_IMU_H

#include <urob4/ufuncplugbase.h>
#include "defs.h"
#include "v4l2_devices.h"
#include "v4l2_formats.h"
#include "v4l2_controls.h"
#include "v4l2uvc.h"
#include "jdatatype.h"
#include <ugen4/uimage.h>

/// @todo - not in a working state

typedef uint32_t guint;
typedef char gchar;


/**
 * This is a simple example plugin, that manipulates global variables, both of its own, and those of other modules.
@author Christian Andersen
*/
class UFuncv4l2 : public UFuncPlugBase
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncv4l2) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncv4l2()
  { // command list and version text
    setCommand("v4l2", "v4l2", "Plugin examples using global variables");
    logf.setLogName("v4l2");
    init();
  }
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  //
private:
  /// init local variables */
  void init();
  /// start read thread
  bool start();
  /** stop read thread
  \param andWait waits until thread is terminated, when false, then the call returns
  when the stop flag is set (i.e. immidiately). */
  void stop(bool andWait);
  /** initialize video mode */
  bool initDevice();
  /**
   * process video ??? */
  bool process_video(VidBuff *proc_buff,
                struct JPEG_ENCODER_STRUCTURE **jpeg_struct);

public:
  /**
  Run receive thread */
  void run();

private:
  /**
  Handles to "own" global variables. */
  UVariable * varCnt;
  UVariable * varTime;
  UVariable * varPose;
  UVariable * varOpen;
  UVariable * varIsOpen;
  UVariable * varDevice;
  UVariable * varCamDev;
  UVariable * varErrCnt;
  UVariable * varErrMsg;
  UVariable * varUpdateRate;
  UVariable * varDataSample;
  UVariable * varPoolImage;
  UVariable * varImageFormat;
  /**
  Logfile */
  ULogFile logf;
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /** receive thread running flag */
  bool threadRunning;
  /** flag to stop receive flag */
  bool threadStop;
  /// new for V4L2
  vdIn videoIn;
  /// GLOBAL status
  GLOBAL global;
  /// file length
  static const int MAX_FILENAME_CNT = 100;
  /// config file name
  char confFileName[MAX_FILENAME_CNT];
  /// video device name
  char videoDevice[MAX_DEV_NAME_CNT];
  /// is device open
  bool isOpen;
  /// pool image
  int poolImageNum;
  /// pool image
  UImage * poolImage;
  /// pool image
  int jpegImageNum;
  /// pool image
  UImage * jpegImage;
};


#endif

