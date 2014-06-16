/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/
#ifndef UFUNC_IMU_H
#define UFUNC_IMU_H

#include <urob4/ufuncplugbase.h>

/**
 * This is a simple example plugin, that manipulates global variables, both of its own, and those of other modules.
@author Christian Andersen
*/
class UFuncImu : public UFuncPlugBase
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncImu) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncImu()
  { // command list and version text
    setCommand("imu", "imu", "Plugin examples using global variables");
    logf.setLogName("imu");
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
public:
  /**
  Run receive thread */
  void run();
    
private:
  /**
  Handles to "own" global variables. */
  UVariable * varCnt;
  UVariable * varRot;
  UVariable * varTime;
  UVariable * varPose;
  UVariable * varAcc;
  UVariable * varGyro;
  UVariable * varMag;
  UVariable * varOpen;
  UVariable * varIsOpen;
  UVariable * varDevice;
  UVariable * varBaud;
  UVariable * varErrCnt;
  UVariable * varErrMsg;
  UVariable * varUpdateRate;
  UVariable * varDataSample;
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
};


#endif

