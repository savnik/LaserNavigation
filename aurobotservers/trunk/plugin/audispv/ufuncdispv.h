/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/
#ifndef UFUNC_AUDISPV_H
#define UFUNC_AUDISPV_H

#include <ugen4/uimg3dpoint.h>
#include <urob4/ufuncplugbase.h>
//#include "pseudocolor.h"
#include <QtGui/QMainWindow>
#include <QtGui/QLabel>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QAction>
#include <QtGui/QApplication>


/**
 * This stereo plugin takes two unprocessed (raw) images and produce a diaparity image and a 3D cloud, based on a calibration values and correlation parameters in the global variables.
@author Christian Andersen
*/
class UFuncDispV : public UFuncPlugBasePush
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncDispV) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncDispV()
  { // command list and version text
    setCommand("dispv", "dispv", "Plug-in to test QT window");
    logf.setLogName("dispv");
    threadRunning = false;
    qmw = NULL;
  }
  /**
  Destructor */
  ~UFuncDispV();
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
  /// start read thread
  bool start();
  /** stop read thread
  \param andWait waits until thread is terminated, when false, then the call returns
  when the stop flag is set (i.e. immidiately). */
  void stop(bool andWait);
  /**
  Run QT GUI thread */
  void run();
  //
private:
  /**
  Handle stereo-push commands */
  bool handleDispv(UServerInMsg * msg);

private:

  static const int MWL = 100;
  /**
  Short explanation to why processing failed */
  char whyString[MWL];
  /**
  Pointers to "own" global variables. */
  UVariable * varUpdateCnt;
  UVariable * varTime;
  /**
  Logfile */
  ULogFile logf;
  /**
  Thread handle for GUI thread. */
  pthread_t threadHandle;
  /// GUI thread status
  bool threadRunning;
  
protected:
  QMainWindow * qmw;
  QApplication * qapp;

};


#endif

