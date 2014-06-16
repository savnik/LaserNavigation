/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/

#ifndef UFUNC_FUNCV4L2_H
#define UFUNC_FUNCV4L2_H

#include <urob4/ufuncplugbase.h>

/**
 * This is a simple example plugin, that manipulates global variables, both of its own, and those of other modules.
@author Christian Andersen
*/
class UFuncCamV4l2 : public UFuncPlugBase
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncCamV4l2) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncCamV4l2()
  { // command list and version text
    setCommand("v4l2", "v4l2", "Plugin examples using v4l2");
    logf.setLogName("v4l2");
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
  /**
  Reset global variables */
  void resetVars();
  /**
  A function to manipulate global variables */
  void update();
  /**
  Handles to "own" global variables. */
  UVariable * varLogName;
  UVariable * varUpdateCnt;
  UVariable * varPose;
  UVariable * varTime;
  UVariable * varAUMatrix;
  UVariable * varCVMatrix;
  /**
  Logfile */
  ULogFile logf;
};


#endif
