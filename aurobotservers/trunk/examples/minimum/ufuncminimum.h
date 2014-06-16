/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/
#ifndef UFUNC_MINIMUM_H
#define UFUNC_MINIMUM_H

#include <urob4/ufuncplugbase.h>

/**
 * This module implements a simple plugin - just to show the bare-bone interface.
@author Christian Andersen
*/
class UFuncMinimum : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc (as in UFuncMinimum) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncMinimum()
  { // command list (space separated) and a descriptive short text
    setCommand("minimum", "minimum", "plugin examples");
  }
  /**
   * Handle incomming commands with commands that match the command list above.
   * \param msg pointer to the message and the client issuing the command
   * \param extra if the command origin is from a event-push command, then this extra parameter
   * may hold the object for the event, e.g. a laserscan or an image.
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
};


#endif

