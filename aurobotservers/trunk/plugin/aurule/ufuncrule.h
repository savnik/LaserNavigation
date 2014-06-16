/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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
#ifndef UFUNC_RULE_H
#define UFUNC_RULE_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>

#include "uresrule.h"
#include "uresrulestate.h"
//#include "uresdummy.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Example plugin that demonstrates a plogin that provides a resource.
A similar example plugin is available that uses the shared resource.

The shared resource provides the simple functionality in the form of a line.
The resource provides functions to calculate the length of the line.

@author Christian Andersen
*/
class UFuncRule : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFuncRule) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncRule()
  { // set commands handled
    setCommand("rule", "ruleif", "interface to rule-based execution");
    // initialization of variables in class - as needed
    seq = NULL;  // initially the resource is not created
    rules = NULL;
//    resDum = NULL;
  }
  /**
  Destructor */
  virtual ~UFuncRule();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor. */
  virtual void createResources();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);

protected:
  /**
   * Handle the PLAN messages */
  bool handleRuleSet(UServerInMsg * msg);
  /**
   * Send current status for mission and plan control */
  bool sendStateMessage(UServerInMsg * msg,
                         const char * tagName);
  /**
   * Load this mission file */
  bool loadRuleFile(UServerInMsg * msg, const char * name);
  /**
   * Send a list of all missions to client
   * \param msg is the command info
   * \param listSource is true, when source is requested - else state
   * \param filename is the filename to print to - if null, then to console */
  bool listAllMissions(UServerInMsg * msg, const char * filename);
  /**
   * List a mission to the console
   * \param msg is the command info
   * \param name is the mission plan name to list
   * \param filename is the filename to print to - if null, then to console */
  bool listMission(UServerInMsg * msg, const char * name, const char * filename);
  /**
   * List a mission to the console
   * \param msg is the command info
   * \param filename is the filename to print to - if null, then to console
   * \returns true if result is send to client/console */
  bool listState(UServerInMsg * msg, const char * filename);
  /**
   * Unload a specific plan or all plans from this source file
   * \param msg the source message and channel information
   * \param name name of a specific plan (not case sensitive) or the filename used when loading the plan(s)
   * \returns true */
  bool unloadRule(UServerInMsg * msg, const char * name);
  /**
   * Load a plan into the sequencer and possibly start it (else it is paused)
   * \param msg handle to the client sending the command
   * \param name the name (not case sensitive) of the mission to load
   * \param start plan if true, otherwise stop - or no change if in this state already
   * \return true (as a reply is send to the client) */
  bool startStopRule(UServerInMsg * msg, const char * name, bool start);
  /**
   * Pause or unpause the current plan in the sequenser
   * \param msg is the link to the client
   * \param value if true then the plan is paused
   * else the plan is started or resumed
   * \return true (reply is send) */
//  bool pauseRule(UServerInMsg * msg, bool value);
  /**
   * Run a single or a number of sequencer iteration steps
   * \param msg is the link to the client
   * \param value is the number of steps to be performed
   * \return true (reply is send) */
  bool runStep(UServerInMsg * msg, int value);
  /**
   * Run a single or a number of sequencer iteration steps
   * \param msg is the link to the client
   * \param open if true the files are opened, if false, they are closed */
  bool openLog(UServerInMsg * msg, bool open);
  /**
   * Create or append the editStr to this plan.
   * \param msg is the link to the client
   * \param name is the name of the plan to work on
   * \param editStr is the line or lines of data to add to the plan
   * \param complete completes, i.e. create a state for the plan and makes it active
   * \returns true if a reply is send to client */
  bool editRule(UServerInMsg * msg, const char * name,
                          const char * editStr,
                          const bool complete);

protected:
  /**
  pointer to mission state sequencer */
  UResRuleState * seq;
  /**
  pointer to mission list resource */
  UResRule * rules;
  /**
   * pointer to dummy plugin */
//  UResDummy * resDum;

private:
};


#endif

