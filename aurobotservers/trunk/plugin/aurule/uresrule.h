/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
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

#ifndef URESRULE_H
#define URESRULE_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/usmrcl.h>
#include <urob4/uresbase.h>
#include <urob4/ulogfile.h>

#include "urule.h"

class UMisLoadedRule : public UMisRule
{
public:
  /**
   * Constructor */
  UMisLoadedRule();
  /**
   * Destructor */
  virtual ~UMisLoadedRule();
  /**
   * Unpack a plan from this tag
   * \param cnn is a connection to the SML (XML) file
   * \param tag is the open tag for the plan
   * \returns true if unpacked successfully */
  bool unpack(USmlSource * cnn, USmlTag * tag, UVarCalc * calc);
  /**
   * Print mission plan to buffer
   * \param preStr a string to add before the plan lines
   * \param buf a string buffer to hold the plan
   * \param bufCnt the size of the buffer  */
//  void print(const char * preStr, char * buf, const int bufCnt);
  /**
   * Is the top plan the default plan
   * \returns true if plan is marked as the default plan (in the file) */
  bool isDefRule();
  /**
   * Is this plan in editing mode - i.e. append is allowed
   * \returns true if plan is marked as under edit, i.e. not runable and not active */
  virtual bool isEdit()
  { return edit; };
  /**
   * set this plan in editing mode or not - i.e. append is allowed if in edit mode */
  void setEdit(bool value)
  { edit = value; };
  /**
   * Adjust the number of times this plan is beeing used by the
   * sequencer - add 1 every time it is used in a plan state and remove one
   * when plan state is released
   * \returns count after the adjustment. */
  virtual int setBusyCnt(int delta);
  /**
   * Is this plan busy - i.e. used by the sequencer.
   * The busy cnt can be set from a locked resource only, so just
   * \return true if the count is not zero. */
  bool isBusy()
  { return busyCnt > 0; };
  /**
   * \brief add line or lines to to the unloaded bart of plan
   * \param lines a zero terminated string to be added
   * \returns true if added (otherwise the memory allocation failed) */
  bool addEditLines(const char * lines);
  /**
   * Add the potential lines in the edit buffer
   * \param calculator reference that should be used for any load calculations
   * \returns true if errors were found during unpack. */
  bool completeEdit(UVarCalc * calc);

protected:

  /**
   * Strip source string from escaped new-line combinaltion "\n" and replace with '\n'.
   * \param dest destination string, assumed to be able to hold the stripped version of source
   * \param source source string - zero terminated. */
  void stripEscape(char * dest, const char * source);
  /**
   * Get the edit lines (un-implemented parts of a plan)
   * \returns NULL if no edit lines are available. */
  virtual const char * getEditLines()
  { return editLines; };


public:
  /**
   * Pointer to next plan */
  UMisLoadedRule * nextRule;
  /**
   * Source filename for the mission plan */
  char sourceFileName[MAX_FILENAME_LENGTH];

private:
  /// Number of times this (root) plan is beeing used
  int busyCnt;
  /// is the plan fully loaded or is it still beeing edited
  bool edit;
  /// unloaded part of plan
  char * editLines;
};
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResRule : public UResVarPool, public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResRule) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResRule()
  { // these first two lines are needed
    // to save the ID, version number and description
    setResID("rule", 205);
    // set description for global variables owned by this resource (optional)
    setDescription("rule definition database", false);
    // create additional (global) variables for this resource
    createBaseVar();
    // set logfile name to the same as this resource
    setLogName(getResID());
    //openLog();
    // other local initializations
    planRoot = NULL;
    synErrBuf[0] = '\0';
//    def[0] = '\0';
  };
  /**
  Destructor */
  virtual ~UResRule();
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
// the above methods are used by the server core and should be present
// the print function is not strictly needed, but is nice.

public:
  /**
   * Get number of loaded missions.
   *  \returns total number of missions (executable or not) */
  int getMissionCnt();
  /**
   * Load a mission file */
  bool loadFile(const char * name);
  /**
   * Get syntax error buffer */
  const char * getSyntaxErrorString()
  { return synErrBuf; };
  /**
   * Get a list of a mission or a list of loaded missions
   * \param name is either "" or a name of a valid mission
   * \param buf is the buffer where to store the result
   * \param bufCnt is the size of the buffer
   * \returns true if data is in the buffer */
  bool getRuleList(const char * name, char * buf, const int bufCnt);
  /**
   * Find a plan with this name.
   * \param name is the name of the plan to get, plan name is not case sensitive
   * \param runableRulesOnly, if true, then plans in edit mode gets ignored
   * \returns a pointer to the found plan or NULL if not found. */
  UMisLoadedRule * getRule(const char * name, bool runableRulesOnly);
  /**
   * Find a plan with this index.
   * The index should be between 0 and (getMissionCount() - 1).
   * \returns a pointer to the found plan or NULL if not found. */
  UMisLoadedRule * getRule(const int idx);
  /**
   * Unload this specific plan if it it is not busy
   * \param name the not case sensitive plan name
   * \return true if plan is deleted - otherwise the plan is not found or busy */
  bool tryUnloadRule(const char * name);
  /**
  Set default plan */
  void setDefRule(const char * defRule);
  /**
   * \brief Add a new rule in edit mode to the list of plans.
   * The new plan is not a rule, and will not be active.
   * It is based on loaded plan, but nothing is entered and the editStr is empty.
   * \param name is the name of the new plan.
   * \returns a pointer to the new plan */
  UMisLoadedRule * addNewRule(const char * name);
  /**
   * Complete the pllan with the lines in the editString buffer
   * \param plan the plan to be completed against the global variable base
   * \returns true if successful */
  bool completeEdit(UMisLoadedRule * plan);

protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();
  /**
   * Unpack and gcreate a new plan tree.
   * \param cnn is the connection to a source (file)
   * \param tag is the opening plan tag
   * \param fileName pointer for source filename (for on-line debug)
   * \returns true if plan is unpacked and added, fails also
   * is a plan with that name already exists.  */
  bool unpackMissionRule(USmlSource * cnn, USmlTag * tag, const char * fileName);
  /**
   * Append a (new) plan at the end of the loaded plan list.
   * \param plan is the new plan to be added. The 'next' pointer
   * is not changed, and should be set to a valid plan or NULL. */
  void appendRule(UMisLoadedRule * plan);

protected:
  /**
   * Root of mission plan tree.
   * The base of the tree is loaded plans, that is including file source etc. */
  UMisLoadedRule * planRoot;
  /** size of syntax error buffer */
  static const int SYN_ERR_BUF_SIZE = 10000;
  /** syntax error buffer */
  char synErrBuf[SYN_ERR_BUF_SIZE];
  /** index to mission count */
  UVariable * varMisCount;
};




#endif

