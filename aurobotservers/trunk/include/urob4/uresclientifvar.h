/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef URESCLIENTIFVAR_H
#define URESCLIENTIFVAR_H

//#include "uclientfuncbase.h"
//#include "uresvarpool.h"
#include "uresifbase.h"

class UResPoseHist;
class USmlTag;

/**
Handling of messages that should be converted into simple variables in the var pool

	@author Christian <chrand@mail.dk>
*/
class UResIfVar : public UResIfBase //public UClientFuncBase, public UResVarPool
{
public:
  /**
  Constructor */
  UResIfVar(const char * resID)
  { // set name and version number
    setResID(resID, 1515);
    verboseMessages = true;
    poseHist = NULL;
    poseUtm = NULL;
    poseMap = NULL;
    createVarSpace(5, 0, 0, "Variable related to a server interface client", false);
    //createBaseVar();
    strncpy(tagList, "var", MAX_TAG_LIST_SIZE);
  };
  /**
  Destructor */
  ~UResIfVar();
  /**
   * Static ID for this resource*/
/*  static const char * getResClassID()
  { return "ifVar"; };*/
  /**
  Called by the server core. Should return the
  name of function. There should be a first short part separated
  by a space to some additional info (e.g. version and author).
  The returned name is intended as informative to clients
  and should include a version number or compile data */
  virtual const char * name();
  /**
  Called by the server core when loaded, to get a list of
  keywords (commands) handled by this plugin.
  Return a list of handled functions in
  one string separated by a space (not case sensitive).
  e.g. return "COG Line".
  The functions should be unique on the server. */
  virtual const char * commandList();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class)
   * Resource shut-down code should be handled in the resource destructor.
   * \return true if any resources are created. */
  virtual void createResources();
  /**
  Create base variables in varPool for this interface type */
  void createBaseVar();
  /**
  Called when resource situation has changed. */
  virtual bool setResource(UResBase * res, bool remove);
  /**
  Test if all needed resources are loaded.
  'missingThese is a buffer where missing resources should be written.
  'missingTheseCnt' isthe buffer size not to be resexceeded.
  Should return true if all resources are available. */
  //virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  * New data handled by this object has arrived.
    Starting with this first tag.
  * The function should request all data from line until the corresponding end-tag
  * is received. (no action needed if the tag is a full tag.) */
  void handleNewData(USmlTag * tag);
  /**
  Add tag type to list of handled tag types */
  void addTags(const char * tags);
  /**
  Print status for this resource */
  virtual char * snprint(const char * preString, char * buff, const int buffCnt);
  /**
  Print status for this resource */
  virtual const char * print(const char * preString, char * buff, const int buffCnt)
  {  return snprint(preString, buff, buffCnt); };

protected:
  /**
  Handle standard variable type tags */
  void handleVar(USmlTag * tag);
  /**
  Handle pose hist relay messages (from a 'posehistpush i=x cmd="posehist pose"') */
  void handlePoseHist(USmlTag * tag);
  /**
  Handle other data messages and try to add the result as variables */
  void handleOther(USmlTag * tag);

protected:
  /**
  Pointer to pose history plugin, if such is loaded */
  UResPoseHist * poseHist;
  /**
  Pointer to pose history plugin for UTM coordinates, if such is loaded */
  UResPoseHist * poseUtm;
  /**
  Pointer to pose history plugin for map coordinates, if such is loaded */
  UResPoseHist * poseMap;

private:
  /**
  Size of string with tag type names. */
  static const int MAX_TAG_LIST_SIZE = 100;
  /**
  Tags handled by this function */
  char tagList[MAX_TAG_LIST_SIZE];
  /** flag for informint possible display function on update */
  UVariable * varCallDispOnNewPose;

};

#endif
