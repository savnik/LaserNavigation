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
#ifndef UFUNCTIONPOSEHIST_H
#define UFUNCTIONPOSEHIST_H

#include "ufunctionbase.h"
#include "uresposehist.h"
/**
Interface function to pose history ressource.

@author Christian Andersen
*/
class UFunctionPoseHist : public UFunctionBase
{
public:
  /**
  Constructor */
  UFunctionPoseHist();
  /**
  Destructor */
  ~UFunctionPoseHist();
  /**
  Name and version number of function */
//  const char * name();
  /**
  Commands handled by this function */
//  const char * commandList();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
  Handle command */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  Set command and resource list based on alias name. */
  virtual void setAliasName(const char * name);
  /**
  Space separated list of resources provided by this function */
//  virtual const char * resourceList();
  /**
  Get a shared resource */
//  virtual UResBase * getResource(const char * resID);
  /**
  Set shared resource */
  virtual bool setResource(UResBase * ressource, bool remove);
  /**
  Function tests if all shared resources are loaded
  for full functionality */
//  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  
private:
  /**
  Reply to simple requests */
  bool handlePoseHistCommand(UServerInMsg * msg);
  /**
  Handle push commands for the pose hist structure */
  bool handlePoseHistPush(UServerInMsg * msg);
  /**
   * List poses */
  bool listPoses(int cnt);

protected:
  /**
  Pointer to pose history ressource */
  UResPoseHist * poseHist;
  /**
  Is pose history locally created. */
  bool poseHistLocal;

private:
  /**
  Command list */
  char aliasCommandList[2 * MAX_ID_LENGTH];
  /**
  name of push command */
  char aliasNamePush[MAX_ID_LENGTH];
  /**
  Name of resources */
  char aliasResList[MAX_RESOURCE_LIST_SIZE];
};

#endif
