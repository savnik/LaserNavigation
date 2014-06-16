/***************************************************************************
 *   Copyright (C) 2008 by DTU (Christian Andersen)                        *
 *   rse@elektro.dtu.dk                                                    *
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
#ifndef UFUNC_OBJ3D_H
#define UFUNC_OBJ3D_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>

#include "uresobj3d.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class UResPoseHist;

/**
* This plugin (or module) converts a 3D point cloud to 3D polygons that can be obstacles or free navigateable ground area.
@author Christian Andersen
*/
class UFuncObj3d : public UFuncPlugBase
{
public:
  /**
  Constructor */
  UFuncObj3d();
  /**
  Destructor */
  virtual ~UFuncObj3d();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class)
   * Resource shut-down code should be handled in the resource destructor. */
  virtual void createResources();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  Send groups of obstacles to client.
  May send newly updated obstacles only when 'updatesOnly' is true. */
  bool sendObjects(UServerInMsg * msg, int maxCnt, bool updateOnly);
  /**
   * Handle object cloud processing commands
   * \param msg message and client information
   * \param pushedCloud is an optional point cloud to be used.
   * \param getAny could ant 3D cloud do, og only new unprocessed clouds.
   * \param makeOnly do not send result to client, just do the processing..
   * \param doObstAll should cloud be obstacle processed for all obstacles above ground
   * \param doObstHuman should only human sized obstacles be processed
   * \param doGround should ground plane be estimated
   * \param doGndEdgeObst should ground polygon be converted into edge obstacles
   * \param maxCnt is the maximum items (obstacle groups) that is to be send to client
   * \param updatesOnly should updates to ubstacles be send only
   * \return true if any reply is send to the client (OK or not) */
  bool handleCloud(UServerInMsg * msg, UDataBase * pushedCloud, bool getAny, bool makeOnly,
                   bool doObstAll, bool doObstHuman, bool doGround,
                   bool doGndEdgeObst,
                   int maxCnt, bool updatesOnly);

  
protected:
  /**
  pointer to resource to do the job */
  UResObj3d * obj3d;
public:
  /**
   * is function to be silent */
  bool silent;
};


#endif

