/***************************************************************************
 *   Copyright (C) 2006 by Christian   *
 *   chrand@mail.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UFUNCTIONSDLDISP_H
#define UFUNCTIONSDLDISP_H

#include <urob4/ufuncplugbase.h>

#include "uresdisp.h"

/**
Class to talk to the display of  initially laserdata

	@author Christian <chrand@mail.dk>
*/
class UFunctionDisp : public UFuncPlugBase
{
public:
  /**
  Constructor */
  UFunctionDisp()
  {
    setCommand("disp", "xdisplay", "data and image display using x and openCV display");
  };
  /**
  Destructor */
  virtual ~UFunctionDisp();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor.
   * \return true if any resources are created. */
  virtual void createResources();
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
    Reference to camera interface connection */
    UResDisp * disp;

};

#endif
