/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                             *
 *   jca@oersted.dtu.dk                                                    *
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
#ifndef UFUNC_NEARGET_H
#define UFUNC_NEARGET_H

#include <cstdlib>

#include <ulms4/ufunclaserbase.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/
class UFuncNear : public UFuncLaserBase
{
public:
  /**
  Constructor */
  UFuncNear()
  { // set the command (or commands) handled by this plugin
    setCommand("near nearget", "laserNearGet", "reports simple positions from laserscan");
  }
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
private:
  /**
  Small support function to make a line in the poly plugin (that can be displayed in client) .
  \param lineName is the name of the line in the poly - plugin.
  \param x1,y1 is the coordinate of the first point (in odometry coordinates)
  \param x2,y2 is the coordinate of the other end. */
  void sendToPolyPlugin(const char * lineName, double x1, double y1, double x2, double y2);
    
};



#endif

