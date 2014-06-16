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
#ifndef URESROBOT_H
#define URESROBOT_H

#include <ugen4/u3d.h>
#include <ugen4/uposrot.h>

#define ROBOTWIDTH 0.85

/**
Class that holds static robot data as sensor and size configuration */
class UResRobot
{
  public:
  /**
  Constructor */
  UResRobot();
  /**
  Constructor */
  ~UResRobot();
  /**
  Get pointer to position of laser device */
  UPosRot * getLaserPos(int index = 0)
  { return &laser; };
  
protected:
  /**
  Position of laser scanner */
  UPosRot laser;
};

#endif
