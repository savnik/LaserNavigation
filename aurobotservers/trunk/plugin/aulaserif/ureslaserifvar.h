/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
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
#ifndef URESLASERIFVAR_H
#define URESLASERIFVAR_H

#include <urob4/uresclientifvar.h>

/**
Handling of messages that should be converted int imple variables in the var pool

	@author Christian <chrand@mail.dk>
*/

// Uses the generic UIfVar defined in URob4

// class UResLaserIfVar : public UResIfVar
// {
// public:
//   UResLaserIfVar();
// 
//   ~UResLaserIfVar();
//   /**
//   Fixed name of this resource type */
//   static const char * getResID()
//   { return "laserVar"; };
//   /**
//   Fixed varsion number for this resource type.
//   Should follow release version, i.e. version 1.28 gives number 128.
//   Should be incremented only when there is change to this class
//   definition, i.e new or changed functions or variables. */
//   static int getResVersion()
//   { return 168; };
// 
// private:
//   /**
//   Create fixed variables for this client plugin resource */
//   void createBaseVar();
// 
// };

#endif
