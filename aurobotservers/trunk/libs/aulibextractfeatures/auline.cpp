/***************************************************************************
 *                                                                         *
 *   \file              auline.cpp                                         *
 *   \author            Lars Valdemar Mogensen                             *
 *   \date              Nov 2007                                           *
 *   \brief             Line class implementation                          *
 *                                                                         *
 *                      Copyright (C) 2006 by DTU                          *
 *                      rse@oersted.dtu.dk                                 *
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

using namespace std;

#include "auline.h"

///////////////////////////////////////////

AULine::AULine() 
{
  clear();
}

///////////////////////////////////////////

AULine::~AULine() {}

///////////////////////////////////////////

const char * AULine::getType()
{ 
  return "line";
}

///////////////////////////////////////////

void AULine::print()
{ 
  fprintf(stdout,"<line x1=%.3f y1=%.3f x2=%.3f y2=%.3f />\n",x1,y1,x2,y2);
}

void AULine::clear()
{
  polarValid=false;
    // Polar representation
  start_th=0;
  end_th=0;
  r0=0;
  th0=0;
    
  cartValid=false;
    // Cartesian representation
  x1=0;y1=0;
  x2=0;y2=0;
    
    // Common data
  length=0;
  slope=0;
  msq=0;
  points=0;
}
