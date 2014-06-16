/***************************************************************************
 *                                                                         *
 *   \file              aucircle.cpp                                       *
 *   \author            Lars Valdemar Mogensen                             *
 *   \date              Nov 2007                                           *
 *   \brief             Circle class implementation                        *
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

#include "aucircle.h"

///////////////////////////////////////////

AUCircle::AUCircle()
{
  clear();
}

///////////////////////////////////////////

AUCircle::~AUCircle() {}

///////////////////////////////////////////

const char * AUCircle::getType()
{
  return "circle";
}

///////////////////////////////////////////

void AUCircle::print()
{
  fprintf(stdout,"<circle cx=%.3f cy=%.3f r=%.3f />\n",cx,cy,r);
}

void AUCircle::clear()
{
  polarValid=false;
  // Polar representation
  cr=0;
  cth=0;

  cartValid=false;
  // Cartesian representation
  cx=0;
  cy=0;

  // Common data
  r=0;
  msq=0;
  coverage=0;
  points=0;
}
