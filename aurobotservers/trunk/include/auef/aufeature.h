/***************************************************************************
 *                                                                         *
 *   \file              aufeature.h                                        *
 *   \author            Lars Valdemar Mogensen                             *
 *   \date              Nov 2007                                           *
 *   \brief             Feature class implementation                       *
 *                                                                         *
 *                      Copyright (C) 2007 by DTU                          *
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

#ifndef AUFEATURE_H
#define AUFEATURE_H

#include <cstdlib>

#include <stdio.h>
#include <math.h>

/**
  \brief AUFeature class for the return of data features

 */
class AUFeature
{
  public:
  /**
    Constructor */
    AUFeature();
  /**
    Destructor */
    virtual ~AUFeature();
  /**
    Return name of this object type */
    virtual const char * getType();
  /**
  Print status in XML format */
    virtual void print();

};

#endif
