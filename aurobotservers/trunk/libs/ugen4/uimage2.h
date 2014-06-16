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

#ifndef UIMAGE2_H
#define UIMAGE2_H

#include "uimage.h"

/**
Image based on UImage, but with maximum size of 800x600 (852x576)
- or other sizes with no more than 852 x 576 pixels -
in 24 bit color */
class UImage800 : public UImage
{
public:
  /**
  Constructor */
  UImage800()
  { // make oversized to fit default with GigE camera
    char * dImage = (char*)malloc(852 * 576 * 3);
    initImage(800, 600, dImage, 852 * 576 * 3,  3);
  };
  /**
  Destructor */
  virtual ~UImage800()
  {};
};

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

/**
Image based on UImage, but with maximum size of 640x480
- or other sizes with no more than 640 x 480 pixels -
in 24 bit color */
class UImage640 : public UImage
{
public:
  /**
  Constructor */
  UImage640()
  {
    char * dImage = (char*)malloc(480 * 640 * 3);
    initImage(480, 640, dImage, 480 * 640 * 3,  3);
  };
  /**
  Destructor */
  virtual ~UImage640()
  {};
};

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

/**
Image based on UImage, but with maximum size of 320x240
- or other sizes with no more than 320 x 240 pixels -
in 24 bit color */
class UImage320 : public UImage
{
public:
  /**
  Constructor */
  UImage320()
  {
    char * dImage = (char*)malloc(320 * 240 * 3);
    initImage(240, 320, dImage, 320 * 240 * 3,  3);
  };
  /**
  Destructor */
  virtual ~UImage320()
  {};
};

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

/**
Image based on UImage, but with maximum size of 160x120
- or other sizes with no more than 160 x 120 pixels -
in 24 bit color */
class UImage160 : public UImage
{
protected:
  /**
  Data buffer area for image */
  //char * dImage; //[160 * 120 * 3];
public:
  /**
  Constructor */
  UImage160()
  {
    char * dImage = (char*)malloc(160 * 120 * 3);
    initImage(160, 120, dImage, 160 * 120 * 3,  3);
  }
  /**
  Destructor */
  virtual ~UImage160()
  {};
};

#endif
