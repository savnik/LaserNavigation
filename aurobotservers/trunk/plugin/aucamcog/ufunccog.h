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
#ifndef UFUNC_COG_H
#define UFUNC_COG_H

#include <cstdlib>

#include <ucam4/ufunctioncambase.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


/** called when server makes a dlopen() (loads plugin into memory) */
void libraryOpen();

/** called when server makes a dlclose() (unloads plugin from memory) */
void libraryClose();

/**
Needed for correct loading and linking of library */
void __attribute__ ((constructor)) libraryOpen(void);
void __attribute__ ((destructor)) libraryClose(void);
/**
Allows server to create object(s) of this class */
extern "C" UFunctionBase * createFunc();
/**
... and to destroy the created object(s) */
extern "C" void deleteFunc(UFunctionBase* p);

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Test camera function to demonstrate handling of one function

COG
with an ROI (region of interest - x,y,w,h), a treshold.
The function allocates an image in image-pool, 
converts the raw image to an RGB image.
Tests all pixels in the requested region and
calculates the center of gravity of the pixels
that are above the specified treshold (in the red channel).

Bark
Just sends a reply to the client

@author Christian Andersen
*/
class UFuncCog : public UFunctionCamBase
{
public:
  /**
  Constructor */
  UFuncCog()
  { // command list and version text
    setCommand("COG bark", "centerOfGravity", "example plugin for camera server");
  }
  /**
  Destructor */
  virtual ~UFuncCog();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
  /**
  Function to handle the bark function */
  bool doBark(UServerInMsg * msg);
  /**
  Extract message parameters and call 'calculateCOG()' */
  bool centerOfGravity(UServerInMsg * msg, UImage * pushImg);
  /**
  Calculate the center of gravity (COG).
  The COG is calculated within a region of interest (ROI)
  for all pixel with red values above the specified threshold. */
  bool calculateCOG(UImage * rawImg,
		int roiX, int roiY, int roiW, int roiH,
		int threshold,
		double * lx, double * ly);
};


#endif
