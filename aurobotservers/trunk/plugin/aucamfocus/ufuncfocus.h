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
#ifndef UFUNC_FOCUS_H
#define UFUNC_FOCUS_H

#include <cstdlib>

#include <ucam4/ufunctioncambase.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Allows server to create object(s) of this class */
extern "C" UFunctionBase * createFunc();

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
class UFuncFocus : public UFunctionCamBase
{
public:
  /**
  Constructor */
  UFuncFocus()
  { // handles command keyword focus
    setCommand("focus", "focusLine", "shows intensity curve across image for focus adjustment");
    // initialization of variables in class - as needed
    fmaxOld = 500;
    fvalOld = 500;
  };
  /**
  Destructor */
  virtual ~UFuncFocus();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
  /**
  Extract message parameters and call 'calculateCOG()' */
  bool doFocus(UServerInMsg * msg, UImage * pushImg);
  /**
  Calculate a focus value as the sum of all pixel to pixel
  intensity differences - vertical and horizontal. */
  long calculateFocusValue(
                    UImage * rawImg, // YUV raw image
                    int roiX, int roiY, int roiW, int roiH);
  /**
  Previous focus value */
  long fvalOld;
  /**
  Upper limit of old focal value */
  long fmaxOld;
};


#endif
