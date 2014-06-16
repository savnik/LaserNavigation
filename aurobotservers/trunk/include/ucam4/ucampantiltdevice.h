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
#ifndef UCAMPANTILTDEVICE_H
#define UCAMPANTILTDEVICE_H

#include "ucambase.h"

/**
Code to support pan-tilt

Especially on PWC logitech sphere device 

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UCamPanTiltDevice : public UCamBase
{
  public:
  /**
    Constructor */
    UCamPanTiltDevice(UCamDevBase * device);

  /**
    Get pan tilt information valid.
    The flag is only valid if pan-tilt is supported
    NB! the flag is set when camera is opened */
    inline bool isPantiltValid() {return pantiltValid;};
  /**
    Get pan supported flag for this camera.
    NB! the flag is set when camera is opened */ 
    inline bool isPantiltSupported() {return pantiltSupported;};
  /**
    Get pantilt panorate (left-right) min range.
    in degrees * 100 - positive is left.
    NB! only valid if isPantiltValid and isPantiltSupport flags are true.  */
    inline int getPanMinRange() {return ptPanMin;};
  /**
    Get pantilt panorate (left-right) max range.
    in degrees * 100.
    NB! only valid if isPantiltValid and isPantiltSupport flags are true.  */
    inline int getPanMaxRange() {return ptPanMax;};
  /**
    Get pantilt tilt (up-down) min range.
    in degrees * 100 - positive is up.
    NB! only valid if isPantiltValid and isPantiltSupport flags are true.  */
    inline int getTiltMinRange() {return ptTiltMin;};
  /**
    Get pantilt tilt (up-down) max range.
    in degrees * 100 - positive is up.
    NB! only valid if isPantiltValid and isPantiltSupport flags are true.  */
    inline int getTiltMaxRange() {return ptTiltMax;};
  /**
    Get pantilt panorate (left-right) position
    in degrees * 100 - positive is left.
    NB! only valid if isPantiltValid and isPantiltSupport flags are true.  */
    inline int getPanPos() {return ptPanPos;};
  /**
  Get pantilt tilt (up-down) position
  in degrees * 100 - positive is up.
  NB! only valid if isPantiltValid and isPantiltSupport flags are true.  */
  inline int getTiltPos() {return ptTiltPos;};
  /**
  Global function to reset pan-tilt function to home position */
  bool pantiltToHomePosition();
  /**
  Set pan and tilt position (must be within range).
  Returns true if call succeded */
  bool pantiltSetPosition(bool relative, int pan, int tilt);
  /**
  Global function to update the camera position from the camera.
  Returns true if position is updated.
  In not, then pan-tilt is most likely not supported. */
  bool pantiltUpdatePosition();
  /**
  Test if pan-tilt is supported.
  and set range and position if so.
  Returns true if supported.  */
  bool setPantiltStatus();

  protected:
  /**
    Reset pan-tilt to level front position.
    Returns true if function was accepted.
    Returns false if no such function (PanTilt not supported) */
    bool protPantiltReset(bool resetPan, bool resetTilt);
  /**
    Get minimum and maximum pan-tilt ranges
    Returns true if successful,
    Returns false if failed or not supported */
    bool protPantiltGetRange();
  /**
    Get current position of camera pan-tilt */
    bool protPantiltGetPosition();

  private:
  // Pan tilt
    int ptPanMin, ptPanMax;   // possible range (deg * 100)
    int ptTiltMin, ptTiltMax; // pan tilt range (deg * 100)
    int ptPanPos, ptTiltPos;  // pan tilt position (deg * 100)
    bool pantiltValid;        // is range and position valid (if supported)
    bool pantiltSupported;    // is pan-tilt supported at all
};

#endif
