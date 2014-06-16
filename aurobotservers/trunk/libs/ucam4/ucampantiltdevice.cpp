
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

#include <sys/ioctl.h>
#include <pwc-ioctl.h>

#include "ucampantiltdevice.h"

UCamPanTiltDevice::UCamPanTiltDevice(UCamDevBase * device)
  : UCamBase(device)
{
  pantiltValid = false;
  pantiltSupported = false;
}

//////////////////////////////////////////////////////////////

bool UCamPanTiltDevice::pantiltToHomePosition()
{ // public function
  bool result = false;
  if (pantiltSupported)
  {
    dev->lock();
    result = protPantiltReset(true, true);
    dev->unlock();
  }
  return result;
}

//////////////////////////////////////////////////////////////

bool UCamPanTiltDevice::protPantiltReset(bool resetPan, bool resetTilt)
{ // private (protected) function, (assumes camera is locked)
  //
  bool result = false;
#ifdef VIDIOCPWCMPTRESET
  int arg = 0; // reset both pan and tilt
  if (dev->isCameraOpen())
  {
    if (resetPan)
      arg = 0x01;
    if (resetTilt)
      arg |= 0x02;
    result = (ioctl(dev->getCamFd(),VIDIOCPWCMPTRESET, &arg) == 0);
  }
#endif
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamPanTiltDevice::protPantiltGetRange()
{ // protected (assumes camera is locked)
  //
  bool result = false;
#ifdef VIDIOCPWCMPTGRANGE
  struct pwc_mpt_range range;
  if (dev->isCameraOpen())
  {
    result = (ioctl(dev->getCamFd(),VIDIOCPWCMPTGRANGE, &range) == 0);
    if (result)
    {
      ptPanMin = range.pan_min;
      ptPanMax = range.pan_max;
      ptTiltMin = range.tilt_min;
      ptTiltMax = range.tilt_max;
    }
  }
#endif
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamPanTiltDevice::setPantiltStatus()
{ // should be possible without device lock
  bool isOK = false;
  //
  isOK = protPantiltGetRange();
  if (isOK)
    isOK = protPantiltGetPosition();
  pantiltSupported = isOK;
  pantiltValid = true;
  //
  return pantiltSupported;
}

///////////////////////////////////////////////////////

bool UCamPanTiltDevice::pantiltUpdatePosition()
{ // global function
  bool result = false;
  //
  if (pantiltSupported)
  {
    dev->lock();
    result = protPantiltGetPosition();
    dev->unlock();
  }
  return result;
}

///////////////////////////////////////////////////////

bool UCamPanTiltDevice::pantiltSetPosition(bool relative, int pan, int tilt)
{ // global function
  bool result = false;
#ifdef VIDIOCPWCMPTSANGLE
  int err = -1;
//struct pwc_mpt_angles
//{
  //  int absolute;    /* write-only */
  //  int pan;    /* degrees * 100 */
  //  int tilt;    /* degress * 100 */
  //  int zoom;    /* N/A, set to -1 */
//};
  struct pwc_mpt_angles moveTo;
  //
  if (dev->isCameraOpen() and pantiltSupported and pantiltValid)
  {
    dev->lock();
    if (relative)
    { // relative position offset, so no range-test
      moveTo.absolute = 0;
      moveTo.pan = pan;
      moveTo.tilt = tilt;
    }
    else
    {
      moveTo.absolute = 1;
      moveTo.pan = mini(ptPanMax, maxi(ptPanMin, pan));
      moveTo.tilt = mini(ptPanMax, maxi(ptPanMin, tilt));
    }
      //moveTo.zoom = -1;
    err = ioctl(dev->getCamFd(),VIDIOCPWCMPTSANGLE, &moveTo);
    result = (err == 0);
    dev->unlock();
  }
#endif
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamPanTiltDevice::protPantiltGetPosition()
{ // assumes camera is locked
  bool result = false;
#ifdef VIDIOCPWCMPTGANGLE
//struct pwc_mpt_angles
//{
  //  int absolute;/* write-only (true 1 or false 0) */
  //  int pan;     /* degrees * 100 */
  //  int tilt;    /* degress * 100 */
  //  int zoom;    /* N/A, set to -1 */
//};
  struct pwc_mpt_angles ptPos;
  ptPos.absolute = 1;
  ptPos.pan = 0;
  ptPos.tilt = 0;
  //ptPos.zoom = -1;
  if (dev->isCameraOpen())
  {
    result = (ioctl(dev->getCamFd(),VIDIOCPWCMPTGANGLE, &ptPos) == 0);
    if (result)
    {
      ptPanPos = ptPos.pan;
      ptTiltPos = ptPos.tilt;
    }
    else
      pantiltValid = false;
  }
#endif
  //
  return result;
}


