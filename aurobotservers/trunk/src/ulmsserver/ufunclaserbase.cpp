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

#include <ulms4/ulaserpool.h>
#include <ulms4/uresv360.h>

#include "ufunclaserbase.h"

UFuncLaserBase::UFuncLaserBase()
{
  lastDevice = -1;
  lastSerial = 0;
}

//////////////////////////////////

UFuncLaserBase::~UFuncLaserBase()
{
}

//////////////////////////////////
// UServerInMsg * msg, ULaserData * pushData
ULaserData * UFuncLaserBase::getScan(UServerInMsg * msg,
                                     ULaserData * pushData, bool any, ULaserDevice ** dev)
{
  ULaserPool * lasPool;
  ULaserData * scan = pushData;
  int device = -1;
  int fake = 0;
  const int MVL = 30;
  char value[MVL];
  ULaserDevice * laserDev = NULL;
  UResV360 * v360 = NULL; 
  //
  lasPool = (ULaserPool *)getStaticResource("lasPool", false);
  if (scan == NULL)
  {
    scan = &dataBuff;
    if (msg->tag.getAttValue("device", value, MVL))
    {
      if (strcasecmp(value, "V360") == 0)
      { // request of data from virtual laser device
        v360 = (UResV360 *)getStaticResource("v360", false, false);
      }
      else
      {
        device = strtol(value, NULL, 0); // device
      }
    }
    if (msg->tag.getAttValue("fake", value, MVL))
      fake = strtol(value, NULL, 10);
    if (msg->tag.getAttValue("any", value, MVL))
      any = true;
    // get pouinter to laser pool (but do not create if not available - should be)
    if (lasPool != NULL)
    { // laser pool is available
      lasPool->getScan(device, &laserDev, NULL, pushData, fake);
      if (laserDev != NULL)
      { // get (or use) scan
        if (laserDev->getDeviceNum() != lastDevice)
        {
          lastDevice = laserDev->getDeviceNum();
          lastSerial = 0;
        }
        else if (any and lastSerial > 0)
          lastSerial = 0;
        // get new scandata from device
        // get data
        if (v360 != NULL)
        { // a virtual scan
          v360->update(NULL, NULL, fake);
          v360->getNewestData(scan, lastSerial, fake);
        }
        else
          laserDev->getNewestData(scan, lastSerial, fake);
        if (dev != NULL)
          *dev = laserDev;
      }
    }
  }
  else if (dev != NULL and lasPool != NULL)
  { // need laser device for svan
    *dev = lasPool->getDevice(scan->getDeviceNum());
  }
  return scan;
}

/////////////////////////////////////////

ULaserDevice * UFuncLaserBase::getDevice(UServerInMsg * msg,
                                     ULaserData * pushData)
{
  ULaserPool * lasPool;
  int device = -1;
  //int fake = 0;
  ULaserDevice * laserDev = NULL;
  const int MVL = 30;
  char value[MVL];
  //
  if (msg->tag.getAttValue("device", value, MVL))
    device = strtol(value, NULL, 0);
//   if (msg->tag.getAttValue("fake", value, MVL))
//     fake = strtol(value, NULL, 10);
  // get pouinter to laser pool (but do not create if not available - should be)
  lasPool = (ULaserPool *)getStaticResource("lasPool", false);
  if (lasPool != NULL)
  { // laser pool is available
    // get laser scanner device only
    lasPool->getScan(device, &laserDev, NULL, pushData, 0);
  }
  return laserDev;
}

