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
#include "ufakedevice.h"

////////////////////////////////////////////////////////

UFakeDevice::UFakeDevice()
 : ULaserDevice(), ULock()
{
  angleResolution = 1.0;
  modeAngleScan = 180;
  modeSimulated = false;
  setDeviceName("sick");
  strncpy(name, "fake", MAX_NAME_LNG);
  open = false;
  maxValidRange = maxRange();
}

////////////////////////////////////////////////////////

UFakeDevice::~UFakeDevice()
{
}

////////////////////////////////////////////////////////

bool UFakeDevice::getNewestData(ULaserData * dest,
                            unsigned long lastSerial,
                            int fake)
{
  bool result = false;
  //
  result = (dest != NULL);
  if (result)
  {
    maxValidRange = maxRange();
    if (not isPortOpen())
      openPort();
    if (fake > 0)
      // set new fake mode
      fakeMode = fake;
    fakeMap.distError = varPoseErr->getDouble(0);
    fakeMap.distOffset = varPoseErr->getDouble(1);
    fakeMap.headError = varPoseErr->getDouble(2);
    fakeMap.headOffset = varPoseErr->getDouble(3);
    fakeMap.rangeError = varRsd->getDouble(0);
    fakeMap.devicePose = getDevicePose();
    fakeMap.keepODOpose = varAddToPose->getBool(0);
    fakeMap.keepUTMpose = varAddToPose->getBool(1);
    getFakeScan( dest, lastSerial, fakeMode, varFakeDt->getDouble());
  }
  if (varAddToPoly->getBool(0))
  { // copy fake map to polygon plug-in
    if (not varAddToPoly->getBool(1))
    { // map needs to be added to polygon plug-in
      fakeMap.copyToPoly();
      // set map as created
      varAddToPoly->setInt(1, 1);
    }
  }
  else
    // remove set flag (if set)
    varAddToPoly->setInt(0, 1);
  return result;
}

/////////////////////////////////////////

bool UFakeDevice::changeMode(int scanangle, double resolution)
{
  bool result = false;
  //
  if (true)
  { // simulator has no options - not supported
    angleResolution = resolution;
    modeAngleScan = scanangle;
    result = true;
  }
  return result;
}

/////////////////////////////

bool UFakeDevice::openPort()
{
  open = true;
  fakeMap.reset();
  return open;
}

/////////////////////////////

void UFakeDevice::closePort()
{
  open = false;
}

const char * UFakeDevice::print(const char * preString, char * buff, int buffCnt)
{
  if (strcasecmp(devName, "rnd") == 0)
    snprintf(buff, buffCnt, "%s %s on %s w=%ddeg, "
        "res=%4.2fdeg, range=%.1fm (random mode)\n", preString,
        getName(), devName,
        modeAngleScan, angleResolution, maxRange());
  else
    snprintf(buff, buffCnt, "%s %s on %s w=%ddeg, "
        "res=%4.2fdeg, range=%.1fm, %.2fx, %.2fy, %.1fdeg\n", preString,
        getName(), devName,
        modeAngleScan, angleResolution, maxRange(),
        fakeMap.currentTruePose.x, fakeMap.currentTruePose.y, fakeMap.currentTruePose.h * 180.0 / M_PI);
  return buff;
}

///////////////////////////////////

double UFakeDevice::maxRange()
{
  return maxValidRange;
}

///////////////////////////////////

void UFakeDevice::setDeviceName(const char * device)
{
  // set name as usual
  ULaserDevice::setDeviceName(device);
  // adapt parameters
  if (strcasecmp(devName, "urg") == 0)
  {
    maxValidRange = 4.1; // max range
    modeAngleScan = 240;
    angleResolution = 360.0/1024.0;
  }
  else if (strcasecmp(devName, "sick") == 0)
  {
    maxValidRange = 8.1; // max range
    modeAngleScan = 180;
    angleResolution = 0.5;
  }
  else if (strcasecmp(devName, "lms100") == 0)
  {
    maxValidRange = 16.1; // max range
    modeAngleScan = 270;
    angleResolution = 0.5;
  }
  else if (strcasecmp(devName, "rnd") == 0)
  {
    maxValidRange = 8.1; // max range
    modeAngleScan = 180;
    angleResolution = 1.0;
  }
  else if (strcasecmp(devName, "360") == 0)
  {
    maxValidRange = 4.1;
    modeAngleScan = 360;
    angleResolution = 1.0;
  }
  // default fake mode
  fakeMode = 3;
}


void UFakeDevice::createBaseVars()
{
  ULaserDevice::createBaseVars();
  varRsd = vars->addVar("rangeSD", 0.0, "d", "(rw) range SD in m - added to value (linear SD^3)");
  varAddToPoly = vars->addVarA("makeMapPoly", "1 0", "d", "(rw) make copy of walls to polygon plugin (0:should, 1:done)");
  varAddToPose = vars->addVarA("updatePose", "1 0", "d", "(rw) update pose history, 0: odoPose (with noise), 1=utmPose (true pose)");
  varPoseErr = vars->addVarA("poseErr", "0 0 0 0", "d", "(rw) add noise to odo pose, 0=distErr/m, 1=distOfset/m, 2=headErr rad/m, 3=headOffset rad/m");
  varFakeDt = vars->addVar("fakeDt", 0.2, "d", "(rw) update time when updating fake pose");
}

