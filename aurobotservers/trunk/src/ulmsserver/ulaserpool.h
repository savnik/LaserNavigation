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
#ifndef ULASERPOOL_H
#define ULASERPOOL_H

#include <urob4/uresbase.h>
#include <ugen4/uposrot.h>
#include <ugen4/upolygon.h>

#include "ulaserdevice.h"

/**
Maximum number of laser devices (including fake and replay devices) */
#define MAX_LASER_DEVS 16

/**
Pool of laserscanner devices

@author Christian Andersen
*/
class ULaserPool : public UResVarPool
{
public:
  /**
  Constructor */
  ULaserPool()
  { // ressource name and version number
    setResID(getResClassID(), 200);
        // set description for global variables owned by this resource (optional)
    setDescription("Laser device pool", false);
    // create global variables
    createBaseVar();
    ULaserPoolInit();
  };
  /**
  Destructor */
  ~ULaserPool();
  /**
   * Initialize laser pool */
  void ULaserPoolInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "lasPool"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 194; };*/
  /**
  Set ressources as needed */
  bool setResource(UResBase * ress, bool remove);
  /**
   * Stop any running subthread - we are going down */
  virtual void stop(bool andWait);

  /**
  Get a device pointer */
  ULaserDevice * getDevice(int deviceNumber);
  /**
  Get a device pointer from specification string */
  ULaserDevice * getDevice(const char * deviceID);
  /**
  Get a device pointer */
  inline ULaserDevice * getDefDevice()
  {  return getDevice(var.def->getInt()); };
  /**
  Get device number from a device pointer.
  Returns -1 if device is not found. */
  int getDeviceNumber(ULaserDevice * device);
  /**
  Get number of created devices */
  inline int getDeviceCnt()
  { return lasDevsCnt; };
  /**
  Add device */
  bool addDevice(ULaserDevice * newDevice);
  /**
  * Set default device.
  * If value is not inside valid range, then the closest valid device is selected. */
  void setDefDevice(int value);
  /**
  * Set default device to the lase device of the type (name) of 'device'.
  * Returns true if such a name match is found */
  bool setDefDevice(const char * device);
  /**
  Get defauylt device number */
  inline int getDefDeviceNumber()
  {  return var.def->getInt();};
  /**
  Set command server to be used by push commands.
  NB! devices must be loaded before this call.
  (or the call repeated). */
  void setCmdExe(UCmdExe * server); // for push commands
  /**
  Print laser pool status to a string buffer
   * \returns pointer to provided buffer. */
  const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Is this resource missing any ressource pointers to function.
  Returns true if all is OK.
  If some is missing, then the missing resource name should be added to
  the missingThese list, */
  bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Get pose (3D) of the laser scanner sensor */
  UPosRot * getDevicePose(int idx);
  /**
  Get maximum number of laser devices allowed in laser pool (a hard coded constant) */
  inline int getDeviceMax()
  { return MAX_LASER_DEVS; };
  /**
   * \brief get the next scan from laser scanner device
   * \param device if -1, then scan is from default device, else from the specified device
   * \param las returns a pointer to the laser scanner device used
   * \param dataBuff a data buffer that may be used for the laserscan if a buffer is not allocated already. The buffer is also used to hold information on last used scan (serial number and device)
   * \param pushData may hold a scan already, if so, then only the corresponding laser device is fetched (lo las).
   * \param fake if > 0 then a fake scan is fetched. 1=random range, 2,3,4 is measurements in a fake building with 4 rooms, with the robot moving through these rooms.
   * \returns a pointer to the new scan - or NULL if no scan or if neither dataBuff nor pushData is available. */
  ULaserData * getScan(const int device,
                        ULaserDevice ** las,
                        ULaserData * dataBuff,
                        ULaserData * pushData,
                        const int fake);
  /**
   * \brief get the next scan from laser scanner device
   * \param device the device name or number as string, e.g. v360 or sick. "-1" or NULL or "" for default scanner.
   * \param las returns a pointer to the laser scanner device used
   * \param dataBuff a data buffer that may be used for the laserscan if a buffer is not allocated already. The buffer is also used to hold information on last used scan (serial number and device)
   * \param pushData may hold a scan already, if so, then only the corresponding laser device is fetched (lo las).
   * \param fake if > 0 then a fake scan is fetched. 1=random range, 2,3,4 is measurements in a fake building with 4 rooms, with the robot moving through these rooms.
   * \returns a pointer to the new scan - or NULL if no scan or if neither dataBuff nor pushData is available. */
  ULaserData * getScan(const char * device,
                        ULaserDevice ** las,
                        ULaserData * dataBuff,
                        ULaserData * pushData,
                        const int fake);
  /**
  Advance replay for this resources - if in replay mode - until just before this time
   * \param untilTime advance with steps in logfile until this time is reached
   * \returns true if advanced */
  virtual bool replayToTime(UTime untilTime);
  /**
  Advance replay for all devices in replay mode until just before this time
  The function sets a flag and the steps are performed in main command thread
  as soon as possible (before next user queue, and before push events) */
  virtual void replayAdvanceTime(UTime untilTime);

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();

protected:
  /**
  Array of pointers to laser devices (already created */
  ULaserDevice * lasDevs[MAX_LASER_DEVS];
  /**
  Count of available devices */
  int lasDevsCnt;
  /**
  Default device */
//  int defaultDevice;
  /**
  Pointer to server core */
  UCmdExe * cmdExe;
  /// variable pointers
  struct
  {
    UVariable * deviceCnt;
    UVariable * devices;
    UVariable * def;
  } var;
};

#endif
