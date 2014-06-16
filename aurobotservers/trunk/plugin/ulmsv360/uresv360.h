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
#ifndef URESV360_H
#define URESV360_H

#include <urob4/uresvarpool.h>
#include <urob4/userverpush.h>
#include <umap4/upose.h>
#include <ulms4/ulaserpool.h>
#include "uv360scan.h"

class UResPoseHist;

/**
Ressource for virtual laserscanner with 360 deg coverage

@author Christian Andersen
*/
class UResV360 : public UResVarPool, public UServerPush
{
public:
  /**
  Constructor */
  UResV360()
  { // set name and version number
    setResID(getResClassID(), 200);
    // user defined initialization
    lasPool = NULL;
    poseHist = NULL;
    v360 = NULL;
    fakeTargetState = 0;
    defDevice = -1;
    lasDev = NULL;
    createBaseVar();
  };
  /**
  Destructor */
  ~UResV360();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "v360"; };
  /**
  Checks if needed ressources are available */
  bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set Resource is called by the core on change of ressource
  configuration (added or removed).
  The function should return true if ressource is used (the link is used) */
  bool setResource(UResBase * resource, bool remove);
  /**
  Get newest virtual scan with a source serial number grater than 'last'
   * \param laserData is where the result is to be placed.
   * \param last is the scannumber of the last scan, newest scannumber has to be bigger that this,
   * \param fake if > 0 then scan is generated from fake source.
   * \returns true if a v360 scan is available */
  bool getNewestData(ULaserData * laserData, int last, int fake);
  /**
  Print status to string preceding the string with this 'preString'.
  The target string is 'buff' with a maximum length of 'buffCnt'. */
  const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Get current setting of laser scanner position on robot */
  UPose getLaserPose();
  /**
  Set current setting of laser scanner position on robot */
  void setLaserPose(UPose newLaserPose);
  /**
  Get number of sectors each scan */
  inline int getSectorCnt()
  {
    if (v360 != NULL)
      return v360->getSectorCnt();
    else
      return 0;
  }
  /**
  Get number of stored measurements in a scan */
  inline int getMeasurementCnt()
  {
    int n = 0;
    if (v360 != NULL)
    {
      lock();
      n = v360->getSectorCnt();
      unlock();
    }
    return n;
  };
  /**
   * Update the virtual laserscan with this laserdata, or get a new from this device
   * \param laserData is a pointer to scandata - NULL if new data is to be obtained
   * \param laserDevice is a pointer to the laserdevice, from where to obtain data,
   * NULL will use default device (may not be V360).
   * \param fake is a fake number - if > 0, then this is used to obtain data.
   * \returns true if updated */
  bool update(ULaserData * laserData, ULaserDevice * laserDevice, int fake);
  /**
   * GEt the default device number for laser scanner source */
  inline int getDefDevice()
  { return defDevice; };
  /**
  * Get the last used device for laser scanner source */
  inline ULaserDevice * getLastDevice()
  { return lasDev; };
  /**
   * Set the default device number for laser scanner source */
  int setDefDevice(const char * devString);
  /**
  Called by cmdExe to get push object
  followed by a call to 'gotNewData(object)'.
  Should be overwritten by push object holder. */
  virtual void callGotNewDataWithObject();

protected:
  /**
  Get fresh source from default device.
  The data serial number must be newer than 'last' to
  avoid reuse of same dataset.
  Data is returned into the user provided buffer 'laserData'.
  The function returns true if data is in buffer (and is valid). */
  bool getSourceData(ULaserData * laserData, int last, int fake);
protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();


protected:
  /**
  Pointer to pose history ressource */
  UResPoseHist * poseHist;
  /**
  Pointer to laser scanner pool */
  ULaserPool * lasPool;
  /**
  Virtual scan */
  UV360Scan * v360;
  /**
  Laser measurement data buffer */
//  ULaserData data;
  /**
  Laser source laser measurement data buffer */
  ULaserData sourceData;
  /**
   * data buffer used during a push call */
  ULaserData pushData;
  /**
  Fake pose target state */
  int fakeTargetState;
  /**
   * Default laserscanner source device */
  int defDevice;
  /** last used laser device */
  ULaserDevice * lasDev;
  /// pointers to local variables in the global variables database
  struct {
    /// area where self-detect should be suppressed.
    UVariable * notValidArea;
    /// last scan number used to update
    UVariable * lastSerial;
    /// number of updates of virtual scanner
    UVariable * updateCnt;
    /// timestamp of last update
    UVariable * updateTime;
    /// virtual scan resolution (in degrees)
    UVariable * resolution;
  } var;
};

#endif
