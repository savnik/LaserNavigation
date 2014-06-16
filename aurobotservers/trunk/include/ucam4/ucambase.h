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
#ifndef UCAMBASE_H
#define UCAMBASE_H


#include "ucamdevbase.h"

/**
This class holds the camera device pointer and possibly related cameras that are to be synchronized with this camera.

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UCamBase{
public:
  /**
  Constructor - and set the device driver */
  UCamBase(UCamDevBase * device);
  /**
  Destructor */
  virtual ~UCamBase();
  /**
  Get pointer to camera device */
  UCamDevBase * getDev()
  { return dev; };
  /**
  Get device number (as known by cam pool) for camera*/
  int getDevNum()
  { return dev->getDeviceNumber(); };
  /**
  Set the parameters derived from resolution, i.e.
  conversion matrix, radial error valeues etc.
  but at this level just the resolution factor. */
  virtual void imageSizeChanged(double iResFactor)
  {};
  /**
  Got this new image.

  NB! This function is called by the image read thread and should not do anything but
  copy the image to another buffer as needed, and no other major processing. */
  virtual void gotNewImage(UImage * raw)
  { printf("UCamBase new image arrived - no handler found\n"); };
  /**
  Request to test push commands for need of new data.
  If new data is needed then 'gotNewImage()' should be called.
  \returns true if data is needed. */
  inline virtual bool needNewPushData()
  { return false; };
  /**
  Image is updated now, note this in client handler */
  virtual void imgUpdated()
  {
    printf("UCamBase::imgUpdated: Should be handled at camPush level\n");
  };
  /**
  Set pointer to established var-pool structure for locally maintained variables */
  void setVarPool(UVarPool * vpd)
  {
    vars = vpd;
    getDev()->setVarPool(vpd);
    createVars();
  };
  /**
  Create locally maintained variables - if any */
  virtual void createVars() {};
  /**
   * Set camera type name */
  void setTypeName(const char * newName);
  /**
   * Set camera type name */
  const char * getCamName()
  {
    const char * typeName = NULL;
    if (dev != NULL)
      typeName = dev->getCameraName();
    return typeName;
  };


protected:
  /**
  The device handler pointer */
  UCamDevBase * dev;
  /**
  Maximum number of related cameras to this device - mainly for stereo processing */
  //static const int MAX_RELATED_CAM_DEVICES = 2;
  /**
  Related devices */
  //UCamDevBase * related[MAX_RELATED_CAM_DEVICES];
  /**
  Number of available related cams */
  //int relatedCnt;
  /**
  Var pool structure for the globally available variables for this camera - i.e. position focallength etc.*/
  UVarPool * vars;
};

#endif
