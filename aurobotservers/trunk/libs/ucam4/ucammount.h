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
#ifndef UCAMMOUNT_H
#define UCAMMOUNT_H

#include <ugen4/u3d.h>
#include <ugen4/uposrot.h>
#include <urob4/uvarpool.h>

#include "ucamrad.h"

// moved to ucommon.h
//#define MAX_MOUNT_NAME_SIZE 20

/**
Holds information of a camera mounted on a robot.
This includes 3D position and rotation, and
rotation matrices to ease conversion between
camera and robot perspective.

@author Christian Andersen
*/
class UCamMounted : public UCamRad
{
public:
  /**
  Constructor */
  UCamMounted(UCamDevBase * device);
  /**
  Destructor */
  virtual ~UCamMounted();

  /**
  Take new image snapshot.
  Returns pointer to image if snapshot succeded.
  Returns NULL if failed */
  //bool getNewSnapshotImage(bool remRadErr, bool justOne);
  /**
  Get snapshot image and remove radial error
  if it is default to do so. */
/*  inline bool getNewSnapshotImage(bool justOne)
      { return getNewSnapshotImage(getDefaultRemRadErr(), justOne); };*/
  /**
  Get pointer to last captured image.
  Returns pointer to image, but image may be invalid. */
/*  inline URawImage * getImage()
      { return &imgRaw; };*/
  /**
  Set the camera position relative to the robot. It is assumed
  that the robot position is at floor level and oriented with forward
  beeing X, right is X and up is Z.
  Rotation around x is Omega, arounf Y is Phi and around Z is Kappa,
  positive is as for right hand coordinate systems.
  Pan-tilt is no longer supported */
  void setPosOnRobot(UPosition * pos, URotation * rot);
  /**
   * Set camera position from 3D pose structure */
  void setPosOnRobot(UPosRot * pose3d);
  /**
  Set the camera position relative to the robot from camera
  coordinates (x is right, y is up and z is back). */
  inline void setPosOnRobotCC(UPosition * pos, URotation * rot)
  {
    UPosRot pr;
    pr.set(-pos->z, -pos->x, pos->y, -rot->Kappa, -rot->Omega, rot->Phi);
    varCamPose->set6D(&pr);
  }
  /**
  Get camera position relative to robot.
  Coordinates are in 'robot' coordinates i.e. x is front, y left, z is up*/
  inline UPosition getPos()
  { 
    UPosRot p = varCamPose->get6D();
    return p.pos();
  };
  /**
  Get camera position relative to robot in camera coordinate system, i.e.
  x is right, y is up and z is back.
  Coordinates are in 'robot' coordinates i.e. x is front, y left, z is up*/
  inline UPosition getPosCC()
  {
    UPosRot pr = varCamPose->get6D();
    UPosition result(-pr.y, pr.z, -pr.x);
    return result;
  };
  /**
  Get camera rotation relative to robot in camera coordinates.
  i.e. x-axis is left (omega), y-axis is up (Phi), z-axis is back (kappa).
  rotation is first Phi around vertical axis (Y), then
  elevation up-down around X axis,
  and last rotation around camera axis Z.
  If pan-tilt is supported, then the pan-tilted rotation is
  returned. (NB! if just moved, it may take some time for the camera
  to get to the returned position). */
  URotation getRotCC();
  /**
  Get camera rotation relative to robot in robot coordinates.
  i.e. x-axis is front (omega), y-axis is right (Phi), z-axis is up (kappa).
  Rotation order is first around vertical axis (Z (kappa)), then
  elevation up-down around Y-axis (Phi),
  and last rotation around robot axis (x or omega).
  If pan-tilt is supported, then the pan-tilted rotation is
  returned. (NB! if just moved, it may take some time for the camera
  to get to the returned position). */
  URotation getRot();
  /**
  Get camera position as URotPos object - in robot coordinate type.
  If pan-tilt is supported, then the pan-tilted rotation is
  returned. (NB! if just moved, it may take some time for the camera
  to get to the returned position). */
  inline UPosRot getPosRot()
  {
    UPosRot result;
    result.set(getPos(), getRot());
    return result;
  }
  /**
  Save camera settings using this key name.
  if no key is provided the camera name is used.
  Returns true if saved. */
  bool saveCamSettings(Uconfig * ini, const char * mountkey);
  /**
  Load camera settings using this key name.
  if no key is provided the camera name is used.
  Returns true if loaded. */
  bool loadCamSettings(Uconfig * ini, const char * mountkey);
  /**
  Set the mount name (position indicator) for this camera */
  inline void setMountName(const char * posName)
  { 
    if (varMountName != NULL)
      varMountName->setValues(posName, 0, true);
  };
  /**
  Get the position name for this camera */
  /* bad name
  inline char * getMountName()
    { return mountName; };
  */
  /**
  Get the position name for this camera */
  inline const char * getPosName()
    { return varMountName->getValues(); };
  /**
  Is this client in charge of camera settings?
  Returns true if so, if not then the client will be in
  charge if noone else has requested to be in charge. */
  bool isClientInCharge(int client);
  /**
  Set this client in charge */
  inline void setClientInCharge(int client)
    { clientInControl = client; };
  /**
  Convert this 3D robot coordinate to a pixel position.
  The mounted camera position is used to convert to
  camera coordinate in 3D, and then to
  pixel position in a undistorted image.
  if 'useRad' is true, then the pixel position is converted to a
  pixel position in the distorted image. */
  bool getMtoPix(UPosition pos3D, bool useRad, float * pixx, float * pixy);
  /**
  Set logfile pointer. if pointer objext is NULL, then
  no logging is performed. */
  inline void setImageLogging(UImageLog * value)
  { logip = value; };

//protected:
  /**
  Note that this image is captured in logfile as needed */
  virtual bool logImage(UImage * img);
  /**
  Create locally maintained variables - if any */
  virtual void createVars();
  
protected:
  /**  Rotation of camera relative to robot */
//   URotation relRot;           // relative rotation (relative to robot)
//   /**  Position of camera relative to robot */
//   UPosition relPos;           // relative position (relative to robot)
  /**  Inner orientation (3x3) pixel image to (top-left) pixel */
//   UMatrix4 mP;
//   /**  inner image plane to pixel image */
//   UMatrix4 mb;
//   /**  (3x3) Image to pixel mb * mP */
//   UMatrix4 mItoP;
//   /**  (3x3) Pixel to image (mb * mP).inverse */
//   UMatrix4 mPtoI;
  /** Client in control of camera */
  int clientInControl;
  /** Last snapshot taken */
  //URawImage imgRaw;
  /**
  Reference to pool of images */
//  UImagePool * imgPool;
  /**
  Camera position name */
  //char mountName[MAX_MOUNT_NAME_SIZE];
  /**
  Logfile for saved images (pointer to), the file itself is
  maintained by the camera pool */
  UImageLog * logip;
  /** camera pose on robot */
  UVariable * varCamPose;
  /** mount name for camera */
  UVariable * varMountName;
};

#endif
