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
#ifndef UPOSROT_H
#define UPOSROT_H

#include "u3d.h"

class UPlane;

/**
A combination of position and orientation in 3D.
This could describe an oriented coordinate system, where the position is that translation and the rotation is the rotation relative to a reference system.

@author Christian Andersen
*/
class UPosRot : public UPosition, public URotation
{
public:
  /**
  Constructor */
  UPosRot();
  /**
  Destructor */
  ~UPosRot();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "posrot";  };
  /**
  Clear all values */
  inline void clear()
  {
    UPosition::clear();
    URotation::clear();
  };
  /**
  Set from position and rotation */
  inline void set(UPosition pos, URotation rot)
  {
    UPosition::setPos(pos);
    URotation::setRot(rot);
  };
  /**
  Set from position part only */
  inline void setPos(UPosition pos)
  { UPosition::setPos(pos); };
  /**
  Set from rotation part only */
  inline void setRot(URotation rot)
  { URotation::setRot(rot); };
  /**
  Set directly from 6 values */
  inline void set(double x, double y, double z,
                  double Omega, double Phi, double Kappa)
  {
    UPosition::set(x, y, z);
    URotation::set(Omega, Phi, Kappa);
  };
  /**
  Set from pose */
  inline void setFromPose(double x, double y, double h)
  {
    UPosition::set(x, y, 0.0);
    URotation::set(0.0, 0.0, h);
  };
  /**
  Get conversion matrix, that can convert a 3D position
  from a local coordinate system to the global
  coordinate system, when this posRot is the position of
  the local coordinate system in the global.
  Returns a UMatrix4 value */
  inline UMatrix4 getRtoMMatrix()
  {  return URotation::asMatrix4x4RtoM(getPos()); };
  /**
  Get conversion matrix, that can convert a 3D position
  from a global coordinate system to the local
  coordinate system, when this posRot is the position of
  the local coordinate system in the global.
  Returns a UMatrix4 value */
  inline UMatrix4 getMtoRMatrix()
  {  return URotation::asMatrix4x4MtoR(getPos()); };
  /**
  Convert this (guidemark) position and Rotation to map
  coordinates, when thw guidemark is seen from this (camera) position.
  If mirror is true, the guidemark is rotated to face the camera.
  Otherwise it is assumed to face the camera already (the values are not inspected). */
  void setCtoR(UPosRot * camera, UPosRot * guidemark, bool mirror);
  /**
  Convert this (guidemark) position and Rotation to camera coordinates,
  when the guidemark and camera is in map coordinates. */
  void setRtoC(UPosRot * camera, UPosRot * guidemark);
  /**
  Print values */
  void print(const char * prestring);
  /**
  Get position only */
  inline UPosition * getPos()
  { return UPosition::getPos(); };
  /**
  Get rotation only */
  inline URotation * getRot()
  { return URotation::getRot(); };
  /**
  Get position only */
  inline UPosition pos()
  { return *UPosition::getPos(); };
  /**
  Get rotation only */
  inline URotation rot()
  { return *URotation::getRot(); };
  /**
  Get UPosRot  */
  inline UPosRot * getPosRot()
  { return this; };
  /**  Get x - front in meter */
  inline double getX() { return x; };
  /**  Get y - left in meter */
  inline double getY() { return y; };
  /**  Get z - height in meter */
  inline double getZ() { return z; };
  /**  Get omega - rotation on x-axis (in radians) */
  inline double getOmega() { return Omega; };
  /**  Get phi - rotation on y-axis (in radians) */
  inline double getPhi() { return Phi; };
  /**  Get kappa - rotation on z-axis (in radians) */
  inline double getKappa() { return Kappa; };
  /**  Get Theta (same as kappa) - rotation on z-axis (in radians) */
  inline double getTheta() { return Kappa; };
  /**  Set x - front in meter */
  inline void setX(double value) { x = value; };
  /**  Set y - left in meter */
  inline void setY(double value) { y = value; };
  /**  Get z - height in meter */
  inline void setZ(double value) { z = value; };
  /**  Get omega - rotation on x-axis (in radians) */
  inline void setOmega(double value) { Omega = value; };
  /**  Get phi - rotation on y-axis (in radians) */
  inline void setPhi(double value) { Phi = value; };
  /**  Get kappa - rotation on z-axis (in radians) */
  inline void setKappa(double value) { Kappa = value; };
  /**  Get Theta (same as kappa) - rotation on z-axis (in radians) */
  inline void setTheta(double value) { Kappa = value; };
  /**
   * \brief Convert this plane from camera to robot coordinates.
   * Assumes that this pose is the camera pose relative to the robot
   * \param plane is the plane to convert.
   * \returns the same plane in robot coordinates. */
  UPlane getCamToRobPlane(UPlane plane);
  /**
   * \brief Convert this plane from robot to camera coordinates.
   * Assumes that this pose is the camera pose relative to the robot
   * \param plane is the plane to convert.
   * \returns the same plane in camera coordinates. */
  UPlane getRobToCamPlane(UPlane plane);
};

#endif
