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

#include "uplane.h"
#include "uposrot.h"

//////////////////////////////////////////////

UPosRot::UPosRot()
        : UPosition(), URotation()
{}

//////////////////////////////////////////////

UPosRot::~UPosRot() 
{}

//////////////////////////////////////////////

void UPosRot::setCtoR(UPosRot * camera, UPosRot * guidemark, bool mirror)
{ // set this posRot as the guidemark position, in
  // as seen from a more global perspective, the
  // 'guidemark' position is seen from the 'camera' position.
  UPosition ref = *guidemark->getPos();
  UPosition refY(0.0, 1.0, 0.0);
  UPosition refZ(0.0, 0.0, 1.0);
  UMatrix4 conv, conv1, conv2;
  UPosition gblY, gblZ, gbl0, gblR;
  URotation rot;
  // convert, so that GMK faces camera
  if (mirror)
    refY.set(0.0, -1.0, 0.0);
  //
  conv1 = guidemark->getRtoMMatrix();
  conv2 = camera->getRtoMMatrix();
  conv = conv2 * conv1;
  gbl0 = conv2 * ref;
  gblY = conv * refY;
  gblZ = conv * refZ;
  setPos(gbl0);
  gblY = gblY - gbl0;
  gblZ = gblZ - gbl0;
  rot.setFromYZ(gblY, gblZ);
  setRot(rot);
}

//////////////////////////////////////////////////

void UPosRot::setRtoC(UPosRot * camera, UPosRot * guidemark)
{ // set this posRot as the guidemark position,
  // as seen from a the camera,
  // 'guidemark' and 'camera' position are in robot coordinates.
  UPosition ref = *guidemark->getPos();
  UPosition refY(0.0, 1.0, 0.0);
  UPosition refZ(0.0, 0.0, 1.0);
  UMatrix4 conv, conv1, conv2;
  UPosition gcrY, gcrZ, gcr0; // guidemark pos in camera reference
  URotation rot;
  //
  conv1 = guidemark->getRtoMMatrix();
  // get conversion from map (robot) to camera
  conv2 = camera->getMtoRMatrix();
  // get guidemark vectors in camera coordinates
  conv = conv2 * conv1;
  gcrY = conv * refY;
  gcrZ = conv * refZ;
  // get guidemark center position in camera coordinates
  gcr0 = conv2 * ref;
  setPos(gcr0);
  gcrY = gcrY - gcr0;
  gcrZ = gcrZ - gcr0;
  rot.setFromYZ(gcrY, gcrZ);
  setRot(rot);
}

//////////////////////////////////////////////////

void UPosRot::print(const char * prestring)
{
  printf("%s  x:%9.4f  y:%9.4f  z:%9.4f\n", prestring, x, y, z);
  printf("  -- O:%9.4f  P:%9.4f  K:%9.4f\n", Omega, Phi, Kappa);
}

//////////////////////////////////////////////////

UPlane UPosRot::getCamToRobPlane(UPlane plane)
{
  UPlane result;
  UMatrix4 mRC;
  UMatrix4 mCR;
  UPosition pro(0.0, 0.0, 0.0);
  UPosition prp;
  // get conversion matrix
  mRC = getMtoRMatrix(); // map (robot) to camera convert
  mCR = getRtoMMatrix(); // camera to map (robot) coordinates
  // Get robot origin in camera coordinates
  pro = mRC * pro;
  // get plane point in camera coordinates and convert to robot coordinates
  prp = mCR * plane.getOnPlane(pro);
  // set resulting plane
  result.set(prp);
  //
  return result;
}

//////////////////////////////////////////////////

UPlane UPosRot::getRobToCamPlane(UPlane plane)
{
  UPlane result;
  UMatrix4 mRC;
  UMatrix4 mCR;
  UPosition pco(0.0, 0.0, 0.0);
  UPosition pcp;
  // get conversion matrix
  mRC = getMtoRMatrix(); // map (robot) to camera convert
  mCR = getRtoMMatrix(); // camera to map (robot) coordinates
  // Get robot origin in camera coordinates
  pco = mCR * pco;
  // get plane point in camera coordinates and convert to robot coordinates
  pcp = mRC * plane.getOnPlane(pco);
  // set resulting plane
  result.set(pcp);
  //
  return result;
}

