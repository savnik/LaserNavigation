/***************************************************************************
 *   Copyright (C) 2006 by DTU                                             *
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
#include "ucammount.h"

UCamMounted::UCamMounted(UCamDevBase * device)
  : UCamRad(device)
{ // constructor
  // client in control is
  // not tested nor set by camera functions
  clientInControl = -1;
  //mountName[0] = '\0';
  logip = NULL;
  varCamPose = NULL;
  varMountName = NULL;
}

//////////////////////////////////////////////////////

UCamMounted::~UCamMounted()
{ // Destructor
}


/////////////////////////////////////////////////////////////

bool UCamMounted::isClientInCharge(int client)
{
  bool result;
  //
  result = (client == clientInControl);
  if (not result)
    if (clientInControl == -1)
    {
      clientInControl = client;
      result = true;
    }
  //
  return result;
}

/////////////////////////////////////////////////////////

URotation UCamMounted::getRotCC()
{
  URotation r = getRot();
  URotation result(-r.Phi, r.Kappa, -r.Omega);
  return result;
}

/////////////////////////////////////////////////////////////

URotation UCamMounted::getRot()
{
//   URotation result;
//   if (isPantiltValid() and isPantiltSupported())
//   {
//     result = relRot;
//     result.Kappa += double(-getPanPos()) * M_PI / 180.0 / 100.0;
//     result.Phi += double(-getTiltPos()) * M_PI / 180.0 / 100.0;
//     return result;
//   }
//   else
//    return relRot;
  UPosRot p = varCamPose->get6D();
  return p.rot();
}

///////////////////////////////////////////////////////////////

void UCamMounted::setPosOnRobot(UPosRot * pose3d)
{
//   relPos = *pose3d->getPos();
//   if (isPantiltValid() and isPantiltSupported())
//   { // compensate for actual pan-tilt position and
//     // save camera position in an-tilt home position.
//     relRot.Omega = pose3d->Omega;
//     relRot.Kappa = pose3d->Kappa - double(-getPanPos()) * M_PI / 180.0 / 100.0;
//     relRot.Phi = pose3d->Phi - double(-getTiltPos()) * M_PI / 180.0 / 100.0;
//   }
//   else
//     relRot = *pose3d->getRot();
  varCamPose->set6D(pose3d);
}

////////////////////////////////////////////////////////////////

void UCamMounted::setPosOnRobot(UPosition * pos, URotation * rot)
{
  UPosRot pr;
  pr.set(*pos, *rot);
  setPosOnRobot(&pr);
}

////////////////////////////////////////////////////////////////

bool UCamMounted::getMtoPix(UPosition pos3D, bool useRad, float * pixx, float * pixy)
{
  UMatrix4 mMtoC;
  UMatrix4 vC;
  UPosition pos;
  URPos xyU, xyD;
  bool result = true;
  bool insideImage = false;
  int allowedBorder = 20;
        // get conversion matrix to camera coordinates
  mMtoC = getPosRot().getMtoRMatrix();
        // convert position to camera coordinates
  vC = mMtoC * pos3D.asVector4(true);
        // copy to UPosition format
  pos.copy(&vC);
        // get pixel coordinates (undeformed by radial error)
  xyU = getCamPar()->getCtoPRob(pos);
  if (not useRad)
    allowedBorder = 0;
  // correcting for radial error, pixels outside may be converted to inside
  // so allow a border outside image for undestorted pixel positions
  insideImage = (xyU.x > -allowedBorder) and (xyU.x < (dev->getWidth() + allowedBorder)) and 
      (xyU.y > -allowedBorder) and (xyU.y < (dev->getHeight() + allowedBorder));
  if (useRad)
  { // NB! radial correction is valid inside image only
    // inside image (allow margin of 20 pixels)
    if (insideImage)
      // get deformed pixel position (real image position)
      result = getCamPar()->getRadialU2D(xyU.x, xyU.y, &xyD.x, &xyD.y);
    else
      // better to use uncorrected position
      xyD = xyU;
  }
  else
    // use position as is
    xyD = xyU;
  //
  if (result)
  {
    if (pixx != NULL)
      *pixx = xyD.x;
    if (pixy != NULL)
      *pixy = xyD.y;
  }
  //
  return result and insideImage;
}

///////////////////////////////////////////

bool UCamMounted::logImage(UImage * img)
{
  bool result = true;
  // write image name into logfile
  if (dev->isLog())
    result = logip->logImage(img, this);
  // mark image as updated - may trigger a push command
  // (when client handler has time)
  img->imgUpdated();
  //
  return result;
}

////////////////////////////////////

void UCamMounted::createVars()
{
  if (vars != NULL)
  {
    varMountName = vars->addVarA("aliasname", "unmounted", "s",
                              "(r/w) nickName for this camera");
    UCamRad::createVars();
    //
    varCamPose = vars->addVarA("camPose", "0 0 0.5 0 0 0", "6d",
                              "(r/w) Camera position on robot [x,y,z,O,P,K]");
  }
}
