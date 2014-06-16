/***************************************************************************
 *   Copyright (C) 2008 by Christian Andersen   *
 *   chrand@mail.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "poseq.h"

PoseQ::PoseQ()
{
  clear();
}


PoseQ::~PoseQ()
{
}



PoseQ PoseQ::getPoseToMapPose(PoseQ poseLocal)
{
  PoseQ result;
  double ch, sh;
  //
  ch = cos(h);
  sh = sin(h);
  //
  result.x =  ch * poseLocal.x - sh * poseLocal.y + x;
  result.y =  sh * poseLocal.x + ch * poseLocal.y + y;
  result.h = limitToPi(poseLocal.h + h);
  return result;
}

////////////////////////////////////////

PoseQ PoseQ::getPoseToMap(double localX, double localY)
{
  PoseQ result;
  double ch, sh;
  //
  ch = cos(h);
  sh = sin(h);
  //
  result.x =  ch * localX - sh * localY + x;
  result.y =  sh * localX + ch * localY + y;
  result.h = 0.0;
  //
  return result;
}
