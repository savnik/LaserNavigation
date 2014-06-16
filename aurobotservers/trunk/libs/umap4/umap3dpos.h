/***************************************************************************
 *   Copyright (C) 2004 by Christian Andersen                              *
 *   jca@oersted.dtu.dk                                                    *
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
#ifndef UMAP3DPOS_H
#define UMAP3DPOS_H

#include <ugen4/u3d.h>
#include "upose.h"

/**
@author Christian Andersen
*/
class UMap3dPos{
public:
  /**
  Constructor */
  UMap3dPos();
  /**
  Destructor */
  ~UMap3dPos();
  /**
  Copy all 3d info from source.
  Returns true. */
  bool copy(UMap3dPos * source);
  /**
  Save data in html-like format.
  Returns true if saved. */
  bool save(Uxml3D * fxmap, const char * name = NULL);
  /**
  Update 3D info from 2D pose.
  This function updates a 3D position partially,
  assuming that height and Omega, Phi is
  independant of pose information. */
  void update(UPoseQ * poseQ);
  /**
  Load this position from an xml-class object.
  Returns true if read. */
  bool load(Uxml3D * fxmap, char * name = NULL);

public:
/**
  Estimated position (x,y,(z)) */
  UPosition estPos;
  /**
  Estimated rotation (Phi used only) */
  URotation estOri;
  /**
  Covariance matrix of position (in segment coordinates)
  Position covariance is expected tobe
  independant of rotation covariance. */
  UMatrix4 estPosQ;
  /**
  Has object an orientation */
  bool oriValid;
  /**
  Covariance matrix of orientation (in segment coordinates)
  Independent of position covariance, */
  UMatrix4  estOriQ;
};

#endif
