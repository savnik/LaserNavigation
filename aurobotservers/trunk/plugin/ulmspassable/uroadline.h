/***************************************************************************
 *   Copyright (C) 2007-2008 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UROADLINE_H
#define UROADLINE_H


#include <ugen4/uline.h>
#include <umap4/upose.h>

/**
Holds information on one road line

	@author Christian <chrand@mail.dk>
*/
class URoadLine{
public:
  /**
  Constructor */
  URoadLine();
  /**
  Destructor */
  ~URoadLine();
  /**
  Reset line filter */
  void clear(int edge);
  /**
  Get line quality (1 is pefect) */
  inline double getQual()
  { return 1.0 / (1 + sqrt(pL) * double(13 - mini(10, updateCnt))); };
  /**
  Get line quality (1 is pefect) */
  inline double getEstSD()
  { return sqrt(pL); };
  /**
  Is line valie */
  inline bool isValid()
  { return valid; };
  /**
  Is line a left, centre or right line */
  bool isA(int lineType)
  { return lineType == edge; };
  /**
  Is line a left (0), centre (1) or right line (2) */
  int getLineType()
  { return edge; };
  /**
  Get distance to this position in number of SDs */
  double getDistanceSD(UPosition * pos);
  /**
  Get estimated road line as sine segment */
  inline ULineSegment * getLine()
  { return &line; };
  /**
  Get number of updates for line */
  inline int getUpdateCnt()
  { return updateCnt; };
  /**
  Get estimated line SD */
  inline double getEstVar()
  { return xR; };
  /**
  Get scan serial number */
  inline unsigned long getScanSerial()
  { return scanSerial; };
  /**
  Get scan serial number */
  inline unsigned long getLineSerial()
  { return lineSerial; };
  /**
  Update the line estimate with this position */
  void update(UPosition pos, UPoseTime pose, unsigned long scan,
              bool moving, int piIdx);
  /**
  Create new line with this new edge estimate */
  void setNew(UPosition pos, UPoseTime pose, unsigned long scan, int piIdx, unsigned long roadSerial);
  /**
  Get pose at last update */
  inline UPoseTime getPose()
  { return odoPose; };
  /**
  Line maintenance, i.e. drop invalid lines */
  void postUpdate(unsigned int scan, UPoseTime * pose, bool moving);
  /**
  Get number of updates for line */
  inline int getPisIdx()
  { return pisIdx; };

  
protected:
  /**
  Estimated road line */
  ULineSegment line;
  /**
  Estimate cvariance of line segment */
  double pL;
  /**
  Estimated variance */
  double xR, pR;
  /**
  Edge 0=left 1=center 2=right */
  int edge;
  /**
  Is line valie */
  bool valid;
  /**  index of this line */
//  int idx;
  /**
  Updated by scan number */
  unsigned long scanSerial;
  /**
  Unique serial number for road - for correlation at client end */
  unsigned long lineSerial;
  /**
  Updated from this robot pose (and time) */
  UPoseTime odoPose;
  /**
  Number of updates to this line */
  int updateCnt;
  
public:
  /** index pf last apassable interval updating this line */
  int pisIdx;
};

#endif
