/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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
#ifndef UVISDATA_H
#define UVISDATA_H

#include <ugen4/ulock.h>
#include <umap4/uprobpoly.h>

//#include "uvisrangeset.h"

/**
Main class for vision based data from camera server

@author Christian Andersen
*/
class UVisData
{
public:
  /**
  Constructor */
  UVisData();
  /**
  Destructor */
  ~UVisData();
  /**
  Get polygon name */
  inline const char * getPolyName()
  { return polyName; };
  /**
  inline set polygon name */
  inline void setPolyName(const char * source)
  { strncpy(polyName, source, MAX_SML_NAME_LENGTH); };
  /**
  Get pointer to polygon structure */
  inline UProbPoly * getPoly()
  { return poly; };
  /**
  Lock for polygon coordinates */
  ULock polyLock;
  /**
  Is new polygon data available */
  inline bool isPolyNew()
  { return polyNew; };
  /**
  Set new polygon flag */
  inline void setPolyNew(bool value)
  { polyNew = value; };
  /**
  Make range intervals for this vision polygon
  from the provided range */
  bool makeRangeIntervals(double startRange, UPoseTime * robPose, UPose * destPose);
  /**
  Get pointer to range intervals */
/*  inline UVisRangeSet * getRangeSet()
  { return visRngs; };*/
  
protected:
  /**
  Pointer to polygon data buffer */
  UProbPoly * poly;
  /**
  Name of received polygon */
  char polyName[MAX_SML_NAME_LENGTH];
  /**
  Is polygon in robot local coordinates */
  bool polyLocalCoordinates;
  /**
  Is new data - set by data entry and reset by
  vision navigation - polling mode */
  bool polyNew;
  /**
  Passable range intervals */
//  UVisRangeSet * visRngs;

private:
  /**
  Polygon buffer */
  UProbPoly polyData;
};

#endif
