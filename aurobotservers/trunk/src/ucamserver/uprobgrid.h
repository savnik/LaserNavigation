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
#ifndef UPROBGRID_H
#define UPROBGRID_H

#include <ugen4/uimage2.h>
#include <umap4/upose.h>
#include <umap4/uprobpoly.h>

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/**
MAX_POLYGON_COUNT is max number of polygons used for gridmap paint */
#define MAX_POLYGON_COUNT 5

/**
Grid solution to passable free aea in front of robot.

@author Christian Andersen
*/
class UProbGrid
{
public:
  /**
  Constructor */
  UProbGrid();
  /**
  Destructor */
  ~UProbGrid();
  /**
  Set image used for grid probability  */
  inline void setGrid(UImage * image)
     { grid = image; };
  /**
  Set image used for temporary painting  */
  inline void setTempImg(UImage * image)
     { tempGrid = image; };
  /**
  Set image used for grid probability  */
  inline UImage * getGrid()
     { return grid; };
  /**
  Set image used for temporary painting  */
  inline UImage * getTempImg()
     { return tempGrid; };
  /**
  Set new polygon from most recent image */
  bool setNewPolygon(UPose robPose, UTime poseTime);
  /**
  Get start pf polyon for the newest set */
  UPosition * getNewPoly();
  /**
  Get start of boolean list if obstacle markers */
  bool * getNewIsObst();
  /**
  Get number of vertices in polygon */
  int getNewPolyCnt();
  /**
  Make probability grid */
  bool makeProbGrid(const char * sourceName);
  /**
  Same as above, but without the fillling of extra images to get masks. etc. */
  bool makeProbGrid2(const char * sourceName);
  /**
  Get cell size for grid in meters per pixel. */
  inline double getCellSize()
    { return gridCellSize; };
  /**
  Save grid in matlab format.
  Filename is exclusive extension (.m) and is saved
  in default dataPath. */
  void saveMatlab(const char * filename);
private:
  /**
  Paint 1 meter grid in this image with zero at this
  pixel position and this pixel size. */
  bool paintGrid(UImage * img,
               int zeroX, int zeroY, // pixel position of zero
               double pixelSize,     // meters per pixel
               UPixel gridRGB,       // colour for 1 m markings
               UPixel zeroRGB);      // colour for zero crossing

protected:
  /**
  Polygons to beused in  */
  UProbPoly polys[MAX_POLYGON_COUNT];
  /**
  Number of used polygons */
  int polysCnt;
  /**
  Number of oldest polygon */
  int newestPoly;
  /**
  Grid of probability */
  UImage * grid;
  /**
  Temporary grid for polygon painting */
  UImage * tempGrid;
  /**
  Cell size for grid */
  double gridCellSize;
  /**
  Grid centre in pixels (cells) */
  int gcX, gcY; // grid center
};


#endif
