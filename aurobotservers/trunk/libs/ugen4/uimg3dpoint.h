/***************************************************************************
 *   Copyright (C) 2008 by DTU (Christian Andersen)                        *
 *   rse@oersted.dtu.dk                                                    *
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
#ifndef UIMG3DPOINT_H
#define UIMG3DPOINT_H

#include "uposrot.h"
#include "ulock.h"
#include "uplane.h"

/**
 * Class with 3D point found by stereo calculation */
class UImg3Dpoint
{
public:
  /**
   * Get red pixel coloure */
  inline int red()
  { return pixLeft & 0xff; };
  /**
   * Get green pixel coloure */
  inline int green()
  { return (pixLeft >> 8) & 0xff; };
  /**
   * Get red pixel coloure */
  inline int blue()
  { return (pixLeft >> 16) & 0xff; };
  /**
   * get gray level value - intensity (as (r+g+b)/3)*/
  inline int gray()
  {
    int v;
    v  = (pixLeft >> 16) & 0xff;
    v += (pixLeft >> 8) & 0xff;
    v += pixLeft & 0xff;
    v /= 3;
    return v;
  };

public:
    /** 3D position in camera coordinates (x is forward, y is left, z is up) */
  UPosition pos;
  /** Pixel information in left image - coded as unsigned long (32 bit) - with
     * red as 8 least significant bit, green as bext 8 bits and blue as next 8 bit
   * - most significant bits are (A) and is unused */
  unsigned long pixLeft;
  /**
   * Image row number in (left) source image. the data is organized
   * in column then row order - to ease data use */
  int row;
  /**
   * Image column number in (left) source image. the data is organized
   * in column then row order - to ease data use */
  int column;
  /**
   * quality value that can be used for annotation of validity or other things */
  int q;
};


/**
 * Class with 3D point found by stereo calculation */
class UImg3Dpoints : public ULock, public UDataBase
{
public:
  /**
   * Constructor */
  UImg3Dpoints()
  {
    p3dCnt = 0;
    serial = 0;
    inPoseCoordinates = true;
    p3d = NULL;
    p3dCntMax = 0;
  };
  /**
   * Constructor */
  ~UImg3Dpoints()
  {
    p3dCnt = 0;
    p3dCntMax = 0;
    free(p3d);
    p3d = NULL;
  }
  /**
    Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "img3d";
  };
  /**
     * Clear the 3D data point set
   * \param imgSerial new serial number valid for the 3D data */
  void clear3d(unsigned int imgSerial, UTime imgTime)
  {
    p3dCnt = 0;
    serial = imgSerial;
    time = imgTime;
    inPoseCoordinates = true;
  };
  /**
   * Convert 3D cloud data in sensor coordinates to robot coordinates
     does nothing, if inPoseCoordinates is false */
  void toRobotCoordinates();
  /**
   * Estimate largest plane in 3D cloud, using RANSAC
   * \param samples is number of samples the RANSAC should try (from probability of false points)
   * \param sigma is the standard deviation for the point distribution around the plane.
   * \param refPlane reference plane for tilt test
   * \param tiltLim is the minimum tilt limit relative to refPlane, for an accepted plane.
   * The value is the dot-product limit in range [0..1],
   * tiltLim=0 accept all planes, tiltLim=1 accept a perfect aligned plane (=refPlane) only.
   * \param xLimit is the maximum acceptable x-value (works as range limit) - only if rngComp=true
   * \param cnt is on return then number of points associated to the plane.
   * \returns the estimated plane.*/
  UPlane getRansacPlane(int samples, double sigma, UPlane refPlane,
                        double tiltLim, double xLimit, int * cnt);
  /**
  Add a 3D point
  \param p is the 3d point to add.
  \returns number of points in the point array. */
  inline UImg3Dpoint * add(UImg3Dpoint * p)
  {
    UImg3Dpoint * p1;
    const int EXPAND_SIZE = 10000;
    if (p3dCnt >= p3dCntMax)
    {
      p3d = (UImg3Dpoint *) realloc(p3d, (p3dCntMax + EXPAND_SIZE)*sizeof(UImg3Dpoint));
      p3dCntMax += EXPAND_SIZE;
    }
    p1 = &p3d[p3dCnt++];
    *p1 = *p;
    return p1;
  }
  /**
  Clear all points from array */
  inline void clear()
  { p3dCnt = 0; };
  /**
  Add point based on values
  \param x,y,z is the 3d position to add.
  \param row source image row.
  \param col source image column
  \param pix is the pixel value
  \param q quality value
  \returns number of points in array */
  UImg3Dpoint * add(double x, double y, double z, int row, int col, int pix, int q)
  {
    const int EXPAND_SIZE = 10000;
    UImg3Dpoint * p;
    if (p3dCnt >= p3dCntMax)
    {
      p3d = (UImg3Dpoint *) realloc(p3d, (p3dCntMax + EXPAND_SIZE)*sizeof(UImg3Dpoint));
      p3dCntMax += EXPAND_SIZE;
    }
    p = &p3d[p3dCnt++];
    p->pos.set(x, y, z);
    p->row = row;
    p->column = col;
    p->pixLeft = (unsigned int)pix;
    p->q = q;
    return p;
  };

  /**
   * Make PCL file (for PCD viewer - and the like.
   * creates filename from data 
   * \param name optional filename. If no provided name, then a name is constructed from data (cloudNNN_time.pcd).
   * \param andRGB will add RGB values in points to the file
   * \param andRowCol will add row and column from source image to the file 
   * \returns true if file is created */
  bool makePCLFile(const char * name, bool andRGB, bool andRowCol);
  
  
public:
  /**
   * maximum number of available elements in p3d buffer */
  //static const int p3dMaxCnt = 320 * 240;
  /**
   * 3D points */
  UImg3Dpoint * p3d;
  /**
   * number of actual points in p3d */
  int p3dCnt;
  /**
   * number of actual points in p3d */
  int p3dCntMax;
  /**
   * Image serial number for these data */
  unsigned int serial;
  /**
   * Valid time for the 3D data (data capture time) */
  UTime time;
  /**
   * Sensor pose (should be set by data source) */
  UPosRot pose;
  /**
   * In sensor pose coordinates (i.e. not converted to 'robot' coordinates */
  bool inPoseCoordinates;
};


#endif
