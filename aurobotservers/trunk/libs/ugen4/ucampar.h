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

#ifndef UCAMPAR_H
#define UCAMPAR_H

#include "umatrix.h"
#include "u3d.h"
// name length from camera itself
//#define MAX_CAMERA_NAME_LENGTH 100

/**
The class holds camera parameters and functions related
to the inner orientation of a camera.
This includes radial error and fical length.
Further is matrices for projecting from 3D to image
coordinates and conversion to image coordinates included.
  *@author Christian Andersen  */
class UCamPar
{
public:
  /**
  Constructor */
  UCamPar();
  /**
  Destructor */
  ~UCamPar();
  /**
  Invalidate information */
  inline void invalidate() { parValid = false; };
  /**
  Is parameters valid */
  inline bool isValid() { return parValid; };
  /**
  From an undistorted pixel position {xu,yu}
  to the cooresponding distorted the pixel value {xd, yd}.
  Res factor is size of pixels relative to
  resolution where camera parameters are valid.
  - typical 1, 2, 4 for (640x480, 320x240, 160x120) */
  bool getRadialU2D(float xu, float yu,
                    float * xd, float * yd);
  /**
  From an image position in a distorted {xd,yd} image to
  a pixel position in the undistorted image {xu,yu} with
  these camera parameters.
  Res factor is size of pixels relative to
  resolution where camera parameters are valid.
  - typical 1, 2, 4 for (640x480, 320x240, 160x120) */
  bool getRadialD2U(float xu, float yu,
                    float * xd, float * yd);
  /**
  Set all camera parameters in one call
  hx, hy, k1, k2, focal-length are all
  as found for a 640x480 image.
  pixelSize is for actual image releative to 640x480.
  ix 320x240 has pixel size 2.
  Returns true if set. */
  bool setCameraParameters(float hx, float hy,
                  float k1, float k2,  float focalLng,
                  float pixSizeFactor);
  /**
  Copy parameters from other set of parameters. */
  bool setCameraParameters(UCamPar * source);
  /**
  Get image center point X in actual resolution */
  inline float getHx() {return headX/resFactor;};
  /**
  Get image center point Y in actual resolution */
  inline float getHy() {return headY/resFactor;};
  /**
  Get focus length in pixels in actual resolution */
  inline float getFocalLength() {return focalLength/resFactor;};
  /**
  Get size of one pixel relative to max resolution.
  Max resolution is normally 640x480, so pixSize
  of 4 corresponds to 160x120 pixels. */
  inline float getPixelSize() {return resFactor;};
  /**
  Set size of one pixel relative to max resolution.
  Max resolution is normally 640x480, so pixSize
  of 4 corresponds to 160x120 pixels. */
  bool setPixelSize(float pixSize);
  /**
  Get radial distorsion value proportional
  to radius from center point in 3rd power.
  Value relates to max camera resolution. */
  inline float getK1() {return radialK1;};
  /**
  Get radial distorsion value proportional
  to radius from center point in 5rd power.
  Value relates to max camera resolution. */
  inline float getK2() {return radialK2;};
  /**
  Get matrix for conversion from image to pixel coordinates. */
  inline UMatrix4 * getItoP() { return & mItoP;} ;
  /**
  Get conversion matrix from a 3d camera coordinate to image
  pixel position, where the camera coordinates are oriented
  as robot coordinates (z is up). */
  UMatrix4 getCtoPRob();
  /**
  Save camera parameters to this configuration file
  under this key name */
  bool savePar(Uconfig * ini, const char * key);
  /**
  Set camera parameters from 'ini' configuration file
  under this 'key'. */
  bool setCameraParameters(Uconfig * ini, const char * key);
  /**
  Print camera parameters to the destination string
  with 'preString' in front of test.
  (format suited for console display).
  'maxLength' is length of 'dest' buffer.
  Returns length of produced string. */
  int snprint(const char * preString,
              char * dest,
              const int maxLength);
  /**
  Print most of values to console (debug) */
  void print(const char * prestring);
  /**
  Convert a 3D position to pixel position.
  3D position is in camera coordinates using
  robot coordinate system (x forward, y left z up). */
  URPos getCtoPRob(UPosition pos3D);
  /**
  Convert pixel position to 3D position with this distance (x) coordinate
  3D position is in robot coordinates system (x forward, y left and z up),
  by oriented as camera. */
  UPosition getPtoCRob(int x, int y, float distance);
  /**
  Convert pixel position to 3D position with this distance (x) coordinate
  3D position is in robot coordinates system (x forward, y left and z up),
  by oriented as camera. */
  UPosition getPtoCRob(float x, float y, float distance);

protected:
  /**
  Set conversion matrices after change in parameters. */
  bool setMatrices(void);

public:
//  char camName[MAX_CAMERA_NAME_LENGTH];
  /**
  Is camera parameters valid */
  bool  parValid;
protected:
  /**
  Focal length for focus at infinity. */
  float focalLength;
  /**
  Radial error correction parameters.
  K1 is proportional to r^3 and K2 to r^5. */
  float radialK1;
  float radialK2;
  /**
  Center point (optical) */
  float headX;
  float headY;
  /**
  Factor from actual coordinates to 640x480.
  i.e resFactor 2 corresponds to 320x240 image.
  resFactor is the same as pixelSize (relative to 640x480). */
  float resFactor;
  /**
  3D (y back) to 2D (0,0 in center, y up) */
  UMatrix4 mP; //
  /**
  Image coordinate (2d y up, 0,0 in center) to
  pixel coordinate (2d y down, 0,0 top left) */
  UMatrix4 mb; //
  /**
  Pixel to Image coordinate matrix */
  //UMatrix4 mPtoI; //
  /**
  Image 3d (z is back) coordinate to pixel (y down) matrix */
  UMatrix4 mItoP; //
  /**
  Matrix to convert camera 3D oordinate to pixel position
  when the 3D position is in robot coordinates (x forward, y left and z up). */
  UMatrix4 mCtoPRob;
};

#endif
