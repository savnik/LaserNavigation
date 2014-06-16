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
#ifndef UFUNC_BALLKL_H
#define UFUNC_BALLKL_H

#include <cstdlib>

#include <ucam4/ufunctioncambase.h>


// //////////////////////////////////////////////////////
// //////////////////////////////////////////////////////
// //////////////////////////////////////////////////////

/**
Example plugin that demonstrates a plogin that provides a resource.
A similar example plugin is available that uses the shared resource.

The shared resource provides the simple functionality in the form of a line.
The resource provides functions to calculate the length of the line.

@author Christian Andersen
*/
class UFuncBallKL : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncBallKL()
  {
    setCommand("ballkl", "ballkl", "camera based ball detect (" __DATE__ " " __TIME__ ") (Villen og Linsey)");
    createBaseVar();
  };
  /**
  Destructor */
  virtual ~UFuncBallKL();
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
  /**
  Create create globally  available status and configuration variables */
  void createBaseVar();

  /**
   * Find red ball in image, using parameters from global var pool
   * \param cam camera from wich the image was taken (has focal length and camera pose)
   * \param img is a pointer to the image to be analized
   * \param debug if true, then more images are producen in the image pool
   * \param blue if true, then the blue color values are used (else the red values)
   * \returns true if at least one red ball is found */
  bool findBall(UCamPush * cam, UImage * img, bool debug, bool blue);

  /**
  Find balls in this image.
  This function is almost a direct
  copy of the 'fitellipse.c' example from the opencv package.
  \param srcImg is a pointer to the image to be analized
  \param dstImg is where the pixels with the ball color is saved.
  \param debug if true, then more images are producen in the image pool
  \param blue are we looking for a blue ball (else a red) */
  void maskColors(UImage * srcImg, UImage * dstImg, bool blue, bool debug);

  /**
  Find balls in this image.
  This function is almost a direct
  copy of the 'fitellipse.c' example from the opencv package.
  \param img is a pointer to a BW mask image with interesting pixels beeing > 0
  \param ellImg optional image where found ellipses are drawn (for debug)
  \returns true if at least one ball candidate is found. */
  bool findBallCandidates(UImage * img, UImage * ellImg);
  /**
   filter the image.
   Source and destination image may be the same.
   \param src is the source image
   \param dst is the destination image */
  void filterMask(UImage * src, UImage * dst);
  /**
   * Find the best of the estimated ball candidate and calculate the ball position
   * relalive to the robot.
   * Expect estimate ellipses that may be balls in the array 'ell' with ellCnt ellipses found.
   * The result is converted to first camera coordinates using the camera focal length and a known ball size.
   * There is no radial error correction applied, so some error must be expected.
   * The camera coordinates are transformed to robot coordinates using a known camera pose (from the camera structure).
   * The result is saved in the 'ballPos' variable in meters relative to the robot.
   * If the ball is close to the edge of the image and partially out of the image, then one of the flags isOutL or isOutR is set to true.
   * \param cam is a pointer to a camera device with camera position and lens parameters
   * \param ellImg optional image for debug paint, emmits also more printout to console
   * \param ballDiam is the known diameter of the ball (in meter)
   * \returns true if possible - and if true, then the ballPos, isOutL and isOutR is set. */
  bool calculateBallPosition(UCamPush * cam, UImage * ellImg, double ballDiam);

private:

  static const int MAX_ELL_CNT = 100;
  /// resulting ellipses found in image
  CvBox2D32f ell[MAX_ELL_CNT];
  /// number of ellipses found
  int ellCnt;
  /// most likely ball position in robot coordinates
  UPosition ballPos;
  /// size of source image in pixels
  int imgSizeH;
  /// size of source image in pixels
  int imgSizeW;


  /// pointer to limiting red values redMin, redMax, greenMin, greenMax
  UVariable * varRedLim;
  /// pointer to ball position {x,y,z}
  UVariable * varRedPos;
  /// pointer to time of detection
  UVariable * varRedTime;
  /// pointer to number of balls found in image
  UVariable * varRedCnt;
  /// pointer to boolean on left and right edge value (red ball)
//  UVariable * varRedEdgeLR;
  /// pointer to limiting red values uMin, uMax, vMin, vMax
  UVariable * varBlueLim;
  /// pointer to ball position {x,y,z}
  UVariable * varBluePos;
  /// pointer to time of detection
  UVariable * varBlueTime;
  /// pointer to number of balls found in image
  UVariable * varBlueCnt;
  /// pointer to boolean on left and right edge value (blue ball)
//  UVariable * varBlueEdgeLR;
  /// pointer to known size of ball
  UVariable * varBallSize;
  /// topmost line in image
  UVariable * varTopLine;
  /// minimum ratio metween major and minor axis of ellipse
  UVariable * varMajMinRatio;
  /// Maximum size for an estimated ball
  UVariable * varMaxSize;
  /// Maximum size for an estimated ball
  UVariable * varMinSize;
  /// default image to use - if not specified
  UVariable * varDefImg;
  /// first temp image number in image pool
  UVariable * varTmpImage;
  /// black limit for normalizing color
  UVariable * varBlackLimit;
};


#endif

