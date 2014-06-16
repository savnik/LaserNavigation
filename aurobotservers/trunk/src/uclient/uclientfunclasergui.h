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
#ifndef UCLIENTFUNCLASERGUI_H
#define UCLIENTFUNCLASERGUI_H

#include <ugen4/uimage.h>

#include "uclientfunclaser.h"
#include "uclientfuncimggui.h"
#include "uobstgrp.h"
#include "usfpool.h"

/**
Shows a receiced laser scan

@author Christian Andersen
*/
class UClientFuncLaserGui : public UClientFuncLaser, public UCallBack
{
public:
  /**
  Constructor */
  UClientFuncLaserGui();
  /**
  Destructor */
  virtual ~UClientFuncLaserGui();
  /**
  Save laser image to file */
  bool saveImage();
  /**
  Save images at every update */
  void setSaveImages(bool value)
    { saveImages = value; };
  /**
  Get save status */
  bool getSaveImages()
  { return saveImages; };
  /**
  Claer all (mostly all) old data */
  void clear();
  /**
  Set number of history scans to display.
  default is -1, that is all available scans */
  inline void setHistDisplay(int scansToDisplay)
  { paintScanHistCnt = scansToDisplay; };
  /**
  Set scale of MMR image i.e. height of image in meter */
  inline void setScale(double height)
    { MaxRange = height; };
  /**
  Set MMR image height and position of mmr.
  All in meter. */
  inline void setScale(double height, double mmrPos)
    {
      MaxRange = height;
      StartPos = mmrPos;
    };
  /**
  Paint an image with laser scan points.
  Or some other hard coded options.
  Parameter may be hadle to the source image (or NULL for filename in code). */
  void doImage(UHighGuiWindowHandle * sourceImg, 
               char * imgFullName,
               UPosition * imPos = NULL, 
               URotation * imRot = NULL);
  /**
  Set paint variable to this value
  Returns true if value is known */
  bool setPaintVar(const char * name, const char * value);
  /**
  Set repaint flag */
  void repaint();
  /**
  Paint the used set of passable intervals 
  in this image.
  Assume image pose is known */
//  void paintPisInImage(UImage * img);
  /**
  Get display window height */
  inline double getHeight()
  { return MaxRange; };
  /**
  Get display window height */
  inline double getRobPos()
  { return StartPos; };
  /**
  Set pointer to obstacles */
  void setObstList(UObstHist * value);
  /**
  Get position on robot floor for this (x,y) pixel position,
  given camera position/rotation (in robot coordinates), 
  the image size and the focal length.
  Result may be invalid if line is parallel to plane and
  behind robot if pointing up.
  f is focal length (in pixels) and (w,h) is image size matching
  this focal length.
  Returns position where a line in the pixel direction
  crosses the floor plane. */
  UPosition getPixToRobFloor(UPosition camPos, URotation camRot,
                             double f, int w, int h,
                             int x, int y);
  /**
  Set pointer to scan feature pool */
  void setSfPool(USFPool * pool);
  /**
  Call back function */
  virtual bool onEvent(const char * interface, const char * dataType, void * data);
  
protected:
  /**
  Called when a new scan is received */
  virtual bool gotNewData(ULaserDataSet * scan);
  /**
  Called when idle */
  virtual void doTimeTick();
  /**
  Repaint newest scan (with potentially new settings */
  bool paintNewestScan();
  /**
  Do the acual repaint of laser scan related data */
  bool doRepaint(ULaserDataSet * scan);

private:
  /**
  Paint laser scanner range circles */
  bool paintRangeRings(CvPoint robPos, UPosition devPos);
  /**
  Paint the robot frame */
  bool paintRobotMmr(CvPoint robPos);
  /**
  Paint the robot frame */
  bool paintRobotSmr(CvPoint robPos);
  /**
  Paint odometry grid */
  bool paintOdoGrid(double stepSize, bool bold);
  /**
  Paint motor speed and other WPC items */
  bool paintWpc(ULaserWpc * wpc);
  /**
  Show EKF position */
  bool paintEkfData();
  /**
  Paint control interval data and
  other related information */
  //bool paintCiData();
  void paintWpListData();
  /**
  Paint odometry position */
  bool paintOdoData(UPose seenFromPose);
  /**
  Paint planner */
  bool paintPlannerData(bool paintAll);
  /**
  Paint laser scan statistics */
  bool paintScanStatData(ULaserDataSet * scan,
                         bool paintVar, //bool paintEdge, bool paintCurv,
                         bool paintVarL, bool paintTilt);
  /**
  Paint passable intervals for this scan */
  void paintPis(UImage * img, ULaserDataSet * scan,
                UPose seenFromPose, const int cnt);
  /**
  Paint line from pose to last segment */
  void paintPostHistLine(UImage * img,
                         ULaserDataSet * scan,
                         UPose seenFromPose,
                         UPose * lastPose);
  /**
  Paint scan history for this scan */
  void paintHistScan(UImage * img, ULaserDataSet * scan,
                     UPose seenFromPose,
                     UPose * lastPose);
  /**
  Paint detected paths */
  bool paintPathData(ULaserPathResult * path, int num);
  /**
  Paint polygon with passable area from vision,
  as seend from this pose. */
  void paintFreePoly(UProbPoly * poly, UPose seenFromPose, bool historic);
  /**
  Paint a cross in image (2 lines in an x) at position x,y
  with size in pixels across. In this color */
  void paintCross(UImage * img, int x, int y,
                  int size, CvScalar col, int lineWidth);
  /**
  Paint a pose circle with heading vector */
  void paintPose(UImage * img, int x, int y, double h, 
                 int size, CvScalar col, int lineWidth);
  /**
  Paint obstacle polygons */
  void paintObstGrp(UImage * img, 
                    UObstacleGroup *obst,
                    UPose seenFromPose, int grpIdx);
  /**
  Paint odometry end pose, using endPoseX, endPoseY, endPoseH from calculator. */
  void paintOdoDest(UImage * img, UPose seenFromPose );
  /**
  Paint lines, e.g. scan feature lines */
  void paintLineSegments(UPose seenFromPose);
  
private:
  /**
  Image to be displayed */
  UImage * img;
  /**
  Max range in image */
  double MaxRange; // meter
  /**
  Start position of robot relative to bottom of image (meter) */
  double StartPos; // meter
  /**
  Number of updates that is received */
  int updates;
  /**
  Save image at every update? */
  bool saveImages;
  /**
  Is laser scanner window created */
  bool scannerWindowOK;
  /**
  openCV variables to show image */
  UHighGuiWindowHandle cvw;
  /**
  timeing? */
  UTime t1;
  /**
  Flag to indicate that image should be repainted */
  bool repaintImage;
  /**
  Timestamp of oldest data not painted */
  UTime repaintImageTime;
  /**
  Pointer to obstacle list */
  UObstHist * obsts;
  /**
  Pointer to scanfeatures */
  USFPool * sfPool;

public:
  /**
  Paint all support stuff */
  //bool paintAll;
  /**
  Paint EKF (GPS) data */
  bool paintGPS;
  /**
  Paint variables */
  bool paintVar;
  /**
  Paint statistical curves */
  bool paintCurves;
  /**
  Paint road lines */
  bool paintRoadLines;
  /**
  Paint interval lines */
  bool paintIntervalLines;
  /**
  Paint path lines - alternative and used path */
  bool paintPathLines;
  /**
  Pint all planned routes */
  bool paintPathAll;
  /**
  Paint path lines for all, also alternaive paths */
  bool paintPathLinesAll;
  /**
  Number of scans to use in path hist */
  int paintPathHistCnt;
  /**
  Number of scans to use in path hist */
  int paintScanHistCnt;
  /**
  Number of vision poygons to paint */
  int paintVisPolyCnt;
  /**
  Paint robot speed */
  bool  paintSpeed;
  /**
  Paint robot as an MMR (else and smr) */
  bool  paintMmr;
  /**
  Paint in bold (thicker bigger) */
  bool paintBold;
  /**
  Paint planning info (var and mission line) */
  bool paintPlan;
  /**
  Paint planning info (var and mission line) */
  bool paintObst;
  /**
  Paint vision based polygon */
  bool paintVisPoly;
  /**
  Distance between grid lines - in meter */
  double paintGridSize;
  /**
  paint grid (rectangular odo grid */
  bool paintGridOdo;
};

#endif
