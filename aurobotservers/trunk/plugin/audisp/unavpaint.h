/***************************************************************************
 *   Copyright (C) 2007 by Christian Andersen   *
 *   jca@oersted.dtu.dk   *
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
#ifndef UNAVPAINT_H
#define UNAVPAINT_H

#ifdef OPENCV2
#include <core/core_c.h>
#else
#include <opencv/cxcore.h>
#endif
#include <umap4/upose.h>
#include <ugen4/uposrot.h>
#include <urob4/uvarpool.h>

class UResLaserIfRoad;
class UObstacleHist;
class UResLaserIfObst;
class UImage;
class UResVarPool;
class UImagePool;
class UResBase;
class UResPoseHist;
class ULaserDataHistory;
class ULaserDataSet;
class UResCamIfPath;
class UObstacleGroup;
class UProbPoly;
class UFeaturePool;
class UResLaserIfScan;
class UResLaserIfSf;
class URoadLineData;
class UResNavIfMan;
class UClientManSeq;
class UResCamIfCam;
class UResCamIfGmk;
class UResPoly;

/**
Class to paint the navigation image

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UNavPaint
{
public:
  /**
  Constructor */
  UNavPaint();
  /**
  Destructor */
  ~UNavPaint();
  /**
  Set ressource as needed (probably not used by this resource) */
  bool setResource(UResBase * resource, bool remove);
  /**
  Paint laser data to this image-pool image */
  void paint(int imgPoolNum);
  /**
  Add or remove paint var structures */
  bool paintVarAdd(const char * name, bool add);
  /**
   * Set all coordinaate systems to the same spot (this) */
  void setRefSystemsHere();
  /**
  Print the coordinate references to console for all available conversions */
  void printRefSystems();
  /**
  Get a color value from a color character - mostly like color in scilab/matlab, i.e. 'r' is red, 'p' is pink */
  CvScalar getColor(char col);
  /**
  pan robot to this pixel position ob the display window
  \param pix is the new pixel position that should appear a centre of window.
  \param newScale is the new height of window in meters.
  \returns a new robotPose in screen coordinates (0,0 is bottom center - in meters)
  \returns a new maxRange to the newScale value. */
  void mousePan(CvPoint pix, double newScale);
  /**
  scale to show this pixel area
  \param pix is the position of the first mouse down click.
  \param dx if the movement in pixels relative to pos1 - in pixels. */
  void mouseScale(CvPoint pix, CvPoint dx);
  /**
  Get image that was last painted on */
  UImage * getImg()
  { return img; };

protected:
  /**
  Paint odometry reference grid */
  bool paintOdoGrid(UPose odoPose, double stepSize, bool bold);
  /**
  Paint laser scanner range rings
  \param sensorPos  is the 6d sensor pose relative to the robot.
  \param ringCnt is the number of range rings to paint. */
  bool paintRangeRings(UPosRot * sensorPos, int ringCnt);
  /**
  Paint scan line-fit variance curves (and other curves) in top-right corner */
  bool paintScanStatData(ULaserDataSet * scan,
                      bool paintVar,
                      bool paintVarL, bool paintTilt);
  /**
  Paint newest scan (big symbole) */
  void paintScanNewest(ULaserDataSet * scan);
  /**
  Paint historic scans */
  void paintHistScan(ULaserDataSet * scan, UPose seenFromPose);
  /**
  Paint pose history line
   * \param seenFromPose current pose in current coordinate system
   * \param path system path to be painted
   * \param colidx index number to paint colour (pt. 0=red, 1=cyan, 2=magenta)
   * \param convert should the path coordinates be converted to local system (true if conversion is needed)
   * \param systemOrigin origin of path coordinate system in current system. */
  void paintPoseHistLine(UPoseTime seenFromPose, UResPoseHist * path, int colidx, bool convert, UPose systemOrigin);
  /**
  Paint obstacles in this obstacle group */
  void paintObstGrp(UObstacleGroup *obst,
                    UPose seenFromPose, int grpIdx);
  /**
  Paint a vision based polygon */
  void paintFreePoly(UProbPoly * poly,
                    UPose seenFromPose,
                    bool historic);
  /**
  Paint current odometry position and time as text
  in bottom left corner */
  bool paintOdoDataText();
  /**
  Paint variables and values in top-left corner (and down) */
  bool paintVarDataText(bool paintAll);
  /**
  Paint passable intervals fount in this scan */
  void paintPis(ULaserDataSet * scan, UPose seenFromPose,
                           const int cnt);
  /**
  Paint simple features from a feature pool */
  void paintFeatures(UFeaturePool * featurePool, UPoseTime seenFromPose);
  /**
  Paint a simple robot outline */
  bool paintRobotMmr(double toFront);
  /**
  Paint a simple robot outline */
  bool paintRobotSmr(double toFront);
  /**
  Paint a simple robot outline */
  bool paintRobotHako(double toFront);
  /**
  Paint a simple robot outline */
  bool paintRobotIrobot(double toFront);
  /**
  Paint a simple robot outline */
  bool paintRobotGuidebot(double toFront);
  /**
  Paint a pose-symbol at this pixel position */
  void paintPose(UImage * img, int x, int y, double h, int size, CvScalar col, int lineWidth);
  /**
  Paint a cross symbol at this position */
  void paintCross(UImage * img, int x, int y, int size, CvScalar col, int lineWidth);
  /**
  Paint road line, as line segment and poly-line */
  void paintRoadLine(URoadLineData * road, UPoseTime seenFromPose, bool inGray);
  /**
  Paint one manoeuver sequence 'man', in res if the best, else in weak colors
  dependent in the 'num' parameter */
  bool paintManData(UClientManSeq * man, int num, UPoseTime seenFromPose);
  /**
  Paint known camera positions */
  void paintCams(UPoseTime seenFromPose);
  /**
  Paint known GMK positions */
  void paintGmks(UPoseTime seenFromPose);
  /**
   * Paint path lines for all available reference systems */
  void paintPoseHistLines(UPoseTime seenFromPose);
  /**
   * Get the origin of my coordinate system in the current paint coordinate syste.
   * if coordinate systems are not the same, then
   * the optional parameter 'convertNeeded' is set to true.
   * \param myOrigin is one of three values 0=odometry, 1=UTM, 2=map coordinates.
   * \param convertNeeded is set to false, if current paint
   * coordinates are the same as myOrigin.
   * \returns the position of 'myOrigin' in current paint coordinate system. */
  UPose getSystemOrigin(int myOrigin,
                        bool * convertNeeded);
  /**
   * paint the lines in the polyItem module
   * The color parameters '1234' are
   * d for default in all positions
   * [1] first position is color - like matlab r=red, p=pink, k=black, w=wheat, 1..9=grayscale
   * [2] is line width in pixels - default is 1
   * [3] if '*' is mark vertices with a circle - default is none
   * [4] if ' ' then edge line is not drawn - default is draw
   * \param seenFromPose is reference position fro drawing */
  void paintPolyItems(UPoseTime seenFromPose);

protected:
  /**
  Image to paint on */
  UImage * img;
  /**
  Pointer to obstacle data */
  UObstacleHist * obsts;
  /**
  Pointer to laser scanner obstacle resource */
  UResLaserIfObst * resObst;
  /**
  Pointer to road data */
  UResLaserIfRoad * road;
  /**
  Pointer to (global) var-pool */
  UResVarPool * varRoot;
  /**
  Image pool pointer */
  UImagePool * imgPool;
  /**
   * Maximum map reference systems (currently 3: odo(0), utm(1), map(2)) */
  static const int MAX_REF_SYS = 3;
  /**
  Pointer to recorded pose history - in odometry coordinates */
  UResPoseHist * poseOdo;
  /**
  Pointer to recorded pose history - in UTM coordinates */
  UResPoseHist * poseUtm;
  /**
  Pointer to recorded pose history - in Map coordinates */
  UResPoseHist * poseMap;
  /**
   * Pose reference conversion offset
   * First index is source second index is destination system */
  UPose poseToRef[MAX_REF_SYS][MAX_REF_SYS];
  /**
  Pointer to copy of basic laser scanner data */
  ULaserDataHistory * laserData;
  /**
  Pointer to resource with laser scan data */
  UResLaserIfScan * resLaserData;
  /**
  Pointer to vision based path */
  UResCamIfPath * camPath;
  /**
  Pool with simple features - as lines and points */
  UResLaserIfSf * laserFeatures;
  /**
  Navigation path resource from navigation server */
  UResNavIfMan * navMan;
  /**
  Pointer to pool of camera details */
  UResCamIfCam * camCam;
  /**
  Pointer to guidemark pool */
  UResCamIfGmk * camGmk;
  /**
   * pointer to polyItems */
  UResPoly * resPoly;

private:
  /** convert a position in one of the supported coordinate systems to a pixel position
  \param pos is the position to converted
  \param coordinateSystem is the ordinal number of the coordinate system of pos, 0=odo, 1= utm, 2=map.
  \param seenFromPose is the current position of the robot in current coordinate system
  \returns pixel position - limited to max 10000 pixels away from display origin */
  CvPoint toPixels(UPosition pos, int coordinateSystem, UPose seenFromPose);

  /** convert a position in one of the supported coordinate systems to a pixel position
  \param pos is the position to converted
  \param seenFromPose is the current position of the robot in current coordinate system
  \param systemOrigin is the origin of the 'pos' position in the displayed coodinate system
  (returned by the getSystemOrigin(...) call)
  \param convert is a flag - if false no coordinate conversion is needed.
  (returned by the getSystemOrigin(...) call)
  \returns pixel position - limited to max 10000 pixels away from display origin */
  CvPoint toPixels(UPosition pos, UPose seenFromPose, UPose systemOrigin, bool convert);
  /**
  Convert this position in displayed coordinate system to a pixel position
  \param pos is the position (only x,y is used).
  \returns a position to be used in paint call. */
  CvPoint toPixels(UPosition pos);
  /**
  Convert this pose position in displayed coordinate system to a pixel position
  \param pose is the position (only x,y is used).
  \returns a position to be used in paint call. */
  inline CvPoint toPixels(UPose pose)
  {
    UPosition pos(pose.x, pose.y);
    return toPixels(pos);
  };

public:
  /**
  Max range is the total height of the navigation window. */
  double maxRange;
  /**
  Robot position on image (in screen coordinates - meters above bottom-center of screen) */
  UPose robotPose;
  /**
  Paint EKF (GPS) data */
  bool paintGPS;
  /**
  Paint statistical curves */
  bool paintCurves;
  /**
  Paint all road lines - not just the primary newest road */
  int paintRoadAll;
  /**
  Paint road lines, if 0 then no roads are painted */
  int paintRoadHistCnt;
  /**
  Paint interval lines */
  bool paintIntervalLines;
  /**
  Paint path lines - alternative and used path */
  int paintPathLinesCnt;
  /**
  Paint tested (mid) poses for generation of this path */
  bool paintPathMidPoses;
  /**
  Paint tangent lines and no-visibility lines */
  bool paintPathSupportLines;
  /**
  Paint poly items - planned missions etc. */
  bool paintPoly;
  /**
  Paint name of poly items - at maximum this number of chars. */
  int paintPolyNameCnt;
  static const int maxStrLng = 100;
  /** hide these polygons (default hide none) */
  char paintPolyHide[maxStrLng];
  /** but show these these polygons among the hidden (default show all) */
  char paintPolyShow[maxStrLng];
  /**
  Paint path lines for all, also alternaive paths */
  //bool paintPathLinesAll;
  /**
  Number of poses in pose history to paint */
  int paintPoseHistCnt;
  /**
  Pose interval for for every heading vector paint (0 is not)  */
  int paintPoseHistVecCnt;
  /**
  Pose heading vector length in pixels  */
  int paintPoseHistVecLng;
  /**
  Number of scans to use in path hist */
  int paintScanHistCnt;
  /**
  Number of vision poygons to paint */
  int paintVisPolyCnt;
  /**
  Paint robot as: 0=mmr, 1=smr, 2=hako */
  int  paintRobot;
  /**
  Paint in bold (thicker bigger) */
  bool paintBold;
  /**
  Paint planning info (var and mission line) */
//  bool ;
  /**
  Number of obstacle groups (max) to paint */
  int paintObstCnt;
  /**
  Paint vision based polygon */
//  bool paintVisPoly;
  /**
  Distance between grid lines - in meter */
  double paintGridSize;
  /**
  paint grid (rectangular odo grid */
  bool paintGridOdo;
  /**
  Max number of painted variable structures */
  static const int PAINT_MAX_STRUCTS = 30;
  /**
  Paint variables from these structures */
  UVarPool * paintStructs[PAINT_MAX_STRUCTS];
  /**
  Number of used paint structures */
  int paintStructsCnt;
  /**
  Paint variables (in structures) - used to hide all */
  bool paintVar;
  /**
  Paint camera position */
  bool paintCam;
  /**
  Paint guidemarks */
  bool paintGmk;
  /**
   * Used pose reference 0=odoPose, 1=utmPose 2= mapPose */
  int paintPoseRef;
  /// paint odo-pose at bottom ov navigation display  - if odo pose plugin is available
  bool paintOdoPose;
  /// paint map-pose at bottom ov navigation display - if map pose plugin is available
  bool paintMapPose;
  /// paint utm-pose at bottom ov navigation display - if utm pose plugin is available
  bool paintUtmPose;
  /// number of range rings to paint.
  int rangeRingCnt;

private:
  /**
  Pixels per meter on display */
  double ppm;
  /**
  Pixel position of robot (heading is up) */
  CvPoint pr;
  /**
   * image 97name */
  static const int IMG97SIZE = 100;
  char img97name[IMG97SIZE];
};

#endif
