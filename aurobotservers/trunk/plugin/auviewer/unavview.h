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

//#include <opencv/cxcore.h>
#include <umap4/upose.h>
#include <ugen4/uposrot.h>
#include <urob4/uvarpool.h>
#include <stdint.h>
#include <pcl/impl/point_types.hpp>

class UPaintManoeuvres;
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
class UPaintPolygons;
class UResPcp;
class UPaintPcp;

class UPaintBase
{
public:
  UPaintBase()
  {
    bold = false;
    maxHist = 30;
    cooSysSource = -1;
    cooSysView = -1;
  }
  /**
   * Paint robot at this post to this viewer. */
  virtual void paint(UPose * currentPose,  pcl::visualization::PCLVisualizer* viewer) {};
  /**
   * remove all items from this viewer. */
  virtual void unPaint(pcl::visualization::PCLVisualizer* viewer) {};
  /**
   * remove items from this viewer, to limit history to maximum this number. */
  virtual void unPaint(pcl::visualization::PCLVisualizer* viewer, int viewMaxHist) 
  { maxHist = viewMaxHist;  };
  /**
   * get name (e.g. of robot) */
  virtual const char * name() { return "noname"; };
  /**
   * is viewer this type */
  bool isA(const char * testName)
  { return strcasecmp(name(), testName) == 0; }; 
  /**
   * Set reference coordinate system
   * \param refSystem convert positions into this system [0 = odo, 1=utm, 2 = map] */
  virtual void setViewCooSys(int refSystem, UPose origin)
  { 
    cooSysView = refSystem; 
    cooSysViewOrigin = origin;
  };

  /**
   * Make circular cloud
   * \param cloud The cloud to modify, if empty, then resized to hold circle
   * \param pose  the pose 6D of the center of the circle, if pose is a unit matrix, then circle rotation axis is around x-axis
   * (on y-z plane) and center is in 0,0,0.
   * \param radius is radius of circle. */
  void makeCircleCloud(pcl::PointCloud<pcl::PointXYZ> * cloud, UMatrix4 * pose, double radius);
  /**
   * View a vertical circle (a wheel) touching the ground (z=0).
   * \param currentPose is current robot pose, relative to wich the circle is to be generated
   * \param viewer is the viewer to receive the data
   * \param cloud is the cloud to modify
   * \param x,y is the (ground) position of the wheel on the robot (x=forward, y=left)
   * \param h = rotation of the wheel axis: pi/2 for a left wheel.
   * \param radius is the wheel radius
   * \param id is then viewer ID of the cloud. */
  void addCircleCloud(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr * cloud, 
                             double x, double y, double h, double radius, const char * ID);
  /**
   * Add wheel, i.e a cylinder laying down, just touching the ground
   * \param viewer - the viewer
   * \param params a permanent available set of parameters (set and resized by the function)
   * \param x,y,h the position and orientation of the wheel, i.e. h=pi/2 is a left wheel
   * \param radius is the wheel radius in meters
   * \param width is the width of the wheel (in meters)
   * \param id is the unique id in the viewer of the wheel. */
  void addWheel(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer, 
                          pcl::ModelCoefficients * params,
                          double x, double y, double h, double radius, double width, const char * id);
  /**
   * add a solid box to the view
   * \param currentPose is robot pose
   * \param viewer is the viewer
   * \param params pointer to static set of box parameters. NB! params.values[7..9] must be set with box length (x), width (y) and height (z).
   * \param x,y,z is the box-center in robot coordinates.
   * \param id is the viewer ID (unique)
   * \param color[3] is the RGB color of the box */
  void addBox(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer, 
                             pcl::ModelCoefficients * params,
                             double x, double y, double z, 
                             const char * id, const double color[3]);
  /**
  * Vertical cylinder
  * \param currentPose is pose of robot
  * \param viewer is PCLviewer to show this
  * \param params is cylinder paramas (may be uninitialized - size 7)
  * \param x1,y1,z1 is center position on robot
  * \param x2,y2,z2 is center position on robot of other end
  * \param radius is radius of cylinder
  * \param id is unique id for this
  * \param color is surface color */
  void addCylinder(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer,
                          pcl::ModelCoefficients * params,
                          double x1, double y1, double z1,
                          double x2, double y2, double z2,
                          double radius, const char * id, const double color[3]);
  /**
   * Get copy of coordinate system number */
  const int getCooSysView()
  { return cooSysView; };
  /**
   * Get coordinates in viewed coordinate system 
   * \param[in] local is a pointer to the pose in local coordinates 
   * \param[in] cooSysLocal is number of coordinate system (0=odo 1=utm, 2=map)
   * \param[in] z is the value to put in the Z position 
   * \returns a pcl::PointXYZ value in viewed coordinates */
  pcl::PointXYZ getInViewedCoordinates(UPose * local, int cooSysLocal, double z)
  {
    UPose p = local;
    pcl::PointXYZ result;
    if (cooSysView != cooSysLocal)
      // need conversion
      p = cooSysViewOrigin.getPoseToMapPose(p);
    result.x = p.x;
    result.y = p.y;
    result.z = z;
    return result;
  };

protected:
  /**
   * known coordinates (source) is in this sytem (-1 is relative to robot) */
  int cooSysSource;
  /**
   * Origin of current system in view coordinate system */
  UPose cooSysViewOrigin;
  /**
   * coordinate system used in viewer [0=odo, 1=UTM, 2=map] */
  int cooSysView;
public:
  /**
   * Maximum history shown for this item, 0=do not show */
  int maxHist;
  /**
   * Is it to be bold */
  bool bold;
};

/**
Class to paint the navigation image

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UNavView
{
public:
  /**
  Constructor */
  UNavView();
  /**
  Destructor */
  ~UNavView();
  /**
  Set ressource as needed (probably not used by this resource) */
  bool setResource(UResBase * resource, bool remove);
  /**
  Paint laser data to this image-pool image */
  void paint();
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
  uint32_t getColor(char col);
  /**
   * Set the viewer to show the data
   * This call is not thread safe, so it should be set before first paint call.
   * \param newViewer must be an instanciated viewer. */
  void setViewer(pcl::visualization::PCLVisualizer* newViewer, double camPar[])
  { 
    viewer = newViewer;
    cmP = camPar;
  }
  /**
   * Save viewed image to a png-file 
   * \param filename is the name to use
   * \returns true if saved */
  bool saveAsPng(const char * filename);
  /**
   * Set view angle relative to robot
   * \param panAngle is view angle relative to robot heading
   * \param reset if true, then camera is moved to camResetPos relative to robot
   * \param camResetPos is a position relative to robot were the camera is placed - may be NULL when not reset
   * \param camResetFocus is the new focus position - relative to robot */
  void moveViewPose(double panAngle, bool reset, UPosition * camResetPos, UPosition * camResetFocus);


protected:
  /**
  Paint odometry reference grid around this pose (and remove old - if any) */
  void paintOdoGrid(UPose seenFromPose, double stepSize, double lastStepSize, bool bold);
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
  * Paint scans so that scanCount are painted, newest scan are painted in bold
  * \param scanClouds the scanners point existing clouds
  * \param scans are the raw scanner data (newest) */
  void paintScansNewest(UPaintBase * scanClouds, ULaserDataHistory * scans);

  /**
   * Paint all Pointclouds in point cloud pool (PCP)
   * \param posesys is the pose history for the current coordinate system */
  void paintPointClouds(UResPoseHist * posesys);

  /**
  Paint historic scans */
//  void paintHistScan(ULaserDataSet * scan, UPose seenFromPose);
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
  void paintOdoDataText();
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
  //void paintRobotMmr(UPose * currentPose);
  /**
  Paint a simple robot outline */
  //bool paintRobotSmr(double toFront);
  /**
  Paint a simple robot outline */
  //bool paintRobotHako(double toFront);
  /**
  Paint a simple robot outline */
  //bool paintRobotIrobot(double toFront);
  /**
  Paint a simple robot outline */
  //bool paintRobotGuidebot(double toFront);
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
   * paint the lines in the polyItem module */
  void paintPolyItems(UPoseTime seenFromPose);

protected:
  /**
  Viewer to show data */
  pcl::visualization::PCLVisualizer* viewer;
  /**
  Viewer to show data */
  double * cmP;
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
  /**
  Pointer to resource with laser scan data */
  UResPcp * resPcp;
  /**
   * robots */
  UPaintBase * robot;
  /// number of pose hists
  static const int PAINT_POSE_HIST_CNT = 3;
  /**
   * Pose history */
  UPaintBase * poseHist[PAINT_POSE_HIST_CNT];
  /**
   * Laser scans from one laser scanner */
  UPaintBase * laserClouds;
  /**
   * Polygons */
  UPaintPolygons * polyClouds;
  /**
   * Manoeuvre data */
  UPaintManoeuvres * manoeuvreClouds;

private:
  /** convert a position in one of the supported coordinate systems to a pixel position
  \param pos is the position to converted
  \param coordinateSystem is the ordinal number of the coordinate system of pos, 0=odo, 1= utm, 2=map.
  \param seenFromPose is the current position of the robot in current coordinate system
  \returns pixel position - limited to max 10000 pixels away from display origin */
  //CvPoint toPixels(UPosition pos, int coordinateSystem, UPose seenFromPose);

  /** convert a position in one of the supported coordinate systems to a pixel position
  \param pos is the position to converted
  \param seenFromPose is the current position of the robot in current coordinate system
  \param systemOrigin is the origin of the 'pos' position in the displayed coodinate system
  (returned by the getSystemOrigin(...) call)
  \param convert is a flag - if false no coordinate conversion is needed.
  (returned by the getSystemOrigin(...) call)
  \returns pixel position - limited to max 10000 pixels away from display origin */
  //CvPoint toPixels(UPosition pos, UPose seenFromPose, UPose systemOrigin, bool convert);
  /**
  Convert this position in displayed coordinate system to a pixel position
  \param pos is the position (only x,y is used).
  \returns a position to be used in paint call. */
  //CvPoint toPixels(UPosition pos);
  /**
  Convert this pose position in displayed coordinate system to a pixel position
  \param pose is the position (only x,y is used).
  \returns a position to be used in paint call. */
//   inline CvPoint toPixels(UPose pose)
//   {
//     UPosition pos(pose.x, pose.y);
//     return toPixels(pos);
//   };

public:
  /**
  Max range is the total height of the navigation window. */
  //double maxRange[2];
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
  /** hide these pcps (default hide none) */
  char paintPcpHide[maxStrLng];
  /** but show these these pcps among the hidden (default show all) */
  char paintPcpShow[maxStrLng];
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
  double paintGridSize[2];
  /**
   * Position around the last odometry grid */
  UPose paintGridPose;
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
  /// follow robot in viewer
  bool followRobot;
  /// camera focus point at last paint
  UPoseTime lastCamFocus;
  /// first time paint requires no remove from viewer
  bool firstTextScannumber;
  /// point cloud update status
  UPaintPcp ** paintPcls;
  /// number of paintpcps available
  int paintPclsMaxCnt;
  /// Pose of robot at last paint (and in last used coordinate system)
  UPose robotPoseLast;
  /// last used coordinate system
  int cooSysLast;
};



#endif
