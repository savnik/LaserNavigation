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
#ifndef URESOBJ3D_H
#define URESOBJ3D_H

#include <cstdlib>

#ifdef OPENCV2
#else
#include <opencv/cxcore.h>
#endif

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <ugen4/uimg3dpoint.h>
#include <urob4/ulogfile.h>
#include "uobj3dpool.h"

class UImg3Dpoints;
class UImg3Dpoint;
class UGridSegs;
class UGridBBoxes;

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

/**
 * a 3D voxel grid for 3D-blob estimation */
class UGrid3D
{
public:
  /**
   * constructor */
  UGrid3D();
  /**
   * Destructor */
  ~UGrid3D();
  /**
   * Clear all voxels */
  void clear();
  /**
   * Set volumen with empty voxels.
   * \param cellSize side size in meters - e.g 0.1 m.
   * \param depth x-size of volumen in meters - 0.0 is camera
   * \param width y-size of volumen in meters - 0.0 at centre.
   * \param height z-size in meters - lower limit is 0.0
   * \returns true if memory is allocated (or reallocated) */
  bool setVolumen(double cellSize, double depth, double width, double height);
  /**
   * Set voxel at this point to point to this image point.
   * \param plane is the ground plane, uset to calculate z-distance (height)
   * \param pnt the image point to place.
   */
  void setVoxel(UPlane * plane, UImg3Dpoint * pnt);
  /**
   * Get pointer to voxel cell (a pointer cell is a pointer too)
   * \param vx voxel number in depth direction
   * \param vy is voxel number left or right - left is positive
   * \param vz is voxel in the height direction.
   * \returns a pointer to the cell, if within voxel volumen. */
  UImg3Dpoint ** getVoxelPnt(int vx, int vy, int vz);
  /**
   * Count voxels with non-zero content
   * \returns count of voxels that is not NULL */
  int countNonZeroVoxels();
  /**
   * Segment the voxel volument in an 8 connectivity test
   * \param minZ is the minimum z that should be segmentated
   * \param maskImg is a bW image in same dimension as voxel planes if masked, then
   * voxel is not used for segmentation.
   * \param maskSideMult is multiplier from voxel x,y to mask r,c
   * \returns number of cluster segments */
  int voxelSegmentation(double minZ, UImage * maskImg, double maskSideMult);
  /**
   * Resolve the equvivalent numbers in this array, and
   * assign a cluster number to this voxel in return.
   * The equvivalens tabel should be updated. */
  int addEquvivalent(int * mee, int meeCnt);
  /**
   * Filter the found bounding boxes after size
   * \param minCellCnt minimum filled cells inside box
   * \param maxDX the maximum size of the human in z-direction - must be large due to stereo uncertanty
   * \param maxDY the maximum size of the human across the camera view
   * \param maxZ the maximum height of a human
   * \param minZ the smallest obstacle classified as a human
   * \param minX is the closest a human should be detected from the camera (false correlation remover)
   * \param roogHgt is the minimum height of the bottom of obstacles to be considered an obstacle, else it is assumed to be a part ot the roof or tree tops etc.
   * \param minDens is the mean density of voxels per square meter
   * \param minDensCnt minimum density is to be applied for boxes with up to this number of cells
   * \returns number of humans detected in view */
  int filter(int minCellCnt, double maxDX, double maxDY,
             double maxZ, double minZ, double minX,
             double roofHgt,
             double minDens, int minDensCnt);
  /**
   * Paint all vixels in top and side view
   * Paint a grid and the selected cells.
   * \param img is the image to paint into
   * \param minQValue paint only thise cells that has at least this q-value
   * \param andBBoxes paint also bounding boxes
   * \param minCellCnt paint only bounding boxes with at least this number of valid cells */
  void paintVoxels(UImage * img, int minQValue,
                   bool andBBoxes, int minCellCnt);
  /**
   * Get the position of the center of this cell (and the cell size)
   * \param ix,iy,iz is the index to the cell, all index are zero or positive
   * \param size is the size of the actual cell
   * \returns the center position in meters */
  UPosition getCellCenter(int ix, int iy, int iz, double * size);
  /**
   * Get the position of the Near Bottom Right of this cell (and the cell size)
   * \param ix,iy,iz is the index to the cell, all index are zero or positive
   * \param size is the size of the actual cell
   * \returns the center position in meters */
  UPosition getCellNBR(int ix, int iy, int iz, double * size);
  /**
   * Get the position of the Far Top Left of this cell (and the cell size)
   * \param ix,iy,iz is the index to the cell, all index are zero or positive
   * \param size is the size of the actual cell
   * \returns the center position in meters */
  UPosition getCellFTL(int ix, int iy, int iz, double * size);
  /**
   * log bounding boxes to this logfile
   * \param lf is a pointer to the logfile object
   * \param validOnly log valid bounding boxes only (from filter process) */
  void logBB(ULogFile * lf, bool validOnly);
  /**
   * Merge bounding boxes that has an overlap.
   * The search for overlap is done once only, so
   * there could be untested overlap of merged boxes. */
  int mergeOverlappingBBs();
  /**
   * Get bounding boxes */
  UGridBBoxes * getBoxes()
  { return boxes; };

protected:
  /**
   * Pointer to data area */
  UImg3Dpoint ** voxels;
  /**
   * Data area size (in pointers) */
  int voxelsCnt;
  /**
   * depth (x direction) of volumen in voxels*/
  int xDepth;
  /**
   * width (y-direction) of volumen in voxels */
  int yWidth;
  /**
   * height (z-direction) of volumen in voxels */
  int zHeight;
  /**
   * Side length of voxel in meters. */
  double voxelSize;
  /**
   * segmentation collisions */
  UGridSegs * segc;
  /**
   * Bounding box list */
  UGridBBoxes * boxes;

};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResObj3d : public UObj3dPool, public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLine) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResObj3d()
  { // set name and version number
    setResID("obj3d", 203);
    // other local initializations
    UResObj3dInit();
  }
  ;
  /**
  Destructor */
  virtual ~UResObj3d();
  /**
  print status to a string buffer */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  print status to a string buffer */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint( preString, buff, buffCnt); };

// the above methods are used by the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
  * Add special variable parameters for obstacle management */
  void addObstacleParameters();
  /**
  * Load parameter values from current settings pool to this obstacle group */
  virtual void getObstacleGroupSettings(UObj3dGroup * og);
  /**
  Obstacle data is updated - tell resource */
  virtual void obstDataUpdated(UTime poseTime);
  /**
   * Get fresh 3D cloud data from svs system and update the object pool as needed.
   * \param getAny takes any data from SVS, if false, then only new data (based on serial number)
   * \returns true if processing were successfull - source data is available) */
  bool do3dCloudFromSvs(UImg3Dpoints * cloud);
  /**
   * \brief find candidates for human detections.
   * Estimates the ground plane from 3D cloud. Isolates 3D voxel volumens, and filter human sized volumens.
   * \param cloud the 3D cloud to be processed.
   * \param justHuman save only those obstacles that are classified human (else all obstacles)
   * \param doImg if true, then more debug images are produced.
   * \returns true if human voxels were found. The human candidates are saved in the class. */
  bool do3dVoxels(UImg3Dpoints * cloud, bool justHuman, bool doImg);
  /**
   * \brief find candidates for human detections.
   * Estimates the ground plane from 3D cloud.
   * \param cloud the 3D cloud to be processed.
   * \param doGndEdgeObst make obstacles from edge of ground polygone
   * \param doImg if true, then more debug images are produced.
   * \returns true if a plane were found. The plane and quality is saved in class. */
  bool do3dGroundPlane(UImg3Dpoints * cloud, bool doGndEdgeObst, bool doImg);
  /**
   * \brief Get the newest point cloud from svs.
   * The cloud is actually stored in the svs module, and a pointer the coud is provided only.
   * Remember to lock it while processing and release after use.
   * \param getAny if false, then an imageset with a new serial number is accepted, else any serial number will do
   * \param cloud is where the function returns a pointer to the 3D point cloud, if any is found.
   * \returns true if a cloud is found. */
  bool get3dCloudFromSvs(bool getAny, UImg3Dpoints ** cloud);
  /**
   * \brief Get the point cloud from ing3d cloud source.
   * The cloud is actually stored in the svs module, and a pointer the coud is provided only.
   * Remember to lock it while processing and release after use.
   * \param source is a string with the plug-in name of the source, i.e. a 'source'.get3d() method call
   * must be available (see var 'source').
   * \param cloud is where the function returns a pointer to the 3D point cloud, if any is found.
   * \returns true if a cloud is found. */
  bool get3dCloud(const char * source, UImg3Dpoints ** cloud);
  /**
   * \brief get estimated ground plane
   * \returns the estimated plane in robot coordinates */
  UPlane getGroundPlane()
  { return gndPlaneR; };
  /**
   * \brief get the quality of the ground-plane
   * Quality is the relative count of ponts in the ground plane, relative to all points
   * \returns the quality figure. */
  double getGroundPlaneQuality()
  { return gndPlaneQ; };
  /**
   * Paint ground plane in-laiers, and do the needed filtering
   * for use as visible ground cells
   * \param maskImgNum is the image number in inage pool for result mask image
   * \param plane is the best estimated ground plane.
   * \param cloud is the clouds to include in the mask
   * \param cellSize is the image cell sixe
   * \param planeDist is the maximum distance from ground-plane to cloud-point to put in mask
   * \param erode erode one more than dilate
   * \param dilate dilate and erode (once)
   * */
  void  doGndPlaneMask(int maskImgNum, UPlane plane, UImg3Dpoints * cloud,
                       double cellSize,
                       double planeDist, bool erode, bool dilate);

protected:

  /**
   * Extract the ground plane polygon object */
  bool addGroundObjects(UImg3Dpoints * cloud,
                        double minZ, double maxZ, double maxX, double maxSep);
  /**
   * Test routine to segment patches of 3D detection */
  int findContour(UImage * imgMask, UImage * dest,
                          CvPoint * flp, int flpMax,
                          bool * tooHigh, bool * tooSmall, bool lookingLeft);
  /**
  Count edges in contour sequebnce */
  int countContourEdges(CvSeq * contours, int * largest);
  /**
  Find square limits of found contour
  returns the point cont, and
  if limit pointers != NULL the left, right, top and
  bottol limits of the points in the sequence. */
  int countContourLimits(CvSeq * contours, int * left,
                         int * right, int * top, int * bottom, bool lookingLeft);
  /**
   * \brief Extract polygons for z-slizes in this image
   * \param img is the source BW image
   * \param imgD is a potential debug image where to paint the polygon
   * \param ps is the original slized 3D point cloud
   * \param psCnt is the number of points in the source point cloud
   * \param pd is the destination array for filtered polygon
   * \param pdCnt is the allocated number of elements in that polygon
   * \param scale is the number of pixels in source image for each pixel in profile image
   * \returns number of polygon points in pd. */
  int findPolygons(UImage * img, UImage * imgD,
                    UImg3Dpoint ** ps, const int psCnt,
                  int scale);
  /**
   * \brief Extract just one polygon
   * \param block is the structure to be converted
   * \param ps is the original slized 3D point cloud
   * \param psCnt is the number of points in the source point cloud
   * \param pd is the destination array for filtered polygon
   * \param pdCnt is the allocated number of elements in pd destination array
   * \param scale is the number of pixels in source image for each pixel in profile image
   * \returns number of found points in pd array */
  int findPolygonsOne(CvSeqBlock * block,
                       UImg3Dpoint ** ps, const int psCnt,
                       UImg3Dpoint ** pd, const int pdCnt,
                     const int maxPoints,
                     int scale);
  /**
   * \brief Extract ground edge polygons, as obstacles. The polygons are returned in gnds[gndsCnt]
   * The method is to convert edges on top line of the polygon in 'block' into obstacles,
   * if thay are not on the left or right edge, and not further away than the
   * distance limit (distLimit)
   * \param block is the structure to be converted
   * \param ps is the original slized 3D point cloud
   * \param psCnt is the number of points in the source point cloud
   * \param scale is the number of pixels in source image for each pixel in profile image
   * \param leftEdge is the left edge of the obstacle area (visibility limit)
   * \param rightEdge is the right edge of the obstacle area (visibility limit)
   * \param distLimit is the distance limit "visibility limit"
   * \returns number of found obstacle polygons in gnds */
  int findPolygonsOnEdge(CvSeqBlock * block,
                         UImg3Dpoint ** ps, const int psCnt,
                         int scale, int leftEdge, int rightEdge, int distLimit );
  /**
   * Paint a polyline in this image from these points */
  void paintPolygonInImage(UImage * img, UImg3Dpoint ** pd, const int pdCnt);
  /**
   * \brief get most distant vertex from line between endpoints
   * \param pnts is first element in array of (pixel) points
   * \param pntsCnt is number of elements to be investigated
   * \param idx is pointer, where index of most distant vertex is to be returned.
   * \returns distance (signed) of most distannt vertex to the line between endpoints. */
  float findMostDistantVertex(const CvPoint pnts[],
                              const int pntsCnt,
                              int * idx);
  /**
   * */
  void approximatePolyLine(const CvPoint pnts[], // point array to test - both ends are included
                             bool pntOK[],         // array of boolean to mark used points
                             const int pntsCnt,    // number of points to test
                             const float toll);     // acceptable distance variation
  /**
   * Paint all 3D points in the ground plane in xz view and in xy view
   * \param imgNum is the image pool image number to use
   * \param cloud is the cloud of 3D points
   * \param maxX is the maximum X value in image
   * \param maxZ is the maximum Z distance from ground plane to paint
   * \param andBoxes should bounding boxes be painted too
   * \param andGndPolygons paint also the polygons in the gnsd array */
  void paintGndPlane(int imgNum, UImg3Dpoints * cloud, double maxX, double maxZ,
                     bool andBoxes, bool andGndPolygons);
  /**
   * Find ground polygons using an image (profile) projection of the
   * pixel inliers for the estimated ground plane,
   * \param inum1 is the image pool number to be used for the source image
   * \param inum2 is the other image pool number to use for interim data
   * \param cloudPts is the inliers close to the ground plane
   * \param cloudPtsCnt is the number of points
   * \param scale is the pixel size if the filter image for each source image.
   * \returns the image with the ground plane profile in camera coordinates */
  UImage * makeGroundPlaneProfileImage(int inum1, int inum2,
                       UImg3Dpoint** cloudPts, int cloudPtsCnt,
                      int scale);
  /**
   * Find largest blob in this image using openCV segmentation.
   * \param img is the 8-bit gray scale image, where contours with value >0 searched.
   * \param leftEdge will retruen the image left edge of the largest polygon if not NULL
   * \param rightEdge will retruen the image right edge of the largest polygon if not NULL
   * \returns a pointer to the largest contour block. */
  CvSeqBlock * findPolygonLargest(UImage * img, int * leftEdge, int * rightEdge);

private:
  /**
   * Remaining initialization */
  void UResObj3dInit();
  /**
   * Get area of a openCV CvSeqBlk structure.
   * The method is (sum(x1*y2 - x2*y1))/2 for all points and the first as the last.
   * Calculating in integer space.
   * \param block is the block in question
   * \returns the area (in integer) */
  int getBlockArea(CvSeqBlock * block, int * leftEdge, int * rightEdge);

protected:
  /**
   * Lase serial number for image (svs) source */
  unsigned int lastSvsSerial;
    /**
  Storage used by openCV */
  CvMemStorage * storage;
  /**
   * Storage for detected obstacles */
  UObj3dPool * pool;
  static const int MAX_GND_POLYGONS = 50;
  /**
   * array of polygons */
  UPolygon40 * gnds[MAX_GND_POLYGONS];
  /**
   * Number of actual polygons */
  int gndsCnt;
  /**
   * best groundplane found in camera coordinates */
  UPlane gndPlaneC;
  /**
   * best groundplane found in robot coordinates */
  UPlane gndPlaneR;
  /**
   * detection time for current ground plane estimate */
  UTime gndPlaneTime;
  /**
   * quality - percentage of pixels in gndPlane */
  double gndPlaneQ;
  /**
   * human logfile */
  ULogFile loghum;
  /**
   * voxel grid for filtering in 3D image */
  UGrid3D * grid;
  // class variables
  /**  Index to variable in variable pool for fast access - max pose distance in one group  */
  UVariable * varGrpMaxDist;
  /**  Index to variable in variable pool for fast access - max time spend in one group */
  UVariable * varGrpMaxTime;
  /**  Index to variable in variable pool for fast access - obstacle merge distance (outdoor) */
  UVariable * varCombineDist;
  /**  Index to variable in variable pool for fast access - should obstacles actually be merged (mostly for debug if false) */
  UVariable * varObstMerge;
  /** Time (tod) of last obstacle update */
  UVariable * varUpdateTime; //
  /** number of obstacle groups in pool */
  UVariable * varGrps; //
  /** newest obstacle group */
  UVariable * varGroup; //
  /// index to minimum Z for gorund finding
  UVariable * varGndMinZ;
  /// index to maximum Z for gorund finding
  UVariable * varGndMaxZ;
  /// minimum quality (share of inliers relative to all) to use plane estamate
  UVariable * varGndMinQ;
  /// index to maximum usable 3D distance
  UVariable * varDistMaxX;
  /// point reduction factor for ground estimate (to save time) def = 3
  UVariable * varGndRedFac;
  /////////////////
  /// voxel cell size (in meter)
  UVariable * varGridSize;
  /// should bounding boxex be allowed to merge
  UVariable * varBBmerge;
  /// box finder parameter maximum X extend
  UVariable * varBBmaxDX; // "(rw) max x extend for human finder");
  /// max box (human) filter y extend
  UVariable * varBBmaxDY; // "(rw) max y extend (width) for human finder");
  /// maximum height for human finder
  UVariable * varBBmaxZ; // "(rw) max height for human finder");
  /// humans lower than this are ignored
  UVariable * varBBminZ; // "(rw) min height for human finder");
  /// humans closer to camera than this is assumed an error
  UVariable * varBBminX; // "(rw) min dist for human");
  /// minimum voxel cound
  UVariable * varBBminVoxCnt;
  /// minimum voxel density per footprint square meter
  UVariable * varMinDens;
  /// minimum voxel density to be tested for voxel blobs with up to this number of cells
  UVariable * varMinDensCnt;
  /// minimum height for roof obstacles
  UVariable * varMinRoofHgt;
  /// should ground filter on voxels be used
  UVariable * varGndFilter;
  /// filter cell size, compared to voxed cell size (0.5 gives 4 times as many cellse)
  UVariable * varGndFilterCell;
  /// should ground filter on voxels be eroded 1 more than dilate
  UVariable * varGndFilterErode;
  /// should ground filter on voxels be dilate-eroded 
  UVariable * varGndFilterDilate;
  /// number of pixels per ground polygin image pixel before segmentation
  UVariable * varGndPolyScale;
};

#endif

