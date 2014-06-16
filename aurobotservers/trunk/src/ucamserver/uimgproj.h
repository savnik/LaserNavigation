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
#ifndef UIMGPROJ_H
#define UIMGPROJ_H

#include <ugen4/u3d.h>
#include <ucam4/ucamrad.h>
#include <umap4/upose.h>
#include <umap4/uprobpoly.h>

#include <urob4/uimagepool.h>

/**
Project an image to a floor allined grid-map.

@author Christian Andersen
*/
class UImgProj{
public:
  /**
  Constructor */
  UImgProj();
  /**
  Destructor */
  ~UImgProj();
  /**
  Clear Color patch image */
  bool clearColMap();
  /**
  Clear Path image */
  bool clearPathMap();
  /**
  Get map position seen from robot */
  UPosition getMapToRob(UPosition mapPos);
  /**
  Get robot coordinate position seen from camera */
  UPosition getRobToCam(UPosition robPos);
  /**
  Get image coordinates from a camera coordinate */
  URPos getCamToImg(UPosition camPos);
  /**
  Convert a position (3d) as seen by robot and convert is to
  map (world) coordinates (3d).
  The rob pose is relative to the robot start pose. */
  UPosition getRobToMap(UPosition robCoordinate);
  /**
  Convert a position (3d) as seen by robot
  at position 'fromPose' and convert is to
  map (world) coordinates (3d).
  The 'fromPose' is relative to the robot start pose. */
  UPosition getRobToMap(UPose fromPose, UPosition robCoordinate);
  /**
  Convert a map 3D position to a 3D position in
  robot perspective. */
  UPosition getCamToRob(UPosition mapCoordinate);
  /**
  Convert a pixel position to a 3d position in the floor plane.
  The optional 'lookDown' parameter is false
  if image line is parallel to floor plane or
  points above the horizon.
  The restriction that the ray must be 2 cm below
  camera at 1 meter distance - e.g. max range is 50 m
  with a camera height of 1m. */
  UPosition getPixToRobFloor(int w, int h, bool * lookDown = NULL);
  /**
  Convert a pixel position to a 3d position in the floor plane.
  The optional 'lookDown' parameter is false
  if image line is parallel to floor plane or
  points above the horizon.
  The restriction that the ray must be 2 cm below
  camera at 1 meter distance - e.g. max range is 50 m
  with a camera height of 1m. */
  UPosition getPixToRobFloor(float w, float h, bool * lookDown = NULL);
  /**
  Convert a camera image pixel position to a position on the map.
  Requires valid camera position and pobot position (and
  camera intrincic inf too).
  Returns position, and flags if camera is looking down, i.e. below
  the horizon. */
  UPosition getPixToMapFloor(int w, int h, bool * lookDown = NULL);
  /**
  Get pixel position in source image 'img' for
  this (floor) position */
  URPos getFloorToPix(UPosition posfl);
  /**
  Set image buffers to hold grid-map images. */
  bool setImages(UImage * mapColorImage, UImage * mapPathImage);
  /**
  Get (and possibly create) color gridmap image */
  inline UImage * getMapColImg() { return mapCol; };
  /**
  Get (and possibly create) path gridmap image */
  inline UImage * getMapPathImg() { return mapPath; };
  /**
  Set cell size in meters for gridmap maps */
  inline void setMapCellSize(double cellSize)
     { mapCellSize = cellSize;};
  /**
  Get gridmap cell size */
  inline double getMapCellSize()
     { return mapCellSize;};
  /**
  Get robot pose */
  inline UPose getRobPose()
      { return robPose; };
  /**
  Set image 3d to image pixel position matrix */
  inline void setCamPar(UCamPar * camp)
    { camPar = *camp; };
  /**
  Set image 3d to image pixel position matrix */
  inline void setCamPar(UCamRad * camr)
    { camPar = *camr->getCamPar(); };
  /**
  Set Robot pose */
  inline void setRobPose(UPose robotPose)
    { robPose = robotPose; };
  /**
  Set start pose */
  inline void setStartPose(UPose pose)
    { startPose = pose; };
  /**
  Set Robot pose */
  inline void setRobPose(double x, double y, double heading)
    { robPose.set(x, y, heading); };
  /**
  Set start pose */
  inline void setStartPose(double x, double y, double heading)
    { startPose.set(x, y, heading); };
  /**
  Set camera position on robot. */
  void setCamPos(double x, double y, double z,
             double omega, double phi, double kappa);
  /**
  Set camera position on robot. */
  void setCamPos(UPosition pos,
                 URotation rot);
  /**
  Provect an imageset for present robot and camera position
  if mask value is > 0, then thecolor will be projected to
  the color map, and the masked position will be marked as
  passable in the path image.
  Returns true if projection is possible. */
  bool doProject(UImage * imColor, UImage * imPathMask);
  /**
  Make map based on polygon of passable floor.
  The result coordinates are in image pixels.
  The polygin is converted to robot floor perspective
  using camera position on the robot. Any points
  above the horizon are just removed (no interpolation), but
  'isObst'-flag is set to false if the point goet to or from
  an above horizon removed point.
  This function modifies source data to the new projection.<br>
  Returns true if pointers are non NULL */
  bool doProjectPolygonToMapPix(
            CvPoint * polygon,
            int * polygonCnt,
            bool * isObst);
  /**
  Convert image polygon to corresponding flat floor
  polygon in robot perspective, from known camera and canera position.
  Any points above the horizon are just removed (no interpolation), but
  'isObst'-flag is set to false if the point goet to or from
  an above horizon removed point.
  This function will not modify original polygon data.<br>
  Returns true if pointers are non NULL
  Returns points in UPosition array of maximum size polyRealCnt
  The number of points are returned into polyRealCnt. */
  bool doProjectPolygonToFloorReal(
                      CvPoint * polygon,
                      bool * isObst,
                      int polygonCnt, // number of items in source polygon
                      UProbPoly * destPoly);
/*                      UPosition * polyReal,
                      bool * isObstReal,
                      int * polyRealCnt);*/
  /**
  Make map based on polygon of passable floor. */
  bool doPolygonMap(UImage * imColor,
            UImage * imPathMask,
            CvPoint * polygon,
            int polygonCnt,
            bool * isObst);
  /**
  Get image name, time and pose.
  Returns true if found.
  Returns name in bufferImgName,
  and image time in imgTime. */
  bool getImageName(
              const char * logImg,
              const char * logPos,
              const char * subDir,
              int imgNum,
              char * bufferImgName,
              const int bufferSize,
              UTime * imgTime,
              int hdrMode,
              int device);
  /**
  Paint 1m grid in image */
  void paintGrid(UImage * img);
  /**
  Paint an arrow (wheel triangle) in red
  on mapCol.Conv3DToPixel(Returns true if map image exist)
  Paints the provided image number (at times).
  If image number is 0, then no number. */
  bool paintRobot(UImage * img, int imageNumber);
  /**
  Paint green line from present position to this position */
  bool paintRobotPath(UImage * img, UPose toPose);
  /**
  Paint the robot path from this odometer file */
  bool paintPath(const char * filename);
  /**
  Paint text in this image (black - large font) */
  void paintText(UImage * img, int x,int y, const char * text);
  /**
  Get pointer to result polygon in floor coordinates */
  inline UProbPoly * getFloorPoly()
    { return &floorPoly; };

private:
  /**
  Gridmap image with projected color */
  UImage * mapCol;
  /**
  Image with path-probability */
  UImage * mapPath;
  /**
  Pixel size in meter for map images. */
  double mapCellSize;
  /**
  Camera position on robot */
  UPosition camPos;
  /**
  Camera rotation on robot */
  URotation camRot;
  /**
  Robot position and orientation on map */
  UPose robPose;
  /**
  Where to start at the map */
  UPose startPose;
  /**
  Conversion from camera 3d position to image coordinate and
  reverse needs knowledge of camera parameters, so
  these must be loaded (copied) to this position. */
  UCamPar camPar;
  /**
  The found polygon in the newest image */
  UProbPoly floorPoly;
};

#endif
