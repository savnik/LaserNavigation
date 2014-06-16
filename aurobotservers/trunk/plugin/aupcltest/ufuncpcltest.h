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
#ifndef UFUNC_PCLTest_H
#define UFUNC_PCLTest_H

#include <cstdlib>

#include <ucam4/ufunctioncambase.h>

//segment
#define BOOST_SYMBOL_VISIBLE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//down sample
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//viewer
//#include <pcl/visualization/cloud_viewer.h>
//rotation
#include <pcl/common/transforms.h>

// filter
#include <pcl/filters/passthrough.h>
// extracting plane points
#include <pcl/filters/extract_indices.h>

/*

#include <pcl/filters/radius_outlier_removal.h>
// remove plane
#include <pcl/filters/extract_indices.h>
*/
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
class UFuncPCLTest : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncPCLTest()
  {
    setCommand("pcltest", "pcltest", "kinect map generation (" __DATE__ " " __TIME__ ")");
    createBaseVar();
  };
  /**
  Destructor */
  virtual ~UFuncPCLTest();
  /**
  Handle in-comming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
  /**
  Create create globally  available status and configuration variables */
  void createBaseVar();
  /**
  Get kinect pointcloud (XYZ only) data from the image pool
  * \param[out] cloud pointer, where the cloud should be placed.
  * \returns the cloud unchanged if no data is available */
  void GetKinectPointCloudData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
  /**
  Get kinect pointcloud (XYZRGN) data from the image pool
  * \param[out] cloud pointer, where the cloud should be placed.
  * \returns the cloud unchanged if no data is available */
  void GetKinectPointCloudDataXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud );
  /**
  get the homogenious transform to rotate a plane to the Z axis */
  void GetRotate(pcl::ModelCoefficients::Ptr coefficients,Eigen::Matrix4f *RR);
  /**
  make a map */
  void CalcMapFromCloud(bool makePCD, const char * basefilename);
  /**
  convert cloud data to a map */
  void MakeMap(pcl::PointCloud<pcl::PointXYZ>::Ptr floor,pcl::PointCloud<pcl::PointXYZ>::Ptr obst,pcl::PointCloud<pcl::PointXYZ>::Ptr roof);
  /**
   * make filename from basename. if base filename starts with '.', '/' ot '~', then no pre-path is added,
   * else the files are placed in the dataPath path.
   * \param[out] fn is destination filename buffer
   * \param MFL is length of buffer
   * \param add is optional namepart to be added before last ".pcd"
   * \param basefilename is the desired base filename.
   * \returns a pointer to destination buffer. */
  const char * makeFilename(char * fn, int  MFL, const char * add, const char * basefilename);

private:


  /// resulting map limits
  UVariable * varMinMapX;
  UVariable * varMaxMapX;
  UVariable * varMinMapY;
  UVariable * varMaxMapY;
  // map square size note this is also double the resolution of the 3D data
  UVariable * varSquareSize;
  // after estimation of a plane, this is the height limit of what is part of the plane and what is not
  UVariable * varPlaneHeight;
  // anything above this height is ignored
  UVariable * varRobotHeight;

  // finished map
  UVariable * varKinectMapPool;

  // optimizing variables
  UVariable * varShowFloorPlane;
  UVariable * varShowRoof;


};


#endif

