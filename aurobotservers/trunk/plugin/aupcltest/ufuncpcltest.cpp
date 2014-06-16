/***************************************************************************
 *   Copyright (C) 2012 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 01 2012 Modified by Kristian Villien                               *
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
 *
 * $Id: ufuncpcltest.cpp 59 2012-10-21 06:25:02Z jcan $
 * $Rev: 59 $
 ***************************************************************************/

#include <urob4/usmltag.h>
#include <sys/time.h>

#include "ufuncpcltest.h"

#define UIMGPOOL_BALL_BASE 45

#ifdef LIBRARY_OPEN_NEEDED

UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncPCLTest' with your classname, as used in the headerfile */
  return new UFuncPCLTest();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncPCLTest::~UFuncPCLTest()
{ // possibly remove allocated variables here - if needed
  printf("PCLTest unloaded\n");
}


///////////////////////////////////////////////////

bool UFuncPCLTest::handleCommand(UServerInMsg * msg, void * extra)
{ // handle command(s) send to this plug-in
  bool ask4help;
  int camDevice = -1;
  int imgPoolNum = -1;
  USmlTag tag;
  bool result;
  bool makePcd = false; // default is debug on
  char pcdFilename[MAX_FILENAME_SIZE] = "cloud.pcd"; // default is <ball ...> reply
  bool gotBlue = false;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", NULL, 0);
  if (not ask4help)
  { // get all other parameters
    msg->tag.getAttValueInt("device", &camDevice);
    msg->tag.getAttValueInt("img", &imgPoolNum);
    msg->tag.getAttValueBool("pcd", &makePcd, false);
    msg->tag.getAttValue("filename", pcdFilename, MAX_FILENAME_SIZE);
    msg->tag.getAttValueBool("blue", &gotBlue, true);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("pcltest");
    sendText("--- available PCLTEST options\n");
    sendText("map               Get a point cloud from kinect and make a map of oblects with floor removed");
    sendText("raw[=M]           Get a pointcloud (M=0 XYZ, M=1 XYZRGB (default)) from kinect");
    sendText("file[=filename]   Save cloud to PCD file");
    sendText("help              This message\n");
    sendText("---\n");
//    sendText("See also: VAR BALL for other parameters and results\n");
    sendHelpDone();
    sendInfo("done");
    result = true;
  }
  else
  { // not help, so - first - get source image
    CalcMapFromCloud(makePcd, pcdFilename);
    printf("map created\n");
    result = true;
  }
  // return true if the function is handled with a positive result
  return result;
}

////////////////////////////////////////////////////////////

void UFuncPCLTest::createBaseVar()
{
  /// resulting map limits
  varMinMapX = addVar("MinMapX", -3.0, "d", "Lower X limit of the map");
  varMaxMapX = addVar("MaxMapX", 3.0, "d", "Upper X limit of the map");
  varMinMapY = addVar("MinMapY", 0.0, "d", "Lower Y limit of the map");
  varMaxMapY = addVar("MaxMapY", 6.0, "d", "Upper Y limit of the map");
  // map square size note this is also double the resolution of the 3D data
  varSquareSize = addVar("SquareSize", 0.01, "d", "map square size note this is also double the resolution of the 3D data ");
  // after estimation of a plane, this is the height limit of what is part of the plane and what is not
  varPlaneHeight = addVar("PlaneHeight", 0.02, "d", "After estimation of a plane, this is the height limit of what is part of the plane and what is not");
  // anything above this height is ignored
  varRobotHeight = addVar("RobotHeight", 0.45, "d", "Anything above this height is ignored");
  // finished map
  varKinectMapPool = addVar("KinectMapPool", 30, "d", "The image pool number where the resulting map is stored.");

  // optimizing variables
  varShowFloorPlane = addVar("ShowFloorPlane", 1, "d", "When this is set to 0 the floor plane will not be show on the final map, making the process faster.");
  varShowRoof = addVar("ShowRoof", 1, "d", "When this is set to 0 anything above the robot will not be show on the final map, making the process faster.");


}

//////////////////////////////////////////////

void UFuncPCLTest::GetKinectPointCloudData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{

 const int MPC = 2; // parameter count
 double pars[MPC], v;
 bool isOK;
 int n = 1;
 pars[0] = 0;
 pars[1] = -1;
 isOK = callGlobal("kinect.GetPointCloud", "dd", NULL, pars, &v, (UDataBase**) &cloud, &n);
 if (not isOK)
   printf("*** not found error: no kinect.GetPointCloud(d,d)\n");
}

////////////////////////////////////////////////


void UFuncPCLTest::GetKinectPointCloudDataXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
{

 const int MPC = 2; // parameter count
 double pars[MPC], v;
 bool isOK;
 int n = 1;
 pars[0] = 1;
 pars[1] = -1;
 isOK = callGlobal("kinect.GetPointCloud", "dd", NULL, pars, &v, (UDataBase**) &cloud, &n);
 if (not isOK)
   printf("*** not found error: no kinect.GetPointCloud(d,d)\n");
}

///////////////////////////////////////////////////////////////////////////////
void UFuncPCLTest::GetRotate(pcl::ModelCoefficients::Ptr coefficients,Eigen::Matrix4f *RR)
{
  // finding the rotation needed to place the ground plane at the ground with Z axis going up
  float P1[3];
  P1[0] = coefficients->values[0];
  P1[1] = coefficients->values[1];
  P1[2] = coefficients->values[2];
  float H1[3][3];
  //make rotation matrix to remove the Y plane part
  H1[0][0] = 1;
  H1[0][1] = 0;
  H1[0][2] = 0;
  H1[1][0] = 0;
  H1[1][1] = P1[2]/sqrt(P1[1]*P1[1] + P1[2]*P1[2]);
  H1[1][2] = -sqrt(1-H1[1][1]*H1[1][1]);
  H1[2][0] = 0;
  H1[2][1] = -H1[1][2];
  H1[2][2] = H1[1][1];
  //get result
  float P2[4];
  P2[0] = H1[0][0]*P1[0]+H1[0][1]*P1[1]+H1[0][2]*P1[2];
  P2[1] = H1[1][0]*P1[0]+H1[1][1]*P1[1]+H1[1][2]*P1[2];
  P2[2] = H1[2][0]*P1[0]+H1[2][1]*P1[1]+H1[2][2]*P1[2];
  float H2[3][3];
  H2[0][0] = -P2[2]/sqrt(P2[0]*P2[0] + P2[2]*P2[2]);;
  H2[0][1] = 0;
  H2[0][2] = sqrt(1-H2[0][0]*H2[0][0]);;
  H2[1][0] = 0;
  H2[1][1] = 1;
  H2[1][2] = 0;
  H2[2][0] = -H2[0][2];
  H2[2][1] = 0;
  H2[2][2] = H2[0][0];
  // combine rotational matrixs
  // transfer to construkt
  (*RR)(0,0) = -(H1[0][0]*H2[0][0] + H1[1][0]*H2[0][1] + H1[2][0]*H2[0][2]);
  (*RR)(0,1) = -(H1[0][1]*H2[0][0] + H1[1][1]*H2[0][1] + H1[2][1]*H2[0][2]);
  (*RR)(0,2) = -(H1[0][2]*H2[0][0] + H1[1][2]*H2[0][1] + H1[2][2]*H2[0][2]);
  (*RR)(1,0) = -(H1[0][0]*H2[1][0] + H1[1][0]*H2[1][1] + H1[2][0]*H2[1][2]);
  (*RR)(1,1) = -(H1[0][1]*H2[1][0] + H1[1][1]*H2[1][1] + H1[2][1]*H2[1][2]);
  (*RR)(1,2) = -(H1[0][2]*H2[1][0] + H1[1][2]*H2[1][1] + H1[2][2]*H2[1][2]);
  (*RR)(2,0) = H1[0][0]*H2[2][0] + H1[1][0]*H2[2][1] + H1[2][0]*H2[2][2];
  (*RR)(2,1) = H1[0][1]*H2[2][0] + H1[1][1]*H2[2][1] + H1[2][1]*H2[2][2];
  (*RR)(2,2) = H1[0][2]*H2[2][0] + H1[1][2]*H2[2][1] + H1[2][2]*H2[2][2];
  (*RR)(3,0) = 0;
  (*RR)(3,1) = 0;
  (*RR)(3,2) = 0;
  (*RR)(3,3) = 1;
  (*RR)(0,3) = 0;
  (*RR)(1,3) = 0;
  (*RR)(2,3) = -coefficients->values[3];
}

/////////////////////////////////////////////////////////////////////////////////

const char * UFuncPCLTest::makeFilename(char * fn, int  MFL, const char * add, const char * basefilename)
{
  const char * p1;
  char * p2;
  int n = 0;
  p2 = fn;
  if (basefilename[0] == '.' or basefilename[0] == '/' or basefilename[0] == '~')
    strncpy(fn, basefilename, MFL);
  else
  {
    strncpy(fn, dataPath, MFL);
    n = strlen(fn);
    p2 = &fn[n];
  }
  p1 = strrchr(basefilename, '.');
  strncpy(p2, basefilename, MFL - n);
  if (p1 != NULL)
  {
    n += p1 - basefilename;
    p2 = &fn[n];
    snprintf(p2, MFL - n, "%s%s", add, p1);
  }
  else
  {
    snprintf(p2, MFL - n, "%s.pcd", add);
  }
  return fn;
}


void UFuncPCLTest::CalcMapFromCloud(bool makePCD, const char * basefilename)
{
  timeval TimeStart, TimeGet, TimeFindPlane, TimeRotate, TimeSort, TimeMap;
  double dTimeStart, dTimeGet, dTimeFindPlane, dTimeRotate, dTimeSort, dTimeMap;
  const int MFL = MAX_FILENAME_SIZE;
  char fn[MFL];
       // find floor plane
       //rotate cloud to put floorplane on Z plane
       // floor plane is all with Z less than 1cm(?)
       // obstickle is all remaining with Z less than 40 cm
       // roof is all remaining
//////////////////////////////////////////////
// init point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // original cloud
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prefiltered (new pcl::PointCloud<pcl::PointXYZ>); // thinned cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); // thinned cloud
//        cloud_filtered->width  = 480*640;
//        cloud_filtered->height = 1;
//        cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Rot (new pcl::PointCloud<pcl::PointXYZ>); // rotated cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor (new pcl::PointCloud<pcl::PointXYZ>); //floorplane cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obst (new pcl::PointCloud<pcl::PointXYZ>); //obstickle cloud
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obst_filt (new pcl::PointCloud<pcl::PointXYZ>); //obstickle cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roof (new pcl::PointCloud<pcl::PointXYZ>); //obstickle cloud

  pcl::PassThrough<pcl::PointXYZ> pass;
//////////////////////////////////////////////
// load point cloud file
  gettimeofday(&TimeStart,NULL);
  std::cerr << "loading point cloud data file..." << std::endl;
  GetKinectPointCloudData(cloud);
  gettimeofday(&TimeGet,NULL);
  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

//////////////////////////////////////////////
// filter cloud
// std::cerr << "Thinning cloud data. gridsize:" << varPlaneHeight->getDouble()/2 << "..." << std::endl;
// pcl::VoxelGrid<pcl::PointXYZ> sor;
// sor.setInputCloud (cloud);
// sor.setLeafSize (varPlaneHeight->getDouble()/2, varPlaneHeight->getDouble()/2, varPlaneHeight->getDouble()/2);
// sor.filter (*cloud_filtered);

// std::cerr << "Thinned point cloud data: " <<
//  cloud_filtered->points.size () << " points" << std::endl;
//pcl::visualization::CloudViewer viewer ("data Viewer");
//  viewer.showCloud (cloud_filtered);
//  while (!viewer.wasStopped ());
//////////////////////////////////////////////
// find plane
 std::cerr << "Finding floor plane (biggest plane)..." << std::endl;
 pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
 pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 // Create the segmentation object
 pcl::SACSegmentation<pcl::PointXYZ> seg;

 // Optional
 seg.setOptimizeCoefficients (true);
 // Mandatory
 seg.setModelType (pcl::SACMODEL_PLANE);
 seg.setMethodType (pcl::SAC_RANSAC);
 seg.setDistanceThreshold (varPlaneHeight->getDouble());
 seg.setInputCloud (cloud->makeShared());
 seg.segment (*inliers, *coefficients);

 if (inliers->indices.size () == 0)
 {
   PCL_ERROR ("Could not estimate a planar model for the given dataset.");
   return;
 }

 std::cerr << "Plane found. " << "Plane coefficients: " << coefficients->values[0] << " "
                                     << coefficients->values[1] << " "
                                     << coefficients->values[2] << " "
                                     << coefficients->values[3] << std::endl;

 std::cerr << "Points in plane: " << inliers->indices.size () << std::endl;
//////////////////////////////////////////////
 // rotating plane
 std::cerr << "Finding plane rotation..." << std::endl;
 Eigen::Matrix4f RR;
 RR << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
 GetRotate(coefficients,&RR);
 std::cerr << "Here is the matrix:\n" << RR << std::endl;
  gettimeofday(&TimeFindPlane,NULL);
//rotate cloud
 if(!varShowFloorPlane->getBool())
 {
   std::cout << "removing floor from data..." << std::endl;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_filtered);
   pcl::transformPointCloud(*cloud_filtered, *cloud_Rot, RR);
   std::cerr << "removed. could size now" << cloud_filtered->points.size () << std::endl;
 }
 else
   pcl::transformPointCloud(*cloud, *cloud_Rot, RR);
 std::cerr << "Plane rotated!" << std::endl;
 gettimeofday(&TimeRotate,NULL);
 //
 pcl::io::savePCDFileASCII<pcl::PointXYZ>(makeFilename(fn, MFL, "rotated", basefilename), *cloud_Rot);
//  RotViewer.showCloud (cloud_Rot);
///////////////////////////////////////////////
// filter out floor
 pass.setInputCloud (cloud_Rot);
 pass.setFilterFieldName ("z");
 pass.setFilterLimitsNegative (false);
 if(varShowFloorPlane->getBool())
 {
   pass.setFilterLimits (-1.0, varPlaneHeight->getDouble());
   pass.filter (*cloud_floor);
   std::cerr << "floorplane cloud data: " << cloud_floor->points.size() << " points" << std::endl;
  }
///////////////////////////////////////////////
// filter out obst
 pass.setFilterLimits (varPlaneHeight->getDouble(), varRobotHeight->getDouble());
 pass.filter (*cloud_obst);
 std::cerr << "Obst cloud data: " << cloud_obst->points.size () << " points" << std::endl;
///////////////////////////////////////////////
// filter out roof
 if(varShowRoof->getBool())
 {
   pass.setFilterLimits (varRobotHeight->getDouble(), 5);
   pass.filter (*cloud_roof);
   std::cerr << "Roof cloud data: " << cloud_roof->points.size () << " points" << std::endl;
  }
  gettimeofday(&TimeSort,NULL);
///////////////////////////////////////////////
  MakeMap(cloud_floor,cloud_obst,cloud_roof);
  gettimeofday(&TimeMap,NULL);


///////////////////////////////////////////////
// timing
//  timeval TimeStart, TimeGet, TimeFindPlane, TimeRotate, TimeSort, TimeMap;
//  double dTimeStart, dTimeGet, dTimeFindPlane, dTimeRotate, dTimeSort, dTimeMap;


dTimeStart = ((double)TimeStart.tv_sec)+((double)TimeStart.tv_usec)/1000000;
dTimeGet = ((double)TimeGet.tv_sec)+((double)TimeGet.tv_usec)/1000000;
dTimeFindPlane = ((double)TimeFindPlane.tv_sec)+((double)TimeFindPlane.tv_usec)/1000000;
dTimeRotate = ((double)TimeRotate.tv_sec)+((double)TimeRotate.tv_usec)/1000000;
dTimeSort = ((double)TimeSort.tv_sec)+((double)TimeSort.tv_usec)/1000000;
dTimeMap = ((double)TimeMap.tv_sec)+((double)TimeMap.tv_usec)/1000000;
printf("time: Total ; Getting data ; Finding Plane ; Rotateing ; Sorting ; Mapping = [%f ; %f ; %f ; %f ; %f ; %f] ",dTimeMap-dTimeStart, dTimeGet-dTimeStart,dTimeFindPlane-dTimeGet,dTimeRotate-dTimeFindPlane,dTimeSort-dTimeRotate,dTimeMap-dTimeSort);
}
///////////////////////////////////////////////////////


void UFuncPCLTest::MakeMap(pcl::PointCloud<pcl::PointXYZ>::Ptr floor,pcl::PointCloud<pcl::PointXYZ>::Ptr obst,pcl::PointCloud<pcl::PointXYZ>::Ptr roof)
{
  double minY = varMinMapY->getDouble();
  double maxY = varMaxMapY->getDouble();
  double minX = varMinMapX->getDouble();
  double maxX = varMaxMapX->getDouble();
  double resolution = varSquareSize->getDouble();
  int Mx,My;
  UImagePool * iPool;
  iPool = (UImagePool *) getStaticResource("imgPool", false, true);
  if (iPool != NULL)
  {
    int h = round((maxY-minY)/resolution);
    int w = round((maxX-minX)/resolution);
// make corrected depth image and update
    UImage * Map = iPool->getImage(varKinectMapPool->getInt(), true, h, w);
    if (Map != NULL)
    {
      if (Map->tryLock())
      {
        printf("making map\n");
        Map->setColorType(PIX_PLANES_RGB);
        Map->setSize(h, w, 3, 8, "RGB");
        Map->setName("kinect data generated map");
        Map->clear();
        UPixel * pix = Map->getData();
        UPixel * pix2 = Map->getData();
//make cam point
        Mx = (int) round(0-minX/resolution);
        My = (int) round(0-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 =pix+((h-1-My)*w+Mx);
            pix2->p3 = 0xFF;
          }
        Mx = (int) round(1-minX/resolution);
        My = (int) round(0-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 =pix+((h-1-My)*w+Mx);
            pix2->p3 = 0xFF;
          }
        Mx = (int) round(-1-minX/resolution);
        My = (int) round(0-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 =pix+((h-1-My)*w+Mx);
            pix2->p3 = 0xFF;
          }
        Mx = (int) round(0-minX/resolution);
        My = (int) round(1-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 =pix+((h-1-My)*w+Mx);
            pix2->p3 = 0xFF;
          }
        Mx = (int) round(0-minX/resolution);
        My = (int) round(-1-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 =pix+((h-1-My)*w+Mx);
            pix2->p3 = 0xFF;
          }
//cam point end
        for (int i = 0; i < (int)floor->points.size(); i++)
        {
          Mx = (int) round(floor->points[i].x/resolution-minX/resolution);
          My = (int) round(floor->points[i].y/resolution-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 = pix+((h-1-My)*w+Mx);
            pix2->p2 = 0xFF;
          }
        }
        for (int i = 0; i < (int)obst->points.size(); i++)
        {
          Mx = (int) round(obst->points[i].x/resolution-minX/resolution);
          My = (int) round(obst->points[i].y/resolution-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 =pix+((h-1-My)*w+Mx);
            pix2->p1 = 0xFF;
            pix2->p2 = pix2->p2/2;
          }
        }
        for (int i = 0; i < (int)roof->points.size(); i++)
        {
          Mx = (int) round(roof->points[i].x/resolution-minX/resolution);
          My = (int) round(roof->points[i].y/resolution-minY/resolution);
          if(Mx >= 0 && Mx < w && My >= 0 && My < h)
          {
            pix2 =pix+((h-1-My)*w+Mx);
            pix2->p3 = 0xFF;
          }
        }
        Map->updated();
        Map->unlock();
      }
    }
  }
}

