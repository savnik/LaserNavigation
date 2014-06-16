/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include <urob4/usmltag.h>
#include <cstdlib>
#include <iostream>

#include <urob4/uresposehist.h>
#include <ucam4/ufunctioncambase.h>

//#include <boost/config.hpp>

#define BOOST_SYMBOL_VISIBLE
#include <boost/typeof/std/locale.hpp>
#include <boost/thread/exceptions.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "ufuncview.h"

///////////////////////////////////////////////////

UFuncView::~UFuncView()
{ // possibly remove allocated variables here - if needed
}

///////////////////////////////////////////////////

bool UFuncView::handleCommand(UServerInMsg * msg, void * extra)
{ // handle command(s) send to this plug-in
  char att[MAX_SML_NAME_LENGTH];
  char att2[MAX_SML_NAME_LENGTH] = "";
  const int VBL = 500;
  char val[VBL];
  const int MRL = 600;
  char reply[MRL];
  bool ask4help = false;
  bool result = false;
  bool replyOK = false;
  double v;
  UNavView * np;
  bool aPaint = false;
  //
  np = &navPaint;
  while (msg->tag.getNextAttribute(att, val, VBL))
  { // camera device
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else
    {
      aPaint = true;
      if (strcasecmp(att, "do") == 0)
        ; // a paint
      else if (strcasecmp(att, "bold") == 0)
        paintBold(str2bool2(val, true));
      else if (strcasecmp(att, "curves") == 0)
        np->paintCurves = str2bool2(val, true);
      else if (strcasecmp(att, "cam") == 0)
        np->paintCam = str2bool2(val, true);
      else if (strcasecmp(att, "gmk") == 0)
        np->paintGmk = str2bool2(val, true);
      else if (strcasecmp(att, "hereNow") == 0)
        np->setRefSystemsHere();
      else if (strcasecmp(att, "autoHereNow") == 0)
        varAutoHereNow->setValued(str2bool2(val, true));
      else if (strcasecmp(att, "gridSys") == 0)
        np->paintPoseRef = strtol(val, NULL, 0);
      else if (strcasecmp(att, "grid") == 0)
      {
        v = strtod(val, NULL);
        np->paintGridOdo = (v > 0);
        if (v > 0)
          np->paintGridSize[0] = v;
      }
      else if (strcasecmp(att, "rangeRings") == 0)
        setRangeRingCnt(strtol(val, NULL, 10));
      else if (strcasecmp(att, "pass") == 0)
        np->paintIntervalLines = str2bool2(val, true);
      else if (strcasecmp(att, "robot") == 0)
        setRobot(val);
      else if (strcasecmp(att, "obst") == 0)
        np->paintObstCnt = strtol(val, NULL, 0);
      else if (strcasecmp(att, "poseHist") == 0)
      {
        np->paintPoseHistCnt = strtol(val, NULL, 0);
      }
      else if (strcasecmp(att, "poseHistVecCnt") == 0)
        np->paintPoseHistVecCnt = strtol(val, NULL, 0);
      else if (strcasecmp(att, "poseHistVecLng") == 0)
        np->paintPoseHistVecLng = strtol(val, NULL, 0);
      else if (strcasecmp(att, "path") == 0)
        np->paintPathLinesCnt = strtol(val, NULL, 0);
      else if (strcasecmp(att, "pathMid") == 0)
        np->paintPathMidPoses = str2bool2(val, true);
      else if (strcasecmp(att, "road") == 0)
        np->paintRoadHistCnt = strtol(val, NULL, 0);
      else if (strcasecmp(att, "roadAll") == 0)
        np->paintRoadAll = str2bool2(val, true);
      else if (strcasecmp(att, "scan") == 0)
        np->paintScanHistCnt = strtol(val, NULL, 0);
      else if (strcasecmp(att, "visPoly") == 0)
        np->paintVisPolyCnt = strtol(val, NULL, 0);
      else if (strcasecmp(att, "var") == 0)
        np->paintVar = str2bool2(val, true);
      else if (strcasecmp(att, "varAdd") == 0)
      {
        if (np->paintVarAdd(val, true))
          np->paintVar = true;
        else
          strncpy(att2, att, MAX_SML_NAME_LENGTH);
      }
      else if (strcasecmp(att, "varDel") == 0)
      {
        if (not np->paintVarAdd(val, false))
          strncpy(att2, att, MAX_SML_NAME_LENGTH);
      }
      else if (strcasecmp(att, "poly") == 0)
        np->paintPoly = str2bool2(val, true);
      else if (strcasecmp(att, "polyNameCnt") == 0)
        np->paintPolyNameCnt = strtol(val, NULL, 10);
      else if (strcasecmp(att, "polyHide") == 0)
        strncpy(np->paintPolyHide, val, np->maxStrLng);
      else if (strcasecmp(att, "polyShow") == 0)
        strncpy(np->paintPolyShow, val, np->maxStrLng);
      else if (strcasecmp(att, "pcpHide") == 0)
        strncpy(np->paintPcpHide, val, np->maxStrLng);
      else if (strcasecmp(att, "pcpShow") == 0)
        strncpy(np->paintPcpShow, val, np->maxStrLng);
      else if (strcasecmp(att, "odoPose") == 0)
        np->paintOdoPose = strtol(val, NULL, 0);
      else if (strcasecmp(att, "utmPose") == 0)
        np->paintUtmPose = strtol(val, NULL, 0);
      else if (strcasecmp(att, "mapPose") == 0)
        np->paintMapPose = strtol(val, NULL, 0);
      else if (strcasecmp(att, "follow") == 0)
        np->followRobot = str2bool2(val, true);
      else if (strcasecmp(att, "savePng") == 0)
      {
        UTime t;
        t.now();
        if (strlen(val) == 0)
        {
          strcpy(saveAsPngName, "view");
          t.getForFilename(&saveAsPngName[4]);
          strncat(saveAsPngName, ".png", MAX_FILENAME_SIZE);
        }
        else if (val[0] == '/' or val[0] == '.' or val[0] == '~')
          strncpy(saveAsPngName, val, MAX_FILENAME_SIZE);
        else
          snprintf(saveAsPngName, MAX_FILENAME_SIZE, "%s/%s", imagePath, val);
        saveAsPng = true;
      }
      else if (strcasecmp(att, "resetView") == 0)
      {
        resetView = true;
      }
      else
        // not found - make a copy for reply
        strncpy(att2, att, MAX_SML_NAME_LENGTH);
    }
  }
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"VIEW\">\n");
    sendText(msg, "------ 3D-view image options ----------\n");

    sendText(msg, "do                        Redraw 3D view\n");
    sendText(msg, "bold[=false]              Paint navigation image using bold lines (for presentations)\n");
//     snprintf(reply, MRL,
//               "curves[=false]            Paint laser line-fit variance curves (is %s)\n", bool2str(np->paintCurves));
//     sendText(msg, reply);
    snprintf(reply, MRL,
              "gridSys=0 | 1 | 2         Paint grid based on 0=odometry, 1=UTM, 2=Map coordinates (is %d)\n", np->paintPoseRef);
    sendText(msg, reply);
    snprintf(reply, MRL,
              "odoPose[=false]           Show odometry pose at bottom of display (is %s)\n", bool2str(np->paintOdoPose));
    sendText(msg, reply);
    snprintf(reply, MRL,
              "utmPose[=false]           Show UTM (GPS) pose at bottom of display (is %s)\n", bool2str(np->paintUtmPose));
    sendText(msg, reply);
    snprintf(reply, MRL,
              "mapPose[=false]           Show map pose at bottom of display (is %s)\n", bool2str(np->paintMapPose));
    sendText(msg, reply);
//    sendText(msg, "hereNow                   Synchronoze all coordinate systems to here now\n");
    snprintf(reply, MRL,
              "grid[=M]                  Paint coordinate grid every M meter (is %gm)\n", np->paintGridSize[0]);
    sendText(msg, reply);
//     snprintf(reply, MRL,
//               "rangeRings[=M]            Paint M range rings around laser scanner (is %dm)\n", np->rangeRingCnt);
//     sendText(msg, reply);
//     snprintf(reply, MRL,
//               "pass[=false]              Paint passable lines from laser scanner (is %s)\n", bool2str(np->paintIntervalLines));
//     sendText(msg, reply);
    snprintf(reply, MRL,
              "poly[=false]              Paint poly items - planned mission lines etc. (is %s)\n", bool2str(np->paintPoly));
    sendText(msg, reply);
    snprintf(reply, MRL,
              "polyNameCnt=N             Paint polygon name, max N characters (last), N=%d\n", np->paintPolyNameCnt);
    sendText(msg, reply);
    snprintf(reply, MRL,
              "polyHide=\"name\"           Hide selected poly items - accept wildchards (is '%s')\n", np->paintPolyHide);
    sendText(msg, reply);
    snprintf(reply, MRL,
              "polyShow=\"name\"           Show among hidden poly items - accept wildchards (is '%s')\n", np->paintPolyShow);
    sendText(msg, reply);
    snprintf(reply, MRL,
              "pcpHide=\"name\"           Hide selected PCP clouds - accept wildchards (is '%s')\n", np->paintPolyHide);
    sendText(msg, reply);
    snprintf(reply, MRL,
              "pcpShow=\"name\"           Show among hidden PCL clouds items - accept wildchards (is '%s')\n", np->paintPolyShow);
    sendText(msg, reply);
    sendText(msg, "robot=[smr|mmr|hako|iRobot|guidebot]  Paint robot outline as SMR, MMR...\n");
    snprintf(reply, MRL,
              "obst=N                    Paint N obstacle groups (is %d)\n", np->paintObstCnt);
    sendText(msg, reply);
    snprintf(reply, MRL,
              "poseHist=N                Paint N pose history positions for robot (is %d)\n", np->paintPoseHistCnt);
    sendText(msg, reply);
//     snprintf(reply, MRL,
//               "poseHistVecCnt=N          Paint every N pose hist cnt a heading vector (is %d)\n", np->paintPoseHistVecCnt);
//     sendText(msg, reply);
//     snprintf(reply, MRL,
//               "poseHistVecLng=N          Paint pose hist heading vector N pixels long (is %d)\n", np->paintPoseHistVecLng);
//     sendText(msg, reply);
    snprintf(reply, MRL,
              "path=[0 | 1 | N]          Paint navigation path plan 0=no, 1=best, N=all (is %d)\n",
            np->paintPathLinesCnt);
    sendText(msg, reply);
//     snprintf(reply, MRL,
//               "pathMid[=false]           Paint mid-poses used in path calculation (requires path > 0) is %s\n",
//               bool2str(np->paintPathMidPoses));
//     sendText(msg, reply);
    snprintf(reply, MRL,
              "follow                    Follow while robot moves (is %s)\n", bool2str(np->followRobot));
    sendText(msg, reply);
//     snprintf(reply, MRL,
//               "roadAll                   Paint all available road lines (not just current best road) (is %s)\n", bool2str(np->paintRoadAll));
//     sendText(msg, reply);
    snprintf(reply, MRL,
              "scan=N                    Paint laserscan and history - up to N scans (is %d)\n", np->paintScanHistCnt);
    sendText(msg, reply);
    sendText(msg, "var[=false]               Paint variables in struct list\n");
    sendText(msg, "varAdd=struct             Paint all variables in this struct\n");
    sendText(msg, "varDel=struct             Hide  all variables in this struct\n");
    sendText(msg, "resetView                 Reset view to default (look at robot pose)\n");
    snprintf(reply, MRL,
              "visPoly=N                 Paint N polygons from vision road detection (is %d)\n", np->paintVisPolyCnt);
    sendText(msg, reply);
    sendText(msg, "savePng[=name]            Save current view as a png-file\n");
//      sendText(msg, "testcap                   openCV test capture function\n");
    sendText(msg, "help                      This help tekst\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else
  {
    if (strlen(att2) > 0)
    {
      snprintf(reply, MRL, "Unknown attribute %s", att2);
      sendWarning(msg, reply);
      replyOK = true;
    }
    else if (aPaint)
    { // an OK paint setting
      sendInfo(msg, "done");
      replyOK = true;
      // force redisplay
      globalNavRedisplay = true;
    }
    if (not replyOK)
    {
      snprintf(reply, MRL, "Unknown subject %s - try 'disp help'\n",
              msg->tag.getTagStart());
      sendWarning(msg, reply);
    }
  }
  return result;
}

///////////////////////////////////////////////////


void * startUFuncViewThread(void * obj)
{ // call the hadling function in provided object
  UFuncView * ce = (UFuncView *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

//////////////////////////////////////////

bool UFuncView::start()
{
  int err = 0;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    err = (pthread_create(&threadHandle, &thAttr,
              &startUFuncViewThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return err;
}

///////////////////////////////////////////////////

void UFuncView::stop(bool andWait)
{
  if (threadRunning and not threadStop)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

///////////////////////////////////////////////////

void UFuncView::run()
{
  const double WAIT_TIME_BEFORE_REPAINT = 0.15;
  bool redraw = false;
  // set camera 4m behind robot 1m to the left in a height of 2m, looking at ground-point
  // 1m in front of robot, up is in z direction
  UPosition vp = varResetViewPos->get3D();
  double cmP[9] = {vp.x, vp.y, vp.z, // camera position
                    0.0, 0.0, 0.0,  // point camera is looking at (rotation point in viewer)
                    0.0, 0.0, 1.0}; // up vector for camera
  double viewangle;
  UTime viewangleTime;
  //
  threadRunning = true;
  UTime lastRedraw;
  // create viewer
  viewer = new pcl::visualization::PCLVisualizer("auviewer");
  viewer->setBackgroundColor(30, 130, 60);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  pcl::PointCloud<pcl::PointXYZRGB> * cloud =  new pcl::PointCloud<pcl::PointXYZRGB>();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudPtr);
  // set camera 4m behind robot 1m to the left in a height of 2m, looking at ground-point
  // 1m in front of robot, up is in z direction
//   viewer->camera_.view[0] = cmP[6];
//   viewer->camera_.view[1] = cmP[7];
//   viewer->camera_.view[2] = cmP[8];
//   viewer->camera_.focal[0] = cmP[3];
//   viewer->camera_.focal[1] = cmP[4];
//   viewer->camera_.focal[2] = cmP[5];
//   viewer->camera_.pos[0] = cmP[0];
//   viewer->camera_.pos[1] = cmP[1];
//   viewer->camera_.pos[2] = cmP[2];
  // this one is in pcl 1.7 only
#if PCL_MINOR_VERSION < 7
   viewer->setCameraPose(cmP[0], cmP[1], cmP[2], cmP[3], cmP[4], cmP[5], cmP[6], cmP[7], cmP[8]);
   // may also set
   // viewer->setCamereFieldOfView(verticalInRadians);
#else
   viewer->setCameraPosition(cmP[0], cmP[1], cmP[2], cmP[3], cmP[4], cmP[5], cmP[6], cmP[7], cmP[8]);
//     viewer->setCameraPosition(campos.x, campos.y, campos.z,   // camera position
//                           camFocus.x, camFocus.y, cam.focal[2],  // view focus point)
//                           0.0, 0.0, 1.0);   // up vector (view vector)
    //viewer->setCameraPosition(c.pos[0], c.pos[1], c.pos[2], seenFromPose.x, seenFromPose.y, 0.0);
#endif
  // this one is in pcl 1.5 only
  //viewer->setCameraPose(cmP[0], cmP[1], cmP[2], cmP[3], cmP[4], cmP[5], cmP[6], cmP[7], cmP[8]);
  //
  // debug
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  uint8_t r(255), g(15), b(15);
  for (float z = 0.0 ; z <= 2.0; z += 0.1)
  { // from height zero to 2 meter
    for (float angle = (-90.0); angle <= 90.0; angle += 5.0)
    { // all around 360 deg
      pcl::PointXYZRGB point;
      point.x = 5.0 * cosf (pcl::deg2rad(angle));
      point.y = sinf (pcl::deg2rad(angle));
      point.z = z;
      point.r = r;
      point.g = g;
      point.b = b;
      cloud->points.push_back (point);
    }
    if (z < 1.0)
    {
      r -= 24;
      g += 24;
    }
    else
    {
      g -= 24;
      b += 24;
    }
  }
  cloud->width = 2; // (int) cloud->points.size ();
  cloud->height = 2;
  //viewer->addPointCloud<pcl::PointXYZRGB> (cloudPtr, rgb, "sample cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  //viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
  //                                   cloud->points[cloud->size() - 1], "line");
  // viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
   pcl::ModelCoefficients coeffs1; //, coeffs2; // plane is ax + by + cz + d = 0
   coeffs1.values.push_back (0.0);//a normal vector x
   coeffs1.values.push_back (0.0);//b normal vector y
   coeffs1.values.push_back (20.0);//c normal vector z
   coeffs1.values.push_back (0.0);//d -position on normal vector
   viewer->addPlane(coeffs1, "plane1"); // painted as square
   viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "plane1");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "plane1");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "plane1");
//   coeffs2.values.push_back (1.0);//a normal vector x
//   coeffs2.values.push_back (0.0);//b normal vector y
//   coeffs2.values.push_back (0.0);//c normal vector z
//   coeffs2.values.push_back (0.0);//d -position on normal vector
//   viewer->addPlane(coeffs2, "plane2"); // painted as square
//   viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane2");
  // make viewer available to painter
  navPaint.setViewer(viewer, cmP);
  resetView = true;

//  viewer = shapesVis(point_cloud_ptr);
  lastRedraw.now();
  while (not threadStop)
  {
    dispSync.lock();
    // Repaint all image-pool images
    if (varResetView->getBool())
    {
      UPosition viewpos = varResetViewPos->get3D();
      UPosition focusPoint = varResetViewFocus->get3D();
      navPaint.moveViewPose(0, true, &viewpos, &focusPoint);
      varResetView->setBool(false);
      viewangle = 0; // atan2(-viewpos.y, -viewpos.x);
      viewangleTime.now();
    }
    else if (varAutoPanView->getBool(0))
    {
      double dt = viewangleTime.getTimePassed();
      if (dt < varAutoPanView->getDouble(2))
      {
        double da = 2.0 * M_PI * dt / varAutoPanView->getDouble(2);
        viewangle = limitToPi(viewangle + da);
        // get new view angle in radians
        double panAng = varAutoPanView->getDouble(1) * M_PI / 180.0 * sin(viewangle);
        navPaint.moveViewPose(panAng, false, NULL, NULL);
      }
      viewangleTime.now();
    }
    // test if other data types is updated
    if (newDataLaser and newDataLaserAt.getTimePassed() > WAIT_TIME_BEFORE_REPAINT)
      redraw = true;
    if (newDataNav and newDataNavAt.getTimePassed() > WAIT_TIME_BEFORE_REPAINT)
      redraw = true;
    if ((redraw or globalNavRedisplay) and lastRedraw.getTimePassed() > 0.1)
    {
      newDataLaser = false;
      newDataNav = false;
      redraw = false;
      globalNavRedisplay = false;
      lastRedraw.now();
      if (varAutoHereNow->getBool())
        navPaint.setRefSystemsHere();
      navPaint.rangeRingCnt = varRangeRings->getInt();
      navPaint.followRobot = varFollowRobot->getBool();
      //
      navPaint.paint();
    }
    if (saveAsPng)
    {
      viewer->saveScreenshot(saveAsPngName);
      saveAsPng = false;
    }
    // update to screen too
    viewer->spinOnce(varPCLspinTime->getInt() + 1);
    dispSync.unlock();
    // wait a bit to reduce CPU load
    Wait(0.01);
  }
  threadRunning = false;
}

///////////////////////////////////////////////////

// void UFuncView::shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//   viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//
//   //------------------------------------
//   //-----Add shapes at cloud points-----
//   //------------------------------------
//   viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
//                                      cloud->points[cloud->size() - 1], "line");
//   viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");
//
//   //---------------------------------------
//   //-----Add shapes at other locations-----
//   //---------------------------------------
//   pcl::ModelCoefficients coeffs;
//   coeffs.values.push_back (0.0);//x
//   coeffs.values.push_back (0.0);//y
//   coeffs.values.push_back (1.0);//z
//   coeffs.values.push_back (0.0);//??
//   viewer->addPlane (coeffs, "plane");
//   coeffs.values.clear ();
//   coeffs.values.push_back (0.3);
//   coeffs.values.push_back (0.3);
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (1.0);
//   coeffs.values.push_back (0.0);
//   coeffs.values.push_back (5.0);
//   viewer->addCone (coeffs, "cone");
//
//   return (viewer);
// }

///////////////////////////////////////////////////////////////

void UFuncView::createBaseVar()
{
  varRunning = addVar("running", 0.0, "d", "(ro) Is display loop (thread) running");
  addVar("paintGrid", 0.0, "d", "(rw) Paint the odometry grid");
  addVar("paintSpeed", 0.0, "d", "(rw) Paint robot speed");
  varAutoHereNow = addVar("autoHereNow", 1.0, "d", "Should coordinate systems be aligned at current robot pose");
  varBold = addVar("bold", 0.0, "d", "(rw) Paint drawn images more bold");
  varRangeRings = addVar("rangeRings", 8.0, "d", "(rw) Number of range rings to paint");
  varResetViewPos = addVarA("defViewPos", "-6.0 1.5 1.5" , "3d", "(rw) Default position of view camera");
  varResetView = addVar("resetView", 1.0, "d", "(r/w) reset to default view camera position");
  varResetViewFocus = addVarA("defViewFocus", "0.5 0.0 0.4", "3d", "(rw) Focal point for view camera - relative to robot)");
  varAutoPanView = addVarA("autoPan", "0 45 3", "d", "(rw) pan automatically [0]=1 active, [1]=pan angle (degrees), [2]=cycle time (sec)");
  varPCLspinTime = addVar("spinTime", 30, "d", "(rw) PCL viewer spin time - to update display and get mouse input)");
  varFollowRobot = addVar("followRobot", 0.0, "d", "(rw) should view try to follow robot (experimental)");
  //
  addMethod("newData", "sd", "Called to trigger repaint of new data. The "
      "string parameter is the data type (pt. img, laser, poly or nav). The number parameter"
          "is an associated number (e.g. image pool number).");
  //
}

///////////////////////////////////////////////////////////

bool UFuncView::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  //
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // delete any local
    if (remove)
      poseHist = NULL;
    else if (poseHist != resource)
      poseHist = (UResPoseHist *)resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getUtmPoseID()))
  { // delete any local
    if (remove)
      poseUtm = NULL;
    else if (poseUtm != resource)
      poseUtm = (UResPoseHist *)resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getMapPoseID()))
  { // delete any local
    if (remove)
      poseMap = NULL;
    else if (poseMap != resource)
      poseMap = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResVarPool::getResID()))
  { // delete any local
    if (remove)
      varRoot = NULL;
    else if (varRoot != resource)
      varRoot = (UResVarPool *)resource;
    else
      result = false;
  }
  else if (resource->isA(UImagePool::getResClassID()))
  { // delete any local
    if (remove)
      imgPool = NULL;
    else if (imgPool != resource)
      imgPool = (UImagePool *)resource;
    else
      result = false;
  }
  else
    result = false;
  // the navigation painter may need these resources too
  result |= navPaint.setResource(resource, remove);
  result |= UFuncPlugBase::setResource(resource, remove);
  //
  return result;
}

////////////////////////////////////////////////////////////////////

bool UFuncView::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  // evaluate standard functions
  if ((strcasecmp(name, "newData") == 0) and (strcmp(paramOrder, "sd") == 0))
  {
    if (strcasecmp(strings[0], "img") == 0)
    { // not user here - for image pool update only
    }
    else if (strcasecmp(strings[0], "laser") == 0)
    {
      if (not newDataLaser)
      { // laser data needs redisplay
        newDataLaserAt.Now();
        newDataLaser = true;
      }
    }
    else
    { // all other types of navigation data update
      if (not newDataNav)
      { // navigation data needs redisplay
        newDataNavAt.Now();
        newDataNav = true;
      }
    }
    // set return values
    if (value != NULL)
      *value = 1;
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
    //
  }
  else
    result = false;
  return result;
}

///////////////////////////////////////////////////

void UFuncView::setRobotPose(char * value)
{
  const char *p2 = value;
  navPaint.robotPose.x = strtod(p2, (char**)&p2);
  if (p2 != NULL)
  {
    if (*p2 == ',')
      p2++;
    navPaint.robotPose.y = strtod(p2, (char**)&p2);
  }
  if (p2 != NULL)
  {
    if (*p2 == ',')
      p2++;
    navPaint.robotPose.h = strtod(p2, (char**)&p2);
  }
}

///////////////////////////////////////////

void UFuncView::paintBold(bool bold)
{
  navPaint.paintBold = bold;
  varBold->setValued(bold);
}

///////////////////////////////////////////

void UFuncView::setRangeRingCnt(int value)
{
  varRangeRings->setValued(value);
  navPaint.rangeRingCnt=value;
}

////////////////////////////////////////////////

void UFuncView::setRobot(const char * robname)
{
  if (strcasecmp(robname, "SMR") == 0)
  {
    navPaint.paintRobot = 0;
    varRangeRings->setValued(4.0);
  }
  else if (strcasecmp(robname, "MMR") == 0)
  {
    navPaint.paintRobot = 1;
    varRangeRings->setValued(8.0);
  }
  else if (strcasecmp(robname, "HAKO") == 0)
  {
    navPaint.paintRobot = 2;
    varRangeRings->setValued(8.0);
  }
  else if (strcasecmp(robname, "irobot") == 0)
  {
    navPaint.paintRobot = 3;
    varRangeRings->setValued(16.0);
  }
  else if (strcasecmp(robname, "guidebot") == 0)
  {
    navPaint.paintRobot = 4;
    varRangeRings->setValued(16.0);
  }
}

/////////////////////////////////////////////////

#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncView' with your classname, as used in the headerfile */
  return new UFuncView();
}
#endif



