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

//#include <opencv/cv.h>
//#include <opencv/highgui.h>

#include <ugen4/ucommon.h>
#include <ugen4/u3d.h>
#include <urob4/uresbase.h>
#include <urob4/uresposehist.h>
#include <urob4/uimagepool.h>
#include <urob4/uvarcalc.h>
#include <umap4/umanseq.h>
#include <umap4/umanarc.h>
#include <umap4/umanline.h>

#include "../aulaserif/uobstaclehist.h"
#include "../ucamif/uclientcamifpath.h"
#include "../aulaserif/ufeaturepool.h"
#include "../aulaserif/ureslaserifroad.h"
#include "../aulaserif/ureslaserifobst.h"
#include "../aulaserifscan/ureslaserifscan.h"
#include "../aulaserif/ureslaserifsf.h"
#include "../aulaserif/uresnavifman.h"
#include "../ucamif/urescamifcam.h"
#include "../ucamif/uclientcamifgmk.h"
#include "../aupoly/urespoly.h"
#include "../aupcp/urespcp.h"

#include <boost/typeof/std/locale.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "unavview.h"

//#include <pcl/visualization/pcl_visualizer.h>

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

class UPaintPcp : public UPaintBase
{
public:
  /// pointer to pcl
  UPcpItem * pcl;
  /// last update time
  UTime updateTime;
  /// last known name (display ID)
  static const int MAX_ID_SIZE = 32;
  char idname[MAX_ID_SIZE];
  /// shown already
  bool shown;
  /// point cloud for the scan
  pcl::PointCloud<pcl::PointXYZRGB> * cloud;
  /// boost pointer to the cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr * cloudPtr;
  /// rgbhandler
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> * rgbHandler;

public:
  /// constructor
  UPaintPcp()
  {
    pcl = NULL;
    updateTime.clear();
    idname[0] = '\0';
    shown = false;
    cloud = NULL;
    cloudPtr = NULL;
    rgbHandler = NULL;
  }
  /// update with this point cloud
  virtual void update(UPcpItem * newPcl, UResPoseHist * posesys, UPose * rp)
  {
    UMatrix4 mt;
    Eigen::Matrix4f td2g;
    shown = false;
    updateTime = newPcl->updateTime;
    pcl = newPcl;
    strncpy(idname, pcl->name, MAX_ID_SIZE);
    if (cloud == NULL)
    {
      cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
      cloudPtr = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud);
      rgbHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(*cloudPtr);
    }
    // it is assumed that the cloud is in robot coordinates, either direct or in sensor (relative) coordinates
    if (newPcl->relPoseUse)
    { // point cloud is in a coordinate system relative
      // to a position (relPose) on the robot 
      UPosRot d2r = newPcl->relPose;
      UMatrix4 mr2g, md2r;
      // get conversion matrix from relative position to robot coordinates (3D)
      md2r = d2r.getRtoMMatrix();
      // get 3D position conversion matrix from (2D) robot pose
      mr2g = rp->asMatrix4x4PtoMPos();
      // make full convertion matrix
      mt = mr2g * md2r;
    }
    else
    { // the cloud is in robot coordinates
      mt = rp->asMatrix4x4PtoM();
    }
    // use PCL coordinate conversion
    double * d = mt.getData();
    // move to PCL conversion matrix
    td2g <<  d[0],  d[1],  d[2],  d[3],
             d[4],  d[5],  d[6],  d[7],
             d[8],  d[9],  d[10], d[11],
             d[12], d[13], d[14], d[15];
    newPcl->cooSys=0;
    if (newPcl->xyzrgb != NULL)
    { // do the transformation - and copy - to viewer coordinates
      pcl::transformPointCloud (*newPcl->xyzrgb, *cloud, td2g);
    }
  }
  /**
   * Paint robot at this post to this viewer. */
  virtual void paint(pcl::visualization::PCLVisualizer* viewer)
  {
    int lw;
    if (cloud == NULL or shown)
      return;
    if (pcl->getCloudType() != UPcpItem::PointXyzRgb)
    {
      printf("Can show XYZRGB point clouds only (%s is not)\n", idname);
      return;
    }
    if (bold)
      lw = 8;
    else
      lw = 4;
    // may be OK
    viewer->addPointCloud<pcl::PointXYZRGB>(*cloudPtr, *rgbHandler, idname, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, lw, idname);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, idname);
    shown = true;
  }
  /**
   * remove all items from this viewer. */
  virtual void unPaint(pcl::visualization::PCLVisualizer* viewer)
  {
    if (shown)
      viewer->removePointCloud(idname);
    shown = false;
  };
  /**
   * get cloud name */
  virtual const char * name() { return idname; };
};


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

class UPaintManoeuvre : public UPaintBase
{
public:
  /// point cloud for the polygon
  pcl::PointCloud<pcl::PointXYZ> * cloud;
  /// boost pointer to the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr * cloudPtr;
  /// is polygon shown
  bool isShown;
  /// manoeuvre sequence 
  UClientManSeq * man;
  /// manoeuver id number
  int id;
  /// time the manouvre is valid
  UTime valTime;
  /// constructor
  UPaintManoeuvre()
  {
    man = NULL;
    isShown = false;
    cloud = NULL;
    cloudPtr = NULL;
    id = -1;
  };
  
  /**
   * Paint this scan into a pointcloud */
  void makeManCloud()
  {
    pcl::PointXYZ pcp1;
    UPosition pw;
    UManPPSeq * mpp;
    int pnt, pntCnt;
    UManoeuvre * mm;
    UManArc * mmArc;
    const int pointsPerRadian = 10;
    //
    if (cloud == NULL)
    {
      cloud = new pcl::PointCloud<pcl::PointXYZ>();
      cloudPtr = new pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud);
    }
    // cound number of points
    pntCnt = 0;
    for (int i = 0; i < man->getP2PCnt(); i++)
    { // a path is divided into groups of 1 to 5 (line, arc, line, arc, line) simple manoeuvres
      mpp = man->getP2P(i);
      for (int j = 0; j < mpp->getSeqCnt(); j++)
      {
        mm = mpp->getMan(j);
        switch (mm->getManType())
        { // either MAN_ARC, MAN_LINE or MAN_STOP
        case UManoeuvre::MAN_ARC:
          mmArc = (UManArc *)mm;
          pntCnt += floor(absd(mmArc->getTurnAngle() * pointsPerRadian)) + 1;
          break;
        case UManoeuvre::MAN_LINE:
          pntCnt++;
          break;
        default:
          break;
        }
      }
    }
    cloud->points.resize(pntCnt);
    UPoseV pv1;
    pcl::PointXYZ pc1;
    const double manZ = 0.01; // a bit above the ground
    pnt = 0;
    for (int i = 0; i < man->getP2PCnt(); i++)
    { // a path is divided into groups of 1 to 5 (line, arc, line, arc, line) simple manoeuvres
      mpp = man->getP2P(i);
      pv1 = man->getStartPoseV();
      // add start point
      cloud->points[pnt++] = getInViewedCoordinates(&pv1, 0, manZ);
      for (int j = 0; j < mpp->getSeqCnt(); j++)
      {
        mm = mpp->getMan(j);
        if (mm->getManType() == UManoeuvre::MAN_ARC)
        { // convert arc to points
          UManArc mac, *mma;
          UPoseV pw = pv1;
          int apCnt = floor(absd(mmArc->getTurnAngle() * pointsPerRadian));
          mma = (UManArc*)mm;
          if (mma->getTurnRadius() < 0.0)
          { // convert to positive radius and signed angle
            mma->setTurnAngle(-mma->getTurnAngle());
            mma->setTurnRadius(-mma->getTurnRadius());
          }
          mac.setTurnRadius(mma->getTurnRadius());
          mac.setTurnAngle(signofd(mma->getTurnAngle()) / double(pointsPerRadian));
          for (int m = 0; m < apCnt; m++)
          { // add point along circle
            pw = mac.getEndPoseV(pw);
            cloud->points[pnt++] = getInViewedCoordinates(&pw, 0, manZ);
          }
        }
        // add end point
        pv1 = mm->getEndPoseV(pv1);
        cloud->points[pnt++] = getInViewedCoordinates(&pv1, 0, manZ);
      }
    }
  };
  /**
   * Remove point cloud from viewer */
  void unPaint(pcl::visualization::PCLVisualizer * viewer)
  {
    if (isShown and man != NULL)
    {
      const int MSL = 20;
      char s[MSL];
      snprintf(s, MSL, "man%04d", id);
      viewer->removeShape(s);
      // remove old dots at vertices
      snprintf(s, MSL, "man%04da", id);
      viewer->removePointCloud(s);
    }
    isShown = false;
  }
  /**
   * add cloud to the viewer - removing the older
   * \param manoeuvre is polygon to view.
   * \param viewer is the 3D viewer */
  void addManToViewer(UClientManSeq * manoeuvre, pcl::visualization::PCLVisualizer * viewer)
  {
    man = manoeuvre;
    if (man == NULL)
      return;
    makeManCloud();
    if (cloud->points.size() > 0)
    {
      double lw;
      const int MSL = 20;
      char s[MSL];
      uint32_t color = 0x303030; // normal is gray
      const uint32_t red = 0xff0000; // red for best man seq (used)
      if (bold)
        lw = 10;
      else
        lw = 5;
      if (man->isBest())
        color = red;
      // paint also dots at vertices
      snprintf(s, MSL, "man%04da", id);
      viewer->addPointCloud<pcl::PointXYZ>(*cloudPtr, s);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, lw, s);
      snprintf(s, MSL, "man%04d", id);
      viewer->addPolygon<pcl::PointXYZ>(*cloudPtr, s);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
                                               double(color >> 16)/256.0, 
                                               double((color >> 8) & 0xff)/256.0, 
                                               double(color & 0xff)/256.0, s);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lw, s);
      isShown = true;
      valTime = man->getUpdTime();
    }
  };
};

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

class UPaintManoeuvres
{
public:
  /// array of manoeuvres
  UPaintManoeuvre ** mans;
  /// number of used manoeuvre slots
  int mansCnt;
  /// number of available pointers 
  int mansMax;
  /// constructor
  UPaintManoeuvres()
  {
    mans = 0;
    mansCnt = 0;
    mansMax = 0;
  }
  /// destructor
  ~UPaintManoeuvres()
  {
    if (mans != NULL)
    {
      for (int i = 0; i < mansCnt; i++)
      {
        if (mans[i] != NULL)
          delete mans[i];
      }
      delete mans;
    }
  }
  /// remove all clouds
  void removeAll()
  {
    mansCnt = 0;
  };
  /// unpaint all
  void unPaintAll(pcl::visualization::PCLVisualizer* viewer)
  {
    for (int i = 0; i < mansCnt; i++)
    {
      if (mans[i] != NULL)
        mans[i]->unPaint(viewer);
    }
  }
  /// expand count of clouds
  void expand()
  {
    const int EXPAND_BY = 100;
    int n = mansMax;
    mansMax += EXPAND_BY;
    int m = mansMax * sizeof(UPaintManoeuvre **);
    mans = (UPaintManoeuvre **)realloc(mans, m);
    bzero(&mans[n], m - n  * sizeof(UPaintManoeuvre **));    
  }
  /**
   * Set (or add) this manoeuvre sequence to the view
   * \param viewer is the viewer
   * \param man is the manoeuvre sequence to show.
   * \param idx is the index of the manoeuvre 
   * \param seenFromPose is the reference pose for the manoeuvre */
  void paintManData(pcl::visualization::PCLVisualizer * viewer,
                    UClientManSeq * man, int idx, int cooSysView, UPose cooSysViewOrigin)
  {
    UPaintManoeuvre * mn;
    //
    while (idx >= mansMax)
      expand();
    if (idx >= mansCnt)
      mansCnt = idx + 1;
    mn = mans[idx];
    if (mn == NULL)
    { // new
      mn = new UPaintManoeuvre();
      mn->id = idx;
      mans[idx] = mn;
    }
    //
    if (man != NULL and mn !=NULL)
    { // basis for inclusion 
      mn->setViewCooSys(cooSysView, cooSysViewOrigin);
      if (man->getUpdTime() != mn->valTime or cooSysView != 0)
      { // remove old manoeuvre
        mn->unPaint(viewer);
        mn->addManToViewer(man, viewer);
      }
    }
  };
};

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

class UPaintPolygon : public UPaintBase
{
public:
  /// point cloud for the polygon
  pcl::PointCloud<pcl::PointXYZ> * cloud;
  /// boost pointer to the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr * cloudPtr;
  /// poly item
  UPolyItem * polyData;  
  /// color of the cloud
  uint32_t color;
  /// name
  const char * name;
  /// is polygon shown
  bool isShown;
  /// is it a polygon or a polyline
  bool isPolyline;
  /// id of the polyline
  int id;
  /// paint also dots at vertex
  bool dots;
  ///
  UPaintPolygon()
  {
    cloud = NULL;
    cloudPtr = NULL;
    isShown = false;
    polyData = NULL;
    color = 0;
    name = NULL;
    isPolyline = false;
    id = -1;
  }
  ~UPaintPolygon()
  {
    if (cloud != NULL)
      delete cloud;
    if (cloudPtr != NULL)
      delete cloudPtr;
  }
  /**
   * Paint this scan into a pointcloud */
  void makePolyCloud()
  {
    pcl::PointXYZ pcp1;
    UPosition * p;
    UPosition pw;
    //
    if (cloud == NULL)
    {
      cloud = new pcl::PointCloud<pcl::PointXYZ>();
      cloudPtr = new pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud);
    }
    cloud->points.resize(polyData->getPointsCnt());
    p = polyData->getPoints();
    if (p->dist() > 10000.0 and hypot(cooSysViewOrigin.x, cooSysViewOrigin.y) < 0.1)
    { // error in coordinate conversion
      // debug
      printf("UPaintPolygon::makePolyCloud unset system coordinate system - rescales\n");
      cooSysViewOrigin.x = -p->x;
      cooSysViewOrigin.y = -p->y;
    }
    for (int i = 0; i < polyData->getPointsCnt(); i++)
    {
      if (polyData->cooSys != cooSysView)
        pw = cooSysViewOrigin.getPoseToMap(*p);
      else
        pw = *p;
      pcp1.x = pw.x;
      pcp1.y = pw.y;
      pcp1.z = pw.z;
      cloud->points[i] = pcp1;
      p++;
    }
  };
  /**
   * Remove point cloud from viewer */
  void unPaint(pcl::visualization::PCLVisualizer * viewer)
  {
    if (isShown and polyData != NULL)
    {
      const int MSL = 20;
      char s[MSL];
      if (polyData->isPolygon())
      {
        snprintf(s, MSL, "poly%04d", id);
        viewer->removeShape(s);
      }
      if (dots)
      { // remove old dots at vertices
        snprintf(s, MSL, "poly%04da", id);
        viewer->removePointCloud(s);
      }
      if (name != NULL)
        if (name[0] > ' ')
        { // remove also name
          snprintf(s, MSL, "poly%04db", id);
          viewer->removeShape(s);
        }
      if (polyData->isPolyline())
      { // remove lines
        for (int i = 1; i < polyData->getPointsCnt(); i++)
        {
          snprintf(s, MSL, "poly%04d_%03d", id, i);
          viewer->removeShape(s);
        }
      }
    }
    isShown = false;
  }
  /**
   * add cloud to the viewer - removing the older
   * \param poly is polygon to view.
   * \param viewer is the 3D viewer */
  void addPolyToViewer(UPolyItem * poly, pcl::visualization::PCLVisualizer * viewer)
  {
    polyData = poly;
    if (polyData == NULL)
      return;
    makePolyCloud();
    if (cloud->points.size() > 0)
    {
      double lw;
      const int MSL = 20;
      char s[MSL];
      if (bold)
        lw = 4;
      else
        lw = 2;
      if (name != NULL)
        if (strlen(name) > 0)
        {
          UPosition pw = polyData->getCogXY();
          if (polyData->cooSys != cooSysView)
            pw = cooSysViewOrigin.getPoseToMap(pw);
          pcl::PointXYZ pcp1;
          pcp1.x = pw.x;
          pcp1.y = pw.y;
          pcp1.z = pw.z;
          snprintf(s, MSL, "poly%04db", id);
          viewer->addText3D<pcl::PointXYZ>(name, pcp1, lw/5.0, double(color >> 16)/256.0,
                              double((color >> 8) & 0xff)/256.0, 
                              double(color & 0xff)/256.0, s);
        }
      if (dots)
      { // paint also dots at vertices
        snprintf(s, MSL, "poly%04da", id);
        viewer->addPointCloud<pcl::PointXYZ>(*cloudPtr, s);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, lw, s);
      }
      snprintf(s, MSL, "poly%04d", id);
      if (polyData->isPolygon())
      {
        viewer->addPolygon<pcl::PointXYZ>(*cloudPtr, s);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                double(color >> 16)/256.0,
                                                double((color >> 8) & 0xff)/256.0,
                                                double(color & 0xff)/256.0, s);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lw, s);
      }
      else if (polyData->isPolyline())
      { // is a polyline - typically short
        UPosition pw, *p;
        pcl::PointXYZ pcp1, pcp2;
        //
        if (cloud == NULL)
        {
          cloud = new pcl::PointCloud<pcl::PointXYZ>();
          cloudPtr = new pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud);
        }
        cloud->points.resize(polyData->getPointsCnt());
        p = polyData->getPoints();
        if (polyData->cooSys != cooSysView)
          pw = cooSysViewOrigin.getPoseToMap(*p);
        else
          pw = *p;
        pcp2.x = pw.x;
        pcp2.y = pw.y;
        pcp2.z = pw.z;
        p++;
        for (int i = 1; i < polyData->getPointsCnt(); i++)
        {
          if (polyData->cooSys != cooSysView)
            pw = cooSysViewOrigin.getPoseToMap(*p);
          else
            pw = *p;
          pcp1.x = pw.x;
          pcp1.y = pw.y;
          pcp1.z = pw.z;
          // add one line
          snprintf(s, MSL, "poly%04d_%03d", id, i);
          viewer->addLine<pcl::PointXYZ,pcl::PointXYZ>(pcp1, pcp2, s);
          viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  double(color >> 16)/256.0,
                                                  double((color >> 8) & 0xff)/256.0,
                                                  double(color & 0xff)/256.0, s);
          viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lw, s);
          p++;
          pcp2 = pcp1;
        }
      }
      isShown = true;
    }
  };
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class UPaintPolygons
{
public:
  /// valid point clouds
  UPaintPolygon ** clouds;
  /// number of clouds
  int cloudsCnt;
  /// number of clouds
  int cloudsMax;
  /// constructor
  UPaintPolygons()
  {
    cloudsMax = 0;
    cloudsCnt = 0;
    clouds = NULL;
  }
  /// destructor
  ~UPaintPolygons()
  {
    if (clouds != NULL)
    {
      for (int i = 0; i < cloudsCnt; i++)
      {
        if (clouds[i] != NULL)
          delete clouds[i];
      }
      delete clouds;
    }
  }
  /// remove all clouds
  void removeAll()
  {
    cloudsCnt = 0;
  };
  /// unpaint all
  void unPaintAll(pcl::visualization::PCLVisualizer* viewer)
  {
    for (int i = 0; i < cloudsCnt; i++)
      clouds[i]->unPaint(viewer);
  }
  /// expand count of clouds
  void expand()
  {
    const int EXPAND_BY = 100;
    int n = cloudsMax;
    cloudsMax += EXPAND_BY;
    int m = cloudsMax * sizeof(UPaintPolygon **);
    clouds = (UPaintPolygon **)realloc(clouds, m);
    bzero(&clouds[n], m - n  * sizeof(UPaintPolygons **));    
  }
  /**
   * Set or update polygon
   * \param viewer is the viewer
   * \param idx is the index number for the polygon (as source data index)
   * \param pi is pointer to source data
   * \param show if false, then polygon is hidden
   * \param color is 3x8bit RGB value
   * \param bold if true, then paint with large line width
   * \param dots if true, then also show vertices as dots (point-cloud)
   * \param id is the name of the polygon
   * \param viewSys is the (new) coordinate system of the viewer
   * \param sysPose is the origin of the source coordinate system, if not the same as the 
   * source coordinate system (in pi.cooSys) */
  void setPolygon(pcl::visualization::PCLVisualizer* viewer,
                  int idx, UPolyItem * pi,
                  bool show, uint32_t color, bool bold, bool dots,
                  const char * id, int viewSys, UPose sysPose)
  {
    UPaintPolygon * pc;
    int pcnt;
    bool newName;
    if (idx < 0 or idx > 64000)
    { // debug
      printf("UPaintPolygons::setPolygon index error\n");
      return;
    }
    while (idx >= cloudsMax)
      expand();
    if (idx >= cloudsCnt)
      cloudsCnt = idx + 1;
    // get polygon data
    pc = clouds[idx];
    if (pc == NULL)
    { // new
      pc = new UPaintPolygon();
      clouds[idx] = pc;
    }
    if (pc->cloud != NULL)
      pcnt = pc->cloud->size();
    else
      pcnt = 0;
    if (id == NULL)
      newName = pc->name != NULL;
    else if (pc->name == NULL)
      newName = true;
    else
      newName = strcmp(id, pc->name) != 0;
    if ((viewSys != pi->cooSys) or
        (show != pc->isShown) or
        (pi->getPointsCnt() != pcnt) or
        (color != pc->color) or
        (dots != pc->dots) or
        (bold != pc->bold) or
        newName or
        (pi->isPolyline() != pc->isPolyline))
    { // something has to be done
      pc->unPaint(viewer);
      pc->bold = bold;
      pc->name = id;
      pc->color = color;
      pc->dots = dots;
      pc->id = idx;
      pc->isPolyline = pi->isPolyline();
      pc->setViewCooSys(viewSys, sysPose);
      pc->addPolyToViewer(pi, viewer);
    }
  }
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class UPaintLaserScan
{
public:
  /// point cloud for the scan
  pcl::PointCloud<pcl::PointXYZRGB> * cloud;
  /// boost pointer to the cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr * cloudPtr;
  /// scan color handler 
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> * rgbHandler;
  /// scannumber for ID
  int scannumber;
  ///
  UPaintLaserScan()
  {
    cloud = NULL;
    cloudPtr = NULL;
    scannumber = -1;
  }
  ~UPaintLaserScan()
  {
    if (rgbHandler != NULL)
      delete rgbHandler;
    if (cloudPtr != NULL)
      delete cloudPtr;
    if (cloud != NULL)
      delete cloud;
  }
  /**
   * Paint this scan into a pointcloud */
  void makeScanCloud(ULaserDataSet * scan,
                     UPose * viewPoseAtScantime)
  {
    int i;
    //CvPoint p1;
    UPosition posm, posr, pos;
    UClientLaserData * pd;
  //  uint32_t pixGray = (100 << 16) + (100 << 8) + 100;
  //  uint32_t pixGreen = (15 << 16) + (210 << 8 ) + 15;
    uint32_t lgreen = (90 << 16) + (210 << 8) + (70);
    UPose poseScan;
    UMatrix4 mLtoR, mRtoM, mLtoM;
    pcl::PointXYZRGB pcp1;
    if (cloud == NULL)
    {
      cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
      cloudPtr = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud);
      rgbHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(*cloudPtr);
    }
    scannumber = scan->getSerial();
    //
    // make a transformation matrix from laser coordinates to robot coordinates.
    mLtoR = scan->getSensorPose()->getRtoMMatrix();
    mRtoM = viewPoseAtScantime->asMatrix4x4PtoMPos();
    mLtoM = mRtoM * mLtoR;
    // get data reference of first measurement
    pd = scan->getData();
    pcp1.rgb = *reinterpret_cast<float*>(&lgreen);
    cloud->points.resize(scan->getCount());
    for (i = 0; i < scan->getCount(); i++)
    {
      if (pd->isValid() and pd->getDistance() < scan->getMaxValidRange()) // or (pz > 1))
      { // get position in view-map coordinates
        posr = pd->getPosition(&mLtoM);
        pcp1.x = posr.x;
        pcp1.y = posr.y;
        pcp1.z = posr.z;
        cloud->points[i] = pcp1;
      }
      pd++;
    }
  };
  /**
   * Remove point cloud from viewer */
  void unPaint(pcl::visualization::PCLVisualizer * viewer)
  {
    if (scannumber < 0)
      return;
    const int MSL = 15;
    char s[MSL];
    snprintf(s, MSL, "scan%06d", scannumber);
    viewer->removePointCloud(s);
    scannumber = -1;
  }
  /**
   * Remove point cloud from viewer */
  void setBold(pcl::visualization::PCLVisualizer * viewer, int pointSize)
  {
    if (scannumber < 0)
      return;
    const int MSL = 15;
    char s[MSL];
    snprintf(s, MSL, "scan%06d", scannumber);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, s);
  }
  /**
   * add cloud to the viewer - removing the older
   * \param scan is scan to convert, if NULL, then old scan is removed from viewer.
   * \param viewPoseAtScantime is the pose in the viewer coordinate system when then scan was valid
   * \param viewer is the 3D viewer */
  void addScanToViewer(ULaserDataSet * scan,
                     UPose * viewPoseAtScantime, pcl::visualization::PCLVisualizer * viewer)
  {
    if (scannumber >= 0)
      unPaint(viewer);
    if (scan != NULL)
    {
      makeScanCloud(scan, viewPoseAtScantime);
      if (cloud->points.size() > 0)
      {
        const int MSL = 15;
        char s[MSL];
        snprintf(s, MSL, "scan%06d", scannumber);
        viewer->addPointCloud<pcl::PointXYZRGB>(*cloudPtr, *rgbHandler, s, 0);
      }
    }
  };
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class UPaintLaserScans : public UPaintBase
{
public:
  /// valid point clouds
  UPaintLaserScan ** clouds;
  /// number of clouds
  int cloudsCnt;
  /// number of clouds
  int cloudsMax;
  
  UPaintLaserScans()
  {
    cloudsMax = 100;
    cloudsCnt = 0;
    int n = cloudsMax * sizeof(UPaintLaserScan **);
    clouds = (UPaintLaserScan **)malloc(n);
    memset(clouds, 0, n);
  }
  /// remove all clouds
  void removeAll()
  {
    cloudsCnt = 0;
  };
  /// remove this number of oldest scans
  void remove(int scans, pcl::visualization::PCLVisualizer* viewer)
  {
    int n = mini(scans, cloudsCnt);
    int empty;
    for (empty = cloudsCnt; empty < cloudsMax; empty++)
    {
      if (clouds[empty] == NULL)
        break;
    } 
    for (int i = 0; i < n; i++)
    { // remove oldest
      if (empty == cloudsMax)
        expand();
      clouds[i]->unPaint(viewer);
      clouds[empty++] = clouds[i];
    }
    // move used pointers to atart of array
    memmove(clouds, &clouds[scans], (empty - scans) * sizeof(UPaintLaserScan **));
    cloudsCnt -= scans;
    // clear leftover pointers
    memset(&clouds[empty - scans], 0, scans * sizeof(UPaintLaserScan **));
  };
  /// unpaint all
  void unPaintAll(pcl::visualization::PCLVisualizer* viewer)
  {
    for (int i = 0; i < cloudsCnt; i++)
      clouds[i]->unPaint(viewer);
  }
  /// expand count of clouds
  void expand()
  {
    int n = cloudsMax;
    cloudsMax += 100;
    int m = cloudsMax * sizeof(UPaintLaserScan **);
    clouds = (UPaintLaserScan **)realloc(clouds, m);
    bzero(&clouds[n], m - n  * sizeof(UPaintLaserScan **));    
  }
  /// add a scan
  void addScan(ULaserDataSet * scan,
                     UPose * viewPoseAtScantime, 
                     pcl::visualization::PCLVisualizer* viewer,
                     int pointSize
              )
  {
    if (cloudsCnt >= cloudsMax)
    { // allocate more space
      expand();
    }
    UPaintLaserScan * sc = clouds[cloudsCnt];
    if (sc == NULL)
    {
      sc = new UPaintLaserScan();
      clouds[cloudsCnt] = sc;
    }
    cloudsCnt++;
    sc->addScanToViewer(scan, viewPoseAtScantime, viewer);
    sc->setBold(viewer, pointSize);
  }
  /**
   * Set newest scan as history scan (i.e. unbold) */
  void ageNewestScan(pcl::visualization::PCLVisualizer* viewer,
                     int pointSize)
  {
    if (cloudsCnt > 0)
    {
      UPaintLaserScan * sc = clouds[cloudsCnt - 1];
      sc->setBold(viewer, pointSize);
    }
  }
  /**
   * Get newest scan serial number */
  int getNewestSerial()
  {
    int result;
    if (cloudsCnt > 0)
      result = clouds[cloudsCnt-1]->scannumber;
    else
      result = -1;
    return result;
  }
};



////////////////////////////////////////////////////////////////////////////////////

class UPaintPoseHist : public UPaintBase
{
public:
  /// pose hist resource to paint
  UResPoseHist * path;
  /// point cloud for the path
  pcl::PointCloud<pcl::PointXYZ> * cloud;
  /// boost pointer to the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr * cloudPtr;
  /// name of line in viewer
  const char * idName;
  /// mewest pose shown
  int poseNewest;
  ///is cloud added to viewer
  bool isInViewer;
  /// 
  UPaintPoseHist(UResPoseHist * poseHistRes, int sourceIndex)
  {
    path = poseHistRes;
    poseNewest = -1;
    cooSysSource = sourceIndex;
    cooSysView = 0;
    maxHist = 30;
    cloud = new pcl::PointCloud<pcl::PointXYZ>();
    cloudPtr = new pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud);
    if (path != NULL)
      idName = path->getResID();
    isInViewer = false;
  };
  /**
   * destructor */
  ~UPaintPoseHist()
  {
  }
  
  void paint(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer)
  // void paint(UPoseTime seenFromPose, 
  {
    if (path == NULL)
      return;
    if (path->getPosesCnt() == 0)
      return;
    UPosition posm, posr, pos;
    UPoseTime pose, pose2;
    UPoseTime lastPose;
    pcl::PointXYZ pc1;
    pc1.z = 0.0;
    //
    path->lock();
    //
    int newCnt;
    if (poseNewest == -1)
      newCnt = path->getPosesCnt();
    else
    {
      newCnt = path->getNewestIndex() - poseNewest;
      if (newCnt < 0)
        newCnt += MAX_HIST_POSES;
    }
    if (newCnt > maxHist)
      newCnt = maxHist;
    int newSize = newCnt + cloud->points.size();
    if (newSize > maxHist)
    { // remove surplus points from cloud
      std::vector <pcl::PointXYZ, Eigen::aligned_allocator_indirection <pcl::PointXYZ > >::iterator i1, i2;
      i1 = cloud->points.begin();
      i2 = i1 + newSize - maxHist;
      // remove points from first (oldest) part of cloud
      cloud->points.erase(i1, i2);
    }
    //m = paintPoseHistVecCnt;
    // get pixel position of start pose
    for (int i = newCnt - 1; i >= 0; i--)
    { // get pose with this age - oldest first
      pose = path->getPose(i);
      if (cooSysSource != cooSysView)
        // convert from other coordinate system
        pose = cooSysViewOrigin.getPoseToMapPose(pose);
      pc1.x = pose.x;
      pc1.y = pose.y;
      cloud->points.push_back(pc1);
      poseNewest = path->getNewestIndex();
    }
    if (isInViewer)
    { // remove old cloud
      viewer->removePointCloud(idName);
      viewer->removeShape(idName);
      isInViewer = false;
    }
    if (cloud->points.size() > 0)
    { // add new/modified cloud
      viewer->addPointCloud<pcl::PointXYZ>(*cloudPtr, idName);
      viewer->addPolygon<pcl::PointXYZ>(*cloudPtr, idName);
      isInViewer = true;
      switch (cooSysSource)
      {
        case 1:
          viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.6, 0.6, idName);
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.6, 0.6, idName);
          break;
        case 2:
          viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.0, 0.7, idName);
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.0, 0.6, idName);
          break;
        default:
          viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.4, 0.4, idName);
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, idName);
          break;
      }
      if (bold)
      {
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, idName);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, idName);
      }
      else
      {
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, idName);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, idName);
      }
    }
    path->unlock();
  }
};

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


void UPaintBase::makeCircleCloud(pcl::PointCloud< pcl::PointXYZ >* cloud, UMatrix4* pose, double radius)
{
  UPosition p1, p2;
  double angle = 0.0;
  if (cloud->points.size() == 0)
    cloud->resize(36);
  for (int i = 0; i < 36; i++)
  { // all around 360 deg
    p1.x = 0.0;
    p1.y = radius * cos(angle);
    p1.z = radius * sin(angle);
    // convert to current coordinate system
    p2 = *pose * p1;
    cloud->points[i].x = p2.x;
    cloud->points[i].y = p2.y;
    cloud->points[i].z = p2.z;
    angle += M_PI/18.0;
  }
}

////////////////////////////////////////////////////////

void UPaintBase::addCircleCloud(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr * cloud, 
                             double x, double y, double h, double radius, const char * id)
{
  UPose pw = currentPose->getPoseToMapPose(x, y, h);
  UPosRot pr;
  UMatrix4 tr;
  if (cooSysSource != cooSysView)
    // convert pose to view system
    pw = cooSysViewOrigin.getPoseToMapPose(pw);
  pr.setFromPose(pw.x, pw.y, pw.h);
  pr.z = radius;
  tr = pr.getRtoMMatrix();
  makeCircleCloud(cloud->get(), &tr, radius);
  viewer->addPointCloud<pcl::PointXYZ>(*cloud, id);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, id);
}

///////////////////////////////////////////////////////

void UPaintBase::addWheel(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer, 
                          pcl::ModelCoefficients * params,
                          double x, double y, double h, double radius, double width, const char * id)
{
  UPose pw = currentPose->getPoseToMapPose(x, y, h);
  UPose pwc(-width/2.0, 0, 0);
  // inner point
  UPose pww = pw + pwc;
  pwc.x +=width;
  // outher point
  pw = pw + pwc;
  if (cooSysSource != cooSysView)
  {  // convert pose to view system
    pw = cooSysViewOrigin.getPoseToMapPose(pw);
    pww = cooSysViewOrigin.getPoseToMapPose(pw);
  }
  params->values.resize(9);
  params->values[0] = pw.x;
  params->values[1] = pw.y;
  params->values[2] = radius;
  params->values[3] = pww.x - pw.x;
  params->values[4] = pww.y - pw.y;
  params->values[5] = 0;
  params->values[6] = radius;
  params->values[7] = 0.5; // height?
  params->values[8] = 1.0; // capped?
  viewer->addCylinder(*params, id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id);
  //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
  const int MIDL = 20;
  char sid[MIDL];
  snprintf(sid, MIDL, "%s1", id);
  params->values[6] = radius * 1.05;
  viewer->addCylinder(*params, sid);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0.4, 0.4, sid);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, sid);
}

////////////////////////////////////////////////////////////////////////////////////

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
void UPaintBase::addCylinder(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer,
                          pcl::ModelCoefficients * params,
                          double x1, double y1, double z1,
                          double x2, double y2, double z2,
                          double radius, const char * id, const double color[3])
{
  UPose pw1 = currentPose->getPoseToMapPose(x1, y1, 0);
  UPose pw2 = currentPose->getPoseToMapPose(x2, y2, 0);
  if (cooSysSource != cooSysView)
  {  // convert pose to view system
    pw1  = cooSysViewOrigin.getPoseToMapPose(pw1);
    pw2  = cooSysViewOrigin.getPoseToMapPose(pw2);
  }
  // resize parameter vector
  params->values.resize(9);
  params->values[0] = (pw1.x + pw2.x)/2.0; // center position
  params->values[1] = (pw1.y + pw2.y)/2.0;
  params->values[2] = z1;
  // direction
  params->values[3] = (pw2.x - pw2.x); // vector to other center
  params->values[4] = (pw2.y - pw2.y);
  params->values[5] = (z2 - z1)/2.0;
  params->values[6] = radius; // radius
  params->values[7] = 0.5; // height?
  params->values[8] = 1.0; // capped?
  viewer->addCylinder(*params, id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2], id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id);
  //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
}

///////////////////////////////////////////////////////

void UPaintBase::addBox(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer, 
                          pcl::ModelCoefficients * params,
                          double x, double y, double z, 
                          const char * id, const double color[3])
{
  Eigen::Matrix3f m;
  double h;
  // paint robot body
  // x,y,z position - of center of body-box
  UPosition p1(x, y, z), p2;
  p2 = currentPose->getPoseToMap(p1);
  h = currentPose->h;
  if (cooSysSource != cooSysView)
  { // convert to view system
    p2 = cooSysViewOrigin.getPoseToMap(p2);
    h -= cooSysViewOrigin.h;
  }
  m = Eigen::AngleAxisf(currentPose->h, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q(m);
  params->values[0] = p2.x; // x-forward
  params->values[1] = p2.y; // y (left)
  params->values[2] = p2.z; // z height
  // rotation quaternion x,y,z,w
  params->values[3] = q.x();
  params->values[4] = q.y();
  params->values[5] = q.z();
  params->values[6] = q.w();
  viewer->addCube(*params, id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2], id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
  const int MIDL = 20;
  char sid[MIDL];
  snprintf(sid, MIDL, "%s1", id);
  viewer->addCube(*params, sid);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, sid);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, sid);
}

//////////////////////////////////////////////////////////////////////

class URobotMmr : public UPaintBase
{
private:
  pcl::ModelCoefficients lowerBody;
  pcl::ModelCoefficients upperBody;
  pcl::ModelCoefficients camFrame[3];
  pcl::ModelCoefficients laser;
  pcl::ModelCoefficients wheelL;
  pcl::ModelCoefficients wheelR;
  pcl::ModelCoefficients wheelF;

public:
  
  URobotMmr()
  {
    lowerBody.values.resize(10, 0.0);
    lowerBody.values[7] = 0.9; // x - length
    lowerBody.values[8] = 0.4; // y - width
    lowerBody.values[9] = 0.2; // z - height
    upperBody.values.resize(10, 0.0);
    upperBody.values[7] = 0.4;    upperBody.values[8] = 0.3;   upperBody.values[9] = 0.3;
    camFrame[0].values.resize(10, 0.0);
    camFrame[1].values.resize(10, 0.0);
    camFrame[2].values.resize(10, 0.0);
    camFrame[0].values[7] = 0.05;     camFrame[0].values[8] = 0.05;    camFrame[0].values[9] = 0.6;
    camFrame[1].values[7] = 0.05;     camFrame[1].values[8] = 0.05;    camFrame[1].values[9] = 0.6;
    camFrame[2].values[7] = 0.10;     camFrame[2].values[8] = 0.5;    camFrame[2].values[9] = 0.06;
    laser.values.resize(10, 0.0);
    laser.values[7] = 0.1;            laser.values[8] = 0.1;       laser.values[9] = 0.15;
  }
  
  ~URobotMmr()
  {
  }
  /** paint MMR robot */
  void paint(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer)
  {
    double colorAlu[3] = {0.5, 0.5, 0.5};
    double colorBlue[3] = {0.1, 0.1, 1.0};
    addBox(currentPose, viewer, &camFrame[0], 0.35, 0.2, 0.45, "camFrame1", colorAlu);
    addBox(currentPose, viewer, &camFrame[1], 0.35, -0.2, 0.45, "camFrame2", colorAlu);
    addBox(currentPose, viewer, &camFrame[2], 0.35, 0.0, 0.75, "camFrame3", colorAlu);
    addBox(currentPose, viewer, &upperBody, 0.0, 0.0, 0.45, "upperBody", colorAlu);
    addBox(currentPose, viewer, &laser, 0.43, 0.0, 0.40, "laser", colorBlue);
    addBox(currentPose, viewer, &lowerBody, 0.17, 0.0, 0.20, "robotBody", colorAlu);
    // wheels are black
    addWheel(currentPose, viewer, &wheelL, 0.0,  0.25,  M_PI/2.0, 0.15, 0.07, "leftWheel");
    addWheel(currentPose, viewer, &wheelR, 0.0, -0.25, -M_PI/2.0, 0.15, 0.07, "rightWheel");
    addWheel(currentPose, viewer, &wheelL, 0.43, 0.00, M_PI/2.0, 0.08, 0.06, "frontWheel");
  };

  ////////////////////////////////////////

  void unPaint(pcl::visualization::PCLVisualizer* viewer) 
  {
    viewer->removeShape("rightWheel");
    viewer->removeShape("leftWheel");
    viewer->removeShape("frontWheel");
    viewer->removeShape("robotBody");
    viewer->removeShape("camFrame1");
    viewer->removeShape("camFrame2");
    viewer->removeShape("camFrame3");
    viewer->removeShape("upperBody");
    viewer->removeShape("laser");
    viewer->removeShape("rightWheel1");
    viewer->removeShape("leftWheel1");
    viewer->removeShape("frontWheel1");
    viewer->removeShape("robotBody1");
    viewer->removeShape("camFrame11");
    viewer->removeShape("camFrame21");
    viewer->removeShape("camFrame31");
    viewer->removeShape("upperBody1");
    viewer->removeShape("laser1");
  };

  /////////////////////////////////////////

  const char * name() { return "mmr"; };

};


class URobotIrobot : public UPaintBase
{
private:
  pcl::ModelCoefficients lowerBody;
  pcl::ModelCoefficients wheel[4];
  pcl::ModelCoefficients camFrame[2];
  pcl::ModelCoefficients laser;

public:
  URobotIrobot()
  {
    lowerBody.values.resize(10, 0.0);
    lowerBody.values[7] = 0.64; // x - length
    lowerBody.values[8] = 0.4; // y - width
    lowerBody.values[9] = 0.3; // z - height
    camFrame[0].values.resize(10, 0.0);
    camFrame[0].values[7] = 0.06;
    camFrame[0].values[8] = 0.22;
    camFrame[0].values[9] = 0.04;
    camFrame[1].values.resize(10, 0.0);
    camFrame[1].values[7] = 0.05;
    camFrame[1].values[8] = 0.14;
    camFrame[1].values[9] = 0.03;
    for (int i = 0; i < 4; i++)
    { //cylinder wheels
      wheel[i].values.resize(9, 0.0);
      wheel[i].values[2] = 0.15; // height over ground
      wheel[i].values[6] = 0.3; // diameter
      wheel[i].values[7] = 0.05; // height - may be not used
      wheel[i].values[8] = 1.0; // capped - may be not used
    }
    laser.values.resize(10, 0.0);
    laser.values[7] = 0.1;            laser.values[8] = 0.1;       laser.values[9] = 0.15;
  }
  /** paint MMR robot */
  void paint(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer)
  {
    double colorRed[3] = {0.6, 0.2, 0.2};
    double colorBlue[3] = {0.1, 0.1, 1.0};
    double colorBlack[3] = {0.1, 0.1, 0.1};
    double colorAlu[3] = {0.5, 0.5, 0.5};
    addBox(currentPose, viewer, &camFrame[0], 0.10, 0.0, 0.45, "icamKinect", colorBlack);
    addBox(currentPose, viewer, &camFrame[1], 0.15, 0.0, 0.52, "icamGuppy", colorAlu);
    addBox(currentPose, viewer, &laser, 0.43, 0.0, 0.30, "ilaser", colorBlue);
    addBox(currentPose, viewer, &lowerBody, 0.0, 0.0, 0.25, "irobotBody", colorRed);
    // wheels are black
    addWheel(currentPose, viewer, &wheel[0], -0.25,  0.25,  M_PI/2.0, 0.15, 0.10, "ileftWheelback");
    addWheel(currentPose, viewer, &wheel[1], -0.25,  -0.25,  M_PI/2.0, 0.15, 0.10, "irightWheelBack");
    addWheel(currentPose, viewer, &wheel[2], 0.25,  0.25,  M_PI/2.0, 0.15, 0.10, "ileftWheelFront");
    addWheel(currentPose, viewer, &wheel[3], 0.25,  -0.25,  M_PI/2.0, 0.15, 0.10, "irightWheelFront");
  };

  ////////////////////////////////////////

  void unPaint(pcl::visualization::PCLVisualizer* viewer) 
  {
    viewer->removeShape("ileftWheelback");
    viewer->removeShape("irightWheelBack");
    viewer->removeShape("ileftWheelFront");
    viewer->removeShape("irightWheelFront");
    viewer->removeShape("irobotBody");
    viewer->removeShape("ilaser");
    viewer->removeShape("icamGuppy");
    viewer->removeShape("icamKinect");
    // also extra features - same name with added '1'
    viewer->removeShape("ileftWheelback1");
    viewer->removeShape("irightWheelBack1");
    viewer->removeShape("ileftWheelFront1");
    viewer->removeShape("irightWheelFront1");
    viewer->removeShape("irobotBody1");
    viewer->removeShape("ilaser1");
    viewer->removeShape("icamGuppy1");
    viewer->removeShape("icamKinect1");
  };

  /////////////////////////////////////////

  const char * name() 
  { 
    return "irobot"; 
  };
};

///////////////////////////////////////////////////////////////////////////

class URobotHako : public UPaintBase
{
private:
  pcl::ModelCoefficients wheelRL;
  pcl::ModelCoefficients wheelRR;
  pcl::ModelCoefficients wheelFL;
  pcl::ModelCoefficients wheelFR;
  pcl::ModelCoefficients backBody;
  pcl::ModelCoefficients frontBody;
  pcl::ModelCoefficients frontAx;
  pcl::ModelCoefficients backTop;
  pcl::ModelCoefficients laser;

public:
  URobotHako()
  {
    backBody.values.resize(10, 0.0);
    backBody.values[7] = 1.15; // x - length
    backBody.values[8] = 0.85; // y - width
    backBody.values[9] = 0.75; // z - height
    frontBody.values.resize(10, 0.0);
    frontBody.values[7] = 0.8;    frontBody.values[8] = 0.5;   frontBody.values[9] = 0.5;
    frontAx.values.resize(10, 0.0);
    frontAx.values[7] = 0.1;    frontAx.values[8] = 1.16;   frontAx.values[9] = 0.07;
    backTop.values.resize(10, 0.0);
    backTop.values[7] = 1.2;    backTop.values[8] = 0.9;   backTop.values[9] = 0.05;
    laser.values.resize(10, 0.0);
    laser.values[7] = 0.1;            laser.values[8] = 0.1;       laser.values[9] = 0.15;
  }
  /** paint Hako robot */
  void paint(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer)
  {
    double colorRed[3] = {1.0, 0.0, 0.0};
    double colorGray[3] = {0.7, 0.7, 0.7};
    double colorBlue[3] = {0.1, 0.1, 1.0};
    double colorBlack[3] = {0.1, 0.1, 0.1};
    const double frontAxleX = 1.95 - 0.26 - 0.45;
    const double frontWheelRadius = 0.275;
    addBox(currentPose, viewer, &backBody,  0.16, 0.0, 0.775, "backBody", colorRed);
    addBox(currentPose, viewer, &frontBody, 0.99, 0.0, 0.65, "frontBody", colorRed);
    addBox(currentPose, viewer, &laser,     1.58, 0.0, 0.55, "laser", colorBlue);
    addBox(currentPose, viewer, &backTop,     0.16, 0.0, 1.175, "backTop", colorGray);
    addBox(currentPose, viewer, &frontAx,   frontAxleX, 0.0, frontWheelRadius, "frontAx", colorBlack);
    addWheel(currentPose, viewer, &wheelFL, frontAxleX,  0.5 + 0.08,  M_PI/2.0, 0.26, 0.08, "FLWheel");
    addWheel(currentPose, viewer, &wheelFR, frontAxleX, -0.5 - 0.08, -M_PI/2.0, 0.26, 0.08, "FRWheel");
    addWheel(currentPose, viewer, &wheelRL, 0.0,  0.51,  M_PI/2.0, 0.46, 0.25, "RLWheel");
    addWheel(currentPose, viewer, &wheelRR, 0.0, -0.51, -M_PI/2.0, 0.46, 0.25, "RRWheel");
  };

  ////////////////////////////////////////

  void unPaint(pcl::visualization::PCLVisualizer* viewer) 
  {
    viewer->removeShape("FLWheel");
    viewer->removeShape("FRWheel");
    viewer->removeShape("RLWheel");
    viewer->removeShape("RRWheel");
    viewer->removeShape("laser");
    viewer->removeShape("backBody");
    viewer->removeShape("frontBody");
    viewer->removeShape("frontAx");
    viewer->removeShape("backTop");
    // also extra features - same name with added '1'
    viewer->removeShape("FLWheel1");
    viewer->removeShape("FRWheel1");
    viewer->removeShape("RLWheel1");
    viewer->removeShape("RRWheel1");
    viewer->removeShape("laser1");
    viewer->removeShape("backBody1");
    viewer->removeShape("frontBody1");
    viewer->removeShape("frontAx1");
    viewer->removeShape("backTop1");
  };

  /////////////////////////////////////////

  const char * name() { return "hako"; };
};

///////////////////////////////////////////////////////////////////////////

class URobotGuidebot : public UPaintBase
{
private:
  pcl::ModelCoefficients Bdylower;
  pcl::ModelCoefficients BdyMid;
  pcl::ModelCoefficients BdyTop;
  pcl::ModelCoefficients wheel[2];
  pcl::ModelCoefficients laser;
  

public:
  URobotGuidebot()
  { // intialize size parameters
    laser.values.resize(10, 0.0);
    laser.values[7] = 0.1;
    laser.values[8] = 0.1;
    laser.values[9] = 0.15;
    BdyMid.values.resize(10, 0.0);
    BdyMid.values[7] = 0.30; // x - length
    BdyMid.values[8] = 0.30; // y - width
    BdyMid.values[9] = 0.75; // z - height
    BdyTop.values.resize(10, 0.0);
    BdyTop.values[7] = 0.25; // x - length
    BdyTop.values[8] = 0.25; // y - width
    BdyTop.values[9] = 0.20; // z - height
  }
  /** paint robot */
  void paint(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer)
  {
//    double colorRed[3] = {1.0, 0.0, 0.0};
//    double colorGray[3] = {0.7, 0.7, 0.7};
    double colorBlue[3] = {0.1, 0.1, 1.0};
    double colorBlack[3] = {0.1, 0.1, 0.1};
    addCylinder(currentPose, viewer, &Bdylower, 0, 0, 0.03,    0, 0, 0.22, 0.30, "guideLower", colorBlack);
    addBox(currentPose, viewer, &BdyMid,   0, 0, 0.47, "guideMid", colorBlack);
    addBox(currentPose, viewer, &BdyTop, 0, 0, 0.95,    "guideTop", colorBlack);
    addBox(currentPose, viewer, &laser,  0.25, 0.0, 0.25, "Glaser", colorBlue);
    addWheel(currentPose, viewer, &wheel[0], 0.0,  0.25 + 0.02,  M_PI/2.0, 0.06, 0.04, "GLWheel");
    addWheel(currentPose, viewer, &wheel[1], 0.0, -0.25 - 0.02, -M_PI/2.0, 0.06, 0.04, "GRWheel");
  };

  ////////////////////////////////////////

  void unPaint(pcl::visualization::PCLVisualizer* viewer)
  {
    viewer->removeShape("GLWheel");
    viewer->removeShape("GRWheel");
    viewer->removeShape("Glaser");
    viewer->removeShape("guideTop");
    viewer->removeShape("guideMid");
    viewer->removeShape("guideLower");
    // also extra features - same name with added '1'
    viewer->removeShape("GLWheel1");
    viewer->removeShape("GRWheel1");
    viewer->removeShape("Glaser1");
    viewer->removeShape("guideTop1");
    viewer->removeShape("guideMid1");
    // viewer->removeShape("guideLower1"); // except cylinders
  };

  /////////////////////////////////////////

  const char * name() 
  { return "guidebot"; };
};

///////////////////////////////////////////////////////////////////////////

class URobotSmr : public UPaintBase
{
private:
  pcl::ModelCoefficients Bdy;
  pcl::ModelCoefficients wheel[2];
  pcl::ModelCoefficients laser;


public:
  URobotSmr()
  { // intialize size parameters
    laser.values.resize(10, 0.0);
    laser.values[7] = 0.05;
    laser.values[8] = 0.05;
    laser.values[9] = 0.05;
    Bdy.values.resize(10, 0.0);
    Bdy.values[7] = 0.28; // x - length
    Bdy.values[8] = 0.28; // y - width
    Bdy.values[9] = 0.15; // z - height
  }
  /** paint robot */
  void paint(UPose * currentPose, pcl::visualization::PCLVisualizer* viewer)
  {
//    double colorRed[3] = {1.0, 0.0, 0.0};
    double colorGray[3] = {0.7, 0.7, 0.7};
    double colorBlue[3] = {0.1, 0.1, 1.0};
    //double colorBlack[3] = {0.1, 0.1, 0.1};
    addBox(currentPose, viewer, &Bdy,   0.1, 0, 0.12, "SMid", colorGray);
    addBox(currentPose, viewer, &laser,  0.25, 0.0, 0.037, "Slaser", colorBlue);
    addWheel(currentPose, viewer, &wheel[0], 0.0,  0.14 + 0.025,  M_PI/2.0, 0.035, 0.03, "SLWheel");
    addWheel(currentPose, viewer, &wheel[1], 0.0, -0.14 - 0.025, -M_PI/2.0, 0.035, 0.03, "SRWheel");
  };

  ////////////////////////////////////////

  void unPaint(pcl::visualization::PCLVisualizer* viewer)
  {
    viewer->removeShape("SLWheel");
    viewer->removeShape("SRWheel");
    viewer->removeShape("Slaser");
    viewer->removeShape("SMid");
    // also extra features - same name with added '1'
    viewer->removeShape("SLWheel1");
    viewer->removeShape("SRWheel1");
    viewer->removeShape("Slaser1");
    viewer->removeShape("SMid1");
  };

  /////////////////////////////////////////

  const char * name() { return "smr"; };
};

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

UNavView::UNavView()
{
  obsts = NULL;
  resObst = NULL;
  road = NULL;
  varRoot = NULL;
  imgPool = NULL;
  poseOdo = NULL;
  camPath = NULL;
  laserData = NULL;
  resLaserData = NULL;
  laserFeatures = NULL;
  camGmk = NULL;
  camCam = NULL;
  resPoly = NULL;
  // default paint setting
  robotPose.set(1.5, 0, 0); 
  for (int i = 0; i < PAINT_POSE_HIST_CNT; i++)
    poseHist[i] = NULL;
  laserClouds = NULL;
  polyClouds = NULL;
  manoeuvreClouds = NULL;
  //
  // paint filter default
  paintBold = false;
  paintPathLinesCnt = 1;
  paintPathMidPoses = true;
  paintPoseHistCnt = 20;
  paintPoseHistVecCnt = -1;
  paintPoseHistVecLng = 50;
  paintScanHistCnt = 5;
  paintVisPolyCnt = 1;
  paintGridSize[0] = 1.0;
  paintGridSize[1] = -1.0;
  paintGridOdo = true;
  paintRoadHistCnt = 100;
  paintRobot = 0;
  rangeRingCnt = 8;
  paintObstCnt = 1;
  paintVar = true;
  paintStructsCnt = 0;
  paintPathSupportLines = true;
  paintPathMidPoses = true;
  paintCam = true;
  paintGmk = true;
  paintPoseRef = 0;
  paintPoly = true;
  paintPolyNameCnt = 5;
  paintOdoPose = true;
  paintMapPose = true;
  paintUtmPose = true;
  paintPolyHide[0] = '\0';
  paintPolyShow[0] = '\0';
  paintPcpHide[0] = '\0';
  paintPcpShow[0] = '\0';
  paintIntervalLines = false;
  paintRoadAll = false;
  paintCurves = false;
  followRobot = true;
  //
  // resource pointers
  poseOdo = NULL;
  poseUtm = NULL;
  poseMap = NULL;
  navMan = NULL;
  resPcp = NULL;
  paintPclsMaxCnt = 0;
  paintPcls = NULL;
  firstTextScannumber = true;
  cooSysLast = -1;
}

//////////////////////////////

UNavView::~UNavView()
{
  if (laserClouds != NULL)
  {
    laserClouds->unPaint(viewer);
    delete laserClouds;
  }
  if (polyClouds != NULL)
  {
    polyClouds->unPaintAll(viewer);
    delete polyClouds;
  }
  for (int i = 0; i < PAINT_POSE_HIST_CNT; i++)
  {
    if (poseHist[i] != NULL)
    {
      poseHist[i]->unPaint(viewer);
      delete poseHist[i];
    }
  }
  if (paintPcls != NULL)
  {
    for (int i = 0; i < paintPclsMaxCnt; i++)
      if (paintPcls[i] != NULL)
        delete paintPcls[i];
    free(paintPcls);
  }
}

//////////////////////////////

bool UNavView::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // odometry pose
    if (remove)
      poseOdo = NULL;
    else if (poseOdo != resource)
      poseOdo = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResPoseHist::getUtmPoseID()))
  { // in UTM coordinates
    if (remove)
      poseUtm = NULL;
    else if (poseUtm != resource)
      poseUtm = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResPoseHist::getMapPoseID()))
  { // im Map coordinates
    if (remove)
      poseMap = NULL;
    else if (poseMap != resource)
      poseMap = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResVarPool::getResClassID()))
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
  else if (resource->isA(UResCamIfPath::getResClassID()))
  { // delete any local
    if (remove)
      camPath = NULL;
    else if (camPath != resource)
      camPath = (UResCamIfPath *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResLaserIfRoad::getResClassID()))
  { // delete any local
    if (remove)
      road = NULL;
    else if (road != resource)
      road = (UResLaserIfRoad *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResLaserIfObst::getResClassID()))
  { // delete any local
    if (remove)
      resObst = NULL;
    else if (resObst != resource)
      resObst = (UResLaserIfObst *)resource;
    else
      result = false;
    if (result)
      obsts = resObst;
  }
  else if (resource->isA("pcp")) // UResPcp::getResClassID()))
  { // delete any local
    if (remove)
      resPcp = NULL;
    else if (resPcp != resource)
      resPcp = (UResPcp *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResLaserIfScan::getResClassID()))
  { // delete any local
    if (remove)
      resLaserData = NULL;
    else if (resLaserData != resource)
      resLaserData = (UResLaserIfScan *)resource;
    else
      result = false;
    if (result)
    {
      if (resLaserData != NULL)
        laserData = resLaserData->getScanHist();
      else
        laserData = NULL;
    }
  }
  else if (resource->isA(UResLaserIfSf::getResClassID()))
  { // delete any local
    if (remove)
      laserFeatures = NULL;
    else if (laserFeatures != resource)
      laserFeatures = (UResLaserIfSf *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResNavIfMan::getResClassID()))
  { // delete any local
    if (remove)
      navMan = NULL;
    else if (navMan != resource)
      navMan = (UResNavIfMan *)resource; 
    else
      result = false;
  }
  else if (resource->isA(UResCamIfCam::getResClassID()))
  { // delete any local
    if (remove)
      camCam = NULL;
    else if (camCam != resource)
      camCam = (UResCamIfCam *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResCamIfGmk::getResClassID()))
  { // delete any local
    if (remove)
      camGmk = NULL;
    else if (camGmk != resource)
      camGmk = (UResCamIfGmk *)resource;
    else
      result = false;
  }
  else if (resource->isA("poly"))
  { // delete any local
    if (remove)
      resPoly = NULL;
    else if (resPoly != resource)
      resPoly = (UResPoly *)resource;
    else
      result = false;
  }
  else
    result = false;
  return result;
}

//////////////////////////////

void UNavView::paint()
{
  int i, n;
  CvPoint p1;
  ULaserDataSet * scan = NULL;
  const int MSL = 100;
  char s[MSL];
//  int pz = 3; // point size (radius in pixels)
  UPoseTime lastPose;
  UPoseTime seenFromPose;
  UProbPoly * poly;
  UPosition posr;
  UPosRot sensorPose;
  UPoseTVQ pvt;
  UResPoseHist * posesys;
  //
  std::vector<pcl::visualization::Camera> cams;
  std::vector<pcl::visualization::Camera>::iterator camsi;
  pcl::visualization::Camera cam;
  viewer->getCameras(cams);
  camsi = cams.begin();
  cam = camsi[0];
  // get window size for text paint
  const int h = cam.window_size[0];
  const int w = cam.window_size[1];
  
  // update new laser measurements
  // laser data comes before other data 
  switch(paintPoseRef)
  {
    case 0:
      if (poseOdo != NULL)
        posesys = poseOdo;
      break;
    case 1:
      if (poseUtm != NULL)
        posesys = poseUtm;
      break;
    case 2:
      if (poseMap != NULL)
        posesys = poseMap;
      break;
    default:
      break;
  }
  if (posesys == NULL)
  {
    printf ("missing pose history system for system %d!\n", paintPoseRef);
    return;
  }
  pvt = posesys->getNewest();
  // get perspective reference
  lastPose = posesys->getPoseAtTime(pvt.t);
  seenFromPose = lastPose;
    // set scale - pixels per meter
    //ppm = img->height() / maxRange;
    // calculate robot position on display in pixels
    //pr.y = img->height() - roundi(ppm * robotPose.x);
    //pr.x = img->width()/2  + roundi(ppm * robotPose.y);
    //
    // seenFromPose.print("seen from pose");
    // printf("scale %.2fm; ppm=%.3f; pr=%dx,%dy; pose = %.2fx,%.2fy,%.4fh\n", maxRange, ppm, pr.x, pr.y, robotPose.x, robotPose.y, robotPose.h);
    //
  if (paintGridOdo)
  { // adjust grid scale
    if (10.0 / paintGridSize[0] > 150)
      paintGridSize[0] *= 10.0;
    if (10.0 / paintGridSize[0] < 1.5)
      paintGridSize[0] /= 10.0;
    if (absd((paintGridSize[0] - paintGridSize[1])/paintGridSize[0]) > 0.1 or
        (seenFromPose.getDistance(paintGridPose) > (5.0 * paintGridSize[0])))
    {
      paintOdoGrid(seenFromPose, paintGridSize[0], paintGridSize[1], false);
      //paintOdoGrid(seenFromPose, paintGridSize[0] * 10.0, paintGridSize[1] * 10.0, true);
      paintGridSize[1] = paintGridSize[0];
      paintGridPose = seenFromPose;
    }
  }
  // range data if laser data
  if (laserData != NULL)
  {
    scan = laserData->getNewest();
    if (scan != NULL)
    {
      sensorPose = *scan->getSensorPose();
      paintRangeRings(&sensorPose, rangeRingCnt);
    }
    else
      sensorPose.set(0.22, 0.0, 0.4, 0.0, 0.0, 0.0);
  }
  else
    scan = NULL;
  // paint statistics data window
  if (scan != NULL)
  {
    if (scan->isStatValid() and paintCurves)
      paintScanStatData(scan, false, true, false);
  }
  // Point cloud from point-cloud-pool
  if (resPcp != NULL)
  { // are there any pcp data
    if (resPcp->getPolysCnt() > 0)
      paintPointClouds(posesys);
//     if (laserClouds == NULL)
//       laserClouds = new UPaintLaserScans();
//     paintScansNewest(laserClouds, laserData);
  }
  //
  if (paintPoly)
  {
    paintPolyItems(seenFromPose);
//    printf("Returned from paintPolyItems()\n");
  }
  //
  // obstacles
  if (false and obsts != NULL)
  {
    obsts->ogLock.lock();
    n = mini(obsts->getGroupsCnt(), paintObstCnt);
    for (i = n - 1; i >= 0; i--)
      // paint from oldest to newest
      paintObstGrp(obsts->getGroupNewest(i), seenFromPose,
                    obsts->getGroupNewest() + i);
    if (paintObstCnt > 0 and obsts->getGroupFixed() != NULL)
      // paint also fixed obstacles
      paintObstGrp(obsts->getGroupFixed(), seenFromPose, -1);
    obsts->ogLock.unlock();
  }
  //
  // laserscans
  if (scan != NULL)
  { // paint history scan data
    if (laserClouds == NULL)
      laserClouds = new UPaintLaserScans();
    paintScansNewest(laserClouds, laserData);
  }
    //
  if (paintIntervalLines and (scan != NULL))
  { // paint other new stuff
    if (scan->getPisCnt() > 0)
      paintPis(scan, seenFromPose, 0);
  }
  // paint robot history path
  
  paintPoseHistLines(seenFromPose);
  // road lines
//   if ((paintRoadHistCnt > 0) and (road != NULL))
//   {
//     if (paintRoadAll)
//       // paint all lines in gray
//       for (n = 0; n < road->getRoadLinesCnt(); n++)
//         paintRoadLine(road->getRoadLine(n), seenFromPose, true);
//     // paint just the current road
//     paintRoadLine(road->getRoadCurrent(0), seenFromPose, false); // left
//     paintRoadLine(road->getRoadCurrent(1), seenFromPose, false); // center
//     paintRoadLine(road->getRoadCurrent(2), seenFromPose, false); // top
//   }
  // navigation planned path
  if ((navMan != NULL) and (paintPathLinesCnt > 0))
  { // path lines and alternative too
    if (navMan->getMansCnt() > 0)
    {
      UClientManSeq ** pMans;
      pMans = navMan->getMans();
      if (navMan->getMansCnt() > 0 and manoeuvreClouds == NULL)
        manoeuvreClouds = new UPaintManoeuvres();
      for (i = 0; i < navMan->getMansCnt(); i++)
      { // paint road edges and drive path
        if (pMans[i] != NULL)
        { // set man data
          UPose sysPose = getSystemOrigin(0, NULL);
          // debug
          // - creates a:
          //*** glibc detected *** /home/aut/mobotware/mobotware-2.1922/build/bin/auclient: malloc(): memory corruption (fast): 0xa5b77c08 ***
          if (false)
                  manoeuvreClouds->paintManData(viewer, pMans[i], i, paintPoseRef, sysPose);
          // debug end
        }
      }
    }
  }
  // vision polygon
  if (camPath != NULL)
  { // paint vision polygon data
    n = mini(paintVisPolyCnt, camPath->getVisDataCnt());
    if (n > 0)
    { // paint old ones first
      for (i = n - 1; i >= 0; i--)
      { // paint first history
        poly = camPath->getVisData(i)->getPoly();
        paintFreePoly(poly, seenFromPose, i > 0);
      }
    }
  }
  // paint planner and variables
  if (paintVar)
      paintVarDataText(paintVar);
  // paint interval lines and scan number
  p1.x = w/2 + 30;
  p1.y = 20 + 13;
  if (paintCurves)
    p1.y += h/5;
  //
  if (laserFeatures != NULL)
    paintFeatures(laserFeatures->getSfPool(), seenFromPose);
  //
  if (scan != NULL)
  { // paint scannumber - top right corner
    int textSize = 14;
    p1.y = 4;
    if (paintBold)
    {
      p1.x = w - 140;
      textSize = 20;
    }
    else
    {
      p1.x = w - 80;
      textSize = 14;
    }
    snprintf(s, MSL, "Scan %u", scan->getSerial());
    if (not firstTextScannumber)
      viewer->removeShape("scanNumber");
    viewer->addText(s, p1.x, p1.y, textSize, 0,0,0, "scanNumber");
    firstTextScannumber = false;
  }
  // paint robot
  if (robot != NULL)
    robot->unPaint(viewer);
  switch (paintRobot)
  {
    case 1:
      if (robot == NULL) robot = new URobotMmr(); 
      else if (not robot->isA("mmr"))
      { // make it an MMR
        delete robot;
        robot = new URobotMmr();
      }
      break;
    case 2:
      if (robot == NULL) robot = new URobotHako(); 
      else if (not robot->isA("hako"))
      { // make it a hako tractor
        delete robot;
        robot = new URobotHako();
      }
      break;
    case 3:
      if (robot == NULL) robot = new URobotIrobot(); 
      else if (not robot->isA("irobot"))
      { // make it a hako tractor
        delete robot;
        robot = new URobotIrobot();
      }
      break;
    case 4:
      if (robot == NULL) robot = new URobotGuidebot(); 
      else if (not robot->isA("guidebot"))
      { // make it a hako tractor
        delete robot;
        robot = new URobotGuidebot();
      }
      break;
    default:      
      if (robot == NULL) robot = new URobotSmr(); 
      else if (not robot->isA("smr"))
      { // make it a hako tractor
        delete robot;
        robot = new URobotSmr();
      }
      break;
  }
  robot->paint(&seenFromPose, viewer);
  //
  // paint pose text
  paintOdoDataText();
  if (paintCam)
    paintCams(seenFromPose);
  if (paintGmk)
    paintGmks(seenFromPose);

  if (followRobot)
  { // top of robot
    UPose deltaPose;
    // get camera position in current coordinate system
    UPosition campos(cam.pos[0], cam.pos[1], cam.pos[2]);
    UPosition camFocus(cam.focal[0], cam.focal[1], cam.focal[2]);
    // get movement since last paint
    if (cooSysLast < 0)
      deltaPose.clear();
    else if (cooSysLast == paintPoseRef)
    { // no change in coordinate system
      // get last camera position (not height) relative to robot
      campos = robotPoseLast.getMapToPose(campos);
      // get camera position in system coordinates (map) - from new robot pose
      campos = seenFromPose.getPoseToMap(campos);
      // get change in movement for movement of camera focus point
      deltaPose = seenFromPose - robotPoseLast;
      // move focus point
      camFocus += deltaPose.getPos();
    }
    else
    { // changed coordinate ref - reset view
      UPose defPose(-4, 1, -0.4); // pose relative to robot - behind a little to the left
      deltaPose = seenFromPose + defPose;
      campos.set(deltaPose.x, deltaPose.y, 3.0);
      camFocus = seenFromPose.getPos();
    }
#if PCL_MINOR_VERSION < 7
      viewer->setCameraPose(campos.x, campos.y, campos.z,   // camera position
                          camFocus.x, camFocus.y, cam.focal[2],  // view focus point)
                          0.0, 0.0, 1.0);   // up vector (view vector)
#else
      viewer->setCameraPosition(campos.x, campos.y, campos.z,   // camera position
                          camFocus.x, camFocus.y, cam.focal[2],  // view focus point)
                          0.0, 0.0, 1.0);   // up vector (view vector)
    //viewer->setCameraPosition(c.pos[0], c.pos[1], c.pos[2], seenFromPose.x, seenFromPose.y, 0.0);
#endif
    //lastCamFocus = seenFromPose;
  }
  cooSysLast = paintPoseRef;
  robotPoseLast = seenFromPose;

}

///////////////////////////////////////////////////////////////

void UNavView::moveViewPose(double panAngle, bool reset, UPosition * camResetPos, UPosition * camResetFocus)
{
  UPose deltaPose;
  std::vector<pcl::visualization::Camera> cams;
  std::vector<pcl::visualization::Camera>::iterator camsi;
  pcl::visualization::Camera cam;
  viewer->getCameras(cams);
  camsi = cams.begin();
  cam = camsi[0];
  // get camera position
  UPosition campos(cam.pos[0], cam.pos[1], cam.pos[2]);
  // and where to look
  UPosition camFocus(cam.focal[0], cam.focal[1], cam.focal[2]);
  // get last camera position (not height) relative to robot
  // debug
//   if (campos.z < 1.4)
//     printf("UNavView::moveViewPose - camera position is too low!\n");
//   if (camFocus.z < 0.2)
//     printf("UNavView::moveViewPose - camera focus is too low!\n");
  // debug end
  if (not reset)
  {
    campos = robotPoseLast.getMapToPose(campos);
    double dist = hypot(campos.x, campos.y);
    campos.x = -cos(panAngle) * dist;
    campos.y = sin(panAngle) * dist;
    campos.z = cam.pos[2];
    // get camera position in system coordinates (map) - from new robot pose
    campos = robotPoseLast.getPoseToMap(campos);
  }
  else if (camResetPos != NULL)
  {
    campos = *camResetPos;
    // look at robot position
    camFocus = robotPoseLast.getPoseToMap(*camResetFocus);
    // get camera position in system coordinates (map) - from new robot pose
    campos = robotPoseLast.getPoseToMap(campos);
  }
  // set viewer camera
#if PCL_MINOR_VERSION < 7
    viewer->setCameraPose(campos.x, campos.y, campos.z,   // camera position
                          camFocus.x, camFocus.y, cam.focal[2],  // view focus point)
                          0.0, 0.0, 1.0);   // up vector (view vector)
#else
    viewer->setCameraPosition(campos.x, campos.y, campos.z,   // camera position
                          camFocus.x, camFocus.y, cam.focal[2],  // view focus point)
                          0.0, 0.0, 1.0);   // up vector (view vector)
    //viewer->setCameraPosition(c.pos[0], c.pos[1], c.pos[2], seenFromPose.x, seenFromPose.y, 0.0);
#endif
//   viewer->setCameraPose(campos.x, campos.y, campos.z,   // camera position
//                         camFocus.x, camFocus.y, camFocus.z,  // view focus point)
//                         0.0, 0.0, 1.0);   // up vector (view vector)
}


///////////////////////////////////////////////////////////////////

void UNavView::paintOdoGrid(UPose seenFromPose, double stepSize, double lastStepSize, bool bold)
{
  UPosition p1, p2;
  int i;
  int mr;
  int steps;
  pcl::PointXYZ pcp1, pcp2;
  UPosition posG1, posG2, posR1, posR2;
  const int MIDL = 12;
  char id[MIDL];
  //
  if (paintGridSize[1] > 0.0)
  { // remove old grid
    mr = int(10.0 * 2.0);
    steps = roundi(mr / lastStepSize);
    for (i = -steps; i <= steps; i++)
    { // paint odo-grid (Y)
      snprintf(id, MIDL, "gridy%d%03d", bold, i);
      viewer->removeShape(id);
      snprintf(id, MIDL, "gridx%d%03d", bold, i);
      viewer->removeShape(id);
    }
  }
  pcp1.z = 0.0;
  pcp2.z = 0.0;
  mr = int(10.0 * 2.0);
  steps = roundi(mr / stepSize);
  // paint odometry grid
  posG1.set(floor(seenFromPose.x / stepSize) * stepSize - double(steps) * stepSize,
            floor(seenFromPose.y / stepSize) * stepSize - double(steps) * stepSize, 0.0);
  posG2 = posG1;
  posG2.x = posG1.x + double(steps * 2) * stepSize;
  for (i = -steps; i <= steps; i++)
  { // paint odo-grid (Y)
    pcp1.x = posG1.x;
    pcp1.y = posG1.y;
    pcp2.x = posG2.x;
    pcp2.y = posG2.y;
    snprintf(id, MIDL, "gridy%d%03d", bold, i);
    viewer->addLine<pcl::PointXYZ>(pcp1, pcp2, id);
    double a = floor(posG1.y / (stepSize * 10)) * stepSize * 10.0;
    double b = floor(posG1.y / stepSize) * stepSize;
    if (fabs(a - b) < (stepSize * 0.1))
      // dividable by 10 - so stronger color
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.7, 0.2, id);
    else
      // dividable by minor grid line
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.85, 0.85, 0.7, id);
    posG1.y += stepSize;
    posG2.y += stepSize;
  }
  //
  posG1.set(floor(seenFromPose.x / stepSize) * stepSize - double(steps) * stepSize,
            floor(seenFromPose.y / stepSize) * stepSize - double(steps) * stepSize, 0.0);
  posG2 = posG1;
  posG2.y = posG1.y + double(steps * 2) * stepSize;
  for (i = -steps; i <= steps; i++)
  { // paint odo-grid (X)
    pcp1.x = posG1.x;
    pcp1.y = posG1.y;
    pcp2.x = posG2.x;
    pcp2.y = posG2.y;
    snprintf(id, MIDL, "gridx%d%03d", bold, i);
    viewer->addLine<pcl::PointXYZ>(pcp1, pcp2, id);
    double a = floor(posG1.x / (stepSize * 10)) * stepSize * 10.0;
    double b = floor(posG1.x / stepSize) * stepSize;
    if (fabs(a - b) < (stepSize * 0.1))
      // dividable by 10 - so stronger color
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.1, 0.7, id);
    else
      // dividable by minor grid line
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.7, 0.99, id);
    posG1.x += stepSize;
    posG2.x += stepSize;
  }
}

/////////////////////////////////////////////////

bool UNavView::paintRangeRings(UPosRot * sensorPos, int ringCnt)
{
//   CvPoint p1;
//   CvFont font;
//   CvScalar lblue = CV_RGB(200, 200 , 250);
//   CvScalar lyellow = CV_RGB(240, 240 , 180);
//   CvScalar lblack = CV_RGB(100, 100, 100);
//   const int MTL = 30;
//   char text[MTL];
//   int i;
//   bool result;
//   //UPosRot devPos;
//   //const int RING_CNT = 8; // number of range rings (each 1 meter)
//   //
//   if (paintBold)
//   {
//     lblue = CV_RGB(50, 50 , 250);
//     lyellow = CV_RGB(200, 200 , 0);
//     lblack = CV_RGB(10, 10, 10);
//     cvInitFont( &font, CV_FONT_HERSHEY_DUPLEX,
//                  1.0, 1.0, 0.0, 1, 8);
//   }
//   else
//     cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
//                  1.0, 1.0, 0.0, 1, 8);
//
//   result = (img != NULL) and (laserData != NULL);
//   if (result)
//   { // paint laser range rings
//     // get device position
//     //devPos = laserData->getDevPos();
//     // convert to pixels
//     p1.y = pr.y - roundi(sensorPos->getX() * ppm);
//     p1.x = pr.x - roundi(sensorPos->getY() * ppm);
//     // paint laser position
//     viewer->addLine
//     cvCircle(img->cvArr(), p1, 3, lblue, 1, 4, 0);
//     // paint range rings
//     for (i = 1; i <= rangeRingCnt; i++)
//     {
//        //cvEllipse( CvArr* img, CvPoint center, CvSize axes, double angle,
//        //         double start_angle, double end_angle, CvScalar color,
//        //         int thickness=1, int line_type=8, int shift=0 );
//       //printf("UNavView::rangerings: size(%d, %d), ppm=%.2f, i=%d\n",  roundi(ppm * i), roundi(ppm * i), ppm, i);
//
//       cvEllipse(img->cvArr(), p1,
//                 cvSize(roundi(ppm * i), roundi(ppm * i)),
//                 0.0, 0.0, 180.0, lblue, 1, 4, 0);
//       //cvCircle(img->cvArr(), p1, roundi(ppm * i), lblue, 1, 4, 0);
//       if (maxRange < 11)
//       {
//         snprintf(text, MTL, "%dm", i);
//         cvPutText(img->cvArr(), text, cvPoint(p1.x + roundi(ppm * i)-15, p1.y+15), &font, lblack);
//       }
//       else if (maxRange < 25)
//       {
//         snprintf(text, MTL, "%d", i);
//         cvPutText(img->cvArr(), text, cvPoint(p1.x + roundi(ppm * i)-5, p1.y+15), &font, lblack);
//       }
//     }
//   }
//   return result;
  return true;
}

///////////////////////////////////////////////////////////////

bool UNavView::paintScanStatData(ULaserDataSet * scan,
                                            bool paintVar, //bool paintEdge, bool paintCurv,
                                            bool paintVarL, bool paintTilt)
{
//   CvPoint p1, p2;
//   CvPoint p1VarL, p1VarI, p1VarLC, p1Tilt, p1Curv, p1X;
//   CvPoint topLeft;
//   CvPoint botRight;
//   CvFont font;
//   CvScalar lBlue = CV_RGB(150, 150 , 255);
//   //CvScalar dBlue = CV_RGB(20, 20 , 155);
//   //CvScalar blue = CV_RGB(0, 0 , 200);
//   //CvScalar dMagenta = CV_RGB(245, 50 , 245);
//   //CvScalar dCyan = CV_RGB(90, 195 , 195);
//   CvScalar black = CV_RGB(0, 0, 0);
//   CvScalar red = CV_RGB(255, 0, 0);
//   //CvScalar redMag = CV_RGB(180, 0, 100);
//   //CvScalar orange = CV_RGB(180, 100, 0);
//   CvScalar green = CV_RGB(0, 155, 0);
//   CvScalar yellow = CV_RGB(175, 175, 0);
//   //CvScalar brown = CV_RGB(100, 100, 0);
//   //CvScalar magenta = CV_RGB(155, 0, 155);
//   bool result;
//   int i, n, h, w;
//   UClientLaserData * lp;
//   //const double maxVar = 0.02;  // top limit
//   const double maxVarLR = 0.2;  // top limit of SD of variance to the left
// //  const double maxEdge = 0.4;
// //  const double maxCurv = M_PI/2.0;
//   const double maxTilt = M_PI/2.0;
//   const double maxX = 4.0;
//   UPosition pos;
//   UPosition posZero(0.0, 0.0, 0.0);
//   double cosLaserTilt;
//   int txtLineH;
//   CvPoint piL, piR;
//   //
//   if (paintBold)
//   {
//     cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
//                  1.0, 1.4, 0.0, 1, 8);
//     txtLineH = 15;
//     topLeft.y = 26 + 5;
//   }
//   else
//   {
//     cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
//                  1.0, 1.0, 0.0, 1, 8);
//     txtLineH = 12;
//     topLeft.y = 13 + 5;
//   }
//   result = (img != NULL);
//   if (result)
//   { // get position of varianve curve
//     n = (img->width() - 10)/(2 * scan->getCount());
//     w = scan->getCount() * n;
//     h = img->height() / 5;
//     botRight.x = img->width() - 5;
//     topLeft.x = botRight.x - w;
//     botRight.y = topLeft.y + h;
//     p1VarL = botRight;
//     p1VarI = botRight;
//     p1VarLC = botRight;
//     p1Tilt = botRight;
//     p1Tilt.y -= h/2;
//     p1Curv = p1Tilt;
//     p1X = botRight;
//     // Paint coordinate system
//     cvRectangle(img->cvArr(), topLeft, botRight, lBlue);
//     p1.y = topLeft.y;
//     p2.y = botRight.y;
//     for (i = 1; i < 4; i++)
//     {
//       p1.x = topLeft.x + 45 * i * n;
//       p2.x = p1.x;
//       cvLine(img->cvArr(), p1, p2, lBlue);
//     }
//     // paint zero line
//     p1.x = topLeft.x;
//     p2.x = botRight.x;
//     p1.y = (topLeft.y + botRight.y)/2;
//     p2.y = p1.y;
//     cvLine(img->cvArr(), p1, p2, lBlue);
//     p1.y = (topLeft.y + botRight.y)/2 + h/4;
//     p2.y = p1.y;
//     cvLine(img->cvArr(), p1, p2, lBlue);
//     p2.x -= 90;
//     p1.y = (topLeft.y + botRight.y)/2 - h/4;
//     p2.y = p1.y;
//     cvLine(img->cvArr(), p1, p2, lBlue);
//     // paint legend
//     p1.x = botRight.x - 75;
//     p1.y = 16 + 13;
//     if (paintVar)
//     {
//       cvPutText(img->cvArr(), "-var bad", p1, &font, yellow);
//       p1.y += txtLineH;
//       cvPutText(img->cvArr(), "-var OK ", p1, &font, green);
//       p1.y += txtLineH;
//     }
// /*    if (paintEdge)
//     {
//     cvPutText(img->cvArr(), "-edge", p1, &font, dMagenta);
//     p1.y += 12;
//   }
//     if (paintCurv)
//     {
//     cvPutText(img->cvArr(), "-curv", p1, &font, dMagenta);
//     p1.y += 12;
//   }*/
//     if (paintVarL)
//     {
//       p1.y = topLeft.y + txtLineH;
//       cvPutText(img->cvArr(), "-sdLeft", p1, &font, red);
// /*      cvPutText(img->cvArr(), "-sdLC", p1, &font, brown);
//       p1.y += 12;*/
//       p2.x = topLeft.x - 37;
//       if (paintBold)
//         p2.x -= 5;
//       p2.y = topLeft.y + 7;
//       cvPutText(img->cvArr(), " cm", p2, &font, red);
//       p2.y += h/4;
//       cvPutText(img->cvArr(), "  15", p2, &font, red);
//       p2.y += h/4;
//       cvPutText(img->cvArr(), "  10", p2, &font, red);
//       p2.y += h/4;
//       cvPutText(img->cvArr(), "   5", p2, &font, red);
//       p2.y += h/4;
//       cvPutText(img->cvArr(), "   0", p2, &font, red);
//     }
//     if (paintTilt)
//     {
//       p1.y += txtLineH;
//       cvPutText(img->cvArr(), "-tilt", p1, &font, red);
//     }
//     if (true)
//     {
//       p1.y += txtLineH;
//       cvPutText(img->cvArr(), "-X", p1, &font, black);
//     }
//     // paint curve
//     lp = scan->getData();
//     p2.y = (topLeft.y + botRight.y)/2;
//     cosLaserTilt = cos(scan->getLaserTilt());
//     piL.x = -1;
//     piL.y = botRight.y + 1;
//     piR.x = -1;
//     piR.y = piL.y - h - 1;
//     for (i = 0; i < scan->getCount(); i++)
//     { // get position of this angle
//       p1.x = botRight.x -
//           roundi((90.0 + lp->getAngle() * 180.0 / M_PI) * double(n));
//       if (true)
//       { // paint scan x-value
//         pos = lp->getPosition(posZero, cosLaserTilt);
//         p1.y = botRight.y - mini(h, roundi(double(h) *
//             pos.x / maxX));
//         cvLine(img->cvArr(), p1X, p1, black);
//         p1X = p1;
//       }
//
//       if (paintVarL)
//       { // "integrated" - running average - varianve to the left (from right).
// /*        p1.y = botRight.y - mini(h, roundi(double(h) *
//         lp->getVarI() / maxVarLR));
//         cvLine(img->cvArr(), p1VarI, p1, dBlue);
//         p1VarI = p1;*/
//         // normal varianve to the left over a constant width
// /*        p1.y = botRight.y - mini(h, roundi(double(h) *
//         lp->getSdLC() / maxVarLR));
//         cvLine(img->cvArr(), p1VarLC, p1, brown);
//         p1VarLC = p1;*/
//         // normal varianve to the left over a constant width
//         p1.y = botRight.y - mini(h, roundi(double(h) *
//             lp->getSdL() / maxVarLR));
//         cvLine(img->cvArr(), p1VarL, p1, red);
//         p1VarL = p1;
//         if (lp->getFlag() >= 10)
//         {
//           if (lp->getFlag() >= 30)
//           { // start and stop at same position - just paint line
//             piL.x = p1.x;
//             piR.x = p1.x;
//             cvLine(img->cvArr(), piL, piR, green);
//             piL.x = -1;
//             piR.x = -1;
//           }
//           else if (piR.x < 0)
//             piR.x = p1.x;
//           else if (piL.x < 0)
//           { // both ends available - paint square
//             piL.x = p1.x;
//             cvRectangle(img->cvArr(), piL, piR, green);
//             piL.x = -1;
//             piR.x = -1;
//           }
//         }
//       }
//       if (paintTilt)
//       {
//         p1.y = p2.y - maxi(-h/2, mini(h/2, roundi(double(h/2) * lp->getTilt() / maxTilt)));
//         if (not lp->isValid())
//           cvLine(img->cvArr(), p1Tilt, p1, red);
//         else
//           cvLine(img->cvArr(), p1Tilt, p1, green);
//         p1Tilt = p1;
//       }
//       lp++;
//     }
//     //
//     // print odometer position
//     //snprintf(text, MTL, "odoPos %6.2fx, %6.2fy, %6.1fdeg",
//     //           odoPose.x, odoPose.y, odoPose.h * 180.0 / M_PI);
//     //cvPutText(img->cvArr(), text, p1, &font, black);
//     // print odometer time
//   }
//   return result;
  return true;
}

////////////////////////////////////////////////////

void UNavView::paintPointClouds(UResPoseHist * posesys)
{
  UPcpItem * pci;
  UPoseTVQ posePcp;
  if (resPcp == NULL)
    return;
  // freeze current status - if possible, else wait to next update
  if (resPcp->tryLock())
  {
    if (resPcp->getPolysCnt() > paintPclsMaxCnt)
    {
      if (paintPcls == NULL) 
      {
        paintPcls = (UPaintPcp **) malloc(resPcp->getPolysCnt() * sizeof(UPaintPcp*));
        paintPclsMaxCnt = 0;
      }
      else
        paintPcls = (UPaintPcp **) realloc(&paintPcls, resPcp->getPolysCnt() * sizeof(UPaintPcp*));
      // initialize new elements
      for (int i = paintPclsMaxCnt; i < resPcp->getPolysCnt(); i++)
        paintPcls[i] = new UPaintPcp;
      // set new limit
      paintPclsMaxCnt = resPcp->getPolysCnt();
    }
    for (int i=0; i < resPcp->getPolysCnt(); i++)
    {
      pci = resPcp->getItem(i);
      if (pci != NULL)
      { // show this point-cloud
        if ((paintPcls[i]->getCooSysView() != paintPoseRef) or
            (pci->updateTime > paintPcls[i]->updateTime) or
            (paintBold != paintPcls[i]->bold))
        { // robot pose in currently displayed coordinate system
          // to be used if point cloud is in robot coordinates
          UPose rp = posesys->getPoseAtTime(pci->updateTime);
          UPose notUsed(0, 0, 0);
          // remove old cloud
          paintPcls[i]->unPaint(viewer);
          // remember new coordinate system
          paintPcls[i]->setViewCooSys(paintPoseRef, notUsed);
          paintPcls[i]->bold = paintBold;
          // update with new cloud
          paintPcls[i]->update(pci, posesys, &rp);
          // show new cloud
          paintPcls[i]->paint(viewer);
        }
      }
      else if (paintPcls[i]->pcl != NULL)
        paintPcls[i]->unPaint(viewer);
    }
    resPcp->unlock();
  }
}


/////////////////////////////////////////////////

void UNavView::paintScansNewest(UPaintBase * scanClouds, ULaserDataHistory * scans)
{
  UPaintLaserScans * vscans = (UPaintLaserScans *) scanClouds;
  UMatrix4 mLtoR;
  int pz;
  UPoseTVQ poseScan;
  int newestId = vscans->getNewestSerial();
  //
  // count scans since last paint
  int newCnt = mini(paintScanHistCnt, laserData->getScansCnt());
  if (newestId >= 0)
  { // somescans are there already, find count of new scans
    int nMax = newCnt;
    newCnt = 0;
    for (int n = 0; n < nMax; n++)
    { // look for last shown scan
      ULaserDataSet * scan = laserData->getScan(n);
      if ((int)scan->getSerial() == newestId)
        break;
      newCnt++;
    }
  }
  // do we need to renew all?
  if ((vscans->getCooSysView() != paintPoseRef) or // coordinate system change
      (paintScanHistCnt > vscans->cloudsCnt and    // or asked for more than viewed right now, and more is available
       (vscans->cloudsCnt + newCnt) < laserData->getScansCnt()) or
      (paintBold != vscans->bold))  
  { // all shown scans are need a redisplay
    vscans->unPaintAll(viewer);
    vscans->removeAll();
    vscans->setViewCooSys(paintPoseRef, poseScan);
    // paint all (available and needed) scans as new
    newCnt = mini(paintScanHistCnt, laserData->getScansCnt());
    vscans->bold = paintBold;
  }
  // do we need to removesome old scans from view
  if (newCnt + vscans->cloudsCnt > paintScanHistCnt)
  { // remove some scans
    vscans->remove(newCnt + vscans->cloudsCnt - paintScanHistCnt, viewer);
  }
  // show the new ones
  if (paintBold)
    pz = 8;
  else
    pz = 2;
  if (newCnt > 0)
  { // old newest scan should be reduced to smallpocs
    vscans->ageNewestScan(viewer, pz);
    for (int i = newCnt - 1; i >= 0; i--)
    {
      UResPoseHist * por;
      ULaserDataSet * scan;
      if (i == 0)
        pz *= 2;
      switch (paintPoseRef)
      {
        case 0: por = poseOdo; break;
        case 1: por = poseUtm; break;
        case 2: por = poseMap; break;
        default: por = NULL; break;
      }
      scan = laserData->getScan(i);
      if (por != NULL)
        poseScan = por->getPoseAtTime(scan->getScanTime());
      else
        // no way to get scanner pose at scantime, use pose provided by scan (odoPose)
        poseScan = scan->getPose();
      vscans->addScan(scan, &poseScan, viewer, pz);
    }
  }
}

///////////////////////////////////////




///////////////////////////////////////

// void UNavView::paintRoadLine(URoadLineData * road, UPoseTime seenFromPose, bool inGray)
// { // paint road segment and poly.line
//   printf("UNavView::paintRoadLine is not implemented\n");
//   return;
//   int i, n, m;
//   UPosition posm, posr, pos;
//   uint32_t red =    (255 << 16) + (0   << 8) + 0;
//   uint32_t green =  (0   << 16) + (255 << 8) + 0;
//   uint32_t yellow = (128 << 16) + (128 << 8) + 0;
//   uint32_t gray   = (100 << 16) + (100 << 8) + 100;
//   UPoseTime pose;
//   UPoseTime lastPose;
//   pcl::PointXYZRGB pc1, pc2;
//   pc1.z = 0.0;
//   //
//   if (road != NULL)
//   {
//     if (inGray)
//       pc1.rgb = *reinterpret_cast<float*>(&gray);
//     else
//       switch (road->edge)
//       { // 0=left, 1=top, 2=right
//         case 0: pc1.rgb = *reinterpret_cast<float*>(&red); break;
//         case 1: pc1.rgb = *reinterpret_cast<float*>(&yellow); break;
//         default: pc1.rgb = *reinterpret_cast<float*>(&green); break;
//       }
//     pc2 = pc1;
//     //
//     pos = seenFromPose.getMapToPose(road->line.pos);
//     pc1.x = pos.x;
//     pc1.y = pos.y;
//     pos = seenFromPose.getMapToPose(road->line.getOtherEnd());
//     pc2.x = pos.x;
//     pc2.y = pos.y;
//     viewer->addLine<pcl::PointXYZRGB>(pc1, pc2, "road");
//     // continue with polyline
//     m = road->edgeLine.getPointsCnt();
//     n = mini(paintRoadHistCnt, m);
//     // get pixel position of start pose
//     for (i = 0; i < n; i++)
//     { // get pose with this age - older and older
//       m--;
//       pc2 = pc1; // save last pixel position
//       // robot pose for this update
//       pos = seenFromPose.getMapToPose(road->edgeLine.getPoint(m));
//       pc1.x = pos.x;
//       pc1.y = pos.y;
//       viewer->addLine<pcl::PointXYZRGB>(pc1, pc2, "road");
//     }
//     if (paintBold)
//       viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "road");
//     else
//       viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "road");
//   }
// }

//////////////////////////////////////////////////////

void UNavView::paintObstGrp(UObstacleGroup *obst,
                             UPose seenFromPose, int grpIdx)
{
  printf("UNavView::paintObstGrp is not implemented\n");
  return;
  int i, n;
  UPosition pos;
  UPosition po1, po2;
  uint32_t purpOdd   = (100 << 16) + (50 << 8) + 100;
  uint32_t yellowOdd = (100 << 16) + (100 << 8) + 50;
  uint32_t purp      = (190 << 16) + (0 << 8) + 190;
  uint32_t yellow    = (190 <<16) + (190 << 8) + 0;
  uint32_t gray      = (100 << 16) + (100 << 8) + 100;
  uint32_t obsValid;
  uint32_t obsInvalid;
  UObstacle * ob;
  UPose sysPose;
  bool convert;
  pcl::PointXYZRGB pc1, pc2;
  //
  // obstacles are always in odometry coordinates.
  sysPose = getSystemOrigin(0, &convert);
  //
  if (grpIdx == -1)
  { // fixed obstacles
    obsValid = gray;
    obsInvalid = gray;
  }
  else if (grpIdx % 2 == 0)
  {
    obsValid = purp;
    obsInvalid = yellow;
  }
  else
  {
    obsValid = purpOdd;
    obsInvalid = yellowOdd;
  }
  //
  for (n = 0; n < obst->getObstsCnt(); n++)
  {
    ob = obst->getObstacle(n);
    // paint obstacle
    po1 = ob->getPoint(ob->getPointsCnt() - 1);
    if (convert)
      po1 = sysPose.getPoseToMap(po1);
    // select colour
    if (ob->isValid())
      pc1.rgb = *reinterpret_cast<float*>(&obsValid);
    else
      pc1.rgb = *reinterpret_cast<float*>(&obsInvalid);
    //pose = scan->getPose();
    for (i = 0; i < ob->getPointsCnt(); i++)
    {
      po2 = po1;
      pc2 = pc1;
      po1 = ob->getPoint(i);
      if (convert)
        po1 = sysPose.getPoseToMap(po1);
      pos = seenFromPose.getMapToPose(po1);
      pc1.x = pos.x;
      pc1.y = pos.y;
      pc1.z = pos.z;
      if (ob->getPointsCnt() > 1)
      {
        pos = seenFromPose.getMapToPose(po2);
        pc2.x = pos.x;
        pc2.y = pos.y;
        pc2.z = pos.z;
      }
      else
      { // one point only, paint short vertical line
        pc2 = pc1;
        pc2.z += 0.1;
      }
      viewer->addLine<pcl::PointXYZRGB>(pc1, pc2, "obstGrp");
    }
  }
  if (paintBold)
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "obstGrp");
  else
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "obstGrp");

}


///////////////////////////////////////////////////////

void UNavView::paintFreePoly(UProbPoly * poly,
                              UPose seenFromPose,
                              bool historic)
{
  printf("UNavView::paintFreePoly is not implemented\n");
  return;
  int i;
  UPosition pos;
  UPosition *po1, *po2;
  bool * poObst;
  uint32_t red = (155 << 16) + (0 << 8) + 0;
  uint32_t pale = (10 << 16) + (200 << 8) + 200;
  uint32_t hist = (150 << 16) + (100 << 8) + 100;
  pcl::PointXYZRGB pc1, pc2;
  //
  po1 = poly->getPoints();
  po2 = po1 + 1;
  poObst = poly->getIsObst();
  for (i = 1; i < poly->getPointsCnt(); i++)
  {
    pos = seenFromPose.getMapToPose(*po1);
    pc1.x = pos.x;
    pc1.y = pos.y;
    pc1.z = pos.z;
    pos = seenFromPose.getMapToPose(*po2);
    pc2.x = pos.x;
    pc2.y = pos.y;
    pc2.z = pos.z;
    if (*poObst and not historic)
      pc1.rgb = *reinterpret_cast<float*>(&red);
    else if (*poObst and historic)
      pc1.rgb = *reinterpret_cast<float*>(&hist);
    else if (not historic)
      pc1.rgb = *reinterpret_cast<float*>(&pale);
    pc2.rgb = pc1.rgb;
    if (*poObst or not historic)
      viewer->addLine<pcl::PointXYZRGB>(pc1, pc2, "freepoly");
    // advance to next
    poObst++;
    po1++;
    po2++;
  }
  if (paintBold)
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "freepoly");
  else
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "freepoly");
}

///////////////////////////////////////////////

void UNavView::paintOdoDataText()
{
  CvPoint p1;
  const int MTL = 130;
  char text[MTL];
  char textPre[MTL];
  const int MSL = 30;
  char t1[MSL];
  char t2[MSL];
  UPosition pos;
  int i;
  UTime t;
  UPoseTVQ pose;
  UResPoseHist * ph;
  const char * sys = NULL;
  const char * syst = NULL; // time from system
  bool disp = false;
  int textSize = 14;
  double red;
  //
  if (poseOdo != NULL)
    t = poseOdo->getNewest().t;
  else if (poseUtm != NULL)
    t = poseUtm->getNewest().t;
  else if (poseMap != NULL)
    t = poseMap->getNewest().t;
  else
    t.clear();

  if (paintBold)
    textSize=20;
  // paint odo position bottom-left as text
  p1.x = 2;
  p1.y = 4;
  //strncpy(textPre, "init", MTL);
  //
  for (i = 2; i >= 0; i--)
  { // paint position of known coordinate systems
    switch (i)
    {
      case 0: ph = poseOdo; sys = "odo "; disp = paintOdoPose; break;
      case 1: ph = poseUtm; sys = "utm"; disp = paintUtmPose; break;
      case 2: ph = poseMap; sys = "map "; disp = paintMapPose; break;
      default: ph = NULL; sys = "NULL"; break;
    }
    if (ph != NULL)
    { // system is available and should be displayed
      pose = ph->getPoseAtTime(t);
      // save time (newest for all systems)
      if (pose.t > t)
      {
        t = pose.t;
        syst = sys;
      }
      if (disp)
      { // display position
        p1.y += 15;
        if (paintPoseRef == i)
          red = 1.0;
        else
          red = 0.0;
        if (paintBold)
        {
          p1.y += 5;
          snprintf(textPre, MTL - 1, " %s %.2fx %.2fy %.0f",
                sys, pose.x, pose.y, pose.h * 180.0 / M_PI);
          snprintf(text, MTL, "%s  %.1fm/s %gq.",
                  textPre, pose.vel, pose.q);
        }
        else
        {
          snprintf(textPre,MTL - 1, " %s %10.2fx %11.2fy %6.1f",
                  sys, pose.x, pose.y, pose.h * 180.0 / M_PI);
          snprintf(text, MTL, "%s  (mat) %.2fm/s %gq.",
                  textPre, pose.vel, pose.q);
        }
        viewer->removeShape(sys);
        viewer->addText(text, p1.x, p1.y, textSize, red, 0, 0, sys);
//         p2.y = p1.y + textSize * 2;
//         p2.x = p1.x + text_size.width - 2;
//         cvPutText(img->cvArr(), "o", p2, &font, black);
        // advance to next pose system
      }
    }
  }
  // time is at bottom line
  p1.x = 5;
  p1.y = 4;
  // paint time
  if (t.getDecSec() < 1.0)
    // no timestamp available - use current time
    t.Now();
  t.getDateString(t1, true);
  t.getTimeAsString(t2, true);
  if (paintBold)
    snprintf(text, MTL, "%s %s (%s)",
            t1, t2, syst);
  else
    snprintf(text, MTL, "%s %s (%lu.%06lu)",
            t1, t2, t.getSec(), t.getMicrosec());
  viewer->removeShape("updatetime");
  viewer->addText(text, p1.x, p1.y, textSize, 0, 0, 0, "updatetime");
}

////////////////////////////////////////////////////

bool UNavView::paintVarDataText(bool paintVar)
{
//   CvPoint p1;
//   CvFont font;
//   //CvScalar bblue = CV_RGB(0, 0 , 100);
//   CvScalar brown = CV_RGB(100, 100 , 0);
//   CvScalar red = CV_RGB(255, 0 , 0);
//   //CvScalar black = CV_RGB(0, 0 , 0);
//   bool result = true;
//   const int MSL = 240;
//   char s[MSL];
//   int i, n, m;
//   int col1, col2, col3, col4w;
//   UTime t;
//   //UVarPool * vp;
//   UVarPool * vpl;
//   UVariable ** var;
//   const int MTL = 5;
//   char vt[MTL];
//   //
//   // <planget planner=true file="mission.txt" usingFile=true
//   //  cmd="gotowaypoint odo 10.0 0.0  (get GPS)" isIdle=false/>
//   // paint variables starting top-left
//   p1.x = 10;
//   p1.y = 2;
//   //
//   p1.y += 18;
//   cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
//                 0.9, 1.0, 0.0, 1, 8);
//   // either display old stop-crits, or some new variables
//   col1 = 10;
//   col2 = col1 + 50;
//   col3 = col2 + 85;
//   col4w = 75;
//   //
//   //vp = varRoot->getVarPool();
//   for (n = 0; n < paintStructsCnt; n++)
//   {
//     vpl = paintStructs[n];
//     if (vpl == NULL)
//       // this structure should not be painted
//       continue;
//     p1.x = col1;
//     cvPutText(img->cvArr(), vpl->getFullPreName(s, MSL), p1, &font, red);
//     p1.y += 14;
//     var = vpl->getVars();
//     for (i = 0; i < vpl->getVarsCnt(); i++)
//     {
//       p1.x = col1 + 5;
//       snprintf(s, MSL, "%s (%s)", (*var)->name, (*var)->getTypeChar(vt , MTL));
//       cvPutText(img->cvArr(), s, p1, &font, brown);
//       p1.x = col3;
//       if ((*var)->isTypeA(UVariable::t))
//       { // double value
//         snprintf(s, MSL, "%8.3f", (*var)->getValued());
//         cvPutText(img->cvArr(), s, p1, &font, brown);
//         t.setTime((*var)->getValued());
//         t.getTimeAsString(s, true);
//         p1.x += 2 * col4w;
//         cvPutText(img->cvArr(), s, p1, &font, brown);
//       }
//       else if ((*var)->isString())
//       { // string
//         cvPutText(img->cvArr(), (*var)->getValues(0), p1, &font, brown);
//       }
//       else
//       { // any variable type will do
//         for (m = 0; m < (*var)->getElementCnt(); m++)
//         {
//           snprintf(s, MSL, "%8.3f", (*var)->getValued(m));
//           cvPutText(img->cvArr(), s, p1, &font, brown);
//           p1.x += col4w;
//         }
//       }
//       p1.y += 14;
//       var++;
//     }
//   }
//   return result;
//  printf("UNavView::paintVarText is not implemented\n");
  return true;
}

///////////////////////////////////////////////////////

void UNavView::paintPis(ULaserDataSet * scan,
                        UPose seenFromPose,
                        const int cnt)
{
  printf("UNavView::paintPis is not implemented\n");
  return;
  int i;
  UPosition pos;
  UClientLaserPi * pp;
  uint32_t red = (155 << 16) + (0 << 8) + 0;
  pcl::PointXYZRGB pc1, pc2;
  //
  pc1.rgb = *reinterpret_cast<float*>(&red);
  pc2.rgb = pc1.rgb;
  pp = scan->getPis();
  for (i = 0; i < scan->getPisCnt(); i++)
  {
    pos = seenFromPose.getMapToPose(pp->getLeftPos());
    pc1.x = pos.x;
    pc1.y = pos.y;
    pc1.z = pos.z;
    pos = seenFromPose.getMapToPose(pp->getRightPos());
    pc1.x = pos.x;
    pc1.y = pos.y;
    pc1.z = pos.z;
    viewer->addLine<pcl::PointXYZRGB>(pc1, pc2, "passableInterval");
    // mark left side of road
//     if (maxRange < 20)
//     {
//       pos = seenFromPose.getMapToPose(pp->getLeftSide());
//       p1.y = pr.y - roundi(pos.x * ppm);
//       p1.x = pr.x - roundi(pos.y * ppm);
//       cvCircle(img->cvArr(), p1, 4, mag, lw, 8, 0);
//       // mark right side of road
//       pos = seenFromPose.getMapToPose(pp->getRightSide());
//       p1.y = pr.y - roundi(pos.x * ppm);
//       p1.x = pr.x - roundi(pos.y * ppm);
//       cvCircle(img->cvArr(), p1, 4, mag, lw, 8, 0);
//       // mark top of road
//       pos = seenFromPose.getMapToPose(pp->getTopPos());
//       p1.y = pr.y - roundi(pos.x * ppm);
//       p1.x = pr.x - roundi(pos.y * ppm);
//       cvCircle(img->cvArr(), p1, 6, mag, lw, 8, 0);
//     }
    //
    pp++;
  }
  if (paintBold)
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "passableInterval");
  else
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "passableInterval");
}

////////////////////////////////////////////////////

void UNavView::paintFeatures(UFeaturePool * featurePool, UPoseTime seenFromPose)
{
  //printf("UNavView::paintFeatures is not implemented\n");
  return;
  int i, n, rl;
  UPosition pos;
  UPosition po1, po2;
  uint32_t purp     = (190 << 16) + (0 << 8) + 190;
  uint32_t yellow   = (145 << 16) + (145 << 8) + 0;
  uint32_t green    = (0 << 16) + (150 << 8) + 0;
  uint32_t red      = (150 << 16) + (0 <<8) + 0;
  uint32_t blue     = (0 << 16) + (0 << 8) + 190;
  uint32_t gray     = (120 << 16) + (120 << 8) + 120;
  UFeatureData * sf;
  ULineSegment * seg;
  pcl::PointXYZRGB pc1, pc2;
  uint32_t * col = &purp;
  int * segInt;
  bool isRoad;
  const int MAX_TYPES = 4;
  int cnt[MAX_TYPES];
  int typ;
  const char * segStr;
  bool convert = false;
  UPose systemOrigin; //
  //
  //
  //cnt = mini(sfPool->getScansCnt(), paintScanHistCnt);
  // debug
  //cnt = mini(1, cnt);
  for (i = 0; i < MAX_TYPES; i++)
    cnt[i] = 0;
  // debug end
  for (n = 0; n < featurePool->getScansCnt(); n++)
  {
    sf = featurePool->getScan(n);
    isRoad = false;
    convert = false;
    if (strcasecmp(sf->getDataType(), "sf") == 0)
    {
      //
      pc1.rgb = *reinterpret_cast<float*>(&red);
      col = &purp;
      typ = 1;
      systemOrigin = getSystemOrigin(0, &convert);
    }
    if (strcasecmp(sf->getDataType(), "mapl") == 0)
    { // short for maplines
      col = &blue;
      typ = 4;
      systemOrigin = getSystemOrigin(2, &convert);
    }
    else if (strcasecmp(sf->getDataType(), "pass") == 0)
    {
      col = &green;
      typ = 2;
      systemOrigin = getSystemOrigin(0, &convert);
    }
    else if (strcasecmp(sf->getDataType(), "road") == 0)
    {
      isRoad = true;
      typ = 3;
      //showLineOnly = (cnt[typ] > 5);
      systemOrigin = getSystemOrigin(0, &convert);
    }
    else
    {
      col = &yellow;
      typ = 0;
      systemOrigin = getSystemOrigin(0, &convert);
    }
    //
    if (cnt[typ]++ < paintScanHistCnt)
    { // make newest more fat than older entries
      seg = sf->getSegs();
      segInt = sf->getSegsInt();
      //segVal = sf->getSegsVal();
      for (i = 0; i < sf->getSegsCnt(); i++)
      {
        segStr = sf->segsStr[i];
        rl = -1;
        if (isRoad)
        {
          if (*segInt < 10)
            col = &gray;
          else
          {
            rl = *segInt % 10;
            // get last road line position
            if (rl == 0)
              col = &red;
            else if (rl == 2)
              col = &green;
            else
              col = &yellow;
          }
        }
        pc1.rgb = *reinterpret_cast<float*>(col);
        pc2.rgb = pc1.rgb;
        pos = seg->getOtherEnd();
        if (convert)
          // convert from other coordinate system
          pos = systemOrigin.getPoseToMap(pos);
        po1 = seenFromPose.getMapToPose(pos);
        pc1.x = po1.x;
        pc1.y = po1.y;
        pc1.z = po1.z;
        pos = seg->pos;
        if (convert)
          pos = systemOrigin.getPoseToMap(pos);
        po2 = seenFromPose.getMapToPose(pos);
        pc2.x = po1.x;
        pc2.y = po1.y;
        pc2.z = po1.z;
        viewer->addLine<pcl::PointXYZRGB>(pc1, pc2, "feature");
        
        if (segStr[0] != 0)
        { // show text near line
          viewer->addText3D<pcl::PointXYZRGB>(segStr, pc2);
          //cvPutText(img->cvArr(), segStr, p2, &font, *col);
        }
//           if (typ == 2)
//           { // paint center position of passable interval too
//             po2 = seenFromPose.getMapToPose(seg->getPositionOnLine(*segVal));
//             p2.y = pr.y - roundi(po2.x * ppm);
//             p2.x = pr.x - roundi(po2.y * ppm);
//             cvCircle(img->cvArr(), p2, 4, *col, lw/2, 8, 0);
//           }
        seg++;
        segInt++;
      }
    }
  }
  if (paintBold)
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "feature");
  else
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "feature");
}

/////////////////////////////////////////////////////////

bool UNavView::paintVarAdd(const char * name, bool add)
{
  UVarPool * vp, *vp2, **vp3 = NULL;
  int i;
  bool found = false;
  bool result = true;
  //
  if (varRoot != NULL)
  {
    vp = varRoot->getVarPool();
    vp2 = vp->getStructDeep(name, NULL, 0);
    if (vp2 != NULL)
    { // such structure exist - find it
      // see if it is in list already
      for (i = 0; i < paintStructsCnt; i++)
      {
        if (paintStructs[i] == NULL)
        { // an empty slot - save if new addition
          if (vp3 == NULL and add)
            vp3 = &paintStructs[i];
        }
        else if (paintStructs[i] == vp2)
        {
          if (not add)
            paintStructs[i] = NULL;
          found = true;
          break;
        }
      }
      if (not found and add)
      { // need to add
        if (vp3 == NULL and (paintStructsCnt < PAINT_MAX_STRUCTS))
            vp3 = &paintStructs[paintStructsCnt++];
        if (vp3 != NULL)
          *vp3 = vp2;
      }
    }
    else
      result = false;
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////

// bool UNavView::paintManData(UClientManSeq * man, int num, UPoseTime seenFromPose)
// {
//   CvPoint p1, p2;
//   CvFont font;
//   bool result;
//   UPosition pos;
//   UPose *ppos;
//   CvScalar red;
//   CvScalar green;
//   CvScalar rtCol;
//   //CvScalar yellow = CV_RGB(150, 150, 0);
//   CvScalar blue = CV_RGB(0, 0, 255);
//   CvScalar magenta = CV_RGB(150, 0, 150);
//   int i, j, redWidth;
//   int lw = 1;
//   UManPPSeq * mpp;
//   UManoeuvre * mm;
//   UManArc * mma;
//   UPoseV pv1, pv2;
//   UPosition pos1, pos2;
//   UPose pose1;
//   double a1, a2;
//   CvSize sz;
//   const int MSL = 20;
//   char s[MSL];
//   ULineSegment * seg;
//   char * segChars;
//   UPose systemOrigin;
//   bool convert;
// /*  U2Dpos pd1;
//   double d;*/
//   //
//   if (man->isBest())
//   {
//     green = CV_RGB(0, 200, 0);
//     red = CV_RGB(200, 0, 0);
//     rtCol = CV_RGB(250, 0, 0);
//     redWidth = 3;
//   }
//   else
//   { // unused path lines
//     green = CV_RGB(0, 200, 200);
//     red = CV_RGB(200, 100, 100);
//     rtCol = CV_RGB(180, 180, 0);
//     blue = CV_RGB(128, 128, 255);
//     redWidth = 2;
//   }
//   if (paintBold)
//     lw = 2;
//   //
//   cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
//                0.8, 1.0, 0.0, 1, 8);
//   result = (img != NULL) and (man != NULL);
//   if (result)
//   { // debug - changing colours
//     if (not man->isBest())
//       rtCol = CV_RGB(90, (num % 2) * 90, ((num / 2) % 3) * 90);
//     // debug end
//     // get possible conversion to other origin
//     systemOrigin = getSystemOrigin(0, &convert);
//     // get all parts
//     for (i = 0; i < man->getP2PCnt(); i++)
//     { // a path is divided into groups of 1 to 5 (line, arc, line, arc, line) simple manoeuvres
//       mpp = man->getP2P(i);
//       pv1 = mpp->getStartPoseV();
// /*      pv2 = mpp->getEndPoseV();
//       pos = seenFromPose.getMapToPose(pv1.getPos());
//       // and to pixel coordinates
//       p1.y = pr.y - roundi(pos.x * ppm);
//       p1.x = pr.x - roundi(pos.y * ppm);
//       pos = seenFromPose.getMapToPose(pv2.getPos());
//       // and to pixel coordinates
//       p2.y = pr.y - roundi(pos.x * ppm);
//       p2.x = pr.x - roundi(pos.y * ppm);*/
//       // paint direct (thin) line
//       //cvLine(img->cvArr(), p1, p2, rtCol, 1);
//       // now paint curved line too
//       for (j = 0; j < mpp->getSeqCnt(); j++)
//       { // paint each of the simple manoeuvres (line or arc)
//         mm = mpp->getMan(j);
//         pv2 = mm->getEndPoseV(pv1);
//         switch (mm->getManType())
//         { // either MAN_ARC, MAN_LINE or MAN_STOP
//         case UManoeuvre::MAN_ARC:
//           // convert to arc type manoeuvre
//           mma = (UManArc*)mm;
//           if (mma->getTurnRadius() < 0.0)
//           {
//             mma->setTurnAngle(-mma->getTurnAngle());
//             mma->setTurnRadius(-mma->getTurnRadius());
//           }
//           // if too big, then draw a line, too big is set to r> ~ 200 meter
//           if (mma->getTurnRadius() > (10.0 * maxRange))
//           { // just a line from start (pv1) to finish (pv2)
//             p1 = toPixels(pv1.getPos(), seenFromPose, systemOrigin, convert);
//             p2 = toPixels(pv2.getPos(), seenFromPose, systemOrigin, convert);
//             // paint direct line
//             cvLine(img->cvArr(), p1, p2, rtCol, redWidth * lw);
//           }
//           else
//           { // OK, draw as arc
//             // find centre
//             if (mma->getTurnAngle() > 0.0)
//             {
//               pos2.y = mma->getTurnRadius();
//               a1 = (pv1.h - seenFromPose.h) * (180.0 / M_PI) - 90.0;
//               a2 = a1 + (180.0 / M_PI) * mma->getTurnAngle();
//             }
//             else
//             {
//               pos2.y = -mma->getTurnRadius();
//               a2 = (pv1.h - seenFromPose.h) * (180.0 / M_PI) + 90.0;
//               a1 = a2 + (180.0 / M_PI) * mma->getTurnAngle();
//             }
//             pos2.x = 0.0;
//             // in real map coordinates
//             pos = pv1.getPoseToMap(pos2);
//             // to displayed coordinate  system
//             if (convert)
//             {
//               pos = systemOrigin.getPoseToMap(pos);
//               a1 += systemOrigin.h * 180.0 / M_PI;
//               a2 += systemOrigin.h * 180.0 / M_PI;
//             }
//             // and to robot coordinates
//             pos = seenFromPose.getMapToPose(pos);
//             // and to pixel coordinates
//             p1 = toPixels(pos);
//             // set arc radius
//             sz.width = roundi(mma->getTurnRadius() * ppm);
//             sz.height = sz.width;
//             cvEllipse( img->cvArr(), p1, sz, 90.0,
//                         a1, a2, rtCol, redWidth * lw);
//           }
//           break;
//         case UManoeuvre::MAN_LINE:
//           p1 = toPixels(pv1.getPos(), seenFromPose, systemOrigin, convert);
//           p2 = toPixels(pv2.getPos(), seenFromPose, systemOrigin, convert);
//           // paint direct line
//           cvLine(img->cvArr(), p1, p2, rtCol, redWidth * lw);
//           if (maxRange < 35.0)
//           { // paint also circle at end of line
//             cvCircle(img->cvArr(), p1, 4, red, lw, 4, 0);
//             cvCircle(img->cvArr(), p2, 4, red, lw, 4, 0);
//           }
//           break;
//         case UManoeuvre::MAN_STOP:
//           break;
//         }
//         pv1 = pv2;
//       }
//     }
//     if (paintPathMidPoses)
//     {
//       ppos = man->getPoses();
//       // make color stronger for text and crosses
//       rtCol.val[0] = rtCol.val[0] / 2;
//       rtCol.val[1] = rtCol.val[1] / 2;
//       rtCol.val[2] = rtCol.val[2] / 2;
//       for (i = 0; i < man->getPosesCnt(); i++)
//       { // convert to robot coordinates
//         pose1 = *ppos;
//         if (convert)
//           pose1 = systemOrigin.getPoseToMapPose(pose1);
//         pose1 = seenFromPose.getMapToPosePose(&pose1);
//                   // and to pixel coordinates
//         p1 = toPixels(pose1);
// /*        p1.y = pr.y - roundi(pose1.x * ppm);
//         p1.x = pr.x - roundi(pose1.y * ppm);*/
//         // paint cross at position (or a fat circle if crashed)
//         if (man->isPathFailed())
//           cvCircle(img->cvArr(), p1, 6, rtCol, 2, 8, 0);
//         else
//           paintCross(img, p1.x, p1.y, 4, rtCol, lw);
//         //
//         p2.y = pr.y - roundi(pose1.x * ppm + 40.0 * cos(pose1.h));
//         p2.x = pr.x - roundi(pose1.y * ppm + 40.0 * sin(pose1.h));
//         cvLine(img->cvArr(), p1, p2, rtCol, lw);
//         // and midPose serial number
//         snprintf(s, MSL, "%d(%d)", num, i);
//         cvPutText(img->cvArr(), s, p1, &font, blue);
//         //
//         ppos++;
//       }
//     }
//     if (paintPathSupportLines)
//     {
//       seg = man->getSegs();
//       segChars = man->getSegChars();
//       for (i = 0; i < man->getSegsCnt(); i++)
//       { // convert to robot coordinates
//         pos1 = seg->pos;
//         if (convert)
//           pos1 = systemOrigin.getPoseToMap(pos1);
//         pos1 = seenFromPose.getMapToPose(pos1);
//         pos2 = seg->getOtherEnd();
//         if (convert)
//           pos2 = systemOrigin.getPoseToMap(pos2);
//         pos2 = seenFromPose.getMapToPose(pos2);
//                   // and to pixel coordinates
//         p1.y = pr.y - roundi(pos1.x * ppm);
//         p1.x = pr.x - roundi(pos1.y * ppm);
//         p2.y = pr.y - roundi(pos2.x * ppm);
//         p2.x = pr.x - roundi(pos2.y * ppm);
//         // get colour
//         if (segChars[i] == 't')
//           rtCol = blue;
//         else if (segChars[i] == 's')
//         {
//           rtCol = blue;
//           if (paintBold)
//             lw = 4;
//           else
//             lw = 2;
//         }
//         else
//           rtCol = magenta;
//         // paint no-visibility line
//         cvLine(img->cvArr(), p1, p2, rtCol, lw);
//         //
//         seg++;
//       }
//     }
//   }
//   return result;
// }

////////////////////////////////////////////////

void UNavView::paintCams(UPoseTime seenFromPose)
{
//   int i;
//   CvPoint p1, p2;
//   UPosition posm, posr, pos;
//   CvScalar red = CV_RGB(255, 0, 0);
//   int lw = 1;
//   UClientCamData * cam;
//   UMatrix4 mCam;
//   UPosRot camPose;
//   //
//   if (paintBold)
//     lw = 2;
//   //
//   if (camCam != NULL)
//   {
//     for (i = 0; i < camCam->getCams()->getCamsCnt(); i++)
//     { // get pose with this age - older and older
//       cam = camCam->getCams()->getCam(i);
//       if (cam != NULL)
//         if (cam->parValid)
//         {
//           camPose.set(cam->pos, cam->rot);
//           mCam = camPose.getRtoMMatrix();
//           // robot pose for this update
//           p1.y = pr.y - roundi(cam->pos.x * ppm);
//           p1.x = pr.x - roundi(cam->pos.y * ppm);
//           if (maxRange < 15.0)
//             // paint also circle at camera position
//             cvCircle(img->cvArr(), p1, 6, red, lw, 8, 0);
//           // line in direction of left pixel (center)
//           pos = cam->getPtoCRob(0, cam->height/2, 1.0);
//           pos.transfer(&mCam);
//           p2.y = pr.y - roundi(pos.x * ppm);
//           p2.x = pr.x - roundi(pos.y * ppm);
//           cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);
//           // line in direction of right pixel (center)
//           pos = cam->getPtoCRob(cam->width, cam->height/2, 1.0);
//           pos.transfer(&mCam);
//           p2.y = pr.y - roundi(pos.x * ppm);
//           p2.x = pr.x - roundi(pos.y * ppm);
//           cvLine(img->cvArr(), p1, p2, red, lw, 8, 0);
//         }
//     }
//   }
}

////////////////////////////////////////////////

void UNavView::paintGmks(UPoseTime seenFromPose)
{
//   int i;
//   CvPoint p1, p2, p3;
//   UPosition posm, posr, pos;
//   CvScalar red = CV_RGB(255, 0, 0);
//   CvScalar blue = CV_RGB(85, 0, 85);
//   int lw = 3;
//   UGmk * gmk;
//   UMatrix4 mGmk;
//   const int MSL = 20;
//   char s[MSL];
//   CvFont font;
//   //
//   cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
//                0.8, 1.0, 0.0, 1, 8);
//   //
//   if (paintBold)
//     lw = 5;
//   //
//   if (camGmk != NULL)
//   {
//     for (i = 0; i < camGmk->getGmkPool()->getGmkCnt(); i++)
//     { // get pose with this age - older and older
//       gmk = camGmk->getGmkPool()->getGmkNum(i);
//       if (gmk != NULL)
//       {
//         mGmk = gmk->getRtoMMatrix();
//         // robot pose for this update
//         p1.y = pr.y - roundi(gmk->getX() * ppm);
//         p1.x = pr.x - roundi(gmk->getY() * ppm);
//         if (maxRange < 15.0)
//           // paint also circle at camera position
//           cvCircle(img->cvArr(), p1, 4, red, lw/2, 8, 0);
//         // line across guidemark
//         pos.set(0.0, 0.12, 0.0);
//         pos.transfer(&mGmk);;
//         p2.y = pr.y - roundi(pos.x * ppm);
//         p2.x = pr.x - roundi(pos.y * ppm);
//         pos.set(0.0, -0.12, 0.0);
//         pos.transfer(&mGmk);;
//         p3.y = pr.y - roundi(pos.x * ppm);
//         p3.x = pr.x - roundi(pos.y * ppm);
//         cvLine(img->cvArr(), p2, p3, red, lw, 8, 0);
//         snprintf(s, MSL, "%lu", gmk->getCodeInt());
//         p1.x += 5;
//         p1.y -= 5;
//         cvPutText(img->cvArr(), s, p1, &font, blue);
//       }
//     }
//   }
}

///////////////////////////////////////////////

void UNavView::printRefSystems()
{
  int i,j;
  UPose p;
  UTime t;
  const int MSL = 30;
  char s[MSL];
  //
  if (poseOdo != NULL)
    t = poseOdo->getNewest().t;
  else if (poseUtm != NULL)
    t = poseUtm->getNewest().t;
  else if (poseMap != NULL)
    t = poseMap->getNewest().t;
  printf("source \\ dest: --------------odo--------------   --------------utm--------------   --------------map--------------\n");
  printf("current ");
  p = poseOdo->getPoseAtTime(t);
  printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
  p = poseUtm->getPoseAtTime(t);
  printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
  p = poseMap->getPoseAtTime(t);
  printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
  printf("\n");
  for (i = 0; i < MAX_REF_SYS; i++)
  {
    switch(i)
    {
      case 0: printf("odo     "); break;
      case 1: printf("utm     "); break;
      case 2: printf("map     "); break;
      default: break;
    }
    for (j = 0; j < MAX_REF_SYS; j++)
    {
      p = poseToRef[i][j];
      printf(" %12.2fx,%12.2fy,%7.4fh;", p.x, p.y, p.h);
    }
    printf("\n");
  }
  printf("at time %lu.%06lu %s\n", t.getSec(), t.getMicrosec(), t.getDateTimeAsString(s, true));
}

///////////////////////////////////////////////

void UNavView::setRefSystemsHere()
{
  int i, j;
  UResPoseHist *psi, *psj;
  UPose pd, ps, pz1, pz;
  UTime t;
  //
  if (poseOdo != NULL)
    t = poseOdo->getNewest().t;
  else if (poseUtm != NULL)
    t = poseUtm->getNewest().t;
  else if (poseMap != NULL)
    t = poseMap->getNewest().t;
  //
  for (i = 0; i < MAX_REF_SYS; i++)
  { // i is the source system
    switch (i)
    {
      case 0: psi = poseOdo; break;
      case 1: psi = poseUtm; break;
      case 2: psi = poseMap; break;
      default: psi = NULL; break;
    }
    for (j = 0; j < MAX_REF_SYS; j++)
    { // j is the destination system
      if (psi == NULL)
        poseToRef[i][j].clear();
      else
      {
        switch (j)
        {
          case 0: psj = poseOdo; break;
          case 1: psj = poseUtm; break;
          case 2: psj = poseMap; break;
          default: psj = NULL; break;
        }
        if (psj == NULL)
          poseToRef[i][j].clear();
        else
        { // both systems exist
          if (psi == psj)
            // 1:1
            poseToRef[i][j].clear();
          else
          { // both systems exist and is different
            // a coordinate in the i system can be converted to
            // destination system by:
            // poseDest = poseToRef[i][j].getPoseToMapPose(poseSource)
            ps.clear();
            ps = psi->getPoseAtTime(t);
            pd = psj->getPoseAtTime(t);
            // source origo relative current source pose
            pz.clear();
            pz1 = ps.getMapToPosePose(&pz);
            // convert this local coordinate to a map pose in destination system
            poseToRef[i][j] = pd.getPoseToMapPose(pz1);
          }
        }
      }
    }
  }
}

///////////////////////////////////////////

void UNavView::paintPoseHistLines(UPoseTime seenFromPose)
{
  int j;
  UPose pr;
  UResPoseHist * psj;
  //
  for (j = 0; j < MAX_REF_SYS; j++)
  {
    switch (j)
    {
      case 0: psj = poseOdo; break;
      case 1: psj = poseUtm; break;
      case 2: psj = poseMap; break;
      default: psj = NULL; break;
    }
    if (psj != NULL)
    { // system exist, paint lines
      if (poseHist[j] == NULL)
        // create a view object for this pose history group
        poseHist[j] = new UPaintPoseHist(psj, j);
      // get origin of system on viewed system
      pr = poseToRef[j][paintPoseRef];
      // tell object about system
      poseHist[j]->bold = paintBold;
      poseHist[j]->maxHist = paintPoseHistCnt;
      poseHist[j]->setViewCooSys(paintPoseRef, pr);
      poseHist[j]->paint(&seenFromPose, viewer);
    }
  }
}

/////////////////////////////////////////////

UPose UNavView::getSystemOrigin(int myOrigin,
                                 bool * convertNeeded)
{
  UPose result;
  //
  if (myOrigin >= 0 and myOrigin < MAX_REF_SYS)
  { // system exist, paint lines
    result = poseToRef[myOrigin][paintPoseRef];
  }
  else
    result.clear();
  if (convertNeeded != NULL)
    *convertNeeded = myOrigin != paintPoseRef;
  return result;
}

////////////////////////////////////////////////

uint32_t UNavView::getColor(char col)
{
  uint32_t result;
  switch(col)
  { // some inspiration from scilab color_list
    case 'k': result = (0<< 16) + (0 << 8) + 0; break;
    case 'r': result = (255<< 16) + (0 << 8) + 0; break;
    case 'g': result = (0<< 16) + (255 << 8) + 0; break;
    case 'b': result = (0<< 16) + (0 << 8) + 255; break;
    case 'y': result = (200<< 16) + (200 << 8) + 0; break;
    case 'm': result = (200<< 16) + (0 << 8) + 200; break;
    case 'c': result = (0<< 16) + (200 << 8) + 200; break;
    case '0': result = (90<< 16) + (50 << 8) + 50; break;
    case '1': result = (100<< 16) + (60 << 8) + 60; break;
    case '2': result = (110<< 16) + (70 << 8) + 70; break;
    case '3': result = (120<< 16) + (80 << 8) + 80; break;
    case '4': result = (130<< 16) + (90 << 8) + 90; break;
    case '5': result = (140<< 16) + (100 << 8) + 100; break;
    case '6': result = (150<< 16) + (110 << 8) + 110; break;
    case '7': result = (160<< 16) + (120 << 8) + 120; break;
    case '8': result = (140<< 16) + (130 << 8) + 120; break;
    case '9': result = (130<< 16) + (140 << 8) + 120; break;
    case 'p': result = (255<< 16) + (200 << 8) + 200; break; // pink
    case 'w': result = (255<< 16) + (231 << 8) + 186; break; // wheat
    case 'n': result = (0<< 16) + (0 << 8) + 128; break;   // navy
    default:  result = (180<< 16) + (0 << 8) + 180; break;
  }
  return result;
}

////////////////////////////////////////////////////////////

void UNavView::paintPolyItems(UPoseTime seenFromPose)
{
  UPose sysPose;
  bool convert;
  UPosition pos;
  int i, j, im;
  UPolyItem * pi;
  uint32_t magd = (180 << 16) +(0 << 8) + 180; // default polygon colour
  uint32_t mag1 = (240 << 16) + (50 << 8) + 180; // odd number
  uint32_t mag2 = (180 << 16) + (50 << 8) + 240; // even number
  uint32_t mag3 = (240 << 16) + (100 << 8) + 240; // has period in name
  uint32_t scicol;
  uint32_t * col = &magd;
//  int lw;
//  int d = 4,e = 5,f = 6;
  char * c1 = NULL, *c2 = NULL;
//  int a = 1,b = 2,c = 3;
  bool show;
  bool hide;
//  int g = 7,h = 8,r = 9;
  UPaintPolygons * polys = polyClouds;
  bool dots;
  //
  if (resPoly != NULL)
  {
    im = resPoly->getPolysCnt();
    if (im > 0 and polys == NULL)
    { // create polygon paint info structure
      polys = new UPaintPolygons();
      polyClouds = polys;
    }
    for (i = 0; i < im; i++)
    {
      pi = resPoly->getItem(i);
//         lw = 2;
//         if (isdigit(pi->color[1]))
//           lw = pi->color[1] - '0';
//         if (paintBold)
//           lw = lw + 2;
      hide = strlen(paintPolyHide) > 0;
      if (hide)
        hide = pattern_match(pi->name, paintPolyHide);
      show = not hide;
      if (not show and strlen(paintPolyShow) > 0)
        show = pattern_match(pi->name, paintPolyShow);
      if (show and pi->getPointsCnt() > 0)
      { // coordinate conversion option
        sysPose = getSystemOrigin(pi->cooSys, &convert);
        // check name for colour options
        if (pi->color[0] != 'd')
        { // use the specified color
          scicol = getColor(pi->color[0]);
          col = &scicol;
        }
        c1 = strrchr(pi->name, '.');
        if (c1 != NULL)
        {
          c1++;
          j = strtol(c1, &c2, 10);
          if (pi->color[0] == 'd')
          { // default color
            if (c2 == c1)
              col = &mag3;
            else if (j % 2 == 0)
              col = &mag2;
            else
              col = &mag1;
          }
          if (paintPolyNameCnt > 0)
          { // paint polygon name-number
            if ((int)strlen(pi->name) > paintPolyNameCnt)
              c1 = &pi->name[strlen(pi->name) - paintPolyNameCnt];
            else
              c1 = pi->name;
          }
        }
      }
      dots = (pi->color[2] == 'd' or pi->color[3] == 'o');
      polyClouds->setPolygon(viewer, i, pi, show, *col, paintBold, dots, c1, paintPoseRef, sysPose);
    }
//         jm = pi->getPointsCnt();
//         for (j = 0; j < jm; j++)
//         {
//           p1 = toPixels(pi->getPoint(j), seenFromPose, sysPose, convert);
//           if (pi->color[2] == 'd' or pi->color[3] == 'o')
//             cvCircle(img->cvArr(), p1, 3, *col, lw, 8, 0);
//           if (c1 != NULL)
//           { // polygon has number
//             // paint polygon line (sort of) inside polygon
//             if (p1.x > pcog.x + lw)
//               p1.x -= lw;
//             else if (p1.x < pcog.x - lw)
//               p1.x += lw;
//             if (p1.y > pcog.y + lw)
//               p1.y -= lw;
//             else if (p1.y < pcog.y - lw)
//               p1.y += lw;
//           }
//           if (j > 0)
//           { // paint line
//             cvLine(img->cvArr(), p1, p2, *col, lw, 8, 0);
//               // save last painted pose position
//           }
//           else
//             // save first point
//             p0 = p1;
//           p2 = p1;
//         }
//         if (pi->isPolygon())
//           // paint closing line too
//           cvLine(img->cvArr(), p1, p0, *col, lw, 8, 0);
//       }
//     }
  }
}

///////////////////////////////////////////////////

// CvPoint UNavView::toPixels(UPosition pos, int coordinateSystem, UPose seenFromPose)
// {
//   bool convert;
//   UPose systemOrigin;
//   //
//   systemOrigin = getSystemOrigin(coordinateSystem, &convert);
//   return toPixels(pos, seenFromPose, systemOrigin, convert);
// }

///////////////////////////////////////////////////

// CvPoint UNavView::toPixels(UPosition pos, UPose seenFromPose, UPose systemOrigin, bool convert)
// {
//   if (convert)
//     pos = systemOrigin.getPoseToMap(pos);
//   pos = seenFromPose.getMapToPose(pos);
//   return toPixels(pos);
// }

///////////////////////////////////////////////////

// CvPoint UNavView::toPixels(UPosition pos)
// {
//   U2Dpos pd1;
//   double d;
//   CvPoint p1;
//   //
//   pd1.y = pr.y - pos.x * ppm;
//   pd1.x = pr.x - pos.y * ppm;
//   d = pd1.dist();
//   if (d > 10000.0)
//   { // too far away in pixels - limit towards visible area
//     pd1.x *= 10000.0 / d;
//     pd1.y *= 10000.0 / d;
//   }
//   p1.y = roundi(pd1.y);
//   p1.x = roundi(pd1.x);
//   return p1;
// }

//////////////////////////////////////////////////

// void UNavView::mousePan(CvPoint pix, double newScale)
// {
// /*  robotPose.x -= (pix.y - pr.y)/ppm;
//   robotPose.y += (pix.x - pr.x)/ppm;*/
//   UPose newCenter;
//   UPose p2;
//   //
//   // get get click in metric screen coordinates (relative to lower center of screen)
//   newCenter.x = (img->height() - pix.y)/ppm;
//   newCenter.y = (pix.x - img->width()/2.0)/ppm;
//   newCenter.h = 0.0;
// 
// //printf("new center on screen in meters %.2fm, ppm=%.4f, new center = %.2fx,%.2fy(meters)\n", maxRange, ppm, newCenter.x, newCenter.y);
// 
//   // get robot pose relative to this new center position
//   p2 = newCenter.getMapToPosePose(robotPose);
//   // move so that center is screen center rather than bottom of screen - in new scale
//   p2.x += newScale / 2.0;
//   robotPose = p2;
//   maxRange = newScale;
// 
// //printf("after  Pan %.2fm, ppm=%.4f, robot pose = %.2fx,%.2fy,%.4fh\n", maxRange, ppm, robotPose.x, robotPose.y, robotPose.h);
// //printRefSystems();
// }

//////////////////////////////////////////////////

// void UNavView::mouseScale(CvPoint pix, CvPoint dx)
// {
//   double sc; // new scale (height of window in meters)
//   CvPoint nc;
//   double x = absi(dx.x);
//   double y = absi(dx.y);
//   //
//   if (x < 5 or y < 5)
//     sc = maxRange;
//   else
//   {  // new height of image in meters -- zooming in
//     if (x/double(img->width()) > y/double(img->height()))
//       // use width
//       sc = x / double(ppm) * double(img->height()) / double(img->width());
//     else
//       // use height
//       sc = y / double(ppm);
//   }
//   if (sc < 0.2)
//     sc = 0.2;
//   //
//   // new centre of display
//   nc.x = pix.x + dx.x / 2;
//   nc.y = pix.y + dx.y / 2;
//   mousePan(nc, sc);
// }


