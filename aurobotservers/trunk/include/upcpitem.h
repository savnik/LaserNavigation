/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
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

#ifndef URESPCPITEM_H
#define URESPCPITEM_H

#include <cstdlib>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

/**
 * A point cloud iten that can hold a XYZ or a XYZRGB point cloud.
 * In addition there is info about the detection time, the coordinate system (and sensor pose on robot - if relevant).
 * The structure should be locked when reading or writing more than one field, to ensure consistent data. */
class UPcpItem : public UDataBase, public ULock
{
public:
  /**
   * shape type, cloud, polygon or polyline */
  typedef enum {cloud, polygon, polyline} SHAPE_TYPE;
  /**
   * shape type, Empty, PointXyz or PointXyzRgb */
  typedef enum {Empty, PointXyz, PointXyzRgb} POINT_CLOUD_TYPE;
  /**
   * constructor */
  UPcpItem()
  {
    name[0] = '\0';
    relPoseUse = false;
    cooSys = 1; // 1=utm
    updateTime.clear();
    relPose.clear();
    xyz = NULL;
    xyzp = NULL;
    xyzrgb = NULL;
    xyzrgbp = NULL;
    shapeType = cloud;
    cloudType = Empty;
  };
  /**
   * get data type for for this transferable class */
  virtual const char * getDataType()
  { return "pcpitem"; }
  /**
   * Get cloud type (either cloud, a po */
  inline SHAPE_TYPE getShapeType()
  { return shapeType; }
  /**
   * Get cloud type (either cloud, a po */
  inline POINT_CLOUD_TYPE getCloudType()
  { return cloudType; }

  /**
   * Destructor */
  ~UPcpItem() {};
  /**
   * Test name of this pcp item
   * \param thisname is the name to be tested against
   * \returns true if name is identical - is not case sensitive*/
  bool isA(const char * thisname);
  /**
  * Test name of this pcp item against a name, that may hold one final wildcard (a *)
  * i.e. 'foo*' will match 'footprint' and 'foo', but not 'fo'
  * and  'foo*print' will match the same as 'foo*'
  * a '*' will match all
  * \param thisname is the name to be tested against (with optionally one wildcard '*')
  * \returns true if name matches 'thisname' - is not case sensitive*/
  bool match(const char * thisname);
  /**
   * Set name of this pcp item
   * \param newname is the new name for the pcpgon - default is empty - max 32 characters (MNL) */
  void setName(const char * newname);
  /**
   * Print current status to this buffer string
   * \param preStr start by inserting this string into buffer
   * \param buff start of buffer
   * \param bufCnt length of buffer
   * \returns a pointer to the buffer */
  const char * print(const char * preStr, char * buff, const int buffCnt);
  /**
   * code this pcp item in XML format
   * \param buff start of buffer
   * \param bufCnt length of buffer
   * \returns a pointer to the buffer */
  const char * codeXML(char * buf, const int bufCnt);
  /**
   * set pcpgon as updated, i.e. set the update time to now. */
  inline void setUpdated()
  { updateTime.now(); };
  /**
   * Get points count */
  inline int getPointsCnt()
  {
    if (isXYZ())
      return xyz->points.size();
    else if (isXYZRGB())
      return xyzrgb->points.size();
    else
      return 0;
  };
  /**
   * is point cloud a xyz cloud */
  inline bool isXYZ()
  {
    return xyz != NULL;
  };
  /**
   * is point cloud a xyzRGB cloud */
  inline bool isXYZRGB()
  {
    return xyzrgb != NULL;
  };
  /**
   * Is the pointcloud a polygon */
  inline bool isPolygon()
  { return shapeType == polygon; }
  /**
   * Is the pointcloud a poly line */
  inline bool isPolyline()
  { return shapeType == polyline; }
  /**
   * has this cloud color */
  inline bool hasColor()
  { return isXYZRGB(); };
  /**
   * Get needed buffer size to code cloud in ASCII */
  int getNeededBufferSize()
  {
    if (isXYZ())
      return 130 + 30 * getPointsCnt();
    else
      return 130 + 40 * getPointsCnt();
  };
  /** get point position in UPosition type
   * \returns all zeros if not a valid index */
  UPosition getPoint(int idx)
  {
    UPosition pos;
    if (idx >= 0 and idx < getPointsCnt())
    {
      if (isXYZ())
      {
        pcl::PointXYZ p;
        p = xyz->points[idx];
        pos.x = p.x;
        pos.y = p.y;
        pos.z = p.z;
      }
      else
      {
        pcl::PointXYZRGB p;
        p = xyzrgb->points[idx];
        pos.x = p.x;
        pos.y = p.y;
        pos.z = p.z;
      }
    }
    return pos;
  };
  /**
   * Remove a point from point cloud */
  void remove(int idx)
  {
    if (idx >= 0 and idx < getPointsCnt())
    {
      if (isXYZ())
      {
        std::vector <pcl::PointXYZ, Eigen::aligned_allocator_indirection <pcl::PointXYZ > >::iterator position;
        position = xyz->points.begin();
        xyz->points.erase(position + idx);
      }
      else
      {
        std::vector <pcl::PointXYZRGB, Eigen::aligned_allocator_indirection <pcl::PointXYZRGB > >::iterator position;
        position = xyzrgb->points.begin();
        xyzrgb->points.erase(position + idx);
      }
    }
  };
  /**
   * Add this point to the pointcloud */
  inline void add(double x, double y, double z)
  {
    if (xyz == NULL)
      return;
    pcl::PointXYZ p(x, y, z);
    xyz->push_back(p);
  }
  /**
   * Add this point to the pointcloud */
  inline void add(double x, double y, double z, uint32_t rgb)
  {
    if (xyz == NULL)
      return;
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.x = z;
    p.rgba = rgb;
    xyzrgb->push_back(p);
  }
  /**
   * Set cloud as polygon */
  inline void setAsPolygon()
  { shapeType = polygon; }
  /**
   * Set cloud as polyline */
  inline void setAsPolyline()
  { shapeType = polyline; }
  /**
   * Set cloud as just a cloud */
  inline void setAsCloud()
  { shapeType = cloud; }
  /**
   * Copy source cloud to this cloud */
  bool copy(UPcpItem * source);
  /**
   * Copy source cloud to this cloud */
  void copy(pcl::PointCloud<pcl::PointXYZ> * source);
  /**
   * Copy source cloud to this cloud */
  void copy(pcl::PointCloud<pcl::PointXYZRGB> * source);
  /**
   * Is this cloud empty */
  bool isEmpty()
  { return xyz == NULL and xyzrgb == NULL; }
  /**
   * delete point cloud */
  void deleteCloud()
  {
    if (xyz != NULL)
      delete xyz;
    if (xyzp != NULL)
      delete xyzp;
    if (xyzrgb != NULL)
      delete xyzrgb;
    if (xyzrgbp != NULL)
      delete xyzrgbp;
    xyz = NULL;
    xyzp = NULL;
    xyzrgb = NULL;
    xyzrgbp = NULL;
    cloudType = Empty;
  }
  /**
   * Make a cloud of this type */
  void makeCloud(POINT_CLOUD_TYPE type)
  {
    if (type != cloudType and cloudType != Empty)
      // is a type change - delete the old
      deleteCloud();
    //
    switch (type)
    {
      case PointXyz:
        if (xyz == NULL)
        {
          xyz = (pcl::PointCloud<pcl::PointXYZ> *) new pcl::PointCloud<pcl::PointXYZ>();
          xyzp = (pcl::PointCloud<pcl::PointXYZ>::Ptr *) new pcl::PointCloud<pcl::PointXYZ>::Ptr(xyz);
        }
        cloudType = PointXyz;
        break;
      case PointXyzRgb:
        if (xyzrgb == NULL)
        {
          xyzrgb = (pcl::PointCloud<pcl::PointXYZRGB> *) new pcl::PointCloud<pcl::PointXYZRGB>();
          xyzrgbp = (pcl::PointCloud<pcl::PointXYZRGB>::Ptr *) new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(xyzrgb);
        }
        cloudType = PointXyzRgb;
        break;
      default: break;
    }
  }
  /**
   * Clear all points in this point cloud, but maintain name and type */
  void clear()
  {
    switch (cloudType)
    {
      case PointXyz:
        xyz->points.resize(0);
        break;
      case PointXyzRgb:
        xyzrgb->points.resize(0);
        break;
      default:
        break;
    }

  }
  /**
   * Save this cloud as a PCD file (ascii),
   * \returns true if saved. */
  bool savePCD(const char * filename);

public:
  /// XYZ point cloud (maintained by the pool)
  pcl::PointCloud<pcl::PointXYZ> * xyz;
  /// pointer to xyz cloud (maintained by the pool)
  pcl::PointCloud<pcl::PointXYZ>::Ptr * xyzp;
  /// XYZRGB cloud
  pcl::PointCloud<pcl::PointXYZRGB> * xyzrgb;
  /// pointer to xyzcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr * xyzrgbp;
  /** size of names */
  static const int MNL = 32;
  /** pcpgon name */
  char name[MNL];
  /** coordinate system 0=odo, 1=utm, 2=map. NB! if relPoseUse is true, then the cloud is in sensor coordinates. */
  int cooSys;
  /** if relative pose, the coordinates are relative to this
   * pose (relative to the robot) */
  UPosRot relPose;
  /** use the relative pose, that is the cloud coordinates is the local relPose coordinates,
      and should then be converted to robot coordinates using this relative pose relPose.
      If relPose is false, then the coordinates are as specified in cooSys */
  bool relPoseUse;
  /**
   * Update time for this item, so that updated only can be send */
  UTime updateTime;
  /**
   * sensor detection time - if related to a sensor */
  UTime sensorTime;
  /**
   * is this cloud a polyline, a polygon or just a cloud */
  SHAPE_TYPE shapeType;
  /**
   * is this cloud just a 3d position, or has it color (or is empty) */
  POINT_CLOUD_TYPE cloudType;

};


#endif

