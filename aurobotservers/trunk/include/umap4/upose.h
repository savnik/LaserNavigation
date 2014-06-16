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
#ifndef UPOSE_H
#define UPOSE_H

#include <ugen4/u3d.h>
#include <ugen4/ucommon.h>
#include <ugen4/udatabase.h>
#include <ugen4/u2dline.h>

class UPoseTime;
class UPoseV;
class UPoseTVQ;

/**
Position for a Robot and functions related to such function. <br>
The main thing is the data in format [x,y,Theta]

@author Christian Andersen
*/
class UPose : public UDataBase
{
public:
  /**
  Constructor */
  UPose();
  /**
  Constructor with initial value */
  UPose(double x, double y, double h);
  /**
  Constructor, tahe data from other object */
  UPose(UPose * source);
  /**
  Set from 3D position (in robot coordiantes, <br>
  3D.x is forward, 3D.y is left, 3D.z is up (ignored). <br>
  3D.Omega is tilt -left +right (ignored), 3D.Phi is nose -up +down (ignored)
  3D.Kappa is turn -right +left (heading). */
  UPose(UPosition * pos, URotation * rot);
  /**
  Destructor */
  virtual ~UPose() {};
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "pose";  };
  /**
  Clear to zero. */
  inline virtual void clear()
  {
    x = 0.0;
    y = 0.0;
    h = 0.0;
  };
/*  inline UPose operator= (UPose source)
  {
    x = source.x; y = source.y; h = source.h;
    return *this;
  };*/
  /**
  get a pose from a pose-time object */
  UPose operator= (UPoseTime source);
  /**
  get a pose from a pose-velocity object */
  UPose operator= (UPoseV source);
  /**
  get a pose from a pose-velocity object */
  UPose operator= (UPoseTVQ source);
  /**
  Advance the pose 'dist' distance with an heading change
  over the distance of 'headingChange'. */
  void add(double dist, double headingChange);
  /**
  Set from 3D position (in robot coordiantes, <br>
  3D.x is forward, 3D.y is left, 3D.z is up. <br>
  3D.Omega is tilt -left +right, 3D.Phi is nose -up +down
  3D.Kappa is turn -right +left. */
  void set(UPosition * pos, URotation * rot);
  /**
  Set value from these data */
  void set(double ix, double iy, double ih);
  /**
  Add this increment to present pose */
  void add(double ix, double iy, double ih);
  /**
  Print pose in single line to console, with
  the provided prestring in front of data. */
  void fprint(FILE * fd, const char * prestring);
  /**
  Print pose in single line to a string buffer of length 'bufLng', with
  the provided prestring in front of data.
  Deprecated call - use snprint(...). */
  inline void print(char * buf, const char * prestring, int bufLng)
  { snprint(prestring, buf, bufLng); };
  /**
  Print pose in single line to a string buffer of length 'bufLng', with
  the provided prestring in front of data. */
  inline void print(const char * str)
  {
    const int MSL = 200;
    char s[MSL];
    snprint(str, s, MSL);
    fprintf(stdout, "%s\n", s);
  }
  /**
  Print status for this structure into a string */
  virtual void snprint(const char * preString, char * buff, const int buffCnt);
  /**
  Returns heading in 0-360 degree range. */
  double getHeadingDeg();
  /**
  Returns heading in radians. */
  inline double getHeadingRad()
     { return h; };
  /**
  Get distance to another pose. */
  inline double getDistance(UPose other)
     { return hypot(other.x - x, other.y - y); };
  /**
  Get distance to another pose. */
  inline double getDistance(UPose * other)
  { return hypot(other->x - x, other->y - y); };
  /**
  Get distance to another position (XY-part only). */
  inline double getDistance(UPosition other)
  { return hypot(other.x - x, other.y - y); };
  /**
  Get distance to another position (XY-part only). */
  inline double getDistance(UPosition * other)
  { return hypot(other->x - x, other->y - y); };
  /**
  Get heading difference to another pose. I.e.
  how much angle should I add th this heading to get the other
  (return limitToPi(other.h - h)). */
  inline double getHeadingDiff(UPose other)
     { return limitToPi(other.h - h); };
  /**
  Save data in html-like format.
  Returns true if saved */
  //bool save(Uxml3D * fmap, const char * name = NULL);
  /**
  Load a pose from this xml-class.
  Returns true if read.
  Reports format error to xml-class. */
  //bool load(Uxml3D * fxmap, char * name = NULL);
  /**
  Negate this delta pose. */
  UPose neg();
  /**
  Convert this local position coordinate to map (global) coordinates. The z value is maintained */
  UPosition getPoseToMap(UPosition posePos);
  /**
  Convert this local position coordinate to map (global) coordinates. */
  U2Dpos getPoseToMap(U2Dpos posePos);
  /**
  Convert this local position coordinate to map (global) coordinates. */
  UPosition getPoseToMap(double localX, double localY);
  /**
  Convert this local pose position coordinate to map (global) coordinates. */
  UPosition getPoseToMap(UPose posePos);
  /**
  Convert this local pose position coordinate to map (global) coordinates.
  The pose is assumed to be in pixel coordinates. the function takes and returns
  pixel coordinates. Used if a known local position is first scaled to pixel size and
  the function is then used to get the correct pixel position. */
  CvPoint getPoseToMap(CvPoint mapPos);
  /**
  Convert this map position to local pose coordinates.
  This is done by translating map coordinates to pose position
  followed by rotation to pose-heading. */
  UPosition getMapToPose(UPosition mapPos);
  /**
  Convert this map position to local pose coordinates.
  This is done by translating map coordinates to pose position
  followed by rotation to pose-heading. */
  U2Dpos getMapToPose(U2Dpos mapPos);
  /**
  Convert this map pose (x,y part) to local pose coordinates.
  This is done by translating map coordinates to pose position
  followed by rotation to pose-heading.
  See also getMapToPosePose(UPose * mapPose)*/
  UPosition getMapToPose(UPose * mapPos);
  inline UPosition getMapToPose(UPose mapPos)
  { return getMapToPose(&mapPos); };
  /**
  Convert this map pose (x,y,h) to local pose coordinates.
  This is done by translating map coordinates to pose position
  followed by rotation to pose-heading.
  See also getPoseToMapPose(UPose * mapPose)*/
  UPose getMapToPosePose(UPose * mapPose);
  inline UPose getMapToPosePose(UPose mapPose)
  { return getMapToPosePose(&mapPose); };
  /**
  Convert this local pose position coordinate to map (global) coordinates.
  This is done by rotating with the heading and translating
  with the pose position.
  See also getMapToPosePose(UPose * mapPose)*/
  UPose getPoseToMapPose(UPose poseLocal);
  /**
  Convert this local pose position coordinate to map (global) coordinates.
  This is done by rotating with the heading and translating
  with the pose position.
  See also getMapToPosePose(UPose * mapPose)*/
  inline UPose getPoseToMapPose(double x, double y, double h)
  {
    UPose a(x,y,h);
    return getPoseToMapPose(a);
  };
  /**
  Convert this position to local value (used in UClient) */
  CvPoint getMapToPose(CvPoint mapPos);
  /**
  Get pose position as an UPosition (3D) structure */
  inline UPosition getPos(double z = 0.0)
  {
    UPosition result(x, y, z);
    return result;
  };
  /**
  Subtract 2 poses to get a delta pose from ref to base.
  i.e. if P1 and P2 is two poses, and
  deltaPose Pd = P2 - P1, then P2 is at
  position Pd in local P1 coordinates, and
  P2 = P1 + Pd */
  inline UPose operator-(UPose pRef)
     { return subCed(pRef); };
  UPose operator-(UPoseV pRef);
  UPose operator-(UPoseTime pRef);
  UPose operator-(UPoseTVQ pRef);
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  inline UPose operator+(UPose pDelta)
     { return addCed(pDelta); };
  UPose operator+(UPoseV pDelta);
  UPose operator+(UPoseTime pDelta);
  UPose operator+(UPoseTVQ pDelta);
  /**
  Code the pose in XML-like format.
  The name string is added as a name attribute.
  @todo This is deprecated, use codeXml() */
  char * getAsSml(const char * name, char * buff, int buffCnt);
  /**
   * Code this structure in XML format. The open tag includes
   * any additional XML attributes from the codeXmlAttributes function call
   * \param buf is the character buffer where the attributes are stored
   * \param bufCnt is the amount of buffer space allocated
   * \param extraAttr is an optional extra set of attributes, pre-coded as form "name=\"value\" name=...".
   * \returns a pointer to the buffer */
  virtual const char * codeXml(char * buf, const int bufCnt, const char * extraAttr);
  /**
  * Code this structure in XML format. The open tag includes
  * any additional XML attributes from the codeXmlAttributes function call
  * \param name optional extra name value for a name attribute.
  * \param buf is the character buffer where the attributes are stored
  * \param bufCnt is the amount of buffer space allocated
  * \param extraAttr is an optional extra set of attributes, pre-coded as form "name=\"value\" name=...".
  * \returns a pointer to the buffer */
  virtual const char * codeXml(const char * name, char * buf, const int bufCnt, const char * extraAttr);
  /**
   * Get distance to line described by this line (signed)
   * \param Px,Py is the position to be evaluated
   * \returns the signed distance (left is positive) */
  double getDistToPoseLineSigned(const double Px, const double Py);
  /**
   * Get distance to line described by this line
   * \param Px,Py is the position to be evaluated
   * \returns the distance */
  inline double getDistToPoseLine(const double Px, const double Py)
  { return fabs(getDistToPoseLineSigned(Px, Py)); };
  /**
  Get this pose as a transformation matrix of size 4x4, that converts a
  pose in homogeous coordinates (x,y,h,1) in local coordinates at this pose to
  a pose in the coordinate system in which this pose is
  defined (a more global coordinate system, often called map or global).\n
  intended use (x', y', h',1) = this * (x,y,h,1)'  \n
  for result (x',y',h');
  This may be a faster method when converting many points and if through many
  transformations.
  \returns a UMatrix4 (openCV matrix packed into UMatrix) */
  UMatrix4 asMatrix4x4PtoM();
  /**
  Get this pose as a transformation matrix of size 4x4 for use to 
  convert 3D positions in local coordinates into to the more global system, where
  this pose is defined (z posiition is left s is)
  \returns a UMatrix4 (openCV matrix packed into UMatrix) */
  UMatrix4 asMatrix4x4PtoMPos();
  /**
  Get this pose as a transformation matrix of size 4x4, that converts
  a pose in homogeous coordinates (x,y,h,1) in map coordinates to
  a pose in local coordinates of this pose.\n
  intended use (x', y', h',1) = this * (x,y,h,1)'  \n
  to result (x', y', h');
  This may be a faster method when converting many points and if through many
  transformations.
  \returns a UMatrix4 (openCV matrix packed into UMatrix) */
  UMatrix4 asMatrix4x4MtoP();
  /**
  Get this pose as a transformation matrix of size 3x3, that converts
  2D positions in homogeous coordinates (x,y,1) in local coordinates at this pose to
  2D coordinates in the coordinate system in which this pose is
  defined (a more global coordinate system, often called map or global).\n
  intended use (x', y', w) = this * (x,y,1)'  \n
  where resulting (x,y) = (x'/w, y'/w);
  This may be a faster method when converting many points and if through many
  transformations.
  \returns a UMatrix4 (openCV matrix packed into UMatrix) */
  UMatrix4 asMatrix3x3PtoM();
  /**
  Get this pose as a transformation matrix of size 3x3, that converts
  2D positions in homogeous coordinates (x,y,1) in map coordinates to
  local 2D coordinates in the coordinate system of this pose.\n
  intended use (x', y', w) = this * (x,y,1)'  \n
  where resulting (x,y) = (x'/w, y'/w);
  This may be a faster method when converting many points and if through many
  transformations.
  \returns a UMatrix4 (openCV matrix packed into UMatrix) */
  UMatrix4 asMatrix3x3MtoP();
  /**
  Get rotation matrix when converting from local (pose) coordinates to map or global coordinates. \n
  R = [cos(h)  -sin(h); sin(h) cos(h)]. */
  UMatrix4 asMatrix2x2PtoM();
  /**
  Get rotation matrix when converting from map (or global) coordinates to local pose coordinates. \n
  R = [cos(h)  sin(h); -sin(h) cos(h)]. */
  UMatrix4 asMatrix2x2MtoP();
  /**
  \Returns pose as homogenous [x,y,h,1] row. */
  UMatrix4 asRow4();
  /**
  \Returns pose as homogenous [x,y,h,1] column. */
  UMatrix4 asCol4();
  /**
  \Returns pose as [x,y,h] row. */
  UMatrix4 asRow3();
  /**
  \Returns pose as [x,y,h] column. */
  UMatrix4 asCol3();
  /**
  Set pose from matrix (column or row vector).
  \param mat is a pointer to the matrix.
  \returns this pose. */
  UPose set(UMatrix * mat);
  /**
  Assignment operator from matrix */
  inline UPose operator=(UMatrix4 mat)
  {
    return set(&mat);
  };
protected:

  /**
  Compounding operation Va = Vbase o+ D, where
  Va, Vbase and are all poses and D is the movement from Vb to
  Va in Vb local coordinates, whereas Va and Vbase are in "global"
  coordinates.
  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
  xa = xb + x cos(hb) - y sin(hb);
  ya = yb + x sin(hb) + y cos(hb);
  ha = hb + h;
  Returns the new pose Va.
  (from Lu and Milios (1997)) */
  void asAddC(UPose Vbase, UPose D);
  void addDeltaPose(UPose D);
  inline UPose addCed(UPose D)
  { return addCed(&D); };
  UPose addCed(UPose * D);
  /**
  Inverse compounding operation calculating the
  pose change from Vbase to Va, i.e. D = Va o- Vb.
  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
  x = (xa - xb)cos(hb) + (ya - yb)sin(hb);
  y = -(xa - xb)sin(hb) + (ya - yb)cos(hb);
  h = ha - hb;
  Returns the inverse compound pose.
  (from Lu and Milios (1997)) */
  void asSubC(UPose Va, UPose Vbase);
  void subC(UPose * Vbase);
  inline UPose subCed(UPose Vbase)
  { return subCed(&Vbase); };
  UPose subCed(UPose * Vbase);
  /**
  Reverse the relative pose D, so that it reverses the change from Vbase to Va
  Returns [0,0,0] o- D, or
  -D = subc([0,0,0], D); */
  void asNegC(UPose D);

public:
  /**
  x position (forward or east) */
  double x; //
  /**
  y position (left or north) */
  double y; //
  /**
  heading (Theta) zero in x direction and positive towards y.
  That is right hand coordinates with z pointing up. */
  double h; //
};

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/**
Class to pose information and time of pose information */
class UPoseTime : public UPose
{
public:
  /**
  Constructor */
  UPoseTime();
  /**
  Destructor */
  virtual ~UPoseTime() {};
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "posetime";  };
  /**
  Clear to zero. */
  inline virtual void clear()
  {
    x = 0.0;
    y = 0.0;
    h = 0.0;
    t.clear();
  };
  /**
  Constructor with initial value */
  UPoseTime(double ix, double iy, double ih, UTime it)
  {
    setPt(ix, iy, ih, it);
  }
  /**
  Constructor with initial value */
  UPoseTime(UPose pose, UTime it)
  {
    setPt(pose, it);
  }
  /**
  Set value */
  inline void setPt(double ix, double iy, double ih, UTime it)
  {
    x = ix;
    y = iy;
    h = ih;
    t = it;
  }
  /**
  Set value */
  inline void setPt(UPose pose, UTime it)
  {
    x = pose.x;
    y = pose.y;
    h = pose.h;
    t = it;
  }
  /**
  Set value */
  inline void setPt(double ix, double iy, double ih, double it)
  {
    x = ix;
    y = iy;
    h = ih;
    t.setTime(it);
  }
  /**
  Make interpolation between this pose
  and the 'otherPose', for 'atTime'.
  The method uses direct interpolation for small movements
  or small heading changes, and a slightly better method
  for larger movements, bu not suited for more
  than approx 45 deg heading change.
  If this and 'otherPose' has same time, this pose is returned.
  if 'atTime' is outside interval, pose is extrapolated! */
  UPose getPoseAtTime(UPoseTime otherPose, UTime atTime);
  /**
  Get just pose part */
  UPose getPose()
  {
    UPose result(x, y, h);
    return result;
  };
  /**
  Print value, preceded by 'prestring' */
  void fprint(FILE * fd, const char * preString);
  /**
  Print status for this structure */
  virtual void snprint(const char * preString, char * buff, const int buffCnt);
  /**
   * Set a UPose time from a source pose. This sets the x,y,h only, the time part is untouched. */
  inline UPoseTime operator= (UPose source)
  {
    x = source.x; y = source.y; h = source.h;
    return *this;
  };
  /**
  Assign from base pose */
  UPoseTime operator= (UPoseTVQ source);
  /**
   * Code this structure in XML format. The open tag includes
   * any additional XML attributes from the codeXmlAttributes function call
   * \param buf is the character buffer where the attributes are stored
   * \param bufCnt is the amount of buffer space allocated
   * \returns a pointer to the buffer */
  virtual const char * codeXml(char * buf, const int bufCnt, const char * extraAttr);

public:
  /**
  Time valid for pose */
  UTime t;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/**
Class to pose information and time of pose information */
class UPoseTVQ : public UPoseTime
{
public:
    /**
  Constructor */
  UPoseTVQ();
  /**
  Constructor with initial value */
  UPoseTVQ(UPose pose, UTime t, double v, double q);
  /**
  Constructor with initial value */
  UPoseTVQ(double x, double y, double h, UTime t, double v, double q);
  /**
  Destructor */
  virtual ~UPoseTVQ()
  {};
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "posetvq";  };
  /**
  Clear to zero. */
  inline virtual void clear()
  {
    x = 0.0;
    y = 0.0;
    h = 0.0;
    t.clear();
    vel = 0.0;
    q = 0.0;
  };
  /**
  Subtract 2 poses to get a delta pose from ref to base.
  i.e. if P1 and P2 is two poses, and
  deltaPose Pd = P2 - P1, then P2 is at
  position Pd in local P1 coordinates, and
  P2 = P1 + Pd */
  UPoseTVQ operator-(UPoseV pRef);
  UPose operator-(UPose pRef)
  { return subCed(&pRef); };
  UPose operator-(UPoseTime pRef)
  { return subCed(&pRef); };
  UPose operator-(UPoseTVQ pRef)
  { return subCed(&pRef); };
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  UPoseTVQ operator+(UPoseV pDelta);
  UPose operator+(UPoseTVQ pDelta)
  { return addCed(&pDelta); };
  UPose operator+(UPoseTime pDelta)
  { return addCed(&pDelta); };
  /**
  Assign from base pose */
  inline virtual UPoseTVQ operator= (UPose source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    return *this;
  };
  /**
  Assign from base pose */
//   UPoseTVQ operator= (UPoseV source);
  /**
  Assign from base pose */
//   inline UPoseTVQ operator= (UPoseTVQ source)
//   {
//     x = source.x;
//     y = source.y;
//     h = source.h;
//     t = source.t;
//     vel = source.vel;
//     q = source.q;
//     return *this;
//   };
  /**
  Assign from base pose */
//   inline UPoseTVQ operator= (UPoseTime source)
//   {
//     x = source.x;
//     y = source.y;
//     h = source.h;
//     t = source.t;
//     return *this;
//   };
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  UPoseTVQ operator+(UPose pDelta);
  /**
  Set all variables from doubles */
  inline void set(double ix, double iy, double ih, UTime time, double iv, double qual)
  {
    x = ix;
    y = iy;
    h = ih;
    t = time;
    vel = iv;
    q = qual;
  };
  /**
  Set all variables from pose and velocity */
  inline void set(UPose pose, UTime time, double iv, double qual)
  {
    x = pose.x;
    y = pose.y;
    h = pose.h;
    t = time;
    vel = iv;
    q = qual;
  };
  /**
  Set all variables from other pose */
  inline void set(UPoseTVQ * pose)
  {
    x = pose->x;
    y = pose->y;
    h = pose->h;
    t = pose->t;
    vel = pose->vel;
    q = pose->q;
  };
  /**
  Set all variables from other pose */
  inline void set(UPoseTime * pose)
  {
    x = pose->x;
    y = pose->y;
    h = pose->h;
    t = pose->t;
    vel = 0.0;
    q = 0.0;
  };
  /**
  Set all variables from other pose */
  void set(UPoseV * pose);
  /**
  Get velocity */
  inline double getVel()
  { return vel; };
  /**
  Get the UPose part only */
  inline UPose getPose()
  {
    UPose result(x, y, h);
    return result;
  };
  /**
  Get the UPose part only */
  UPoseV getPoseV();
  /**
  Get the UPose part only */
  inline UPoseTime getPoseTime()
  {
    UPoseTime result(x, y, h, t);
    return result;
  };
  /**
  Set velocity */
  inline void setVel(double velocity)
  { vel = velocity; };
  /**
  Print tatus to string.
  Deprecated call format - use snprint(...) */
  inline void print(const char * prestring, char * buff, const int buffCnt)
  { snprint(prestring, buff, buffCnt); };
  /**
  Print status for this structure */
  virtual void snprint(const char * preString, char * buff, const int buffCnt);
  /**
  Print tatus to console */
  inline void print(const char * prestring)
  {
    const int MSL = 50;
    char s[MSL];
    snprint(prestring, s, MSL);
    printf("%s", s);
  }

public:
  /**
    Actual velocity. velocity may be negative for
    reverse speed. */
    double vel;
    /**
     * Pose quality - this may be dependent on use, but basically a double value associated with pose */
    double q;
};


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/**
This classholds 2 2D position and orientation and the position uncertainty
related to this position, i.e. [x,y,h].
The uncertainty is in the covariance matrix form (3x3)
*/
class UPoseQ
{
public:
  /**
  Constructor */
  UPoseQ();
  /**
  Constructor as copy from other UPosRobQ */
  UPoseQ(UPoseQ * source);
  /**
  Destructor */
  virtual ~UPoseQ();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "poseq";  };
  /**
  Set position and covariance to zero */
  inline void clear()
  {
    pose.clear();
    Q.init(3,3,0.0);
    QiValid = false;
  };
  /**
  Assignment operator. This is needed
  as the matrix holds a pointer, that needs to be moved. */
  inline UPoseQ operator = (UPoseQ source)
  { // move elements properly
    pose = source.pose;
    Q = source.Q;
    QiValid = source.QiValid;
    if (QiValid)
      Qi = source.Qi;
    return *this;
  };
  /**
  Set position of only.
  NB! this do not change the covariance matrix. */
  inline void setPose(UPose iPose)
  {
    pose = iPose;
  };
  /**
  Set position of only.
  NB! this do not change the covariance matrix. */
  inline void setPose(double x, double y, double h)
  {
    pose.set(x, y, h);
  };
  /**
  Add to position estimate.
  NB! this do not change the covariance matrix. */
  inline void addPose(double x, double y, double h)
  {
    pose.add(x, y, h);
  };
  /**
  Set covariance matrix - and invalidate the inverse version */
  inline void setQ(UMatrix4 * mQ)
  {
    if (mQ != NULL)
      Q = *mQ;
    else
      Q.init(3,3,1.0);
    QiValid = false;
  }
  /**
  Get covariance matrix copy. */
  inline UMatrix4 getQ() {return Q;};
  /**
  Get (pointer to) covariance matrix . */
  inline UMatrix4 * getpQ() {return &Q;};
  /**
  Get pointer to inverse of covariance matric.
  If not valid, then calculated. If not possible
  then Qi.err == -1. and QiValid is set to false. */
  inline UMatrix4 * getQi()
  {
    if (QiValid)
      return &Qi;
    else
      return makeQi();
  };
  /**
  Generate inverse of Q. and calculated determinant d.
  If this is possible, then QiValid is set true, if
  not then QiValid is set to false. */
  UMatrix4 * makeQi();
  /**
  Get estimated pose */
  inline UPose getPose() {return pose;};
  /**
  Get pointer to pose */
  inline UPose * getpPose() {return &pose;};
  /**
  Print pose in single line to console, with
  the provided prestring in front of data. */
  inline void printPose(char * prestring) {pose.fprint(stdout, prestring);};
  /**
  Save this qualified pose to html-like file.
  Return true is saved.  */
  //bool save(Uxml3D * fmap, const char * name = NULL);
  /**
  Load a poseQ from this xml-class.
  Returns true if read.
  Reports format error to xml-class. */
  //bool load(Uxml3D * fxmap, char * name = NULL);

protected:
  /**
  Position in [x,y,h] coordiantes, where
  x if forward (or north), y is left (or west) and h is
  heading (or normal mathematic angle in an x,y coordinate system) */
  UPose  pose;
  /**
  Covariance matrix related to this position */
  UMatrix4 Q;
  /**
  Inverse of the covariance matrix Q.
  But is only valid if QiValid is true. */
  UMatrix4 Qi;
  /**
  Determinant of Q, but only valid if QiValid is true. */
  double d;
public:
  bool QiValid;
};




#endif
