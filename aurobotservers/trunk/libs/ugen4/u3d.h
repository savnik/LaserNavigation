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

#ifndef U3D_H
#define U3D_H

#include "umatrix.h"
#include "ucommon.h"
#include "utime.h"
#include "udatabase.h"

class U2Dpos;
//////////////////////////////////////////////////////////////
/**
general class for 2D position in integer coordinates */
class UPos
{
public:
  /**
  Show the values of this object
  on the console.  */
  void show(const char * prestring);
public:
  int x;
  int y;
};

//////////////////////////////////////////////////////////////

/**
general class for 2D position in float coordinates */
class URPos
{
public:
  /**
  Show the values of this object
  on the console. */
  void show(const char * prestring);
public:
  float x;
  float y;
};

class URotation; // forward declaration
class UPosition;

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/** General class to hold x,y,z position
    x is left, y is up and z is forward
    for 3D coordinates unit is meter */
class UPosition : public UDataBase
{
public:
  /**
  Constructor with zero initial value. */
  UPosition();
  /**
  Destructor. */
  virtual ~UPosition();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "3d";  };
  /**
  Constructor with initial values. */
  UPosition(double ix, double iy, double iz);
  /**
  Constructor with initial values. */
  inline UPosition(double ix, double iy)
  {
    x = ix;
    y = iy;
    z = 0.0;
  };
  /**
  Returns position as [x,y,z] vector.
  column or row. */
  UMatrix4 asVector3(bool column);   // (x,y,z)
  /**
  Returns position as homogenous [x,y,z,1] vector.
  Column or row */
  UMatrix4 asVector4(bool column);
  /**
  \Returns position as homogenous [x,y,z,1] row. */
  inline UMatrix4 asRow4()
  { return asVector4(false); };
  /**
  \Returns position as homogenous [x,y,z,1] column. */
  inline UMatrix4 asCol4()
  { return asVector4(false); };
  /**
  \Returns position as [x,y,z] row. */
  inline UMatrix4 asRow3()
  { return asVector3(false); };
  /**
  \Returns position as [x,y,z] column. */
  inline UMatrix4 asCol3()
  { return asVector3(false); };
  //
  /**
  * Return as translation matrix,
  * Returns: (1,0,0,-x; 0,1,0,-y; 0,0,1,-z; 0,0,0,1). */
  UMatrix4 asMatrix4x4();
  /**
  * Set position from vector. If vector has length 3, then
  * just copy x = pos.m[0], ...
  * if vector has length 4, then scale with pos.m[3].*/
  int copy(UMatrix4 * pos);
  /**
  * Set position from vector. If vector has length 3, then
  * just copy x = pos.m[0], ...
  * if vector has length 4, then scale with pos.m[3].*/
  //int copy(UMatrix4 pos);
  /**
  * Set position as numeric copy of three rotation values.
  * x = rot.Omega, y = rot.Phi, z = rot.Kappa. */
  int copy(const URotation * rot);
  /**
  Add 'pos' to this position. */
  void add(const UPosition * pos);
  /**
  Add 'pos' to this position. */
  void add(const UPosition pos);
  /**
  Add (ix, iy, iz) to this position. */
  void add(const double ix, const double iy, const double iz);
  /**
  Set position to (ix, iy, iz). */
  inline void set(const double ix, const double iy, const double iz)
  { x = ix; y = iy, z = iz; };
  /**
  Set position to of element idx to this value (idx: 0=x, 1=y, 2=z). */
  inline void set(const int idx, const double value)
  {
    if (idx == 0)
      x = value;
    else if (idx == 1)
      y = value;
    else if (idx == 2)
      z = value;
  };
  /**
  Set position from another position */
  inline void setPos(UPosition fromPos)
  {  *this = fromPos; };
  /**
  Get pointer to this UPosition (e.g. from decendent classes). */
  inline UPosition * getPos()
  {  return this; };
  /**
   * Get value from an index (0=x,1=y, 2=z, 3=1.0)
   * \param idx index to value
   * \returns value x,y or z from index, if not in range, then 1.0 is returned */
  inline double get(int idx)
  {
    if (idx == 0)
      return x;
    else if (idx == 1)
      return y;
    else if (idx == 2)
      return z;
    else
      return 1.0;
  }
  /**
  Create a position on the fly from values */
  static UPosition position(double ix, double iy, double iz);
  /**
  Clear position to (0,0,0). */
  void clear(void);
  /**
  Scale thisposition with 'val'. */
  void scale(const double val);
  /**
  Scale this position */
  void operator*= (const double scalar);
  /**
  Add 'pos' to this position */
  void operator+= (const UPosition pos);
  /**
  Subtract 'pos' from this position */
  void operator-= (const UPosition pos);
  /**
  Return this position added with 'pos', but
  leave this position unchanged. */
  UPosition added(UPosition * pos);
  /**
  Return this position added with 'pos', but
  leave this position unchanged. */
  UPosition added(UPosition pos);
  /**
  Return this position added with 'pos' */
  UPosition operator+ (UPosition pos);
  /**
  Return this position subtracted with 'pos', but
  leave this position unchanged. */
  UPosition subtracted(UPosition * pos);
  /**
  Return this position subtracted with 'pos', but
  leave this position unchanged. */
  UPosition subtracted(UPosition pos);
  /**
  Return this position subtracted with 'pos'. */
  UPosition operator- (UPosition pos);
  /**
  Subtract 'pos' from this position. */
  void subtract(const UPosition * pos);
  /**
  return a scaled (or multiplied) version of this position with 'val',
  but leave this position unchanged. */
  UPosition scaled(const double val);
  /**
  return a scaled (or multiplied) version of this position with 'val'. */
  UPosition operator* (const double scalar);
  /**
  Assign position from vector */
  UPosition operator= (UMatrix4 vec);
  /**
  Assign value from rotation */
  UPosition operator = (const URotation rot);
  /**
  Assign value from rotation */
  UPosition operator = (const U2Dpos val);
  //
  /**
  Just print this position to console */
  void println(const char * leadString);
  /**
  Prints x, y and z values after the lead string */
  inline void print(const char * leadString)
    { println(leadString); };
  /**
  Print this position to a string with this 'leadString' */
  void sprint(char * s, const char * leadString);
  /**
  As above function, but with limit on buffer length. */
  virtual void snprint(const char * leadString, char * s,
                          const int bufferLength);
  /**
  Show position value on console */
  void show(const char * leadString /* = NULL */);
  //
  // Load and save to configuration structure
  /**
  Save this position to 'ini' configuration file
  under this 'subject' and with this 'key'.
  returns -1 if no space for data. */
  int SaveToReg(Uconfig * ini, const char * subject, const char * key);
  /**
  Load position from 'ini' configuration file
  under this 'subject' and with this 'key'.
  Returns 0 if successful, else -1 and value set to (0,0,0) */
  int LoadFromReg(Uconfig * ini, const char * subject, const char * key);
  //
  /**
  Returns absolute distance to another position squared */
  double distSq(const UPosition * pTo); // returns square of distance to pTo
  /**
  Returns absolute distance to another position. */
  double dist(const UPosition * pTo);   // returns distance to pTo
  /**
  Returns absolute distance to another position. */
  inline double dist(UPosition To)
     { return dist(&To);};
  /**
  Returns absolute length of this vector (distance to (0,0,0)). */
  inline double dist()
  {
    return sqrt(distSq());
  };
  /**
  Returns squared length of this vector (distance to (0,0,0)). */
  inline double distSq()
  {
    return sqr(x) + sqr(y) + sqr(z);
  };
  /**
  Get the xy distance from 0,0. */
  inline double distXY()
  { return hypot(x, y); };
  /**
  Get the xy distance from second point to this.
  \param p2 is the other point.
  \returns XY distance.*/
  inline double distXY(UPosition p2)
  { return hypot(x - p2.x, y - p2.y); };
  /**
  Scales the position (assumed to be a vector to a unit vector.
  i.e. (x^2 + y^2 + z^2) = 1.
  Returns -1 if zero vector. */
  int toUnitVector();
  /**
  Convert 3D position to pixel position
  using two conversion matrices.
  Normally A matrix translates and rotates to 3D image coordinates (4x4 matrix)
  and b converts to 2D image (top-left oriented coordinates.
  The result is a normalized vector [x (width), y (down), 1] */
  UMatrix4 getPixelPos(UMatrix4 * A,
                       UMatrix4 * b);
  /**
  Transfers this position to new position acording to mA (4x4) */
  void transfer(UMatrix4 * mA); // mA must be 4x4
  /**
  Returns a transferred position of this position, but do not
  change this position. */
  UPosition transferred(UMatrix4 * mA); // mA must be 4x4
  /**
  Save position in html-like format.
  Returns true if saved. */
  bool save(FILE * fmap, const char * key);
  /**
  Load values from comma separated string values in %e format.
  Returns true if all 3 values were read. */
  bool load(const char * valueString);
  /**
  Return tangent point, where a line from this point
  touches a circle with centre in c and radius bc.
  if 'rightSide', then b is on the right side of
  the circle - seen from this point.
  If this point is inside circle, then 'valid'
  will be set false and the center point will be returned.
  The z value of the result will be the same as the centre.
  If 'ang' != NULL, then the coordinate system angle to the tangent point
  will be returned here. */
  UPosition getTangentPointXY(
                          const UPosition centre,
                          const double radius,
                          const bool rightSide,
                          bool * valid,
                          double * ang = NULL);
  /**
  Calculate dot-product with this position (vector)
   * \param v is the other 3d vector (v.x, v.y, v.x)
   * \returns (this * v) = (x*v.x + y*v.y + z*v.z) */
  double dot(UPosition p2);
  /**
  Calculate cross-product with this position (vector)
   * \param p2 is a 3d vector
   * \returns (this x p2) = (y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x) */
  UPosition cross(UPosition p2);
  /**
  Sets this vector as a cross product of 2 vectors */
  inline void cross(UPosition p1, UPosition p2)
  { *this = p1.cross(p2); };
  /**
   * Code this structure in XML format. The open tag includes
   * any additional XML attributes from the codeXmlAttributes function call
   * \param buf is the character buffer where the attributes are stored
   * \param bufCnt is the amount of buffer space allocated
   * \param extraAttr is an optional extra set of attributes, pre-coded with name and value
   * \returns a pointer to the buffer */
  virtual const char * codeXml(char * buf, const int bufCnt, const char * extraAttr);
  /**
  * Code this structure in XML format. The open tag includes
  * any additional XML attributes from the codeXmlAttributes function call
  * \param name optional extra name value for a name attribute.
  * \param buf is the character buffer where the attributes are stored
  * \param bufCnt is the amount of buffer space allocated
  * \param extraAttr is an optional extra set of attributes, pre-coded with name and value
  * \returns a pointer to the buffer */
  virtual const char * codeXml(const char * name, char * buf, const int bufCnt, const char * extraAttr);
  
public:
  double x;
  double y;
  double z;
};

//////////////////////////////////////////////////////////////

/**
General class to hold rotation around x,y,z axiz
 Omega is rotation around x axis
 Phi   is rotation around y axis
 Kappa is rotation around z axis
 (unit is radians - right hand rotation)
 *
 * Normal rotation order is Phi then Omega then Kappa
 * just like a crane with a camera at the top
 * This order is assumed in all camera conversion functions.
*/
class URotation
{
public:
  /**
   * Omega is rotation around the x-axis (tilt up towards cealing is positive) */
  double Omega;
  /**
   * Phi is rotation around the y-axis (rotate to the left is positive) */
  double Phi;
  /**
  Kappa is rotation around the z axis (rotate ccv is positive) */
  double Kappa;
  //
  /**
  Constructor with default initialization. */
  URotation();
  /** destructor */
  virtual ~URotation();
  /**
  Constructor with specific initialization */
  URotation(double iOmega, double iPhi, double iKappa);
  //
  // to vector format
  /**
  Converts parameters to vector format.
  Returns (Omega, Phi, Kappa). */
  UMatrix4 asVector3(); // returns (Omega, Phi, Kappa)
  /**
  Converts parameters to vector format.
  Returns (Omega, Phi, Kappa, 1). */
  UMatrix4 asVector4(); // returns (Omega, Phi, Kappa, 1)
  //
  /**
  Returns a unit vector in z-direction in camera
  coordinates rotated by this rotation,
  assuming this is the camera rotation:
  returns R * (0, 0, -1). */
  UMatrix4 asUnitZVector3CtoW(); // returns R * (0, 0, -1)
  /**
  Returns a unit vector in z-direction in camera coordinates
  rotated by this rotation,
  assuming this is the camera rotation:
  returns R * (0, 0, -1, 1). */
  UMatrix4 asUnitZVector4CtoW(); // returns R * (0, 0, -1, 1)
  /**
  Get rotation matrix around X axis only */
  UMatrix4 asMatrix3x3O();
  /**
  Get rotation matrix around X axis only and transposed */
  UMatrix4 asMatrix3x3Ot();
  /**
  Get rotation matrix around Y axis only */
  UMatrix4 asMatrix3x3P();
  /**
  Get rotation matrix around Y axis only and transposed */
  UMatrix4 asMatrix3x3Pt();
  /**
  Get rotation matrix around Z axis only */
  UMatrix4 asMatrix3x3K();
  /**
  Get rotation matrix around Z axis only and transposed */
  UMatrix4 asMatrix3x3Kt();
  /**
  Get rotation matrix to rotate a position from
  robot coordinates to map coordinates */
  UMatrix4 asMatrix3x3RtoM();
  /**
  Get rotation matrix to rotate a position from
  map coordinates to robot coordinates */
  UMatrix4 asMatrix3x3MtoR();
  /**
  Get rotation matrix to rotate a position from
  map coordinates to robot coordinates. <br>
  Returns 4x4 matrix. */
  UMatrix4 asMatrix4x4MtoR();
  /**
  Get rotation matrix to rotate a position from
  robot coordinates to map coordinates. <br>
  Returns 4x4 matrix. */
  UMatrix4 asMatrix4x4RtoM();
  /**
   Gets rotation and conversion matrix: <br>
        Camera to World (CtoW) and World to Camera (WtoC) <br>
   all based on rotation seen in world coordinate perspective <br>
   with first turn around Y axis (up) left is positiove <br>
        then  elevate around x axis (right) up is positive <br>
        then  rotate around z axis (back) ccv is positive <br>
   translation position (pos) is camera position in world coordinates. */
  UMatrix4 asMatrix3x3WtoC();
  /**
  Phi   (P) is y-axis (turn camera to the left is positive) <br>
  Omega (O) is x-axis (tilt camera up towards cealing is positive) <br>
  Kappa (K) is z axis (rotate camera ccv is positive) <br>
  usage: matrix from world to camera oriented coordinates is K * O * P (* T). */
  UMatrix4 asMatrix4x4WtoC();
  /**
  Phi   (P) is y-axis (turn camera to the left is positive) <br>
  Omega (O) is x-axis (tilt camera up towards cealing is positive) <br>
  Kappa (K) is z axis (rotate camera ccv is positive) <br>
  Matrix from world to camera oriented coordinates is K * O * P * pos). <br>
  Usage: Xc = asMatrix4x4WtoC(&pos) * Xw, <br>
  where Xc is 3D coordinates in camera perspective and Xw is the same in world
  perspective. */
  UMatrix4 asMatrix4x4WtoC(UPosition * pos);
  /**
  Makes rotation translation matrix from map coordiantes to
  robot coordinates,with the following robot coordinates
  X is forward, Y is left, Z is Up. <br>
  (Right hand coordinates) <br>
  Phi   (P) is y-axis (robot nose up or down down is positive) <br>
  Omega (O) is x-axis (tilt robot left or right, right is positive)<br>
  Kappa (K) is z axis (turn robot left or right, left is positive)<br>
  pos   (T) is is position of robot in world coordinates (meter).
  Returns a 4x4 matrix. */
  UMatrix4 asMatrix4x4MtoR(UPosition * pos);
  /**
  Same as above, but full position as parameter */
  inline UMatrix4 asMatrix4x4MtoR(UPosition pos) { return asMatrix4x4MtoR(&pos);};
  /**
  Makes rotation translation matrix from robot coordiantes to
  map coordinates,with the following robot coordinates. <br>
  X is forward, Y is left, Z is Up.
  (Right hand coordinates) <br>
  Phi   (P) is y-axis (robot nose up or down down is positive) <br>
  Omega (O) is x-axis (tilt robot left or right, right is positive) <br>
  Kappa (K) is z axis (turn robot left or right, left is positive) <br>
  pos   (T) is is position of robot in world coordinates (meter).
  Returns a 4x4 matrix. */
  UMatrix4 asMatrix4x4RtoM(UPosition * pos);
  /**
  Same as above, but full position as parameter */
  inline UMatrix4 asMatrix4x4RtoM(UPosition pos) { return asMatrix4x4RtoM(&pos);};
  /**
  Get transformation matrix to convert a 3D position [x,y,z] from camera
  to world perspective using both camera rotation only.
  Matrix is calculated as: P' * O' * K' <br>
  usage: Xw = asMatrix4x4CtoW(&pos) * Xc <br>
  where Xc is 3D coordinates in camera perspective and Xw is the same in world
  perspective. */
  UMatrix4 asMatrix3x3CtoW();
  /**
  Get transformation matrix to convert a 3D position [x,y,z,1] from camera
  to world perspective using both camera rotation only.
  Matrix is calculated as: P' * O' * K' <br>
  usage: Xw = asMatrix4x4CtoW(&pos) * Xc <br>
  where Xc is 3D coordinates in camera perspective and Xw is the same in world
  perspective. */
  UMatrix4 asMatrix4x4CtoW();
  /**
  Get transformation matrix to convert a 3D position [x,y,z,1] from camera
  to world perspective using both camera rotation and position.
  Matrix is calculated as: T(-pos) * P' * O' * K' <br>
  usage: Xw = asMatrix4x4CtoW(&pos) * Xc <br>
  where Xc is 3D coordinates in camera perspective and Xw is the same in world
  perspective. */
  UMatrix4 asMatrix4x4CtoW(UPosition * pos);
  //
  // basic operations
  /**
  Copy rotation data from vector , assuming that first element is Omega, second
  is Phi and third is Kappa. If size 4 vector then fourth value is assumed scale. */
  int copy(UMatrix4 * rot);
  /**
  Function as copy described above. */
  //int copy(const UMatrix4 rot);
  /**
  Copy a vector to a rotation type, from vector [Omega, Phi, Kappa], or
  from vector [w * Omega, w * Phi, w * Kappa, w]. */
  URotation operator = (UMatrix4 vec);
  /**
  Copy a 3D position as a rotation type. */
  URotation operator = (UPosition pos)
  {
    copy(&pos);
    return *this;
  };
  /**
  Make a sum of two rotation sets */
  URotation operator+ (URotation rot);
  /**
  Make diffrence of two rotation sets */
  URotation operator- (URotation rot);
  /**
  Make scaled version of a rotation set */
  URotation operator* (double val);
  /**
  Add an angle set to this value */
  void operator+= (URotation rot);
  /**
  Subtract an angle set to this value */
  void operator-= (URotation rot);
  /**
  Scale this rotation set with a factor */
  void operator*= (double val);
  /**
  Copy values from a position, transferring Omega = x, Phi = y, Kappa = z. */
  int copy(UPosition * pos);
  /**
  Just add parameter rotation to this and adjust result to within +/- PI. */
  void add(URotation * rot);
  /**
  Just as above, but parameter is not a pointer  */
  void add(URotation rot);
  /**
  Set value directly */
  void set(double iOmega, double iPhi, double iKappa);
  /**
  Set value directly */
  inline void setRot(URotation rot)
  { *this = rot; };
  /**
  Get a pointer to this URotation value (for decendent classes) */
  inline URotation * getRot()
  { return this; };
  /**
  Add values directly, and adjust result to +/- PI; */
  void add(double iOmega, double iPhi, double iKappa);
  /**
  Return a value, that is the sum of this and the parameter.
  This value is unchanged. */
  URotation added(URotation * rot);
  /**
  Returns a value where the referenced is subtracted. This value is unchanged.*/
  URotation subtracted(URotation * rot);
  /**
  Subtract parameter rotation from this, and adjust the result to within +/- PI.*/
  void subtract(URotation * rot);
  /**
  Return a scaled verion of this rotation, caled with this factor. */
  URotation scaled(double val);
  /**
  Scale this rotation with provided factor. */
  void scale(double val);
  /**
  Limit Omega, Phi and Kappa to within the -Pi to -Pi limit. */
  void LimitToPi();
  /**
  lower initial letter version.
  Limit Omega, Phi and Kappa to within the -Pi to -Pi limit. */
  inline void limitToPi() { LimitToPi();};
  /**
  Set all values to zero. */
  void clear(void);
  //
  // debug print
  /**
  Print the values of the rotation to the console directly, with
  the provided string as lead in string. */
  void print(const char * leadString);
  /**
  Print the values of the rotation to the console directly, with
  the provided string as lead in string. */
  inline void show(const char * leadString) {print(leadString);};
  /**
  Print values to string 's' with 'leadString' in front.
  Values are in degrees if 'inDegree' is true, else in
  radians. */
  void sprint(char * s, const char * leadString, bool inDegree = true);
  /**
  As above function, but with limit on buffer length. */
  void snprint(const char * leadString,
               bool inDegree, // rather than radians
               char * s, const int bufferLength);
  //
  /**
  Save rotation values to configuration structure under the provided
  subject and key. If subject or key do not exist then it is created. */
  int SaveToReg(Uconfig * ini, const char * subject, const char * key);
  /**
  Load rotation values from configuration structure 'ini' as found
  under the provided subject and key. If subject or key do not exist,
  then rotation is cleared. */
  int LoadFromReg(Uconfig * ini, const char * subject, const char * key);
  /**
  Save rotation in html-like format.
  Returns true if saved. */
  bool save(FILE * fmap, const char * key);
  /**
  Load values from comma separated string values in %e format.
  Returns true if all 3 values were read. */
  bool load(const char * valueString);
  /**
  Set rotation from these 2 vectors pointing in Y and Z direction (robot coordinate type).
  Rotation is set as azimult first - a rotation arounf Z axis (positive CCV) (kappa).
  then as elevation +/- PI/2 around the Y axis (Phi) positive down.
  And finally rotation around the x axis (+/- Pi) - positive to the right (Omega) */
  void setFromYZ(UPosition posY, UPosition posZ);
  /**
  * Code this structure in XML format. The open tag includes
  * any additional XML attributes from the codeXmlAttributes function call
  * \param buff is the character buffer where the attributes are stored
  * \param buffCnt is the amount of buffer space allocated
  * \param extra is an extra attribute string that is added to the tag
  * \returns a pointer to the buffer */
  virtual const char * codeXml(char * buff, const int buffCnt, const char * extra);
  /**
  * Code this structure in XML format. The open tag includes
  * any additional XML attributes from the codeXmlAttributes function call
  * \param optional name value for a name attribute (if NULL, no name attribute.
  * \param buff is the character buffer where the attributes are stored
  * \param buffCnt is the amount of buffer space allocated
  * \param extra is an extra attribute string that is added to the tag
  * \returns a pointer to the buffer */
  virtual const char * codeXml(const char * name, char * buff, const int buffCnt, const char * extra);
};

/////////////////////////////////////////////////////

int DoFindPeak(     const float x1, const float y1,
                    const float x2, const float y2,
                    const float x3, const float y3,
                    double * xr, double * yr);
// finds the peak, by matrix evaluation of
// A x X = B
// The three (x,y) points is fitted on
// a second order curve d = ax + bx*x + c
// the x-value of the peak is returned in xr,
// and y-value at that peak is returned in yr.
// dy/dx = a + 2*b*x = 0 => x=-a/(2*b)
// if no solution then err < 0
// if peak is a max, then err = +1.

//////////////////////////////////////////////////////


/**
An oriented plane in 3D space.
The plane spans the x-y axis, and its primary orientation (up) is the y-axis.
The definition of the oriented plane is the position of the origo and a point on
the x and y (in-plane) coordinates.
The class holds functions to convert to and from a position-rotation type specification,
and to to and from the plane coordinate specification.
This could be used as a camera image plane, and the corresponding 3D coordinate system as
the camera coordinates. */
class UOriPlane
{
public:
  /**
  Default constructor */
  UOriPlane();
  /**
  destructor */
  ~UOriPlane();
  /**
  Set to valid plane in origo */
  void clear();
  /**
  Set plane from position and rotation */
  void set(UPosition pos, URotation rot);
  /**
  Print to console the position of the plane */
  void print(const char * prestring);
  /**
  Ensure vectors are unit vectors */
  void norm();
  /**
  Get world coordinates of a position in plane coordinates */
  UPosition getRobToWorld(UPosition pos);

protected:
  /**
  origo position - default (0.0, 0.0, 0.0) */
  UPosition poso;
  /**
  A position on the x-axis
  Default (1.0, 0.0, 0.0)*/
  UPosition posx; // a position on the xaxis
  /**
  A position on the y-axis
  Default (1.0, 0.0, 0.0)*/
  UPosition posy; // a position on the yaxis
};


#endif

