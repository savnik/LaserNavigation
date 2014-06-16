/***************************************************************************
 *   Copyright (C) 2004 by Christian Andersen                              *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UMAPROBOT_H
#define UMAPROBOT_H

#include <ugen4/u3d.h>
#include <ugen4/utime.h>

#include "upose.h"
/**
Robot (measurement) iformation.
Stores robot specific information in addition to the more generic map information
in the map object. */
class UMapRobot
{
public:
  /**
  Constructor */
  UMapRobot();
  /**
  Destructor */
  ~UMapRobot();
  /**
  Clear measurement and including ID and size */
  void clear();
  /**
  Set Guidemark code.
  Code -1 is invalid */
  void setID(const unsigned long ID);
  /**
  Make this object a copy og source object */
  void copy(UMapRobot * source);
  /**
  Get ID */
  inline const unsigned long getID() { return code;};
  /**
  Set measurement position.
  if optional parameter 'reset' is true, then
  this is the first measurement, so no
  compare with previous measurement is meaningful.
  Returns false if no valid previous odometer position
  is valid, i.e. odometer position is reset to
  provided position. */
  bool setOdoPosition(float x, float y,
                    float theta,
                    UTime t,
                    bool testReset,
                    bool doReset);
  /**
  Set position measurement error parameters */
  // void setErrorValues(float distSD, float thetaSD);
  /**
  Set position measurement error parameters
  from known distance SD (meter per moved meter)
  and wheel base B (meter) */
  void setErrorValuesB(float distSD, float B);
  /**
  Set odometer reset flag */
  inline void setOdoReset(bool value) {odoReset = value;};
  /**
  Set wheel base (distance between odometer wheels */
  void setWheelBase(double B);
  /**
  Set wheel base (distance between odometer wheels */
  inline double getWheelBase() {return wheelBase;};
  /**
  Set known odometer position of (last) update.
  The theta is not set (left as is). */
  void setOdo(double x, double y);
  /**
  Set known odometer position of (last) update. */
  void setOdo(double x, double y, double theta);
  /**
  Get odometer reset flag.
  This flag indicates hat odometer based position is reset
  (has new reference) */
  inline bool getOdoReset() { return odoReset;};
  /**
  Get update time */
  inline UTime getT() { return zT;};
  /**
  Get measured heading (from odometer calculation) */
  inline double getHeading() { return zTheta;};
  /**
  Get measured pose */
  UPose getPose();
  /**
  Get heading SD */
  inline double getHeadingSD() { return headingSD;};
  /**
  Get distance SD */
  inline double getDistanceSD() { return distanceSD;};
  /**
  Is heading SD calculated from wheel-base, i.e. is stearing
  done by controlling left and right drive wheel. */
  inline bool isHeadingSDfromWheelBase() { return headingSDfromWheelBase;};
  /**
  Get moved distance from this new odometer
  x,y position */
  double getMovedDistance();
  /**
  Get heading change.
  Returned value is within +/- PI */
  // double getDeltaHeading(UMapRobot * from);
  /**
  Get heading change to this new heading.
  Returned value is within +/- PI */
  double getDeltaHeading();
  /**
  Get covariance matrix for this moved distance.
  The matrix is a 2x2 of the form [Vd, 0; 0, Vtheta].*/
  UMatrix4 getQ(double dist);
  /**
  Assign new values from measurement */
  UMapRobot operator= (UMapRobot source);
  /**
  Get heading to new position relative to old heading */
  inline double getToNewHeading() { return toNewXY;} ;
  /**
  Get last calculated turn radius (from last odometer update) */
  inline double getTurnRadius() { return turnRadius;};
  /**
  Get movement of left wheel since last update */
  inline double getdL() { return dL;};
  /**
  Get movement of right wheel since last update */
  inline double getdR() { return dR;};
  /**
  Get last movement in local map coordinates */
  inline double getdx() { return dx;};
  /**
  Get last movement in local map coordinates */
  inline double getdy() { return dy;};
  /**
  Get last x-position in robot odometer coordinates */
  inline double getzX() { return zX;};
  /**
  Get last y-position in robot odometer coordinates */
  inline double getzY() { return zY;};
  /**
  Get last Theta (heading) in robot odometer coordinates */
  inline double getzTheta() { return zTheta;};
  /**
  Get last odometer update time */
  inline UTime getzTime() { return zT;};
  /**
  SAve robot info in html-like format.
  Returns true if saved. */
  bool save(Uxml3D * fxmap, const char * name = NULL);
  /**
  Load robot odometry information from a xml-class object.
  Returns true if read. */
  bool load(Uxml3D * fxmap, char * name = NULL);

private:
  /**
  Robot (unique) ID */
  unsigned long code;
  /**
  Measurement position (2D) */
  double zX, zY;
  /**
  Last moved distance */
  double dx, dy;
  /**
  Last moved distance for each wheel */
  double dL, dR;
  /**
  Turn radius (1e5-1 if straight) */
  double turnRadius;
  /**
  Measurement heading (newest) */
  double zTheta;
  /**
  Heading change since last update */
  double dTheta;
  /**
  angle from old heading to new x,y position */
  double toNewXY;
  /**
  Wheel separation */
  double wheelBase;
  /**
  measurement heading SD for one meter distance travelled */
  double headingSD;
  /**
  Heading SD is set from wheel base. If false, then
  heading SD is set directly independent of wheelbase. */
  bool headingSDfromWheelBase;
  /**
  measurement distance SD for one meter traveled distance */
  double distanceSD;
  /**
  Measurement time */
  UTime zT;
  /**
  Is odometer position reset. this means that this position
  can not be compared with the previous, and that
  no real update can be performed with this data. */
  bool odoReset;
};



#endif
