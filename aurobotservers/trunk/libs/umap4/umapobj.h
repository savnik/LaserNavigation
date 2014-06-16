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
#ifndef UMAPOBJ_H
#define UMAPOBJ_H

#include <list.h>
//#include <iterator>

#include <ugen4/u3d.h>
#include "upose.h"
#include "umapcommon.h"
#include "umapgmk.h"
#include "umaprobot.h"
#include "umap3dpos.h"

#define MAX_POSE_HISTORY 5


class UMapSegment;
class UMapRel;

/**Map object
  *@author Christian Andersen
  */
class UMapObj
{
public:
  /**
  Constructor */
  UMapObj(UMapSegment * parentSegment);
  /**
  Constructor from known pose and measurement error */
//  UMapObj(UPose zPose, UMatrix4 * zQ, UTime atTime);
  /**
  Constructor from known pose details and measurement error */
  /*
  UMapObj(double x, double y, // position
                 double h,           // heading
                 UMatrix4 * zQ,      // measurement squared error estimate
                 UTime atTime        // position time
                 );
              */
  /**
  Destructor */
  ~UMapObj();
  /**
  Create (if needed) estimated pose (with covariance)
  and set to provided parameters.
  If covariance is NULL, then covariance set to unit matrix. */
  void setEstPoseQ(double x, double y, // position
                 double h,             // heading
                 UMatrix4 * zQ = NULL); // covariance matrix
  void setEstPose(double x, double y, // position
                 double h); // heading
  /**
  Reset object position */
  void clear();
  /**
  Set content of this object as a copy from the source,
  except for parent and ID.
  If source do not contain a sub-elements then that
  existing sub-elements will be maintained.
  NB! any relation relations in source or destination
  will be ignored (left as is).
  Returns true if successful.
  Fails if object is a segment or if
  needed sub-objects could not be created. */
  bool copy(UMapObj * source);
  /**
  Update this measurement to absolute zero with zero covariance */
  bool setInitialPose();
  /**
  Update with this measurement as position only object */
  bool setInitialPose(UPose zPose, UMatrix4 * zQ, UTime atTime);
  /**
  Set object as guidemark from this first measurement */
  bool setInitialGmk(UMapObj * robot, UMapZGmk * measured);
  /**
  Set guidemark information */
  bool setGmk(unsigned long ID, float blockSize, int frameSize);
  /**
  Set guidemark information */
  bool setGmk(UMapGmk * source);
  /**
  Set - make if needed - robot object with the
  following parameters. */
  bool setRobot(unsigned long ID,
                float wheelBase,
                float distSD,
                double odoX,
                double odoY,
                double odoTheta);
  /**
  Set robot inf as copy from source */
  bool setRobot(UMapRobot * source);
  /**
  Set - make if needed - object 3D position, with
  optional orientation and (block) covariance matrix
  for position and orientation separately.
  If covariance is set to NULL, then matrix
  is filled with 10.0 in main diaginal for
  position and orientation (~3pi). */
  bool set3dPos(UPosition pos,
                UMatrix4 * posQ = NULL,
                URotation * ori = NULL,
                UMatrix4 * oriQ = NULL);
  /**
  Set 3d position object information as a copy of source. */
  bool set3d(UMap3dPos * source);
  /**
  Set object as new local map.
  Parent must not be NULL and name must be usable as filename
  (extension will be added on save). */
  bool setSegment(UMapObj * parentObj,
                  unsigned long segID,
                  char * name);
  /**
  Get pointer to this */
  UMapObj * get() { return this;};
  /**
  Get reference to guidemark info (read only) */
  UMapSegment * getSegment() {return toSeg;};
  /**
  Get reference to guidemark info (read only) */
  UMapGmk * getGmk() {return toGmk;};
  /**
  Get reference to last robot info (read only) */
  UMapRobot * getRobot()  {return toRob;};
  /**
  get estimated position */
  inline UPose getPose() { return estPose->getPose();};
  /**
  get pointer to estimated position. */
  inline UPose * getpPose() {return estPose->getpPose();};
  /**
  get pointer to estimated position block. */
  inline UPoseQ getPoseQ() {return *estPose;};
  /**
  get pointer to estimated position block. */
  inline UPoseQ * getpPoseQ() {return estPose;};
  /**
  get estimated orientation */
  inline UMap3dPos * get3d() { return to3d;};
  /**
  Get pointer to update time */
  inline UTime * time() {return &updTime;};
  /**
  Get number of updates for this object. */
  inline int getEstUpdates() { return estUpds;};
  /**
  Set relation to parent map segment.
  This should not be needed, as new objects should be
  created using add(), that assigne ID and parent relation itself */
  inline void setParent(UMapSegment * parentSegment)
  {
    printf("Do not use UMapObj::setParent(..)\n");
    parent = parentSegment;
  };
  /**
  Set estimated pose and covariance from this value */
  void setPose(UPoseQ * source);
  /**
  Set serial number */
  inline void setSerial(unsigned int value) {serial = value;};
  /**
  Set serial number */
  inline void setUpdTime(UTime t) {updTime = t;};
  /**
  Get serial number */
  inline unsigned long getSerial() {return serial;};
  /**
  Get type of object. */
  inline mapObjType getType() { return type;};
  /**
  Get last update tile as UTime type. */
  inline UTime getUpdTime() { return updTime;};
  /**
  Get first relation pointer (as iterator) */
  list<UMapRel *>::iterator beginRel() { return mapRelations.begin();};
  /**
  Get last (end) relation pointer (as iterator) */
  list<UMapRel *>::iterator endRel() { return mapRelations.end();};
  /**
  Compare two elements for sorting */
  bool compareSerial (UMapObj bigger)
     {return  serial < bigger.serial;};
  /**
  Compare two elements for sorting */
  inline bool compareSerial (UMapObj smaller, UMapObj bigger)
    { return smaller.compareSerial(bigger);};
  /**
  Print to console */
  void show(const char * prestring);
  /**
  Update object with new robot measurement data.
  Returns true if the update seems valid, i.e.
  Reset flag is not set.
  Time since last update is less than 10 seconds,
  Heading change is less than o.5 radians,
  Distance traveled is less than 1 meter. */
  //bool updateRobot(UMapRobot * measured);
  /**
  Update robot with this new odometer position */
  void updateOdo(double x, double y,
                 double theta, UTime t,
                 bool testReset);
  /**
  Just move position - and leave the rest as is */
  void moveTo(double x, double y);
  /**
  Save pose history */
  void saveHist();
  /**
  Set as robot object and set error values to provided.
  If reset, then odometer values are reset at next update. */
  //void setErrorValues(float distSD, float thetaSD, bool reset);
  /**
  Set as robot object and set error values to provided.
  Heading SD is calculated from wheel separation B.
  If reset, then odometer values are reset at next update. */
  void setErrorValuesB(float distSD, float B, bool reset);
  /**
  Set object type */
  inline void setType(mapObjType typ) { type = typ;};
  /**
  Get type of this object as string */
  const char * typeAsString();
  /**
  Get the provided object type as string */
  static const char * typeAsString(mapObjType typ);
  /**
  Convert string to object type, the string must
  be the one set by typeAsString() */
  static mapObjType typeFromString(const char * typs);
  /**
  Save map object data to file in html-like format.
  returnes true if saved. */
  bool save(Uxml3D * fmap);
  /**
  Load an object from this xml-class.
  Returns true if read correctly (legally). */
  bool load(Uxml3D * fxmap);
  /**
  Unlink the provided relation from
  the list of relations.
  This is normally called from the relation
  object, when a relation is removed or eliminated. */
  bool unlink(UMapRel * rel);
  /**
  Add a link to a relation */
  void linkAdd(UMapRel * constraint);

protected:
  /**
  Parent segment */
  UMapSegment * parent;
  /**
  Object type marker */
  mapObjType type;
  /**
  Object serial number */
  unsigned long serial;
  /**
  Base position estimate of this object */
  UPoseQ * estPose;
  /**
  Number of updates */
  int estUpds;
  /**
  Time of last update */
  UTime updTime;
  /**
  Object type details, this could be guidemark,
  correlation detaild, object type estimatein etc. */
  UMapGmk * toGmk;
  /**
  Object type details for moving robot with odometry information */
  UMapRobot * toRob;
  /**
  3D position supplement to map position */
  UMap3dPos * to3d;
  /**
  Map segment (local map) */
  UMapSegment * toSeg;
  /**
  List of relation constraints to other objects.*/
  list<UMapRel *> mapRelations;


public:
  /**
  Pose history */
  UPose histPose[MAX_POSE_HISTORY];
  /**
  Used history points */
  int histPoseCnt;
};


#endif
