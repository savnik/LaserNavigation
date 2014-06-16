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
#ifndef URESNAVIFMAN_H
#define URESNAVIFMAN_H

#include <urob4/uresvarpool.h>
#include <urob4/uresifbase.h>
#include <umap4/umanseq.h>

/**
Class extended from UManSeq to hold added information, when used on client side for display purposes. */
class UClientManSeq : public UManSeq
{
public:
  /**
  Constructor */
  UClientManSeq()
  {
    isACrash = false;
  };
  /**
  Destructor */
  virtual ~UClientManSeq()
  {};
  /**
  Is this sequence the best (the used) path */
  inline bool isBest()
  { return pathUsed; };
  /**
  Set this manoeuvre sequence from this tag - and subsequent tags */
  bool setFromTag(USmlTag * tag);
  /**
  get first of mid-poses tested in generation of path */
  inline UPose * getPoses()
  { return poses;};
  /**
  Get number of available poses on poses array */
  inline int getSegsCnt()
  { return segsCnt; };
  /**
  get first of mid-poses tested in generation of path */
  inline ULineSegment * getSegs()
  { return segs;};
  /**
  Get number of available poses on poses array */
  inline int getPosesCnt()
  { return posesCnt; };
  /**
  Is path crashed in the planning process, i.e. did not find a obstacle free route
  to the destination pose. */
  inline bool isPathFailed()
  { return isACrash; }
  /**
  Get character array wit tope information on the line segments associiated with
  this path */
  char * getSegChars()
  { return segsChar; };
  /**
   * Get the manoeuvre update time */
  UTime getUpdTime()
  { return updTime; };

protected:
  /**
  Manoeuvre flag for a crashed path - probably just partially completed
  until an impossible passage were discovered, but may be available for analysis purposes. */
  bool isACrash;
  /**
  Update time */
  UTime updTime;
  /**
  Flag to say that this path is the selected best path */
  bool pathUsed;
  /**
  Maximum number of tested poses in generation of this path */
  static const int MAX_TESTED_POSES = 50;
  /**
  Poses tested during generation of path */
  UPose poses[MAX_TESTED_POSES];
  /**
  Valid route points */
  int posesCnt;
  /**
  Maximum number of tested lines in generation of this path */
  static const int MAX_TESTED_LINES = 500;
  /**
  No-visibility segments and tangents used when
  generating this path */
  ULineSegment segs[MAX_TESTED_LINES];
  /**
  Number of generated segmnents in avoidance process */
  int segsCnt;
  /**
  A character for each segment to be able to discriminate in display.
  The first character in the name segment is stored here. */
  char segsChar[MAX_TESTED_LINES];
};

/**
Client interface for basic scan data

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UResNavIfMan : public UResIfBase //UClientFuncBase , public UResVarPool
{
public:
  /**
  Constructor */
  UResNavIfMan()
  {
    setResID(getResClassID(), 200);
    UResNavIfManInit();
  };
  /**
  Destructor */
  virtual ~UResNavIfMan();
  /**
   * Initialize class */
  void UResNavIfManInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "navMan"; };
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name();
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList();
  /**
  Got fresh data destined to this function. */
  virtual void handleNewData(USmlTag * tag);
  /**
  The server has set (or changed) the namespace */
//  virtual void changedNamespace(const char * newNamespace);
  /**
  Fixed name of this resource type */
/*  static const char * getResID()
  { return "navMan"; };*/
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 173; };*/
  /**
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Get number of available manoeuvre sequences available (valid) */
  inline int getMansCnt()
  { return mansCnt; };
  /**
  Get first pointer to the array of manoeuvre sequences (first element of a pointer array) */
  inline UClientManSeq ** getMans()
  { return mans; };

public:
  /**
   * The varPool has methods, and a call to one of these are needed.
   * Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
   * If the returnStruct and returnStructCnt is not NULL, then
  a number (no more than initial value of returnStructCnt) of
  structures based on UDataBase may be returned into returnStruct array
  of pointers. The returnStructCnt should be modified to the actual
  number of used pointers (if needed). */
/*  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);*/

protected:
  /**
  Called when a new set of manoeuvres are received.
  The selected manoeuvre is used as parameter. */
  virtual void gotNewData();
  /**
  Create variables for this resource */
  void createBaseVar();
  /**
  Decode laser scan parameters */
  bool handleManData(USmlTag * tag);

protected:
  /**
  Max number of stored manoeuver alternatives */
  static const int MAX_STORED_MANS = 50;
  /**
  Man sequences */
  UClientManSeq * mans[MAX_STORED_MANS];
  /**
  Index to the best of the alternative manoeuvre sequences */
  int manBest;
  /**
  Number of valid manoeuvre alternatives */
  int mansCnt;
  /**
  Call display dunction when new data is available */
  bool callDispOnNewData;
  /**
  Index to manoeuvre count variable */
  UVariable * varMansCnt;
  /**
  Index to latest update time */
  UVariable * varManTime;
};


#endif
