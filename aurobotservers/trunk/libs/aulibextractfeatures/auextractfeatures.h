/***************************************************************************
 *                                                                         *
 *   \file              auextractfeatures.h                                *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \date              Dec 2006                                           *
 *   \brief             Feature extraction from laserscanner data          *
 *                                                                         *
 *   Feature extraction from laser scanner data is done by means of the    *
 *   split-and-merge principle.                                            *
 *                                                                         *
 *   Data is segmented by a SEF (Successive Edge Following) algorithm and  *
 *   features are extracted by the split-and-merge principle.              *
 *                                                                         *
 *                      Copyright (C) 2006 by DTU                          *
 *                      rse@oersted.dtu.dk                                 *
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


#ifndef AUEXTRACTFEATURES_H
#define AUEXTRACTFEATURES_H

#include <cstdlib>

//#include <urob4/uresbase.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "polarlinefit.h"
#include "circlefit.h"
#include "rangedata.h"

#define SEGMENTS_MAX 50   //!< Maximum number of segments

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class AUEFReturnStruct
{
  public:
  /**
  Constructor */
  AUEFReturnStruct()
  {
    noLines=0;
    noCircles=0;
  }
  /**
  Destructor */
  ~AUEFReturnStruct() {}
  /**
  Print contents of the class */
  void print()
  {
    fprintf(stdout,"AUEFReturnStruct noLines %d   nocircles %d\n",noLines,noCircles);
  }

  /**
  Clear the internal variables.*/
  void clear()
  {
    noLines=0;
    noCircles=0;

    for(int i=0;i<SEGMENTS_MAX;i++)
    {
      lines[i].clear();
      circles[i].clear();
    }
  }

  public:

  int noLines;
  AULine lines[SEGMENTS_MAX];
  int noCircles;
  AUCircle circles[SEGMENTS_MAX];
};

/// \brief Extract features with the Split-and-Merge principle
///
class AUExtractFeatures// : public UResBase
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLine) followed by
  // a descriptive extension for this specific resource
  public:
  /**
  Constructor */
  AUExtractFeatures();
  /**
  Destructor */
  virtual ~AUExtractFeatures();
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
  public:
  // Public functions that this resource provides for all users.
  /// Extract features from a RangeData dataset
  void extractFeatures(RangeData *data_);
  void extractFeatures(RangeData *data_,AUEFReturnStruct *res);
  int getLineCount();
  int getCircleCount();
  int getSegmentCount();
  void segmentToStringPolar(int seg, char *reply);
  void segmentToStringCart(int seg, char *reply);
  PolarLineFit * getSegmentLine(int idx);
  CircleFit * getSegmentCircle(int idx);

  PolarLineFit * findLongestLine(bool rightwise);  // returns longest line

  // config interface functions
  const char * paramName(int p);
  int paramCount();
  void setParam(int p, const char *value);
  int getParam(int p, char *value);

  void setDefaults();

  // functions for split and merge functionality
  void doClusters(double configClusterDiff,double configRangeMin);
  int findSplit(PolarLineFit *line);
  int doSplits(double configSplitMSQ);
  void doMerge(double configMergeMSQ);
  void doFindCircles(int configCircleMinDots,double configCircleMSQ,double configCircleMinR,double configCircleMaxR,double configCircleMinCg,double configCircleMaxCg);
  void doDiscard(int configDiscardDots,double configDiscardSize);
  int getFeatures(AUEFReturnStruct *res);

  protected:
  // Local variables provided by this resource
  /**
    Handle for the log-file */
    //FILE *log;
  /**
    Variable to be used for writing the first line in the log. */
    //bool firstline;
  // Configuration data
    ////////////////////////

  // Split and Merge algorithm mode:
  // Clustering step is mandatory.
  #define EF_DO_SPLIT 1
  #define EF_DO_MERGE 2
  #define EF_SEE_CIRCLES 4
  #define EF_DO_DISCARD 8
  #define EF_NO_LINES 16

    char configMode;           //!< Split and merge configuration
    int configMinSegment;      //!< Minimum number of points in segment

    double configClusterDiff;  //!< Range diff. resulting in cluster split
    double configSplitMSQ;     //!< Mean square error resulting in split
    double configMergeMSQ;     //!< Max mean square error accepted for merges

  // double configDiscardMSQ;    //!< Discard elements exceeding this MSQ
    int configDiscardDots;    //!< Discard elements exceeding this MSQ
    double configDiscardSize;   //!< Discard segments with length/diameter smaller than this

    double configCircleMSQ;    //!< Max MSQ for circle
    double configCircleMaxR;   //!< Max allowed radius for circle
    double configCircleMinR;   //!< Min allowed radius for circle
    double configCircleMinCg;  //!< Min coverage for circle
    double configCircleMaxCg;  //!< Min coverage for circle
    int configCircleMinDots;   //!< Min dots for circle
    double configRangeMin;     //!< Dots below range min. are disregarded

  // Split data
    ///////////////

    RangeData *data;            //!< Laser scanner data

  /// \brief Linked list struct
    struct segment_s{
      char prev;              //!< previous
      char next;              //!< next
      int center;             //!< center angle of segment
      PolarLineFit line;      //!< PolarLineFit object
      CircleFit circle;       //!< CircleFit object
      bool isline;            //!< if false: circle
    } segment[SEGMENTS_MAX];

    int segments;              //!< number of segments allocated

  #define FIRST_SEGMENT 0    //!< sentinel: first segment in list
  #define LAST_SEGMENT 1     //!< sentinel: last segment in list
  #define FIRST_SEGMENT_INDEX 2

  private:
  // Private functions
  /////////////////////

  // functions for linked list of segments
    void addLineSegment(int first, int last); // add new line segment
    void addSegment(); // properly add segment[segments] to linked list
    void removeSegment(int index); // remove a segment
    void resetSegments(); // deletes all segments

};

#endif
