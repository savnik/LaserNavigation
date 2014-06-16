/***************************************************************************
 *                                                                         *
 *   \file              auextractfeatures.cpp                              *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \author            Lars Valdemar Mogensen                             *
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


using namespace std;

#include "polarlinefit.h"
#include "auextractfeatures.h"

///////////////////////////////////////////

AUExtractFeatures::AUExtractFeatures()
{ // these first two lines is needed
  // to save the ID and version number
  //setResID(getResID());
  //resVersion = getResVersion();
  // other local initializations
  // set default configuration vars
  setDefaults();
  resetSegments();
}

///////////////////////////////////////////

AUExtractFeatures::~AUExtractFeatures() {
}

///////////////////////////////////////////

const char * AUExtractFeatures::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s ExtractFeatures\n", preString);
  return buff;
}

/////////////////////////////////////
// ExtractFeatures implementation
//

/** \brief Reset all segments */
void AUExtractFeatures::resetSegments()
{
  // Create segment sentinels
  segments = FIRST_SEGMENT_INDEX;  // =2

  segment[FIRST_SEGMENT].prev = -1;
  segment[FIRST_SEGMENT].next = LAST_SEGMENT;
  segment[FIRST_SEGMENT].center = -10000; // must be smaller than any angle

  segment[LAST_SEGMENT].prev = FIRST_SEGMENT;
  segment[LAST_SEGMENT].next = -1;
  segment[LAST_SEGMENT].center = 10000;   // must be larger than any angle

}

/// \brief Add one segment to list
void AUExtractFeatures::addSegment()
{
  // printf("add seg\n");

  int i,center,p;

  // traverse segments looking for the _next_ segment

  center = segment[segments].center;

  for (i = FIRST_SEGMENT; segment[i].center < center; i = segment[i].next);

  // insert into chain, before element i
  p = segment[i].prev;
  segment[i].prev = segments;
  segment[p].next = segments;
  segment[segments].prev = p;
  segment[segments].next = i;

  segments++;
}


/**
  \brief Add line segment to list

  \param first, last \n
  first and last data point in data range for segment.

*/
void AUExtractFeatures::addLineSegment(int first, int last)
{
  if ((last - first + 1) >= configMinSegment) {
    if (segments < SEGMENTS_MAX) {
      // Segment accepted, add it
      segment[segments].line.fitRange(data, first, last);
      segment[segments].center = (last+first)/2;
      segment[segments].isline = true;
      addSegment();
    }
    else
      printf("WARNING: Segment overflow!\n");
  }
}


/**
  \brief Remove segment from list

  \param index
  Index in the linked list to be removed

*/
void AUExtractFeatures::removeSegment(int index)
{
  // printf("remove seg\n");

  int i,j;

  if (index >= segments)
    // invalid index
    return;

  // Remove index from chain
  i = segment[index].prev;
  j = segment[index].next;
  segment[i].next = j;
  segment[j].prev = i;

  if (index < (segments-1)) {
    // move the last segment to this place
    segment[index] = segment[segments-1];

    // update chain
    i = segment[index].prev;
    j = segment[index].next;
    segment[i].next = index;
    segment[j].prev = index;
  }

  segments--;

}

//  void addLineSegment(int first, int last); // add new line segment
//  void addNewSegment(); // properly add segment[segments] to linked list
//  void removeSegment(int index); // remove a segment


/// \brief Divide laser scanner data into clusters
void AUExtractFeatures::doClusters(double configClusterDiff,double configRangeMin)
{
  // Setup initial segments according to clustering rules
  int i;
  double last_r=0;
  bool insegment=false; // true if currently building up a segment
  int start=0;      // start index of segment
  int outcount=0;   // out of range count

  // Reset line and circle segment count
  resetSegments();

  //csegments = 0;

  // printf("doClusters\n");

  for (i=0; i<data->points; i++) {
    if (!insegment) {
      if ((data->point_r[i] != OUT_OF_RANGE) &&
          (data->point_r[i] > configRangeMin)) {
        // in range, start new segment
        start = i;
        insegment = true;
      }
    }
    else {
      if ((data->point_r[i] != OUT_OF_RANGE) &&
          (data->point_r[i] > configRangeMin)) {
        outcount = 0;
        // in range, check if range diff is too large
        if (fabs(data->point_r[i] - last_r) > configClusterDiff) {
          // too large diff, end segment
          addLineSegment(start, i-1);
          // start new
          start = i;
        }
      }
      else {
        // out of range, end segment
        addLineSegment(start, i-1);
        // not in segment now
        insegment = false;
      }
    }

    last_r = data->point_r[i];
  }

  if (insegment) {
    // end last segment
    addLineSegment(start, data->points - 1);
  }

}


/**
  \brief Find point to split segment

  Finds the point (extremum) with maximum distance to the line. The point cannot
  be an end point.

  \param *line \n
  Pointer to PolarLineFit object

*/
int AUExtractFeatures::findSplit(PolarLineFit *line)
{
  int i,j;
  double d, max;
  int max_index;

  i = line->getFirst();
  j = line->getLast();
  // printf("findSplit i=%i j=%i\n",i,j);

  // first phase, move end points until sign change
  if (line->distToLine(data->point_r[i], data->point_th[i]) > 0)
    for (; (line->distToLine(data->point_r[i], data->point_th[i]) > 0) && (i<j); i++);
  else
    for (; (line->distToLine(data->point_r[i], data->point_th[i]) < 0) && (i<j); i++);

  if (line->distToLine(data->point_r[j], data->point_th[j]) > 0)
    for (; (line->distToLine(data->point_r[j], data->point_th[j]) > 0) && (i<j); j--);
  else
    for (; (line->distToLine(data->point_r[j], data->point_th[j]) < 0) && (i<j); j--);

  // find extremum
  max = 0;
  max_index = i;
  for (; i <= j; i++) {
    d = fabs(line->distToLine(data->point_r[i], data->point_th[i]));
    if (d > max) {
      max = d;
      max_index = i;
    }
  }

  if (max_index == line->getFirst())
    max_index++;

  if (max_index == line->getLast())
    max_index--;

  // printf("findSplit returns %i\n",max_index);

  return max_index;

}


/**
  \brief Splits segments if msq > configSplitMSQ

  Not recursive, a segment is maximally split once.

  Returns number of splits done
*/
int AUExtractFeatures::doSplits(double configSplitMSQ)
{
  int i,j,inext,c;
  PolarLineFit *line;

  c = 0;
  i = segment[FIRST_SEGMENT].next;

  while (i != LAST_SEGMENT) {
    inext = segment[i].next;
    if (segment[i].isline) {
      line = &(segment[i].line);
      if (line->getMSQ() > configSplitMSQ) {
        // this needs to be splitted, find split point
        j = findSplit(line);
        // create new segments based on split
        addLineSegment(line->getFirst(),j);
        addLineSegment(j,line->getLast());
        // remove this segment
        removeSegment(i);
        c++;
      }
    }

    i = inext;
  }

  return c;
}

/**
  \brief Tries to make circles out of neighbouring line segments

*/
void AUExtractFeatures::doFindCircles(int configCircleMinDots,double configCircleMSQ,double configCircleMinR,double configCircleMaxR,double configCircleMinCg,double configCircleMaxCg)
{
  int i,j,k;
  int first,last;
  // PolarLineFit *line;
  CircleFit *circle;
  double msqlines, msqcircle, cg;

  i = segment[FIRST_SEGMENT].next;
  j = segment[i].next;

  while ((i != LAST_SEGMENT) && (j != LAST_SEGMENT)) {

    // k is next segment to check
    k = j;

    // Check if i and j segments should be made a circle
    if (segment[i].isline && segment[j].isline) {
      // both segments are lines:
      first = segment[i].line.getFirst();
      last = segment[j].line.getLast();
      msqlines = segment[i].line.getMSQ() + segment[j].line.getMSQ();
      segment[segments].center = (last+first)/2;

      // first requirement, dots enough for circle?
      if ((last-first+1) >= configCircleMinDots) {

        // try a circle:
        segment[segments].isline = false;
        circle = &(segment[segments].circle);
        circle->fitLines(data, &(segment[i].line), &(segment[j].line));
        msqcircle = circle->getMSQ();

        // Does circle fit requirements?
        cg = circle->getCoverage();
        if ((!circle->getInverted()) &&
            (msqcircle < configCircleMSQ) &&
            (circle->getRadius() < configCircleMaxR) &&
            (circle->getRadius() > configCircleMinR) &&
              (cg < configCircleMaxCg) && (cg > configCircleMinCg)) {
          // Take the circle. Update next segment to check
          k = segment[j].next;

          // Add the circle segment and remove lines
          addSegment();
          removeSegment(i);
          removeSegment(j);
        }
      }
    }

    i = k;
    j = segment[i].next;
  }

}


/**
  \brief Merges segments

  The merge is completed if a new segment will have less MSQ than the previous
  segment.\n
  Both lines and circles are handled.

*/
void AUExtractFeatures::doMerge(double configMergeMSQ)
{
  // For each segment:

  int i,j,k;
  int first,last;
  PolarLineFit *line;
  bool mergeok;

  i = segment[FIRST_SEGMENT].next;
  j = segment[i].next;

  while ((i != LAST_SEGMENT) && (j != LAST_SEGMENT)) {
    // Check if i and j segments can be merged
    mergeok = false;

    if (segment[i].isline && segment[j].isline) {
      // both segments are lines, try merge:
      first = segment[i].line.getFirst();
      last = segment[j].line.getLast();
      segment[segments].center = (last+first)/2;
      segment[segments].isline = true;

      // test msq for line merge
      line = &(segment[segments].line);
      line->fitRange(data, first, last);

      // is line good enough:
      if (line->getMSQ() < configMergeMSQ) {
        mergeok = true;
      }
    }

    if (mergeok) {
      // Save previous segment
      k = segment[i].prev;

      // Add the merged segment and remove others
      addSegment();
      removeSegment(i);
      removeSegment(j);

      // next segments to check:
      i = segment[k].next;
      j = segment[i].next;
    }
    else {
      // No merge, check j and the next
      i = j;
      j = segment[i].next;
    }
  }

}


void AUExtractFeatures::doDiscard(int configDiscardDots,double configDiscardSize)
{
  int i;
  bool discard;

  // This step does not care about the order of segments, and thus just loops
  // through the array, not following the linked list.

  for (i=FIRST_SEGMENT_INDEX; i<segments; i++) {
    discard = false;
    if (segment[i].isline) {
      if (((segment[i].line.getLast() - segment[i].line.getFirst() + 1)
           < configDiscardDots) ||
          (segment[i].line.getLength() < configDiscardSize))
        discard = true;
    }
    else {
      if (segment[i].circle.getRadius()*2 < configDiscardSize)
        discard = true;
    }

    if (discard) {
      removeSegment(i);
      // Test this segment again, as it will be reoccupied
      i--;
    }

  }

}

/**
  \brief Return features to struct

  Interal features extracted via calls to the class are returned.\n
  Both lines and circles are handled.

 */
int AUExtractFeatures::getFeatures(AUEFReturnStruct *res)
{
  // Copy internal data to return struct.

  res->clear();

  for (int i=FIRST_SEGMENT_INDEX; i<segments; i++) {
    if (segment[i].isline) {
      res->lines[res->noLines].start_th=segment[i].line.getStartTh();
      res->lines[res->noLines].end_th=segment[i].line.getEndTh();
      res->lines[res->noLines].r0=segment[i].line.getR0();
      res->lines[res->noLines].th0=segment[i].line.getTh0();

      res->lines[res->noLines].slope=segment[i].line.getSlope();
      res->lines[res->noLines].length=segment[i].line.getLength();
      res->lines[res->noLines].msq=segment[i].line.getMSQ();

      res->lines[res->noLines].points=(abs(segment[i].line.getLast() - segment[i].line.getFirst()) + 1);

      res->lines[res->noLines].polarValid = true;
      res->noLines++;

    }
    else
    {
      res->circles[res->noCircles].cr=segment[i].circle.getCr();
      res->circles[res->noCircles].cth=segment[i].circle.getCth();

      res->circles[res->noCircles].r=segment[i].circle.getRadius();
      res->circles[res->noCircles].coverage=segment[i].circle.getCoverage();
      res->circles[res->noCircles].msq=segment[i].circle.getMSQ();

      res->circles[res->noCircles].points=(abs(segment[i].circle.getLast() - segment[i].circle.getFirst()) + 1);

      res->circles[res->noCircles].polarValid=true;
      res->noCircles++;
    }
  }

  return res->noLines + res->noCircles;

}

/// \brief Initiates the feature extraction from a RangeData object
/// \param *data_ \n Pointer to a RangeData object.
void AUExtractFeatures::extractFeatures(RangeData *data_)
{

  // UTime t;

  // printf("start extract\n");
  // t.Now();

  data = data_;
  doClusters(configClusterDiff,configRangeMin);

  if (configMode & EF_DO_SPLIT) {
    // Do first round of splitting
    doSplits(configSplitMSQ);
    if (configMode & EF_SEE_CIRCLES) {
      // Circles are detected after segments are splitted once
      doFindCircles(configCircleMinDots,configCircleMSQ,configCircleMinR,configCircleMaxR,configCircleMinCg,configCircleMaxCg);
    }
    // Finish splitting
    while(doSplits(configSplitMSQ));
  }

  if (configMode & EF_DO_MERGE)
    doMerge(configMergeMSQ);

  if (configMode & EF_DO_DISCARD)
    doDiscard(configDiscardDots,configDiscardSize);

  if (configMode & EF_NO_LINES) {
    for (int i=FIRST_SEGMENT_INDEX; i<segments; i++) {
      if (segment[i].isline) {
        removeSegment(i);
        // Test this segment again, as it will be reoccupied
        i--;
      }
    }
  }

  // Expand with more feature extraction below

  // printf("end extract, time spend %g\n",t.getTimePassed());
}

/// \brief Initiates the feature extraction from a RangeData object
/// \param *data_ \n Pointer to a RangeData object.
/// \param *res \n Pointer to a EFReturnStruct.
void AUExtractFeatures::extractFeatures(RangeData *data_,AUEFReturnStruct *res)
{
  // Call to the above method
  extractFeatures(data_);

  // Copy internal data to return struct.
  getFeatures(res);
}

/// \brief Number of segments extracted
/// \return Number of segments extracted
int AUExtractFeatures::getSegmentCount()
{
  return segments-FIRST_SEGMENT_INDEX;
}

/// \brief Number of lines extracted
/// \return Number of lines extracted
int AUExtractFeatures::getLineCount()
{
  int lines=0;

  for (int i=FIRST_SEGMENT_INDEX; i<segments; i++) {
    if (segment[i].isline)
      lines++;
  }

  return lines;
}


/// \brief Number of circles extracted
/// \return Number of circles extracted
int AUExtractFeatures::getCircleCount()
{
  int circles=0;

  for (int i=FIRST_SEGMENT_INDEX; i<segments; i++) {
    if (!segment[i].isline)
      circles++;
  }

  return circles;
}

/// \brief XML format of segment
/// \param seg, *reply \n seg: segment index. \n reply: pointer to string.
void AUExtractFeatures::segmentToStringPolar(int seg, char *reply)
{
  if (segment[seg+2].isline)
    segment[seg+2].line.toStringPolar(reply);
  else
    segment[seg+2].circle.toStringPolar(reply);
}

/// \brief XML format of segment
/// \param seg, *reply \n seg: segment index. \n reply: pointer to string.
void AUExtractFeatures::segmentToStringCart(int seg, char *reply)
{
  if (segment[seg+2].isline)
    segment[seg+2].line.toStringCart(reply);
  else
    segment[seg+2].circle.toStringCart(reply);
}

/////////////////////////////////////////////////////

/** \brief get line segment
    \param idx index to requested line segment, where 0 is the first
    \returns pointer to fitted line segment if segment is a line
    \returns NULL if segment is not a line. */
PolarLineFit * AUExtractFeatures::getSegmentLine(int idx)
{
  if (segment[idx+2].isline)
    return &segment[idx+2].line;
  else
    return NULL;
}

/** \brief get corcle fit structure
    \param idx index to requested line segment, where 0 is the first
    \returns pointer to fitted segment if segment is a circle,
    \returns NULL when the segment is not a circle*/
CircleFit * AUExtractFeatures::getSegmentCircle(int idx)
{
  if (not segment[idx+2].isline)
    return &segment[idx+2].circle;
  else
    return NULL;
}

/////////////////////////////////////////////////////

PolarLineFit * AUExtractFeatures::findLongestLine(bool rightwise)
{
  int best=-1;
  double length=0;

  for (int i=FIRST_SEGMENT_INDEX; i<segments; i++) {
    if (segment[i].isline) {
      if ((segment[i].line.getTh0() > 0) == rightwise) {
        if (segment[i].line.getLength() > length) {
          length = segment[i].line.getLength();
          best = i;
        }
      }
    }
  }

  if (best != -1)
    return &(segment[best].line);
  else
    return NULL;

}


///////////////////////////
///////////////////////////
//////////////////////////


/**
  \brief Set all configurable vars to default */
void AUExtractFeatures::setDefaults()
{
  configClusterDiff = 0.10;

  configSplitMSQ = 0.010*0.010;
  configMergeMSQ = 0.010*0.010;

  configMode = EF_DO_SPLIT | EF_SEE_CIRCLES |  EF_DO_DISCARD | EF_DO_MERGE;
  configMinSegment = 3;

  // configDiscardMSQ = 0.010*0.010;
  configDiscardDots = 10;
  configDiscardSize = 0.12;

  configCircleMSQ  = 0.008*0.008;
  configCircleMaxR = 2.0;
  configCircleMinR = 0.08;
  configCircleMinCg = 1.0;     // min 1 radians coverage
  configCircleMinDots = 13;     // min 1 radians coverage
  configCircleMaxCg = 4;

  configRangeMin = 0.2;     // min range

}



#define EF_PCOUNT 14
static const char *pName[EF_PCOUNT] = {"mode","segmentmin","splitdev",
  "mergedev","clusterdiff","discarddots","discardsize","circledev","circlemaxr","circleminr","circlemaxcg","circlemincg","circlemindots","rangemin"};

const char * AUExtractFeatures::paramName(int p)
{
  return pName[p];
}

int AUExtractFeatures::paramCount()
{
  return EF_PCOUNT;
}

void AUExtractFeatures::setParam(int p, const char * value)
{
  int i;
  double d;

  switch(p) {
  case 0:  // mode

    i=0;
    if (strchr(value,'s'))
      i |= EF_DO_SPLIT;

    if (strchr(value,'m'))
      i |= EF_DO_MERGE;

    if (strchr(value,'c'))
      i |= EF_SEE_CIRCLES;

    if (strchr(value,'d'))
      i |= EF_DO_DISCARD;

    if (strchr(value,'n'))
      i |= EF_NO_LINES;

    configMode = i;

    break;
  case 1:
    configMinSegment = atoi(value);
    break;
  case 2:
    d = atof(value);
    configSplitMSQ = d*d; // the msq is deviation squared
    break;
  case 3:
    d = atof(value);
    configMergeMSQ = d*d;
    break;
  case 4:
    d = atof(value);
    configClusterDiff = d;
    break;
  case 5:
    i = atoi(value);
    configDiscardDots = i;
    break;
  case 6:
    d = atof(value);
    configDiscardSize = d;
    break;
  case 7:
    d = atof(value);
    configCircleMSQ = d*d;
    break;
  case 8:
    d = atof(value);
    configCircleMaxR = d;
    break;
  case 9:
    d = atof(value);
    configCircleMinR = d;
    break;
    case 10:
      d = atof(value);
      configCircleMaxCg = d;
      break;
  case 11:
    d = atof(value);
    configCircleMinCg = d;
    break;
  case 12:
    i = atoi(value);
    configCircleMinDots = i;
    break;
  case 13:
    i = atoi(value);
    configRangeMin = i;
    break;
  }
}

int AUExtractFeatures::getParam(int p, char *target)
{
 switch(p) {
  case 0:
    target[0] = configMode & EF_DO_SPLIT ? 's' : ' ';
    target[1] = configMode & EF_DO_MERGE ? 'm' : ' ';
    target[2] = configMode & EF_SEE_CIRCLES ? 'c' : ' ';
    target[3] = configMode & EF_DO_DISCARD ? 'd' : ' ';
    target[4] = configMode & EF_NO_LINES ? 'n' : ' ';
    target[5] = 0;
    return 5;
  case 1:
    return sprintf(target,"%i",configMinSegment);
  case 2:
    return sprintf(target,"%g",sqrt(configSplitMSQ));
  case 3:
    return sprintf(target,"%g",sqrt(configMergeMSQ));
  case 4:
    return sprintf(target,"%g",configClusterDiff);
  case 5:
    return sprintf(target,"%i",configDiscardDots);
  case 6:
    return sprintf(target,"%g",configDiscardSize);
  case 7:
    return sprintf(target,"%g",sqrt(configCircleMSQ));
  case 8:
    return sprintf(target,"%g",configCircleMaxR);
  case 9:
    return sprintf(target,"%g",configCircleMinR);
 case 10:
     return sprintf(target,"%g",configCircleMaxCg);
  case 11:
    return sprintf(target,"%g",configCircleMinCg);
  case 12:
    return sprintf(target,"%i",configCircleMinDots);
  case 13:
    return sprintf(target,"%g",configRangeMin);
  }

  return 0;
}

