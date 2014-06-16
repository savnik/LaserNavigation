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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <ugen4/u2dline.h>
#include <urob4/uimgpush.h>

#include "ucalibrate.h"

// To show and file more details enable this CALIB_DEBUG define
// and send many more images to remote control and to file

#define CALIB_DEBUG

/**
Logfile */
FILE * logGmk;
/**
Level of information to log */
int logLevelGmk;
/**
Should more (and all logged info) be send to console? */
bool verboseMessagesGmk;

/**
Save this message in logfile if significance is less than logLevel.
Print also to console (debug feature) if verbose messages.
Timestamp is optional, if not used, then Now() is used in logfile.*/
void toLogGmk(const char * info, int significance, UTime * timestamp)
{
  const int MSL = 30;
  char s[MSL];
  UTime tnow;
  UTime * t;
  //
  if (significance <= logLevelGmk)
  {
    if (timestamp == NULL)
    {
      t = &tnow;
      t->Now();
    }
    else
      t = timestamp;
    t->getTimeAsString(s, true);
    if (verboseMessagesGmk)
      printf("%s (%d): %s\n", s, significance, info);
    if (logGmk != NULL)
      fprintf(logGmk, "%s (%d): %s\n", s, significance, info);
  }
}

////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////
// UCalibXYStat
/**
Constructor, just resets values */
UCalibXYStat::UCalibXYStat()
: x(0.0), y(0.0), vx(0.0), vy(0.0),
  n(0), flag(true), id(-1), d(0.0)
{
}
////////////////////////////////////////////////////////////
/**
Clear just clear values */
void UCalibXYStat::clear()
{
  x = 0.0;
  y = 0.0;
  vx = 0.0;
  vy = 0.0;
  n = -1;
  flag = true;
  id = -1;
  endId = -1;
  d = 1e10;
}

//////////////////////////////////////////////////////////////

int UCalibXYStat::add(float ax, float ay)
{ // Add a new set of values and returns the number of samples.
  x  += ax;
  vx += sqr(ax);
  y  += ay;
  vy += sqr(ay);
  n++;
  return n;
}

//////////////////////////////////////////////////////
  /**
  Convert to statistics - mean, variance and std.deviation.
  returns n if valid else -1 */
int UCalibXYStat::FromSumToStat(void)
{ // do average
  x = x / float(n);
  y = y / float(n);
  // do variace
  vx = vx / float(n) - sqr(x);
  vy = vy / float(n) - sqr(y);
  // and deviation
  if (vx > 0)
    vx = sqrt(vx);
  else
    vx = 0;
  if (vy > 0)
    vy = sqrt(vy);
  else
    vy = 0;
  return n;
}

//////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// UCalibrationComponent

void UCalibrationComponent::clear()
{ // cust clear object
  x = 0.0;
  y = 0.0;
  w = 0;
  row = -1000;
  col = -1000;
  rx = 0.0;
  ry = 0.0;
  filter = -1;
  clearNSEW();
}

////////////////////////////////////////////////////////////
/**
Clear relations to neighbors */
void UCalibrationComponent::clearNSEW()
{
  int i;
  //
  for (i = 0; i < 8; i++)
    NSEW[i].clear();
}

//////////////////////////////////////////////////////

float UCalibrationComponent::GetMaxBlockDist(UCalibrationComponent * other,
                                      float * dx, float * dy)
{ // find max distance in either x or y direction
  *dx = other->x - x;
  *dy = other->y - y;
  if (absf(*dx) > absf(*dy))
    return absf(*dx);
  else
    return absf(*dy);
}

////////////////////////////////////////////////////////

void UCalibrationComponent::GetDefaultOffset(
                              int head, /* heading (0..7) */
                              int MinSep, /* minimum */
                              float * dx, float * dy)
{ // moves a bit in the head-direction, in order to give the
  // best chance of minimum distance to the correct neighbor
  // or use an average of known neighbors
  //int  n, i;
  //float e, d;
  //
  *dx = 0;
  *dy = 0;
  switch (head)
  {
  case 0: /*N*/  *dy = -MinSep; break;
  case 1: /*NE*/ *dy = -MinSep; *dx =  MinSep; break;
  case 2: /*E*/  *dx =  MinSep; break;
  case 3: /*SE*/ *dy =  MinSep; *dx =  MinSep; break;
  case 4: /*S*/  *dy =  MinSep; break;
  case 5: /*SW*/ *dy =  MinSep; *dx = -MinSep; break;
  case 6: /*W*/  *dx = -MinSep; break;
  case 7: /*NW*/ *dy = -MinSep; *dx = -MinSep; break;
  }
  // if other neighbors are found, then use the
  // closest of these as distance measure instead
  /*
  d = 100.0; //distance
  e = 100.0; // minimum distance
  n = 0;
  for (i = 0; i<8; i++)
  {  // in all directions
     if (NSEW[i].id >= 0)
     { // a neighbor is valid
       d = NSEW[i].d;
       if (d < e)
         e = d;
       n++;
     }
  }
  if (n > 1)
  { // at least 2 neighbors are found
    //the closer one should be OK
    *dx = e * *dx / MinSep;
    *dy = e * *dy / MinSep;
  }
  */
}

//////////////////////////////////////////////////////////////
  /**
  Get distance to another component. */
float UCalibrationComponent::GetDist(UCalibrationComponent * other)
{
  return hypot(x - other->x, y - other->y);
}

/////////////////////////////////////////////////////////////////

bool UCalibrationComponent::IsInRightQuadrant(
                      UCalibrationComponent * other,
                      int head)
{ // is the other candidate in the expected (head) heading
  float rx, ry;
  bool result = false;
  //
  rx = other->x - x;
  ry = other->y - y;
  //
  switch (head)
  {
  case  0: /*N*/  result =(ry < 0) and (absf(ry) > absf(rx)); break;
  case  1: /*NE*/ result =(ry < 0) and (rx > 0); break;
  case  2: /*E*/  result =(rx > 0) and (absf(rx) > absf(ry)); break;
  case  3: /*SE*/ result =(ry > 0) and (rx > 0); break;
  case  4: /*S*/  result =(ry > 0) and (absf(ry) > absf(rx)); break;
  case  5: /*SW*/ result =(ry > 0) and (rx < 0); break;
  case  6: /*W*/  result =(rx < 0) and (absf(rx) > absf(ry)); break;
  case  7: /*NW*/ result =(ry < 0) and (rx < 0); break;
  }
  return result;
}

/////////////////////////////////////////////////////

void UCalibrationComponent::SetRc(int r, int c, int head)
{ // set row and column number from the
  // neighbor opposit head direction
  switch(head)
  {
  case 0 /*N*/ : row = r-1 ; col = c  ; break;
  case 1 /*NE*/: row = r-1 ; col = c+1; break;
  case 2 /*E*/ : row = r   ; col = c+1; break;
  case 3 /*SE*/: row = r+1 ; col = c+1; break;
  case 4 /*S*/ : row = r+1 ; col = c  ; break;
  case 5 /*SW*/: row = r+1 ; col = c-1; break;
  case 6 /*W*/ : row = r   ; col = c-1; break;
  case 7 /*NW*/: row = r-1 ; col = c-1; break;
  }
} // SetRc

/////////////////////////////////////////////////////////////

void UCalibrationComponent::PaintNeighbors(UImage * image,
                  UPixel pix, bool halfWay,
                  UCalibrationComponent * ccHits,
                  float scale /* = 1.0 */)
{ // paint lines to neighbors (all the way or just half)
  UCalibrationComponent * to;
  int i, n;
  int ix, iy;
  int dx, dy;
  UPixel cPix;
  //
  ix = roundi(x * scale);
  iy = roundi(y * scale);
  for (i = 0; i < 8; i++)
  { // set color palette
    n = NSEW[i].id;
    // debug
    if (n >= MAX_CALIBRATION_COMPONENTS)
      n = 0;
    // debug end
    if ((n >= 0) and (n < MAX_CALIBRATION_COMPONENTS))
    {
      to = &ccHits[n];
      dx = roundi((to->x - x) * scale);
      dy = roundi((to->y - y) * scale);
      if (halfWay)
      { // paint just half the way?
        dx = dx / 2;
        dy = dy / 2;
      }
      cPix.setYUVPix8(i, 255);
      cvLine( image->cvArr(), cvPoint(ix,iy), cvPoint(ix+dx,iy+dy),
          cPix.cvRGB(), 1, 8);
      //image->PaintLine(ix, iy, ix+dx, iy + dy, &cPix);
    }
  }
  // and center dot in desired color
  cvCircle(image->cvArr(), cvPoint(ix, iy), 1, pix.cvRGB(), -1);
  //cvCircle( CvArr* img, CvPoint center, int radius, CvScalar color,
  //             int thickness=1, int line_type=8, int shift=0 );
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
// UCalibBestArray

void UCalibBestArray::AddCandidate(int candidate, float d /* distance */,
                                   float dx, float dy)
{ // adds a candidate in distance (d) order, closest gets lowest number
  int j;
  // add an element
  if (count < MAX_BEST_ARRAY_SIZE)
    items[count++].valid = true;
  // find best placement
  for (j = count-1; j >= 0; j--)
     if (d < items[j].d)
     {  // move one up (if space)     this
        if (j < count-1)
          items[j+1] = items[j];
        // if at bottom
        if (j == 0)
        { // add as best
          items[j].d = d;
          items[j].id = candidate;
          items[j].dx = dx;
          items[j].dy = dy;
        }
     }
     else
     { // not better any more - add here (if space)
        if (j < count-1)
        { // space OK
          items[j+1].d = d;
          items[j+1].id = candidate;
          items[j+1].dx = dx;
          items[j+1].dy = dy;
        }
        // now exit loop
        break;
     }
}

///////////////////////////////////////////////////////

void UCalibBestArray::DoLimitCandidates(float MaxNeighborDistanceFactor)
{ // limit candidates using minimum distance in x and y
  // Cut the list if too far
  int i;
  float md = 1000.0;
  //
  for (i = 0; i < count; i++)
  { // find minimum length and calc real distance
    items[i].d = sqrt(sqr(items[i].dx) + sqr(items[i].dy));
    if (items[i].d < md)
      md = items[i].d;
  }
  // do not allow more than k * longer distance
  // than minimum
  md *= MaxNeighborDistanceFactor; // 1.8 is too small if not at right angle
  for (i = 0; i < count; i++)
  {
    if (items[i].d > md)
    {
      count = i;
      break;
    }
  }
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// UCalibrationComponents

UCalibrationComponents::UCalibrationComponents()
: hitCount(0)
{
  clear();
}
////////////////////////////////////////////////////////

void UCalibrationComponents::clear()
{
  int i;
  //
  for (i = 0; i < MAX_CALIBRATION_COMPONENTS; i++)
    hits[i].clear();
  hitCount = 0;
}

////////////////////////////////////////////////////////

int UCalibrationComponents::Pack()
{
  int n;
  int nEmpty = -1;
  //
  if (hitCount > 0)
  { // find first empty
    for (n = 0; n < hitCount; n++)
      if (hits[n].w == 0)
      { // this one is the first empty hits[4]
        nEmpty = n;
        break;
      }
    if (nEmpty >= 0)
    { // now move the rest of the non empty slots
      for (n = nEmpty + 1; n < hitCount; n++)
        if (hits[n].w != 0)
          hits[nEmpty++] = hits[n];
      hitCount = nEmpty;
    }
  }
  return hitCount;
}

////////////////////////////////////////////////////////////

bool UCalibrationComponents::EnumerateCalibrationDetections(
                                UImage * cim, int connectivity)
{ // Do a component labeling using
  // 4, 8, 12 or 20 neighborhood connectivity
  const int MAX_PAIRS = 256; // NB! remember size of enn[][] elements
  typedef struct record
        { // structure for ???
           int n;   // id number
           int x,y; // pixel position
           float sx, sy; // pixel position sum for centroid calculation
           int sn;  // number of summs in sx,sy
        } UCPos;
  bool result;
  int i,j,m;
  UPixel * Pix;
  int n, h, l, y, w;
  bool f;
  // array with pair numbers index number may point
  // at itself, then no alias
  int pair[MAX_PAIRS];
  int pairs = 1; // number of pairs used (0 not used)
  // found corner positions
  UCPos pos[MAX_CALIBRATION_COMPONENTS];
  int count; // count of found positions
  // id's within one connectivity cell
  int id[11];
  // enumeration numbers one possible ID per pixel in image
  static unsigned char enn[MAX_IMAGE_HEIGHT][MAX_IMAGE_WIDTH];
  //
  result = (cim != NULL);
  if (result)
  { // remove any old components
    hitCount = 0;
    // initiate numbers in row 0 to zero
    for (i = 0; i < int(cim->width()); i++)
      enn[0][i] = 0;
    //
    n = sizeof(enn);
    bzero(enn, n);
    /*
    for (j = 0; j < int(cim->height); j++)
    { // and left and right flank to zero as well
      enn[j][0] = 0;
      enn[j][1] = 0;
      enn[j][cim->width - 1] = 0;
      enn[j][cim->width - 2] = 0;
    }
    */
    // first run with center top and left pixel test
    // for all rows minus top row
    for (j = 1; j < int(cim->height()); j++)
    { // and for all pixels minus the outhermost 2 pixels
      for (i = 2; i < int(cim->width())-2; i++)
      { // get pixel value
        Pix = cim->getPixRef(j,i);
        if (Pix->y == 0)
          enn[j][i] = 0; // empty
        else
        { // not empty center cell
          // find ID numbers in close cells to h[] array
          h = 0;
          id[h] = enn[j][i-1]; // cell to the left
          if (id[h] > 0) h++;
          id[h] = enn[j-1][i]; // cell above
          if (id[h] > 0) h++;
          if (connectivity > 4)
          { // then 2 more more cells needs testing
            id[h] = enn[j-1][i-1]; // cell (-1,-1) above 1, left 1
            if (id[h] > 0) h++;
            id[h] = enn[j-1][i+1]; // cell (-1,+1) above 1, right 1
            if (id[h] > 0) h++;
            if ((connectivity > 8) and (j > 1))
            { // then further 2 cells needs test
              id[h] = enn[j][i-2]; // cell left 2
              if (id[h] > 0) h++;
              id[h] = enn[j-2][i]; // cell two above
              if (id[h] > 0) h++;
              if (connectivity > 12)
              { // then four more cells are added
                id[h] = enn[j-1][i-2]; // up 1, left 2
                if (id[h] > 0) h++;
                id[h] = enn[j-2][i-1]; // up 2, left 1
                if (id[h] > 0) h++;
                id[h] = enn[j-2][i+1]; // up 2, right 1
                if (id[h] > 0) h++;
                id[h] = enn[j-1][i+2]; // up 1, right 2
                if (id[h] > 0) h++;
              }
            }
          }
          n = 1024;
          if (h > 0)
          { // find minimum number from the candidates in id[] array
            for (l = 0; l < h; l++)
              if (id[l] < n)
                n = id[l];
            // connect pairs if more than one number in h[] array
            for (l = 0; l < h; l++)
              if (id[l] != n)
                pair[id[l]] = n;
            // this cell gets the lowest number
            enn[j][i] = n;
          }
          else
          { // create new component number
            pair[pairs] = pairs;
            enn[j][i] = pairs;
            if (pairs < MAX_PAIRS - 1)
              pairs++;
          }
        }
      }
    }
    // second pass - merge matches (pair 0 not used)
    if (pairs > 1)
    {
      count = 0;
      for (j = 1; j < int(cim->height()); j++)
      { // for each row
        for (i = 1; i < int(cim->width())-1; i++)
        { // for each column
          if (enn[j][i] > 0)
          { // position is not empty
            h = pair[enn[j][i]];
            f = false;
            n = 0;
            // find component number for this pixel
            // if already existing
            for (m = 0; m < count; m++)
              if (h == pos[m].n)
              {
                f = true;
                n = m;
              }
            // get intensity (weight of corner)
            Pix = cim->getPixRef(j,i);
            y = Pix->y;
            w = y + 64; // test offset
            if (not f)
            { // no component yet, so make one
              pos[count].n  = h;
              pos[count].x  = i;
              pos[count].y  = j;
              pos[count].sx = i * w;
              pos[count].sy = j * w;
              pos[count].sn = w;
              n = count;
              if (count < MAX_CALIBRATION_COMPONENTS)
                count++;
            }
            else
            { // exist already, so sum for centroid calculation
              pos[n].sx += i * w;
              pos[n].sy += j * w;
              pos[n].sn += w;
            };
          };
        };
      };

      for (i = 0; i < count; i++)
      { // center of gravity calculation
        hits[i].x = pos[i].sx/float(pos[i].sn);
        hits[i].y = pos[i].sy/float(pos[i].sn);
        hits[i].w = pos[i].sn;
      }   // calib.hitCount
      hitCount = count;
    }
  }
  return result;
}

//////////////////////////////////////////////////////////

bool UCalibrationComponents::EnumerateCalibrationDetections4(
                                UImage * cim, int connectivity)
{ // Do a component labeling using
  // 4, 8, 12 or 20 neighborhood connectivity
  const int MAX_PAIRS = 1000; // NB! remember size of enn[][] elements
  const float MIN_CORNER_SEPERATION = 2.95; // different filter [pixels]
  const float MIN_CORNER_SEPERATION_2 = 1.95; // same filter bank [pixels]
  typedef struct record
        { // structure for ???
           int n;   // id number
           int x,y; // pixel position
           float sx, sy; // pixel position sum for centroid calculation
           int sn;  // number of summs in sx,sy
           int typ; // wich filter
        } UCPos;
  bool result;
  int i,j,m;
  UPixel * Pix;
  int n, h, l, y, w, q, qn;
  bool f;
  float dist;
  // array with pair numbers index number may point
  // at itself, then no alias
  int pair[MAX_PAIRS];
  int pairs; // number of pairs used (0 not used)
  // found corner positions
  UCPos pos[MAX_CALIBRATION_COMPONENTS];
  int count, count0; // count of found positions
  // id's within one connectivity cell
  int id[11];
  int * idh;
  // enumeration numbers one possible ID per pixel in image
  static unsigned char enn[MAX_IMAGE_HEIGHT][MAX_IMAGE_WIDTH];
  //static unsigned int enn[MAX_IMAGE_HEIGHT][MAX_IMAGE_WIDTH];
  unsigned char *enn00, *enn01, *enn10;
  unsigned char *enn11, *enn1p1;
  unsigned char *enn02, *enn20, *enn21, *enn12;
  unsigned char *enn2p1, *enn1p2;
  //
  // UL with top and left
  // UR with top and left
  // top with UL and UR
  // left with UL and UR
  result = (cim != NULL);
  if (result)
  { // remove any old components
    hitCount = 0;
    // initiate numbers in row 0 to zero
    //for (i = 0; i < int(cim->width); i++)
    //  enn[0][i] = 0;
    //
    //n = sizeof(enn);
    //bzero(enn, n);
    /*
    for (j = 0; j < int(cim->height); j++)
    { // and left and right flank to zero as well
      enn[j][0] = 0;
      enn[j][1] = 0;
      enn[j][cim->width - 1] = 0;
      enn[j][cim->width - 2] = 0;
    }
    */
    count = 0;
    count0 = 0;
    for (qn = 1; qn <= 4; qn++)
    { // zero the enumeration array
      n = sizeof(enn);
      bzero(enn, n);
      bzero(id, sizeof(id));
      pairs = 1; // pair 0 is not used
      // first run with center top and left pixel test
      // for all rows minus top row
      for (j = 1; j < int(cim->height()); j++)
      { // and for all pixels minus the outhermost 2 pixels
        Pix = cim->getPixRef(j,2);
        enn00 = &enn[j][2];
        enn01 = &enn[j][2-1];
        enn10 = &enn[j-1][2];
        enn11 = &enn[j-1][2-1];
        enn1p1 = &enn[j-1][2+1];
        enn02 = &enn[j][2-2];
        enn20 = &enn[j-2][2];
        enn12 = &enn[j-1][2-2];
        enn21 = &enn[j-2][2-1];
        enn2p1 = &enn[j-2][2+1];
        enn1p2 = &enn[j-1][2+2];
        for (i = 2; i < int(cim->width())-2; i++)
        { // get pixel value
          //Pix = cim->getPixRef(j,i);
          q = Pix->y;
          if (q > 0)
          {
            // data is availbale
            if (Pix->u > 128)
            {
              if (Pix->v > 128)
                q = 1;
              else
                q = 4;
            }
            else
            {
              if (Pix->v > 128)
                q = 2;
              else
                q = 3;
            }
          }
          // use one quadrant at a time
          if ((q == 0) or (q != qn))
          {
            // no usable data
            // enn[j][i] = 0; // empty
            *enn00 = 0;
          }
          else
          { // not empty center cell
            // find ID numbers in close cells to h[] array
            if (true)
            { // pointer method
              idh = id;
              if (*enn01 > 0) *idh++ = *enn01;
              if (*enn10 > 0) *idh++ = *enn10;
              if (connectivity > 4)
              { // then 2 more more cells needs testing
                if (*enn11 > 0) *idh++ = *enn11;
                if (*enn1p1 > 0) *idh++ = *enn1p1;
                if ((connectivity > 8) and (j > 1))
                { // then further 2 cells needs test
                  if (*enn02 > 0) *idh++ = *enn02;
                  if (*enn20 > 0) *idh++ = *enn20;
                  if (connectivity > 12)
                  { // then four more cells are added
                    if (*enn12 > 0) *idh++ = *enn12;
                    if (*enn21 > 0) *idh++ = *enn21;
                    if (*enn2p1 > 0) *idh++ = *enn2p1;
                    if (*enn1p2 > 0) *idh++ = *enn1p2;
                  }
                }
              }
              n = 1024;
              h = idh - id;
              if (h > 0)
              { // find minimum number from the candidates in id[] array
                idh = id;
                for (l = 0; l < h; l++)
                {
                  if (*idh < n)
                    n = *idh;
                  idh++;
                }
                // connect pairs if more than one number in h[] array
                idh = id;
                for (l = 0; l < h; l++)
                {
                  if (*idh != n)
                    pair[*idh] = n;
                  idh++;
                }
                // this cell gets the lowest number
                //enn[j][i] = n;
                *enn00 = n;
              }
              else
              { // create new component number
                pair[pairs] = pairs;
                //enn[j][i] = pairs;
                *enn00 = pairs;
                if (pairs < MAX_PAIRS - 1)
                  pairs++;
                else
                  // error
                  printf("**** UCalibrationComponents::EnumerateCalibrationDetections4"
                      " corner pair (%d) overflow!\n", pairs);
              }
            }
            else
            { // index method
              h = 0;
              idh = id;
              id[h] = *enn01; //enn[j][i-1]; // cell to the left
              if (id[h] > 0) h++;
              id[h] = *enn10; //enn[j-1][i]; // cell above
              if (id[h] > 0) h++;
              if (connectivity > 4)
              { // then 2 more more cells needs testing
                id[h] = *enn11; // enn[j-1][i-1]; // cell (-1,-1) above 1, left 1
                if (id[h] > 0) h++;
                id[h] = *enn1p1; //enn[j-1][i+1]; // cell (-1,+1) above 1, right 1
                if (id[h] > 0) h++;
                if ((connectivity > 8) and (j > 1))
                { // then further 2 cells needs test
                  id[h] = enn[j][i-2]; // cell left 2
                  if (id[h] > 0) h++;
                  id[h] = enn[j-2][i]; // cell two above
                  if (id[h] > 0) h++;
                  if (connectivity > 12)
                  { // then four more cells are added
                    id[h] = enn[j-1][i-2]; // up 1, left 2
                    if (id[h] > 0) h++;
                    id[h] = enn[j-2][i-1]; // up 2, left 1
                    if (id[h] > 0) h++;
                    id[h] = enn[j-2][i+1]; // up 2, right 1
                    if (id[h] > 0) h++;
                    id[h] = enn[j-1][i+2]; // up 1, right 2
                    if (id[h] > 0) h++;
                  }
                }
              }
              n = 1024;
              if (h > 0)
              { // find minimum number from the candidates in id[] array
                for (l = 0; l < h; l++)
                  if (id[l] < n)
                    n = id[l];
                // connect pairs if more than one number in h[] array
                for (l = 0; l < h; l++)
                  if (id[l] != n)
                    pair[id[l]] = n;
                // this cell gets the lowest number
                //enn[j][i] = n;
                *enn00 = n;
              }
              else
              { // create new component number
                pair[pairs] = pairs;
                //enn[j][i] = pairs;
                *enn00 = pairs;
                if (pairs < MAX_PAIRS - 1)
                  pairs++;
                else
                  // error
                  printf("**** UCalibrationComponents::EnumerateCalibrationDetections4"
                      " corner pair (%d) overflow!\n", pairs);
              }
            }
          }
          enn00++;
          enn01++;
          enn10++;
          if (connectivity > 4)
          {
            enn11++;
            enn1p1++;
            if (connectivity > 8)
            {
              enn02++;
              enn20++;
              if (connectivity > 4)
              {
                enn12++;
                enn21++;
                enn2p1++;
                enn1p2++;
              }
            }
          }
          Pix++;
        } // for i
      //
      } // for j
      // second pass - merge matches (pair 0 not used)
      if (pairs > 1)
      {
        for (j = 1; j < int(cim->height()); j++)
        { // for each row
          for (i = 1; i < int(cim->width())-1; i++)
          { // for each column
            if (enn[j][i] > 0)
            { // position is not empty
              h = pair[enn[j][i]];
              f = false;
              n = 0;
              // find component number for this pixel
              // if already existing
              for (m = count0; m < count; m++)
                if (h == pos[m].n)
                {
                  f = true;
                  n = m;
                }
              // get intensity (weight of corner)
              Pix = cim->getPixRef(j,i);
              y = Pix->y;
              w = y + 64; // test offset
              if (not f)
              { // no component yet, so make one
                pos[count].n  = h;
                pos[count].x  = i;
                pos[count].y  = j;
                pos[count].sx = i * w;
                pos[count].sy = j * w;
                pos[count].sn = w;
                pos[count].typ = qn;
                n = count;
                if (count < MAX_CALIBRATION_COMPONENTS)
                  count++;
                else
                // error
                  printf("**** UCalibrationComponents::EnumerateCalibrationDetections4"
                      " component (%d) overflow!\n", count);
              }
              else
              { // exist already, so sum for centroid calculation
                pos[n].sx += i * w;
                pos[n].sy += j * w;
                pos[n].sn += w;
              };
            }
          }
        }
      }
      // this type of corner is finished
      // freese these and goto next set
      count0 = count;
    }
    // NB! ikke testet
    j = 0;
    for (i = 0; i < count; i++)
    { // center of gravity calculation
      hits[j].x = pos[i].sx/float(pos[i].sn);
      hits[j].y = pos[i].sy/float(pos[i].sn);
      hits[j].w = pos[i].sn;
      hits[j].filter = pos[i].typ;
      // test for other corners at this position
      f = true;
      for (m = 0; m < j; m++)
      {
        dist = hypot(hits[m].x - hits[j].x, hits[m].y - hits[j].y);
        if (dist < MIN_CORNER_SEPERATION)
          if ((((hits[j].filter - hits[m].filter) % 2) == 0) or
             (dist < MIN_CORNER_SEPERATION_2))
          {
            f = false;
            break;
          }
      }
      if (f)
        // use new hit
        j++;
      else
      { // merge with old
        hits[m].x = (hits[j].x * hits[j].w + hits[m].x * hits[m].w)/(hits[j].w + hits[m].w);
        hits[m].y = (hits[j].y * hits[j].w + hits[m].y * hits[m].w)/(hits[j].w + hits[m].w);
        hits[m].w =  hits[j].w + hits[m].w;
      }
    }   // calib.hitCount
    hitCount = count;
  }
  return result;
}

/////////////////////////////////////////////////////////////

int UCalibrationComponents::PaintHitsInImage(UImage * image, UPixel pix)
{ // paint valid hits in this base color
  int x;
  int y;
  int i;
  //
  for (i = 0; i < hitCount; i++)
    if (hits[i].w > 0)
    {
      pix.y = mini(255, hits[i].w);
      x = int(hits[i].x);
      y = int(hits[i].y);
      cvCircle(image->cvArr(), cvPoint(x, y), 2, pix.cvRGB(), -1);
    }
  return 0;
}

////////////////////////////////////////////////

int UCalibrationComponents::PaintChartInImage(
                UImage * image, UPixel pix,
                UPosition * chartPos,
                UCamPar * cm,
                const char * matlabFilename)
{ // paint valid hits in this base color
  int x;
  int y;
  int i;
  URotation dRot(0.0, 0.0, 0.0);
  UPosition dPos(0.0, 0.0, 0.0);
  UMatrix4 mItoP;
  UMatrix4 mWtoP;
  UMatrix4 realPos(1, 4, 1.0);
  UMatrix4 pixPos(3);
  FILE * f = NULL;
  UMatrix4 T(4,4, 0.0);
  UMatrix4 R(4,4, 0.0);
  UMatrix4 RT(4,4, 0.0);
  //UCamera *cm = (UCamera *)image->cam;
  //
  mItoP = *cm->getItoP();
  mItoP.expand(3,4,0.0);
  /*
  dRot = cm->pDevice->relRot;
  dPos = cm->pDevice->relPos;
  T = dPos.asMatrix4x4();
  R = dRot.asMatrix4x4WtoC();
  */
  //mWtoP = mItoP * dRot.asMatrix4x4WtoC(&dPos);
  RT = R * T;
  mWtoP = mItoP; // * RT;
  //f = fopen("/home/Chr/chr/results/pixmix.m", "w");
  if (matlabFilename != NULL)
    f = fopen(matlabFilename, "w");
  //
  for (i = 0; i < hitCount; i++)
    if (hits[i].w > 0)
    {
      //pix.y = min(255, hits[i].w);
      realPos.set(hits[i].rx + chartPos->x,
                  hits[i].ry + chartPos->y,
                  chartPos->z, 1.0);
      pixPos = mWtoP * realPos;
      pixPos.normalize();
      x = roundi(pixPos.get(0));
      y = roundi(pixPos.get(1));
      cvCircle(image->cvArr(), cvPoint(x, y), 3, pix.cvRGB(), 1);
      if (f != NULL)
        fprintf(f,
         "%3d px %3.0f,%3.0f rl %5.2f,%5.2f rc %5.2f,%5.2f,%5.2f cnv %4.0f,%4.0f\n",
              i, hits[i].x, hits[i].y, hits[i].rx, hits[i].ry,
              realPos.get(0), realPos.get(1), realPos.get(2),
              pixPos.get(0), pixPos.get(1));
    }
  if (f != NULL)
    fclose(f);
  return 0;
}

//////////////////////////////////////////////////////////////////

int UCalibrationComponents::PaintGridInImage(UImage * image, UPixel pix)
{
  int x2, y2;
  int r, c;
  UCalibrationComponent * subj1 = NULL;
  // paint horizontal lines
  for (r = 0; r < MAX_CALIB_GRID_SIZE; r++)
  { // set to first point
    subj1 = NULL;
    for (c = 0; c < MAX_CALIB_GRID_SIZE; c++)
    {
      if (grid[r][c] != NULL)
        if (grid[r][c]->w > 0)
        { // paint blob
          x2 = roundi(grid[r][c]->x);
          y2 = roundi(grid[r][c]->y);
          cvCircle(image->cvArr(), cvPoint(x2, y2), 3, pix.cvRGB(), 3);
          if (subj1 != NULL)
          { // paint line
            cvLine(image->cvArr(),
                cvPoint(roundi(subj1->x), roundi(subj1->y)),
                cvPoint(x2, y2), pix.cvRGB());
          }
          else
            subj1 = grid[r][c];
        }
    }
  }
  // paint vertical lines in green
  pix.setRGBto(0,255,0, PIX_PLANES_RGB);
  for (c = 0; c < MAX_CALIB_GRID_SIZE; c++)
  { // set to first point
    subj1 = NULL;
    for (r = 0; r < MAX_CALIB_GRID_SIZE; r++)
    {
      if (grid[r][c] != NULL)
        if (grid[r][c]->w > 0)
        { // paint blob
          x2 = roundi(grid[r][c]->x);
          y2 = roundi(grid[r][c]->y);
          cvCircle(image->cvArr(), cvPoint(x2, y2), 3, pix.cvRGB());
          if (subj1 != NULL)
          { // paint line
            cvLine(image->cvArr(),
                    cvPoint(roundi(subj1->x), roundi(subj1->y)),
                    cvPoint(x2, y2), pix.cvRGB());
          }
          else
            subj1 = grid[r][c];
        }
    }
  }
  return 0;
}

////////////////////////////////////////////////

int UCalibrationComponents::doMergeNearHits(float dist)
{ // merges hist, if distance in x or y is below 'Dist'
  // returns number of valid hits
  int i,j;
  float x,y;
  int w;
  int result = 0;
  //
  for (i = 0; i < hitCount; i++)
    if (hits[i].w > 0)
      for (j = i+1; j < hitCount; j++)
      {
        if (hits[j].w > 0)
          if (absf(hits[i].x - hits[j].x) < dist)
            if (absf(hits[i].y - hits[j].y) < dist)
            { // merge
              w = hits[i].w + hits[j].w;
              // calculate weighted sum
              x = hits[i].x * hits[i].w + hits[j].x * hits[j].w;
              x = x / float(w);
              y = hits[i].y * hits[i].w + hits[j].y * hits[j].w;
              y = y / float(w);
              // save merged component
              hits[i].x = x;
              hits[i].y = y;
              hits[i].w = w;
              // mark other as empty
              hits[j].w = 0;
            }
      }
  // count valid hits
  for (i = 0; i < hitCount; i++)
    if (hits[i].w > 0)
      result ++;
  return result;
}

////////////////////////////////////////////////

int UCalibrationComponents::DoSizeLimit()
{
  int i,j;
  float w = 0,v = 0;
  //
  // calculate statistics on size
  // to get average and distribution
  j = 0;
  for (i = 0; i < hitCount; i++)
    if (hits[i].w > 0)
    { // sum only valid components
      j++;
      w += hits[i].w;
      v += sqr(hits[i].w);
    }
  w = w/float(j);
  v = v/float(j);
  v = sqrt(absf(v-sqr(w)));
  //
  j = 0;
  for (i = 0; i < hitCount; i++)
  { // if within 2 * sd of averge and a bit, then OK
    if (hits[i].w > 0)
    {
      if ((hits[i].w < (w*1.25+v*2.0))
          and ((hits[i].w > (w*0.75-v*2.0))))
        // valid size
        j++;
      else
        // invalid size - just remove
        hits[i].w = 0;
    }
  }
  return j;
}

//////////////////////////////////////////////////////

// int UCalibrationComponents::DoOrderHits(UImage * /*image*/, // image)
//                                         float stride) // side size of square
// {
//   int i, j, b, c, p;
//   UCalibrationComponent * subj;
//   UPixel pix;
//   float strideL, strideW; // celle distance on calibration chart
//   //
//   // remove any empty cells
//   Pack();
//   // clear relations
//   for (i = 0; i < hitCount; i++)
//     hits[i].clearNSEW();
//   // find new relations for all calibration candidates
//   for (i = 0; i < hitCount; i++)
//   {
//     DoFindNSEW(i);
//     /* debug
//     pix.SetPixRGB(0,255,255);
//     hits[i].PaintNeighbors(image, pix);
//     image->imgTime.Now();
//     sysState.SetImageCopy(1, image, false);
//     // re paint in black
//     pix.SetPixRGB(0,0,0);
//     hits[i].PaintNeighbors(image, pix);
//     // debug end */
//   }
//   // calculate new distances (squared)
//   for (i = 0; i < hitCount; i++)
//     for (j = 0; j < 8; j++)
//       if (hits[i].NSEW[j].id >= 0)
//         hits[i].NSEW[j].d = sqr(hits[i].NSEW[j].x)+
//                             sqr(hits[i].NSEW[j].y);
//   //
//   // find one start item with 8 neighbors
//   b = 0;  // best score
//   c = -1; // candidate with best score
//   for (j = 0; j < hitCount; j++)
//   { //
//     p = 0;
//     for (i = 0; i < 8; i++)
//       if (hits[j].NSEW[i].id >= 0)
//         p++;
//     if (p == 8)
//     { // a fully neighbor'ed candidate is found
//       c = j;
//       break;
//     }
//     else if (p > b)
//     { // candidate is better
//       b = p; // score
//       c = j; // candidate with best score
//     }
//   }
//   // remove all row/col numbers
//   // and make all directions valid
//   for (j = 0; j < hitCount; j++)
//   {
//     subj = &hits[j];
//     subj->row = -1024;
//     subj->col = -1024;
//     for (i = 0; i < 8; i++)
//       subj->NSEW[i].flag = true;
//   }
//   // now number all hits starting with c and number 50, 50
//   hits[c].row = 50;
//   hits[c].col = 50;
//   SetRowColForNeighbors(c);
//   // find center cross and assign positions in grd using chart stride
//   /*
//   strideW = conf.doubleGet("calibration", "chartStrideWidth", 0.02);
//   strideL = conf.doubleGet("calibration", "chartStrideLength", 0.02);
//   */
//   //
//   // debug
//   b = 100000;
//   c = -10000;
//   p = -10000;
//   i = 100000;
//   for (j = 0; j < hitCount; j++)
//   {
//     subj = &hits[j];
//     if (subj->row > c)
//       c = subj->row;
//     if (subj->row < b)
//       b = subj->row;
//     if (subj->col > p)
//       p = subj->col;
//     if (subj->col < i)
//       i = subj->col;
//   }
//   // debug end
//   //
//   strideW = stride;
//   strideL = stride;
//   j = DoFindCenterCross(strideL, strideW);
//   // save again - just to ensure that they exist in config file
//   /*
//   conf.doublePut("calibration", "chartStrideWidth", strideW);
//   conf.doublePut("calibration", "chartStrideLength", strideL);
//   */
//   // j is -1 if no cross is found, else number of points in grid
//   return j;
// }

///////////////////////////////////////////////////////

bool UCalibrationComponents::findClosestNeighbors(
               float maxDistance,
               bool extraImages,
               UImage * source,
               UImage * destination)
{
  bool result = false;
  bool found;
  int i;
  UPixel pix;
  // int j;
  // UCalibrationComponent * c;
  // FILE * fin;
  // remove invalid corners
  Pack();
  // clear relations
  for (i = 0; i < hitCount; i++)
    hits[i].clearNSEW();
  //
  /* - this is done already
  if (extraImages and (source != NULL) and (destination != NULL))
  { // debug paint
     destination->copyToMaxRes(source);
    pix.SetPix(255, 128, 128); // white
    destination->Tone(&pix,50);
    destination->imgTime.Now();
    snprintf(destination->name, MaxImageNameLength, "Neighbors%d", source->imageNumber);
  }
  */
  //
  // find new relations for all calibration candidates
  for (i = 0; i < hitCount; i++)
  { // find closest neighbors nearer than this limit
    found = DoFindNSEW4(i, maxDistance, NULL);
    if (found)
      result = true;
    //
    if (extraImages and (source != NULL) and (destination != NULL))
    { // paint in black
      pix.setRGBto(0,0,0, PIX_PLANES_RGB);
      hits[i].PaintNeighbors(destination, pix, true, hits,
                     float(destination->height()) / float(source->height()));
    }
    //
  }
  // debug
  /*
  fin = fopen("neighbors.txt", "w");
  if (fin != NULL)
  { // print neighbor info to file
    fprintf(fin,"Neighbors at end of UCalibrationComponents::findClosestNeighbors\n");
    fprintf(fin,"In total %d hits\n", hitCount);
    fprintf(fin,"ID, 0..7 of neighbors with ref to other point and distance in pixels\n");
    for (i = 0; i < hitCount; i++)
    {
      c = &hits[i];
      fprintf(fin, "%2d: (%3.0f,%3.0f) ns: ", i, c->x, c->y);
      for (j = 0; j < 8; j++)
      { // print
        if (c->NSEW[j].id >= 0)
          fprintf(fin,"%d->%2d (%4.1f),", j, c->NSEW[j].id,
                         c->NSEW[j].d);
        else
          fprintf(fin,"%d->%2d (    ),", j, c->NSEW[j].id);
      }
      fprintf(fin,"\n");
    }
    fclose(fin);
  }
  */
  // debug end
  return result;
}

///////////////////////////////////////////////////////

bool UCalibrationComponents::doOrderHits4(int size, // barcode size to look for
                   int * k0, int * k1, int * k2, // 3 corners
                   int * k0dk1, int * k0dk2, int * k1dB, int * k2dB,
                   bool extraImages,
                   UImage * img, int pixFactor) // directions
{ // Searching neighbors to find straight borders
  const int MAX_SQUARE_CORNERS = 100;
  struct SBest
    { // structure for finding best corner candidates
      int N;
      int ID;
      int Dir;
    };
  bool result = false;
  int i, j, n, k=0;
  float dx, dy, d, limit;
  UPixel pix;
  struct SBest corners[MAX_SQUARE_CORNERS];
  int cornerCnt = 0;
  UCalibrationComponent *ht, *ht1, *ht2;
  UCalibXYStat htTok1, htTok2, k1ToB, k2ToB;
  //char s[MAX_CAM_INFO_SIZE];
  //
  // find row length
  for (i = 0; i < hitCount; i++)
  {
    j = DoFindRowLength4(i, &k, size);
    if (j > 1)
    { // more than one row of size corners
      // take as one of the best
      if (cornerCnt < MAX_SQUARE_CORNERS)
      {
        corners[cornerCnt].N = j;
        corners[cornerCnt].ID = i;
        corners[cornerCnt].Dir = k;
        cornerCnt++;
      }
      else if (j > corners[cornerCnt-1].N)
      { // insert this as last element
        corners[cornerCnt - 1].N = j;
        corners[cornerCnt - 1].ID = i;
        corners[cornerCnt - 1].Dir = k;
      }
      //
      for (n = cornerCnt - 2; n >= 0; n--)
      { // order using boble sort
        if (j > corners[n].N)
        { // this one is better, so move
          corners[n + 1] = corners[n];
          corners[n].N = j;
          corners[n].ID = i;
          corners[n].Dir = k;
        }
        else
          // no need to continue
          break;
      }
    }
  }
  //
  if ((img != NULL) and extraImages)
  {
    //printf("Found %d potential frame corners\n", cornerCnt);
    for (i = 0; i < cornerCnt; i++)
    {
      ht = &hits[corners[i].ID];
      cvCircle(img->cvArr(),
               cvPoint(int(roundi(ht->x * pixFactor)),
                       int(roundi(ht->y * pixFactor))),
               5, CV_RGB(255, 255, 255));
    }
  }
  // debug
  /*
  if (cornerCnt > 2)
  {
    printf("Found %d candidate corners, the least has %d sides\n",
      cornerCnt, corners[cornerCnt - 1].N);
    for (n = 0; n < cornerCnt; n++)
    {
      ht = &hits[corners[n].ID]; // cornerpoint
      k = corners[n].Dir;  // direction along one edge
      ht1 = &hits[ht->NSEW[k].endId]; // get corner at end of side
      printf(" -- #%2d at %3.0f,%3.0f - %d dirs, ->%d is %3.0f,%3.0f\n",
          n, ht->x, ht->y, corners[n].N, corners[n].Dir,
          ht1->x, ht1->y);
    }
  }
  */
  // debug end
  //
  //
  // now the N best corner candidates are found
  // one of these will be the reference corner
  if (cornerCnt >= 2)
  { // there is at least 2 valid corners
    for (i = 0; i < cornerCnt; i++)
    { // find a corner with two equal length sides.      - hits[32]
      *k0 = corners[i].ID;      // reference corner
      *k0dk1 = corners[i].Dir;  // direction code to k1
      ht = &hits[*k0];
      *k1 = ht->NSEW[*k0dk1].endId;
      *k2 = -1;
      for (j = 0; j < 8; j++)
      { // use only valid directions
        if (ht->NSEW[j].id < 0)
          break;
        // try all other directions
        if (j != corners[i].Dir)
        { // find ID of other end
          if (ht->NSEW[j].n == size)
          { // equally good direction found
            *k2 = ht->NSEW[j].endId;
            *k0dk2 = j;
            break;
          }
        }
      }
      // now k1 and k2 are ID on tow near corners
      //  k0 --- k1 (may be oriented differently)
      //  |      |
      //  |
      //  k2 --  B
      if (k2 >= 0)
      { // k1 and k2 corners are valid
        // now see if (ht--k1) are parallel with (k2--B).
        *k1dB = -1; // direction from corner k1 to B
        *k2dB = -1; // direction from corner k2 to B
        // First find direction to B from k1 and k2
        ht1 = &hits[*k1];
        // limit value for parallel side
        n = size - 3;
        for (j = 0; j < 8; j++)
        { // only valid directions
          if (ht1->NSEW[j].id < 0)
            break;
          // not the direction back to k0
          if (ht1->NSEW[j].endId != *k0)
          { // get best direction - need not quite as long
            // if 2 shorter, there is still a good chance
            // of valid content
            if (ht1->NSEW[j].n > n)
            {
              n = ht1->NSEW[j].n;
              *k1dB = j; // direction to B from k1
            }
          }
        }
        // and now for corner k2
        ht2 = &hits[*k2];
        // limit value for parallel side
        n = size - 3;
        for (j = 0; j < 8; j++)
        { // only valid directions
          if (ht2->NSEW[j].id < 0)
            break;
          // not the direction back to k0
          if (ht2->NSEW[j].endId != *k0)
          { // get best direction - need not quite as long
            // if 2 shorter, there is still a good chance
            // of valid content
            if (ht2->NSEW[j].n > n)
            {
              n = ht2->NSEW[j].n;
              *k2dB = j; // direction to B from k1
            }
          }
        }
        //
        // if both found
        if ((*k1dB >= 0) and (*k2dB >= 0))
        { // now check to se if they are reasonably parallel
          // k1--B should be parallel with ht--k2
          htTok1 = ht->NSEW[*k0dk1];
          htTok2 = ht->NSEW[*k0dk2];
          k1ToB  = ht1->NSEW[*k1dB];
          k2ToB  = ht2->NSEW[*k2dB];
          //
          dx = (htTok1.vx / htTok1.n) - (k2ToB.vx / k2ToB.n);
          dy = (htTok1.vy / htTok1.n) - (k2ToB.vy / k2ToB.n);
          d = sqrt(sqr(dx) + sqr(dy));
          limit = htTok1.d/NON_PARALLEL_FACTOR;
          //
          // debug
          /*
          snprintf(s, MAX_CAM_INFO_SIZE,
               " Try: --- Corners: 1st (%3.0f,%3.0f), 2nd (%3.0f,%3.0f), 3rd(%3.0f,%3.0f)",
               ht->x, ht->y, ht1->x, ht1->y, ht2->x, ht2->y);
          sVinfo(s, ewarning, 0);
          */
          // debug end
          //
          if (d < limit)
          { // then try if the other sides are parallel too
            dx = (htTok2.vx / htTok2.n) - (k1ToB.vx / k1ToB.n);
            dy = (htTok2.vy / htTok2.n) - (k1ToB.vy / k1ToB.n);
            d = sqrt(sqr(dx) + sqr(dy));
            limit = htTok2.d/NON_PARALLEL_FACTOR;
            //
            if (d < limit)
            { // everything is good.
              // we need a final test to awoid too flat
              // squares
              result = testForFlatness(ht, ht1, ht2);
              if (result)
              {
                result = true;
                // debug
                //printf(" -- #%d is good!\n", i);
                // debug end
                break;
              }
              // debug
/*              else
                printf("doOrderHits4(...) ignored too flat frame\n");*/
                // debug end
            }
            else
            { /*
              snprintf(s, MAX_CAM_INFO_SIZE,
                  " -- #%d Failed on not parallel k0->k2 and k1->B as (%1.1f > %1.1f)",
                   i, d, limit);
              sVinfo(s, ewarning, 0); */
            }
          }
          else
          { /*
            snprintf(s, MAX_CAM_INFO_SIZE,
                   " -- #%d Failed on not parallel k0->k1 and k2->B as (%1.1f > %1.1f)",
                   i, d, limit);
            sVinfo(s, ewarning, 0); */
          }
        }
        else
        { // debug
          //printf(" -- corner #%d failed on k1dB and k2dB not both found\n", i);
          // debug end
        }
      }
      else
      { // debug
        //printf(" -- corner #%d failed on k2 = -1\n", i);
        // debug end
      }
      // this is not usable, and should not be used again
      //hits[*k0].w = 0;
      // debug
      // printf(" -- corner #%d failed (hits[%d]) is marked as invalid\n", i, *k0);
      // debug end
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////

bool UCalibrationComponents::testForFlatness(UCalibrationComponent * ht,
                                              UCalibrationComponent * ht1,
                                              UCalibrationComponent * ht2)
{
  bool result = false;
  UCalibrationComponent *p2, *p3;
  float d1, d2, d3;
  U2Dline lin;
  //
  d1 = hypot(ht->x - ht1->x,  ht->y - ht1->y);
  d2 = hypot(ht->x - ht2->x,  ht->y - ht2->y);
  d3 = hypot(ht1->x - ht2->x,  ht1->y - ht2->y);
  if ((d1 < d2) and (d1 < d3))
  { // ht to ht1 is the short side
    p2 = ht2;
    p3 = ht1;
  }
  else
  { // ht to ht1 is one of the long sides
    p2 = ht1;
    p3 = ht2;
  }
  lin.set2P(ht->x, ht->y, p2->x, p2->y);
  d1 = absf(lin.distanceSigned(p3->x, p3->y));
  if (d1 > 14.0)
    result = true;
  return result;
}


///////////////////////////////////////////////////


bool UCalibrationComponents::doMakeCodeGrid(
              int k0, int k1, int k2,  // three corners
              int k0dk1, int k0dk2, int k1dB, int k2dB, // direction to other
              int size, // side size of barcode frame (e.g. 7x7)
              int sideFactor,
              float stride, // block size in meter
              UCalibrationMarkSet * markSet,
              bool alignFrame, // rectify corners to straight lines
              bool saveToFile)
{ // extracts code from 2D barchard
  bool result = true;
  UCalibrationComponent * k0c  = &hits[k0];
  UCalibrationComponent * k1c = &hits[k1];
  UCalibrationComponent * k2c = &hits[k2];
  UCalibrationComponent * cc;
  int i, j, m, n;
  float x, y, dx, dy;
  int k0Rk1[MAX_SIDE_LENGTH+1]; // index numbers for corners on this line
  int k0Rk2[MAX_SIDE_LENGTH+1]; // index numbers for corners on this line
  int k1RB[MAX_SIDE_LENGTH+1];  // index numbers for corners on this line
  int k2RB[MAX_SIDE_LENGTH+1];  // index numbers for corners on this line
  // 2D point arrays for side points
  float k0Rk1px[MAX_SIDE_LENGTH + 1];
  float k0Rk1py[MAX_SIDE_LENGTH + 1];
  float k0Rk2px[MAX_SIDE_LENGTH + 1];
  float k0Rk2py[MAX_SIDE_LENGTH + 1];
  float k1RBpx[MAX_SIDE_LENGTH + 1];
  float k1RBpy[MAX_SIDE_LENGTH + 1];
  float k2RBpx[MAX_SIDE_LENGTH + 1];
  float k2RBpy[MAX_SIDE_LENGTH + 1];
  // 2D point arrays for side points - unalligned
  float k0Rk1pxo[MAX_SIDE_LENGTH + 1];
  float k0Rk1pyo[MAX_SIDE_LENGTH + 1];
  float k0Rk2pxo[MAX_SIDE_LENGTH + 1];
  float k0Rk2pyo[MAX_SIDE_LENGTH + 1];
  float k1RBpxo[MAX_SIDE_LENGTH + 1];
  float k1RBpyo[MAX_SIDE_LENGTH + 1];
  float k2RBpxo[MAX_SIDE_LENGTH + 1];
  float k2RBpyo[MAX_SIDE_LENGTH + 1];
  //
  char frameCW[5];  // 4 sides plus string zero
  char frameFwd[5]; // 4 sides plus string zero
  // line equations [A,B,C] Ax + By + C = 0
  U2Dline k0Lk1;
  U2Dline k0Lk2;
  U2Dline k1LB;
  U2Dline k2LB;
  U2Dline Lrow, Lcol;
  //
  //bool isOK;
  //
  // debug
  FILE * fl = NULL;
  char s[100];
  // debug end
  //
  // get route stations on all sides
  /*   void UCalibrationComponents::findNextInThisDirection4(
                  int id,   // candidate
                  float dx, // this far in x direction
                  float dy, // and this far in y direction
                  float dd, // at this distance
                  int * ID, // id of last corner in row
                  float * vx, // add x-dist here if found
                  float * vy, // add y-dist here if found
                  int   *  n, // increase this count if found
                  int * next) // if != NULL, then save route */
  k0Rk1[0] = k0;
  k0Rk1[1] = k0c->NSEW[k0dk1].id;
  k0Rk1[size] = 0;
  j = 1;
  findNextInThisDirection4(k0Rk1[1],
                 k0c->NSEW[k0dk1].x, k0c->NSEW[k0dk1].y, k0c->NSEW[k0dk1].d,
                 &i, &x, &y, &j, k0Rk1, size);
  if (j < size)
    result = false;
  k0Rk2[0] = k0;
  k0Rk2[1] = k0c->NSEW[k0dk2].id;
  k0Rk2[size] = 0;
  j = 1;
  findNextInThisDirection4(k0Rk2[1],
                 k0c->NSEW[k0dk2].x, k0c->NSEW[k0dk2].y, k0c->NSEW[k0dk2].d,
                 &i, &x, &y, &j, k0Rk2, size);
  if (j < size)
    result = false;
  k1RB[0] = k1;
  k1RB[1] = k1c->NSEW[k1dB].id;
  k1RB[size] = 0;
  j = 1;
  findNextInThisDirection4(k1RB[1],
                 k1c->NSEW[k1dB].x, k1c->NSEW[k1dB].y, k1c->NSEW[k1dB].d,
                 &i, &x, &y, &j, k1RB, size);
  if (j < size)
    result = false;
  k2RB[0] = k2;
  k2RB[1] = k2c->NSEW[k2dB].id;
  k2RB[size] = 0;
  j = 1;
  findNextInThisDirection4(k2RB[1],
                 k2c->NSEW[k2dB].x, k2c->NSEW[k2dB].y, k2c->NSEW[k2dB].d,
                 &i, &x, &y, &j, k2RB, size);
  if (j < size)
    result = false;
  //
  //
  // k0(ht) --- k1    side length is sideSize elements
  // |          |     distnace from ht to k1 is (x,y)=(htTok1dx,htTok1dy)
  // |          |
  // k2 ------- B
  //
  if (result)
  {
    for (i = 0; i <= size; i++)
    {
      cc = &hits[k0Rk1[i]];
      k0Rk1px[i] = cc->x;
      k0Rk1py[i] = cc->y;
    }
    j  = k0Lk1.set(k0Rk1px, k0Rk1py, size);
    for (i = 0; i <= size; i++)
    {
      cc = &hits[k0Rk2[i]];
      k0Rk2px[i] = cc->x;
      k0Rk2py[i] = cc->y;
    }
    j += k0Lk2.set(k0Rk2px, k0Rk2py, size);
    for (i = 0; i <= size; i++)
    {
      cc = &hits[k1RB[i]];
      k1RBpx[i] = cc->x;
      k1RBpy[i] = cc->y;
    }
    j += k1LB.set(k1RBpx, k1RBpy, size);
    for (i = 0; i <= size; i++)
    {
      cc = &hits[k2RB[i]];
      k2RBpx[i] = cc->x;
      k2RBpy[i] = cc->y;
    }
    j += k2LB.set(k2RBpx, k2RBpy, size);
    if (j != 0)
      result = false;
  }
  if (alignFrame or true)
  {
    if (result)
    { // save un-corrected point set
      // for camera calibration
      for (i = 0; i <= size; i++)
      { // last position may be missing??
        k0Rk1pxo[i] = k0Rk1px[i];
        k0Rk1pyo[i] = k0Rk1py[i];
        k0Rk2pxo[i] = k0Rk2px[i];
        k0Rk2pyo[i] = k0Rk2py[i];
        k1RBpxo[i]  = k1RBpx[i];
        k1RBpyo[i]  = k1RBpy[i];
        k2RBpxo[i]  = k2RBpx[i];
        k2RBpyo[i]  = k2RBpy[i];
      }
      // rectify crossings
      // set corners, where the found lines cross
      // first reference corner by k0
      result = k0Lk1.getCrossing(k0Lk2, &k0Rk1px[0], &k0Rk1py[0]);
      //if (alignFrame)
      {
        k0Rk2px[0] = k0Rk1px[0];
        k0Rk2py[0] = k0Rk1py[0];
      }
      // the corner by k1
      result = result and  k0Lk1.getCrossing(k1LB, &k1RBpx[0], &k1RBpy[0]);
      //if (alignFrame)
      {
        k0Rk1px[size] = k1RBpx[0];
        k0Rk1py[size] = k1RBpy[0];
      }
      // then corner by k2
      result = result and  k0Lk2.getCrossing(k2LB, &k2RBpx[0], &k2RBpy[0]);
      //if (alignFrame)
      {
        k0Rk2px[size] = k2RBpx[0];
        k0Rk2py[size] = k2RBpy[0];
      }
      // then the last corner by B
      result = result and  k1LB.getCrossing(k2LB, &k2RBpx[size], &k2RBpy[size]);
      //if (alignFrame)
      {
        k1RBpx[size] = k2RBpx[size];
        k1RBpy[size] = k2RBpy[size];
      }
      //
      /*
      // add the last raw corners (size+1) in each line
        k0Rk1pxo[size] = k1RBpxo[0];
        k0Rk1pyo[size] = k1RBpyo[0];
        k0Rk2pxo[size] = k2RBpxo[0];
        k0Rk2pyo[size] = k2RBpyo[0];
        k1RBpxo[size] = k2RBpx[size];
        k1RBpyo[size] = k2RBpy[size];
        // the last point in B is then taken from
        // the alligned corner - not quite right
        k2RBpxo[size] = k1RBpxo[size]
        k2RBpyo[size] = k1RBpyo[size]
      //
      */
      //
    }
    if (result /*and alignFrame*/)
    { // now straighten the other points,
      // first by moving them to the nearest point on line
      // k0 to k1
      for (i = 1; i < size; i++)
        k0Lk1.getOnLine(k0Rk1px[i], k0Rk1py[i], &k0Rk1px[i], &k0Rk1py[i]);
      // k0 to k2
      for (i = 1; i < size; i++)
        k0Lk2.getOnLine(k0Rk2px[i], k0Rk2py[i], &k0Rk2px[i], &k0Rk2py[i]);
      // k1 to B
      for (i = 1; i < size; i++)
        k1LB.getOnLine(k1RBpx[i], k1RBpy[i], &k1RBpx[i], &k1RBpy[i]);
      // k2 to B
      for (i = 1; i < size; i++)
        k2LB.getOnLine(k2RBpx[i], k2RBpy[i], &k2RBpx[i], &k2RBpy[i]);
      //
      // the points along the lines should be equally spaced, following
      // a curve - like y = a / (z * b) - (b - c* n)/(sqr(n) * d)
      // it is anyhow aproximized to a linear curve
      //
      // not done yet
      //
      //
    }
  }
  if (result and (markSet != NULL))
  { // Save frame points in markSet in clockwise order from
    // top left image corner.
    // There are then 8 possibilities of side odering of A,B,C,D:
    //                  1: ACDB
    //   k0 +--A--+ k1  2: ABCD
    //      |     |     3: CDBA
    //      B     C     4: BDCA
    //      |     |     5: DBCA
    //   k2 +--D--+ B   6: DCBA
    //                  7: BACD
    //                  8: CABD
    // first find top left corner
    // what is most top K0 or B
    if (k0Rk1py[0] < k1RBpy[size])
    { // k0 is more top, what is more top K1 or K2
      if (k1RBpy[0] < k2RBpy[0])
      { // k1 is more top, find is k0 or k1 first
        if (k0Rk1px[0] < k1RBpx[0])
        { // k0 is top left,  find clockwise next corner
          if (k0Rk1px[size] > k0Rk2px[size])
          { // k1 is next clockwise: k0->k1->B->k2->k0
            strcpy(frameCW,  "ACDB"); // 1
            strcpy(frameFwd, "ttff");
          }
          else
          { // k2 is next clockwise: k0->k2->B->k1->k0
            strcpy(frameCW,  "BDCA"); // 4
            strcpy(frameFwd, "ttff");
          }
        }
        else
        { // k1 is top left (k0 or B is next), find clockwise next corner
          if (k0Rk1px[0] > k1RBpx[size])
          { // k0 is next clockwise: k1->k0->k2->B->k1
            strcpy(frameCW,  "ABDC"); // 2
            strcpy(frameFwd, "fttf");
          }
          else
          { // B is next clockwise: k1->B->k2->k0->k1
            strcpy(frameCW,  "CDBA"); // 3
            strcpy(frameFwd, "tfft");
          }
        }
      }
      else
      { // k2 is more top than k1, find is k0 or k2 first
        if (k0Rk1px[0] < k2RBpx[0])
        { // k0 is top left, find clockwise next corner (k2 or k1)
          if (k0Rk1px[size] > k0Rk2px[size])
          { // k1 is next clockwise: k0->k1->B->k2->k0
            strcpy(frameCW,  "ACDB"); // 1
            strcpy(frameFwd, "ttff");
          }
          else
          { // k2 is next CW: k0->k2->B->k1->k0
            strcpy(frameCW,  "BDCA"); // 4
            strcpy(frameFwd, "ttff");
          }
        }
        else
        { // k2 to top left, find clockwise next (k0 or B)
          if (k0Rk2px[0] > k2RBpx[size])
          { // k0 is next clockwise: k2->k0->k1->B->k2
            strcpy(frameCW,  "BACD"); // 7
            strcpy(frameFwd, "fttf");
          }
          else
          { // B is next clockwise: k2->B->k1->k0->k2
            strcpy(frameCW,  "DCAB"); // 6
            strcpy(frameFwd, "tfft");
          }
        }
      }
    }
    else
    { // B is more top than k0, what is more top K1 or K2
      if (k1RBpy[0] < k2RBpy[0])
      { // k1 is more top, find is B or k1 first
        if (k1RBpx[size] < k1RBpx[0])
        { // B is top left,  find clockwise next corner (k1 or k2)
          if (k1RBpx[0] > k2RBpx[0])
          { // k1 is next clockwise: B->k1->k0->k2->B
            strcpy(frameCW,  "CABD");   // 8
            strcpy(frameFwd, "fftt");
          }
          else
          { // k2 is next clockwise: B->k2->k0->k1->B
            strcpy(frameCW,  "DBAC"); // 5
            strcpy(frameFwd, "fftt");
          }
        }
        else
        { // k1 is top left (k0 or B is next), find clockwise next corner
          if (k0Rk1px[0] > k1RBpx[size])
          { // k0 is next clockwise: k1->k0->k2->B->k1
            strcpy(frameCW,  "ABDC"); // 2
            strcpy(frameFwd, "fttf");
          }
          else
          { // B is next clockwise: k1->B->k2->k0->k1
            strcpy(frameCW,  "CDBA"); // 3
            strcpy(frameFwd, "tfft");
          }
        }
      }
      else
      { // k2 is more top than k1, find is B or k2 first
        if (k2RBpx[size] < k2RBpx[0])
        { // B is top left, find clockwise next corner (k2 or k1)
          if (k0Rk1px[size] > k0Rk2px[size])
          { // k1 is next clockwise: B->k1->k0->k2->B
            strcpy(frameCW,  "CABD"); // 8
            strcpy(frameFwd, "fftt");
          }
          else
          { // k2 is next clockwise: B->k2->k0->k1->B
            strcpy(frameCW,  "DBAC");  // 5
            strcpy(frameFwd, "fftt");
          }
        }
        else
        { // k2 to top left, find clockwise next (k0 or B)
          if (k0Rk2px[0] > k2RBpx[size])
          { // k0 is next clockwise: k2->k0->k1->B->k2
            strcpy(frameCW,  "BACD"); // 7
            strcpy(frameFwd, "fttf");
          }
          else
          { // B is next clockwise: k2->B->k1->k0->k2
            strcpy(frameCW,  "DCAB"); // 6
            strcpy(frameFwd, "tfft");
          }
        }
      }
    }
    //
    // save coord positions in right order
    if (alignFrame)
      markSet->setFramePositions(size, stride, frameCW, frameFwd,
                          k0Rk1px, k0Rk1py, // A
                          k0Rk2px, k0Rk2py, // B
                          k1RBpx,  k1RBpy,  // C
                          k2RBpx,  k2RBpy); // D
    else
      // use uncorrected corner positions
      markSet->setFramePositions(size, stride, frameCW, frameFwd,
                          k0Rk1pxo, k0Rk1pyo, // A
                          k0Rk2pxo, k0Rk2pyo, // B
                          k1RBpxo,  k1RBpyo,  // C
                          k2RBpxo,  k2RBpyo); // D
    //
    if (saveToFile)
    {  // save frame to MATLAB file, both alligned and raw positions
      snprintf(s, 100, "cornerPoints%03.0fx%03.0f.m", k0Rk1px[0], k0Rk1py[0]);
      fl = fopen(s, "w");
      if (fl != NULL)
      {
        fprintf(fl, "%% frame positions alligned and not alligned\n");
        fprintf(fl, "%% [x-raw, x-alligned, y-raw, y-alligned]\n");
        fprintf(fl, "k0Rk1 = [ ...\n");
        for (i = 0; i <= size; i++)
        {
          fprintf(fl, "%f, %f, %f, %f",
                     k0Rk1pxo[i], k0Rk1px[i], k0Rk1pyo[i], k0Rk1py[i]);
          if (i < size)
            fprintf(fl,"; ...\n");
          else
            fprintf(fl,"];\n");
        }
        fprintf(fl, "k0Rk2 = [ ...\n");
        for (i = 0; i <= size; i++)
        {
          fprintf(fl, "%f, %f, %f, %f",
                     k0Rk2pxo[i], k0Rk2px[i], k0Rk2pyo[i], k0Rk2py[i]);
          if (i < size)
            fprintf(fl,"; ...\n");
          else
            fprintf(fl,"];\n");
        }
        fprintf(fl, "k1RB = [ ...\n");
        for (i = 0; i <= size; i++)
        {
          fprintf(fl, "%f, %f, %f, %f",
                     k1RBpxo[i], k1RBpx[i], k1RBpyo[i], k1RBpy[i]);
          if (i < size)
            fprintf(fl,"; ...\n");
          else
            fprintf(fl,"];\n");
        }
        fprintf(fl, "k2RB = [ ...\n");
        for (i = 0; i <= size; i++)
        {
          fprintf(fl, "%f, %f, %f, %f",
                     k2RBpxo[i], k2RBpx[i], k2RBpyo[i], k2RBpy[i]);
          if (i < size)
            fprintf(fl,"; ...\n");
          else
            fprintf(fl,"];\n");
        }
        fprintf(fl, "hold off\n");
        fprintf(fl, "plot([k0Rk1(:,2);k1RB(:,2);k0Rk2(:,2);k2RB(:,2)], ...\n");
        fprintf(fl, "     -[k0Rk1(:,4);k1RB(:,4);k0Rk2(:,4);k2RB(:,4)],'r');\n");
        fprintf(fl, "hold on\n");
        fprintf(fl, "plot([k0Rk1(:,1);k1RB(:,1);k0Rk2(:,1);k2RB(:,1)], ...\n");
        fprintf(fl, "     -[k0Rk1(:,3);k1RB(:,3);k0Rk2(:,3);k2RB(:,3)],'b');\n");
        fprintf(fl, "plot([k0Rk1(:,2);k1RB(:,2);k0Rk2(:,2);k2RB(:,2)], ...\n");
        fprintf(fl, "     -[k0Rk1(:,4);k1RB(:,4);k0Rk2(:,4);k2RB(:,4)],'*r');\n");
        fprintf(fl, "plot([k0Rk1(:,1);k1RB(:,1);k0Rk2(:,1);k2RB(:,1)], ...\n");
        fprintf(fl, "     -[k0Rk1(:,3);k1RB(:,3);k0Rk2(:,3);k2RB(:,3)],'sb');\n");
        fprintf(fl, "grid on\n");
        fprintf(fl, "xlabel('pixel x (0 is left)');\n");
        fprintf(fl, "ylabel('pixel y (0 is top)');\n");
        fprintf(fl, "legend('alligned','raw');\n");
        fprintf(fl, "title('frame positions');\n");
        fclose(fl);
      }
    }
    // debug end
  } // end of save frame corner positions into markSet
  //
  // Corners and line equation variables
  //     0  1 . . . . .  j
  //
  // 0    k0 ---k0Lk1---- k1
  // 1    |               |
  // .  k0Lk2            k1LB
  // .    |               |
  // i    k2 ---k2LB----- B
  if (result)
  { // extract all top-left corners
    for (i = 0; i < size + 1; i++)
    { // i is row
      Lrow.set2P(k0Rk2px[i], k0Rk2py[i], k1RBpx[i], k1RBpy[i]);
      for (j = 0; j < size + 1; j++)
      { // j is column
        Lcol.set2P(k0Rk1px[j], k0Rk1py[j], k2RBpx[j], k2RBpy[j]);
        result = Lcol.getCrossing(Lrow, &x, &y);
        if (not result)
        { //
          printf("finding crossing failed!!\n");
        }
        // offset by 0.5 pixel to get to center of pixel
        mCode[i * sideFactor][j * sideFactor].px = x + 0.5;
        mCode[i * sideFactor][j * sideFactor].py = y + 0.5;
        // first fill in the flanks in left and right side
        if ((i > 0) and (sideFactor > 1))
        { // fill in the extra corners
          // first between rows at regular columns
          dx =  (mCode[i * sideFactor][j * sideFactor].px
                - mCode[(i - 1) * sideFactor][j * sideFactor].px) / float(sideFactor);
          dy =  (mCode[i * sideFactor][j * sideFactor].py
                - mCode[(i - 1) * sideFactor][j * sideFactor].py) / float(sideFactor);
          for (m = 1; m < sideFactor; m++)
          {
            x = mCode[(i - 1) * sideFactor][j * sideFactor].px + m * dx;
            y = mCode[(i - 1) * sideFactor][j * sideFactor].py + m * dy;
            mCode[(i - 1) * sideFactor + m][j * sideFactor].px = x;
            mCode[(i - 1) * sideFactor + m][j * sideFactor].py = y;
          }
          if (j > 0)
          { // then between columns for all rows between i and (i - 1)
            for (m = (i - 1) * sideFactor; m <= (i * sideFactor); m++)
            { // the last sub row should only be done for the last main row
              if ((m == (i * sideFactor)) and (i < size))
                break;
              dx =  (mCode[m][j * sideFactor].px
                    - mCode[m][(j - 1) * sideFactor].px) / float(sideFactor);
              dy =  (mCode[m][j * sideFactor].py
                    - mCode[m][(j - 1) * sideFactor].py) / float(sideFactor);
              for (n = 1; n < sideFactor; n++)
              {
                x = mCode[m][(j - 1) * sideFactor].px + n * dx;
                y = mCode[m][(j - 1) * sideFactor].py + n * dy;
                mCode[m][(j - 1) * sideFactor + n].px = x;
                mCode[m][(j - 1) * sideFactor + n].py = y;
              }
            }
          }
        }
      }
    }
  }
  //
  return result;
}

//////////////////////////////////////////////////////////////////////

bool UCalibrationComponents::doFindCodeCellIntensity(
                    int size, int sideFactor,
                    UImage * source, // needed for average intensity
                    int * borderIntensity, // resulting border intensity
                    bool extraImages,
                    UImage * destination,  // for extra images
                    int codeNumber)        // for histogram paint in extra images
{
  bool result = true;
  int i, j, k, l;
  unsigned int m, n;
  unsigned int x1, x2, y1, y2;
  UPixel pix;
  int intens = 0, intensCnt = 0;
  float scale = 1.0;
  int hist[128];
  //
  result = (source != NULL);
  // debug
  if (not result)
    printf("UCalibrationComponents::doFindCodeCellIntensity failed no source\n");
  // debug end
  if (result)
  { // Get intensity value in cells
    intens = 0;
    intensCnt = 0;
    // debug
    // changed i to start from 1, as value 0 is uninitialized?
    //
    for (i = 0; i < size * sideFactor; i++)
      for (j = 0; j < size * sideFactor; j++)
      { // clear values
        mCode[i][j].v = 0;
        mCode[i][j].n = 0;
        // get pixel square that covers the cell
        x1 = roundi(minf(minf(mCode[i][j].px, mCode[i+1][j].px),
                         minf(mCode[i][j+1].px, mCode[i+1][j+1].px)));
        y1 = roundi(minf(minf(mCode[i][j].py, mCode[i+1][j].py),
                         minf(mCode[i][j+1].py, mCode[i+1][j+1].py)));
        x2 = int(maxf(maxf(mCode[i][j].px, mCode[i+1][j].px),
                         maxf(mCode[i][j+1].px, mCode[i+1][j+1].px)));
        y2 = int(maxf(maxf(mCode[i][j].py, mCode[i+1][j].py),
                         maxf(mCode[i][j+1].py, mCode[i+1][j+1].py)));
        //
        // debug
//         if ((x1 > 640) or (y1 > 640) or (x2 > 640) or (y2 > 640))
//         { // sanity check
//           result = false;
//           // debug
//           printf("UCalibrationComponents::doFindCodeCellIntensity\n:");
//           printf(" px[i,j]=%f, px[i+1,j]=%f, px[i,j+1]=%f, px[i+1,j+1]=%f\n",
//                 mCode[i][j].px, mCode[i+1][j].px,
//                 mCode[i][j+1].px, mCode[i+1][j+1].px);
//           printf(" gave insane values:  x1,y1(%ud,%ud), x2,y2(%ud,%ud)\n", x1, y1, x2, y2);
//           // debug end
//         }
        // debug end
        //
        if (result)
        { // scan over this area to get average intensity within cell
          for (m = x1; m <= x2; m++)
            for (n = y1; n <= y2; n++)
            { // test if within cell
              if (isWithinSquare(float(m) + 0.5,float(n) + 0.5,
                          mCode[i][j].px, mCode[i][j].py,
                          mCode[i][j+1].px, mCode[i][j+1].py,
                          mCode[i+1][j+1].px, mCode[i+1][j+1].py,
                          mCode[i+1][j].px, mCode[i+1][j].py))
              {
                // get pixel inside this region
                if (source->inRange(n, m))
                { // is in image
                  // -- at times part of code may be (falsely) estimated outside
                  pix = source->getPix(n,m);
                  // add pixel intensity to cell value
                  mCode[i][j].v += pix.y;
                  mCode[i][j].n ++;
                }
              }
            }
          // calculate aveage intensity
          if (mCode[i][j].n > 0)
          {
            mCode[i][j].v /= mCode[i][j].n;
            if ((i < sideFactor) or (i > ((size - 1) * sideFactor)) or
                (j < sideFactor) or (j > ((size - 1) * sideFactor)))
            { // get average intensity in border area
              intens += mCode[i][j].v;
              intensCnt++;
            }
          }
        }
      }
  }
  //
  //
  if (result)
  { // calculate average intensity in border area
    if (intensCnt > 0)
      intens /= intensCnt;
  }
  //
  //
  if (result and extraImages and (destination != NULL))
  { // prepare destination image
    // destination->copyToMaxRes(source); -- done already
    // pix.SetPix(255, 128, 128); // white
    // destination->Tone(&pix,50);
    // destination->imgTime.Now();
    // snprintf(destination->name, MaxImageNameLength, "grid%d", source->imageNumber);
    scale = float(destination->width()) / float(source->width());
    // paint cells in result image
    for (i = 0; i < size * sideFactor; i++)
      for (j = 0; j < size * sideFactor; j++)
      { // get pixel square that covers the cell
        x1 = roundi(minf(minf(mCode[i][j].px, mCode[i+1][j].px),
                         minf(mCode[i][j+1].px, mCode[i+1][j+1].px)));
        y1 = roundi(minf(minf(mCode[i][j].py, mCode[i+1][j].py),
                         minf(mCode[i][j+1].py, mCode[i+1][j+1].py)));
        x2 = int(maxf(maxf(mCode[i][j].px, mCode[i+1][j].px),
                         maxf(mCode[i][j+1].px, mCode[i+1][j+1].px)));
        y2 = int(maxf(maxf(mCode[i][j].py, mCode[i+1][j].py),
                         maxf(mCode[i][j+1].py, mCode[i+1][j+1].py)));
        // debug
//         if ((x1 > 640) or (y1 > 640) or (x2 > 640) or (y2 > 640))
//           // sanity check
//           result = false;
        // debug end
        if (result)
          for (m = x1; m <= x2; m++)
            for (n = y1; n <= y2; n++)
              if (isWithinSquare(float(m)+0.5,float(n)+0.5,
                         mCode[i][j].px, mCode[i][j].py,
                         mCode[i][j+1].px, mCode[i][j+1].py,
                         mCode[i+1][j+1].px, mCode[i+1][j+1].py,
                         mCode[i+1][j].px, mCode[i+1][j].py))
              { // make green in result image
                pix = source->getPix(n,m);
                // tone cells in result area
                if (mCode[i][j].v > intens)
                { // tone towards green ((-u,-v) is green)    pix.y
                  // if cell is white'ish
                  pix.u = (pix.u + 255) / 2;
                }
                else
                { // tone towards dark red
                  // if cell is black'ish
                  pix.u /= 2;
                  pix.v /= 2;
                }
                // set pixel in result image
                for (k = 0; k < roundi(scale); k++)
                  for (l = 0; l < roundi(scale); l++)
                    destination->setPix(n * roundi(scale) + k, m * roundi(scale) + l, pix);
              }
          //
      }
    //
    // paint cell border lines in code color
    // get paletted color (1 of 6 colors)
    pix.set(codeNumber, 6);
    pix = pix.asYUV(PIX_PLANES_BGR);
    //
    for (i = 0; i <= size * sideFactor; i++)
    { // i is row
      for (j = 0; j <= size * sideFactor; j++)
      { // j is column
        x1 = roundi(mCode[i][j].px * scale);
        y1 = roundi(mCode[i][j].py * scale);
        x1 = maxi(0, mini(destination->width(), x1));
        y1 = maxi(0, mini(destination->height(), y1));
        if (i < (size * sideFactor))
        {
          x2 = roundi(mCode[i+1][j].px * scale);
          y2 = roundi(mCode[i+1][j].py * scale);
          x2 = maxi(0, mini(destination->width(), x2));
          y2 = maxi(0, mini(destination->height(), y2));
          cvLine(destination->cvArr(), cvPoint(x1, y1), cvPoint(x2, y2), pix.cvRGB());
        }
        if (j < (size * sideFactor))
        {
          x2 = roundi(mCode[i][j+1].px * scale);
          y2 = roundi(mCode[i][j+1].py * scale);
          x2 = maxi(0, mini(destination->width(), x2));
          y2 = maxi(0, mini(destination->height(), y2));
          //destination->PaintLine(x1, y1, x2, y2, &pix);
          cvLine(destination->cvArr(), cvPoint(x1, y1), cvPoint(x2, y2), pix.cvRGB());
        }
      }
    }
    //
    // paint intensity histograms
    for (i = 0; i < 128; i++)
    { // clear histogram
      hist[i] = 0;
    }
    for (i = 1; i < size * sideFactor - 1; i++)
      for (j = 1; j < size * sideFactor - 1; j++)
      { // get intensity
        l = (mCode[i][j].v >> 1) & 0x7f;
        hist[l]++;
      }
    //
    //
    y1 = destination->height() - 20 - 20 * codeNumber;
    for (i = 0; i < 128; i++)
    {
      x1 = i*2 + 20;
      x2 = x1;
      y2 = y1 - hist[i] * 4 - 1;
      cvLine(destination->cvArr(), cvPoint(x1, y1),
                         cvPoint(x2, y2),
                         pix.cvRGB());
      x1++;
      x2++;
      cvLine(destination->cvArr(), cvPoint(x1, y1),
                         cvPoint(x2, y2), pix.cvRGB());
    }
    // paint circle at cross-over intensity
    pix.setRGBto(255, 0, 0, PIX_PLANES_RGB);
    x1 = intens + 22;
    cvCircle(destination->cvArr(), cvPoint(x1, y1), 5, pix.cvRGB());
  }
  //
  if (result)
    *borderIntensity = intens;
  return result;
}

///////////////////////////////////////////////////////////////////
int getCellCode(char code[], int size, int sideFactor, int row, int col)
{ // get code from bit value in cell of side size 'sideFactor'
  // in an array of cells size 'w'x'w', where w = 'size' - 2
  int result;
  char * crow;
  int i,j;
  int w, b;
  //    col |
  // ---+---+--
  // row|1 2|  (sideFactor == 2)
  //    |3 4|
  // ---+---+--
  result = 0;
  b = 1 << (sideFactor * sideFactor - 1); // MSB
  w = (size - 2) * sideFactor; // bits per row
  for (i = 0; i < sideFactor; i++)
  { // get LSB row first
    crow = &code[row * w * sideFactor + i * w];
    for (j = 0; j < sideFactor; j++)
    { // get LSB bit first
      result = result >> 1;                  // code[0] code[1] code[12] code[13]
      if (crow[j + col * sideFactor] == 1)   // crow[0] crow[1] crow[2] crow[3]
        result += b;
    }
  }
  //
  return result;
}

//////////////////////////

bool UCalibrationComponents::doFindCodeValue(
                             int size, int sideFactor,
                             int code[], int * codeLng, int maxCodeLng,
                             unsigned long * intCode,
                             bool debug,
                             int splitIntensity,
                             int * indexM, // index corner
                             int * indexN)
{
  bool result = true;
  //int hist[128];
  //int intensVally = 0;
  int i, j, m, n; //, k, l,
  int w;
  int sm, sn, mdr, mdc, ndr, ndc;
  const int cw = (MAX_SIDE_LENGTH - 2) * MAX_CELL_FACTOR;
  char bCode[cw * cw];
  char * bCodeRow;
  const int MSL = 250;
  char s[MSL];
  //
  w = cw;
  //
  //  k0 ----- k1
  //  |         |
  //  |         |
  //  k2 ------ B
  //
  // make raw code grid
  if (result)
  { // assume k0 is upper left corner
    sm = sideFactor; // start row
    mdr = 1;         // m change per code row change
    mdc = 0;         // m change per column row change
    sn = sideFactor; // start column
    ndc = 1;         // n change per code column change
    ndr = 0;         // n change per code row change
    for (i = 0; i < (size - 2) * sideFactor; i++)
    {
      bCodeRow = &bCode[i * (size - 2) * sideFactor];
      for (j = 0; j < (size - 2) * sideFactor; j++)
      {
        m = i * mdr + j * mdc + sm;
        n = i * ndr + j * ndc + sn;
        if (mCode[m][n].v < splitIntensity) //intensVally)
          bCodeRow[j] = 1; // black is one
        else
          bCodeRow[j] = 0; // white is zero
      }
    }
  }
  //
  //
  if (result and debug)
  { // paint raw code to console
    for (i = 0; i < (size - 2) * sideFactor; i++)
    { // paint a row
      bCodeRow = &bCode[i * (size - 2) * sideFactor];
      snprintf(s, MSL, "Raw code %2d: ", i);
      for (j = 0; j < (size - 2) * sideFactor; j++)
        if (bCodeRow[j] == 1)
          s[13 + j] = '1';
        else
          s[13 + j] = '0';
      // terminate string
      s[13 + (size - 2) * sideFactor] = 0;
      toLog(s, 3);
    }
  }
  //
  //
  //  k0 ----- k1                              bCode[0] bCode[1]
  //  | xx      |                              bCode[12] bCode[13]
  //  |         |
  //  k2 ------ B (4)
  //
  // find filled corner cell and orientation
  if (result)
  { // for 7x7 guidemark 'size' is 5 and code area is 3x3
    w = size - 2; // width of code
    if (getCellCode(bCode, size, sideFactor, 0, 0) == 0xf)
    { // index is at k1
      sm = sideFactor; // skip margin
      sn = sideFactor; // skip margin
      if (getCellCode(bCode, size, sideFactor, 0, 1) == 0xf)
      { // index row points to k1
        mdr = 1; //  k0 ----- k1
        mdc = 0; //  | xx      |
        ndr = 0; //  |         |
        ndc = 1; //  k2 -----  B
      }
      else if (getCellCode(bCode, size, sideFactor, 1, 0) == 0xf)
      { // index row points to k2
        mdr = 0; //  k0 ----- k1
        mdc = 1; //  | x       |
        ndr = 1; //  | x       |
        ndc = 0; //  |         |
      }          //  k2 -----  B
      else
        result = false;
    }
    else if (getCellCode(bCode, size, sideFactor, 0, w - 1) == 0xf)
    { // index corner is at k1
      sm = sideFactor;
      sn = (w + 1) * sideFactor - 1;
      if (getCellCode(bCode, size, sideFactor, 1, w - 1) == 0xf)
      { // index row points to B
        mdr = 0;  //  k0 ----- k1
        mdc = 1;  //  |      x  |
        ndr = -1; //  |      x  |
        ndc = 0;  //  |         |
      }           //  k2 -----  B
      else if (getCellCode(bCode, size, sideFactor, 0, w - 2) == 0xf)
      { // index row points to B
        mdr = 1;  //  k0 ----- k1
        mdc = 0;  //  |     xx  |
        ndr = 0;  //  |         |
        ndc = -1; //  |         |
      }           //  k2 -----  B
      else
        result = false;
    }
    else if (getCellCode(bCode, size, sideFactor, w - 1, 0) == 0xf)
    { // index corner is at k2
      sm = (w + 1) * sideFactor - 1;
      sn = sideFactor;
      if (getCellCode(bCode, size, sideFactor, w - 1, 1) == 0xf)
      { // index row points to B
        mdr = -1; //  k0 ----- k1
        mdc = 0;  //  |         |
        ndr = 0;  //  |         |
        ndc = 1;  //  | xx      |
      }           //  k2 -----  B
      else if (getCellCode(bCode, size, sideFactor, w - 2, 0) == 0xf)
      { // index row points to B
        mdr = 0;  //  k0 ----- k1
        mdc = -1; //  |         |
        ndr = 1;  //  | x       |
        ndc = 0;  //  | x       |
      }           //  k2 -----  B
      else
        result = false;
    }
    else if (getCellCode(bCode, size, sideFactor, w - 1, w - 1) == 0xf)
    { // index corner is at B
      sm = (w + 1) * sideFactor - 1;
      sn = (w + 1) * sideFactor - 1;
      if (getCellCode(bCode, size, sideFactor, w - 1, w - 2) == 0xf)
      { // index row points to B
        mdr = -1; //  k0 ----- k1
        mdc = 0;  //  |         |
        ndr = 0;  //  |         |
        ndc = -1; //  |      xx |
      }           //  k2 -----  B
      else if (getCellCode(bCode, size, sideFactor, w - 2, w - 1) == 0xf)
      { // index row points to B
        mdr = 0;  //  k0 ----- k1
        mdc = -1; //  |         |
        ndr = -1; //  |       x |
        ndc = 0;  //  |       x |
      }           //  k2 -----  B
      else
        result = false;
    }
    else
      result = false;
  }
  //
  if (result)
  { // save position of index corner
    *indexM = sm;
    *indexN = sn;
    // get code bits in right order
    for (i = 0; i < (size - 2) * sideFactor; i++)
    {
      bCodeRow = &bCode[i * (size - 2) * sideFactor];
      for (j = 0; j < (size - 2) * sideFactor; j++)
      {
        m = i * mdr + j * mdc + sm;
        n = i * ndr + j * ndc + sn;
        if (mCode[m][n].v < splitIntensity) //intensVally)
          bCodeRow[j] = 1; // black is one
        else
          bCodeRow[j] = 0; // white is zero
      }
    }
  }
  //
  if (result and debug)
  { // paint raw code to console
    snprintf(s, MSL, "sm:%2d, mdr:%2d, mdc%2d, sn:%2d, ndr%2d, ndc%2d",
                         sm, mdr, mdc, sn, ndr, ndc);
    toLog(s, 4);
    for (i = 0; i < (size - 2) * sideFactor; i++)
    { // paint a row
      bCodeRow = &bCode[i * (size - 2) * sideFactor];
      snprintf(s, MSL, "Rgt code %2d: ", i);
      for (j = 0; j < (size - 2) * sideFactor; j++)
        if (bCodeRow[j] == 1)
          s[13 + j] = '1';
        else
          s[13 + j] = '0';
      // terminate string
      s[13 + (size - 2) * sideFactor] = 0;
      toLog(s, 4);
    }
  }
  // get code
  if (result)
  {
    *codeLng = 0;
    *intCode = 0;
    for (i = 0; i < (size - 2); i++)
      for (j = 0; j < (size - 2); j++)
      { // get code string
        if (*codeLng >= maxCodeLng)
          break;
        if ((j >= 2) or (i > 0))
        { // skip first two index blocks    code[0] code[1] code[2] code[4]
          code[*codeLng] = getCellCode(bCode, size, sideFactor, i, j);
          // assuming 2x2 code
          if (*codeLng < 8)
            // only use first 8 code values
            *intCode = (*intCode << 4) + (code[*codeLng] & 0xf);
          (*codeLng)++;
        }
      }
  }
  //
  if (result and debug)
  { // show code
    snprintf(s, MSL, "Length %2d Code:   ", *codeLng);
    for (i = 0; i < *codeLng; i++)
      snprintf(&s[i + 16], MSL - i - 16, "%x", code[i]);
    toLog(s, 3);
  }
  //
  return result;
}
///////////////////////////////////////////////////

int UCalibrationComponents::DoFindRowLength4(int candidate,
                    int * bestDirection,
                    int size)
{
  float dx, dy, dd;
  int i;
  int bestCnt = 0; // number of hops
  UCalibrationComponent * ht;
  UCalibXYStat * st;
  //
  ht = &hits[candidate];
  // debug
  /* if ((absf(ht->x - 44) < 3) and
      (absf(ht->y -  74) < 3))
  printf("debug here 1\n"); */
  // debug end
  // debug
  /* if ((absf(ht->x - 47) < 3) and
       (absf(ht->y -  91) < 3))
    printf("debug here 2\n");*/
  // debug end
  for (i = 0; i < 8; i++)
  {
    st = &ht->NSEW[i];
    if ((st->id >= 0) and (st->id < MAX_CALIBRATION_COMPONENTS))
    {
      dx = st->x;
      dy = st->y;
      dd = st->d;
      st->vx = dx;
      st->vy = dy;
      st->n = 1;
      findNextInThisDirection4(st->id, dx, dy, dd,
                               &st->endId, &st->vx,
                               &st->vy, &st->n,
                               NULL, size);
      if (st->n >= size)
      { // count number of directions of right length
        bestCnt++;
        // save one of good directions
        *bestDirection = i;
      }
    }
    else
      break;
  }
  return bestCnt;
}

///////////////////////////////////////////////////

void UCalibrationComponents::findNextInThisDirection4(
                  int id,   // candidate
                  float dx, // this far in x direction
                  float dy, // and this far in y direction
                  float dd, // at this distance
                  int * ID, // id of last corner in row
                  float * vx, // add x-dist here if found
                  float * vy, // add y-dist here if found
                  int   *  n, // increase this count if found
                  int * next, // if != NULL, then save route
                  int stopSize) // stop when n == stopSize
{
  int i;
  UCalibrationComponent * ht;
  UCalibXYStat * st;
  float lm = dd / 2.7; // reject distance from expected position
  //float lm = dd / 2.1; // reject distance from expected position
  float d;
  int best = -1;
  float bestD = sqr(lm);
  //float kOld = 0.3;
  float kOld = 0.5;
  float kNew = 1 - kOld;
  //
  ht = &hits[id];
  //
  for (i = 0; i < 8; i++)
  { // all candidates are in the first slots of NSEW
    // regardless of direction in increased distance order
    st = &ht->NSEW[i];
    if (st->id < 0)
      // stop if no more candidates
      break;
    // is distance too far - then stop now
    d = st->d - dd;
    if (d < lm)
    { // not too far away
      // get squared distance from expected position
      d = sqr(dx - st->x) + sqr(dy - st->y);
      // find best candidate
      if (d < bestD)
      { // p.t. best candidate
        best = i;
        bestD = d;
      }
    }
  }
  // save end ID in this direction
  *ID = id;
  // test for found candidate
  if ((best >= 0) and (*n < stopSize))
  { // there is a next node
    // update distance and node count
    st = &ht->NSEW[best];
    *vx += st->x;
    *vy += st->y;
    *n  += 1;
    if (next != NULL)
      next[*n] = st->id;
    if (*n <= stopSize)
      // continue with the next
      findNextInThisDirection4(st->id,
                  dx * kOld + st->x * kNew,
                  dy * kOld + st->y * kNew,
                  dd * kOld + st->d * kNew,
                  ID,
                  vx, vy, n, next, stopSize);
  }
}


///////////////////////////////////////////////////

bool UCalibrationComponents::DoFindNSEW4(int candidate, float limit, FILE * fl)
{ // calculation for square barcode
  bool result = false;
  int i, j;
  UCalibBestArray best;
  UCalibBestArrayElement * be;
  float minDist;
  int minElem;
  // debug
  if (fl != NULL)
    fprintf(fl, "*%d", candidate);
  // debug end
  // find best candidate to best array
  GetClosestItems(candidate, &best, 5.0, fl);
  minDist = 1e10;
  minElem = -1;
  for (i = 0; i < best.count; i++)
  { // get shortest distance item
    be = &best.items[i];
    if (be->d < minDist)
    {
      minElem = i;
      minDist = be->d;
    }
  }
  // debug
  if (fl != NULL)
    fprintf(fl, ", minE %d", minElem);
  // debug end
  //
  if (minElem >= 0)
  {
    result = true;
    be = &best.items[minElem];
    hits[candidate].NSEW[0].id = be->id;
    hits[candidate].NSEW[0].d = be->d;
    hits[candidate].NSEW[0].x = be->dx;
    hits[candidate].NSEW[0].y = be->dy;
    j = 1;
    for (i = 0; i < best.count; i++)
    {
      // debug
      if (fl != NULL)
        fprintf(fl, ", %d", i);
      // debug end
      if (i != minElem)
      { // add reference to the rest of the
        // best neighbors
        be = &best.items[i];
        if (be->d < limit)
        { // a major part can be within image
          // with this distance
          hits[candidate].NSEW[j].id = be->id;
          hits[candidate].NSEW[j].d = be->d;
          hits[candidate].NSEW[j].x = be->dx;
          hits[candidate].NSEW[j].y = be->dy;
          j++;
        }
        // debug
        if (fl != NULL)
          fprintf(fl, "(%5.2f)", be->d);
        // debug end
      }
      if (j == 8)
        break;
    }
  }
  // debug
  if (fl != NULL)
    fprintf(fl, "\n");
  // debug end
  return result;
} //

///////////////////////////////////////////////////

void UCalibrationComponents::DoFindNSEW(int candidate)
{ // for calibration chart
  int i;
  UCalibBestArray best;
  // find best candidate to best array - with narrow limits
  GetClosestItems(candidate, &best, 1.8);
  //
  for (i = 0; i < 8; i++)
    DoFindNSEWHead(candidate, i, &best);
  //
  DoValidateCandidates(candidate, &best);
} //

///////////////////////////////////////////////////////

void UCalibrationComponents::GetClosestItems(int candidate,
              UCalibBestArray * best,
              float distanceLimitFactor,
              FILE * /*fl*/)
{
   int i,j;
   float d;
   float dx, dy;
   // initialize array and set distance to far away
   best->count = 0;
   for (j = 0; j < MAX_BEST_ARRAY_SIZE; j++)
   {
      best->items[j].d = 1e10;
      best->items[j].id = -1;
      best->items[j].dx = 1e10;
      best->items[j].dy = 1e10;
      best->items[j]. valid = false;
   }
   // find candidates
   for (i = 0; i < hitCount; i++)
   { // test all other candidates but this one
      if (i != candidate)
      { // get distance to new candidate
        d = hits[candidate].GetMaxBlockDist(&hits[i], &dx, &dy);
        best->AddCandidate(i, d, dx, dy);
      }
   }
   // do distance variation limittatin
   if (best->count > 8)
     // limit candidate list if not within distribution
     // and calculate real distance
     best->DoLimitCandidates(distanceLimitFactor);
}

///////////////////////////////////////////////////////

void UCalibrationComponents::DoFindNSEWHead(int candidate,
                    int head, /* heading to look for candidates */
                    UCalibBestArray * best /* returned array of candidates */)
{  // - L is index to hit and candidate element
   // - head is direction to find candidate (in range 0..7)
   // - best is (12) best candidate to search from.
   // Function: associate candidates to HL.items[L] and
   //   returns this candidate
  int c,i,j,n;
  float x,y, md, d, dx, dy, rx, ry;
  UCalibrationComponent * subject;
  bool bUse;
  int  MinOfs;
  const float MaxSep = 60.0; // maksimum corner separation
  //
  //get offset to neighbor candidate
  MinOfs = roundi(best->items[0].d);
  //reset any existing relation in this direction
  subject = &hits[candidate];
  subject->NSEW[head].clear();
  // test opposite direction id
  //if (hits[candidate].NSEW[(head + 4) % 8].id == 0)
    // no opporsite component, so use default offset in direction
  // use default in all cases
  subject->GetDefaultOffset(head, MinOfs, &dx, &dy);
  // get likely position
  x =subject->x + dx;
  y =subject->y + dy;
  // now find the better candidate for this direction
  md = sqr(MaxSep);
  n = -1;
  for (c = 0; c < best->count; c++)
  {
    i = best->items[c].id;
    if ((i != candidate) and best->items[c].valid)
    {
      dx = x - hits[i].x;
      dy = y - hits[i].y;
      d  = sqr(dx) + sqr(dy);
      if (d < md)
      {
        rx = hits[i].x - subject->x;
        ry = hits[i].y - subject->y;
        bUse = subject->IsInRightQuadrant(&hits[i], head);
        if (bUse)
        { // have other directions found the same neighbor?
          for (j = 0; j < 8; j++)
          {
            if ((j != head) and
                (subject->NSEW[j].id == i) and // is same neighbor
                (subject->NSEW[j].id >= 0) and // valid
                (subject->NSEW[j].d < d))      // distance beter
            {
               bUse = false; // this item is not better off
               break;        // in j direction, keep here
            }
          }
          //
          if (bUse)
          { // best off in this direction
            md =d;
            n = i;
            subject->NSEW[head].d  = md;
            subject->NSEW[head].x  = rx;
            subject->NSEW[head].y  = ry;
            subject->NSEW[head].id = n;
          }
        }
      }
    }
  }
  // test if this it invalids another
  if (n >= 0)
    for (j = 0; j < 8; j++)
      if ((j != head) and                  // not same heading
         (subject->NSEW[j].id == n) and  // same as found above
         (subject->NSEW[j].id >= 0) and // and valid
         (subject->NSEW[j].d > md))     // and worse
      { // node in j direction is moved
        // find a new
        subject->NSEW[j].id = -1;
        DoFindNSEWHead(candidate, j, best);
      }
}

////////////////////////////////////////////////////////////////

/**
An invalid neighbor candidate is found, so remove it and try
to find a new. Used by DoValidateCandidates only. */
void UCalibrationComponents::DoRemoveAndRetry(
                     int candidate,
                     int head,
                     UCalibBestArray * best)
{
  int i,j;
  UCalibXYStat * xy;
  //
  // limit to 0..7
  i  = head % 8;
  xy = &(hits[candidate].NSEW[i]);
  for (j = 0; j < best->count; j++)
    if (best->items[j].id == xy->id)
    { // remove this candidate from usable list
      best->items[j].valid = false;
      // and find another
      DoFindNSEWHead(candidate, i, best);
      // stop looking
      break;
    }
}

/////////////////////////////////////////////////////////////
/**
*/
void UCalibrationComponents::DoValidateCandidates(int candidate,
                         UCalibBestArray * best)
{
  int i;
  float f1,f2,f0;
  UCalibrationComponent * subject = & hits[candidate];;
  UCalibXYStat * xy1, * xy2, * xy0;
  bool doBreak;
  //
  for (i = 0; i < 8; i++)
  { // check angles in clockwise direction
    doBreak = false;
    if (subject->NSEW[i].id >= 0)
    { // heading is valid, so get this and neighbors
      xy1 = &(subject->NSEW[i]);           // this
      xy2 = &(subject->NSEW[(i + 1) % 8]); // more cv neighbor
      xy0 = &(subject->NSEW[(i + 7) % 8]); // more ccv neighbor
      f1 = atan2(-xy1->y, xy1->x); // this direction in math radians
      if (xy2->id >= 0)
      { // neighbor more clockvise is valid
        // f1 should always be grater than angle to xy2, so
        f2 = atan2(-xy2->y, xy2->x) - f1;
        // limit to +/- PI
        if (f2 > M_PI) f2 -= 2 * M_PI;
        else if (f2 < -M_PI) f2 += 2 * M_PI;
        if (f2 > -0.03)
        { // f2 should be negative with some margin (2 deg)
          // is not, so remove the one who is further away
          if (xy1->d > xy2->d)
            DoRemoveAndRetry(candidate, i, best);
          else
            DoRemoveAndRetry(candidate, i+1, best);
          doBreak = true;
        }
      }
      if ((xy0->id >= 0) and not doBreak)
      { // Neighbor more ccv is valid
        f0 = atan2(-xy0->y, xy0->x) - f1;
        if   (f0 > M_PI)     f0 -= 2 * M_PI;
        else if (f0 < -M_PI) f0 += 2 * M_PI;
        if (f0 < 0.03)
        {
          // remove the one who is further away
          if (xy1->d > xy0->d)
            DoRemoveAndRetry(candidate, i, best);
          else
            DoRemoveAndRetry(candidate, i-1, best);
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////

int UCalibrationComponents::SetRowColForNeighbors(int candidate)
{ // set all empty neighbors from this
  int i, n, v;
  int vm = 0; // best vote;
  int cm = 0; // candidate with best vote
  UCalibrationComponent * subj = &hits[candidate];
  int toDo[8];
  int toDoCount = 0;
  //
  for (i = 0; i < 8; i++)
  {
    n = subj->NSEW[i].id;
    if ((n >= 0) and (n < MAX_CALIBRATION_COMPONENTS))
      if (hits[n].col < 0)
      { // no number yet, and return vote if not all neighbors are numbered
        v = SetRowColFromNeighbor(n);
        if (v > vm)
        { // candidate with best vote so far
          vm = v;
          cm = n;
        }
        if (v > 0)
          toDo[toDoCount++] = n;
      }
  }
  // now are all neighbors numbered, so
  // continue numbers with best candidate
  if (toDoCount > 0)
  {
    // there are neighbors potentially without number
    SetRowColForNeighbors(cm);
    for (i = 0; i < toDoCount; i++)
      if (toDo[i] != cm)
        SetRowColForNeighbors(toDo[i]);
  }
  // return always 0
  return 0;
}

//////////////////////////////////////////////////////////////////

int UCalibrationComponents::SetRowColFromNeighbor(int candidate)
{
  typedef struct {int row, col, votes;} Urc;
  Urc vote[8];
  int i, j, n;
  int count = 0;
  UCalibrationComponent * subj = &hits[candidate];
  bool allNumbered = true;
  // count votes
  for (i = 0; i < 8; i++)
  { // check for row/col candidates
    n = subj->NSEW[i].id;
    if (n >= 0)
    { // check if neighbor has a valid row/col
      if (hits[n].row > 0)
      { // set row/col form this
        subj->SetRc(hits[n].row, hits[n].col, (i + 4) % 8);
        // set as next valid (vill not get any votes if exist)
        vote[count].row = subj->row;
        vote[count].col = subj->col;
        vote[count].votes = 0;
        for (j = 0; j <= count; j++)
        { // increase vote for this row/col candidate
          if ((vote[j].row == subj->row) and
              (vote[j].col == subj->col))
          {
            vote[j].votes++;
            if (j == count)
              count++;
            break;
          }
        }
      }
      else
        allNumbered = false;
    }
  }
  // now count votes
  j = 0;
  for (i = 1; i < count; i++)
  {
    if (vote[i].votes == 0)
    // no more candidates
      break;
    if (vote[j].votes < vote[i].votes)
      j = i;
  }
  // now j is the best candidate, so use
  subj->row = vote[j].row;
  subj->col = vote[j].col;
  //
  if (allNumbered)
    return 0; // not a candidate for next numbering
  else
    return vote[j].votes;
}


////////////////////////////////////////////////////////////////

int  UCalibrationComponents::DoFindCenterCross(float strideL, float strideW)
{ // Find center cross and assign chart positions to detections
  // relative to center.
  // stride is spacing between markers in grid chart.
  // strideL is on the long dimention of chart
  // strideW is on the short (width) dimention of chart
  int  i, j;
  int minR, minC;
  int d; // score
  int p, rr, cc;
  int p66, rr66 = 0, cc66 = 0; // using 6x6 mask
  int p55p, rr55p = 0, cc55p = 0; // using 5x5 mask portrat
  int p55l, rr55l = 0, cc55l = 0; // using 5x5 mask landscape
  float offset; // offset from mask to center mask;
  float sX, sY; // stride in x and y (set to strideL and strideW)
#ifdef CALIB_DEBUG
  const int MSL = 250;
  char s[MSL];
#endif
  // find minimum row and col
  minR = 1024;
  minC = minR;
  for (i = 0; i < hitCount; i++)
  {
    if ((hits[i].row < minR) and (hits[i].row > 0))
      minR = hits[i].row;
    if ((hits[i].col < minC) and (hits[i].col > 0))
      minC = hits[i].col;
  }
  // empty result array
  for (i = 0; i < MAX_CALIB_GRID_SIZE; i++)
    for (j = 0; j < MAX_CALIB_GRID_SIZE; j++)
      grid[i][j] = NULL;
  // insert items in array
  for (i = 0; i < hitCount; i++)
    if (hits[i].w > 0) // only if valid candidate
    { // find ordered index numbers in 0..49 range
      rr = hits[i].row - minR;
      cc = hits[i].col - minC;
      if ((rr < 50) and  (cc < 50) and (rr >= 0) and (cc >= 0))
      {
        if (grid[rr][cc] == NULL)
          // insert into grid
          grid[rr][cc] = &(hits[i]);
        else
          // one exist already, so take the one with the best
          // neighbor vote (w)
          if (grid[rr][cc]->w < hits[i].w)
            grid[rr][cc] = &(hits[i]);
      }
    }
  // find best score for cross (stop if perfect)
  p66 = -10; // bad score (5 places wrong)
  p55p = -10;
  p55l = -10;
  d = -1;
  for (i = 0; (i < 44) and (d < 0); i++)
  { // for all row with space for 6x6 cross
    for (j = 0; (j < 44) and (d < 0); j++)
    { // for all columns with space for 6x6 cross
      // get score with 6x6 mask with four 2x2 wholes
      d = CenterCrossScore66(i, j);
      if (d > p66)
      { // better score
        p66 = d;  rr66 = i;  cc66 = j;
      }
      if (d < 0) // test for perfect
      { // get score with 5x5 mask with
        // two 2x2 wholes (top right, bottom left)
        d = CenterCrossScore55(i, j, false);
        if (d > p55p)
        { // better score
          p55p = d; rr55p = i; cc55p = j;
        }
      }
      if (d < 0) // test for perfect
      { // get score with 6x6 mask with
        // two 2x2 wholes (top left, bottom right)
        d = CenterCrossScore55(i, j, true);
        if (d > p55l)
        { // better score
          p55l = d; rr55l = i;  cc55l = j;
        }
      }
    }
  }
  // collect result
  if ((p66 > p55p) and (p66 > p55l))
  {  // best mask is 6x6 with 4 wholes
    p = p66; rr = rr66; cc = cc66;
    maskN = 0;
    offset = -0.01; // ofset to chart centre in meter from centre grid
    centR = rr + 2; centC = cc + 3; // (should be 1,1 cm from center)
    sX = strideL; // not detected, but assumed landscape
    sY = strideW;
  }
  else if (p55p > p55l)
  { // best mask is 5x5 in portrait format
    p = p55p; rr = rr55p; cc = cc55p;
    maskN = 1;
    offset = 0.0; // ofset to chart centre in meter
    centR = rr + 2; centC = cc + 2; // center of chart
    sX = strideW;
    sY = strideL;
  }
  else
  { // best mask is 5x5 in landscape format
    p = p55l; rr = rr55l; cc = cc55l;
    maskN = 2;
    offset = 0.0; // ofset to chart centre in meter
    centR = rr + 2; centC = cc + 2; // center of chart cell
    sX = strideL;
    sY = strideW;
  }
  //
#ifdef CALIB_DEBUG
  snprintf(s, MSL, "CenterCross(%d): 6x6: %d, 5x5: %d, 5x5i, %d (max is 0)",
                    maskN, p66, p55p, p55l);
  toLog(s, 5);
#endif
  // if a cross is found (p > -5) (p = 0 is all corners right)
  if (p > -9)
  { // assign chart position in meter to candidates corners
    // (rc,cc) is in top-left of (6x6) cross mask
    // make xenter (0,0) m and assign position
    // to valid positions in grid using stride spacing
    d = 0;
    for (i = 0; i < MAX_CALIB_GRID_SIZE; i++)
      for (j = 0; j < MAX_CALIB_GRID_SIZE; j++)
        if (grid[i][j] != NULL)
        { // i is row (or up-down: -y), j is column (left-right: x)
          grid[i][j]->ry = -(float(i - centR)) * sY + offset;
          grid[i][j]->rx = (float(j - centC)) * sX + offset;
          d++;
        }
  }
  else
    d = -1;
  return d;
}

/////////////////////////////////////////////////////////////////

int UCalibrationComponents::CenterCrossScore66(int row, int col)
{ // mask: 1 = expect hit, 0 expect NULL
  int mask[6][6] = {{ 1, 1, 0, 0, 1, 1},
                    { 1, 1, 0, 0, 1, 1},
                    { 0, 0, 1, 1, 0, 0},
                    { 0, 0, 1, 1, 0, 0},
                    { 1, 1, 0, 0, 1, 1},
                    { 1, 1, 0, 0, 1, 1}};
  int result = 0;
  int r,c;
  //
  for (r = 0; r < 6; r++)
    for (c = 0; c < 6; c++)
    {
      if (((mask[r][c] == 1) and (grid[r + row][c + col] != NULL)) or
          ((mask[r][c] == 0) and (grid[r + row][c + col] == NULL)))
        result++;
      else
        result--;
    }
  return result - 36;
}

/////////////////////////////////////////////////////////////////

int UCalibrationComponents::CenterCrossScore55(int row, int col, bool inverse)
{ // mask: 1 = expect hit, 0 expect NULL
  // inverse has missing corners top-left and bottom right
  char mask[5][5] = {{ 1, 1, 1, 0, 0},
                     { 1, 1, 1, 0, 0},
                     { 1, 1, 1, 1, 1},
                     { 0, 0, 1, 1, 1},
                     { 0, 0, 1, 1, 1}};
  int result = 0;
  int r,c;
  //
  if (inverse)
    for (r = 0; r < 5; r++)
      for (c = 0; c < 5; c++)
      {
        if (((mask[4 - r][c] == 1) and (grid[r + row][c + col] != NULL)) or
            ((mask[4 - r][c] == 0) and (grid[r + row][c + col] == NULL)))
          result++;
        else
          result--;
      }
  else
    for (r = 0; r < 5; r++)
      for (c = 0; c < 5; c++)
      {
        if (((mask[r][c] == 1) and (grid[r + row][c + col] != NULL)) or
            ((mask[r][c] == 0) and (grid[r + row][c + col] == NULL)))
          result++;
        else
          result--;
      }
  return result - 25;
}

//////////////////////////////////////////////////////////////

int UCalibrationComponents::FindGridLimits()
{ // store result in maxR, maxC
  int err = 0;
  int r, c;
  bool last = false;
  //
  maxR = -1;
  maxC = -1;
  for (r = 0; r < MAX_CALIB_GRID_SIZE and not last; r++)
  {
    for (c = 0; c < MAX_CALIB_GRID_SIZE and not last; c++)
    {
      if (grid[r][c] != NULL)
      { // an element is not empty
        maxR = r;
        if (c > maxC) maxC = c;
      }
    }
    // if all were empty, then last row and col is found
    last = (maxR < r);
  }
  // empty grid?
  if (maxR * maxC < 0)
    err = -1;
  return err;
}

//////////////////////////////////////////////////////////////

int UCalibrationComponents::SaveCalibPointsSquare(char * filename)
{
  int err = 0;
  int r, c;
  FILE * pkt;
  //
  FindGridLimits();
  // empty grid?
  if (maxR * maxC < 0)
    err = -1;
  // now print to file
  if (err == 0)
  {
    pkt = fopen(filename, "w");
    if (pkt != NULL)
    {
      fprintf(pkt, "# Grid coordinates found in image\n");
      fprintf(pkt, "# filename: %s\n", filename);
      fprintf(pkt, "# %d rows and %d columns\n", maxR+1, maxC+1);
      for (r = 0; r <= maxR; r++)
      {
        fprintf(pkt, "R%2d:", r);
        for (c = 0; c <= maxC; c++)
        {
          if (grid[r][c] == NULL)
            fprintf(pkt, " --,--");
          else
            fprintf(pkt, " %2d,%2d", grid[r][c]->row, grid[r][c]->col);
        }
        fprintf(pkt,"\n");
      }
      fclose(pkt);
    }
    else
      err = -1;
  }
  return err;
}

//////////////////////////////////////////////////////////////

int UCalibrationComponents::SaveCalibCoordinates(char * filename)
{
  int err = 0;
  int r, c;
  bool first = true;
  UCalibrationComponent * subj;
  FILE * pkt;
  //
  FindGridLimits();
  // empty grid?
  if (maxR * maxC < 0)
    err = -1;
  // now print to file
  if (err == 0)
  {
    pkt = fopen(filename, "w");
    if (pkt != NULL)
    {
      fprintf(pkt, "%% Grid coordinates found in image\n");
      fprintf(pkt, "%% filename: %s\n", filename);
      fprintf(pkt, "%% %d rows and %d columns\n", maxR+1, maxC+1);
      fprintf(pkt, "p = [ ...\n");
      for (r = 0; r <= maxR; r++)
      {
        for (c = 0; c <= maxC; c++)
        {
          subj = grid[r][c];
          if (subj != NULL)
          {
            if (first)
            { // first line (no ;)
              fprintf(pkt, " %f,%f, 0.0, %f, %f, 0.0, 0.0, 1.0 ...\n",
                  subj->rx, subj->ry, subj->x, subj->y);
              first = false;
            }
            else
              // other lines
              fprintf(pkt, "; %f,%f, 0.0, %f, %f, 0.0, 0.0, 1.0 ...\n",
                  subj->rx, subj->ry, subj->x, subj->y);
          }
        }
      }
      // last line
      fprintf(pkt, "];\n");
      fclose(pkt);
    }
    else
      err = -1;
  }
  return err;
}

////////////////////////////////////////////////

int UCalibrationComponents::SaveAllHits(char * filename)
{
  int i;
  UCalibrationComponent * cp;
  FILE * fp = NULL;
  //
  fp = fopen(filename, "w");
  if (fp != NULL)
  {
    fprintf(fp, "%% save of all corners in image %d\n", hitCount);

    printf("UCalibrationComponents::SaveAllHits - hertil not finished\n");

    for (i = 0; i < hitCount; i++)
    {
      cp = &hits[i];
      if (cp != NULL)
      {
        ; // @todo UCalibrationComponents::SaveAllHits(char * filename) - is not completed
      }
    }
    fclose(fp);
  }
  return hitCount;
}


////////////////////////////////////////////////

UCalibrationMark UCalibrationComponents::FindImagePosition(int sm, int sn)
{ // get pixel position of top-left of code cell.
  UCalibrationMark result;
  //
  result.row = sm;
  result.col = sn;
  result.ix = mCode[sm][sn].px;
  result.iy = mCode[sm][sn].py;
  // w, rx and ry not set.
  return result;
}

///////////////////////////////////////////////////////


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
//  UCalibrationMark

/**
Constructor */
UCalibrationMark::UCalibrationMark()
{
  valid = false;
}

////////////////////////////////////////////////////////////

  /**
  Copy entry from a UCalibrationComponent */
void UCalibrationMark::copy(UCalibrationComponent * source)
{ // copy useable information from source
  ix = source->x; // image x coordinate
  iy = source->y;
  rx = source->rx; // chart position
  ry = source->ry;
  row = source->row;
  col = source->col;
  valid = true;
}

////////////////////////////////////////////////////////////


void UCalibrationMark::setMark(int frameRow, int frameCol,
                               float imageX, float imageY,
                               float realX, float realY)
{
  ix = imageX;
  iy = imageY;
  rx = realX;
  ry = realY;
  row = frameRow;
  col = frameCol;
  valid = true;
}


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
//  UCalibrationMarkSet

/**
  Constructor */
UCalibrationMarkSet::UCalibrationMarkSet()
{
  valid = false;
  markCnt = 0;
  barcode = 0;
}

////////////////////////////////////////////////////////////

void UCalibrationMarkSet::clear()
{
  markCnt = 0;
  barcode = 0;
  valid = false;
  calValid = false;
  implemented = false;
  // stop iteration if error below ~ 3 cm
  errDistStopLimit = 0.03;
  // default 0.04 degree (~ 0.0007 rad)
  errRotStopLimit = 0.0007;
}

////////////////////////////////////////////////////////////

bool UCalibrationMarkSet::setFramePositions(int fw,  // inner frame width
                 float stride, char order[], char fwd[],
                 float AX[], float AY[], // A
                 float BX[], float BY[], // B
                 float CX[], float CY[], // C
                 float DX[], float DY[] // D
                            )
{
  int i, n, k;
  int r, c;      // row and column
  float rx, ry;  // real position
  float tr = float(fw) * stride / 2.0; // from center to frame
  float * kx, * ky;
  bool forward;
  bool result;
  //
  result = (((fw - 1) * 4) <= MAX_FRAME_POSITIONS);
  n = 0;
  if (result)
  {
    for (k = 0; k < 4; k++)
    {
      forward = (fwd[k] == 't');
      switch (order[k])
      {
        case 'A': kx = AX;
                  ky = AY;
                  break;
        case 'B': kx = BX;
                  ky = BY;
                  break;
        case 'C': kx = CX;
                  ky = CY;
                  break;
        case 'D': kx = DX;
                  ky = DY;
                  break;
        default:
          kx = NULL;
          ky = NULL;
          result = false;
      }
      if (result)
      { // save lements
        for (i = 0; i < fw; i++) // frame 7 => fw=5 => 6 points each side (0 to 5)
        { // find column position
          switch (k)
          { // traversing clockwise
            case 0: r = 0;      c = i;      break; // top
            case 1: r = i;      c = fw;     break; // right
            case 2: r = fw;     c = fw - i; break; // bottom
            case 3: r = fw - i; c = 0;      break; // left
            default: r = 0; c = 0; break;
          }
          rx = -tr + c * stride; // real x distance from center of barcode chart
          ry =  tr - r * stride; // real y distance from center
          if (forward)
            mark[n].setMark(r, c, kx[i], ky[i], rx, ry);
          else
            mark[n].setMark(r, c, kx[fw - i], ky[fw - i], rx, ry);
          n++;
        }
      }
    }
  }
  markCnt = n;
  valid = true;
  return result;
}

////////////////////////////////////////////////////////////

bool UCalibrationMarkSet::evaluatePosRot(
                    const char * params,
                    UCamPar * camp,
                    UPosition * oldPos,
                    URotation * oldRot,
                    UPosition * chartP,
                    bool makeLogfile)
{ // Calculate new values for parameters in param
  // and returns result in posCal and rotCal.
  // Params may contain any of 6 characters: "xyzOPK"
  // for x, y, z, Omega, Phi, Kappa. at least 1 and max 6.
  // Expects chart position in 'chartPos'
  //
  // Outset values are taken from present camera position/rotation
  bool result;
  //
  UMatrixBig B(40, 6, 0.0); // equatin parameters for 20 samples
  UMatrixBig K(40, 1, 0.0); // 20 samples with default position/roation
  UMatrix4 R(6, 1, 0.0); // result vector [Kappa, Omega, Phi, x, y, z]
  double hx, hy, camC;  // camera constants
  UCalibrationMark * pkt; // calibration point with one pixel and chart position
  UPosition dPos; // result change in position
  URotation dRot; // result change in rotation
  UPosition posOld; // old position before calculation loop
  URotation rotOld; // old rotation before calculaton loop
  double errDistOld = 5.00; // only try to correct errors less than (meter)
  UPosition errRotPos; // used for error distance in radians
  double errRotOld = 1.57; // only try correct errors less than Pi/2
  const char paramStr[] = "xyzOPK"; // valid parameter list
  int i, n, paramCnt = 0;
  int eqSets;     // equation set count (2 equations each set)
  int useMark;    // distance between mused marks;
  UPosition posE; // postion for error calculation
  URotation rotE; // rotation for error calculation
  bool finished;
  //
#ifdef CALIB_DEBUG
  const int MSL = 250;
  char s[MSL];
  FILE * f = NULL;
#endif
  //
  result = (valid and markCnt >= 8);
  if (result)
  {
    // find number of parameters
    for (i = 0; i < int(strlen(paramStr)); i++)
    {
      if (strchr(params, paramStr[i]) != NULL)
        paramCnt++;
    }
    // equation count for these purposes
    // marks for barchart are organized from top-left clockwise
    // e.g. for 7x7 frame, that makes 6x6 corners:
    //     0  1  2  3  4  5
    //    19              6
    //    18              7
    //    17              8
    //    16              9
    //    15 14 13 12 11 10
    // use only the four corners (here 0, 5, 10, 15)
    eqSets = 4;            // 4 sets makes 8 equations
    useMark = markCnt / 4; // count between corners
    // adjust size of array and vectors
    B.setSize(eqSets * 2, paramCnt); // liniarized matrix
    R.setSize(paramCnt, 1);          // parameters to find
    K.setSize(eqSets * 2, 1);        // value at reference point
    //
    // Get camera values constants
    hx = camp->getHx(); // center point
    hy = camp->getHy();
    camC = camp->getFocalLength(); // focus length in pixels
    // get initial estimate
    // get last known camera position rotation
    posOld = *oldPos;
    rotOld = *oldRot;
    // set position and rotation for calibration chart
    // for calculating pixel error only
    posE = *chartP;
    rotE.set(0.0, 0.0, 0.0);
    //
    errDist = 0.0;
    errRot = 0.0;
    loops = 0;

#ifdef CALIB_DEBUG
    if (makeLogfile)
    {
      snprintf(s, MSL,
                  "Finding camera position and rotation params '%s' from:", params);
      toLog(s, 7);
      posOld.sprint(s, "old Pos:");
      toLog(s, 7);
      rotOld.sprint(s, "old Rot:");
      toLog(s, 7);
      snprintf(s, MSL, "%scalibConv.m", dataPath);
      f = fopen(s, "w");
      if (f != NULL)
      {
        fprintf(f, "Finding camera position and rotation params '%s' from:\n", params);
        fprintf(f, "Calibration iterations: \n");
        fprintf(f, "loop, x,y,x,Omega,Phi,Kappa,ErrDist, ErrRot, ErrPixels for '%s'\n",
                params);
        fprintf(f, "%d %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f\n",
            loops, posOld.x, posOld.y, posOld.z,
            rotOld.Omega, rotOld.Phi, rotOld.Kappa,
            errDist, errRot);
      }
      else
        toLog("**** UCalibrationMarkSet::evaluatePosRot Open logfile failed", 2);
    }
#endif
    //
    implemented = false;
    calValid = false;
    finished = false;
    do
    {  // repeat until error is small
      loops++;
      //
      n = 0;
      for (i = 0; i < eqSets; i ++)
      { // get pointset
        pkt = &mark[n];
        // fill into matrix at row i and i + 1
        result = setCameraPosRowSet(params, &B, &K, i * 2,
                  hx, hy, camC,
                  &posOld, &rotOld,
                  pkt, chartP);
        n += useMark;
      }
      if (result)
      { // Now B and K is ready to least square calculation
        // in solving the equation B * R = K, where
        // R is the resulting change in parameters.
        // Solved by: R = (B' * B)^(-1) * B' * K
        result = B.solve(&K, &R);
      }
      if (result)
      { // get parameter changes to dRot, dPos
        // parameters may be missing, but always in this order
        paramCnt = 0;
        if (strchr(params, 'O') != NULL)
          dRot.Omega = R.get(paramCnt++);
        if (strchr(params, 'P') != NULL)
          dRot.Phi =   R.get(paramCnt++);
        if (strchr(params, 'K') != NULL)
          dRot.Kappa = R.get(paramCnt++) ;
        if (strchr(params, 'x') != NULL)
          dPos.x =     R.get(paramCnt++);
        if (strchr(params, 'y') != NULL)
          dPos.y =     R.get(paramCnt++);
        if (strchr(params, 'z') != NULL)
          dPos.z =     R.get(paramCnt++);
        // calculate parameter variations for distance and rotation
        errDist = dPos.dist();
        errRotPos.copy(&dRot);
        errRot = errRotPos.dist();
        //
        // reduce gain in some situations
        // As x and Phi are tightly correlated
        // so implement less if both estimated
        if ((strchr(params, 'x') != NULL) and
            (strchr(params, 'P') != NULL))
        { // reduce values for improved stability
          dPos.x *= 0.5;
          dRot.Phi  *= 0.5;
        }
        // As y and Omega are tightly correlated
        // so implement less if both estimated
        if ((strchr(params, 'y') != NULL) and
            (strchr(params, 'O') != NULL))
        { // reduce values for improved stability
          dPos.y *= 0.5;
          dRot.Omega  *= 0.5;
        }
        // adjust parameters with found values
        posOld.subtract(&dPos);
        rotOld.subtract(&dRot);
        //
        // get average error with this estimate
        errEpix  = geterrorInPixels(&posE, &rotE,
             &posOld, &rotOld, useMark, camp, NULL, false);
#ifdef CALIB_DEBUG
        if (f != NULL)
        {
          fprintf(f, "%d %9.4f %9.4f %9.4f " // pos
                        "%9.4f %9.4f %9.4f " // rot
                        "%9.5f %9.5f %9.4f\n", // err
              loops, posOld.x, posOld.y, posOld.z,
              rotOld.Omega, rotOld.Phi, rotOld.Kappa,
              errDist, errRot, errEpix);
        }
#endif
        //
        if ((errDist > (errDistOld + 0.15)) or
           (errRot > (errRotOld + 0.08 /*5deg*/)))
          // calculation do not converge
          result = false;
        else
        {
          errDistOld = errDist;
          errRotOld = errRot;
        }
        if (true)
          finished = (errDist < errDistStopLimit) and
                     (errRot < errRotStopLimit);
        else
        { // unreasonable requirement?
          finished = errEpix < 0.0005;
          // debug
#ifdef CALIB_DEBUG
          snprintf(s, MSL,
                   "Pixel error %3.4f (%d loops)", errEpix, loops);
          toLog(s, 4);
#endif
          // end debug
        }
      }
    } while (result                          // stop if unstable
             and (not finished)
             and loops < 50);                  // stop if too many loops
    //
    //  iteration loop finished
    //
    if (result)
    { // implement only if converge (many loops is OK too)
      // save camera orientation result
      posCal = posOld;
      rotCal = rotOld;
      calValid = true;
      implemented = false;
    }
    //
#ifdef CALIB_DEBUG
    if (result)
    { // show final result
      snprintf(s, MSL, "Succeded in %d loops", loops);
      toLog(s, 7);
      posOld.sprint(s, "Pos:");
      toLog(s, 7);
      rotOld.sprint(s, "Rot:");
      toLog(s, 7);
    }
    else
    {
      snprintf(s, MSL, "Failed after %d loops", loops);
      toLog(s, 7);
      if (f != NULL)
        fprintf(f, "failed\n");
    }
    if (f != NULL)
      fclose(f);
#endif
  }
  else
    result = false;
  return result;
}

//////////////////////////////////////////////////////////

bool UCalibrationMarkSet::evaluateChartPosRot(
                    UPosition * initialChartPos,
                    URotation * initialChartRot,
                    double stopLimDist,
                    double stopLimRot,
                    UCamPar * camp,
                    bool adjRadial,
                    bool makeLogfile)
{
  FILE * f = NULL;
  bool result;

#ifdef CALIB_DEBUG
  const int NUM_STEPS = 20;
  const int MSL = 250;
  char s[MSL];
  UTime t;
  UPosition posOld;
  URotation rotOld;
  UPosition posE;
  URotation rotE;
  double errPix;
  double minD, stepD;
  int i, j;
  //
  t.Now();
  // save used pixel size
  pixSize = camp->getPixelSize();
  // show filename
  if (makeLogfile)
  {
    snprintf(s, MSL, "barcode position logfile '%sB%lx_%02d%02d%02d.m'",
                 dataPath, barcode, t.GetHour(), t.GetMin(), t.GetJustSec());
    toLog(s, 7, &t);
    // create logfile
    snprintf(s, MSL, "%sB%lx_%02d%02d%02d.m",
                 dataPath, barcode, t.GetHour(), t.GetMin(), t.GetJustSec());
    f = fopen(s, "w");
  }
  if (f != NULL)
  {
    if (initialChartPos != NULL)
      posOld = * initialChartPos;
    if (initialChartRot != NULL)
      rotOld = * initialChartRot;
    fprintf(f, "%% Finding barcode position and rotation\n");
    fprintf(f, "%% filename %s\n", s);
    fprintf(f, "%% Calibration iterations: \n");
    fprintf(f, "%% loop, x,y,x,Omega,Phi,Kappa,ErrDist, ErrRot, ErrPixels\n");
    fprintf(f, "%% %d %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f\n",
        0, posOld.x, posOld.y, posOld.z,
        rotOld.Omega, rotOld.Phi, rotOld.Kappa,
        0.0, 0.0);
  }
  else if (makeLogfile)
    toLog("**** UCalibrationMarkSet::evaluatePosRot Open logfile failed", 3);
#endif
  //
  // The actual parameter estimation
  result = evaluateChartPosRot(initialChartPos,
                    initialChartRot,
                    stopLimDist,
                    stopLimRot,
                    camp,
                    adjRadial,
                    f);
  //
#ifdef CALIB_DEBUG
  // make carpet in matlab to
  // show local minima in area around estimated position
  // get average error with this estimate
  if (f != NULL)
  {
    rotOld = rotBarcode;
    posOld = posBarcode;
    if (absd(rotOld.Omega) > absd(rotOld.Phi))
      minD = absd(rotOld.Omega) * 2.0;
    else
      minD = absd(rotOld.Phi) * 2.0;
    stepD = minD / double(NUM_STEPS);
    fprintf(f, "%% Make array with error values and array with axis values\n");
    fprintf(f, "lmPhi = %f:%f:%f;\n",
           -minD * 180.0/M_PI, stepD * 180.0/M_PI, minD * 180.0/M_PI * 1.0001);
    fprintf(f, "lmOmega = %f:%f:%f;\n",
           -minD * 180.0/M_PI, stepD * 180.0/M_PI, minD * 180.0/M_PI * 1.0001);
    fprintf(f, "lm = zeros(41,41);\n");
    for (i = -NUM_STEPS; i <= NUM_STEPS; i++)
      for (j = -NUM_STEPS; j <= NUM_STEPS; j++)
      {
        //rotOld.Omega = stepD * double(i);
        rotOld.Omega = stepD * double(i);
        rotOld.Phi =   stepD * double(j);
        errPix  = geterrorInPixels(&posOld, &rotOld,
                   &posE, &rotE, markCnt / 4, camp, NULL, adjRadial);
        fprintf(f, "lm(%d, %d) = %f;\n", j+NUM_STEPS+1, i+NUM_STEPS+1, mind(7.0, errPix));
      }
    fprintf(f, "%% plot the mesh\n");
    fprintf(f, "figure\n");
    fprintf(f, "meshc(lmOmega, lmPhi, lm);\n");
    fprintf(f, "hold on\n");
    minD *= (180.0 / M_PI);
    fprintf(f, "axis([%f, %f, %f, %f])\n",
            -minD, minD, -minD, minD);
    fprintf(f, "plot3(OPEpp(:,1), OPEpp(:,2), OPEpp(:,3), 'kd-');\n");
    fprintf(f, "plot3(OPEpm(:,1), OPEpm(:,2), OPEpm(:,3), 'ko-');\n");
    fprintf(f, "plot3(OPEmm(:,1), OPEmm(:,2), OPEmm(:,3), 'k*-');\n");
    fprintf(f, "plot3(OPEmp(:,1), OPEmp(:,2), OPEmp(:,3), 'kx-');\n");
    fprintf(f, "plot3(OPEmain(:,1), OPEmain(:,2), OPEmain(:,3), 'k+-');\n");
    fprintf(f, "xlabel('\\Phi (tilt)')\n");
    fprintf(f, "ylabel('\\kappa (panorate)')\n");
    fprintf(f, "zlabel('pixel error')\n");
/*    fprintf(f, "title('Guidemark %lu at estimated "
                   "(x,y,z) =(%.2f,%.2f,%.2f) m, (\\omega,\\phi,\\kappa)= "
                   "(%.1f,%.1f,%.1f)');\n",
                   barcode, posBarcode.x,
                   posBarcode.y, posBarcode.z,
                   rotBarcode.Omega * 180.0 / PI, rotBarcode.Phi * 180.0 / PI,
                   rotBarcode.Kappa * 180.0 / PI);*/
    fprintf(f, "view(-80.0, 20.0);");
    snprintf(s, MSL, "B%lu_%02d%02d%02d",
                      barcode, t.GetHour(), t.GetMin(), t.GetJustSec());
    fprintf(f, "print -depsc %s\n", s);
    fprintf(f, "print -dpng -r75 %s\n", s);
    fprintf(f, "%% finished\n");
    fclose(f);
  }
#endif
  return result;
}

//////////////////////////////////////////////////////////

bool UCalibrationMarkSet::evaluateChartPosRot(
                    UPosition * initialChartPos,
                    URotation * initialChartRot,
                    double stopLimDist,
                    double stopLimRot,
                    UCamPar * camp,
                    bool adjRadial,
                    FILE * f)
{ // Calculate new values for parameters in param
  // and returns result in posBarcode and rotBarcode.
  // Exected position and rotation may be specified.
  bool result;
  //
  UMatrixBig B(8, 6, 0.0); // equatin parameters for 4 corners
  UMatrixBig K(8, 1, 0.0); // 4 corners is 8 equations
  UMatrix4 R(6, 1, 0.0); // result vector [Kappa, Omega, Phi, x, y, z]
  //double hx, hy, camC;  // camera constants
  UCalibrationMark * pkt; // calibration point with one pixel and chart position
  UPosition dPos; // result change in position
  URotation dRot; // result change in rotation
  UPosition posOld; // old position before calculation loop
  URotation rotOld; // old rotation before calculaton loop
  double errDistOld = 5.00; // only try to correct errors less than (meter)
  UPosition errRotPos; // used for error distance in radians
  double errRotOld = 1.57; // only try correct errors less than Pi/2
  int i, n;
  int eqSets;     // equation set count (2 equations each set)
  int useMark;    // distance between mused marks;
  UPosition posE; // postion for error calculation
  URotation rotE; // rotation for error calculation
  bool finished;
  double bestPixErr = 500.0; // pixels
  UPosition testPos; // test position for elimination of local minima
  URotation testRot; // test rotation for elimination of local minima
  const double testAngle = 42.0 * M_PI / 180.0; // 10 degrees
#ifdef CALIB_DEBUG
  const int MSL = 250;
  char s[MSL];
#endif
  //
  result = (valid and markCnt >= 8);
  if (result)
  { // marks for barchart are organized from top-left clockwise
    // e.g. for 7x7 frame, that makes 6x6 corners:
    //     0  1  2  3  4  5
    //    19              6
    //    18              7
    //    17              8
    //    16              9
    //    15 14 13 12 11 10
    // use only the four corners (here 0, 5, 10, 15)
    eqSets = 4;            // 4 sets makes 8 equations
    useMark = markCnt / 4; // count between corners
    // adjust size of array and vectors
    B.setSize(eqSets * 2, 6); // liniarized matrix
    R.setSize(6, 1);             // parameters to find
    K.setSize(eqSets * 2, 1);    // value at reference point
    //
    // Get camera values constants
//     hx = camp->getHx(); // center point
//     hy = camp->getHy();
//    camC = camp->getFocalLength(); // focus length in pixels
    // get initial estimate
    // set initial guess for barcode chart position
    if (initialChartPos != NULL)
      posOld = *initialChartPos;
    else
      posOld.set(0.0, 0.0, 0.0);
    if (initialChartRot != NULL)
      rotOld = *initialChartRot;
    else
      rotOld.set(0.0, 0.0, 0.0);
    // set position and rotation for camera
    // for calculating pixel error only
    posE.set(0.0, 0.0, 0.0);
    rotE.set(0.0, 0.0, 0.0);
    //
    if (initialChartRot == NULL)
    { // no knowledge of initial barcode orientation, so try all quadrants
      if (f != NULL)
        toLog("Now testing possible directions", 4);
      // try with initial positive Omega and Phi
      testRot.set(+testAngle, +testAngle, 0.0);
      testPos = posOld;
      if (f != NULL)
        fprintf(f, "OPEpp = [%f, %f, %f ...\n",
                  testRot.Omega * 180.0/M_PI, testRot.Phi * 180.0/M_PI, 5.0);
      if (evaluateChartPosRot(&testPos, &testRot,
                      stopLimDist * 20.0, stopLimRot * 20.0,
                      camp, adjRadial, f))
        if (errEpix < bestPixErr)
        { // better solution
          rotOld = rotBarcode;
          posOld = posBarcode;
          bestPixErr = errEpix;
        }
      if (f != NULL)
      {
        fprintf(f, "];\n");
        fprintf(f, "%% Result of +Om +Ph: %9.3f pixels average error\n", errEpix);
      }
      // try with initial positive Omega and negative Phi
      testRot.set(+testAngle, -testAngle, 0.0);
      if (f != NULL)
        fprintf(f, "OPEpm = [%f, %f, %f ...\n",
                 testRot.Omega * 180.0/M_PI, testRot.Phi * 180.0/M_PI, 5.0);
      if (evaluateChartPosRot(&testPos, &testRot,
                     stopLimDist * 20.0, stopLimRot * 20.0,
                     camp, adjRadial, f))
        if (errEpix < bestPixErr)
        { // better solution
          rotOld = rotBarcode;
          posOld = posBarcode;
          bestPixErr = errEpix;
        }
      if (f != NULL)
      {
        fprintf(f, "];\n");
        fprintf(f, "%% Result of +Om -Ph: %9.3f pixels average error\n", errEpix);
      }
      // try with initial negative Omega and negative Phi
      testRot.set(-testAngle, -testAngle, 0.0);
      if (f != NULL)
        fprintf(f, "OPEmm = [%f, %f, %f ...\n",
                testRot.Omega * 180.0/M_PI, testRot.Phi * 180.0/M_PI, 5.0);
      if (evaluateChartPosRot(&testPos, &testRot,
                    stopLimDist * 20.0, stopLimRot * 20.0,
                    camp, adjRadial, f))
        if (errEpix < bestPixErr)
        { // better solution
          rotOld = rotBarcode;
          posOld = posBarcode;
          bestPixErr = errEpix;
        }
      if (f != NULL)
      {
        fprintf(f, "];\n");
        fprintf(f, "%% Result of -Om -Ph: %9.3f pixels average error\n", errEpix);
      }
      // try with initial negative Omega and positiove Phi
      testRot.set(-testAngle, +testAngle, 0.0);
      if (f != NULL)
        fprintf(f, "OPEmp = [%f, %f, %f ...\n",
                   testRot.Omega * 180.0/M_PI, testRot.Phi * 180.0/M_PI, 5.0);
      if (evaluateChartPosRot(&testPos, &testRot,
                    stopLimDist * 20.0, stopLimRot * 20.0,
                    camp, adjRadial, f))
        if (errEpix < bestPixErr)
        { // better solution
          rotOld = rotBarcode;
          posOld = posBarcode;
          bestPixErr = errEpix;
        }
      if (f != NULL)
      {
        fprintf(f, "];\n");
        fprintf(f, "%% Result of -Om +Ph: %9.3f pixels average error\n", errEpix);
        fprintf(f, "%% end of testing possible minima\n");
        fprintf(f, "OPEmain = [%f, %f, %f ...\n",
               rotOld.Omega * 180.0/M_PI, rotOld.Phi * 180.0/M_PI, bestPixErr);
        toLog("End testing possible directions - now improve best result", 4);
      }
    }
    //
    errDist = 0.0;
    errRot = 0.0;
    loops = 0;
    errEpix = 0.0;
    barcodeValid = false;
    finished = false;
    //
#ifdef CALIB_DEBUG
    // show initial position
    if (f != NULL)
    { // log results before this iteration
      fprintf(f, "%% %d %9.4f %9.4f %9.4f " // pos
                    "%9.4f %9.4f %9.4f " // rot
                    "%9.5f %9.5f %9.4f\n", // err
          loops, posOld.x, posOld.y, posOld.z,
          rotOld.Omega, rotOld.Phi, rotOld.Kappa,
          errDistOld, errRotOld, errEpix);
    }
#endif
    //
    do
    {  // repeat until error is small
      loops++;
      //
      n = 0;
      for (i = 0; i < eqSets; i ++)
      { // get pointset
        pkt = &mark[n];
        if (f != NULL)
          fprintf(f, "%% point%3d rc(%2d,%2d) pix(%6.1f,%6.1f)"
                     " cht(%5.3f,%5.3f)\n",
                      i, pkt->row, pkt->col, pkt->ix,
                      pkt->iy, pkt->rx, pkt->ry);
        // fill into matrix at row i and i + 1
        result = setBarcodePosRowSet(&B, &K, i * 2,
                  camp, adjRadial,
                  &posOld, &rotOld,
                  pkt);
        n += useMark;
      }
      if (result)
      { // Now B and K is ready to least square calculation
        // in solving the equation B * R = K, where
        // R is the resulting change in parameters.
        // Solved by: R = (B' * B)^(-1) * B' * K
        //
        // debug
        //printf("UCalibrationMarkSet::evaluateChartPosRot:Solve start\n");
        // debug end
        if (f != NULL and false)
        { // save matrix B * R = K
          // i.e. R = (B' * B)^(-1) * B' * K
          B.save(f, "B");
          K.save(f, "K");
        }
        result = B.solve(&K, &R);
        if (f != NULL and false)
        { // save result of solve
          R.save(f, "R");
        }
        //result = solveLeastSquare(&B, &R, &K);
        // debug
        /*
        if (result)
          printf("UCalibrationMarkSet::evaluateChartPosRot:Solve OK\n");
        else
          printf("UCalibrationMarkSet::evaluateChartPosRot:Solve failed\n");
        */
        // debug end
      }
      if (result)
      { // get parameter changes to dRot, dPos
        // parameters may be missing, but always in this order
        dRot.Omega = R.get(0);
        dRot.Phi =   R.get(1);
        dRot.Kappa = R.get(2);
        dPos.x =     R.get(3);
        dPos.y =     R.get(4);
        dPos.z =     R.get(5);
        // calculate parameter variations for distance and rotation
        errDist = dPos.dist();
        errRotPos.copy(&dRot);
        errRot = errRotPos.dist();
        //
        // reduce gain in some situations
        dRot.scale(0.75);
        // adjust parameters with found values
        posOld.subtract(&dPos);
        rotOld.subtract(&dRot);
        //
        // get average error with this estimate
        errEpix  = geterrorInPixels(&posOld, &rotOld,
                      &posE, &rotE, useMark, camp, NULL, adjRadial);
        //errEpix = sqrt(K.sqSum());
#ifdef CALIB_DEBUG
        if (f != NULL)
        { // log results in this iteration
          fprintf(f, "%% %d %9.4f %9.4f %9.4f " // pos
                        "%9.4f %9.4f %9.4f " // rot
                        "%9.5f %9.5f %9.4f\n", // err
              loops, posOld.x, posOld.y, posOld.z,
              rotOld.Omega, rotOld.Phi, rotOld.Kappa,
              errDist, errRot, errEpix);
          fprintf(f, "; %f, %f, %f ...\n",
               rotOld.Omega * 180.0/M_PI, rotOld.Phi * 180.0/M_PI, errEpix);
        }
#endif
        //
        if ((errDist > (errDistOld + 0.15)) or
           (errRot > (errRotOld + 0.08 /*5deg*/)))
          // calculation do not converge
          result = false;
        else
        {
          errDistOld = errDist;
          errRotOld = errRot;
        }
        // loop stop
        finished = (errDist < stopLimDist) and
                   (errRot < stopLimRot);
      }
    } while (result                          // stop if unstable
             and (not finished)
             and loops < 50);                  // stop if too many loops
    //
    //  iteration loop finished
    //
    if (result)
    { // save barcode position result
      posBarcode = posOld;
      rotBarcode = rotOld;
      barcodeValid = true;
    }
    //
#ifdef CALIB_DEBUG
    if (f != NULL)
    {
      if (result)
      { // show final result
        snprintf(s, MSL, "Succeded in %d loops", loops);
        toLog(s, 7);
        posOld.sprint(s, "Barcode Pos:");
        toLog(s, 7);
        rotOld.sprint(s, "Barcode Rot:");
        toLog(s, 7);
      }
      else
      {
        snprintf(s, MSL, "Failed after %d loops", loops);
        toLog(s, 7);
        if (f != NULL)
          fprintf(f, "%% failed\n");
      }
    if ((initialChartRot == NULL) and (f != NULL))
      // terminate result array
      fprintf(f,"];\n");
    }
#endif
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////
/*
bool UCalibrationMarkSet::solveLeastSquare(
            UMatrixBig * B, UMatrixBig * R, UMatrixBig * K)
{
  bool result = false;
  UMatrixBig BB(40,40);
  UMatrixBig Bt(40,40);
  UMatrixBig mt(40,40);
  //
  Bt.transpose(B);
  BB.mult(&Bt, B);
  if (BB.err == 0)
  {
    mt.inverse(&BB);
    if (mt.err == 0)
    { // R = (B' * B)^-1 * B' * K
      BB.mult(&mt, &Bt);
      R->mult(&BB, K);
    }
    result = (R->err == 0);
  }
  return result;
}
*/
///////////////////////////////////////////////////////

bool UCalibrationMarkSet::setCameraPosRowSet(const char * params,
              UMatrixBig * B, UMatrixBig * K, int r,
              double hx, double hy, double c,
              UPosition * camPos0, URotation * camRot0,
              UCalibrationMark * pkt, UPosition * chartPos)
{
  bool result;
  // get variables in easy form
  const double sk = sin(camRot0->Kappa);
  const double ck = cos(camRot0->Kappa);
  const double sp = sin(camRot0->Phi);
  const double cp = cos(camRot0->Phi);
  const double so = sin(camRot0->Omega);
  const double co = cos(camRot0->Omega);
  const double xp = pkt->rx + chartPos->x;
  const double yp = pkt->ry + chartPos->y;
  const double zp = chartPos->z;
  const double tx = camPos0->x;
  const double ty = camPos0->y;
  const double tz = camPos0->z;
  const double ix = pkt->ix;
  const double iy = pkt->iy;
  double w, dwo, dwp, v;
  int paramCnt = 0;
  //
  result = (c > 0.1) and (r < int(K->maxSize()));
  if (result)
  { // Focus length must be >0
    // first result vector equal to equation with
    // initial parameters is K vector.
    // help expression w and dw/do
    w   =  (-co*sp*xp + so*yp - co*cp*zp
            +co*sp*tx - so*ty + co*cp*tz)/c;
    dwo = ( so*sp*xp + co*yp + so*cp*zp
           - so*sp*tx - co*ty - so*cp*tz)/c;
    dwp = co * (-cp*xp + sp*zp + cp*tx - sp*tz)/c;
    // Equation 1 with default values of [Omega, Phi, Kappa, tx, ty, tx]
    // watch K->m[r]
    v =  (ck*cp + sk*so*sp - hx*co*sp/c)*xp
        + (sk*co + hx*so/c)*yp
        + (-ck*sp + sk*so*cp - hx*co*cp/c)*zp
        - (ck*cp + sk*so*sp)*tx - sk*co*ty
        - (-ck*sp + sk*so*cp)*tz
        - hx*(-co*sp*tx + so*ty - co*cp*tz)/c
        - ix*w;
    K->setRC(r + 0, 0, v);
    // Equation 1 with default values of [Omega, Phi, Kappa, tx, ty, tx]
    // watch K->m[r+1]
    v =  (sk*cp - ck*so*sp - hy*co*sp/c)*xp
        + (-ck*co + hy*so/c)*yp
        + (-sk*sp - ck*so*cp - hy*co*cp/c)*zp
        + (-sk*cp + ck*so*sp)*tx + ck*co*ty
        + (sk*sp + ck*so*cp)*tz
        - hy*(-co*sp*tx + so*ty - co*cp*tz)/c
        - iy*w;
    K->setRC(r + 1, 0, v);
    // then the big matrix with up to 6 columns for each row
    if (strchr(params, 'O') != NULL)
    { // d(L1)/dOmega B->m[(r+0)*B->cols + 0]
      B->setRC(r + 0, paramCnt, (sk*co*sp + hx*so*sp/c)*xp
                     + (-sk*so + hx*co/c)*yp
                     + (sk*co*cp + hx*so*cp/c)*zp - sk*co*sp*tx
                     + sk*so*ty - sk*co*cp*tz
                     - hx*(so*sp*tx + co*ty + so*cp*tz)/c
                     - ix*dwo);
      // d(L2)/dOmega
      B->setRC(r + 1, paramCnt++, (-ck*co*sp + hy*so*sp/c)*xp
                     + (ck*so + hy*co/c)*yp
                     + (-ck*co*cp + hy*so*cp/c)*zp
                     + ck*co*sp*tx - ck*so*ty + ck*co*cp*tz
                     - hy*(so*sp*tx + co*ty + so*cp*tz)/c
                     - iy*dwo);
    }
    if (strchr(params, 'P') != NULL)
    { // d(L1)/dPhi
      B->setRC(r + 0, paramCnt, (-ck*sp + sk*so*cp - hx*co*cp/c)*xp
                     + (-ck*cp - sk*so*sp + hx*co*sp/c)*zp
                     - (-ck*sp + sk*so*cp)*tx
                     - (-ck*cp - sk*so*sp)*tz
                     - hx*(-co*cp*tx + co*sp*tz)/c
                     - ix*dwp);
      // d(L2)/dPhi
      B->setRC(r + 1, paramCnt++, (-sk*sp - ck*so*cp - hy*co*cp/c)*xp
                     + (-sk*cp + ck*so*sp + hy*co*sp/c)*zp
                     + (sk*sp + ck*so*cp)*tx
                     + (sk*cp - ck*so*sp)*tz
                     - hy*(-co*cp*tx + co*sp*tz)/c
                     - iy*dwp);
    }
    if (strchr(params, 'K') != NULL)
    { // d(L1)/dKappa
      B->setRC(r + 0, paramCnt, (-sk*cp + ck*so*sp)*xp + ck*co*yp
                     + (sk*sp + ck*so*cp)*zp
                     - (-sk*cp+ ck*so*sp)*tx - ck*co*ty
                     - (sk*sp + ck*so*cp)*tz);
      // d(L2)/dKappa
      B->setRC(r + 1, paramCnt++, (ck*cp + sk*so*sp)*xp + sk*co*yp
                     + (-ck*sp + sk*so*cp)*zp
                     + (-ck*cp - sk*so*sp)*tx - sk*co*ty
                     + (ck*sp - sk*so*cp)*tz);
    }
    if (strchr(params, 'x') != NULL)
    { // d(L1)/dtx
      B->setRC(r + 0, paramCnt, -ck*cp - sk*so*sp + hx*co*sp/c - ix*co*sp/c);
      // d(L2)/dtx
      B->setRC(r + 1, paramCnt++, -sk*cp + ck*so*sp + hy*co*sp/c - iy*co*sp/c);
    }
    if (strchr(params, 'y') != NULL)
    { // d(L1)/dty
      B->setRC(r + 0, paramCnt, -sk*co - hx*so/c + ix*so/c);
      // d(L2)/dty
      B->setRC(r + 1, paramCnt++, ck*co - hy*so/c + iy*so/c);
    }
    if (strchr(params, 'z') != NULL)
    { // d(L1)/dtz
      B->setRC(r + 0, paramCnt, ck*sp - sk*so*cp + hx*co*cp/c - ix*co*cp/c);
      // d(L2)/dtz
      B->setRC(r + 1, paramCnt++, sk*sp + ck*so*cp + hy*co*cp/c - iy*co*cp/c);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCalibrationMarkSet::setBarcodePosRowSet(
              UMatrixBig * B, UMatrixBig * K, int r,
              UCamPar * camp, bool adjRadial,
              UPosition * chartPos0, URotation * chartRot0,
              UCalibrationMark * pkt)
{
  bool result;
  //
  // get variables in easy form
  const double sk = sin(chartRot0->Kappa);
  const double ck = cos(chartRot0->Kappa);
  const double sp = sin(chartRot0->Phi);
  const double cp = cos(chartRot0->Phi);
  const double so = sin(chartRot0->Omega);
  const double co = cos(chartRot0->Omega);
  const double xc = pkt->rx;    // opsition of corner in chart -
  const double yc = pkt->ry;    // relative to center
  const double xt = chartPos0->x; // translation of chart from camera
  const double yt = chartPos0->y;
  const double zt = chartPos0->z;
  const double xh = camp->getHx();
  const double yh = camp->getHy();
  const double c  = camp->getFocalLength();
  double xm = pkt->ix;   // image point in pixels
  double ym = pkt->iy;
  int paramCnt = 0;
  double ccc, ccs, sss, ssc, css, csc, scs, scc;
  double pcsmssc, mccmsss, mscpcss, psspcsc, pscmcss;
  double ztc;
  float fxm, fym;
  //
  result =  (camp->getFocalLength() > 0.1) and (r < int(K->maxSize()));
  if (result)
  { // remove radial error from pixel coordinates
    if (adjRadial)
    { // remove radial error from pixel coordinates
      result = camp->getRadialD2U(pkt->ix, pkt->iy, & fxm, & fym);
      xm = fxm;
      ym = fym;
    }
    else
    { // radial error removed already
      xm = pkt->ix;
      ym = pkt->iy;
    }
  }
  if (result)
  {  // prepare calculation
    ccc = cp*co*ck;
    ccs = cp*co*sk;
    sss = sp*so*sk;
    ssc = sp*so*ck;
    css = cp*so*sk;
    csc = cp*so*ck;
    scs = sp*co*sk;
    scc = sp*co*ck;

    pcsmssc = (+cp*sk - ssc)/c;
    mccmsss = (-cp*ck - sss)/c;
    mscpcss = (-sp*ck + css)/c;
    psspcsc = (+sp*sk + csc)/c;
    pscmcss = (+sp*ck + css)/c;
    ztc     = zt/c;

    // Equation 1 with default values of [Omega, Phi, Kappa, tx, ty, tz]
    // watch K->m[r]
    K->setRC(r + 0, 0, ( cp*ck + sss - xh*mscpcss) * xc +
                  (-cp*sk + ssc - xh*psspcsc) * yc +
                   xt - xh*ztc +
                  ( mscpcss*xc + psspcsc*yc + ztc)*xm);
    // Equation 1 with default values of [Omega, Phi, Kappa, tx, ty, tz]
    // watch K->m[r+1]
    K->setRC(r + 1, 0, (-co*sk - yh*mscpcss)*xc +
                  (-co*ck - yh*psspcsc)*yc -
                  yt - yh*ztc +
                  ( mscpcss*xc + psspcsc*yc + ztc)*ym);
    //
    // Omega
    // then the big matrix with up to 6 columns for each row
    { // d(L1)/dOmega B->m[(r+0)*B->cols + 0]
      B->setRC(r + 0, paramCnt, (scs - xh*ccs/c)*xc +
                  (scc - xh*ccc/c)*yc +
                  (ccs*xc/c + ccc*yc/c)*xm);
      // d(L2)/dOmega
      B->setRC(r + 1, paramCnt++, (so*sk - yh*ccs/c)*xc +
                  (so*ck - yh*ccc/c)*yc +
                  (ccs*xc/c + ccc*yc/c)*ym);
    }
    // Phi
    { // d(L1)/dPhi
      B->setRC(r + 0, paramCnt, (-sp*ck + css - xh*mccmsss)*xc +
                  (sp*sk + csc - xh*pcsmssc)*yc +
                  (mccmsss*xc + pcsmssc*yc)*xm);
      // d(L2)/dPhi
      B->setRC(r + 1, paramCnt++, -yh*mccmsss*xc -
                   yh*pcsmssc*yc +
                   (mccmsss*xc + pcsmssc*yc)*ym);
    }
    // Kappa
    { // d(L1)/dKappa
      B->setRC(r + 0, paramCnt, (-cp*sk + ssc - xh*psspcsc) *xc +
                  (-cp*ck - sss - xh*pscmcss)*yc +
                  (psspcsc*xc + pscmcss*yc)*xm);
      // d(L2)/dKappa
      B->setRC(r + 1, paramCnt++, (-co*ck - yh*psspcsc)*xc +
                  ( co*sk - yh*pscmcss)*yc +
                  ( psspcsc*xc + pscmcss*yc)*ym);
    }
    // X-position
    { // d(L1)/dtx
      B->setRC(r + 0, paramCnt, 1.0);
      // d(L2)/dtx
      B->setRC(r + 1, paramCnt++, 0.0);
    }
    // Y position
    { // d(L1)/dty
      B->setRC(r + 0, paramCnt, 0.0);
      // d(L2)/dty
      B->setRC(r + 1, paramCnt++, -1.0);
    }
    // Z position
    { // d(L1)/dtz
      B->setRC(r + 0, paramCnt, (-xh + xm)/c);
      // d(L2)/dtz
      B->setRC(r + 1, paramCnt++, (-yh + ym)/c);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////
/*
bool UCalibrationMarkSet::implementCalibResults(UCamPar * camp)
{ // implement camera position and rotation
  bool result = true;
  //
  if (valid and calValid and (cam != NULL))
  {
    cam->relRot = rotCal;
    cam->relPos = posCal;
    implemented = true;
  }
  else
    result = false;
  //
  return result;
}
*/
//////////////////////////////////////////////

bool UCalibrationMarkSet::info(int cam)
{
  bool result = true;
  const int MSL = 250;
  char s[MSL];
  //
  if (valid)
  {
    snprintf(s, MSL, "Device %d calibration data has %d points",
                     cam, markCnt);
    toLog(s, 4);
    if (calValid)
    {
      snprintf(s, MSL,
          "Calibration OK, error is %6.4f m and %6.4f rad in %d loops",
          errDist, errRot,  loops);
      toLog(s, 4);
      posCal.sprint(s, "New position");
      toLog(s, 4);
      rotCal.sprint(s, "New rotation");
      toLog(s, 4);
      if (implemented)
        toLog("New data is implemented", 4);
      else
        toLog("New data is NOT implemented", 4);
    }
  }
  else
  {
    snprintf(s, MSL,
         "No valid calibration data for device %d", cam);
    toLog(s, 4);
    result = false;
  }
  return result;
}

///////////////////////////////////////////////////////

bool UCalibrationMarkSet::saveMarksToFile(const char filename[],
                                     bool radialErrorRemoved,
                                     UCamPar * cam,
                                     int imageNumber)
{
  const int FILENAME_LENGTH = 500;
  bool result;
  FILE * f;
  char fn[FILENAME_LENGTH];
  int i;
  float cx, cy, e, se = 0.0;
  //
  result = (valid and (markCnt > 0));
  if (result)
  { // something to write, so open the file
    if (filename != NULL)
      if (strchr(filename,'/') != NULL)
        snprintf(fn, FILENAME_LENGTH, "%s.m", filename);
      else
        snprintf(fn, FILENAME_LENGTH, "%s/%s.m", imagePath, filename);
    else
      snprintf(fn, FILENAME_LENGTH, "%s/image%ldcalib.m", dataPath, barcode);
    f = fopen(fn, "w");
    result = (f != NULL);
    if (not result)
      toLog("File open in 'UCalibrationMarkSet::saveMarksToFile' failed", 2);
  }
  if (result)
  { // file is valid
    // write mark positions (in matlab format)
    fprintf(f, "%% Matlab formatted file with barcode frame positions.\n");
    fprintf(f, "%% Original filename %s\n", fn);
    fprintf(f, "%% File format\n");
    fprintf(f, "%% image coordinates in pixels\n");
    fprintf(f, "%% followed by position on chart in meters\n");
    fprintf(f, "%% image pixel size relative to max-resolution %5.2f\n",
                        cam->getPixelSize());
    if (radialErrorRemoved)
      fprintf(f, "%% Radial error removed in image\n");
    else
      fprintf(f, "%% Radial error not removed in image\n");
    // camera assumed values
    fprintf(f, "%% Assumed focal length %f pixels\n",
                        cam->getFocalLength());
    fprintf(f, "focusLength = %f;\n",
                        cam->getFocalLength());
    fprintf(f, "k1 = %e;\n", cam->getK1());
    fprintf(f, "k2 = %e;\n", cam->getK2());
    fprintf(f, "hx = %e;\n", cam->getHx());
    fprintf(f, "hy = %e;\n", cam->getHy());
    // marks
    fprintf(f, "%% image x, y [pix] -- barcode x, y [m] -- "
                        "projected x, y [pix] --- error [pix]\n");
    fprintf(f, "mark%d = [...\n", imageNumber);
    for (i = 0; i < markCnt; i++)
    { // get projected position to cx, cy - and error e
      e = getErrorInPixels(&mark[i], &cx, &cy, cam, not radialErrorRemoved);
      se += e;
      fprintf(f, "  %7.3f %7.3f   %9.6f %9.6f  %7.3f %7.3f  %7.3f",
                  mark[i].ix, mark[i].iy,
                  mark[i].rx, mark[i].ry,
                  cx, cy, e);
      if (i < (markCnt-1))
        fprintf(f, "; ...\n");
      else
        fprintf(f, "];\n");
    }
    // make plottings
    fprintf(f, "%% plot of chart\n");
    fprintf(f, "figure\n");
    fprintf(f, "plot(mark%d(:,1), -mark%d(:,2), '.b', "
                    "mark%d(:,5), -mark%d(:,6), 'sr', "
                    "hx, -hy, '*k');\n", imageNumber,
                    imageNumber, imageNumber, imageNumber);
    fprintf(f,"axis equal\n");
    fprintf(f,"grid on\n");
    fprintf(f,"title('Calibration image %d with a3=%9.2e, a5=%9.2e (e=%4.2f)');\n",
                     imageNumber, cam->getK1(), cam->getK2(), se/markCnt);
    fprintf(f,"xlabel('Image x (width) in pixels');\n");
    fprintf(f,"ylabel('Image y (height) in pixels');\n");
    fprintf(f,"legend('image', 'projected', 'image center');\n");

    fprintf(f, "%% plot of error\n");
    fprintf(f, "rp = sqrt((mark%d(:,1) - hx).^2 + (mark%d(:,2) - hy).^2)\n",
                  imageNumber, imageNumber);
    fprintf(f, "rr = sqrt((mark%d(:,5) - hx).^2 + (mark%d(:,6) - hy).^2)\n",
                  imageNumber, imageNumber);
    fprintf(f, "re = rr - rp;\n");
    fprintf(f, "figure\n");
    fprintf(f, "plot(rr, re, '.');\n");
    fprintf(f,"grid on\n");
    fprintf(f,"title('Residual error image %d with a3=%9.2e, a5=%9.2e (e=%4.2f)');\n",
                     imageNumber, cam->getK1(), cam->getK2(), se/markCnt);
    fprintf(f,"xlabel('Radius in pixels');\n");
    fprintf(f,"ylabel('Error \\Delta_r = r_{img} - r_{proj}. [pixels]');\n");
    fclose(f);
  }
  //
  return result;
}

///////////////////////////////////////////////////

double UCalibrationMarkSet::getErrorInPixels(
             UCalibrationMark * thisMark,
             float * cx, float * cy,
             UCamPar * camp,
             bool radErrAdj)
{ // NB! expects barcode position/rotation are in camera coordiantes
  // Xpix = b*P*Tbar^-1*Pbar'*Obar'* Kbar'*Xbar. <br>
  // b and P is taken from camp->mb and camp->mP as mItoP.
  double result = 0.0;
  //int i, n;
  UMatrix4 mItoP;
  UMatrix4 mTr;
  UMatrix4 vXp; // pixel position
  UMatrix4 vXc(4, 1); // position on chart
  bool isOK;
  float xp, yp, xu, yu;
  // calculate transformation matrix
  mItoP = *camp->getItoP();
  isOK = (valid and (mItoP.err == 0));
  if (isOK)
  { // make total transform matrix
    mItoP.expand(3,4,0.0);
    mTr = mItoP * rotBarcode.asMatrix4x4CtoW(&posBarcode);
    isOK = (mTr.err == 0);
  }
  if (isOK)
  { // find expected pixel from 3D chart position to vXp
//     i = 0;
//     n = 0;
    // insert position to vector as [x, y, 0, 1]
    vXc.set(thisMark->rx, thisMark->ry, 0.0, 1.0);
    vXp = mTr * vXc;
    // vXp is now [w*x, w*y, w]
    isOK = (vXp.err == 0);
  }
  if (isOK)
  { // get difference
    xu = vXp.get(0) / vXp.get(2);
    yu = vXp.get(1) / vXp.get(2);
    if (radErrAdj)
      camp->getRadialU2D(xu, yu, &xp, &yp);
    else
    { // no radial error correction applied (removed in image)
      xp = xu;
      yp = yu;
    }
    *cx = xp;
    *cy = yp;
    result = hypot(xp - thisMark->ix, yp - thisMark->iy);
  }
  //
  if (not isOK)
    // error in calculation
    result = -1.0;
  //
  return result;
}

///////////////////////////////////////////////////

double UCalibrationMarkSet::geterrorInPixels(
              UPosition * chartPos,// assume this barcode position
              URotation * chartRot,// assume this barcode rotation
              UPosition * camPos,  // assume this camera position
              URotation * camRot,  // assume this camera rotation
              int markDist,         // use every 'markDist' mark
              UCamPar * camp,
              double * variance,   // return also variance
              bool adjRadial)      // correct for radial error
{ //  Xpix = b * P * Pcam * Ocam * Kcam * Tcam * Tbar^-1 * Pbar' * Obar' * Kbar' * Xbar. <br>
  // If 'markDist' is 1, then all points are used else only [0, markDist, ... n* markDist],
  // for n = 1,2,3... until no more marks.
  // b and P is taken from cam->mb and cam->mP as mItoP.
  double result = 0.0;
  double var = 0.0;
  double d;
  int i, n = 0;
  UMatrix4 mItoP;
  UMatrix4 mTr;
  UMatrix4 vXp; // pixel position
  UMatrix4 vXc(4, 1); // position on chart
  bool isOK;
  double xp, yp;
  float dx, dy;
  // debug
  FILE * fl = NULL;
  const int MSL = 250;
  char s[MSL];
  //
  if (adjRadial and false)
  {
    fl = fopen("calibPar.txt","w");
    fprintf(fl, "mPos := <%e | %e | %e | 1>;\n", chartPos->x, chartPos->y, chartPos->z);
    fprintf(fl, "mRot := <%e | %e | %e | 1>;\n",
         chartRot->Omega, chartRot->Phi, chartRot->Kappa);
    mTr = chartRot->asMatrix4x4CtoW();
    mTr.snprint("", s, MSL);
    mTr = chartRot->asMatrix4x4CtoW(chartPos);
    fprintf(fl, "mR:=%s;\n", s);
    mTr.snprint("", s, MSL);
    fprintf(fl, "mTR:=%s;\n", s);
    fprintf(fl, "mhx := %e;\n", camp->getHx());
    fprintf(fl, "mhy := %e;\n", camp->getHy());
    fprintf(fl, "mc := %e;\n", camp->getFocalLength());
    fprintf(fl, "mK1 := %e;\n", camp->getK1());
    fprintf(fl, "mK2 := %e;\n", camp->getK2());
  }
  // end debug
  // calculate transformation matrix
  mItoP = *camp->getItoP();
  isOK = (valid and (mItoP.err == 0));
  if (isOK)
  { // make total transform matrix
    mItoP.expand(3,4,0.0);
    if (camPos != NULL)
      // calculate in robot coordinates using camera position on robot
      mTr = mItoP * camRot->asMatrix4x4WtoC(camPos) *
                    chartRot->asMatrix4x4CtoW(chartPos);
    else
      // all in camera coordinates
      mTr = mItoP * chartRot->asMatrix4x4CtoW(chartPos);
    isOK = (mTr.err == 0);
  }
  if (isOK)
  { // sum errors
    i = 0;
    n = 0;
    while ((i < markCnt) and isOK)
    { // insert position to vector as [x, y, 0, 1]
      vXc.set(mark[i].rx, mark[i].ry, 0.0, 1.0);
      vXp = mTr * vXc;
      // vXp is now [w*x, w*y, w]
      isOK = (vXp.err == 0);
      if (isOK)
      {
        xp = vXp.get(0) / vXp.get(2);
        yp = vXp.get(1) / vXp.get(2);
        if (adjRadial)
        {
          isOK = camp->getRadialU2D(xp, yp, &dx, &dy);
          // debug
          if (fl != NULL)
          {
            fprintf(fl, "X3d%d := <%e , %e , 0 , 1>;\n", n, mark[i].rx, mark[i].ry);
            fprintf(fl, "Xmd%d := <%e , %e >;\n", n, dx, dy);
            fprintf(fl, "Xmm%d := <%e , %e >;\n", n, mark[i].ix, mark[i].iy);
          }
          // end debug
          xp = dx;
          yp = dy;
        }
        d = sqr(xp - mark[i].ix) + sqr(yp - mark[i].iy);
        result += sqrt(d);
        var += d;
        n++;
        i += markDist;
      }
    }
  }
  // debug
  if (fl != NULL)
    fclose(fl);
  // end debug
  if (isOK and (n > 0))
  { // make average distance
    result /= double(n);
    if (variance != NULL)
    {
      var /= double(n);
      var -= sqr(result);
      *variance = var;
    }
  }
  //
  if (not isOK)
    // error in calculation
    result = -1.0;
  //
  return result;
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// UCalibrationMarkSets

UCalibrationMarkSets::UCalibrationMarkSets()
{ /* nothing yet */
  setCnt = 0;
  chartPos.clear();
}


///////////////////////////////////////////////////////

void UCalibrationMarkSets::clear()
{
  setCnt = 0;
}

///////////////////////////////////////////////////////

UCalibrationMarkSet * UCalibrationMarkSets::getSet(int setNumber)
{
  UCalibrationMarkSet * result = NULL;
  int i, n = -1;
  //
  for (i = 0; i < setCnt; i++)
  {
    if (set[i].valid)
      n++;
    if (n == setNumber)
    {
      result = &set[i];
      break;
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

UCalibrationMarkSet * UCalibrationMarkSets::getNewSet()
{ // get an unused (invalid) set
  UCalibrationMarkSet * result = NULL;
  int i;
  //
  for (i = 0; i < mini(setCnt+1, MAX_CALIBRATION_SETS_SAVED); i++)
  {
    if (not set[i].valid)
    {
      result = &set[i];
      result->clear();
      if (i >= setCnt)
        setCnt = i + 1;
      break;
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

int UCalibrationMarkSets::countSetsWith(unsigned int codeValue)
{
  int result = 0;
  int i;
  //
  for (i = 0; i < setCnt; i++)
    if (set[i].barcode == codeValue)
      result++;
  //
  return result;
}

///////////////////////////////////////////////////////

UCalibrationMarkSet * UCalibrationMarkSets::getThisSet(
                           unsigned int codeValue,
                           int nth)
{
  UCalibrationMarkSet * result = NULL;
  int i, j = 0;
  //
  for (i = 0; i < setCnt; i++)
  { // look for the code
    if (set[i].barcode == codeValue)
    { // right code - are we there yet
      if (j == nth)
      { // found
        result = &set[i];
        break;
      }
      j++;
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCalibrationMarkSets::saveNewSet(UCalibrationMarkSet * frameSet)
{
  bool result = false;
  UCalibrationMarkSet * newSet;
  //UCamera * cam;
  //
  if (frameSet->valid and
      frameSet->barcodeValid)
  { // there is basis for storing the data
    newSet = getNewSet();
    if (newSet != NULL)
    { // copy data
      *newSet = *frameSet;
      /*
      Do not test if position is valid
      if (not newSet->calValid)
      { // no camera position - copy from camerea ref
        cam = newSet->getCameraRef();
        newSet->posCal = cam->relPos;
        newSet->rotCal = cam->relRot;
        newSet->calValid = true;
      }
      */
      result = true;
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////////////////////

int UCalibrationMarkSets::getSavedSets()
{
  int result = 0;
  int i;
  //
  for (i = 0; i < setCnt; i++)
    if (set[i].valid)
      result++;
  //
  return setCnt;
}

///////////////////////////////////////////////////////

bool UCalibrationMarkSets::doEstimateCameraParameters(
                 UCamPar * oldEstimate,
                 bool debug)
{
  bool result;
  int i, l, n, m;
  const int SETS_USED = 5;
  const int MAX_EQUATIONS_SETS = 20;
  const int MAX_LOOPS = 50;
  UCalibrationMarkSet * cset[SETS_USED];
  UCalibrationMarkSet * cs;
  UCalibrationMark * cm;
  UPosition posC; // position on chart
  UPosition pos0(0.0, 0.0, 0.0);
  URotation rot0(0.0, 0.0, 0.0);
  bool finished;
  double e, pixError, oldPixError;
  const int MSL = 250;
  char s[MSL];
  char s2[MSL];
  UPosition posDebug(0.0, 0.0, 0.0);
  // at least 5 set of data is required
  n = 0;
  result = (setCnt >= 5);
  if (result)
  { // get the sets to use
    // if more than 5, then there
    // should probably be
    // a selection for best variance in position
    for (i = 0; i < setCnt; i++)
    {
      if (set[i].valid)
        cset[n++] = &set[i];
      if (n >= SETS_USED)
        break;
    }
  }
  result = (n == SETS_USED);
  if (debug)
  {
    snprintf(s, MSL,
      "doEstimateCameraParameters - found %d sets", n);
    toLog(s, 5);
  }
  //
  if (result)
  {
    estCamPar.setCameraParameters(oldEstimate);
    // estimate parameters
    finished = false;
    l = 0;
    while (not finished)
    { // estimate chart position for all chart positions
      // using current camera parameters.
      pixError = 0.0;
      oldPixError = 100.0;
      //
      // debug
      if (debug)
      {
        snprintf(s, MSL,
          "loop %d, pixErr ", l);
      }
      // end debug
      //
      for (i = 0; i < SETS_USED; i++)
      {
        cs = cset[i];
        //result = cs->evaluateChartPosRot(&posDebug, NULL,
        //                                  0.0001, 0.0002, &estCamPar, false, true);
        result = cs->evaluateChartPosRot(&cs->posBarcode, NULL,
                                            0.0001, 0.0002, &estCamPar, true, false);
        if (not result)
          break;
        //pixError += cs->geterrorInPixels(&cs->posBarcode, &cs->rotBarcode,
        //                                  &pos0, &rot0, 1, &estCamPar, false);
        e = cs->geterrorInPixels(&cs->posBarcode, &cs->rotBarcode,
                                 &pos0, &rot0, 1, &estCamPar, NULL, true);
        pixError += e;
        //
        // debug
        if (debug)
        {
          snprintf(s2, MSL, " %5.3f", e);
          strcat(s, s2);
        }
        // enddebug
        //
      }
      // average pixel error
      pixError /= SETS_USED;
      //
      // debug
      if (debug)
      {
        snprintf(s2, MSL, " avg: %5.3f", pixError);
        strcat(s, s2);
        toLog(s, 5);
      }
      // end debug
      //
      if (pixError < oldPixError)
      { // stop criteria
        if ((oldPixError - pixError) < 0.005)
          finished = true;
        if (pixError < 0.1)
          finished = true;
        oldPixError = pixError;
      }
      //
      if (finished or not result)
        break;
      // estimate new camera parameters
      m = 0;
      estCamPar.initParEst();
      for (i = 0; i < SETS_USED; i++)
      {
        cs = cset[i];
        for (n = 0; n < cs->markCnt;
             n = n + cs->markCnt /
                 (MAX_EQUATIONS_SETS/SETS_USED))
        {
          cm = &cs->mark[n];
          posC.set(cm->rx, cm->ry, 0.0);
          result = estCamPar.setEstMatrix35(
                     m,   // equation element
                     cm->ix * cs->pixSize,
                     cm->iy * cs->pixSize,
                     posC,
                     cs->posBarcode,  // translation
                     cs->rotBarcode,  // rotation
                     i); // chart number
          m = m + 2;
        }
        if (not result)
          break;
      }
      if (result)
      { // solve equations
        result = estCamPar.adjustParameters(true);
      }
      if (l++ > MAX_LOOPS)
        break;
    } // while
    if (not finished)
    {
      snprintf(s, MSL,
        "doEstimateCameraParameters - failed after %d loops, pixErr = %f",
                l, pixError);
      toLog(s, 3);
    }
    else
    {
      snprintf(s, MSL,
        "doEstimateCameraParameters - succeded after %d loops, pixErr = %f",
                l, pixError);
      toLog(s, 4);
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////////////////

float UCalibrationMarkSets::evaluatePixError(
                       UCalibrationMarkSet * cset[],
                       const unsigned int SETS_USED,
                       const float hx, const float hy,
                       const float k1, const float k2,
                       const float c,
                       float pixSize)
{
  float result;
  UCalibrationMarkSet * cs;
  float pixError;
  unsigned int i;
  bool isOK = true;
  UPosition pos0(0.0, 0.0, 0.0);
  URotation rot0(0.0, 0.0, 0.0);
  FILE * fl = NULL;
  //
  estCamPar.setCameraParameters(hx, hy, k1, k2, c, pixSize);
  pixError = 0.0;
  //
  for (i = 0; i < SETS_USED; i++)
  {
    cs = cset[i]; // cset[2]
    //result = cs->evaluateChartPosRot(&posDebug, NULL,
    //                                 0.0001, 0.0002, &estCamPar, false, true);
    if (cs == NULL)
      perror("UCalibrationMarkSets::evaluatePixError:Pointer missing 0\n");
    else
      isOK = cs->evaluateChartPosRot(&cs->posBarcode, NULL,
                                     0.0001, 0.0002, // estimate stop criteria
                                     &estCamPar, // camera parameters
                                     true, // adjust for radial error
                                     fl);
    if (not isOK)
      break;
    //pixError += cs->geterrorInPixels(&cs->posBarcode, &cs->rotBarcode,
    //                                  &pos0, &rot0, 1, &estCamPar, false);
    if (cs == NULL)
      perror("UCalibrationMarkSets::evaluatePixError:Pointer missing 2\n");
    else
      pixError += cs->geterrorInPixels(&cs->posBarcode, &cs->rotBarcode,
                                   &pos0, &rot0, // camera is at 0.0
                                   1,   // evaluate for all points (step = 1)
                                   &estCamPar, // camera parameters
                                   NULL,  // variance
                                   true); // adjust for radial error
    if (cs == NULL)
      perror("UCalibrationMarkSets::evaluatePixError:Pointer missing 3\n");
    //
  }
  // average pixel error
  result = pixError / SETS_USED;
  return result;
}

/////////////////////////////////////////////////////////////////

float UCalibrationMarkSets::evaluatek2PixError(
                       UCalibrationMarkSet * cset[],
                       const unsigned int SETS_USED,
                       const float hx, const float hy,
                       const float k1,
                       const float k2min, float k2max, float * k2e, int steps,
                       const float c,
                       float pixSize, bool alsoK2)
{
  float pmin = k2min;
  float pmax = k2max;
  float emin;
  float emax;
  int i;
  //
  if (not alsoK2) // cset[0]
    emax = evaluatePixError(cset, SETS_USED, hx, hy, k1, *k2e, c, pixSize);
  else
  { // do full estimation
    emin = evaluatePixError(cset, SETS_USED, hx, hy, k1, pmin, c, pixSize);
    emax = evaluatePixError(cset, SETS_USED, hx, hy, k1, pmax, c, pixSize);
    for (i = 0; i < steps; i++)
    { // best estimate is between limits
      *k2e = (pmax - pmin)/2.0 + pmin;
      if (emin < emax)
      { // min limit is best - change max
        pmax = *k2e;
        emax = evaluatePixError(cset, SETS_USED, hx, hy, k1, pmax, c, pixSize);
      }
      else
      { // max limit is best move min
        pmin = *k2e;
        emin = evaluatePixError(cset, SETS_USED, hx, hy, k1, pmin, c, pixSize);
      }
    }
  }
  return emax;
}

/////////////////////////////////////////////////////////////////

float UCalibrationMarkSets::evaluatek1PixError(
                       UCalibrationMarkSet * cset[],
                       const unsigned int SETS_USED,
                       const float hx, const float hy,
                       const float k1min, float k1max, float * k1e, int steps1,
                       const float k2min, float k2max, float * k2e, int steps2,
                       const float c,
                       float pixSize,
                       bool alsoK1, bool alsoK2)
{
  float pmin = k1min;
  float pmax = k1max;
  float emin;
  float emax;
  float d;
  int i, t;
  const int MSL = 250;
  char s[MSL];
  //
  if (not alsoK1)
    emax = evaluatek2PixError(cset, SETS_USED, hx, hy, *k1e,
                k2min, k2max, k2e, steps2, c, pixSize, alsoK2);
  else
  {
    emin = evaluatek2PixError(cset, SETS_USED, hx, hy, pmin,
                k2min, k2max, k2e, steps2, c, pixSize, alsoK2);
    emax = evaluatek2PixError(cset, SETS_USED, hx, hy, pmax,
                k2min, k2max, k2e, steps2, c, pixSize, alsoK2);
    for (i = 0; i < steps1; i++)
    { // best is between limits
      d = (pmax - pmin)/3.5;
      if (emin < emax)
      { // min is best - move max
        pmax -= d;
        t = -1;
        emax = evaluatek2PixError(cset, SETS_USED, hx, hy, pmax,
                  k2min, k2max, k2e, steps2, c, pixSize, alsoK2);
      }
      else
      { // max is best - move min
        pmin += d;
        t = 1;
        emin = evaluatek2PixError(cset, SETS_USED, hx, hy, pmin,
                   k2min, k2max, k2e, steps2, c, pixSize, alsoK2);
      }
      *k1e = (pmax + pmin)/2.0;
      snprintf(s, MSL, "---- %d a3:%11.3e, a5%11.3e, c:%9.3f, err:%7.5f, d:%2d",
                          i, *k1e, *k2e, c, emin, t);
      toLog(s, 5);
    }
  }
  return emax;
}

/////////////////////////////////////////////////////////////////


bool UCalibrationMarkSets::doEstimateCameraParametersBinary(
                 UCamPar * oldEstimate,
                 bool debug,
                 unsigned long setsToUse,
                 bool alsoFocalLength,
                 bool alsoK1, bool alsoK2)
{
  bool result;
  unsigned int i, m, n;
  const unsigned int SETS_USED = 5;
  const int MAX_EQUATIONS_SETS = 20;
  UCalibrationMarkSet * cset[SETS_USED];
  unsigned int csetCnt;
  UCalibrationMarkSet * cs;
  UCalibrationMark * cm;
  UPosition posC; // position on chart
  const int MSL = 250;
  char s[MSL];
  UPosition posDebug(0.0, 0.0, 0.0);
  int ic = 0;
  float k1r, k2r, cr;
  float k1e, k2e, ce = 0.0, d;
  float k1min, k1max;
  float k2min, k2max;
  float cmin, cmax;
  float cemin = 0.0, cemax;
  //double pixErrorb;
  float pixSize = 0.0;
  int t;
  // at least 5 set of data is required
  csetCnt = 0;
  if (setsToUse == 0)
    setsToUse = 0xffffffff;
  // get the sets to use
  // if more than 5, then there
  // should probably be
  // a selection for best variance in position
  for (i = 0; i < (unsigned int)setCnt; i++)
  {
    if ((setsToUse & (0x01 << i)) != 0)
        cset[csetCnt++] = getSet(i);
    if (csetCnt >= SETS_USED)
      break;
  }
  result = csetCnt > 1;
  if (debug)
  {
    snprintf(s, MSL,
      "doEstimateCameraParameters - found %d sets", csetCnt);
    toLog(s, 5);
  }
  //
  if (result)
  {
    estCamPar.setCameraParameters(oldEstimate);
    pixSize = oldEstimate->getPixelSize();
    // estimate parameters
    // estimate chart position for all chart positions
    // using current camera parameters.
    k1r = estCamPar.getK1();
    k2r = estCamPar.getK2();
    cr = estCamPar.getFocalLength();
    if (absf(k1r) < 1e-10)
    { // zero, so wide range
      k1min = -1e-6;
      k1max = +1e-6;
    }
    else
    { // fine tune only
      k1min = k1r-1e-7;
      k1max = k1r+1e-7;
    }
    if (absf(k2r) < 1e-14)
    { // zero, so wide range possible
      k2min = -2e-12;
      k2max = +2e-12;
    }
    else
    { // fine tuning only
      k2min = k2r-3e-13;
      k2max = k2r+3e-13;
    }
    cmin = cr - 100.0;
    cmax = cr + 100.0;
    //
    //pixErrorb = 1000.0;
    k1e = k1r;
    k2e = k2r;
    ce = cr;
    if (not alsoFocalLength)
    { // do the others only
      cemin = evaluatek1PixError(cset, csetCnt, 320.0/pixSize, 240.0/pixSize,
                          k1min, k1max, &k1e, 12,
                          k2min, k2max, &k2e, 10, cr, pixSize, alsoK1, alsoK2);
      snprintf(s, MSL, "Loop %d a3:%11.3e, a5%11.3e, c:%9.3f, err:%7.5f",
                            ic, k1e, k2e, cr, cemin);
      toLog(s, 6);
    }
    else
    {
      cemin = evaluatek1PixError(cset, csetCnt, 320.0/pixSize, 240.0/pixSize,
                            k1min, k1max, &k1e, 12,
                            k2min, k2max, &k2e, 10, cmin, pixSize, alsoK1, alsoK2);
      cemax = evaluatek1PixError(cset, SETS_USED, 320.0/pixSize, 240.0/pixSize,
                          k1min, k1max, &k1e, 10,
                          k2min, k2max, &k2e, 8, cmax, pixSize, alsoK1, alsoK2);
      snprintf(s, MSL, "Loop %d a3:%11.3e, a5%11.3e, c:%9.3f, err:%7.5f",
                            ic, k1e, k2e, ce, cemin);
      toLog(s, 6);
      for (ic = 0; ic < 12; ic++)
      {
        d = (cmax - cmin)/4.5;
        if (cemin < cemax)
        {
          cmax -= d;
          t = -1;
          cemax = evaluatek1PixError(cset, SETS_USED, 320.0/pixSize, 240.0/pixSize,
                          k1min, k1max, &k1e, 12,
                          k2min, k2max, &k2e, 8, cmax, pixSize, alsoK1, alsoK2);
        }
        else
        {
          cmin += d;
          t = 1;
          cemin = evaluatek1PixError(cset, SETS_USED, 320.0/pixSize, 240.0/pixSize,
                          k1min, k1max, &k1e, 12,
                          k2min, k2max, &k2e, 8, cmin, pixSize, alsoK1, alsoK2);
        }
        ce = (cmax + cmin) / 2.0;
        snprintf(s, MSL, "Loop %d, center c %9.4f, d:%2d",
                            ic, ce, t);
        toLog(s, 6);
      }
    }
  }
  if (cemin < 0.5)
    estCamPar.setCameraParameters(320.0/pixSize, 240.0/pixSize,
                                  k1e, k2e, ce, pixSize);
  if (false)
  {  // estimate new camera parameters
    m = 0;
    estCamPar.initParEst35();
    for (i = 0; i < csetCnt; i++)
    {
      cs = cset[i];
      for (n = 0; n < (unsigned int)cs->markCnt;
           n = n + cs->markCnt /
               (MAX_EQUATIONS_SETS/csetCnt))
      {
        cm = &cs->mark[n];
        posC.set(cm->rx, cm->ry, 0.0);
        result = estCamPar.setInitialValues35(
                   m,   // equation element
                   cm->ix * cs->pixSize,
                   cm->iy * cs->pixSize,
                   posC,
                   cs->posBarcode,  // translation
                   cs->rotBarcode,  // rotation
                   i); // chart number
        m++;
      }
      if (not result)
        break;
    }
    if (result)
    { // solve equations
      result = estCamPar.adjustParameters35(true);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCalibrationMarkSets::doEstimateCameraParametersCarpet(
                 UCamPar * oldEstimate,
                 bool debug)
{
  bool result;
  unsigned int i, m, n;
  const unsigned int SETS_USED = 5;
  const int MAX_EQUATIONS_SETS = 20;
  const int width = 30;
  UCalibrationMarkSet * cset[SETS_USED];
  UCalibrationMarkSet * cs;
  UCalibrationMark * cm;
  UPosition posC; // position on chart
  const int MSL = 250;
  char s[MSL];
  UPosition posDebug(0.0, 0.0, 0.0);
  int ic, ik;
  float k1r, cr;
  float k1e, k2e, ce, d1, d2;
  float k1min, k1max;
  float k2min, k2max;
  float cmin, cmax;
  float cemin = 0.0, cemax;
  float pixErrorb, k1b = 0.0, k2b = 0.0, ceb = 0.0;
  float pixSize = 0.0;
  FILE * fl;
  // at least 5 set of data is required
  n = 0;
  result = (setCnt >= 5);
  if (result)
  { // get the sets to use
    // if more than 5, then there
    // should probably be
    // a selection for best variance in position
    for (i = 0; i < (unsigned int)setCnt; i++)
    {
      if (set[i].valid)
        cset[n++] = &set[i];
      if (n >= SETS_USED)
        break;
    }
  }
  result = (n == SETS_USED);
  if (debug)
  {
    snprintf(s, MSL,
      "doEstimateCameraParameters - found %d sets", n);
    toLog(s, 5);
  }
  //
  if (result)
  {
    fl = fopen("par_est_carpet.m", "w");
    result = (fl != NULL);
  }
  if (result)
  {
    estCamPar.setCameraParameters(oldEstimate);
    pixSize = cset[0]->pixSize;
    // estimate parameters
    // estimate chart position for all chart positions
    // using current camera parameters.
    k1r = estCamPar.getK1();
    //k2r = estCamPar.getK2();
    cr = estCamPar.getFocalLength();
    pixErrorb = 2.0;
    k1e = k1r;
    k1min = -1e-6;
    k1max = +1e-6;
    k2min = -1.0e-12;
    k2max = +1.0e-12;
    cmin = 650.0;
    cmax = 1250.0;
    d1 = (k1max - k1min)/(float(width) - 1.0);
    d2 = (cmax - cmin)/(float(width) - 1.0);
    ce = cr;
    fprintf(fl, "%% parameter estimate carpet\n");
    fprintf(fl, "%% Min and max values\n");
    fprintf(fl, "focal_length_minmax = [%f, %f];\n", cmin, cmax);
    fprintf(fl, "a3_minmax = [%e, %e];\n", k1min, k1max);
    fprintf(fl, "a5_minmax = [%e, %e];\n", k2min, k2max);
    fprintf(fl, "dc = [");
    for (ic = 0; ic < width; ic++)
      fprintf(fl, "%f ", cmin + float(ic) * d2);
    fprintf(fl, "];\n");
    //
    fprintf(fl, "dk1 = [");
    for (ik = 0; ik < width; ik++)
      fprintf(fl, "%e ", k1min + float(ik) * d1);
    fprintf(fl, "];\n");
    //
    fprintf(fl, "cp = [ ...\n");
    for (ic = 0; ic < width; ic++)
    {
      for (ik = 0; ik < width; ik++)
      {
        k1e = k1min + float(ik) * d1;
        ce = cmin + float(ic) * d2;
        cemax = evaluatek2PixError(cset, SETS_USED, 320.0/pixSize, 240.0/pixSize,
                      k1e,
                      k2min, k2max, &k2e, 7, ce, pixSize, true);
        //
        fprintf(fl, "%f ", cemax);
        //
        if (cemax < pixErrorb)
        {
          pixErrorb = cemax;
          k1b = k1e;
          ceb = ce;
          k2b = k2e;
          snprintf(s, MSL, "best: c:%7.2f, a3:%11.3e, a5:%11.3e, err:%7.4f",
                                ceb, k1b, k2b, cemax);
          toLog(s, 6);
        }
      }
      if (ic < (width-1))
        fprintf(fl, "; ...\n   ");
      else
        fprintf(fl, "];\n");
      snprintf(s, MSL, "Line %d", ic);
      toLog(s, 6);
    }
    //
    fprintf(fl, "bestc = %f;\n", ceb);
    fprintf(fl, "besta3 = %e;\n", k1b);
    fprintf(fl, "besta5 = %e;\n", k2b);
    fprintf(fl, "hold off\n");
    fprintf(fl, "surf(dk1, dc, cp)\n");
    fprintf(fl, "title('Pixel error as function of focal length and a3 parameter')\n");
    fprintf(fl, "xlabel('a3 size');\n");
    fprintf(fl, "ylabel('Focal length');\n");
    fprintf(fl, "zlabel('average error [pixels]');\n");
    fprintf(fl, "%% end\n");
    fclose(fl);
  }
  if (cemin < 0.5)
    estCamPar.setCameraParameters(320.0/pixSize, 240.0/pixSize,
                                  k1b, k2b, ceb, pixSize);
  if (false)
  {  // estimate new camera parameters
    m = 0;
    estCamPar.initParEst35();
    for (i = 0; i < SETS_USED; i++)
    {
      cs = cset[i];
      for (n = 0; n < (unsigned int)cs->markCnt;
           n = n + cs->markCnt /
               (MAX_EQUATIONS_SETS/SETS_USED))
      {
        cm = &cs->mark[n];
        posC.set(cm->rx, cm->ry, 0.0);
        result = estCamPar.setInitialValues35(
                   m,   // equation element
                   cm->ix * cs->pixSize,
                   cm->iy * cs->pixSize,
                   posC,
                   cs->posBarcode,  // translation
                   cs->rotBarcode,  // rotation
                   i); // chart number
        m++;
      }
      if (not result)
        break;
    }
    if (result)
    { // solve equations
      result = estCamPar.adjustParameters35(true);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// UCalibrate

UCalibrate::UCalibrate()
{ // constructor
  // allocate processing images
  pRawImage = NULL;
  pFiltImage = NULL;
  pCornerImage = NULL;
  pGridImage = NULL;
  //image = NULL; // set to source raw image
  calcamset.clear();
  //
  logGmk = NULL;
  logGmk = fopen("gmk.log", "w");
  logLevelGmk = 99;
  verboseMessagesGmk = true;

}

////////////////////////////////////////////////

UCalibrate::~UCalibrate()
{ // close logfile
  if (logGmk != NULL)
    fclose(logGmk);
}

/////////////////////////////////////////////////////

bool UCalibrate::allocateBufferImages(UImagePool * imgPool)
{
  bool result;
  //
  pRawImage = imgPool->getImage(UCAM_IMS_CALIB_RAW, true);
  pFiltImage = imgPool->getImage(UCAM_IMS_CALIB_FILTER, true);
  pCornerImage = imgPool->getImage(UCAM_IMS_CALIB_CORNER, true);
  pGridImage = imgPool->getImage(UCAM_IMS_CALIB_GRID, true);
  // test result
  result = (pRawImage != NULL) and (pFiltImage != NULL) and
           (pCornerImage != NULL) and (pGridImage != NULL);
  return result;
}

/////////////////////////////////////////////////////

bool UCalibrate::isBufferImagesAvailable()
{
  return ((pFiltImage != NULL) and
          (pCornerImage != NULL) and
          (pGridImage != NULL));
}

/////////////////////////////////////////////////////


// int UCalibrate::Find100CalibrationPoints(
//                       UImage * image, // (raw) image with points
//                       char * name,
//                       bool debug,
//                       float stride)
// { // do find calibration points
//   // and load these to cc.hits list
// //  int erri = 0; // image save error
//   bool isOK;
//   int oldHitCount;
//   int validHits = 0;
//   //int averageIntensity;
//   int i, j;
//   UPixel pix(255,0,0);
//   UPixel pixi;
//   char filename[MaxFileNameLength];
//   const char * fpath = imagePath;
//   const int MSL = 250;
//   char s[MSL];
//   //UImage640 img;
//   UCalibrationMarkSet * mkSet;
//   UCalibrationComponent * cmp;
//   //
//   isOK = (image != NULL);
//   if (not isOK)
//     toLog("UCalibrate::Find100CalibrationPoints: Find calibration got no image?", 2);
//   if (isOK)
//     // filter source to filtered image
//     //isOK = LowPassFilter(image, pFiltImage, pFiltVarImage, 9, false);
//     cvSmooth(image->cvArr(), pFiltImage->cvArr(), CV_BLUR, 3, 3, 0.0);
//   if (isOK)
//   { // make an image where corners are marked with intensity > 0
//     //averageIntensity = image->intensity;
//     isOK = MarkCornerDetections4(
//                     pFiltImage,      // filtered image
//                     //pFiltVarImage,   // variance image
//                     pCornerImage,    // corner image
//                     //&averageIntensity,
//                     3,   // horizontal guard band
//                     3,   // diagonal   guard band
//                     3,  // horizontal and vertical
//                     3,  // diagonal
//                     true); // black is zero
//   }
//   if (isOK)
//   { // combine corner detections to numbered calibration components
//     // using a (12 pixel) connectivity mask.
//     // result transferred to hits array.
//     cc.clear();
//     isOK = cc.EnumerateCalibrationDetections(
//                 pCornerImage, 12);
//   }
//   if (isOK and debug)
//   { // debug paint
//     // save raw corner detection image
//     snprintf(filename, MaxFileNameLength,
//              "%s%s%06ldraw", fpath, name, image->imageNumber);
//     pix.setRGBto(128,128,128, PIX_PLANES_RGB);
//     pCornerImage->copy(image);
//     pCornerImage->tone(&pix, 60);
//     pGridImage->copy(image);
//     pGridImage->tone(&pix, 60);
//   }
//   //
//   validHits = cc.hitCount;
//   //
//   if (isOK and (validHits > 35))
//   { // now do size filtering
//     do
//     { // repeat until all sizes are within 3 sd
//       oldHitCount = validHits;
//       validHits = cc.DoSizeLimit();
//     } while (validHits < oldHitCount);
//     //
//     snprintf(s, MSL, "UCalibrate::Find100CalibrationPoints: After size limit %d hits", validHits);
//     toLog(s, 7);
//     //
//     if (debug)
//     { //sprintf(s, "Found %d corners of equal size", validHits);
//       //sVinfo(s, einfo, 3);
//       // now paint remaining corners white
//       pix.setRGBto(255, 255, 255, PIX_PLANES_RGB);
//       cc.PaintHitsInImage(pFiltImage, pix);
//       // and send to remote control
//       pFiltImage->imgTime.Now();
//       // sysState.SetImageCopy(1, pFiltImage, false);
//       // save
//       /*
//       snprintf(filename, MaxFileNameLength,
//               "%s%scal%dcm3", fpath, name, image->imageNumber);
//       img.copy(pFiltImage);
//       erri = img.SaveImageToFile(filename, NULL);
//       if (erri == 0)
//       {
//         snprintf(s, MAX_CAM_INFO_SIZE, "Saved %s", img.fileName);
//         sVinfo(s, einfo, 0);
//       }
//       */
//     }
//   }
//   //
//   isOK = validHits > 35;
//   // find camera calibration chart
//   if (isOK)
//   { // about 45 hits are minimum to find center cross.
//     // - Now order hits, find center and assign chart positions.
//     // - Center is returned in cc.centR, cc.centC and
//     //   result is ordered in cc.grid[][]
//     validHits = cc.DoOrderHits(pFiltImage, stride);
//     if (validHits > 0)
//     { // then valid hits are ordered in cc.grid[row,col] (rov/col range [0..49])
//       // with pixel position in cc.grid[row,col]->x and ...y   (pixel)
//       // and  chart position in cc.grid[row,col]->rx and ...ry (meter)
//       mkSet = calcamset.getNewSet();
//       if (mkSet != NULL)
//       { // save positions as calibration mark set
//         mkSet->clear();
//         // use image number as barcode - for reference
//         mkSet->barcode = image->imageNumber;
//         // find max row and max column - stored in cc.maxR and cc.maxC
//         cc.FindGridLimits();
//         for (i = 0; i <= cc.maxR; i++)
//           for (j = 0; j <= cc.maxC; j++)
//           { // exit if no more space
//             if (mkSet->markCnt >= MAX_FRAME_POSITIONS)
//               break;
//             // get component
//             cmp = cc.grid[i][j];
//             if (cmp != NULL)
//             { // data is available
//               mkSet->mark[mkSet->markCnt].setMark(cmp->row, cmp->col,
//                               cmp->x, cmp->y,    // image pixel position
//                               cmp->rx, cmp->ry); // position on chart
//               mkSet->markCnt++;
//             }
//           }
//         if (mkSet->markCnt > 50)
//           mkSet->valid = true;
//       }
//     }
//   }
//   if (isOK and debug)
//   { // display results as image and files
//     pix.setRGBto(255,0,0, PIX_PLANES_RGB);
//     for (i = 0; i< MAX_CALIB_GRID_SIZE; i++)
//       for (j = 0; j < MAX_CALIB_GRID_SIZE; j ++)
//         if (cc.grid[i][j] != NULL)
//           cc.grid[i][j]->PaintNeighbors(pGridImage, pix, true, cc.hits);
//     if ((validHits > 0) and (cc.grid[cc.centR][cc.centC] != NULL))
//     // paint center circle
//       cvCircle(image->cvArr(),
//              cvPoint(roundi(cc.grid[cc.centR][cc.centC]->x),
//                      roundi(cc.grid[cc.centR][cc.centC]->y)),
//              6, pix.cvRGB());
//     // paint same real coordinates
//     // save
//     //snprintf(filename, MaxFileNameLength, "%s%dcdots", name, image->imageNumber);
//     sImage->copy(image);
//     snprintf(s, MSL, "%s/%s%06lddots.bmp", imagePath, name, image->imageNumber);
//     sImage->saveBMP(s);
//     //sImage->save(filename, NULL, fpath);
//     //
//     sImage->copy(pCornerImage);
//     snprintf(s, MSL, "%s/%s%06ldcorn.bmp", imagePath, name, image->imageNumber);
//     sImage->saveBMP(s);
//     //erri = sImage->save(filename, NULL, fpath);
//     {
//       snprintf(s, MSL, "Saved %s%s.png", fpath, pCornerImage->name);
//       toLog(s, 4);
//     }
//     snprintf(filename, MaxFileNameLength, "%sorg", name);
//     //
//     snprintf(filename, MaxFileNameLength, "%s%sfilt", fpath, name);
//     // test for save-to-file request
//     if (strlen(name) > 0)
//     { // build full path
//       snprintf(filename, MaxFileNameLength, "%s%s.txt", fpath, name);
//       // save grid debug textfile
//       cc.SaveCalibPointsSquare(filename);
//       snprintf(s, MSL, "Saved grid layout to %s", filename);
//       toLog(s, 7);
//       // save MATLAB file
//       snprintf(filename, MaxFileNameLength, "%s%s.m", fpath, name);
//       cc.SaveCalibCoordinates(filename);
//       snprintf(s, MSL, "Saved Matlab file to %s", filename);
//       toLog(s, 7);
//       // save image file
//       snprintf(filename, MaxFileNameLength, "%sgrid.bmp", name);
//       sImage->copy(pGridImage);
//       snprintf(s, MSL, "%s/%s%06ldgrid.bmp", imagePath, name, image->imageNumber);
//       sImage->saveBMP(s);
//       //sImage->save(filename, NULL, fpath);
//     }
//   }
//   return validHits;
// }

///////////////////////////////////////////////////////////

bool UCalibrate::ShowStatus(int setNumber)
{ // show calibration status
  bool result = false;
  UCalibrationMarkSet * set;
  //
  // just show for calibration mark set
  set = calcamset.getSet(setNumber);
  if (set != NULL)
    result = set->info(setNumber);
  //
  return result;
}

/////////////////////////////////////////////////////

bool UCalibrate::MarkCornerDetections4
                  (UImage * fim, //UImage * fsdim,
                   UImage * cim,
                   //int * avgCornerInt,
                   int GuardBandH /*= 3*/,    // horizontal/vertical
                   int GuardBandD /*= 3*/,    // diagonal
                   int IntensDiffH /*= 50*/,  // horizontal-vertical
                   int IntensDiffD /*= 50*/,  // diagonal
                   bool assumeBlackIsZero)
{ // compares four pixels from the filtered
  // source image (fim) around a center cell,
  // if there is sufficient difference the pixel
  // in the destination image (cim) gets a white
  // intensity (otherwise black).
  // finds both corners with black upper right/lower left
  // and corners with black upper left/lower right.
  // and black top / bottom
  // and black left / right
  // returns o if sucessfull.
  bool result =true;
  int i,j,k,l, n;
  UPixel * pix;
  UPixel rPix(0,0,0); //rgb2,rgb, rgbA;
  int y1 = 0, y2 = 0, y3 = 0, y4 = 0; // horizontal-vertical crossing
  int f1 = 0, f2 = 0, f3 = 0, f4 = 0; // diagonal crossing
  //int yv1, yv2, yv3, yv4; // horizontal-vertical crossing
  //int fv1, fv2, fv3, fv4; // diagonal crossing
  int mDiff, mfDiff;
  int mC21, mC24, mC31, mC34;
  int mCf21, mCf24, mCf31, mCf34;
  float dw2ul; // black upper left
  float dw2ur; // black upper right
  float dw2top; // black on top
  float dw2left; // black left
  const int w = fim->width();
  const int h = fim->height();
  int mdh = GuardBandH;
  int mdd = GuardBandD;
  // debug
  FILE * fcn = NULL;
  const int MSL = 250;
  char s[MSL];
  //
  snprintf(s, MSL, "%s/filt%06ldCnDet.txt", dataPath, fim->imageNumber);
  // debug file
  // fcn = fopen(s, "w");
  if (fcn != NULL)
  {
    fprintf(fcn, "%% corner detections"
                  "%% - guardH %d, guardD %d\n"
                  "%% - 5-9 pixel column of raw image\n"
                  "%% - IntensDiffH %d, IntensDiffD %d\n",
                  GuardBandH, GuardBandD, IntensDiffH, IntensDiffD);
    fprintf(fcn, "%% Position (x,y), corner-value UL, UR, Top, Left\n");
    fprintf(fcn, "%%  HV    1 diag , Intensity h-v, Intensity diagonal\n");
    fprintf(fcn, "%%  1 3  2 3     , SD h-v, SD diagonal\n");
    fprintf(fcn, "%%  2 4   4      , Criteria result HV 21,24,31,34 - diag 21,24,31,34\n");
  }
  n = 0;
  // debug end
  //
  if (cim == NULL)
    result = false;
  else
  {
    if (cim->maxBytes() < fim->imgBytes())
      result = false;
    else
    { // set corner image attributes
      cim->copyMeta(fim, true);
      cim->clear(rPix);
      cim->valid = true;
      cim->used = false;
      cim->setColorType("YUV");
      snprintf(cim->name, MAX_IMG_NAME_SIZE, "corners%06ld", fim->imageNumber);
    }
  }
  if (result)
  { // for all pixels in filtered image
    for (j = 0; j < h; j++)
      for (i = 0; i < w; i++)
      { // summ values over 4 squares
        // y1 = top left pixel
        if (mdh >= 0)
        { // find pixels for normal - horisontal - orientation
          l = mini(maxi(0, j - mdh),h - 1);
          k = mini(maxi(0, i - mdh),w - 1);
          pix = fim->getPixRef(l,k);
          y1 = pix->y;
          //pix = fsdim->getPixRef(l,k);
          //yv1 = sqri(pix->y);
          // y2 = bottom left pixel
          l = mini(maxi(0, j + mdh),h - 1);
          k = mini(maxi(0, i - mdh),w - 1);
          pix = fim->getPixRef(l,k);
          y2 = pix->y;
          //pix = fsdim->getPixRef(l,k);
          //yv2 = sqri(pix->y);
          // y3 top right
          l = mini(maxi(0, j - mdh),h - 1);
          k = mini(maxi(0, i + mdh),w - 1);
          pix = fim->getPixRef(l,k);
          y3 = pix->y;
          //pix = fsdim->getPixRef(l,k);
          //yv3 = sqri(pix->y);
          // y4 bottom right
          l = mini(maxi(0, j + mdh),h - 1);
          k = mini(maxi(0, i + mdh),w - 1);
          pix = fim->getPixRef(l,k);
          y4 = pix->y;
        }
        if (mdd >= 0)
        { // find pixels for diagomal orientation
          l = mini(maxi(0, j - mdd),h - 1);
          k = i; // same column
          pix = fim->getPixRef(l,k);
          f1 = pix->y;
          //pix = fsdim->getPixRef(l,k);
          //fv1 = sqri(pix->y);
          // f2 = left
          l = j; // same row
          k = mini(maxi(0, i - mdd),w - 1);
          pix = fim->getPixRef(l,k);
          f2 = pix->y;
          //pix = fsdim->getPixRef(l,k);
          //fv2 = sqri(pix->y);
          // f3 = right
          l = j; // same row
          k = mini(maxi(0, i + mdd),w - 1);
          pix = fim->getPixRef(l,k);
          f3 = pix->y;
          //pix = fsdim->getPixRef(l,k);
          //fv3 = sqri(pix->y);
          // f4 bottom
          l = mini(maxi(0, j + mdd),h - 1);
          k = i; // same column
          pix = fim->getPixRef(l,k);
          f4 = pix->y;
        }
        //pix = fsdim->getPixRef(l,k);
        //fv4 = sqri(pix->y);
        //f1234 = f1 + f2 + f3 + f4;
        //fv1234 = fv1 + fv2 + fv3 + fv4;
        // test for corners - 4 different types
        dw2ul = 0.0;
        dw2ur = 0.0;
        dw2top = 0.0;
        dw2left = 0.0;
        //
        if (mdh >= 0)
        { // horisontal guidemark test
          // new calculation based on max-difference
          mC34 = maxi(maxi(y1,y2),maxi(y3,y4)); // max intens
          if (assumeBlackIsZero)
          { // use difference between brightest and zero
            mC21 = 0; // assume black is zero intensity
            mDiff = (mC34 - 0)/3 + IntensDiffH; // minimum required diff
          }
          else
          { // use difference between brightest and darkest
            mC21 = mini(mini(y1,y2),mini(y3,y4)); // min intens
            mDiff = (mC34 - mC21)/2 + IntensDiffH; // minimum required diff
          }
          mC21 = y2-y1; // crit 1
          mC24 = y2-y4;
          mC31 = y3-y1;
          mC34 = y3-y4;
          //
          // try black upper right
          // find bb ww  y1 y3
          //      ww bb  y2 y4
          if ((mC21 > mDiff) and (mC24 > mDiff) and
              (mC31 > mDiff) and (mC34 > mDiff))
            dw2ul = (y2+y3-y1-y4);
          // try white upper right
          // find ww bb  y1 y3
          //      bb ww  y2 y4
          if ((mC21 < -mDiff) and (mC24 < -mDiff) and
              (mC31 < -mDiff) and (mC34 < -mDiff))
            dw2ur = (y1+y4-y2-y3);
        }
        // then diagonal test
        // difference for diagonal cross
        // find   bb     f1
        //      ww  ww  f2 f3
        //        bb     f4
        /*
        mfDiff = 0;
        mCf21 = 0;
        mCf24 = 0;
        mCf31 = 0;
        mCf34 = 0;
        */
        if (mdd >= 0)
        {
          mCf34 = maxi(maxi(f1,f2),maxi(f3,f4)); // max intens
          if (assumeBlackIsZero)
          { // use difference between brightest and zero
            mCf21 = 0; // assume black is zero intensity
            mfDiff = (mCf34 - mCf21)/3 + IntensDiffD; // minimum required diff
          }
          else
          { // use difference between brightest and darkest
            mCf21 = mini(mini(f1,f2),mini(f3,f4)); // min intens
            mfDiff = (mCf34 - mCf21)/2 + IntensDiffD; // minimum required diff
          }
          mCf21 = f2-f1; // crit 1
          mCf24 = f2-f4;
          mCf31 = f3-f1;
          mCf34 = f3-f4;
          // test
          if ((mCf21 > mfDiff) and (mCf24 > mfDiff) and
              (mCf31 > mfDiff) and (mCf34 > mfDiff))
            // corner detection found
            dw2top = (f2+f3-f1-f4);
          // find   ww     f1
          //      bb  bb  f2 f3
          //        ww     f4
          if ((mCf21 < -mfDiff) and (mCf24 < -mfDiff) and
              (mCf31 < -mfDiff) and (mCf34 < -mfDiff))
            dw2left = (f1+f4-f2-f3);
        }
        //
        // debug
        if ((dw2left + dw2top + dw2ur + dw2ul) > 1)
        { // there is a corner
          n++;
          if (fcn != NULL)
          {
            fprintf(fcn, " %3d,%3d, c %4.0f %4.0f %4.0f %4.0f (serial #%d)\n",
                        i,j, dw2ul, dw2ur, dw2top, dw2left, n);
            fprintf(fcn, "          y %4d %4d %4d %4d, f %4d %4d %4d %4d dy %3d, df %3d\n",
                              y1,y2,y3,y4,f1,f2,f3,f4, mDiff, mfDiff);
            /*
            fprintf(fcn, "          v %4d %4d %4d %4d, v %4d %4d %4d %4d\n",
                        int(sqrt(yv1)),int(sqrt(yv2)),
                        int(sqrt(yv3)),int(sqrt(yv4)),
                        int(sqrt(fv1)),int(sqrt(fv2)),
                        int(sqrt(fv3)),int(sqrt(fv4))); */
            fprintf(fcn, "          c %4d %4d %4d %4d, c %4d %4d %4d %4d\n",
                        mC21,mC24,mC31,mC34, mCf21,mCf24,mCf31,mCf34);
          }
        }
        //
        // debug end
        // mark pixel in result image
        // intensity > 0 if corner found
        rPix.y = mini(maxi(0, roundi((dw2ul + dw2ur + dw2top + dw2left)*32.0)), 255);
        // type of corner is marked by color
        // cyan'ish if black upper left
        // red'ish if white upper left
        rPix.u = mini(maxi(0, roundi((dw2ul-dw2ur)*250.0)+128), 255);
        rPix.v = mini(maxi(0, roundi((dw2top-dw2left)*250.0)+128), 255);
        // put pixel in result image
        cim->setPix(j, i, rPix);
      }
  }
  // *avgCornerInt = roundi(avgInt / (avgIntCount * 4.0));
  // debug
  if (fcn != NULL)
  {
    fprintf(fcn,"%% end\n");
    fclose(fcn);
  }
  //debug end
  return result;
}


////////////////////////////////////////////////

bool UCalibrate::findGmk( UCamMounted * cam,
                          UImage * img,
                          int frameSizeBlocks,
                          int codeBlockFactor, // normally 2
                          float sideStride, // size of a block (meter)
                          int maxCodesToLookFor,
                          bool findCode,
                          bool findPosition,
                          bool findRotation,
                          float chartCenterHeight,
                          bool findChartPos,
                          bool nearGMK, bool useVertFilt, bool useDiagFilt,
                          bool extraImages)
{
  bool result;
/*  UImage * img = NULL; // source image for analysis
  URawImage * rimg = NULL;*/
  //int averageIntensity, oldAverageIntensity;
  int validHits = 0;
  int i, j, n;
  float dx, dy;
  UPixel pix, pixi;
  const int MSL = 200;
  char s[MSL] = "";
  // barcode frame corners
  int k0, k1, k2;
  int k0dk1, k0dk2, k1dB, k2dB;
  // code result
  int codeNumber = 0;   // if more than one code in same image - for any purpose
  int code[MAX_BARCODE_LENGTH];
  int codeLng = 0;
  unsigned long intCode; // 4 byte code in one integer
  bool posFound; // barcode position found
  float appDist; // aproximate distance
  int borderIntensity; // intensity along border
  int guardBandV; // guard band when finding corners
  int guardBandD; // guard band when finding corners
  int connectivityMask; // when enumeration corner detections
  UPosition camPos, robPos;
  URotation camRot, robRot;
  int sm, sn;
  int intensCritXY, intensCritDiag; // corner detect criteris Vertical and Ddiagonal
  UCalibrationMark indexMark;   // index position of guidemark code
  char rc; // used in building filename for saved files
  bool assumeBlackIsZero;
  bool possiblyMoreCodes;
  UTime t1;
  //UImgPush * imgpush;
  //
  t1.Now();
  result = (img != NULL) and (cam != NULL);
  if (result)
  { // Do command
    // is command executable
    if (result)
      result = (img->valid and
          (findCode or findPosition or
          findRotation));
    if (result)
    { // there is something to do
      // filter source to filtered image
      if (extraImages)
      {  // copy full image
        pFiltImage->copy(img);
      }
      else
        // copy meta and size only
        pFiltImage->copyMeta(img, true);
      pCornerImage->copyMeta(img, true);
      strcpy(pFiltImage->name, "filt");
      strcpy(pCornerImage->name, "corner");
      //imgpush = (UImgPush*)pGridImage;
      if (nearGMK) // filter mask is 5, 9 or 25
      {
        //result = calib->LowPassFilter(img, calib->pFiltImage, calib->pFiltVarImage, 25, true);
        cvSmooth(img->cvArr(), pFiltImage->cvArr(), CV_BLUR, 5, 5, 0.0);
      }
      else
      {
        //result = calib->LowPassFilter(img,
        //            calib->pFiltImage, calib->pFiltVarImage, 5, true);
        cvSmooth(img->cvArr(), pFiltImage->cvArr(), CV_GAUSSIAN, 3, 3, 0.0);
      }
      pFiltImage->setColorType("YUV");
    }
    if (result)
    { // make an image where corners are marked with intensity > 0
      //averageIntensity = img->intensity;
      assumeBlackIsZero = false;
      if (img->width() > 320)
      { // high resolution has less focus => larger guard band
        guardBandV = 2;
        guardBandD = 3;
      }
      else
      { // downsampled images can tolerate smaller squares
        guardBandV = 2;
        guardBandD = 3;
      }
      if (nearGMK)
      { // more distance if larger mask
        guardBandV += 2;
        guardBandD += 2;
        // stronger criteria if near
        if (assumeBlackIsZero)
        {
          intensCritXY   = 0; // -5 .. 5
          intensCritDiag = 0; // -5 .. 5
        }
        else
        {
          intensCritXY   = 14; // 4 .. 20
          intensCritDiag = 15; // 4 .. 20
        }
      }
      else
      { // far away (best)
        if (assumeBlackIsZero)
        { // reduces bright corner-look-alikes
          intensCritXY   = -2; // -5 .. 5  // negative is more sensitive
          intensCritDiag = -2; // -5 .. 5
        }
        else
        { /* far - small mask */
          intensCritXY   = 7; //  4 .. 20
          intensCritDiag = 7; //  4 .. 20
        }
      }
      if (not useVertFilt)
        guardBandV = -2;
      if (not useDiagFilt)
        guardBandD = -2;
      // mark cornes using average intensity of corners
      // that may be different from image average intensity
      //oldAverageIntensity = averageIntensity;
      // mark only horizontal/vertical
      //tt.Now();
      result = MarkCornerDetections4(
          pFiltImage,   // low pass filtered image
            //calib->pFiltVarImage, // variance (SD) image
          pCornerImage, // result image
                //&averageIntensity,
          guardBandV,     // guard horizontal/vertical
          guardBandD,     // guard band diagonal
          intensCritXY,   // intens diff horiz/vert
          intensCritDiag, // intens diff diagonal
          assumeBlackIsZero); // Use dist white-zero i.p.o white-black
      // debug timing
      //printf("GMK timing %.2fms MarkCornerDetections4\n", tt.getTimePassed() * 1000.0);
      // debug timing end;
      //
    }
    if (result)
    { // combine corner detections to numbered calibration components
      // using a (4 pixel) connectivity mask.
      // result transferred to hits array with center of gravity and weight.
      cc.clear();
      // clear result dataset
      for (i = 0; i < MAX_CODES_IN_ONE_IMAGE; i++)
        gmks[i].clear(i);
      // smaller images are more crisp
      if (img->width() > 320)
        connectivityMask = 8;
      else
        connectivityMask = 4;
      //tt.Now();
      result = cc.EnumerateCalibrationDetections4(
          pCornerImage, connectivityMask);
      // debug timing
      // printf("GMK timing %.2fms cc.EnumerateCalibrationDetections4\n", tt.getTimePassed() * 1000.0);
      // debug timing end;
      if (extraImages)
      { // paint found corners in empty image
        for (i = 0; i < int(pCornerImage->width()); i++)
          for (j = 0; j < int(pCornerImage->height()); j++)
          { // change black to gray - to ease printout
            pix = pCornerImage->getPix(j,i);
            if (pix.y == 0)
            { // tone background with original in BW
              pixi = img->getPix(j,i);
              pix.y = 256 - 64 + ((pixi.y)/4);
              pix.u = 128;
              pix.v = 128;
            }
            pCornerImage->setPix(j,i,pix);
          }
        for (n = 0; n < cc.hitCount; n++)
        { // additionally paint center pixel for corner
          i = roundi(cc.hits[n].x);
          j = roundi(cc.hits[n].y);
          pix.y = 64;
          switch (cc.hits[n].filter)
          {
            case 1:
              pix.u = 255;
              pix.v = 255;
              break;
            case 2:
              pix.u = 0;
              pix.v = 255;
              break;
            case 3:
              pix.u = 0;
              pix.v = 0;
              break;
            case 4:
              pix.u = 255;
              pix.v = 0;
              break;
            default:
              pix.u = 128;
              pix.v = 128;
              break;
          }
          pCornerImage->setPix(j,i,pix);
        }
      }
      // debug
      if (false)
      { // debug save corner images
        snprintf(s, MAX_IMG_NAME_SIZE, "%s/im%06ldcornPre.bmp", imagePath, img->imageNumber);
        pCornerImage->saveBMP(s);
      }
      // tell imagepool that image is updated (may trigger push command)
      pCornerImage->imgUpdated();
      // debug end
      // there must be corners to continue
      result = result and (cc.hitCount > 0);
    }
    //
    if (img != NULL)
    { // be verbose (message on camera console)
      validHits = cc.hitCount;
      // snprintf(s, MSL,
      //          "Found %d corners in image %06ld",
      //          validHits, img->imageNumber);
      // toLog(s, 5);
    }
    //
    gmksCnt = 0;
    if (result and extraImages)
    { // prepare paint of grid/neighbor image
      pGridImage->copyToMaxRes(img);
      pGridImage->toRGB(NULL);
      pix.set(150, 150, 150); // gray
      pGridImage->tone(&pix,50);
      pGridImage->imgTime.Now();
      snprintf(pGridImage->name, MAX_IMG_NAME_SIZE, "grid%06ld", img->imageNumber);
    }
    // copy actual camera parameters to calibration object
    setCameraParameters(cam->getCamPar());
    //
    possiblyMoreCodes = result;
    //
    while (possiblyMoreCodes and (codeNumber < maxCodesToLookFor))
    { // find all or the right barcode - if more than one in image
      // set camera ref etc
      frameSet.clear();
      result = true;
      //
      if (result)
      { // find neighbors for each corner
        //tt.Now();
        result = cc.findClosestNeighbors(float(img->height()/5),
                              extraImages and (codeNumber == 0),
                              img, pGridImage);
        if ((not result) and (codeNumber == 0))
          snprintf(s, MSL, "find barcode neighbors failed");
        // debug timing
        //printf("GMK timing %.2fms cc.findClosestNeighbors\n", tt.getTimePassed() * 1000.0);
        // debug timing end;
        possiblyMoreCodes = result;
      }
      if (result)
      { // find square 2D barcode corners
        //tt.Now();
        result = cc.doOrderHits4(frameSizeBlocks - 2,
                                        &k0, &k1, &k2,
                                        &k0dk1, &k0dk2, &k1dB, &k2dB,
                                        extraImages,
                                        pGridImage, pGridImage->width()/img->width());
        // debug timing
        //printf("GMK timing %.2fms cc.doOrderHits4\n", tt.getTimePassed() * 1000.0);
        // debug timing end;
        //
        possiblyMoreCodes = result;
        // debug
        if (false and result)
        {
          printf("Code grid info - for code count %d\n", codeNumber);
          printf("K0(%d)k0->k1(%d)k0->k2(%d)\nK1(%d)k1->B(%d)\nK2(%d)k2->B(%d)\n",
                 k0, k0dk1, k0dk2, k1, k1dB, k2, k2dB);
          printf("K0: x%5.1f, y%5.1f (w%d)\n",
                 cc.hits[k0].x, cc.hits[k0].y, cc.hits[k0].w);
          printf("K1: x%5.1f, y%5.1f (w%d)\n",
                 cc.hits[k1].x, cc.hits[k1].y, cc.hits[k1].w);
          printf("K2: x%5.1f, y%5.1f (w%d)\n",
                 cc.hits[k2].x, cc.hits[k2].y, cc.hits[k2].w);
        }
        // debug end
        if ((not result) and (codeNumber == 0))
          snprintf(s, MSL, "No usable frame found (among %d corners)", cc.hitCount);
      }
      if (result)
      { // find barcode cell grid corners
        // frame corner square positions are saved in 'frameSet'
        frameSet.valid = false;
        //tt.Now();
        result = cc.doMakeCodeGrid(k0, k1, k2,
                                          k0dk1, k0dk2, k1dB, k2dB,
                                          frameSizeBlocks - 2,
                                          codeBlockFactor,
                                          sideStride,
                                          &frameSet,
                                          true,   // align positions to line
                                          false);  // save square to matlab file
/*        if (result)
          printf("- Found code grid OK\n");*/
        if ((not result) and (codeNumber == 0))
          snprintf(s, MSL, "Could not make intensity grid (for this frame)");
        // debug timing
        //printf("GMK timing %.2fms cc.doMakeCodeGrid\n", tt.getTimePassed() * 1000.0);
        // debug timing end;
        //
        if (false and extraImages and frameSet.valid)
        {
          //frameSet.setCameraRef(&camera);
          snprintf(s, MSL, "%sbarcode%06ld_%d",
                   dataPath, cam->getDev()->getImageNumber(), codeNumber);
          frameSet.saveMarksToFile(s,
                                   img->radialErrorRemoved,
                                   cam->getCamPar(),
                                   img->imageNumber);
        }
      }
      //
      //
      if (result and findCode)
      { // evaluate code
        //tt.Now();
        result = cc.doFindCodeCellIntensity(
            frameSizeBlocks - 2, codeBlockFactor,
            img,
            &borderIntensity,
            extraImages,
            pGridImage,
            codeNumber);
/*        if (result)
          printf("- Found code cell intensity OK\n");*/
        if (not result)
          snprintf(s, MSL, "Find GMK intensity failed for gmk %d", codeNumber);
        // debug timing
        //printf("GMK timing %.2fms cc.doFindCodeCellIntensity\n", tt.getTimePassed() * 1000.0);
        // debug timing end;
      }
      //
      if (result and findCode)
      { // find code
        //tt.Now();
        result = cc.doFindCodeValue(
            frameSizeBlocks - 2, codeBlockFactor,
            code, &codeLng, MAX_BARCODE_LENGTH,
            &intCode,
            false and extraImages,
            borderIntensity, &sm, &sn);
/*        if (result)
          printf("- Found code value OK\n");*/
        if ((not result) and (codeNumber == 0))
          snprintf(s, MSL,
                   "Did not find barcode value - orientation mark missing");
        if (result)
          frameSet.barcode = intCode;
        // debug timing
        //printf("GMK timing %.2fms cc.doFindCodeValue\n", tt.getTimePassed() * 1000.0);
        // debug timing end;
      }
      if (result)
      { // code is found - is it the right one
        if (findCode)
        { // estimate barcode position
          dx = cc.hits[k1].x - cc.hits[k2].x;
          dy = cc.hits[k1].y - cc.hits[k2].y;
          appDist = sideStride * float(frameSizeBlocks - 2) * 1.414 /
              hypot(dx, dy) * cam->getCamPar()->getFocalLength();
          // barcode or camera position calculation
          posFound = false;
          if (result and frameSet.valid)
          { // estimate camera or barcode position
            if (findPosition or findRotation)
            { // find camera position
              posFound = findCameraPosition(cam,
                                            &frameSet,      // frame data
                                            findPosition,   // camera pos/rot
                                            chartCenterHeight,
                                            extraImages);
              if (posFound and findRotation and frameSet.calValid)
              { // implement new camera position and send
                //if (not frameSet.implemented)
                { // store result in camera structure
                  // implement calibration result;
                  cam->setPosOnRobotCC(
                      &frameSet.posCal, &frameSet.rotCal);
                  // tell the client(s)
                  //cnn->sendCameraInfo(cam);
                  toLog("Position saved as new camera position!", 4);
                }
              }
            }
            if (findChartPos)
            { // find barcode position
              indexMark = cc.FindImagePosition(sm,sn);
              // debug
              // printf("Find Chart position (index:x:%6.1f, %6.1f)\n",
              //        indexMark.ix, indexMark.iy);
              // debug end
              //tt.Now();
              posFound = findBarcodeChartPosition(cam,
                  &frameSet,
                  &indexMark,
                  frameSizeBlocks,
                  sideStride,
                  pGridImage,
                  pGridImage->width() / img->width(),
                  not img->radialErrorRemoved,
                  extraImages and pGridImage->valid);
              // debug timing
              //printf("GMK timing %.2fms findBarcodeChartPosition\n", tt.getTimePassed() * 1000.0);
              // debug timing end;
              // debug end
              //
              if (posFound and findChartPos and frameSet.barcodeValid)
              { // save barcode position dataset.
                // do not save this type of data, as they are no
                // good for camera parameter estimation.
                // calib.calcamset.saveNewSet(&frameSet);
                //
                // change coordinates to "robot" type
                // @todo shift coordinates to robot perspective
                //       by using camera position
                robPos.set(-frameSet.posBarcode.z,
                           -frameSet.posBarcode.x,
                            frameSet.posBarcode.y);
                robRot.set(-frameSet.rotBarcode.Kappa,
                           -frameSet.rotBarcode.Omega,
                            frameSet.rotBarcode.Phi);
                //
                // save for later transmission
                gmks[gmksCnt].setData(code, codeLng, appDist, gmksCnt,
                                       frameSet.barcodeValid,
                                       robPos,
                                       robRot,
                                       //intCode,
                                       img->imgTime);
                gmksCnt++;
              }
            }
          }
          // if not full position, then just report code
          if (not posFound)
          {
            gmks[gmksCnt].setData(code, codeLng,
                                   appDist,
                                   gmksCnt,
                                   posFound,
                                   frameSet.posBarcode,
                                   frameSet.rotBarcode,
                                   //intCode,
                                   img->imgTime);
            gmksCnt++;
          }
          //
        }
      }
      if (result)
        // valid code - count number of valid codes in image
        codeNumber++;
      // remove these points as candidates for new search
      if (possiblyMoreCodes)
      { // mark corners for this code frame invalid
        // weight set to zero
        if ((k0 >= 0) and (k0 < cc.hitCount))
          cc.hits[k0].w = 0;
/*        if ((k1 >= 0) and (k1 < cc.hitCount))
          cc.hits[k1].w = 0;
        if ((k2 >= 0) and (k2 < cc.hitCount))
          cc.hits[k2].w = 0;*/
      }

      if (not result and (codeNumber == 0) and (strlen(s) > 0))
      { // show result of evaluating this code
        toLog(s, 5);
        //if ((strlen(resultMsg) + strlen(s)) < (MAX_CAM_INFO_SIZE-1))
        //  strcpy(resultMsg, s);
      }
      if (not possiblyMoreCodes)
      {
        if (gmksCnt == 0)
          snprintf(s, MSL,
                 "GMK found 0 codes in %.1fms (%d corners) in image %06ld",
                 t1.getTimePassed() * 1000.0,
                 validHits, img->imageNumber);
        else
          snprintf(s, MSL,
                 "GMK found %d (first %lu) in %.1fms (%d corners) in image %06ld",
                 gmksCnt, getGmk(0)->getCodeInt(),
                 t1.getTimePassed() * 1000.0,
                 validHits, img->imageNumber);
        toLog(s, 5);
      }
    } // end while
    //
    if (extraImages)
    {
      if (nearGMK)
        rc = 'n';
      else
        rc = 'f';
      // decoded data
#ifdef CALIB_DEBUG
      FILE * fl;
      snprintf(s, MSL ,"%simg%06ld%c.txt", imagePath,
               img->imageNumber, rc);
      fl = fopen(s, "w");
      if (fl != NULL)
      {
        for (i = 0; i < gmksCnt; i++)
        { //
          fprintf(fl, "Barcode in image %s\n", s);
          fprintf(fl, "Code (all %d chars) : ", strlen(gmks[i].getCode()));
          for (j = 0; j < (int)strlen(gmks[i].getCode()); j++)
            fprintf(fl, "%x", gmks[i].getCode()[j]);
          fprintf(fl, "\n");
          fprintf(fl, "Code as long       : %lx\n", gmks[i].getCodeInt());
          fprintf(fl, "Code is found as %d of %d codes in this image\n",
                  gmks[i].codeNumber, gmksCnt);
          fprintf(fl, "Apprroximate distance %f meter\n", gmks[i].appDist);
          gmks[i].getPos()->snprint("Barcode position", s, MSL);
          fprintf(fl, "%s (meter)\n", s);
          gmks[i].getRot()->snprint("Barcode rotation", true, s, MSL);
          fprintf(fl, "%s\n", s);
          gmks[i].getTime().getDateString(s, true);
          fprintf(fl, "Code time %s", s);
          gmks[i].getTime().getTimeAsString(s, true);
          fprintf(fl, " %s\n\n", s);
        }
        fclose(fl);
      }
#endif
    }
//    if (pGridImage != NULL)
//      pGridImage->imgUpdated();
    pGridImage->imgUpdated();
    pFiltImage->imgUpdated();
    pCornerImage->imgUpdated();
  }
  //
  return (codeNumber > 0);
}

////////////////////////////////////////////////////////////////

bool UCalibrate::findCameraPosition(UCamMounted * cam,
                                 UCalibrationMarkSet * dataset,
                                 bool findPosition, // also position - default is just rotation (and z)
                                 float chartCenterHeight, // find barchart position
                                 bool debug) // make logfiles etc.
{ // Find camera position assuming barcode chart is
  // not rotated relative to camera orientation.
  bool result = true;
  UPosition chtPos(0.0, chartCenterHeight, -1.0);
  URotation chtRot(0.0, 0.0, 0.0);
  URotation camRot(-6.445*M_PI/180.0,  0.726*M_PI/180.0,  -1.970*M_PI/180.0);
  UPosition camPos(0.0, 0.177,  0.0);
  //
  camPos = cam->getPosCC();
  camRot = cam->getRotCC();
  // stop if change is below these limits
  dataset->calValid = false;
  if (findPosition)
  { // less accurate if position is to be estimated as well
    dataset->setStopLimit(0.003, 0.002); // 3 mm change and 0.002 = 0.1 deg change
    result = dataset->evaluatePosRot("xyzOPK",
                                     cam->getCamPar(),
                                     &camPos, // 'old' position
                                     &camRot,
                                     &chtPos, debug);
  }
  else
  { // just rotation and distance z
    dataset->setStopLimit(0.0001, 0.0002); // 0.1 mm change and 0.0002 = 0.01 deg change
    result = dataset->evaluatePosRot("zOPK",
                                     cam->getCamPar(),
                                     &camPos, // 'old' position
                                     &camRot,
                                     &chtPos, debug);
  }
  // validate found result compared to expected value
  if (cam != NULL)
  { // get present camera
    camRot = cam->getRotCC();
    if (findPosition)
      camPos = cam->getPosCC();
    // distance to chart approx 1 meter
    if (((dataset->posCal.z - chtPos.z) < 1.5) and
         // positioned less than 15 cm from centerline
          (absd(dataset->posCal.x) < 0.25)) // and
        // camera height is within 10 cm of the expected value
        // (absd(dataset->posCal.y - camPos.y) < 0.1) and
        // rotation in Phi must be limited (< 4 degrees)
        //(absd(dataset->rotCal.Phi) < 0.07))
      dataset->calValid = true;
    else
      dataset->calValid = false;
  }
  return result;
}

//////////////////////////////////////////////////////////////////////////


bool UCalibrate::findBarcodeChartPosition(UCamMounted * cam,
                                       UCalibrationMarkSet * dataset,
                                       UCalibrationMark * indexCorner,
                                       int gmkBlocks,      // blocks in frame (typical 7)
                                       float gmkBlockSize, // size of one block (typical 0.02)
                                       UImage * img,        // debug image
                                       float pixFactor,      // size of debug image relative to source
                                       bool removeRadialError, // if not remover already
                                       bool debug
                                      )
{ // Find position of barcode chart
  bool result = true;
  UPosition chtPos;
  URotation chtRot;
  UPosition camPos;
  URotation camRot;
//  UMatrix4 mCtoR(4,4);
/*  const int MSL = 300;
  char s[MSL];*/
  int n;
  double d, best;
  UPixel rgb;
  float cx, cy;
  // debug
  CvFont font;
  const int SL = 20;
  char s[SL];
  //
  // find position and orientation of barcode chart relative to camera.
  // Here there is local optima in rotation around both x (omega) and y (phi)
  // if the rotation is more than a few degrees.
  result = true;
  if (result)
  { // find the position of the barcode
    // initial guess
    chtPos.set(0.0, 0.0, -1.0);
    // 0.1 mm change and 0.0002 = 0.01 deg change is used as iteration stop limits
    // no initial guess of rotation to allow all rotation possibilities
    result = dataset->evaluateChartPosRot(&chtPos, NULL,
                                           0.0001, 0.0002,
                                           cam->getCamPar(),
                                           removeRadialError,
                                           debug);
    if (result)
    { // Adjust Kappa after index mark position.
      // Now pos- and rotBarcode is in camera
      // coordiantes.
      chtRot = dataset->rotBarcode;
      best = img->width(); // distance from index in pixels.
      // corner is in top-right position
      indexCorner->ry = float(gmkBlocks - 4)/2.0 * gmkBlockSize;
      indexCorner->rx = -indexCorner->ry; // negative x, positive y
      if (debug)
      { // paint position of (top-left) of first index
        // corner cell
        //rgb.setPixRGB(255,128,128);
        cvCircle(img->cvArr(),
                 cvPoint(int(indexCorner->ix * pixFactor),
                         int(indexCorner->iy * pixFactor)),
                 2, CV_RGB(255, 128, 128));
        //rgb.SetPixRGB(255,128,255);
        cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0.0, 1, 8);
        snprintf(s, SL, "%lu", dataset->barcode);
        cvPutText(img->cvArr(), s,
                  cvPoint(roundi(indexCorner->ix * pixFactor),
                          roundi(indexCorner->iy * pixFactor)),
                  &font, CV_RGB(255,255,255));
      }
      for (n = 0; n < 4; n++)
      { // four possible positions of index
        d = dataset->getErrorInPixels(indexCorner,
                                      &cx, &cy, cam->getCamPar(), removeRadialError);
        if (debug)
          // paint circle at the calculated 4 corners
          cvCircle(img->cvArr(),
                   cvPoint(int(cx * pixFactor),
                           int(cy * pixFactor)),
                   4, CV_RGB(255, 128, 255));

        if (d < best)
        { // this is closer - save
          best = d;
          chtRot = dataset->rotBarcode;
        }
        // rotate 90 deg
        dataset->rotBarcode.Kappa += M_PI/2.0;
        dataset->rotBarcode.LimitToPi();
      }
      // implement best
      dataset->rotBarcode = chtRot;
    }
    if (result)
    { // now convert to robot position
      if (dataset->calValid)
      { // use new position
        camPos = dataset->posCal;
        camRot = dataset->rotCal;
      }
      else
      { // get position from camera
        camRot = cam->getRotCC();
        camPos = cam->getPosCC();
      }
    }
  }
  dataset->barcodeValid = result;
  //
  return result;
}


