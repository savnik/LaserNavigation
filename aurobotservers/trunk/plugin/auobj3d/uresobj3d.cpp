/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)                        *
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

#include <stdio.h>
#include <math.h>

//#include <ugen4/ucommon.h>
#include <ugen4/u2dline.h>
#include <urob4/uvarcalc.h>
#include <ugen4/uimg3dpoint.h>
#include <ugen4/uimage.h>
#include <urob4/uimagepool.h>
#include <urob4/uresposehist.h>

#include "uresobj3d.h"

class UGridSegEq
{
public:
  UGridSegEq()
  {
    csCnt = 0;
  };
  /// maximum number of merged clusters
  static const int MaxEqC = 80;
  /// cluster equvivalent list
  int cs[MaxEqC];
  /// number of found equvivallences
  int csCnt;
  /////////////////////////////////
  /// test if this number is in list
  bool isInList(int c)
  {
    int i;
    for (i = 0; i < csCnt; i++)
    {
      if (cs[i] == c)
        break;
    }
    return (i < csCnt);
  };
  /////////////////////////////////
  /// add this serial number to list if not there already
  void add(int c)
  {
    if (csCnt < MaxEqC and not isInList(c))
      cs[csCnt++] = c;
  }
  ////////////////////////////////
  /// add this list of numbers
  void addList(int * ee, int eeCnt)
  {
    int e;
    for (e = 0; e < eeCnt; e++)
    {
      if (ee[e] > 0)
        add(ee[e]);
    }
  }
  /////////////////////////////////
  /// clear list of equvivalent serial numbers
  void clear()
  {
    csCnt = 0;
  }
  /////////////////////////////////
  /// match if any number in these lists are equal
  bool match(UGridSegEq * s2)
  {
    bool found = false;
    int i;
    for (i = 0; i < csCnt; i++)
    {
      if (s2->isInList(cs[i]))
      {
        found = true;
        break;
      }
    }
    return found;
  };
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

class UGridSegs
{
public:
  /// constructor
  UGridSegs()
  {
    int i;
    ccCnt = 0;
    for (i = 0; i < MaxClustColl; i++)
      cc[i] = NULL;
    serial = 1;
  };
  /// destructor
  ~UGridSegs()
  {
    int i;
    for (i = 0; i < MaxClustColl; i++)
    {
      if (cc[i] != NULL)
      {
        delete cc[i];
        cc[i] = NULL; // to ease debug
      }
    }
  };
  /// calss variables
  /// max number of (unique) cluster collisions
  static const int MaxClustColl = 500;
  /// array of cluster collisions
  UGridSegEq * cc[MaxClustColl];
  /// number of used collision elements
  int ccCnt;
  /// next serial number to use.
  int serial;
  /// clear cluster collision list
  void clear()
  {
    ccCnt = 0;
    serial = 1;
  };
  /// process this set of cluster numbers
  int addEquvivalent(int * iee, int ieeCnt)
  {
    int e, i, c, n, m;
    UGridSegEq see;
    UGridSegEq * se = NULL;
    bool added = false;
    int * ee, eeCnt;
    //
    see.addList(iee, ieeCnt);
    ee = see.cs;
    eeCnt = see.csCnt;
    n = 0;
    m = 0;
    for (e = 0; e < eeCnt; e++)
    {
      c = ee[e];
      if (c > 0)
      {
        for (i = 0; i < ccCnt; i++)
        {
          se = cc[i];
          if (se->isInList(c))
          {
            se->addList(ee, eeCnt);
            added = true;
            break;
          }
        }
        // save usable serial number
        n = c;
        m++; // count valid numbers
        if (added) break;
      }
    }
    if (not added and ccCnt < MaxClustColl and m > 1)
    {
      if (cc[ccCnt] == NULL)
        cc[ccCnt] = new UGridSegEq();
      se = cc[ccCnt]; 
      se->addList(ee, eeCnt);
      ccCnt++;
    }
    if (n == 0)
      n = serial++;
    // else
      // else only one cluster number, and this is in n already-
    //
    return n;
  };
  /// simplify collision list
  void simplifyList()
  {
    int i, j;
    UGridSegEq *ge, *ge2;
    //
    for (i = 0; i < ccCnt - 1; i++)
    { // test for multible collision tests
      ge = cc[i];
      if (ge->csCnt > 0)
      { // this is not empty
        for (j = i + 1; j < ccCnt; j++)
        { // test against all remaining collisions
          ge2 = cc[j];
          if (ge->match(ge2))
          { // supplement this list with numbers from other list
            ge->addList(ge2->cs, ge2->csCnt);
            ge2->csCnt = 0;
          }
        }
      }
    }
  };
  /// get new cluster number replacing old
  int getAndCountNumber(int old)
  {
    int i, c = old;
    UGridSegEq *ge;
    for (i = 0; i < ccCnt; i++)
    {
      ge = cc[i];
      if (ge->csCnt > 0)
        if (ge->isInList(old))
        {
          c = ge->cs[0];
          break;
        }
    }
    return c;
  }
};


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/**
 * Bounding box for a number of voxels */
class UGridBBox
{
public:
  // constructor
  UGridBBox()
  {
    valid = true;
    footprint = new UPolygon40();
    profile = new UPolygon40();
    sideView = new UPolygon40();
    footprintConvex = false;
    colCnt = 0;
  };
  /// destructor
  ~UGridBBox()
  {
    delete footprint;
    if (profile != NULL)
      delete profile;
    if (sideView != NULL)
      delete sideView;
  };
  /// clear
  void clear()
  {
    valid = true;
    hitCnt = 0;
    footprint->clear();
    if (sideView != NULL)
      sideView->clear();
    if (profile != NULL)
      profile->clear();
    footprintConvex = false;
    human = false;
    roof = false;
    filtered = false;
    colCnt = 0;
  }
  /// is box valid
  bool valid;
  /// is box filtered
  bool filtered;
  /// limits of box in voxel index values
  int minx, maxx, miny, maxy, minz, maxz;
  /// count of cells
  int hitCnt;
  /// original cluster number
  int serial;
  /// sample pixels in box
  UImg3Dpoint * pix;
  /// polygon for the foodprint of the box
  UPolygon40 * footprint, *sideView, *profile;
  /// is the footprint convex
  bool footprintConvex;
  /// is likeky to be a human
  bool human;
  /// is a roof or ignorable high obstacle
  bool roof;
  /// number of pixels in average array
  static const int MCL = 20;
  /** array of pixels sorted in intensity order,
   * a few is kept only, but should give a fair colour representation of
   * the polygin */
  int col[MCL];
  /**
   * Number of colours in the col array */
  int colCnt;
  ////////////////////////////
  /// add this cell to the bounding box
  void addCell(int ix, int iy, int iz, UImg3Dpoint * v)
  {
    if (hitCnt == 0)
    {
      minx = ix;
      maxx = ix;
      miny = iy;
      maxy = iy;
      minz = iz;
      maxz = iz;
      serial = v->q;
    }
    else
    {
      if (ix < minx) minx = ix;
      if (ix > maxx) maxx = ix;
      if (iy < miny) miny = iy;
      if (iy > maxy) maxy = iy;
      if (iz < minz) minz = iz;
      if (iz > maxz) maxz = iz;
    }
    pix = v;
    hitCnt++;
    valid = true;
    addPosToFootprint(&pix->pos);
    addPosToProfile(&pix->pos, false);
    addPosToSideView(&pix->pos, false);
    addCol(v->pixLeft);
  }
  /////////////////////////////////////////////
  /// add a position to the footprint of this box.
  /// if too full, then the points are reduced to a convex
  void addPosToFootprint(UPosition * pos)
  { // is there space for another point
    double dist;
    UPosition pz;
    //
    if (footprint->getPointsCnt() == 40)
    { // need to reduce the vertex count - do this
      // by reducing to a convex polygon
      reduceToConvexFootprint();
      dist = 0.4;
      while (footprint->getPointsCnt() == 40)
      { // is still a too big polygon
        // so simplify
        footprint->removeNearVertex(dist);
        dist += 0.2;
      }
    }
    if (footprint->getPointsCnt() < 40)
    {
      footprint->add(pos);
      footprintConvex = false;
    }
    else
    { // debug
      printf("UGridBox::addCell: Footprint polygon too full - ignored point\n");
      // debug end
    }
  };
  /////////////////////////////////////////////
  /// add a position to the footprint of this box.
  /// if too full, then the points are reduced to a convex
  /// \param pos is the position to add to polygon
  /// \param isProfile true if the coordinates are exchanged to side view
  void addPosToSideView(UPosition * pos, bool isSideView)
  { // is there space for another point
    double dist;
    UPosition pz;
    //
    if (sideView != NULL)
    {
      if (sideView->getPointsCnt() == 40)
      { // need to reduce the vertex count - do this
        // by reducing to a convex polygon
        reduceToConvexFootprint();
        // ensure count is less than 40
        dist = 0.4;
        while (sideView->getPointsCnt() == 40)
        { // is still a too big polygon
          sideView->removeNearVertex(dist);
          dist += 0.2;
        }
      }
      if (sideView->getPointsCnt() < 40)
      { // make x forward and y up
        if (isSideView)
          sideView->add(pos);
        else
        {
          pz.x = pos->x;
          pz.y = pos->z;
          pz.z = pos->y;
          sideView->add(&pz);
        }
        footprintConvex = false;
      }
      else
      { // debug
        printf("UGridBox::addCell: Sideview polygon too full - ignored point\n");
        // debug end
      }
    }
  };
  /////////////////////////////////////////////
  /// add a position to the footprint of this box.
  /// if too full, then the points are reduced to a convex
  /// \param pos is the position to add to polygon
  /// \param isProfile true if the coordinates are exchanged to profile view
  void addPosToProfile(UPosition * pos, bool isProfile)
  { // is there space for another point
    double dist;
    UPosition pz;
    //
    if (profile != NULL)
    {
      if (profile->getPointsCnt() == 40)
      { // need to reduce the vertex count - do this
        // by reducing to a convex polygon
        reduceToConvexFootprint();
        // ensure count is less than 40
        dist = 0.4;
        while (profile->getPointsCnt() == 40)
        { // is still a too big polygon
          profile->removeNearVertex(dist);
          dist += 0.2;
        }
      }
      if (profile->getPointsCnt() < 40)
      { // make x positive right and y up
        if (isProfile)
          profile->add(pos);
        else
        {
          pz.x = -pos->y;
          pz.y = pos->z;
          pz.z = pos->x;
          profile->add(&pz);
        }
        footprintConvex = false;
      }
      else
      { // debug
        printf("UGridBox::addCell: profile polygon too full - ignored point\n");
        // debug end
      }
    }
  };
  /////////////////////////////////
  /// is this box valid or not, based on size
  void filter(int minCellCnt, int maxDX, int maxDY,
              int maxHeight, int minHeight, int minX,
              int roofLimit,
              double minDens, int minDensCnt)
  {
    double d;
    valid = hitCnt >= minCellCnt;
    if (valid and hitCnt < minDensCnt)
    { // test density too
      d = getDensity(footprint);
      valid = d > minDens;
    }
    if (valid)
    {
      human = maxz <= maxHeight and
              maxz >= minHeight and
              (maxx - minx) < maxDX and
              (maxy - miny) < maxDY and
              minx >= minX;
      roof = minz >= roofLimit;
    }
    filtered = true;
  };
  ////////////////////////////////////////
  double getDensity(UPolygon * pgn)
  {
    double a, dens;
    a = pgn->getXYarea();
    dens = hitCnt/a;
    return dens;
  };
  /////////////////////////////////////////
  /// test if this box overlaps another box
  /// returns true if so
  bool hasOverlap(UGridBBox * nb, int allowedZdist)
  {
    bool result;
    if (not footprintConvex)
      reduceToConvexFootprint();
    if (not nb->footprintConvex)
      nb->reduceToConvexFootprint();
    result = footprint->isOverlappingXY(nb->footprint);
    // NB! embedded is not the same as overlap, so ..
    if (not result)
      result = footprint->isEmbedded(nb->footprint, NULL);
    if (not result)
      result = nb->footprint->isEmbedded(footprint, NULL);
    //
    if (result)
      // should not be separated in height
      result = maxz > (nb->minz - allowedZdist) and
               minz < (nb->maxz + allowedZdist);
    return result;
  };
  /////////////////////////////////////////
  /// Merge the other box into this
  /// \param other assimilated box
  void merge(UGridBBox * other)
  {
    int i;
    UPosition * p;
    //
    minx = mini(minx, other->minx);
    maxx = maxi(maxx, other->maxx);
    miny = mini(miny, other->miny);
    maxy = maxi(maxy, other->maxy);
    minz = mini(minz, other->minz);
    maxz = maxi(maxz, other->maxz);
    hitCnt += other->hitCnt;
    p = other->footprint->getPoints();
    for (i = 0; i < other->footprint->getPointsCnt(); i++)
    {
      addPosToFootprint(p);
      p++;
    }
    p = other->sideView->getPoints();
    for (i = 0; i < other->sideView->getPointsCnt(); i++)
    {
      addPosToSideView(p, true);
      p++;
    }
    p = other->profile->getPoints();
    for (i = 0; i < other->profile->getPointsCnt(); i++)
    {
      addPosToProfile(p, true);
      p++;
    }
  }
  /////////////////////////////////
  /// merge is not allowed if resulting y-size is above maxdy cells
  bool allowMerge(UGridBBox * nb, int maxdy)
  { // allow merge with wider boxes, if width is not increased
    bool result;
    int oldw, neww;
    oldw = maxi(maxy - miny, nb->maxy - nb->miny);
    neww = maxi(maxy, nb->maxy) - mini(miny, nb->miny);
    result = (neww <= maxdy or neww == oldw);
    return result;
  };
  ///////////////////////////////////
  /// reduce the footprint list of cell-points to a
  /// convex footprint, using the x,y coordinates only
  void reduceToConvexFootprint()
  {
    UPolygon40 * po2;
    //
    if (not footprintConvex)
    {
      po2 = new UPolygon40();
      footprint->extractConvexTo(po2);
      delete footprint;
      footprint = po2;
      if (sideView != NULL)
      {
        po2 = new UPolygon40();
        sideView->extractConvexTo(po2);
        delete sideView;
        sideView = po2;
      }
      if (profile != NULL)
      {
        po2 = new UPolygon40();
        profile->extractConvexTo(po2);
        delete profile;
        profile = po2;
      }
      footprintConvex = true;
    }
  };
  /**
   * Add colour sample to the total samples of this blob,
   * if array is full, then remove one from both ends.
   * So that it is possible to find the a sort of median color
   * at the end. based on Red-Green only.
   * The color order is assumed to be 8-bit blue;green;red (i.e. red is lower 8 bits) */
  void addCol(int pixel)
  {
    int i;
    int rg = (pixel >> 8 & 0xff) + (pixel & 0xff);
    int * pc = col, nrg;
    //
    if (colCnt == MCL)
    { // too long - remove 2
      memmove(col, &col[1], (MCL - 2) * sizeof(int));
      colCnt = MCL - 2;
    }
    for (i = 0; i < colCnt; i++)
    {
      nrg = (*pc >> 8 & 0xff) + (*pc & 0xff);
      if (rg < nrg)
        break;
      pc++;
    }
    if (i < colCnt)
    { // move remaining pixels one up
      memmove(&col[i+1], &col[i], (colCnt - i) * sizeof(int));
    }
    col[i] = pixel;
    colCnt++;
  };
  /**
   * Get representative colour for this bounding box */
  int getCol()
  {
    return col[colCnt / 2];
  }
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/**
 * list of bounding boxes */
class UGridBBoxes
{
public:
  /// constructor
  UGridBBoxes()
  {
    bbCnt = 0;
  };
  /// destructor
  ~UGridBBoxes()
  {
    int i;
    for (i = 0; i < MBB; i++)
      if (bb[i] != NULL)
        delete bb[i];
  };
  /// clear box list
  void clear()
  {
    bbCnt = 0;
  }
  /// global variables
  static const int MBB = 10000;
  /// box-list
  UGridBBox * bb[MBB];
  /// number of boxes esatblished
  int bbCnt;
  //////////////////////////
  /// find box based on this serial number
  ///\param cluster serial number
  ///\returns pointer to bounding box
  UGridBBox * getBBox(int cluster)
  {
    UGridBBox * b = NULL;
    int i;
    //
    for (i = 0; i < bbCnt; i++)
    {
      b = bb[i];
      if (b->serial == cluster)
        break;
    }
    if (i == bbCnt)
    { // no bounding box for this number
      if (i < MBB)
      {
        if (bb[i] == NULL)
          bb[i] = new UGridBBox();
        b = bb[i];
        b->clear();
        bbCnt++;
      }
      else
      {
        printf("UGridBBoxes::getBBox:: No more space for new bounding-boxes\n");
        b = NULL;
      }
    }
    return b;
  }
  //////////////////////////////////
  /// filter boxes after size and hit-count
  int filter(int minCellCnt, int maxDX, int maxDY, int maxZ, int minZ, int minX,
             int roofLimit,
             double minDens, int minDensCnt)
  {
    int i, n = 0;
    UGridBBox * b;
    //
    for (i = 0; i < bbCnt; i++)
    {
      b = bb[i];
      if (b->valid)
      {
        b->filter(minCellCnt, maxDX, maxDY, maxZ, minZ, minX, roofLimit, minDens, minDensCnt);
        if (b->valid)
          n++;
      }
    }
    return n;
  };
  ////////////////////////////////////
  /// make boxes larger by mergind boxes with overlap
  /// NB! not this merge should perhaps be repeated until no more reduce
  int mergeOverlappingBBs(int maxdy, int allowedZdist)
  {
    UGridBBox ** b1, **b2;
    int i, j, n = 0, m = 0;
    bool ovl;
    //
    b1 = bb;
    for (i = 0; i < bbCnt - 1; i++)
    {
      if ((*b1)->valid)
      {
        b2 = &bb[i + 1];
        for (j = i + 1; j < bbCnt; j++)
        {
          if ((*b2)->valid)
          {
            ovl = (*b1)->hasOverlap(*b2, allowedZdist);
            if (ovl)
            {
              // do not allow merge if result is too wide
              if ((*b1)->allowMerge(*b2, maxdy))
              {
                (*b1)->merge(*b2);
                (*b2)->valid = false;
                n++;
              }
            }
          }
          b2++;
        }
        m++;
      }
      b1++;
    }
    //printf("Removed %d of %d boxes by merge, now %d left\n", n, bbCnt, m);
    return m;
  };
  /////////////////////////////////////////
  /// reduce the footprint list of cell-points to a
  /// convex footprint, using the x,y coordinates only
  void reduceToConvexFootprint()
  {
    int i;
    UGridBBox * bx;
    //
    for (i = 0; i < bbCnt; i++)
    {
      bx = bb[i];
      if (bx->valid)
        bx->reduceToConvexFootprint();
    }
  };
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

UGrid3D::UGrid3D()
{
  voxels = NULL;
  voxelsCnt = 0;
  xDepth = 0;
  yWidth = 0;
  zHeight = 0;
  voxelSize = -0.1; // not legal
  boxes = NULL;
  segc = NULL;
}

/////////////////////////////////////////////////////////

UGrid3D::~UGrid3D()
{
  if (voxels != NULL)
  {
    voxelsCnt = 0;
    xDepth = 0;
    yWidth = 0;
    zHeight = 0;
    free(voxels);
    voxels = NULL;
  }
  if (segc != NULL)
    delete segc;
  if (boxes != NULL)
    delete boxes;
}

/////////////////////////////////////////////////////////

void UGrid3D::clear()
{
  int i;
  UImg3Dpoint ** p = voxels;
  for (i = 0; i < voxelsCnt; i++)
  {
    *p = NULL;
    p++;
  }
}

/////////////////////////////////////////////////////////

bool UGrid3D::setVolumen(double cellSize, double depth, double width, double height)
{
  int newCnt;
  //
  if (cellSize != voxelSize)
  {
    if (cellSize < 1e-4)
      voxelSize = 1e-4;
    else
      voxelSize = cellSize;
    xDepth = roundi(depth/voxelSize);
    yWidth = roundi(width/voxelSize);
    zHeight = roundi(height/voxelSize);
    newCnt = xDepth * yWidth * zHeight;
    if (newCnt > voxelsCnt)
      voxels = (UImg3Dpoint **)realloc(voxels, newCnt * sizeof(UImg3Dpoint *));
    if (voxels != NULL)
    {
      voxelsCnt = newCnt;
    }
    else
    {
      voxelsCnt = 0;
      xDepth = 0;
      yWidth = 0;
      zHeight = 0;
      printf("Error voxel allocation of %d voxels failed!!!!\n", newCnt);
    }
  }
  clear();
  return voxels != NULL;
}

//////////////////////////////////////////////////////////

void UGrid3D::setVoxel(UPlane * plane, UImg3Dpoint * pnt)
{
  int x = roundi(pnt->pos.x / voxelSize);
  int y = roundi(pnt->pos.y / voxelSize);
  double d;
  UImg3Dpoint ** pp;
  int z;
  // change to plane coordinates - z-value only
  d = plane->distSigned(pnt->pos);
  //pnt->pos.z = d;
  z = roundi(d / voxelSize);
  //
  pp = getVoxelPnt(x,y + yWidth/2,z);
  if (pp != NULL)
  {
    *pp = pnt;
    pnt->q = 0;
  }
}

//////////////////////////////////////////////////////////

UImg3Dpoint ** UGrid3D::getVoxelPnt(int vx, int vy, int vz)
{
  UImg3Dpoint ** result = NULL;
  //
  if (vx < xDepth and vy < yWidth and vz < zHeight and vx >= 0 and vy >= 0 and vz >= 0)
  {
    int idx = vx + vy * xDepth + vz * xDepth * yWidth;
    result = &voxels[idx];
  }
  return result;
}

//////////////////////////////////////////////////////////

int UGrid3D::countNonZeroVoxels()
{
  int i = 0;
  int j, n;
  UImg3Dpoint ** p = voxels;
  //
  n = xDepth * yWidth * zHeight;
  for (j = 0; j < n; j++)
  {
    if (*p != NULL)
      i++;
    p++;
  }
  return i;
}

//////////////////////////////////////////////////////////

int UGrid3D::voxelSegmentation(double minZ, UImage * maskImg, double maskSideMult)
{
  int ix, iy, iz, j, k;
  UImg3Dpoint **v, **vv;
  UGridBBox * box;
  int miz, r, c;
  unsigned char pv = 0;
  //
  const int MEC = 13;
  int mee[MEC];
  int meeCnt = 0;
  miz = roundi(minZ / voxelSize);
  if (segc == NULL)
    segc = new UGridSegs();
  segc->clear();
  for (iz = zHeight - 2; iz >= miz; iz--)
  {
    for (iy = yWidth - 2; iy > 0; iy--)
    {
      for (ix = 1; ix < xDepth - 2; ix++)
      {
        meeCnt = 0;
        v = getVoxelPnt(ix, iy, iz);
        if (*v != NULL)
        {
          if (maskImg != NULL)
          {
            r = int(iy * maskSideMult);
            c = int(ix * maskSideMult);
            pv = *maskImg->getUCharRef(r, c);
          }
          if (pv > 0)
            // invalid pixel
            (*v)->q = 0;
          else
          {
            if (v[-1] != NULL) mee[meeCnt++] = v[-1]->q;
            vv = v + xDepth - 1;
            for (j = 0; j < 3; j++)
              if (vv[j] != NULL) mee[meeCnt++] = vv[j]->q;
            vv = v + xDepth * yWidth -1 - xDepth;
            for (k = 0; k < 3; k++)
            {
              for (j = 0; j < 3; j++)
                if (vv[j] != NULL) mee[meeCnt++] = vv[j]->q;
              vv = vv + xDepth;
            }
            (*v)->q = segc->addEquvivalent(mee, meeCnt);
          }
        }
      }
    }
  }
  // now create bounding boxes
  if (boxes == NULL)
    boxes = new UGridBBoxes();
  else
    boxes->clear();
  //
  for (iz = zHeight - 1; iz >= miz; iz--)
  {
    for (iy = yWidth - 2; iy > 0; iy--)
    {
      for (ix = 1; ix < xDepth - 2; ix++)
      {
        meeCnt = 0;
        v = getVoxelPnt(ix, iy, iz);
        if (*v != NULL)
        {
          if ((*v)->q > 0)
          {
            box = boxes->getBBox((*v)->q);
            if (box != NULL)
              box->addCell(ix, iy, iz, *v);
          }
        }
      }
    }
  }
  // make profiles convex - if not already
  boxes->reduceToConvexFootprint();
  //
  printf("Found %d bouding boxes\n", boxes->bbCnt);
  //
  return boxes->bbCnt;
}

//////////////////////////////////////////////////////////

int UGrid3D::filter(int minCellCnt, double maxDX, double maxDY,
                    double maxZ, double minZ, double minX,
                    double roofHgt,
                    double minDens, int minDensCnt)
{
  int miz, maz, mady, madx, mix, roofLimit;
  // convert limits to cells
  maz = roundi(maxZ / voxelSize);
  miz = roundi(minZ / voxelSize);
  mady = roundi(maxDY / voxelSize);
  madx = roundi(maxDX / voxelSize);
  mix = roundi(minX / voxelSize);
  roofLimit = roundi(roofHgt / voxelSize);
  //
  if (boxes != NULL)
    return boxes->filter(minCellCnt, madx, mady, maz, miz, mix,
                         roofLimit, minDens, minDensCnt);
  else
    return 0;
}

//////////////////////////////////////////////////////////

void UGrid3D::paintVoxels(UImage * img, int minQValue, bool andBBoxes, int minCellCnt)
{
  int ix, iy, iz, n, m, i;
  CvScalar col;
  CvScalar red=CV_RGB(255,0,0);
  CvScalar lightblue=CV_RGB(100,100,255);
  CvScalar blue=CV_RGB(0,0,255);
  //CvScalar yellow=CV_RGB(140,140,0);
  const int MPL = 40; // max polygon edge count
  CvPoint p1, p2, p0, p0s, poly[MPL];
  CvPoint *pop;
  double ppm = img->width()/(xDepth*voxelSize + 1.0);
  UGridBBox * bb;
  CvFont font;
  UPosition pNBR; // cell corner with less values of each axis (near bottom right)
  UImg3Dpoint * pc;
  int vSize = roundi(voxelSize * ppm);
  const int MSL = 100;
  char s[MSL];
  UPosition * pos;
  //
  // top-view
  p0.x = 5;
  p0.y = 300;
  // side view above
  p0s.x = 5;
  p0s.y = 300 - roundi(7.0 * ppm);
  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
               0.5, 0.5, 0.0, 1, 8);
  // paint coordinmate system for side view
  img->paintGridAligned(p0.y, p0.x, ppm, 5);
  p1.x = img->width() - 1;
  p1.y = p0s.y;
  cvLine(img->cvArr(), p0s, p1, blue, 4, 2);
  p1.x = p0s.x;
  p1.y = 0;
  cvLine(img->cvArr(), p0s, p1, blue, 4, 2);
  //
  for (iz = 0; iz < zHeight; iz++)
    for (ix = 0; ix < xDepth; ix++)
      for (iy = 0; iy < yWidth; iy++)
      {
        pc = *getVoxelPnt(ix, iy, iz);
        if (pc != NULL and pc->q >= minQValue)
        { // get one cornet of cell
          pNBR.x = voxelSize * (double(ix) - 0.5);
          pNBR.y = voxelSize * (double(iy - yWidth/2) - 0.5);
          pNBR.z = voxelSize * (double(iz) - 0.5);
          //
          p1.x = p0.x + roundi(pNBR.x * ppm);
          p1.y = p0.y - roundi(pNBR.y * ppm);
          p2.x = p1.x + vSize;
          p2.y = p1.y - vSize;
          if (p1.x < 640)
          {
            col = CV_RGB(pc->red(), pc->green(), pc->blue());
            cvRectangle(img->cvArr(), p1, p2, col, CV_FILLED, 4);
            // paint side view too
            p1.x = p0s.x + roundi(pNBR.x * ppm);
            p1.y = p0s.y - roundi(pNBR.z * ppm);
            p2.x = p1.x + vSize;
            p2.y = p1.y - vSize;
            cvRectangle(img->cvArr(), p1, p2, col, CV_FILLED, 4);
          }
        }
      }
  // paint also bounding boxes
  if(andBBoxes and boxes != NULL)
  {
    for (n = 0; n < boxes->bbCnt; n++)
    {
      bb = boxes->bb[n];
      if (bb->valid and bb->hitCnt > minCellCnt)
      {
        pNBR.x = voxelSize * (double(bb->minx) - 0.5);
        pNBR.y = voxelSize * (double(bb->miny - yWidth/2) - 0.5);
        pNBR.z = voxelSize * (double(bb->minz) - 0.5);
        p1.x = p0.x + roundi(pNBR.x * ppm);
        p1.y = p0.y - roundi(pNBR.y * ppm);
        p2.x = p1.x + vSize * (bb->maxx - bb->minx + 1);
        p2.y = p1.y - vSize * (bb->maxy - bb->miny + 1);
        if (p1.x < 640)
        {
          if (bb->filtered)
          {
            if (bb->roof)
              col = lightblue;
            else if (bb->human)
              col = red;
            else
              col = blue;
          }
          else
            col = blue;
          cvRectangle(img->cvArr(), p1, p2, col, 1, 4);
          p1.y -= 2;
          snprintf(s, MSL, "%d:%.1f", bb->hitCnt, bb->getDensity(bb->footprint));
          cvPutText(img->cvArr(), s, p1, &font, col);
            // paint side view too
          p1.x = p0s.x + roundi(pNBR.x * ppm);
          p1.y = p0s.y - roundi(pNBR.z * ppm);
          p2.x = p1.x + vSize * (bb->maxx - bb->minx + 1);
          p2.y = p1.y - vSize * (bb->maxz - bb->minz + 1);
          cvRectangle(img->cvArr(), p1, p2, col, 1, 4);
          p1.y -= 2;
          cvPutText(img->cvArr(), s, p1, &font, col);
        }
        // draw the foodprint too
        m = bb->footprint->getPointsCnt();
        pos = bb->footprint->getPoints();
        for (i = 0; i < m; i++)
        {
          poly[i].x = p0.x + roundi(pos->x * ppm);
          poly[i].y = p0.y - roundi(pos->y * ppm);
          pos++;
        }
        pop = poly;
        cvPolyLine(img->cvArr(), &pop, &m, 1, true,
                   col, 1, 4, 0);
      }
    }
  }
}

//////////////////////////////////////////////////////////

UPosition UGrid3D::getCellCenter(int ix, int iy, int iz, double * size)
{
  UPosition result;
  result.x = (ix) * voxelSize;
  result.y = (ix - yWidth/2) * voxelSize;
  result.z = (iz) * voxelSize;
  return result;
}

///////////////////////////////////////////////////

UPosition UGrid3D::getCellNBR(int ix, int iy, int iz, double * size)
{ // near bottom right
  UPosition result;
  result.x = (ix  - 0.5) * voxelSize;
  result.y = (ix - yWidth/2  - 0.5) * voxelSize;
  result.z = (iz  - 0.5) * voxelSize;
  return result;
}

///////////////////////////////////////////////////

UPosition UGrid3D::getCellFTL(int ix, int iy, int iz, double * size)
{ // far top left
  UPosition result;
  result.x = (ix  + 0.5) * voxelSize;
  result.y = (ix - yWidth/2  + 0.5) * voxelSize;
  result.z = (iz  + 0.5) * voxelSize;
  return result;
}

///////////////////////////////////////////////////

void UGrid3D::logBB(ULogFile * lf, bool validOnly)
{
  int n;
  UGridBBox * bb;
  UPosition pnbr, pftl;
  double snbr, sftl;
  const int MSL = 200;
  char s[MSL];
  //
  if(boxes != NULL)
  {
    for (n = 0; n < boxes->bbCnt; n++)
    {
      bb = boxes->bb[n];
      if (bb->valid or not validOnly)
      {
        pnbr = getCellCenter(bb->minx, bb->miny, bb->minz, &snbr);
        pftl = getCellCenter(bb->maxx, bb->maxy, bb->maxz, &sftl);
        snprintf(s, MSL, "%d %s nbr(%.2f %.2f %.2f) ftr(%.2f %.2f %.2f)", n, bool2str(bb->valid),
                pnbr.x, pnbr.y, pnbr.z, pftl.x, pftl.y, pftl.z);
      }
    }
  }
}

//////////////////////////////////////////////////////////

int UGrid3D::mergeOverlappingBBs()
{
  int m = 0, m2 = 1;
  int maxdy;
  int maxzdist;
  //
  if (boxes != NULL)
  { //
    maxdy = roundi(2.2/voxelSize);
    maxzdist = roundi(0.7/voxelSize);
    // to reduce effect of non-convex objects boxes, reduce size
    while (m != m2)
    {
      m2 = m;
      m = boxes->mergeOverlappingBBs(maxdy, maxzdist);
    }
    // reduce bounding box footprint to convex polygons
    boxes->reduceToConvexFootprint();
  }
  return m;
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void UResObj3d::UResObj3dInit()
{
  int i;
  //
  addObstacleParameters();
  lastSvsSerial = 0;
  storage = NULL;
  gndsCnt = 0;
  for (i = 0; i < MAX_GND_POLYGONS; i++)
    gnds[i] = NULL;
  grid = NULL;
}

///////////////////////////////////////////

UResObj3d::~UResObj3d()
{
  int i;
  if (loghum.isOpen())
    loghum.closeLog();
  for (i = 0; i < MAX_GND_POLYGONS; i++)
    if (gnds[i] != NULL)
      delete gnds[i];
  if (grid != NULL)
    delete grid;
}

///////////////////////////////////////////

void UResObj3d::addObstacleParameters()
{
  varCombineDist = addVar("combineDist", 0.25, "d",
        "(rw) Combine obstacles if separation is less than this");  //
  varGrpMaxDist = addVar("obstGrpMaxDistance", 7.0, "d",
        "(rw) [m] max pose distance in one obstacle group");
  varGrpMaxTime = addVar("obstGrpMaxTime", 7.0, "d",
        "(rw) [sec] max pose time span in one group");
  varObstMerge = addVar("obstMerge", 1.0, "d",
        "(rw) Should obstacles be merged over scans");
  varUpdateTime = addVar("updTime", 0.0, "d",
        "(ro) Time (tod) of last obstacle update");
  varGrps =     addVar("obstGrps",  0.0, "d", "(ro) number of obstacle groups in pool");
  varGroup =    addVar("obstGroup", 0.0, "d", "(ro) newest obstacle group");
  varGndMinZ =  addVar("gndMinZ",  -0.4, "d", "(rw) limit for ground finding (Z axis (height))");
  varGndMaxZ =  addVar("gndMaxZ",   0.1, "d", "(rw) limit for ground finding (Z axis (height))");
  varGndMinQ =  addVar("gndMinQ",   0.6, "d", "(rw) Min Q for usage of ground plane estimate");
  varGndRedFac = addVar("gndRedFac", 3.0, "d", "(rw) Point reduction factor for ground plane estimate - to save time (def=3)");
  varDistMaxX = addVar("maxDist",  19.0, "d", "(rw) limit for ground finding (X axis (fromt))");
  varBBminX = addVar("minDist",  0.5, "d",
                     "(rw) Detections closer to robot origin are considered false");
  //
  // filter parameters for bounding boxes
  varGridSize = addVar("voxelCellSize",  0.25, "d", "(rw) cell size for voxel grid finder");
  varBBmerge = addVar("BBmerge",  1.0, "d", "(rw) should bounding box merge be allowed");
  varBBmaxDX = addVar("humMaxDX",  4.0, "d", "(rw) max x extend for human finder");
  varBBmaxDY = addVar("humMaxDY",  2.2, "d", "(rw) max y extend (width) for human finder");
  varBBmaxZ = addVar("humMaxZ",  2.3, "d", "(rw) max height for human finder");
  varBBminZ = addVar("humMinZ",  0.5, "d", "(rw) min height for human finder");
  varBBminVoxCnt = addVar("minVoxelCnt",  20.0, "d", "(rw) Minimum number of cells for a human");
  varMinDens = addVar("minDens", 30.0, "d", "(rw) Minimum density in voxels per "
      "footprint square meter for footprint with less than 'minDEnsCnt' cells");
  varMinDensCnt = addVar("minDensCnt", 30.0, "d", "(rw) Minimum density is "
      "tested for voxel blobs with up to this number of cells");
  varMinRoofHgt = addVar("minRoofHgt", 2.3, "d", "(rw) Obstacles with all parts "
      "higher than this is assubed to be harmless roof-parts or tree tops, that can be ignored.");
  varGndFilter = addVar("gndFilter", 1.0, "d", "(rw) Filter voxels above solid ground");
  varGndFilterCell = addVar("gndFilterCell", 0.33, "d",
        "(rw) gnd filter side size compared to voxel side size");
  varGndFilterErode = addVar("gndFilterErode", 1.0, "d",
        "(rw) Erode ground filter one more than dilate");
  varGndFilterDilate = addVar("gndFilterDilate", 0.0, "d",
        "(rw) Dilate (fatten) and erode ground filter");
  varGndPolyScale = addVar("gndPolyScale", 2.0, "d",
                           "(rw) Number of pixels (width) in source image for each pixel in ground polygon image");
}

////////////////////////////////////////////////////////

void UResObj3d::getObstacleGroupSettings(UObj3dGroup * og)
{
  double v;
  //
  // get newest obstacle group
  setNewGrpDist(varGrpMaxDist->getValued());
  setNewGrpTime(varGrpMaxTime->getValued());
  if (og != NULL)
  {
    v = varCombineDist->getValued();
    og->setMergeDistance(v, varObstMerge->getBool());
  }
}

///////////////////////////////////////////

const char * UResObj3d::snprint(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  int i, n, m;
  char * p1;
  UObj3dGroup * og;
  //
  snprintf(buff, buffCnt, "%s groupsCnt=%d\n", preString, groupsCnt);
  n = strlen(buff);
  p1 = &buff[n];
  m = mini(groupsCnt, 4);
  for (i = 0; i < m; i++)
  {
    og = getGroup(i);
    og->print("   ", p1, buffCnt - n, false);
    n += strlen(p1);
    p1 = &buff[n];
  }
  if (m < groupsCnt)
    snprintf(p1, buffCnt - n, "    ...\n");
  return buff;
}

///////////////////////////////////////////

void UResObj3d::obstDataUpdated(UTime poseTime)
{
  //printf("www\n");
  varUpdateTime->setTime(poseTime);
  varGrps->setInt(groupsCnt, 0);
  varGroup->setInt(newest, 0);
}

////////////////////////////////////////////

bool UResObj3d::do3dCloudFromSvs(UImg3Dpoints * cloud)
{
  bool result = true;
  double minZ, maxZ, maxX, maxSep;
  UResPoseHist * odoPose;
  UPoseTVQ robotPose;
  //
  result = cloud != NULL;
  if (result)
  { // convert to robot coordinates (if not already)
    // convert to local robot coordinates - so that
    // ground plane should be around z=0.0 and about horizontal
      cloud->toRobotCoordinates();
  }
  if (result)
  { // get robot pose
    odoPose = (UResPoseHist*)getStaticResource("odoPose", false);
    result = (odoPose != NULL);
    robotPose = odoPose->getPoseAtTime(cloud->time);
    // get limit parameters
    minZ = varGndMinZ->getValued();
    maxZ = varGndMaxZ->getValued();
    maxX = varDistMaxX->getValued();
    maxSep = varCombineDist->getValued();
    addGroundObjects(cloud, minZ, maxZ, maxX, maxSep);
  }
  //
  return result;
}

////////////////////////////////////////////

bool UResObj3d::get3dCloudFromSvs(bool anAny, UImg3Dpoints ** cloud)
{
  bool result = true;
  UDataBase * data = NULL;
  double val;
  double ser = roundi(lastSvsSerial);
  int n;
  //
  if (anAny)
    ser = -1.0;
  /*  */
  result = callGlobal("svs.get3d", "dc", NULL, &ser, &val, &data, &n);
  if (result)
  {
    result = (data != NULL);
    if (result)
    { // get full class pointer
      if (data->isA("svs3d") or isA("img3d"))
      {
        *cloud = (UImg3Dpoints *)data;
        lastSvsSerial = (*cloud)->serial;
      }
      else
        result = false;
    }
  }
  //
  return result;
}

////////////////////////////////////////////

bool UResObj3d::get3dCloud(const char * source, UImg3Dpoints ** cloud)
{
  bool result = true;
  UDataBase * data = NULL;
  UVariable * pars = {NULL};
  int n = 1;
  const int MSL = 100;
  char s[MSL];
  //
  snprintf(s,MSL, "%s.get3d", source);
  result = callGlobalV(s, "", &pars, &data, &n);
  if (result and n == 1 and data != NULL)
  {
    if (data->isA("img3d"))
      *cloud = (UImg3Dpoints *)data;
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UResObj3d::addGroundObjects(UImg3Dpoints* cloud,
                                 double minZ, double maxZ, double maxX,
                                double maxSep)
{
  UImg3Dpoint **pts, *sp, **pd;
  int ptsCnt = 0;
  UImage *img  = NULL, *img1 = NULL, *img2 = NULL;
  UImage *img3 = NULL, *img4 = NULL, *img5 = NULL;
  UImagePool * imgPool;
  //CvScalar blue = CV_RGB(255, 0 , 0);
  CvScalar red = CV_RGB(255, 0 , 0);
  //CvScalar mag = CV_RGB(175, 0 , 175);
  CvScalar yellow = CV_RGB(175, 175 , 0);
  CvFont font;
  const int MSL = 80;
  char s[MSL];
  int r, c, pCnt;
  const int w = 320;
  const int h = 240;
  unsigned char *imgdata;
  UPixel * ipx;
  double mY, mZ;
  CvPoint p1, p2, p0;
  int g;
  UPolygon40 * gnd;
  UPosition pos;
  // debug file save
  FILE * pFile = NULL;
  //pFile = fopen("groundPoints.log", "w");
  // debug end
  //
  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
               0.5, 0.5, 0.0, 1, 8);
  //
  pts = (UImg3Dpoint**) malloc(sizeof(UImg3Dpoint*) * cloud->p3dCnt);
  sp = cloud->p3d; // source point
  pd = pts; // destination point
  for (int i = 0; i < cloud->p3dCnt; i++)
  { // find points in usable range
    if (sp->pos.z > minZ and sp->pos.z < maxZ and sp->pos.x < maxX)
    {
      *pd = sp;
      pd++;
      ptsCnt++;
    }
    sp->q = 0;
    sp++;
  }
  // now divide into gouups unitil no more points
  if (true)
  {
    imgPool = (UImagePool *)getStaticResource("imgPool", false);
    if (imgPool != NULL)
    { // BW image for dilate/erode
      img  = imgPool->getImage(46, true);
      img->setSize(240, 320, 1, 8, "gray");
      img->clear(0);
      img->imgTime = cloud->time;
      img->imageNumber = cloud->serial;
      strncpy(img->name, "ground-final", MAX_IMG_NAME_SIZE);
      img->camDevice = 11; // 11 is left
      // BW image for filtering
      img1  = imgPool->getImage(41, true);
      img1->setSize(240, 320, 1, 8, "gray");
      img1->clear(0);
      img1->imgTime = cloud->time;
      img1->imageNumber = cloud->serial;
      strncpy(img1->name, "ground-dilated", MAX_IMG_NAME_SIZE);
      img1->camDevice = 11; // 11 is left
      // BW image for filtering
      img5  = imgPool->getImage(40, true);
      img5->setSize(240, 320, 1, 8, "gray");
      img5->clear(0);
      img5->imgTime = cloud->time;
      img5->imageNumber = cloud->serial;
      strncpy(img5->name, "dilated-eroded", MAX_IMG_NAME_SIZE);
      img5->camDevice = 11; // 11 is left
/*      snprintf(s, MSL, "#%lu", img->imageNumber);
      cvPutText(img->cvArr(), s, cvPoint(240, 20), &font, red);*/
      //img->updated();
      img2  = imgPool->getImage(37, true);
      img2->setSize(240, 320, 3, 8, "RGB");
      img2->clear(255);
      img2->imgTime = cloud->time;
      img2->imageNumber = cloud->serial;
      img2->camDevice = 11; // 11 is left
      strncpy(img2->name, "left-ground-2", MAX_IMG_NAME_SIZE);
      snprintf(s, MSL, "slize-%lu", img->imageNumber);
      cvPutText(img2->cvArr(), s, cvPoint(10, 20), &font, yellow);
      //
      img3  = imgPool->getImage(38, true);
      img3->setSize(240, 320, 3, 8, "RGB");
      img3->clear(255);
      img3->imgTime = cloud->time;
      img3->imageNumber = cloud->serial;
      img3->camDevice = 11; // 11 is left
      strncpy(img3->name, "left-ground-xz", MAX_IMG_NAME_SIZE);
      snprintf(s, MSL, "xz-%lu", img->imageNumber);
      cvPutText(img3->cvArr(), s, cvPoint(10, 20), &font, yellow);
      //
      img4  = imgPool->getImage(39, true);
      img4->setSize(240, 320, 3, 8, "RGB");
      img4->clear(255);
      img4->imgTime = cloud->time;
      img4->imageNumber = cloud->serial;
      img4->camDevice = 11; // 11 is left
      strncpy(img4->name, "left-ground-xy", MAX_IMG_NAME_SIZE);
      snprintf(s, MSL, "xy-%lu", img->imageNumber);
      cvPutText(img4->cvArr(), s, cvPoint(10, 20), &font, yellow);
    }
  }
  if (img != NULL)
  {
    pd = pts;
    imgdata = (unsigned char *)img->getData();
    for (int i = 0; i < ptsCnt; i++)
    { // get next point
      c = (*pd)->column;
      r = (*pd)->row;
      imgdata[r*w+c] = 255;
      pd++;
    }
    cvDilate(img->cvArr(), img1->cvArr(), NULL, 1);
    img1->updated();
    cvErode(img1->cvArr(), img5->cvArr(), NULL, 3);
    img5->updated();
    cvDilate(img5->cvArr(), img->cvArr(), NULL, 2);
    img->updated();
    // annotate the approved points
    pd = pts;
    imgdata = (unsigned char *)img->getData();
    for (int i = 0; i < ptsCnt; i++)
    { // get next point
      c = (*pd)->column;
      r = (*pd)->row;
      if (imgdata[r*w+c] != 0)
        (*pd)->q += 1;
      // debug
      if (pFile != NULL)
        fprintf(pFile, "%3d %3d %d %g     %g      %g\n", r, c, (*pd)->q, (*pd)->pos.x, (*pd)->pos.y, (*pd)->pos.z);
      // debug end
      pd++;
    }
    // debug
    if (pFile != NULL)
      fclose(pFile);
    // debug end
  }
  //
  if (img2 != NULL)
  { // 3D Z-slize in image coordinates
    pd = pts;
    for (int i = 0; i < ptsCnt; i++)
    { // get next point
      c = (*pd)->column;
      r = (*pd)->row;
      ipx = img2->getPixRef(r, c);
      ipx->set((*pd)->blue(), (*pd)->green(), (*pd)->red());
      pd++;
    }
  }
  if (img != NULL)
  { // enumerate contures and paint largest in img2
    pCnt = findPolygons(img, img2, pts, ptsCnt, 1);
    //
    printf("UResObj3d::addGroundObjects found a ground contour of %d points\n", pCnt);
  }
  if (img3 != NULL)
  { // x-z projection
    img3->paintGridAligned(h-70, 0, w / maxX, 5);
    pd = pts;
    mZ = maxX *3.0 / 4.0;
    for (int i = 0; i < ptsCnt; i++)
    { // get next point
      c = roundi((*pd)->pos.x / maxX * w);
      r = roundi((*pd)->pos.z / mZ * h) + 70;
      r = h - r;
      if (r >= 0 and r < h and c >= 0 and c < w and (*pd)->q > 0)
      {
        ipx = img3->getPixRef(r, c);
        ipx->set((*pd)->blue(), (*pd)->green(), (*pd)->red());
        ipx[1] = ipx[0];
        ipx[w] = ipx[0];
        ipx[w+1] = ipx[0];
      }
      pd++;
    }
    img3->updated();
  }
  if (img4 != NULL)
  { // x-z projection
    img4->paintGridAligned(h/2, 0, w / maxX, 5);
    pd = pts;
    mY = maxX *3.0 / 4.0;
    for (int i = 0; i < ptsCnt; i++)
    { // get next point
      c = roundi((*pd)->pos.x / maxX * w);
      r = roundi((*pd)->pos.y / mY * h) + h/2;
      r = h - r;
      if (r >= 0 and r < h-1 and c >= 0 and c < w-1 and (*pd)->q > 0)
      { // paint block of 4 pixels
        ipx = img4->getPixRef(r, c);
        ipx->set((*pd)->blue(), (*pd)->green(), (*pd)->red());
        ipx[1] = ipx[0];
        ipx[w] = ipx[0];
        ipx[w+1] = ipx[0];
      }
      pd++;
    }
    p1.x = 0;
    p1.y = 0;
    for (g = 0; g < gndsCnt; g++)
    {
      gnd = gnds[g];
      for (int i = 0; i < gnd->getPointsCnt(); i++)
      { // get next point
        p2 = p1;
        pos = gnd->getPoint(i);
        c = roundi(pos.x / maxX * w);
        r = roundi(pos.y / mY * h) + h/2;
        r = h - r;
        //if (r >= 0 and r < h-1 and c >= 0 and c < w-1)
        {
          p1.x = c;
          p1.y = r;
          if (i > 0)
            cvLine(img4->cvArr(), p1, p2, red);
          else
            p0 = p1;
        }
      }
      // last line
      cvLine(img4->cvArr(), p1, p0, red);
    }
    img4->updated();
  }
  //
  return false;
}

///////////////////////////////////////////////////////

// void UResObj3d::paintGridAligned(UImage * img,
//                           const int r0, const int c0, const double pixPerM,
//                const int strongTicEvery)
// {
//   CvPoint p1, p2;
//   const CvScalar blue = CV_RGB(100, 100 , 255);
//   const CvScalar gray1 = CV_RGB(175, 175 , 175);
//   const CvScalar gray2 = CV_RGB(100, 100 , 100);
//   CvScalar col;
//   int w, h, m, x, y;
//   //
//   w = img->getWidth();
//   h = img->getHeight();
//   if (pixPerM > 1.5)
//   { // find leftmost meter line
//     m = 0;
//     x = c0;
//     while (x >= 0)
//     {
//       m--;
//       x = c0 + roundi(m * pixPerM);
//     }
//     m++;
//     p1.y = 0;
//     p2.y = h - 1;
//     p1.x = c0 + roundi(m * pixPerM);
//     while (p1.x < w)
//     { // paint all vertical lines
//       p1.x = c0 + roundi(m * pixPerM);
//       p2.x = p1.x;
//       if (p1.x == c0)
//         col = blue;
//       else if (m % strongTicEvery == 0)
//         col = gray2;
//       else
//         col = gray1;
//       cvLine(img->cvArr(), p1, p2, col, 1);
//       m++;
//       p1.x = c0 + roundi(m * pixPerM);
//     }
//     // find leftmost meter line
//     m = 0;
//     y = r0;
//     while (y >= 0)
//     {
//       m--;
//       y = r0 + roundi(m * pixPerM);
//     }
//     m++;
//     // do the lines
//     p1.x = 0;
//     p2.x = w - 1;
//     p1.y = r0 + roundi(m * pixPerM);
//     while (p1.y < h)
//     { // paint all horizontal lines
//       if (p1.y == r0)
//         col = blue;
//       else if (m % strongTicEvery == 0)
//         col = gray2;
//       else
//         col = gray1;
//       p2.y = p1.y;
//       cvLine(img->cvArr(), p1, p2, col, 1);
//       m++;
//       p1.y = r0 + roundi(m * pixPerM);
//     }
//   }
// }

//////////////////////////////////////////////

int UResObj3d::findContour(UImage * img, UImage * dest,
                           CvPoint * flpNotUsed, int flpMax,
                           bool * tooHigh, bool * tooSmall, bool lookingLeft)
{
  bool isOK;
//  int levels = 3;
  CvSeq * contours = NULL;
  //CvMemStorage * storage = cvCreateMemStorage(0);
  CvPoint * point1 = NULL;
  CvPoint * point2;
  CvSeqBlock * block;
  int i, cnt = 0, n;
//  bool * ob; // border point
  CvSeq * seq;
//  const int TOO_SMALL_LIMIT = 70; // edge count of area
//  const int TOO_HIGH_LIMIT = 3; // pixels from top of image
  int high;
  UTime t1, t2;
  //
  //clearStorage();
  if (storage == NULL)
    storage = cvCreateMemStorage(0);
  //
  isOK = ((img != NULL) and (dest != NULL));
  if (isOK)
  {
    // debug timing
    t1.Now();
    // debug end timing
    cvFindContours( img->cvArr(), storage, &contours, sizeof(CvContour),
                    CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
    // debug timing
    t2.Now();
    printf("cvFindContours took %f sec\n", t2 - t1);
    // debug end timing
    //
    //if (tooSmall != NULL)
    {
      i = countContourEdges(contours, &n);
      //*tooSmall = (i < TOO_SMALL_LIMIT);
      printf ("Found contours with total %d points, largest has %d points\n", i, n);
    }
    //
    // debug timing
    t1.Now();
    // debug end timing
    // comment this out if you do not want approximation
    contours = cvApproxPoly( contours, sizeof(CvContour), storage,
                             CV_POLY_APPROX_DP, // only method supported
                             2.8,        // accuracy
                             1 );        // 0 = not closed, 1 = closed curve
    // debug timing
    t2.Now();
    {
      i = countContourEdges(contours, &n);
      //*tooSmall = (i < TOO_SMALL_LIMIT);
      printf ("Fount contours with total %d points, largest has %d points\n", i, n);
    }
    // debug end timing
    //
    //if (tooHigh != NULL)
    {
      i = countContourLimits(contours, NULL, NULL, &high, NULL, lookingLeft);
      //*tooHigh = (high < TOO_HIGH_LIMIT);
      printf("Path top pixel is %d\n", high);
    }
    //printf("Reduced contours to %d points\n", contours->first->count);
    //
    // find largest contour
    cnt = 0;
    i = 0;
    seq = contours;
    while (true)
    { // try all sequences in seqence list
      block = seq->first;
      while (true)
      { // find the contour witth the largest point-set
        if (block->count > cnt)
        { // save best
          point1 = (CvPoint *) block->data;
          cnt = block->count;
        }
        i++;
        // advance to next contour
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        seq = seq->h_next;
    }
    // copy the largest contour
    //cnt = mini(cnt, flpMax);
    //memcpy(flp, point1, cnt * sizeof(CvPoint));
    //printf("Saved %d CvPoints (best of %d contours)\n", cnt, i);
    // mark if border-point

    //
    if (false)
    { // draw largest contour
      cvPolyLine(dest->cvArr(), &point1, &cnt, 1, true,
                 CV_RGB(255,0,0), 1, 4, 0);
    }
    else
    {
      seq = contours;
      while (true)
      { // try all sequences in seqence list
        block = seq->first;
        while (true)
        {
          point2 = (CvPoint *) block->data;
          for (i = 1; i < block->count; i++)
          {
            point1 = point2++;
            //void cvLine( CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color,
            //       int thickness=1, int line_type=8, int shift=0 )
            cvLine(dest->cvArr(), *point1, *point2, CV_RGB(255,0,0), 1, 4, 0);
          }
          // close contour
          point1 = (CvPoint *) block->data;
          cvLine(dest->cvArr(), *point1, *point2, CV_RGB(255,0,0), 1, 4, 0);
          //
          if (block->next == seq->first)
            break;
          else
            block = block->next;
        }
        if (seq->h_next == NULL)
          break;
        else
          seq = seq->h_next;
      }
    }
    // point[4]
    //cvDrawContours( imgMask->cvArr(), contours, CV_RGB(255,0,0),
    //                CV_RGB(0,255,0), levels-3, 3, CV_AA );
    // remove data
    cvClearSeq(contours);
    cvClearMemStorage( storage ); // sets the storage as empty (but allocated)
  }
  //
  return cnt;
}

////////////////////////////////////////////////////////

int UResObj3d::countContourEdges(CvSeq * contours, int * largest)
{
  int res = 0;
  CvSeqBlock * block;
  CvSeq * seq;
  //
  *largest = 0;
  if (contours != NULL)
  {
    seq = contours;
    while (true)
    { // try all blocks in seqence list
      block = seq->first;
      while (true)
      { // get count in this contour
        res += block->count;
        if (block->count > *largest)
            *largest = block->count;
        // advance to next contour
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        seq = seq->h_next;
    }
  }
  //
  return res;
}

//////////////////////////////////////////////

int UResObj3d::countContourLimits(CvSeq * contours, int * left,
                                  int * right, int * top, int * bottom, bool lookingLeft)
{
  int res = 0;
  CvSeqBlock * block;
  CvSeq * seq;
  CvPoint * point;
  int l = 1000, r = 0, t = 1000, b = 0;
  int i;

  //
  if (contours != NULL)
  {
    seq = contours;
    while (true)
    { // try all blocks in seqence list
      block = seq->first;
      while (true)
      { // get count in this contour
        res += block->count;
        //
        point = (CvPoint *)block->data;
        for (i = 0; i < block->count; i++)
        {
          if (point->x < l)
            l = point->x;
          if (point->x > r)
            r = point->x;
          if (point->y > b)
            b = point->y;
          if (lookingLeft)
          { // top is only valid to the left
            // of the image
            if (point->x < 50)
              if (point->y < t)
                t = point->y;
          }
          else
          {
            if (point->x > 270)
              if (point->y < t)
                t = point->y;
          }
          point++;
        }
        // advance to next contour
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        seq = seq->h_next;
    }
  }
  if (left != NULL)
    *left = l;
  if (right != NULL)
    * right = r;
  if (bottom != NULL)
    *bottom = b;
  if (top != NULL)
    * top = t;
  //
  return res;
}

///////////////////////////////////////////////////

CvSeqBlock * UResObj3d::findPolygonLargest(UImage * img, int * leftEdge, int * rightEdge)
{
  CvSeq * contours = NULL;
  CvSeqBlock * block, *blockMax = NULL;
  CvSeq * seq;
//  UPolygon40 * p40;
//  UImg3Dpoint ** pd;
  int a, aMax, le, re, leMax = 0, reMax = 0;
  //
  //clearStorage();
  if (storage == NULL)
    storage = cvCreateMemStorage(0);
//  pd = (UImg3Dpoint**) malloc(sizeof(UImg3Dpoint*) * p40->MAX_POINTS);
  //
//  cvFindContours( img->cvArr(), storage, &contours, sizeof(CvContour),
//                  CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
  cvFindContours( img->cvArr(), storage, &contours, sizeof(CvContour),
                  CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0,0) );
/*  contours = cvApproxPoly( contours, sizeof(CvContour), storage,
                           CV_POLY_APPROX_DP, // only method supported
                             2.0,        // accuracy
                             1 );        // 0 = not closed, 1 = closed curve*/
  //
  //go through polygons
//  *largest = 0;
  if (contours != NULL)
  { 
    gndsCnt = 0;
    seq = contours;
    // find the largest block of the conturs
    // this is assumed to be the best ground polygon
    a = 0;
    aMax = 0;
    blockMax = NULL;
    while (true)
    { // try all blocks in seqence list
      block = seq->first;
      while (true)
      { // get count in this contour
        a = getBlockArea(block, &le, &re);
        if (a > aMax)
        {
          aMax = a;
          blockMax = block;
          leMax = le;
          reMax = re;
        }
        // advance to next contour in this block
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        // advance to next block of contours
        seq = seq->h_next;
    }
  }
  // return results
  if (leftEdge != NULL)
    *leftEdge = leMax;
  if (rightEdge != NULL)
    *rightEdge = reMax;
  return blockMax;
}

//////////////////////////////////////////////

int UResObj3d::getBlockArea(CvSeqBlock * block, int * leftEdge, int * rightEdge)
{
  int i, aSum, le = 10000, re = 0;
  CvPoint *p1, *p2;
  CvSeqBlock * blk;
  //
  blk = block;
  p1 = (CvPoint *)blk->data;
  i = 1;
  aSum = 0;
  while (true)
  {
    p2 = p1;
    // find left-right edge
    if (p2->x > re)
      re = p2->x;
    if (p2->x < le)
      le = p2->x;
    // find ńext point
    if (i < blk->count)
    { // in same sequence, so just advance
      p1++;
      i++;
    }
    else
    { // is in next sequence (or loop back to start)
      blk = blk->next;
      p1 = (CvPoint *)blk->data;
      i = 1;
      if (blk == block)
        break;
    }
    aSum += p1->x * p2->y - p2->x * p1->y;
  }
  // and the last p1 too
  aSum += p1->x * p2->y - p2->x * p1->y;
  //
  if (leftEdge != NULL)
    *leftEdge = le;
  if (rightEdge != NULL)
    *rightEdge = re;
  //
  // return result area too.
  return aSum/2;
}
///////////////////////////////////////////////////

int UResObj3d::findPolygons(UImage * img, UImage * imgD,
                            UImg3Dpoint ** ps, const int psCnt,
                            int scale)
{
  CvSeq * contours = NULL;
  CvSeqBlock * block;
  CvSeq * seq;
  int n=0, i;
  UPolygon40 * p40;
  UImg3Dpoint ** pd;
  int p40Max = p40->MAX_POINTS;
  //
  //clearStorage();
  if (storage == NULL)
    storage = cvCreateMemStorage(0);
  pd = (UImg3Dpoint**) malloc(sizeof(UImg3Dpoint*) * p40->MAX_POINTS);
  //
//  cvFindContours( img->cvArr(), storage, &contours, sizeof(CvContour),
//                  CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
  cvFindContours( img->cvArr(), storage, &contours, sizeof(CvContour),
                  CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0,0) );
/*  contours = cvApproxPoly( contours, sizeof(CvContour), storage,
  CV_POLY_APPROX_DP, // only method supported
  2.0,        // accuracy
  1 );        // 0 = not closed, 1 = closed curve*/
  //
  //go through polygons
//  *largest = 0;
  if (contours != NULL)
  { // iterate found conturs
    gndsCnt = 0;
    seq = contours;
    while (true)
    { // try all blocks in seqence list
      block = seq->first;
//      poly4.clear();
      while (true)
      { // get count in this contour
        n = findPolygonsOne(block, ps, psCnt, pd, p40Max, p40Max, scale);
        // paint found points in image
        if (n > 0 and imgD != NULL)
          paintPolygonInImage(imgD, pd, n);
        // save as polygon
        if (n > 0 and gndsCnt < MAX_GND_POLYGONS)
        {
          if (gnds[gndsCnt] == NULL)
            gnds[gndsCnt] = new UPolygon40;
          p40 = gnds[gndsCnt];
          p40->clear();
          for (i = 0; i < n; i++)
            p40->add(pd[i]->pos);
          gndsCnt++;
        }
        // advance to next contour in this block
        if (block->next == seq->first)
          break;
        else
          block = block->next;
      }
      if (seq->h_next == NULL)
        break;
      else
        // advance to next block of contours
        seq = seq->h_next;
    }
  }
  if (pd != NULL)
    free(pd);
  return n;
}

///////////////////////////////////////////////////

int UResObj3d::findPolygonsOne(CvSeqBlock * block,
                                UImg3Dpoint ** ps, const int psCnt,
                                UImg3Dpoint ** pd, const int pdCnt,
                                    const int maxPoints,
                                    int scale
                              )
{
//  UPolygon400 * poly4;
  CvPoint * point;
  CvPoint * pointLeft;
  CvPoint * pointRight;
  CvSeqBlock * blk;
  int i, j, dCnt = 0, c, r, nt, nb;
  const int MIW = 800; // max image width
  CvPoint top[MIW];
  CvPoint * topp, *pt;
  CvPoint bot[MIW];
  CvPoint * botp, *pb;
  bool topOK[MIW];
  bool botOK[MIW];
  UImg3Dpoint * a3t[MIW];
  UImg3Dpoint * a3b[MIW];
  UImg3Dpoint * p3s, **p3t, **p3b;
  const float approxToll = 2.0;
  float toll;
// int n;
//  CvMat mT, mB;
//  CvSeq *seqT, *seqB;
  //
  for (i = 0; i < MIW; i++)
  {
    top[i].x = -1;
    bot[i].x = -1;
    a3t[i] = NULL;
    a3b[i] = NULL;
    topOK[i] = false;
    botOK[i] = false;
  }
  // test for impossible concavities, i.e.
  // points need to be continous for increasing x values from
  // the left-most to the right most (and back)
  // So take all points in contour and note the min and max y value for each x
  blk = block;
  pointRight = (CvPoint *)block->data;
  pointLeft = pointRight;
  j = 0;
  while (true)
  {
    point = (CvPoint *)blk->data;
    for (i = 0; i < blk->count; i++)
    { // save uppermost point in each column
      topp = &top[point->x];
      botp = &bot[point->x];
      if (topp->x == -1)
      { // no points in this column yet, so set this as both top and bottom
        *topp = *point;
        *botp = *point;
      }
      else
      { // there is more than one, so expand as needed
        if (topp->y > point->y)
          *topp = *point;
        if (botp->y < point->y)
          *botp = *point;
      }
      // save also the left and rightmost point
      if (point->x < pointLeft->x)
        pointLeft = point;
      if (point->x > pointRight->x)
        pointRight = point;
      point++;
    }
    blk = blk->next;
    if (blk == block)
      break;
    j++;
  }
  // do not use too narrow objects - at least 10 pixels widt
  dCnt = 0;
  if (pointRight->x - pointLeft->x > 10)
  { // now correlate with original point cloud, and find
    // the best fitted cloud point for each of the new polygon
    // contour points
    for (j = 0; j < psCnt; j++)
    {
      p3s = ps[j];
      // is this point part valid part of point slize
      if (p3s->q > 0)
      { // get a shorthand for column and row number
        c = p3s->column/scale;
        r = p3s->row/scale;
        // is it inside polygon limits - column wise
        if (c >= pointLeft->x and c <= pointRight->x)
        { // get the two y-limits for this column
          pt = &top[c];
          pb = &bot[c];
          if (pt != NULL)
          { // is it inside polygon - row wise
            if (r >= pt->y and r <= pb->y)
            { // get the resulting 2 cloud points for this column
              p3t = &a3t[c];
              p3b = &a3b[c];
              if (*p3t == NULL)
              { // none so far, so use this cloud point
                *p3t = p3s;
                *p3b = p3s;
              }
              else if ((r*scale) < (*p3t)->row)
                // this is better as top point
                *p3t = p3s;
              else if ((r*scale) > (*p3b)->row)
                // this is better as bottom point
                *p3b = p3s;
            }
          }
        }
      }
    }
    // reduce the polygon outline
    // now approximate by taking only a subset of the points in both top and bottom lines
    i = pointLeft->x;
    j = pointRight->x;
    while (i < j and a3t[i] == NULL)
      i++;
    while (j > i and a3b[j] == NULL)
      j--;
    pointLeft->x = i;
    pointRight->x = j;
    // always use endpoints
    topOK[i] = true;
    botOK[i] = true;
    topOK[j] = true;
    botOK[j] = true;
    // reduce the rest of the points
    nt = maxPoints + 1;
    toll = approxToll;
    // allow 2/3 for to line
    while (nt > (maxPoints * 2) / 3)
    { // ensure result fits into polygon (40 points)
      approximatePolyLine(&top[i], &topOK[i], j - i + 1, toll);
      nt = 0;
      for (c = i; c < j+1; c++)
      { // count number of vertices
        if (topOK[c])
          nt++;
      }
      // increase tollerance after each iteration
      toll += 1.0;
    }
    nb = maxPoints + 1;
    toll = approxToll;
    // allow the rest for the bottom line
    while ((nt + nb) > maxPoints - 2)
    {
      approximatePolyLine(&bot[i], &botOK[i], j - i + 1, toll);
      nb = 0;
      for (c = i; c < j+1; c++)
      { // count number of vertices
        if (botOK[c])
          nb++;
      }
      // increase tollerance after each iteration
      toll += 1.0;
    }
    //
    // get points
    // forward on top of polygon
    for (i = pointLeft->x; i <= pointRight->x; i++)
    {
      p3t = &a3t[i];
      if (*p3t != NULL and topOK[i])
      {
        pd[dCnt] = *p3t;
        if (dCnt < pdCnt)
          dCnt++;
      }
    }
    // debug
    printf("reduced top line from %d to %d\n", pointRight->x - pointLeft->x + 1, dCnt);
    // debug end
    // and back on bottom
    for (i = pointRight->x; i >= pointLeft->x; i--)
    {
      p3b = &a3b[i];
      if (*p3b != NULL and botOK[i])
      {
        pd[dCnt] = *p3b;
        if (dCnt < pdCnt)
          dCnt++;
      }
    }
    // debug
    printf("reduced polygon line from %d to %d\n", 2*(pointRight->x - pointLeft->x + 1), dCnt);
    // debug end
  }
  return dCnt;
}

////////////////////////////////////////////////////////////////

int UResObj3d::findPolygonsOnEdge(CvSeqBlock * block,
                               UImg3Dpoint ** ps, const int psCnt,
                               int scale, int leftEdge, int rightEdge, int distLimit
                              )
{
//  UPolygon400 * poly4;
  CvPoint * point;
  CvPoint * pointLeft;
  CvPoint * pointRight;
  CvSeqBlock * blk;
  int i, j, c, r;
  const int MIW = 800; // max image width
  CvPoint top[MIW];
  CvPoint * topp, *pt;
  CvPoint bot[MIW];
  CvPoint * botp, *pb;
  UImg3Dpoint * a3t[MIW];
  UImg3Dpoint * a3b[MIW];
  UImg3Dpoint * p3s, **p3t, **p3b;
  UPolygon400 * pog = NULL;
  bool inPoly, endPoly = false;
  UPosition extra, p1;
  double maxX, avgy1 = 0.0, avgy2 = 0.0;
  //
  for (i = 0; i < MIW; i++)
  {
    top[i].x = -1;
    bot[i].x = -1;
    a3t[i] = NULL;
    a3b[i] = NULL;
  }
  // test for "unlikely" concavities, i.e.
  // points need to be continous for increasing x values from
  // the left-most to the right most (and back)
  // So take all points in contour and note the min and max y value for each x
  blk = block;
  pointRight = (CvPoint *)block->data;
  pointLeft = pointRight;
  j = 0;
  while (true)
  {
    point = (CvPoint *)blk->data;
    for (i = 0; i < blk->count; i++)
    { // save uppermost point in each column
      topp = &top[point->x];
      botp = &bot[point->x];
      if (topp->x == -1)
      { // no points in this column yet, so set this as both top and bottom
        *topp = *point;
        *botp = *point;
      }
      else
      { // there is more than one, so expand as needed
        if (topp->y > point->y)
          *topp = *point;
        if (botp->y < point->y)
          *botp = *point;
      }
      // save also the left and rightmost point
      if (point->x < pointLeft->x)
        pointLeft = point;
      if (point->x > pointRight->x)
        pointRight = point;
      point++;
    }
    blk = blk->next;
    if (blk == block)
      break;
    j++; 
  }
  // do not use too narrow objects - at least 10 pixels widt
  // now correlate with original point cloud, and find
  // the best fitted cloud point for each of the new polygon
  // contour points
  for (j = 0; j < psCnt; j++)
  {
    p3s = ps[j];
    // is this point part valid part of point slize
    if (p3s->q > 0)
    { // get a shorthand for column and row number
      c = p3s->column/scale;
      r = p3s->row/scale;
      // is it inside polygon limits - column wise
      if (c >= pointLeft->x and c <= pointRight->x)
      { // get the two y-limits for this column
        pt = &top[c];
        pb = &bot[c];
        if (pt != NULL)
        { // is it inside polygon - row wise
          if (r >= pt->y and r <= pb->y)
          { // get the resulting 2 cloud points for this column
            p3t = &a3t[c];
            p3b = &a3b[c];
            if (*p3t == NULL)
            { // none so far, so use this cloud point
              *p3t = p3s;
              *p3b = p3s;
            }
            else if ((r*scale) < (*p3t)->row)
              // this is better as top point
              *p3t = p3s;
            else if ((r*scale) > (*p3b)->row)
              // this is better as bottom point
              *p3b = p3s;
          }
        }
      }
    }
  }
  // reduce the polygon outline
  // now approximate by taking only a subset of the points in both top and bottom lines
  i = pointLeft->x;
  j = pointRight->x;
  while (i < j and a3t[i] == NULL)
    i++;
  while (j > i and a3b[j] == NULL)
    j--;
  pointLeft->x = i;
  pointRight->x = j;
  //
  inPoly = false;
  maxX = 0.0;
  gndsCnt = 0;
  for (i = pointLeft->x; i <= pointRight->x; i++)
  {
    p1 = a3t[i]->pos;
    if (p1.x < distLimit)
    {
      if (not inPoly)
      { // start a new polygon
        if (pog == NULL)
          pog = new UPolygon400();
        pog->clear();
        avgy1 = p1.y;
        inPoly = true;
        endPoly = false;
      }
      // add points until over max distance or at end
      pog->add(p1);
      if (p1.x > maxX)
        maxX = p1.x;
      avgy2 = p1.y;
    }
    else if (inPoly)
      endPoly = true;
    if (inPoly and (endPoly or i == pointRight->x))
    { // terminate this polygon by a more distant point
      if (pog->getPointsCnt() > 1)
      { // save as edge based obstacle
        extra.x = maxX + 0.7;
        extra.y = (avgy1 + avgy2)/2.0;
        extra.z = 0.0;
        pog->add(extra);
        // move to gnds array of polygons
        if (gnds[gndsCnt] == NULL)
          gnds[gndsCnt] = new UPolygon40();
        pog->extractConvexTo(gnds[gndsCnt]);
        gndsCnt++;
        if (gndsCnt >= MAX_GND_POLYGONS)
        {
          printf("No space for more than %d edge obstacles! - ignores the rest\n", gndsCnt);
          break;
        }
      }
      //
      inPoly = false;
      maxX = 0.0;
    }
  }
  //
  if (pog != NULL)
    delete pog;
  //
  return gndsCnt;
}

////////////////////////////////////////////////////

float UResObj3d::findMostDistantVertex(const CvPoint pnts[], // point array to test - both ends are included
                           const int pntsCnt, // number of points to test
                           int * idx)  // index of most distant vertex
{
  U2Dline lin; // from-to line
  int im = -1, i;
  float d, imd = 0.0;
  const CvPoint * pnt = pnts;
  //
  if (pntsCnt > 2)
  {
    lin.set2P(pnts->x, pnts->y, pnts[pntsCnt-1].x, pnts[pntsCnt-1].y);
    for (i = 1; i < pntsCnt - 1; i++)
    {
      d = lin.distanceSigned(pnt->x, pnt->y);
      if (fabs(d) > fabs(imd))
      {
        imd = d;
        im = i;
      }
      pnt++;
    }
  }
  if (idx != NULL)
    *idx = im;
  return imd;
}

///////////////////////////////////////////////////////////

void UResObj3d::approximatePolyLine(const CvPoint pnts[], // point array to test - both ends are included
                           bool pntOK[],         // array of boolean to mark used points
                           const int pntsCnt,    // number of points to test
                           const float toll)     // acceptable distance variation
{
  int i, j;
  double d;
  //
  d = findMostDistantVertex(pnts, pntsCnt, &i);
  if (fabs(d) > toll)
  {
    pntOK[i] = true;
    if (i > 1)
      // interval from 0 to i must be tested too
      approximatePolyLine(pnts, pntOK, i+1, toll);
    if ((pntsCnt - i) > 2)
      // interval after index i must be tested too
      approximatePolyLine(&pnts[i], &pntOK[i], pntsCnt - i, toll);
  }
  else
  { // invalidate all other points
    for (j = 1; j < pntsCnt - 1; j++)
      pntOK[j] = false;
  }
}

///////////////////////////////////////////////////

void UResObj3d::paintPolygonInImage(UImage * img, UImg3Dpoint ** pd, const int pdCnt)
{
  UImg3Dpoint * p3;
  int i;
  CvPoint p1, p2;
  CvScalar red = CV_RGB(255, 0 , 0);
//  CvScalar blue = CV_RGB(0, 0 , 255);
  //
  p3 = pd[0];
  p1.x = p3->column;
  p1.y = p3->row;
  for (i = 1; i < pdCnt; i++)
  {
    p2 = p1;
    p3 = pd[i];
    p1.x = p3->column;
    p1.y = p3->row;
    cvLine(img->cvArr(), p1, p2, red);
  }
  p2 = p1;
  p3 = pd[0];
  p1.x = p3->column;
  p1.y = p3->row;
  cvLine(img->cvArr(), p1, p2, red);
  img->updated();
}

////////////////////////////////////////////////////

bool UResObj3d::do3dVoxels(UImg3Dpoints * cloud, bool justHuman, bool doImg)
{
  bool isOK;
  int i, n, m, ipg;
  UImage *img2, *img3, *imgMask = NULL;
  UImagePool * imgPool;
  CvFont font;
  CvScalar yellow = CV_RGB(125, 125 , 0);
  const int MSL = 100;
  char s[MSL];
  UTime t;
  double maxDX, maxDY, minH, maxH, minX, cellSize;
  double minDens, minRoofHgt;
  int minDensCnt;
  int minVoxelCnt;
  const double gridDepth = 15.0;
  const double gridWidth = 9.0;
  const double gridHeight = 4.0;
  UImg3Dpoint * ip;
  UGridBBox * bb;
  UGridBBoxes * bbs;
  UResPoseHist * resOdo;
  UPoseTime robotPose;
  //
  if (not loghum.isOpen())
    loghum.openLog("human");
  // first do the ground plane
  isOK = (cloud->p3dCnt > 20000 and gndPlaneQ > 0.2);
  if (isOK)
  { // now do some voxel segmentation
    cellSize = varGridSize->getValued();
    minX = varBBminX->getValued();
    if (grid == NULL)
    {
      grid = new UGrid3D();
    }
    grid->setVolumen(cellSize, gridDepth, gridWidth, gridHeight);
    // add points to voxel grid
    t.now();
    ip = cloud->p3d;
    n = 0;
    m = 0;
    // ensure points are in robot coordinates
    cloud->toRobotCoordinates();
    for (i = 0; i < cloud->p3dCnt; i++)
    { // do not use too bright pixels (either bright road or sky)
      // as these bright pixels usually has bad correlation
      ipg = ip->gray();
      if (ipg > 220 or (ipg > 190 and ip->blue() > ip->red()))
      {
        if (ipg > 220)
          n++;
        else
          m++;
      }
      else if (cloud->p3d[i].pos.x > minX)
        grid->setVoxel(&gndPlaneR, &cloud->p3d[i]);
      ip++;
    }
    printf("UResObj3d::do3dVoxels removed %d too bright and %d too bright blue points\n", n, m);
    printf("UResObj3d::do3dVoxels fill voxel grid: %.1fms\n", t.getTimePassed()*1000.0);
    // paint voxel cell image
    imgPool = (UImagePool *)getStaticResource("imgPool", false);
    if (doImg and false)
    { // all voxels, including ground - not so usefull
      cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
                   0.5, 0.5, 0.0, 1, 8);
      img2  = imgPool->getImage(43, true);
      img2->setSize(480, 640, 3, 8, "RGB");
      img2->clear(180);
      img2->imgTime = cloud->time;
      img2->imageNumber = cloud->serial;
      img2->camDevice = 11; // 11 is left
      strncpy(img2->name, "raw-voxels", MAX_IMG_NAME_SIZE);
      snprintf(s, MSL, "slize-%u", cloud->serial);
      cvPutText(img2->cvArr(), s, cvPoint(10, 20), &font, yellow);
      grid->paintVoxels(img2, 0, false, 0);
      img2->updated();
    }
    // now do some segmentation down to this height limit
    t.now();
    if (varGndFilter->getValueBool())
      imgMask = imgPool->getImage(41, false);
    i = grid->voxelSegmentation(0.5, imgMask, 1.0/varGndFilterCell->getValued());
    printf("UResObj3d::do3dVoxels voxel segmentation: %.1fms\n", t.getTimePassed() * 1000.0);
    t.now();
    if (varBBmerge->getValueBool())
      i = grid->mergeOverlappingBBs();
    isOK = (i > 0);
    if (isOK)
    {
//      printf("Voxel grid with %d cells found %d boxes\n", grid->countNonZeroVoxels(), i);
      if (doImg)
      {
        cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
                     0.5, 0.5, 0.0, 1, 8);
        img3  = imgPool->getImage(44, true);
        img3->setSize(480, 640, 3, 8, "RGB");
        img3->clear(255);
        img3->imgTime = cloud->time;
        img3->imageNumber = cloud->serial;
        img3->camDevice = 11; // 11 is left
        strncpy(img3->name, "filtered", MAX_IMG_NAME_SIZE);
        snprintf(s, MSL, "slize-%u", cloud->serial);
        cvPutText(img3->cvArr(), s, cvPoint(10, 20), &font, yellow);
        grid->paintVoxels(img3, 1, true, 5);
        img3->updated();
      }
    }
    // now filter the found bounding boxes.
    t.now();
    maxDX = varBBmaxDX->getValued();
    maxDY = varBBmaxDY->getValued();
    maxH = varBBmaxZ->getValued();
    minH = varBBminZ->getValued();
    minX = varBBminX->getValued();
    minDens = varMinDens->getValued();
    minDensCnt = varMinDensCnt->getInt();
    minVoxelCnt = varBBminVoxCnt->getInt();
    minRoofHgt = varMinRoofHgt->getValued();
    i = grid->filter(minVoxelCnt, maxDX, maxDY, maxH, minH, minX,
                     minRoofHgt, minDens, minDensCnt);
    // log findings
    if (loghum.isOpen())
    {
      loghum.setLogTime(cloud->time);
      grid->logBB(&loghum, false);
    }
    // add found obstacles to obstacle pool
    resOdo = (UResPoseHist*)getStaticResource("odoPose", false);
    isOK = (resOdo != NULL);
    if (isOK)
    { // odo pose is available, add
      bbs = grid->getBoxes();
      robotPose = resOdo->getPoseAtTime(cloud->time);
      robotPose.t = cloud->time;
      for (n = 0; n < bbs->bbCnt; n++)
      { // get next box
        bb = bbs->bb[n];
        if (bb->valid)
        {
          if (bb->human or not justHuman)
            addObstacle(bb->footprint, robotPose, bb->human, false);
        }
      }
    }
    //
    printf("UResObj3d::do3dVoxels fill filtering: %gms\n", t.getTimePassed() * 1000.0);
    if (isOK)
    {
//      printf("Voxel grid with %d cells found %d boxes\n", grid->countNonZeroVoxels(), i);
      if (doImg)
      {
        cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
                     0.5, 0.5, 0.0, 1, 8);
        img2  = imgPool->getImage(45, true);
        img2->setSize(480, 640, 3, 8, "RGB");
        img2->clear(255);
        img2->imgTime = cloud->time;
        img2->imageNumber = cloud->serial;
        img2->camDevice = 11; // 11 is left
        strncpy(img2->name, "filtered", MAX_IMG_NAME_SIZE);
        snprintf(s, MSL, "slize-%u", cloud->serial);
        cvPutText(img2->cvArr(), s, cvPoint(10, 20), &font, yellow);
        grid->paintVoxels(img2, 1, true, 5);
        img2->updated();
      }
    }
    //
    // paint a new ground plane view
    paintGndPlane(47, cloud, varDistMaxX->getDouble(), varGndMaxZ->getDouble(), true, true);
  }
  return isOK;
}

///////////////////////////////////////////////

bool UResObj3d::do3dGroundPlane(UImg3Dpoints * cloud, bool doGndEdgeObst, bool doImg)
{
  bool result = true;
  int cnt, i;
  const int MSL = 1000;
  char s[MSL];
  UMatrix4 mMC, mCM;
  UPosition pro, prp, prn;
  double maxZ, maxX, cellSize;
  UPlane refPlane;
  UImg3Dpoint *pc, *pcr;
  UPosition mxY, mmY, mmX, mnl, mnr, mfl, mfr, mp;
  UTime t;
  UImg3Dpoint **cloudPts = NULL, **cloudPt, *cloudp3d;
  int cloudPtsCnt;
  UImg3Dpoints *cloudRed = new UImg3Dpoints();
  double d, maxZgp, maxXgp, minXgp, minZ;
  UImage * img;
  CvSeqBlock * block;
  int leftEdge, rightEdge;
  UResPoseHist * resOdo;
  UPoseTime robotPose;
  int redFac;
  //
  // get parameters
  maxZ = varGndMaxZ->getDouble();
  maxZgp = maxZ * 2.0;
  maxX = varDistMaxX->getDouble();
  maxXgp = maxX + 5.0;
  minXgp = varBBminX->getDouble();
  minZ = varGndMinZ->getDouble();
  redFac = varGndRedFac->getInt();
  if (redFac < 1)
    redFac = 1;
  //
  result = (cloud != NULL);
  if (result and doImg)
  {
    // convert to robot coordinates (if not already)
    t.now();
    cloud->toRobotCoordinates();
    // debug timing
    printf("[OK] convert to robot coordinates took %.1fms\n", t.getTimePassed() * 1000.0);
    // debug end
  }
  if (result)
  { // get limit parameters
    refPlane.clear();
    if (cloud->inPoseCoordinates)
      refPlane = cloud->pose.getRobToCamPlane(refPlane);
//    printf("3D cloud reduced from %d points\n", cloud->p3dCnt);
    t.now();
    //
    // limit the number of used points in range and height
    pc = cloud->p3d;
    pcr = cloudRed->p3d;
    for (i = 0; i < cloud->p3dCnt; i += redFac)
    { // use only every (third) position with a height less than 1.4m
      if (pc->pos.z < 1.4 and
          pc->pos.x < maxX and
          pc->pos.x > minXgp and
          pc->pos.z > minZ)
      {
        pcr = cloudRed->add(pc);
        pcr->q = 0;
      }
      pc += redFac; // advance pointer to next sample
    }
    result = cloudRed->p3dCnt > 20;
  }
  if (result)
  {  // do the plane extraction
    gndPlaneC = cloudRed->getRansacPlane(100, maxZ, refPlane, 0.95, maxX, &cnt);
    gndPlaneQ = cnt/double(cloudRed->p3dCnt);
    gndPlaneTime = cloud->time;
    // debug
    gndPlaneC.print("ground plane (cam):", s, MSL);
    printf("%s - count=%d (q=%.2f%%) - took %.1fms\n", s, cnt, gndPlaneQ * 100.0, t.getTimePassed() * 1000.0);
    // debug end
    if (gndPlaneQ > varGndMinQ->getValued())
    {
      if (cloud->inPoseCoordinates)
        // convert plane to robot coordinates
        gndPlaneR = cloud->pose.getCamToRobPlane(gndPlaneC);
      else
        gndPlaneR = gndPlaneC;
      //
      cellSize = varGridSize->getValued() * varGndFilterCell->getValued();
      doGndPlaneMask(41, gndPlaneR, cloud, cellSize, 0.3,
                     varGndFilterErode->getValueBool(),
                     varGndFilterDilate->getValueBool());
      // result mask image is image 41
      if (doGndEdgeObst)
      { // find ground polygon
        cloudPts = (UImg3Dpoint**) malloc(cloud->p3dCnt * sizeof(UImg3Dpoint*));
        cloudPt = cloudPts;
        cloudp3d = cloud->p3d;
        cloudPtsCnt = 0;
        for (i = 0; i < cloud->p3dCnt; i++)
        { // get list of in-liers
          d = gndPlaneR.dist(cloudp3d->pos);
          if (d < maxZgp and cloudp3d->pos.x < maxXgp and cloudp3d->pos.x > minXgp)
          { // save inliers
            cloudPtsCnt++;
            cloudp3d->q = 1;
            *cloudPt++ = cloudp3d;
          }
          cloudp3d++;
        }
        img = makeGroundPlaneProfileImage(38, 39,
                        cloudPts, cloudPtsCnt, varGndPolyScale->getInt());
        if (false)
        { // find all ground plane ponygons, and put result into gnds[]
          if (img != NULL)
          {
            gndsCnt = 0;
            findPolygons(img, NULL, cloudPts, cloudPtsCnt, varGndPolyScale->getInt());
          }
        }
        else
        { // find rather polygons at edge of largest ground plane polygon
          if (img != NULL)
          {
            block = findPolygonLargest(img, &leftEdge, &rightEdge);
            if (block != NULL)
            {
              findPolygonsOnEdge(block,cloudPts, cloudPtsCnt,
                               varGndPolyScale->getInt(),
                               leftEdge, rightEdge, roundi(maxX));
              // add edge polygons to obstacle pool
              resOdo = (UResPoseHist*)getStaticResource("odoPose", false);
              result = (resOdo != NULL);
              if (result)
              { // odo pose is available, add
                robotPose = resOdo->getPoseAtTime(cloud->time);
                robotPose.t = cloud->time;
                for (i = 0; i < gndsCnt; i++)
                  addObstacle(gnds[i], robotPose, false, true);
              }
            }
          }
        }
      }
    }
    else
    {
      printf("ground detect failed, as Q=%.2f%% is less than the limit of %.2f%%\n",
             gndPlaneQ * 100.0, varGndMinQ->getValued()*100.0);
    }
    // paint result
    paintGndPlane(42, cloud, maxX, maxZ, false, true);
  }
  if (cloudRed != NULL)
    delete cloudRed;
  if (cloudPts != NULL)
    free(cloudPts);
  //
  return result;
}

////////////////////////////////////////////////////

UImage * UResObj3d::makeGroundPlaneProfileImage(int inum1, int inum2,
                                UImg3Dpoint** cloudPts, int cloudPtsCnt,
                               int scale)
{
  UImagePool * imgPool;
  UImage *img = NULL, *img2;
  const int w = 320 / scale;
  const int h = 240 / scale;
  int i, r, c;
  UImg3Dpoint ** pd;
  
  //
  imgPool = (UImagePool *)getStaticResource("imgPool", false);
  if (imgPool != NULL)
  { // get images
    img  = imgPool->getImage(inum1, true);
    img2  = imgPool->getImage(inum2, true);
  }
  if (img != NULL)
  { //
    img->setSize(h, w, 1, 8, "gray");
    img->clear(0);
    strncpy(img->name, "ground-profile-1", MAX_IMG_NAME_SIZE);
    img2->setSize(h, w, 1, 8, "gray");
    strncpy(img->name, "ground-profile-2", MAX_IMG_NAME_SIZE);

    pd = cloudPts;
    for (i = 0; i < cloudPtsCnt; i++)
    { // get next point
      c = (*pd)->column / scale;
      r = (*pd)->row / scale;
      img->setPixUChar(r, c, 255);
      pd++;
    }
    // filtyer the profile
    cvDilate(img->cvArr(), img2->cvArr(), NULL, 1);
    cvErode(img2->cvArr(), img->cvArr(), NULL, 1);
    img->updated();
    img2->updated();
  }
  return img;
}

////////////////////////////////////////////////////

void UResObj3d::doGndPlaneMask(int maskImgNum, UPlane plane, UImg3Dpoints * cloud,
                       double cellSize,
                       double planeDist, bool erode, bool dilate)
{
  UImage * img1;
  UImage * img2 = NULL;
  int i, r, c;
  UImagePool * imgPool;
  const double ppm = 1.0/cellSize;
  const double gridDepth = 25.0;
  const double gridWidth = 16.0;
  const int maxY = roundi(gridWidth * ppm);
  const int maxX = roundi(gridDepth * ppm);
  UImg3Dpoint * pc;
  double d;
  int sourceImg = 40;
  int filtImg = maskImgNum;
  //
  imgPool = (UImagePool *)getStaticResource("imgPool", false);
  if (imgPool != NULL)
  { // get images
    if (not erode)
    { // reverse images. so that result is in image 41
      sourceImg = maskImgNum;
      filtImg = 40;
    }
    img1  = imgPool->getImage(sourceImg, true);
    img1->setSize(maxY , maxX, 1, 8, "gray");
    img1->clear(0);
    img1->imgTime = cloud->time;
    img1->imageNumber = cloud->serial;
    strncpy(img1->name, "ground-plane", MAX_IMG_NAME_SIZE);
    img1->camDevice = 11; // 11 is left
    if (erode or dilate)
    { // BW image for filtering
      img2  = imgPool->getImage(filtImg, true);
      img2->setSize(maxY, maxX, 1, 8, "gray");
      img2->clear(0);
      img2->imgTime = cloud->time;
      img2->imageNumber = cloud->serial;
      strncpy(img2->name, "filtered", MAX_IMG_NAME_SIZE);
      img2->camDevice = 11; // 11 is left
    }
    //
    // fill pixels into image
    pc = cloud->p3d;
    for (i = 0; i < cloud->p3dCnt; i++)
    {
      d = plane.dist(pc->pos);
      if (d < planeDist)
      {
        r = maxY/2 - roundi(pc->pos.y * ppm);
        c = roundi(pc->pos.x * ppm);
        if (r >= 0 and r < maxY and c >= 0 and c < maxX)
        {
          img1->setPixUChar(r, c, 255);
        }
      }
      pc++;
    }
    img1->updated();
    if (dilate)
    { // dilate and erode, to mage ground more solid
      cvDilate(img1->cvArr(), img2->cvArr(), NULL, 1);
      cvErode(img2->cvArr(), img1->cvArr(), NULL, 1);
    }
    if (erode)
    {
      cvErode(img1->cvArr(), img2->cvArr(), NULL, 1);
      img2->updated();
    }
  }
}

//////////////////////////////////////////////

void UResObj3d::paintGndPlane(int imgNum, UImg3Dpoints * cloud, double maxX, double maxZ,
                              bool andBoxes, bool andGndPolygons)
{
  UImg3Dpoint *pc;
  UPosition mxY, mmY, mmX, mnl, mnr, mfl, mfr, mp;
  CvPoint p1, p2, p0={0,0}, p0s;
  UImagePool * imgPool;
  UImage * img2 = NULL;
  CvScalar mag = CV_RGB(175, 0 , 175);
  CvScalar blue = CV_RGB(90, 90 , 255);
  CvScalar yellow = CV_RGB(125, 125 , 0);
  CvScalar red = CV_RGB(255, 0 , 0);
  CvScalar col = yellow;
  CvFont font;
  double d, ppm = 10.0;
  const int MSL = 1000;
  char s[MSL];
  int i, n, m;
  UGridBBoxes * boxes;
  UGridBBox * bb;
  UPosition * pos, posR;
  const int MPL = 40; // max polygon edge count
  CvPoint poly[MPL];
  CvPoint *pop;
  UPolygon40 * p40;
  UResPoseHist * odoPose;
  UPoseTime robotPose;
  //
  imgPool = (UImagePool *)getStaticResource("imgPool", false);
  if (imgPool != NULL)
  {
    cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,
                 0.5, 0.5, 0.0, 1, 8);
    img2  = imgPool->getImage(imgNum, true);
  }
  if (img2 != NULL)
  {
    img2->setSize(480, 640, 3, 8, "RGB");
    p0.x = 5;
    p0.y = (img2->height() * 2) / 3;
    ppm = img2->width()/maxX;
    img2->clear(255);
    img2->imgTime = cloud->time;
    img2->imageNumber = cloud->serial;
    img2->camDevice = 11; // 11 is left
    strncpy(img2->name, "ground-plane", MAX_IMG_NAME_SIZE);
    snprintf(s, MSL, "slize-%u", cloud->serial);
    cvPutText(img2->cvArr(), s, cvPoint(10, 18), &font, yellow);
    img2->paintGridAligned(p0.y, p0.x, ppm, 10);
    // get limit rectangle on plane
    // find minimum x and leftmost and rightmost positions for envolope display
    pc = cloud->p3d;
    mxY = pc->pos;
    mmY = mxY;
    mmX = mxY;
    pc++;
    for (i = 1; i < cloud->p3dCnt; i++)
    {
      if (pc->pos.y/pc->pos.x < mmY.y/mmY.x)
        mmY = pc->pos;
      if (pc->pos.y/pc->pos.x > mxY.y/mxY.x)
        mxY = pc->pos;
      if (pc->pos.x < mmX.x)
        mmX = pc->pos;
      pc++;
    }
    mp.x = mmX.x;
    mp.y = mmY.y / mmY.x * mp.x;
    mp.z = 0.0;
    mnr = gndPlaneR.getOnPlane(mp);
    mp.y = mxY.y / mxY.x * mp.x;
    mnl = gndPlaneR.getOnPlane(mp);
    mp.x = maxX + 8.0;
    mp.y = mxY.y / mxY.x * mp.x;
    mfl = gndPlaneR.getOnPlane(mp);
    mp.y = mmY.y / mmY.x * mp.x;
    mfr = gndPlaneR.getOnPlane(mp);
      // paint far part on side view
    p0s.x = p0.x; // side view origin
    p0s.y = p0.y/3; // - roundi(10.0 * ppm);
      // left edge
    p1.x = p0s.x + roundi(mnl.x * ppm);
    p1.y = p0s.y - roundi(mnl.z * ppm);
    p2.x = p0s.x + roundi(mfl.x * ppm);
    p2.y = p0s.y - roundi(mfl.z * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);
    p2.x = p0s.x + roundi(mnr.x * ppm);
    p2.y = p0s.y - roundi(mnr.z * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);
    p1.x = p0.x + roundi(mnl.x * ppm);
    p1.y = p0.y - roundi(mnl.y * ppm);
    p2.x = p0.x + roundi(mfl.x * ppm);
    p2.y = p0.y - roundi(mfl.y * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);
    p2.x = p0.x + roundi(mnr.x * ppm);
    p2.y = p0.y - roundi(mnr.y * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);
    //
    pc = cloud->p3d;
      // convert to robot coordinates (if not already)
    cloud->toRobotCoordinates();
    for (i = 0; i < cloud->p3dCnt; i++)
    {
      if (pc->pos.x < (maxX + 10.0))
        d = gndPlaneR.dist(pc->pos);
      else
        d = 100.0;
      if (d < maxZ)
      {
        p1.x = p0.x + roundi(pc->pos.x * ppm);
        p1.y = p0.y - roundi(pc->pos.y * ppm);
        p2 = p1;
        p2.x += 4;
        if (p1.x < 640)
        { // paint top view - if within image
          col = CV_RGB(pc->red(), pc->green(), pc->blue());
          cvLine(img2->cvArr(), p1, p2, col, 4);
            // paint side view too
          p1.x = p0s.x + roundi(pc->pos.x * ppm);
          p1.y = p0s.y - roundi(pc->pos.z * ppm);
          p2 = p1;
          p2.x += 4;
          cvLine(img2->cvArr(), p1, p2, col, 4);
        }
      }
      pc++;
    }
    p1.x = p0s.x + roundi(mfr.x * ppm);
    p1.y = p0s.y - roundi(mfr.z * ppm);
    p2.x = p0s.x + roundi(mfl.x * ppm);
    p2.y = p0s.y - roundi(mfl.z * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);
    p2.x = p0s.x + roundi(mnr.x * ppm);
    p2.y = p0s.y - roundi(mnr.z * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);
    p1.x = p0.x + roundi(mfr.x * ppm);
    p1.y = p0.y - roundi(mfr.y * ppm);
    p2.x = p0.x + roundi(mfl.x * ppm);
    p2.y = p0.y - roundi(mfl.y * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);
    p2.x = p0.x + roundi(mnr.x * ppm);
    p2.y = p0.y - roundi(mnr.y * ppm);
    cvLine(img2->cvArr(), p1, p2, mag, 1);

    snprintf(s, MSL, "planeR (%.3f, %.3f, %.3f, %.2f) q=%.2f",
            gndPlaneC.a, gndPlaneC.b, gndPlaneC.c, gndPlaneC.d, gndPlaneQ);
    cvPutText(img2->cvArr(), s, cvPoint(200, 18), &font, yellow);
    ////
    if (grid != NULL and andBoxes)
    {
      boxes = grid->getBoxes();
      if (boxes != NULL)
      { // test all boxes
        for (n = 0; n < boxes->bbCnt; n++)
        { // get next box
          bb = boxes->bb[n];
          if (bb->valid)
          { // draw the box foodprint
            m = bb->footprint->getPointsCnt();
            pos = bb->footprint->getPoints();
            for (i = 0; i < m; i++)
            { // convert odometry pose to local pose
              posR = robotPose.getMapToPose(*pos);
              poly[i].x = p0.x + roundi(posR.x * ppm);
              poly[i].y = p0.y - roundi(posR.y * ppm);
              pos++;
            }
            pop = poly;
            col = UPixel::toCvRGB(bb->getCol());
            cvPolyLine(img2->cvArr(), &pop, &m, 1, true,
                       col, 3, 8, 0);
            if (bb->human)
              col = red;
            else if (bb->roof)
              col = blue;
            else
              col = mag;
            cvPolyLine(img2->cvArr(), &pop, &m, 1, true,
                       col, 1, 4, 0);
            if (bb->sideView != NULL)
            { // draw the box side view
              m = bb->sideView->getPointsCnt();
              pos = bb->sideView->getPoints();
              for (i = 0; i < m; i++)
              { // NB! in side view polygon Y is actually Z
                poly[i].x = p0s.x + roundi(pos->x * ppm);
                poly[i].y = p0s.y - roundi(pos->y * ppm);
                pos++;
              }
              col = UPixel::toCvRGB(bb->getCol());
              cvPolyLine(img2->cvArr(), &pop, &m, 1, true,
                         col, 3, 8, 0);
              if (bb->human)
                col = red;
              else if (bb->roof)
                col = blue;
              else
                col = mag;
              cvPolyLine(img2->cvArr(), &pop, &m, 1, true,
                        col, 1, 4, 0);
            }
          }
        }
      }
    }
    if (grid != NULL and andGndPolygons)
    {
      UPoseTime robotPose;
      //
      odoPose = (UResPoseHist *)getStaticResource("odoPose", false);
      if (odoPose != NULL)
        robotPose = odoPose->getNewest();
      for (n = 0; n < gndsCnt; n++)
      {
        p40 = gnds[n];
        m = p40->getPointsCnt();
        pos = p40->getPoints();
        for (i = 0; i < m; i++)
        {
          poly[i].x = p0.x + roundi(pos->x * ppm);
          poly[i].y = p0.y - roundi(pos->y * ppm);
          pos++;
        }
        pop = poly;
        cvPolyLine(img2->cvArr(), &pop, &m, 1, true,
                   red, 1, 8, 0);
      }
    }
    img2->updated();
  }
}



