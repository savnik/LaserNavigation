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

#include <ugen4/u2dline.h>

#include "uprobgrid.h"

////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////


UProbGrid::UProbGrid()
{
  polysCnt = 0;
  gridCellSize = 0.05; // meter per pixel
  newestPoly = -1;
  gcX = 0;
  gcY = 0;
}

////////////////////////////////////////


UProbGrid::~UProbGrid()
{
}

////////////////////////////////////////

bool UProbGrid::setNewPolygon(UPose robPose, UTime poseTime)
{
  bool res;
  UProbPoly * pp;
  //
  newestPoly = (newestPoly + 1) % MAX_POLYGON_COUNT;
  if (newestPoly >= polysCnt)
    polysCnt++;
  pp = &polys[newestPoly];
  res = pp->setPoly(robPose, poseTime);
  //
  return res;
}

///////////////////////////////////////////////

UPosition * UProbGrid::getNewPoly()
{
  UPosition * res = NULL;
  //
  if (newestPoly < polysCnt)
    res = polys[newestPoly].getPoints();
  //
  return res;
}

///////////////////////////////////////////////

bool * UProbGrid::getNewIsObst()
{
  bool * res = NULL;
  //
  if (newestPoly < polysCnt)
    res = polys[newestPoly].getIsObst();
  //
  return res;
}

///////////////////////////////////////////////

int UProbGrid::getNewPolyCnt()
{
  int res = 0;
  //
  if (newestPoly < polysCnt)
    res = polys[newestPoly].getPointsCnt();
  //
  return res;
}


////////////////////////////////////////
bool UProbGrid::paintGrid(UImage * img,
               int zeroX, int zeroY, // pixel position of zero
               double pixelSize,     // meters per pixel
               UPixel gridRGB,       // colour for 1 m markings
               UPixel zeroRGB)       // colour for zero crossing
{
  bool res = img != NULL;
  int i, w, h, r, c;

  //
  if (res)
  { // paint also a zero cross
    h = img->height();
    w = img->width();
    for (i = roundi(-zeroX * pixelSize);
         i < roundi((w - zeroX) * pixelSize);
         i++)
    { // vertical 1m lines
      c = roundi(i / pixelSize) + zeroX;
      cvLine(img->cvArr(), cvPoint(c, 0), cvPoint(c, h), gridRGB.cvRGB(), 1);
    }
    for (i = roundi(-zeroY * pixelSize);
         i < roundi((h - zeroY) * pixelSize);
         i++)
    { // horizontal 1m lines
      r = roundi(i / pixelSize) + zeroY;
      cvLine(img->cvArr(), cvPoint(0, r), cvPoint(w, r), gridRGB.cvRGB(), 1);
    }
    cvLine(img->cvArr(), cvPoint(zeroX, 0), cvPoint(zeroX, h), zeroRGB.cvRGB(), 1);
    cvLine(img->cvArr(), cvPoint(0, zeroY), cvPoint(w, zeroY), zeroRGB.cvRGB(), 1);
  }
  return res;
}

/////////////////////////////////////////////////////////////////////////////

bool UProbGrid::makeProbGrid(const char * sourceName)
{
  bool res = (grid != NULL) and (tempGrid != NULL);
  int i, n, v;
  UProbPoly * pp;
  UPose robPose;
  UTime poseTime;
  CvPoint poly[UProbPoly::MAX_POINTS];
  CvPoint * pd;
  UPosition * ps;
  CvPoint * ppoly = poly;
  int polyCnt;
  int gcX, gcY; // grid center
  CvPoint * point1;
  CvPoint * point2;
  bool * isObst1;
  bool * isObst2;
  int h, w, r, c;
  UPixel * pixs;
  UPixel * pixd;
  int probFree = 50;
  int probObst = 50;
  // move other poses
  if (res)
  {
    pp = &polys[newestPoly];
    robPose = pp->getPoseNow();
    poseTime = pp->getPoseTime();
    for (i = 0; i <  polysCnt; i++)
    {
      if (i != newestPoly)
      { // move old polygons to be relative to this new pose
        pp = &polys[i];
        pp->moveToPose(robPose, poseTime);
      }
    }
  }
  // initialize gridmap
  if (res)
  {
    grid->setSize(240, 320, 3, 8, "BGR");
    tempGrid->setSize(240, 320, 3, 8, "BGR");
    grid->clear(128); // all gray (0.5 probability)
  }
  // position of robot in map now
  h = grid->height();
  w = grid->width();
  gcX = w / 5;
  gcY = h / 2;
  // paint all robot positions first
  if (res)
  { // paint also a zero cross
    paintGrid(grid, gcX, gcY, gridCellSize, UPixel::pixRGB(0, 128, 200), UPixel::pixRGB(200, 128, 0));
    // paint pobot positions used in this prob-grid
    for (i = 0; i < polysCnt; i++)
    {
      pp = &polys[i];
      pp->paintRobot(grid, gcX, gcY, gridCellSize);
    }
  }
  // now all polygons are in robot perspective, but
  // they can not be painted like that, they need to be moved
  // to about grid center to be usefull,
  if (res)
  { // convert to grid-center coordinates
    // start with the oldest
    if (newestPoly == (polysCnt - 1))
      n = 0; // not full - or newest is also last
    else
      n = (newestPoly + 1) % MAX_POLYGON_COUNT;
    // loop over all saved polygons from 'n'
    for (i = 0; i < polysCnt; i++)
    {
      pp = &polys[n];
      // convert to center
      ps = pp->getPoints();
      pd = poly;
      polyCnt = pp->getPointsCnt();
      for (v = 0; v < polyCnt; v++)
      {
         pd->x = gcX + roundi(ps->x/gridCellSize);
         pd->y = gcY - roundi(ps->y/gridCellSize);
         pd++;
         ps++;
      }
      // paint polygon in temp grid
      tempGrid->clear(0);
      // paint passable area in red
      cvFillPoly( tempGrid->cvArr(), &ppoly, &polyCnt, 1,
                 CV_RGB(0,255,0), 0, 0 );
      // paint obstacle border-lines in blue
      if (res)
      { // now paint the edges that are obstacles
        point1 = poly;
        isObst1 = pp->getIsObst();
        for (v = 0; v < polyCnt - 1; v++)
        {
          point2 = point1++;
          isObst2 = isObst1++;
          if (*isObst1 and *isObst2)
            cvLine(tempGrid->cvArr(), *point1, *point2, CV_RGB(255,0,0), 4, 8, 0);
        }
      }
      // now the temp image is ready
      // search this and tone the probability grid as appropriate
      for (r = 0; r < h; r++)
      {
        pixs = tempGrid->getLine(r);
        pixd = grid->getLine(r);
        for (c = 0; c < w; c++)
        {
          if (pixs->p2 > 0)
            pixd->p2 = mini(255, pixd->p2 + probFree);
          if (pixs->p1 > 0)
            pixd->p2 = maxi(0, pixd->p2 - probObst);
          pixs++;
          pixd++;
        }
      }
      //
      // debug
      //tempGrid->saveBMP(imagePath, sourceName, i, "tempGrid");
      //grid->saveBMP(imagePath, sourceName, i, "probGrid");
      // debug end
      //
      // advance to one step older
      n = (n + 1) % MAX_POLYGON_COUNT;
      // reduce influence of older polygons
      //probFree -= 4;
      //probObst -= 4;
    }
    // debug
    //tempGrid->saveBMP(imagePath, sourceName, i, "tempGrid");
    grid->saveBMP(imagePath, sourceName, polysCnt, "probGrid");
    // debug end
  }
  //
  return res;
}


/////////////////////////////////////////////////////////////////////////////

bool UProbGrid::makeProbGrid2(const char * sourceName)
{ // produce probability grid using line crossings
  bool res = (grid != NULL) and (tempGrid != NULL);
  int i, j, n, r, c;
  UProbPoly * pp;
  UPose robPose;
  UTime poseTime;
  int h, w;
  double minX, maxX, minY, maxY;
  int rowMin, rowMax;
  bool isOK;
  const int CROSSX_CNT = 30;
  double crossX[CROSSX_CNT];
  bool isObst[CROSSX_CNT];
  double * x;
  bool * obst, obs1, obs2;
  int x1;
  int x2;
  UPixel * pix;
  int crossCnt;
  int probFree = 50;
  int probObst = 50;
  UTime t1, t2;
  //const int SL = 30;
  //char s[SL];
  const double obstWidthInMeter = 0.4;
  int obstWidth = roundi(obstWidthInMeter / gridCellSize);
  //
  // debug timing
  t1.Now();
  // debug end timing
  //
  // move other poses
  if (res)
  {
    pp = &polys[newestPoly];
    robPose = pp->getPoseNow();
    poseTime = pp->getPoseTime();
    for (i = 0; i <  polysCnt; i++)
    {
      if (i != newestPoly)
      { // move old polygons to be relative to this new pose
        pp = &polys[i];
        pp->moveToPose(robPose, poseTime);
      }
    }
  }
  // initialize gridmap
  if (res)
  {
    grid->setSize(240, 320, 3, 8, "BGR");
    grid->clear(128); // all gray (0.5 probability)
  }
  // position of robot in map now
  h = grid->height();
  w = grid->width();
  gcX = w / 5;
  gcY = h / 2;
  // paint all robot positions first
  if (res)
  { // paint also a zero cross
    paintGrid(grid, gcX, gcY, gridCellSize, UPixel::pixRGB(0, 128, 200), UPixel::pixRGB(200, 128, 0));
    // paint pobot positions used in this prob-grid
    for (i = 0; i < polysCnt; i++)
    {
      pp = &polys[i];
      pp->paintRobot(grid, gcX, gcY, gridCellSize);
    }
  }
  // now all polygons are in perspective of current robot pose, but
  // they can not be painted like that, they need to be moved
  // to about grid center to be usefull,
  if (res)
  { // convert to grid-center coordinates
    // start with the oldest
    if (newestPoly == (polysCnt - 1))
      n = 0; // not full - or newest is also last
    else
      n = (newestPoly + 1) % MAX_POLYGON_COUNT;
    // loop over all saved polygons from 'n'
    for (i = 0; i < polysCnt; i++)
    {
      pp = &polys[n];
      // get limits (in robot coordinates)
      isOK = pp->getLimits(&minX, &maxX, &minY, &maxY);
      //
      // debug
      /*
      snprintf(s, SL, "poly #%d", i);
      pp->show(s);
      printf("limits x %f to %f, y %f to %f\n", minX, maxX, minY, maxY);
      */
      // debug end
      //
      if (isOK)
      { // test for all gridlines (rows) within these limist
        rowMin = roundi(gcY - maxY / gridCellSize);
        rowMin = mini(h-1, maxi( 0, rowMin));
        rowMax = roundi(gcY - minY / gridCellSize);
        rowMax = mini(h-1, maxi( 0, rowMax));
        for (r = rowMin; r < rowMax; r++)
        { // for all rows with polygon parts find crossings with grid row line
          crossCnt = pp->getCrossingsAtY(
                       (gcY - double(r)) * gridCellSize,
                       crossX, isObst, CROSSX_CNT);
          if ((crossCnt % 2) != 0)
            printf("Got an odd number of crossings (poly %d, at y = %f) - should not be\n",
                      i, (gcY - double(r)) * gridCellSize);
          x = crossX;
          obst = isObst;
          c = 0;
          for (j = 0; j < (crossCnt / 2); j++)
          { // get first segment in pixel coordinates
            x1 = gcX + roundi(*x++ / gridCellSize);
            x2 = gcX + roundi(*x++ / gridCellSize);
            // get obstacles flags
            obs1 = *obst++;
            obs2 = *obst++;
            // limit to within grid
            x1 = mini(w-1, maxi(0, x1));
            x2 = mini(w-1, maxi(0, x2));
            if (obs1)
              c = maxi(c, x1 - obstWidth);
            else
              c = x1;
            pix = grid->getPixRef(r, c);
            for (c = c; c < x1; c++)
            { // make probability towards obstacle
              pix->p2 = maxi(0, pix->p2 - probObst);
              pix++;
            }
            for (c = x1; c < x2; c++)
            { // make probability towards passable
              pix->p2 = mini(255, pix->p2 + probFree);
              pix++;
            }
            if (obs2)
            { // far end is obstacle
              for (c = x2; c < mini(w, x2 + obstWidth); c++)
              { // make probability towards obstacle
                pix->p2 = maxi(0, pix->p2 - probObst);
                pix++;
              }
            }
          }
        }
      }
      //
      // advance to one step older
      n = (n + 1) % MAX_POLYGON_COUNT;
      // reduce influence of older polygons
      //probFree -= 4;
      //probObst -= 4;
    }
    //
    // debug timing
    t2.Now();
    printf("Make probability grid took %f secs\n", t2 - t1);
    // debug end timing
    //
    // debug
    grid->saveBMP(imagePath, sourceName, polysCnt, "probGrid");
    saveMatlab(sourceName);
    // debug end
  }
  //
  return res;
}

////////////////////////////////////////////////////////

void UProbGrid::saveMatlab(const char * filename)
{
  FILE * fm = NULL;
  const int FNL = 500;
  char fnm[FNL];
  UPixel * rgb;
  int r,c;
  //
  snprintf(fnm, FNL, "%s/%s.m", dataPath, filename);
  // avoid non-matlab compatible characters in filename '.'
  for (c = strlen(dataPath); c < int(strlen(fnm)) - 3; c++)
    if (fnm[c] == '.')
      fnm[c] = '_';
  //
  fm = fopen(fnm, "w");
  if (fm != NULL)
  {
    fprintf(fm, "%% Probability grid values fro MATLAB\n");
    fprintf(fm, "%% grid cell size [in meters]:\n");
    fprintf(fm, "cellSize = %e;\n", gridCellSize);
    fprintf(fm, "%% grid x axis values:\n");
    fprintf(fm, "xVec = %d:%d;\n", -gcX, grid->width() - gcX - 1);
    fprintf(fm, "xVec = xVec * cellSize;\n");
    fprintf(fm, "%% grid y axis values:\n");
    fprintf(fm, "yVec = %d:%d;\n", gcY - (grid->height() - 1), gcY);
    fprintf(fm, "yVec = yVec * cellSize;\n");
    fprintf(fm, "%% grid values:\n");
    fprintf(fm, "pgrid = zeros(%d, %d);\n", grid->height(), grid->width());
    for ( r = 1; r <= int(grid->height()); r++)
    {
      rgb = grid->getLine(grid->height() - r);
      fprintf(fm, "pgrid(%d,:) = [", r);
      for (c = 1; c < int(grid->width()); c++)
      {
        fprintf(fm, " %3d", rgb->p2);
        rgb++;
      }
      fprintf(fm, " %d];\n", rgb->p2);
    }
    fprintf(fm,"%% end of grid\n");
    //
    fclose(fm);
  }
}

