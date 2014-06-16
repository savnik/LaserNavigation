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
#include "uimg3dpoint.h"


void UImg3Dpoints::toRobotCoordinates() 
{
  int i;
  UImg3Dpoint * pt;
  UMatrix4 mi2r;
  UPosition pr;
  // convert to robot coordinates
  if (inPoseCoordinates)
  { // get conversion matrix from sensor pose
    mi2r = pose.getRtoMMatrix();
    pt = p3d;
    for (i = 0; i < p3dCnt; i++)
    {
      pr = mi2r * pt->pos;
      pt->pos = pr;
      pt++;
    }
    inPoseCoordinates = false;
  }
}

////////////////////////////////////////

UPlane UImg3Dpoints::getRansacPlane(int samples, double sigma, UPlane refPlane,
                        double tiltLim, double xLimit, int * cnt)
{
//  int samples = 160;
  const int MPC = 160;
  UPlane planes[MPC];
  //int planesHit[MPC];
  int planesPts[MPC][3];
  int planesCnt = 0;
  int best = -1;
  int bestCnt = 0;
  int i, n, k, m[3], rej;
  double rsc = p3dCnt / (RAND_MAX + 1.0);
  UPlane * plane;
  int * planePts;
  UPosition p[3];
  UImg3Dpoint * pp;
  double d;
  UPosition refNormal = refPlane.getNormal();
  //
  planePts = planesPts[0];
  plane = planes;
  rej = 0;
  for (i = 0; i < samples; i++)
  { // get 3 points at random
    do
    { // ensure all 3 points are different
      for (n = 0; n < 3; n++)
      {
        m[n] = (int) (rsc * rand());
        p[n] = p3d[m[n]].pos;
      }
    } while (m[0] == m[1] or m[0] == m[2] or m[1] == m[2]);
    // no test to see if two sets are equal
    //
    // make plane
    plane->set(p[0], p[1], p[2]);
    // test if plane is too far away from beeing horizontal
    d = refNormal.dot(plane->getNormal());
    if (d < tiltLim)
    { // NB! this reduces the number of found samples
      rej++;
    }
    else
    { // count points near this plane
      pp = p3d;
      k = 0;
      for (n = 0; n < p3dCnt; n++)
      {
        if (pp->pos.x > xLimit)
          d = 100.0;
        else
          d = plane->dist(pp->pos);
        if (d < sigma)
          k++;
        pp++;
      }
      //planesHit[i] = k;
      if (k > bestCnt)
      {
        best = i;
        bestCnt = k;
      }
    }
    //
    plane++;
    planePts++;
    planesCnt++;
  }
  //printf("Rejected %d of %d due to tilt\n", rej, samples);
  if (best >= 0)
  {
    plane = &planes[best];
    if (cnt != NULL)
      *cnt = bestCnt;
  }
  return *plane;
}

////////////////////////////////////////////////////////
bool UImg3Dpoints::makePCLFile(const char * name, bool andRGB, bool andRowCol) //makes a single PCL file
{
  const int MNL = 500;
  char FileName[MNL];
  const char * fn = FileName;
  char s[40];
  FILE* PCLFile;
  //
  if (name != NULL and strlen(name) > 0)
  {
    if (name[0] == '.' or name[0] == '/' or name[0] == '~')
      fn = name;
    else
      snprintf(FileName, MNL, "%s/%s", dataPath, name);
  }
  else
  {
    time.getForFilename(s, true);
    snprintf(FileName, MNL, "%s/cloud%06u_%s.pcd", dataPath, serial, s);
    printf("writing to %s\n",FileName);
  }
  PCLFile = fopen(fn,"w");
  if (PCLFile != NULL)
  {

//file format found at http://pointclouds.org/documentation/tutorials/pcd_file_format.php#pcd-file-format
/*
# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 213
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 213
DATA ascii
0.93773 0.33763 0 4.2108e+06
...
*/
    fprintf(PCLFile,"# .PCD v.7 - Point Cloud Data file format\n");
    fprintf(PCLFile,"VERSION .7\n");
    if (andRGB and andRowCol)
    {
      fprintf(PCLFile,"FIELDS x y z rgb r c\n");
      fprintf(PCLFile,"SIZE 4 4 4 4 4 4\n");
      fprintf(PCLFile,"TYPE F F F U U U\n");
      fprintf(PCLFile,"COUNT 1 1 1 1 1 1\n");
    }
    else if (andRGB)
    {
      fprintf(PCLFile,"FIELDS x y z rgb\n");
      fprintf(PCLFile,"SIZE 4 4 4 4\n");
      fprintf(PCLFile,"TYPE F F F U\n");
      fprintf(PCLFile,"COUNT 1 1 1 1\n");
    }
    else
    {
      fprintf(PCLFile,"FIELDS x y z\n");
      fprintf(PCLFile,"SIZE 4 4 4\n");
      fprintf(PCLFile,"TYPE F F F\n");
      fprintf(PCLFile,"COUNT 1 1 1\n");
    }
    fprintf(PCLFile,"WIDTH %d\n",p3dCnt);
    fprintf(PCLFile,"HEIGHT %d\n",1);
    fprintf(PCLFile,"VIEWPOINT 0 0 0 1 0 0 0\n");
    fprintf(PCLFile,"POINTS %d\n",p3dCnt);
    fprintf(PCLFile,"DATA ascii\n");
    UImg3Dpoint * p = p3d;
    for (int i = 0; i < p3dCnt; i++)
    {
      if (andRGB and andRowCol)
        fprintf(PCLFile,"%.3f %.3f %.3f %lu %u %u\n", -p->pos.y, -p->pos.z, p->pos.x, p->pixLeft, p->row, p->column);
      else if (andRGB)
        fprintf(PCLFile,"%.3f %.3f %.3f %lu\n", -p->pos.y, -p->pos.z, p->pos.x, p->pixLeft);
      else
        fprintf(PCLFile,"%.3f %.3f %.3f\n", -p->pos.y, -p->pos.z, p->pos.x);
      p++;
    }
    fclose(PCLFile);
  }
  return (PCLFile != NULL);
}
