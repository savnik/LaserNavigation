/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistPoster emneribute it and/or modify  *
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
#include <urob4/ucmdexe.h>

#include "uprobpoly.h"

UProbPoly::UProbPoly()
{
  pointsMax = MAX_POINTS;
  points = data;
  ppoints = pdata;
  clear();
}

////////////////////////////////////////

UProbPoly::~UProbPoly()
{
}

////////////////////////////////////////

bool UProbPoly::setPoly(UPose robPose, UTime poseTime)
{
  poseOrg = robPose;
  poseNow = robPose;
  poseOrgTime = poseTime;
  valid = true;
  //
  return true;
}

///////////////////////////////////////////

void UProbPoly::clear()
{
  UPolygon::clear();
  valid = false;
  seedCromaSD = 0.0;
}

///////////////////////////////////////////

bool UProbPoly::addPos(UPosition pos, bool hardEdge)
{
  bool result = (pointsCnt < MAX_POINTS);
  //
  if (result)
  {
    isObst[pointsCnt] = hardEdge;
    add(pos);
  }
  return result;
}


///////////////////////////////////////////

bool UProbPoly::moveToPose(UPose poseNew, UTime poseTime)
{
  bool res = true;
  int n;
  UPosition * pnt = points;
  // new pose seen from present position
  UPose dp = poseNew - poseNow;
  //
  if ((absd(dp.x) + absd(dp.y) + absd(dp.h)) > 0.001)
  { // act only if robot has mooved
    for (n = 0; n < pointsCnt; n++)
    { // convert points as seen from new position
      *pnt = dp.getMapToPose(*pnt);
      pnt++;
    }
    // save new reference pose
    poseNow = poseNew;
  }
  //
  return res;
}

///////////////////////////////////////////

bool UProbPoly::moveToMap(UPose toPose)
{
  bool res = true;
  int n;
  UPosition * pnt = points;
  // new pose seen from present position
  for (n = 0; n < pointsCnt; n++)
  { // convert points as seen from new position
    *pnt = toPose.getPoseToMap(*pnt);
    pnt++;
  }
  poseNow = toPose;
  //
  return res;
}

/////////////////////////////////////////////////////

bool UProbPoly::paintRobot(UImage * img, int refX, int refY, double cellSize)
{
  bool res = (img != NULL);
  UPosition mp;
  CvPoint mpt[4];
  CvScalar rgb = CV_RGB(255, 128, 128);
  UPose pose = poseOrg - poseNow;
  //
  if (res)
  { // The robot  is 65 cm long and 45 cm wheel distance
    mp = pose.getPoseToMap(UPosition::position(0.0,   0.225, 0.0)); // left wheel
    mpt[0].x = refX + roundi(mp.x / cellSize);
    mpt[0].y = refY - roundi(mp.y / cellSize);
    mp = pose.getPoseToMap(UPosition::position(0.0,  -0.225, 0.0)); // right wheel
    mpt[1].x = refX + roundi(mp.x / cellSize);
    mpt[1].y = refY - roundi(mp.y / cellSize);
    mp = pose.getPoseToMap(UPosition::position(0.65,  0.0,   0.0)); // front wheel
    mpt[2].x = refX + roundi(mp.x / cellSize);
    mpt[2].y = refY - roundi(mp.y / cellSize);
    mp = pose.getPoseToMap(UPosition::position(-0.25,  0.0,   0.0)); // back edge
    mpt[3].x = refX + roundi(mp.x / cellSize);
    mpt[3].y = refY - roundi(mp.y / cellSize);
    // paint robot triangle (arrow)
    cvLine(img->cvArr(), mpt[0], mpt[1], rgb, 1, 8, 0);
    cvLine(img->cvArr(), mpt[1], mpt[2], rgb, 1, 8, 0);
    cvLine(img->cvArr(), mpt[2], mpt[0], rgb, 1, 8, 0);
    cvLine(img->cvArr(), mpt[2], mpt[3], rgb, 1, 8, 0);
  }
  //
  return res;
}


///////////////////////////////////////////

// bool UProbPoly::getLimits(double * minX, double * maxX,
//                           double * minY, double * maxY)
// {
//   bool res = (pointsCnt > 0) and
//              (minX != NULL) and
//              (maxX != NULL) and
//              (minY != NULL) and
//              (maxY != NULL);
//   int i;
//   UPosition * pnt = poly;
//   //
//   if (res)
//   { // initialize using first point
//     *minX = pnt->x;
//     *maxX = *minX;
//     *minY = pnt->y;
//     *maxY = *minY;
//     pnt++;
//     //
//     for (i = 1; i < polyCnt; i++)
//     {
//       if (pnt->x < *minX)
//         *minX = pnt->x;
//       if (pnt->x > *maxX)
//         *maxX = pnt->x;
//       if (pnt->y < *minY)
//         *minY = pnt->y;
//       if (pnt->y > *maxY)
//         *maxY = pnt->y;
//       pnt++;
//     }
//   }
//   return res;
// }

////////////////////////////////////////

int UProbPoly::getCrossingsAtX(double atX, // xrossings of this x value
                     double * ys,  // buffer for results
                     bool * isYAnObst,
                     int bufSize)  // buffer size
{
  bool res = (ys != NULL) and (bufSize > 0);
  int i;
  UPosition * p2 = points;
  UPosition * p1 = p2;
  U2Dline linRef;
  U2Dline linPol;
  bool isOK;
  double * yVal = ys;
  float x, y;
  int yCnt = 0;
  bool * obs1 = isObst;
  bool * obs2 = obs1;
  //
  if (res)
  {  // set reference line
    linRef.setPV(atX, 0.0, 0.0, 1.0);
    p1++; // p1 points to element point + 1
    obs1++;
    for (i = 1; i < pointsCnt; i++)
    {
      if (((p1->x > atX) and (p2->x < atX)) or
          ((p1->x < atX) and (p2->x > atX)))
      { // there is a crossing
        linPol.set2P(p1->x, p1->y, p2->x, p2->y);
        isOK = linPol.getCrossing(linRef, &x, &y);
        if (isOK)
        { // advance to next line
          *yVal++ = double(y);
          if (isYAnObst != NULL)
            *isYAnObst++ = (*obs1 or *obs2);
          if (yCnt >= bufSize)
            // too many crossings
            break;
          yCnt++;
        }
      }
      p2 = p1++;
      obs2 = obs1++;
    }
  }
  return yCnt;
}

////////////////////////////////////////

int UProbPoly::getCrossingsAtY(double atY, // xrossings of this y value
                     double * xs,  // buffer for results
                     bool * xIsAnObst,
                     int bufSize)  // buffer size
{
  bool res = (xs != NULL) and (bufSize > 0);
  int i, j;
  UPosition * p1 = points;
  UPosition * p2 = p1;
  bool * obs1 = isObst;
  bool * obs2 = obs1;
  U2Dline linRef;
  U2Dline linPol;
  bool isOK;
  double * xVal = xs;
  bool * obst = xIsAnObst;
  float x, y;
  int xCnt = 0;
  double vd;
  bool vb;
  //
  if (res)
  {  // set reference line (position-vector format)
    linRef.setPV(0.0, atY, 1.0, 0.0);
    p1++; // p1 points to element point + 1
    obs1++;
    for (i = 1; i < pointsCnt; i++)
    {
      if (((p1->y > atY) and (p2->y < atY)) or
          ((p1->y < atY) and (p2->y > atY)))
      { // there is a crossing
        linPol.set2P(p1->x, p1->y, p2->x, p2->y);
        isOK = linPol.getCrossing(linRef, &x, &y);
        if (isOK)
        { // save result
          *xVal++ = double(x);
          *obst++ = (*obs1 or *obs2);
          // sort x values so that the least is first
          for (j = xCnt-1; j > 0; j--)
          { // boble the newest value down
            if (xs[j] < xs[j-1])
            { // swap
              vd = xs[j-1];
              xs[j-1] = xs[j];
              xs[j] = vd;
              // swap also the obst value
              vb = xIsAnObst[j-1];
              xIsAnObst[j-1] = xIsAnObst[j];
              xIsAnObst[j] = vb;
            }
            else
              // all is now OK
              break;
          }
          if (xCnt >= bufSize)
            // too many crossings
            break;
          xCnt++;
        }
      }
      p2 = p1++;
      obs2 = obs1++;
    }
    // debug
    /*
    for (j = 0; j < xCnt; j++)
    {
      printf("Line at y = %f (%2d) : %f obs1 %s, obs2 %s\n",
                atY, j, x, bool2str(*xIsAnObst), bool2str(*xIsAnObst));
    } */
    // debug end
  }
  return xCnt;
}

////////////////////////////////////////

void UProbPoly::print(const char * prestring, bool verbose)
{
  int i;
  const int SL = 40;
  char s[SL];
  printf("%s polygon with %d elements ", prestring, pointsCnt);
  poseOrgTime.print(" at");
  if (verbose)
  {
    poseNow.print("Pose now");
    poseNow.print("Pose org");
    for (i = 0; i < pointsCnt; i++)
    {
      snprintf(s, SL, "#%3d obst %5s", i, bool2str(isObst[i]));
      points[i].print(s);
    }
  }
}

////////////////////////////////////////

// bool UProbPoly::sendSml(UServerInMsg * msg, const char * name)
// {
//   bool result = (msg != NULL);
//   const int MRL = 200;
//   char reply[MRL];
//   UCmdExe * exe;
//   UPosition * pos;
//   int i;
//   //
//   if (result)
//   {
//     exe = msg->exe;
//     result = (exe != NULL);
//   }
//   if (result)
//   { // Send a reply
//     snprintf(reply, MRL, "<%s name=\"%s\" type=\"UProbPoly\">",
//                          msg->tag.getTagName(), name);
//     exe->sendMsg(msg->client, reply);
//     // send UPrbPoly header
//     snprintf(reply, MRL, "<UProbPoly count=\"%d\" obstFlags=\"true\">\n", polyCnt);
//     exe->sendMsg(msg->client, reply);
//     // send time
//     exe->sendMsg(msg->client, poseOrgTime.getAsSml("imgTime", reply, MRL));
//     // send source pose
//     poseOrg.getAsSml("org", reply, MRL);
//     exe->sendMsg(msg->client, reply);
//     // send polygon values
//     pos = poly;
//     for (i = 0; i < polyCnt; i++)
//     { // send 2D with binary flag
//       snprintf(reply, MRL, "<pos2db x=\"%g\" y=\"%g\" obst=\"%s\"/>\n",
//          pos->x, pos->y, bool2str(isObst[i]));
//       exe->sendMsg(msg->client, reply);
//       pos++;
//     }
//     // debug
//     printf("UProbPoly send polyCnt vertices to client %d\n", msg->client);
//     // debug end
//     // send UProbPoly end tag
//     snprintf(reply, MRL, "</UProbPoly>");
//     exe->sendMsg(msg->client, reply);
//     // send end tag
//     snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
//     result = exe->sendMsg(msg->client, reply);
//   }
//   //
//   return result;
// }

////////////////////////////////////////

int UProbPoly::getCrossingsAtXinYorder(double atX, // xrossings of this x value
                               double * ys,  // buffer for results
                               bool * isYAnObst,
                               int bufSize)  // buffer size
{
  int i, j;
  int hits;
  double b;
  bool a;
  //
  hits = getCrossingsAtX(atX, ys, isYAnObst, bufSize);
  for (i = 1; i < hits; i++)
  {
    for (j = i; j > 0; j--)
    {
      if (ys[j] > ys[j-1])
      { // swap
        b = ys[j-1];
        ys[j-1] = ys[j];
        ys[j] = b;
            // swap also ysObst
        a = isYAnObst[j-1];
        isYAnObst[j-1] = isYAnObst[j];
        isYAnObst[j] = a;
      }
      else
        break;
    }
  }
  return hits;
}

////////////////////////////////////////


