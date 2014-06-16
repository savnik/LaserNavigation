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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>

//#include "umapsegment.h"
//#include "umapobj.h"
//#include "umaprobot.h"
//#include "umapgmk.h"
//#include "umaprel.h"
#include "upose2pose.h"
#include "umanseq.h"
#include "umanline.h"
#include "umanarc.h"

using namespace std;

void testMapSave();
void testMapLoad();
void testPose();
void testPoseCoordinateTransfer();
void testTurnR();
void testManSeq();
void testManDistance();
void testTurn2();
void testPoseToPose();
void testManToSeg();
void testManLineToSeg();
void testPose2Pose();
void testManSeqDeviation();
void testPoseDist();
void testDriveonEst();
void testManPoly();
void testPoseMatrix();

const char pathAtDTU[] = "/vhome/jca/chr/results";
const char pathAtHOME[] = "/home/chr/chr/results";
const char * pathResults = pathAtHOME;


///////////////////////////////////////////////////

int main(int argc, char *argv[])
{

  //int result = -1;

  // UMapSegment * map;
  int n;
  // adjust path from program parameter
  for (n = 0; n < argc; n++)
    if (strcmp(argv[n], "DTU") == 0)
      pathResults = pathAtDTU;
  //
//  cout << "This is a library project, and has no usable function" << endl;
//  cout << "Just here to test the library functions" << endl;
//  cout << "" << endl;
  //  testMapSave();
  //  testMapLoad();
  //testPose();
  //testTurnR();
  //testManSeq();
  //testManDistance();
  //testPoseCoordinateTransfer();
  //testTurn2();
  //testPoseToPose();
  // testManToSeg();
  // testManLineToSeg();
  // testPose2Pose();
  //testManSeqDeviation();
  // testPoseDist();
  //testDriveonEst();
  //testManPoly();
  testPoseMatrix();
  //
  return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////

// void testMapSave()
// {
//   UMapSegment * map;
//   UMapSegment * lmap;
//   UMapObj * mo = NULL;
//   UMapObj * mor = mo;
//   UMapRel * mr;
//   //UMapRobot * mor;
//   //UMapGmk * mog;
//   UMatrix4 mQ(3,3,5.1);
//   UTime t;
//   int i,j;
//   UPosition pos;
//   URotation ori;
//   list<UMapObj>::iterator moi;
//   list<UMapRel>::iterator mri;
//   //
//   map = new UMapSegment(NULL);
//   // initialize map
//   map->setNextSerial(100);
//   map->setMapLimits(-10.0, 11.0, -9.0, 8.0);
//   // fill some map objects and positions
//   for (i = 0; i < 5; i++)
//   {
//     t.Now();
//     mo = map->addObj(2.0+i, 3.0-i, PI/(4.0+double(i)), &mQ, t);
//     // make some history
//     for (j = 0; j < i; j++)
//       mo->moveTo(2.0+i+0.1*(j+1.0), 3.0-i+0.1*(j+1.0));
//     switch (i%5)
//     {
//       case 1:
//         pos.set(5.1, 6.1, 7.1);
//         mo->set3dPos(pos);
//         break;
//       case 2:
//         mo->setGmk(2000+i, 0.02, 7);
//         pos.set(2.1, 2.3, 1.1);
//         ori.set(0.001, 0.002, 0.003);
//         mo->set3dPos(pos, NULL, &ori);
//         break;
//       case 3:
//         mo->setRobot(10+i, 0.28, 1.0e-4, 0.0, 0.0, 0.0);
//         // save a copy
//         mor = mo;
//         break;
//       case 4:
//         mo->setSegment(mo, 1, "Loacl 1");
//         lmap = mo->getSegment();
//         if (lmap != NULL)
//         {// add also 2 object to new local map
//          lmap->addObjCopy(mor);
//           mo = map->addObj(12.0, 13.0, 0.0, &mQ, t);
//         }
//         break;
//       default:
//         break;
//     }
//   }
//   // add 2 relations
//   moi = map->begin();
//   // first relation (object 1 and robot (3))
//   mo = moi->get();
//   mr = map->addRel(mo, mor);
//   mr->getRelPose(true)->setPose(-1.0, 1.0, 0.0);
//   mr->getRelPose(true)->setQ(&mQ);
//   // second relation (object 2 and robot (3))
//   advance(moi, 1);
//   mo = moi->get();
//   mr = map->addRel(mo, mor);
//   mr->getRelPose(true)->setPose(22.0, 33.0, 3.14);
//   mQ.add(2.0);
//   mr->getRelPose(true)->setQ(&mQ);
//   //
//   // debug reiterate map objects
//   moi = map->begin();
//   while (moi != map->end())
//   {
//     mo = moi->get();
//     advance(moi, 1);
//   }
//   mri = map->beginRel();
//   while (mri != map->endRel())
//   {
//     mr = mri->get();
//     advance(mri,1);
//   }
//   // debug end
//   //
//   if (map->save("testmap", pathResults))
//     printf("Map saved successfully\n");
//   else
//     printf("Map save error\n");
//   //
//   delete map;
// }

//////////////////////////////////////////

// void testMapLoad()
// {
//   bool result;
//   UMapSegment * map;
//   //
//   map = new UMapSegment(NULL);
//   // load from file
//   result = map->load("testmap", pathResults);
//   if (result)
//     printf("Map (testmap) loaded successfully\n");
//   else
//     printf("Map load error\n");
//   // set map limits
//   map->updateLimits();
//   result = map->save("testmap-loaded-and-saved", pathResults);
//   if (result)
//     printf("Map re-saved successfully\n");
//   else
//     printf("Map re-save error\n");
//   //
//   // now load this map and make an (exact) copy.
//   //
//   map->clear();
//   // load from file
//   result = map->load("testmap-loaded-and-saved", pathResults);
//   if (result)
//     printf("Map reloaded successfully\n");
//   else
//     printf("Map load error\n");
//   // set map limits
//   map->updateLimits();
//   result = map->save("testmap-loaded-and-resaved", pathResults);
//   if (result)
//     printf("Map re-re-saved successfully\n");
//   else
//     printf("Map re-re-save error\n");
//   //
//   delete map;
// }

////////////////////////////////////////

void testPose()
{
  UPose v1(0.0, 0.0, 0.0);
  UPose v2(2.0, 1.0, M_PI/4.0);
  UPose v3(3.0, 3.0, M_PI/2.0);
  UPose v4(0.0, 3.0, M_PI);
  UPose pd;
  UPose ps;
  //
  //
  ps = v1;
  ps.fprint(stdout, "before");
  // move to v2
  pd = v2 - v1;
  ps = ps + pd;
  // move to v3
  pd = v3 - v2;
  ps = ps + pd;
  // move to v4
  pd = v4 - v3;
  ps = ps + pd;
  // move to v1
  pd = v1 - v4;
  ps = ps + pd;
  // now back
  pd = v1 - v4;
  ps = ps + pd.neg();
  // back to v3
  pd = v4 - v3;
  ps = ps + pd.neg();
  // back to v2
  pd = v3 - v2;
  ps = ps + pd.neg();
  // back to v1
  pd = v2 - v1;
  ps = ps + pd.neg();

  //
  ps.fprint(stdout, "now");
}

//////////////////////////////////////////

void testPoseCoordinateTransfer()
{
  UPose ps1(1.0, 1.0, 1.57);
  //UPose ps2(0.0, 0.0, 0.05);
  UPosition pos1(5.0, 2.0, 7.7);
  UPosition pos2;
  //
  ps1.fprint(stdout, "Pose position");
  pos1.show(" -  Map position");
  pos2 = ps1.getMapToPose(pos1);
  pos2.show(" -> Pose position");
  printf("And back\n");
  pos1 = ps1.getPoseToMap(pos2);
  pos1.show(" - Map position");
}

//////////////////////////////////////////////

// void testTurnR()
// {
//   //UPose2pose p1(0.0, 0.0, 0.0);
//   UPose2pose p2(5.0, -0.1, -85.0*M_PI/180.0, 1.0);
//   double uA2; // unknown 1. Phi2
//   double uR; // unknown 2. Turn radius (common)
//   double uA1;
//   double uD; // line distance
//   int mt;
//   bool isOK;
//   bool left1st;
//   int MST = 72;
//   int i;
//   FILE * fp = fopen("/home/jca/chr/results/pose2pose_5.0_-0.1.log", "w");
//   //
//   for (i = 0; i < MST; i++)
//   {
//     p2.h = double(i) * M_PI * 2.0 / double(MST) - M_PI;
//     isOK = p2.get2here(&mt, &uD, &uR, &uA1, &uA2, &left1st, fp);
//     //p2.print("Destination");
//     printf("x=%5.2f,y=%5.2f,h=%5.2f: (%s)\n",
//            p2.x, p2.y, p2.h * 180.0 / M_PI, bool2str(isOK));
//     fprintf(fp, "x=%5.2f,y=%5.2f,h=%5.2f: (%s) ",
//            p2.x, p2.y, p2.h * 180.0 / M_PI, bool2str(isOK));
//     switch (mt)
//     {
//       case 0: // 2 arc
//         printf("2Arc     left=%s  R=%7.2f, Arc1=%5.1f, Arc2=%5.1f (deg)\n",
//               bool2str(left1st), uR, uA1*180.0/M_PI, uA2*180.0/M_PI);
//         fprintf(fp, "2Arc     left=%s  R=%7.2f, Arc1=%5.1f, Arc2=%5.1f (deg)\n",
//               bool2str(left1st), uR, uA1*180.0/M_PI, uA2*180.0/M_PI);
//         break;
//       case 1:
//         printf("Line Arc left=%s  R=%7.2f, Arc1=%5.1f (deg), Line=%5.3f\n",
//               bool2str(left1st), uR, uA1*180.0/M_PI, uD);
//         fprintf(fp, "Line Arc left=%s  R=%7.2f, Arc1=%5.1f (deg), Line=%5.3f\n",
//               bool2str(left1st), uR, uA1*180.0/M_PI, uD);
//         break;
//       case 2:
//         printf("Arc Line left=%s  R=%7.2f, Arc1=%5.1f (deg), Line=%5.3f\n",
//               bool2str(left1st), uR, uA1*180.0/M_PI, uD);
//         fprintf(fp, "Arc Line left=%s  R=%7.2f, Arc1=%5.1f (deg), Line=%5.3f\n",
//               bool2str(left1st), uR, uA1*180.0/M_PI, uD);
//         break;
//       case 3:
//         printf("Line      Dist=%5.3f\n", uD);
//         fprintf(fp, "Line      Dist=%5.3f\n", uD);
//         break;
//       default:
//         printf("unknovn manoeuver\n");
//         fprintf(fp, "unknovn manoeuver\n");
//         break;
//     }
//   }
//   if (fp != NULL)
//     fclose(fp);
// }

///////////////////////////////////////////////


void testMan()
{
  double startV = -2.0; // start speed in m/sec
  UManLine ml;
  double a,d,v,t, ev;
  // test som man calculations
  a = -0.6; // acceleration
  d = 1.0; // distance
  ev = -3.1; // end vel
  ml.setAcc(a);
  ml.setDistance(d);
  ml.setVel(ev);
  v = ml.getEndV(startV);
  t = ml.getManTime(startV);
  printf("From %g m/s to %g m/s acc=%gm/s2 takes %g m and %g sec\n",
         startV, v, a, d, t);
}

/////////////////////////////////////////

void testManSeq()
{
  double startV = 0.7; // start speed in m/sec
  UPoseV fromP(0.0, 0.0, 0.0, startV); // start position
  UPoseV p1(3.0, 0.0, 0.0, 1.2); // desired end pose-speed
  UPoseV p2(10.0, 3.0, 90.0 * M_PI / 180.0, 1.3); // desired end pose-speed
  UPoseV p3(3.0, 10.0, M_PI, 1.3); // desired end pose-speed
  UPoseV p4(0.0, 10.0, -M_PI / 2.0, 1.1); // desired end pose-speed
  UPoseV p5(0.0, 0.0, 0.0, 0.5); // desired end pose-speed
  UManSeq seq;
  UPosition pos(11.0, 10.5, 0.0); // obstacle
  UPose pHit;
  int idx;
  bool posIsRight;
  int w;
  double t; // distance into route segment
  double d;
  const double maxDist = 1.5;
  double okVel;
  bool isOK;
  const int MSL = 2000;
  char s[MSL];
  // Testing manoeuver from a start pose (fromP) to an end pose (endP)
  // adding the needed manuevres to the manoeuvre sequence (seq)
  // and print the result.
  //
  fromP.fprint(stdout,"Start pose");
  p1.fprint(stdout,  "End pose  ");
  p2.fprint(stdout,  "End pose  ");
  p3.fprint(stdout,  "End pose  ");
  p4.fprint(stdout,  "End pose  ");
  p5.fprint(stdout,  "End pose  ");
  // add manoeuvre to stack.
  seq.releaseAllMan();

  isOK = seq.addMan(fromP, p1, 0.7, 0.3, 0.2, &okVel);
  p1.setVel(seq.getEndPoseV().getVel());
  seq.print( "to 3,0,0)", s, MSL);
  printf("%s\n", s);
  isOK = seq.addMan(p1, p2, 0.7, 0.3, 0.2, &okVel);
  p2.setVel(seq.getEndPoseV().getVel());
  seq.print( "to 10,0,pi/2)", s, MSL);
  printf("%s\n", s);
  isOK = seq.addMan(p2, p3, 0.7, 0.3, 0.2, &okVel);
  p3.setVel(seq.getEndPoseV().getVel());
  isOK = seq.addMan(p3, p4, 0.7, 0.3, 0.2, &okVel);
  p4.setVel(seq.getEndPoseV().getVel());
  isOK = seq.addMan(p4, p5, 0.7, 0.3, 0.2, &okVel);
  // print result.
  printf("---------- now the full square ----------\n");
  seq.fprint(stdout, "ppseq");
  // find position closest to this point, should be in segment 3
  posIsRight = true;
  d = seq.getDistanceXYSigned(pos, &w, posIsRight, maxDist, &pHit, &idx, &t);
  //
  printf("Finished, result=%s\n", bool2str(okVel >= p4.getVel()));
}

////////////////////////////////////////////////////////

void testManSeqDeviation()
{
  double startV = 0.7; // start speed in m/sec
  UPoseV fromP(0.0, 0.0, 0.0, startV); // start position
  UPoseV p1(3.0,   0.3, 0.1, 0.2); // desired end pose-speed
  UPoseV p2(10.0,  -.3, -0.11, 0.2); // desired end pose-speed
  UPoseV p3(13.0, -.50, 0.0, 0.2); // desired end pose-speed
  UManSeq seq;
  UPosition pos(11.0, 10.5, 0.0); // obstacle
  UPose pHit;
  double d, h;
  double okVel;
  bool isOK;
  const int MSL = 2000;
  char s[MSL];
  // Testing manoeuver from a start pose (fromP) to an end pose (endP)
  // adding the needed manuevres to the manoeuvre sequence (seq)
  // and print the result.
  //
  fromP.fprint(stdout,"Start pose");
  p1.fprint(stdout,  "End pose  ");
  p2.fprint(stdout,  "End pose  ");
  p3.fprint(stdout,  "End pose  ");
  // add manoeuvre to stack.
  seq.releaseAllMan();

  isOK = seq.addMan(fromP, p1, 0.7, 0.3, 0.2, &okVel);
  p1.setVel(seq.getEndPoseV().getVel());
  seq.print( "#p1", s, MSL);
  printf("%s\n", s);
  isOK = seq.addMan(p1, p2, 0.7, 0.3, 0.2, &okVel);
  p2.setVel(seq.getEndPoseV().getVel());
  seq.print( "#p2", s, MSL);
  printf("%s\n", s);
  isOK = seq.addMan(p2, p3, 0.7, 0.3, 0.2, &okVel);
  p3.setVel(seq.getEndPoseV().getVel());
  // find position closest to this point, should be in segment 3
  d = seq.getDistanceFromEndPoseLine(&h);
  //
  printf("max deviation is %gm and %g radians\n", d, h);
}

////////////////////////////////////////////////////////

void testManDistance()
{
  UManLine ml;
  UManArc ma;
  UPosition pos(10.0, -0.0, 0.0);
  UPose pHit;
  double d, t;
  int w;
  bool posIsRight = false;
  //
  ma.setTurnAngle(-90.0 * M_PI / 180.0);
  ma.setTurnRadius(10.0);
  pos.print("pos");
  ma.fprint(stdout, "arc");
  d = ma.getDistanceXYSigned(pos, &w, posIsRight, false, &pHit, &t);
  pHit.fprint(stdout,"pHit");
  printf("Distance=%f at %d (0=mid, 1=first, 2=last)\n", d, w);
  //
  printf("\n");
  ml.setDistance(10.0);
  ml.fprint(stdout, "line", NULL);
  d = ml.getDistanceXYSigned(pos, &w, posIsRight, false, &pHit, &t);
  pHit.print("pHit");
  printf("Distance=%f at %d (0=mid, 1=first, 2=last)\n", d, w);
  //
  printf("ended\n");
}

//////////////////////////////////////////////////////////////

void testTurn2()
{
  //UPose2pose p1(0.0, 0.0, 0.0);
  UPose2pose p2(20.0, -20.6, -70.0*M_PI/180.0, 0.707);
  //
  double r1, r2, a1, a2, d;
  const double maxAcc = 0.2;
  const double maxTurnAcc = 0.2;
  double initVel = 0.707;
  bool result;
  int mt;
  //
  printf("\n");
  p2.fprint(stdout,"to: ");
  result = p2.get2hereALA(&mt, initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, NULL);
  //
  if (result)
  {
    printf("-- %d: R1=%.3f, A1=%.4f, Dist=%.2f, R2=%.2f, A2=%.4f\n",
           mt, r1, a1, d, r2, a2);
    printf("-- Dist = %.2f + %.2f + %2f = %.2fm\n",
           r1 * a1, d, r2 * a2, r1 * a1 + d + r2 * a2);
  }
  else
    printf("Too fast inittial velocity! - slow down first\n");
}

/////////////////////////////////////////////////////

void testPoseToPose()
{
  UPose pM(1.0, -1.5, -30.0 * M_PI / 180.0);
  UPose pR(2.0, -0.2, 200.0 * M_PI / 180.0);
  UPose p1;
  //
  pM.fprint(stdout,"Map pose");
  pR.fprint(stdout,"Rob pose");
  p1 = pR.getMapToPosePose(&pM);
  p1.fprint(stdout,"Local Po");

}

//////////////////////////////////////////////////////////

void testManToSeg()
{
  UManArc mArc;
  ULineSegment seg;
  double dist;
  int wS, wM;
  UPosition p1, p2;
  UPosition pS;
  UPose pM;
  //
  printf("Testing distance from line segment to manoeuvre\n");
  mArc.setTurnAngle(1.6);
  mArc.setTurnRadius(1.0);
  mArc.fprint(stdout, "Arc");
  p1.set(1.00, 1.5, 0.0);
  p2.set(-0.5, 0.0, 0.0);
  seg.setFromPoints(p1, p2);
  seg.print("seg");
  dist = mArc.getMinDistanceXYSigned(&seg, &wS, &pS, false, &wM, &pM);
  printf("Dist = %gm, wS=%d, wM=%d\n", dist, wS, wM);
  pS.print("pS");
  pM.fprint(stdout,"pM");
  printf("----- fin -------\n");
}

//////////////////////////////////////////////////////////

void testManLineToSeg()
{
  UManLine mLine;
  ULineSegment seg;
  double dist;
  int wS, wM;
  UPosition p1, p2;
  UPosition pS;
  UPose pM;
  //
  printf("Testing distance from line segment to manoeuvre line\n");
  mLine.setDistance(1.0);
  mLine.fprint(stdout, "Line");
  p1.set(5.5,  -0.5, 0.0);
  p2.set(0.2,  1.5, 0.0);
  seg.setFromPoints(p1, p2);
  seg.print("seg");
  dist = mLine.getMinDistanceXYSigned(&seg, &wS, &pS, true, &wM, &pM);
  printf("Dist = %gm, wS=%d, wM=%d\n", dist, wS, wM);
  pS.print("pS");
  pM.print("pM");
  printf("----- fin -------\n");
}

////////////////////////////////////////////////////////////////////////

void testPose2Pose()
{
  UPose2pose pp1;
  bool result;
  double b, r1, r2, d, a1, a2;
  double iniVel = 2.01;
  double breakDist = 0.0;
  double maxAcc = 0.4;
  double maxTurnAcc = 0.3;
  double toVel;
  double minTurnRad = 0.2;
  double bFinal = 0.0;
  //
/*  maxTurnAcc = 0.3;
  initVel = 1.4;
  vel = 2.01;
  maxAcc = 0.4;
  x = 0.0;
  y = -1;
  h = 0.13;
  MIN_SPEED = 0.2;*/
  //
  pp1.x = 0.0;
  pp1.y = -1.0;
  pp1.h = 0.13; //-10 * M_PI / 180.0;
  iniVel = 1.4;
  pp1.setVel(iniVel);
  //pp1.set(4.0, -2.0, 0.0, 1.5);
  result = pp1.get2ViaBreakRightLineLeft(iniVel, maxAcc, maxTurnAcc, minTurnRad,
                                         &b, &toVel, &r1, &a1, &d, &r2, &a2, &bFinal);
  breakDist = 0;
  printf("From (0,0, 0deg) at %.2fm/s to %g, %g, %.2fdeg at %gm/s\n", iniVel, pp1.x, pp1.y, pp1.h * 180.0 / M_PI, pp1.getVel());
  printf("break %.2fm to %.2fm/s, R %.1fdeg (r=%.2fm), -> %.2fm, L %.1fdeg (r=%.2fm) break %.2fm\n",
         b, toVel, a1 * 180.0 / M_PI, r1, d, a2 * 180.0 / M_PI, r2, bFinal);
  // now to the same spot using line-right-line-right-line
  result = pp1.get2ViaBreakRightLineRight(iniVel, maxAcc, maxTurnAcc, minTurnRad,
                                         &b, &toVel, &r1, &a1, &d, &r2, &a2, &bFinal);
  //
  printf("break %.2fm to %.2fm/s, R %.1fdeg (r=%.2fm), -> %.2fm, R %.1fdeg (r=%.2fm) break %.2fm\n",
         b, toVel, a1 * 180.0 / M_PI, r1, d, a2 * 180.0 / M_PI, r2, bFinal);
  // now to the same spot using line-right-line-right-line
  result = pp1.get2ViaBreakLeftLineRight(iniVel, maxAcc, maxTurnAcc, minTurnRad,
                                          &b, &toVel, &r1, &a1, &d, &r2, &a2, &bFinal);
  //
  printf("break %.2fm to %.2fm/s, L %.1fdeg (r=%.2fm), -> %.2fm, L %.1fdeg (r=%.2fm) break %.2fm\n",
         b, toVel, a1 * 180.0 / M_PI, r1, d, a2 * 180.0 / M_PI, r2, bFinal);
  // now to the same spot using line-right-line-right-line
  result = pp1.get2ViaBreakLeftLineLeft(iniVel, maxAcc, maxTurnAcc, minTurnRad,
                                          &b, &toVel, &r1, &a1, &d, &r2, &a2, &bFinal);
  //
  printf("break %.2fm to %.2fm/s, L %.1fdeg (r=%.2fm), -> %.2fm, L %.1fdeg (r=%.2fm) break %.2fm\n",
         b, toVel, a1 * 180.0 / M_PI, r1, d, a2 * 180.0 / M_PI, r2, bFinal);
  printf("----- fin --------\n");
}

/////////////////////////////////////////////////

void testPoseDist()
{
  UPose p1, p2;
  double d;
  //
  printf("Distance (signed) to pose line - left is positive\n");
  p1.set(0.0, 0.0, 45 * M_PI / 180.0);
  p2.set(0.0, 0.0, 0.0);
  d = p1.getDistToPoseLineSigned(p2.x, p2.y);
  printf("distance from %gx, %gy to pose lint %gx, %gy, %.4fdeg is %.5fm\n",
         p2.x, p2.y, p1.x, p1.y, p1.h * 180.0 / M_PI, d);
  p2.set(1.0, 0.0, 0.0);
  d = p1.getDistToPoseLineSigned(p2.x, p2.y);
  printf("distance from %gx, %gy to pose lint %gx, %gy, %.4fdeg is %.5fm\n",
         p2.x, p2.y, p1.x, p1.y, p1.h * 180.0 / M_PI, d);
  p2.set(0.0, 1.0, 0.0);
  d = p1.getDistToPoseLineSigned(p2.x, p2.y);
  printf("distance from %gx, %gy to pose lint %gx, %gy, %.4fdeg is %.5fm\n",
         p2.x, p2.y, p1.x, p1.y, p1.h * 180.0 / M_PI, d);
  p1.set(2.0, 2.0, 175.0 * M_PI / 180.0);
  p2.set(1.0, -1.0, 0.0);
  d = p1.getDistToPoseLineSigned(p2.x, p2.y);
  printf("distance from %gx, %gy to pose lint %gx, %gy, %.4fdeg is %.5fm\n",
         p2.x, p2.y, p1.x, p1.y, p1.h * 180.0 / M_PI, d);
  p2.set(-1.0, 2.5, 0.0);
  d = p1.getDistToPoseLineSigned(p2.x, p2.y);
  printf("distance from %gx, %gy to pose lint %gx, %gy, %.4fdeg is %.5fm\n",
         p2.x, p2.y, p1.x, p1.y, p1.h * 180.0 / M_PI, d);
}

//////////////////////////////////////////////////////////////

void testDriveonEst()
{
  UPose2pose pp;
  double turnRad, vel;
  UPose po;
  double turn1, straight, turn2;
  int i;
  //
  vel = 0.7;
  turnRad = 4.0;
  if (false)
  {
    pp.set(1.0, 0.0, 63.5 * M_PI / 180.0, vel);
    pp.print("destination");
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    printf("\n- should exit at about 4.1,7.2 (%.2fx,%.2fy,%.3fr) and ~95deg (%.2f), 0 (%.2f), ~-30deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
    //
    pp.set(0.0, 0.75, 26.5 * M_PI / 180.0, vel);
    pp.print("destination");
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    printf("\n- should exit at about 4.8, 3.0 (%.2fx,%.2fy,%.3fr) and ~65deg (%.2f), 0 (%.2f), ~-30deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
          //
    pp.set(4.5, 4.5, -18.2 * M_PI / 180.0, vel);
    pp.print("destination");
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    printf("\n- should exit at about 7.9, 3.25 (%.2fx,%.2fy,%.3fr) and ~65deg (%.2f), 0 (%.2f), ~-75deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
          //
    pp.set(0.0, 8.0, 135.0 * M_PI / 180.0, vel);
    pp.print("destination");
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    printf("\n- should exit at about -1.0, 9.0 (%.2fx,%.2fy,%.3fr) and ~165deg (%.2f), 0 (%.2f), ~-25deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
          //
    pp.set(0.0, 4.5, -160.0 * M_PI / 180.0, vel);
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    pp.print("destination");
    printf("\n- should exit at about -6.2, 2.25 (%.2fx,%.2fy,%.3fr) and ~260deg (%.2f), 0 (%.2f), ~-60deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
          //
    // straight > 0.0
    //
    pp.set(11.0, 0.0, -73.0 * M_PI / 180.0, vel);
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    pp.print("destination");
    printf("\n- should exit at about 11.4, -1.3 (%.2fx,%.2fy,%.3fr) and ~16deg (%.2f), 5.4 (%.2f), ~-90deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
          //
    pp.set(0.0, 14.0, -34.0 * M_PI / 180.0, vel);
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    pp.print("destination");
    printf("\n- should exit at about 11.3, 6.5 (%.2fx,%.2fy,%.3fr) and ~66deg (%.2f), 4.6 (%.2f), ~-90deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
          //
    pp.set(0.0, 8.1, 0.0 * M_PI / 180.0, vel);
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    pp.print("destination");
    printf("\n- should exit at about 8.0, 8.1 (%.2fx,%.2fy,%.3fr) and ~90deg (%.2f), 0.1 (%.2f), ~-90deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
          //
  }

  turnRad = 1.0;
  for (i = 0; i < 10; i++)
  {
    pp.set(0.5, 0.0, double(5 - i)/100000 * M_PI / 180.0, vel);
    po = pp.get2line(turnRad, &turn1, &straight, &turn2);
    pp.print("destination");
    printf("\n- should exit at about 0.5, 0.0 (%.2fx,%.2fy,%.3fr) and ~0deg (%.2f), 0.5 (%.2f), ~-0deg (%.2f)\n\n",
          po.x, po.y, po.h  * 180 / M_PI, turn1 * 180 / M_PI, straight, turn2 * 180 / M_PI);
  }       //
       
}

///////////////////

void testManPoly()
{
  UManArc ma;
  UManLine ml;
  UPolygon40 po1, po2;
  UPose po;
  //
  po.set(0.0, 0.0, 0.0);
  ma.setTurnAngle(116.0 * M_PI / 180.0);
  ma.setTurnRadius(3.0);
  ma.getFoodprintPolygon(&po1, &po2, 5.5, 1.0, 5.5, -1.0, 0.1, &po);
  po1.print("include");
  po2.print("exclude");
  ma.setTurnAngle(-116.0 * M_PI / 180.0);
  ma.setTurnRadius(3.0);
  ma.getFoodprintPolygon(&po1, &po2, 5.5, 1.0, 5.5, -1.0, 0.1, &po);
  po1.print("include");
  po2.print("exclude");
}

///////////////////////

void testPoseMatrix()
{
  UMatrix4 m4(4,4), m1, m2;
  UPose p1, p2 ,p3, p4, p5;
  U2Dpos a1, a2, a3;

  p1.set(0.5, 1.5, 0.67);
  a2.set(1.4, -1.4);
  p2.set(1.4, -1.4, -0.67);
  a1 = p1.getPoseToMap(1.4, -1.4);
  printf("Position from pose.getPoseToMap(): %.3fx,%.3fy\n", a1.x, a1.y);
  p3 = p1.getPoseToMapPose(p2);
  p3.print("Pose from pose.getPoseToMapPose()");
  //
  // position
  m4 = p1.asMatrix3x3PtoM();
  m4.print("m4 3x3PtoM");
  m1 = m4 * a2.asCol3();
  m1.print("position from matrix calc (asMatrix3x3PtoM())");
  //
  // pose
  m4 = p1.asMatrix4x4PtoM();
  m4.print("m4 4x4PtoM");
  m1 = m4 * p2.asCol4();
  m1.print("pose from matrix calc (asMatrix4x4PtoM())");
  p5 = m1;
  p3.print("Pose from  pose=m4 * p2.asCol4");
  //
  // and back
  a3 = p1.getMapToPose(a1);
  printf("back: Position from pose.getMapToPose(): %.3fx,%.3fy\n", a3.x, a3.y);
  p4 = p1.getMapToPosePose(&p3);
  p4.print("back: Pose from pose.getMapToPosePose()");
  // position
  m4 = p1.asMatrix3x3MtoP();
  m4.print("m4 3x3MtoP");
  m1 = m4 * a1.asCol3();
  m1.print("back: position from matrix calc (asMatrix3x3MtoP())");
  //
  // pose
  m4 = p1.asMatrix4x4MtoP();
  m4.print("m4 3x3MtoP");
  m1 = m4 * p3.asCol4();
  m1.print("back: pose from matrix calc (asMatrix4x4MtoP())");
  p5 = m1;
  p5.print("back: Pose from  pose=m4 * p2.asCol4");
}


