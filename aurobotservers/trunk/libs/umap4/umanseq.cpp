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

#include <ugen4/uline.h>

#include "umanseq.h"
#include "upose2pose.h"
#include "umanarc.h"
#include "umanline.h"

UManSeq::UManSeq()
{
  int i;
  for (i = 0; i < MAX_REUSABLE_LINE_OBJ; i++)
    reuse[i] = NULL;
  reuseCnt = 0;
  for (i = 0; i < MAX_PP_MAN_CNT; i++)
    p2p[i] = NULL;
  p2pCnt = 0;
  reuseCnt = 0;
}

//////////////////////////////////////////////

UManSeq::~UManSeq()
{
  releaseAllMan();
  freeAll();
}

//////////////////////////////////////////////

void UManSeq::freeAll()
{
  int i;
  for (i = 0; i < reuseCnt; i++)
  {
    if (reuse[i] != NULL)
      delete reuse[i];
    reuse[i] = NULL;
  }
  reuseCnt = 0;
  for (i = 0; i < p2pCnt; i++)
    delete p2p[i];
  p2pCnt = 0;
}

//////////////////////////////////////////////

void UManSeq::releaseAllMan()
{
  int i;
  for (i = 0; i < p2pCnt; i++)
  {
    if (p2p[i] != NULL)
      releaseAll(p2p[i]);
  }
  p2pCnt = 0;
}

//////////////////////////////////////////////

void UManSeq::releaseLast()
{
  if (p2pCnt > 0 and p2p[p2pCnt - 1] != NULL)
    releaseAll(p2p[--p2pCnt]);
}

//////////////////////////////////////////////

void UManSeq::truncate(UManPPSeq * ppseq, int newCnt)
{
  int i;
  //
  if (ppseq != NULL)
    for (i = ppseq->getSeqCnt() - 1; i >= newCnt; i--)
      release(ppseq->extract(i));
}

//////////////////////////////////////////////

void UManSeq::releaseAll(UManPPSeq * ppseq)
{
  int i;
  // release fixation too
  ppseq->setFirstFixed(false);
  ppseq->setLastFixed(false);
  // release manoeuvres
  for (i = ppseq->getSeqCnt() - 1; i >= 0; i--)
    release(ppseq->extract(i));
}

//////////////////////////////////////////////

void UManSeq::release(UManoeuvre * toRelease)
{
  int i;
  UManoeuvre ** pm = reuse;
  bool bug = false;
  //
  // debug
  // is the object reused already?
  for (i = 0; i < reuseCnt; i++)
  {
    if (*pm == toRelease)
    {
      printf("UManSeq::release released object is released already!\n");
      bug = true; // workarounds are sometime usefull
      break;
    }
    pm++;
  }
  pm = reuse;
  // debug end
  //
  if (not bug)
  {
    for (i = 0; i < reuseCnt; i++)
    {
      if ((*pm) == NULL)
        break;
      pm++;
    }
    if (i < MAX_REUSABLE_LINE_OBJ)
    { // space to reuse
      (*pm) = toRelease;
      if (i >= reuseCnt)
        reuseCnt = i + 1;
    }
    else
      // no more reuse space - delete totally
      delete toRelease;
  }
}

//////////////////////////////////////////////

UManoeuvre * UManSeq::requestObj(int manType)
{
  int i;
  UManoeuvre ** pm = reuse;
  UManoeuvre * result;
  //
  for (i = 0; i < reuseCnt; i++)
  {
    if ((*pm) != NULL)
    {
      if ((*pm)->getManType() == manType)
        break;
    }
    pm++;
  }
  if (((*pm) == NULL) or (i >= MAX_REUSABLE_LINE_OBJ))
  { // no reusable object found
    switch (manType)
    {
      case UManoeuvre::MAN_ARC: result = new UManArc(); break;
      case UManoeuvre::MAN_LINE: result = new UManLine(); break;
      case UManoeuvre::MAN_STOP: result = new UManStop(); break;
      default: result = NULL; break;
    }
  }
  else
  { // object found to reuse
    result = (*pm);
    (*pm) = NULL;
    // reduce reuse stack if possible
    if (i == (reuseCnt - 1))
      reuseCnt = i;
  }
  return result;
}


//////////////////////////////////////////////


bool UManSeq::replaceMan(UPoseV midPose, bool fixate,
                         double maxAcc, double maxTurnAcc, double minTurnRad,
                        int fromIdx, int toIdx,
                        bool newEndpoint,
                        double * okVel)
{
  UPoseV fromPose;
  UPoseV toPose;
  UManPPSeq  *mpp1 = NULL, *mpp2 = NULL;
  bool result = ((p2pCnt + fromIdx - toIdx + 1) < MAX_PP_MAN_CNT);
  int i, d, s;
  double okInitialVel = 1001.0; // not used any more
//  double okInitialVel2;
//  const double MIN_SPEED = 0.2;
  bool lastMan;
  //
  if (result)
  { // ensure that exactly two sequental manoeuvre
    // sequences 'UManPPSeq' are available.
    fromIdx = maxi(0, fromIdx);
    toIdx = mini(p2pCnt-1, toIdx);
    mpp1 = p2p[fromIdx];
    mpp2 = p2p[toIdx];
    fromPose = mpp1->getStartPoseV();
    toPose = mpp2->getEndPoseV();
    if (fromIdx == toIdx)
    { // only one sequence, there must be 2, so add one
      toIdx++;
      if (toIdx < p2pCnt)
      { // copy one seq in from end of list
        mpp2 = p2p[p2pCnt];
        memmove(&p2p[toIdx + 1], &p2p[toIdx], sizeof(UManPPSeq*) * (p2pCnt - toIdx));
        p2p[toIdx] = mpp2;
        //
        if (mpp2 != NULL)
          // the new one should be cleared, but all is replaced
          // except the fixate flag for the last pose.
          mpp2->setLastFixed(false);
      }
      p2pCnt++;
    }
    if ((toIdx - fromIdx) > 1)
    { // two is needed only, so delete the rest
      // debug
      // if (toIdx - fromIdx > 3)
      //   printf("Big chunk huh!\n");
      // debug end
      d = fromIdx + 1;
      s = d + 1;
      for (i = fromIdx + 1; i < toIdx; i++)
      { // move surplus to end of list, one at a time
        mpp2 = p2p[d]; // save surplus
        memmove(&p2p[d], &p2p[s], sizeof(UManPPSeq*) * (p2pCnt - (fromIdx + 2)));
        p2p[--p2pCnt] = mpp2; // insert surplus at end
      }
    }
    toIdx = fromIdx + 1;
    mpp2 = p2p[toIdx];
    if (mpp2 == NULL)
    { // create new if needed
      mpp2 = new UManPPSeq();
      p2p[toIdx] = mpp2;
    }
  }
  i = 0;
  lastMan = (toIdx == p2pCnt - 1);
  while (result)
  {
    // debug
/*    if (fabs(midPose.h) > 1000)
      printf("UManSeq::replaceMan: Huge heading in midPose!!!?\n");*/
    // debug end
    if (result)
    { // now two pose-to-pose sequence objects are in place
      // then just find the needed manoeuvres to get to endpose
      // passing the mid-pose
      result = expandMan(fromPose, midPose, lastMan and newEndpoint,
                         maxAcc, maxTurnAcc, minTurnRad,
                         mpp1, &okInitialVel);
      if (result)
        mpp1->setLastFixed(fixate); // fixate if in narrow passage
      else
      {
      // debug
/*      if (not result)
        midPose.print("***UManSeq::replaceMan: Failed to expand to mid-pose 1");*/
      // debug end
      }
    }
    if (not result)
    { // stop route here
      p2pCnt = fromIdx;
      break;
    }
    else if (newEndpoint)
    { // new endpoint, so last manoeuvre is not needed
      p2pCnt = fromIdx + 1;
      break;
    }
    else
    { // and the second part, using the end velocity from first part
      midPose = mpp1->getEndPoseV(); // gets desired end velocity, not real
      // replace with actual obtained velocity
      midPose.setVel(mpp1->getEndVel());
      result = expandMan(midPose, toPose, lastMan,
                         maxAcc, maxTurnAcc, minTurnRad,
                         mpp2, &okInitialVel);
      if (not result)
      { // terminate here
        p2pCnt = fromIdx + 1;
        // debug
/*        if (not result)
          midPose.print("***UManSeq::replaceMan: Failed to expand to mid-pose 2");*/
        // debug end
        break;
      }
      else
      {
        mpp2->setFirstFixed(fixate); // fixate if in narrow passage
        break;
/*        if ((fabs(okInitialVel2) >= MIN_SPEED) and
            (fabs(okInitialVel2) < fabs(midPose.getVel())))
        { // try a new first manoeuvre with slower exit velocity
          midPose.setVel(okInitialVel2);
        }
        else if ((fabs(okInitialVel2) < MIN_SPEED) and
                  (fabs(okInitialVel) >= fabs(fromPose.getVel())) and
                  (fabs(midPose.getVel()) > MIN_SPEED))
        { // second part is bad, but first part is OK,
          // limit first part to minimum speed, as this is above
          // minimum speed still
          midPose.setVel(MIN_SPEED * signofd(midPose.getVel()));
        }
        else if (okInitialVel2 < MIN_SPEED)
        { // bad end result restart with a lower initial speed.
          if (okInitialVel >= fromPose.getVel())
            okInitialVel = fromPose.getVel();
          okInitialVel *= 0.67;
        }
        else
        { // just keep result
          break;
        }*/
      }
    }
    // debug
/*    printf("--- (loop %d) Trying 1st part end at %.2f,%.2f, %.3fh, %.2fm/s\n",
        i, midPose.x, midPose.y, midPose.h, midPose.getVel());*/
    // debug end
    i++;
    if (i > 8)
    {  // this is to prevent endless loop, but should not happen
      printf("*** UManSeq::replaceMan: Endless loop trap, or just too difficult\n");
      result = false;
      break;
    }
  }
  if (okVel != NULL)
    *okVel = okInitialVel;
  return result;
}

//////////////////////////////////////////////

bool UManSeq::expandMan(UPoseV fromPose, UPoseV toPose, bool last,
                        double maxAcc, double maxTurnAcc, double minTurnRad,
                        UManPPSeq * ppSeq, double * okVel)
{
  bool result;
  double v, t, d;
  UPoseV calcEndPose;
  //
  if (okVel != NULL)
    *okVel = 1000.0;
  releaseAll(ppSeq);
  ppSeq->setStartPoseV(fromPose);
  ppSeq->setEndPoseV(toPose);
  //result = expandAddManALA(ppSeq, last, maxAcc, maxTurnAcc, okVel);
  result = expandAddManLALA(ppSeq, last, maxAcc, maxTurnAcc, minTurnRad);
  if (result)
  { // calculate final velocity and time used
    v = ppSeq->calcEndVel(&t, &d);
    ppSeq->setEndVel(v);
    ppSeq->setManTime(t);
    ppSeq->setManDist(d);
    // debug
/*    calcEndPose = ppSeq->calcEndPose(ppSeq->getStartPoseV());
    d = calcEndPose.getDistance(toPose);
    if (d > 0.05)
    {
      printf("****UManSeq::expandMan: NB! discrepancy %g from predicted to expected pose!\n", d);
      fromPose.print(   "\n    From");
      toPose.print(     "\nExpected");
      calcEndPose.print("\nPrediced");
      printf("\n");
      ppSeq->fprint(stdout, ":--");
    }*/
    // debug end
  }
  else
  { // debug
    //printf("UManSeq::expandMan:   Truncating solution!)\n");
  } // debug end
  //
  return result;
}

///////////////////////////////////////////////

// bool UManSeq::expandAddMan(UPoseV fromPose, UPoseV toPose,
//                      double maxAcc, double maxTurnAcc,
//                      UManPPSeq * ppSeq)
// {
//   UPose2pose relEnd;
//   UPose2pose::MAN_SOL_TYP manType;
//   double dist, rad, arc1, arc2;
//   bool left;
//   bool result = true;
//   bool finished = false;
//   UManoeuvre * ma;
//   double minRad = 0.0;
//   //double velM = maxd(fromPose->getVel(), toPose->getVel());
//   double velOK;
//   double tToOKvel;
//   double sToOKvel;
//   bool accOK = true; // is acceleration
//   double emergencyAcc = 2.0; // m/s�
//   double acc;
//   double vel2; // target velocity
//   double vel1; // start velocity
//   double d;
//   const double MIN_SPEED = 0.19;
//   //
//   relEnd = fromPose.getMapToPosePose(&toPose);
//   vel1 = fromPose.getVel();
//   if (fabs(vel1) < MIN_SPEED)
//     vel1 = MIN_SPEED;
//   vel2 = toPose.getVel();
//   //
//   if (maxTurnAcc > 1e-3)
//     minRad = sqr(fromPose.getVel()) / maxTurnAcc;
//   // find manoeuvre solution
//   result = relEnd.get2here((int *)&manType, &dist, &rad, &arc1, &arc2, &left);
//   if (result)
//   { // MT_2ARC, MT_LINE_ARC, MT_ARC_LINE, MT_LINE, MT_NONE
//     velOK = sqrt(absd(rad * maxTurnAcc));
//     if (((manType == UPose2pose::MST_2ARC) or
//           (manType == UPose2pose::MST_ARC_LINE) or
//           (manType == UPose2pose::MST_LINE_ARC)) and
//           (rad < minRad))
//     { // this is not so good, so try a breaking action
//       // tome to get down to an OK velocity
//       tToOKvel = (vel1 - velOK)/maxAcc;
//       if (tToOKvel < 0.0)
//       { // this is not ment to reverse  - stop right away
//         printf("UManSeq::expandAddMan: Got into a reverse situation? -- stopping\n");
//         ma = getNewStop();
//         result = ppSeq->add(ma);
//         finished = true;
//       }
//       else
//       { // distance to an OK velocity
//         sToOKvel = getNeededDist(vel1, velOK, -maxAcc) + 0.01;
//         // is start velocity OK
//         accOK = false;
//         if ((not accOK) and
//             ((manType == UPose2pose::MST_2ARC) or
//               (manType == UPose2pose::MST_ARC_LINE)))
//         { // breaking action needed and first manoeuvre is a turn
//           if (sToOKvel > (arc1 * rad / 2.0))
//           { // not OK to turn and break hard, so emergency brake straight
//             tToOKvel = (vel1 - velOK)/emergencyAcc;
//             // distance to an OK velocity (plus a bit to be on the safe side)
//             sToOKvel = getNeededDist(vel1, velOK, -emergencyAcc) + 0.05;
//             // get a break manoeuvre
//             ma = getNewLine(sToOKvel, -emergencyAcc, velOK);
//           }
//           else
//           { // normal break - (while turning at max allowd turnradius
//             ma = getNewArc(minRad, sToOKvel/minRad, left, -maxAcc, velOK);
//           }
//           // add break man
//           result = ppSeq->add(ma);
//           if (result)
//             // try to take it from there
//             result = expandAddMan(ma->getEndPoseV(fromPose), toPose,
//                               maxAcc, maxTurnAcc, ppSeq);
//           // finished
//           finished = true;
//         }
//         else if (sToOKvel > dist)
//         { // a line first, so break hard first (almost the remaining distance + 10 cm)
//           ma = getNewLine(dist * 0.8,
//                      (velOK - vel1)/(2.0 * dist * 0.8),
//                      velOK);
//           result = ppSeq->add(ma);
//           if (result)
//             // try a normal man after this break.
//             result = expandAddMan(ma->getEndPoseV(fromPose), toPose,
//                               maxAcc, maxTurnAcc, ppSeq);
//           finished = true;
//         }
//       }
//     }
//     if (result and not finished)
//     {
//       switch (manType)
//       {
//         case UPose2pose::MST_2ARC: // a 2-acr solution
//           // calculate the required acceleration to get to
//           // end
//           vel2 = toPose.getVel();
//           if ((sqr(vel2)/rad) > maxTurnAcc)
//             // too fast for last turn -- acc less
//             acc = getNeededAcc(vel1, sqrt(maxTurnAcc * rad), (arc1 + arc2) * rad);
//           else
//           { // may accelerate as fast as possible
//             if (vel2 > vel1)
//               acc = maxAcc;
//             else
//               acc = -maxAcc;
//             d = getNeededDist(vel1, vel2, acc);
//             if (d < (arc1 * rad))
//             { // add an initial acceleration part
//               ma = getNewArc(rad, d/rad, left, acc, vel2);
//               result = ppSeq->add(ma);
//               // reduse rest of first manoeuvre arc
//               arc1 -= d/rad;
//               acc = 0.0;
//             }
//             else
//               // accelerate over full distance
//               acc = getNeededAcc(vel1, vel2, (arc1 + arc2) * rad);
//           }
//           if (result)
//           { // add first arc (with normal acceleration)
//             ma = getNewArc(rad, arc1, left, acc, vel2);
//             result = ppSeq->add(ma);
//           }
//           if (result)
//           { // add second arc
//             ma = getNewArc(rad, arc2, not left, acc, vel2);
//             result = ppSeq->add(ma);
//           }
//           break;
//         case UPose2pose::MST_ARC_LINE:
//           vel2 = toPose.getVel();
//           if ((sqr(vel2)/rad) > maxTurnAcc)
//             // too fast for first turn -- acc less
//             acc = getNeededAcc(vel1, sqrt(maxTurnAcc * rad), (arc1) * rad);
//           else
//             acc = getNeededAcc(vel1, vel2, arc1 * rad);
//           ma = getNewArc(rad, arc1, left, acc, sqrt(maxTurnAcc * rad));
//           result = ppSeq->add(ma);
//           if (result)
//           { // now the straight part
//             vel1 = ma->getEndV(fromPose);
//             acc = getNeededAcc(vel1, vel2, dist);
//             if ((acc > 0.0) and (acc > maxAcc))
//               acc = maxAcc;
//             else if ((acc > 0.0) and (acc < maxAcc/2.0))
//             { // accelerate faster at start of straight part
//               d = getNeededDist(vel1, vel2, maxAcc);
//               ma = getNewLine(d, maxAcc, vel2);
//               result = ppSeq->add(ma);
//               dist -= d;
//               acc = 0.0;
//             }
//           }
//           if (result)
//           { // the (remaining) straight line part
//             ma = getNewLine(dist, acc, vel2);
//             result = ppSeq->add(ma);
//           }
//           break;
//         case UPose2pose::MST_LINE_ARC:
//           vel2 = toPose.getVel();
//           if ((sqr(vel2)/rad) > maxTurnAcc)
//             // too fast for end turn -- reduce speed at end of straight line
//             vel2 = sqrt(maxTurnAcc * rad);
//           // get acceleration to get to this velocity before turn
//           acc = getNeededAcc(vel1, vel2, dist);
//           if ((absd(acc) > maxAcc) or (acc > 0.0))
//              // too much lateral acceleration
//                acc = maxAcc * signofd(acc);
//           // insert the straight part direct
//           ma = getNewLine(dist, acc, vel2);
//           result = ppSeq->add(ma);
//           if (result)
//           { // add arc part
//             // get max speed in arc
//             vel1 = sqrt(maxTurnAcc * rad);
//             if (vel1 > absd(vel2))
//               vel1 = vel2;
//             else
//               vel1 *= signofd(vel2);
//             ma = getNewArc(rad, arc1, left, maxAcc, vel1);
//             result = ppSeq->add(ma);
//           }
//           break;
//         case UPose2pose::MST_LINE:
//           // just a straight line will do
//           vel2 = toPose.getVel();
//           if ((dist < 0.0) and (vel2 > 0.0) and (vel1 > 0.0))
//             // just get there at speed 0.0;
//             vel2 = 0.0;
//           d = getNeededDist(vel1, vel2, maxAcc);
//           if ((vel1 > 0.0) and (dist < 0.0))
//           { // goind forward, but should go back
//             // stop as fast as possible
//             d = getNeededDist(vel1, 0.0, -maxAcc);
//             ma = getNewLine(d, -maxAcc, 0.0);
//             result = ppSeq->add(ma);
//             dist -= d; // need to go further back
//             vel1 = 0.0;
//           }
//           if (result and (dist < 0.0))
//           { // accelerate back and stop
//             vel2 = toPose.getVel();
//             d = getNeededDist(0.0, vel2, maxAcc);
//             if (d > absd(dist)/2.0)
//               // add two segments: acc and breake
//               d = absd(dist)/2.0;
//             ma = getNewLine(-dist - d, maxAcc, -vel2);
//             result = ppSeq->add(ma);
//             // stop after reverse distance
//             ma = getNewLine(d, maxAcc, 0.0);
//             ppSeq->add(ma);
//             // make a full stop here.
//             ma = getNewStop();
//             result = ppSeq->add(ma);
//           }
//           if (result and (dist > 0.0))
//           { // go forward at max acc.
//             d = absd(d);
//             if ((vel2 < vel1) and (d < dist))
//             { // breake as late as possible
//               // add a traight part maintaining vel
//               ma = getNewLine(dist - d, maxAcc, vel1);
//               result = ppSeq->add(ma);
//               dist = d;
//             }
//             ma = getNewLine(dist, maxAcc, vel2);
//             result = ppSeq->add(ma);
//             dist = 0.0;
//           }
//           break;
//         default: // unknown or stop
//           ma = getNewStop();
//           result = ppSeq->add(ma);
//           break;
//       }
//     }
//   }
//   return result;
// }

//////////////////////////////////////////////////////

bool UManSeq::expandAddManALA(UManPPSeq * ppSeq, bool last,
                           double maxAcc, double maxTurnAcc,
                           double * okVel)
{
  UPose2pose relEnd;
  int manType;
  double dist, rad1, rad2, arc1, arc2;
  double arc1a, arc1b;
  bool left;
  bool result = true;
  UManoeuvre * ma;
  // double minRad = 0.0;
  double velOK;
  double acc;
  double vel2; // target velocity
  double vel1, vel1m; // start velocity
  double d;
  const double MIN_SPEED = 0.2;
  double MAX_BREAKE_ACC = 3.5 * maxAcc;
  UPoseV fromPose, toPose;
  double a1, a2;
  const double OK_ANGLE_FOR_STRATGHT_START_MAN = 10.0 * M_PI / 180.0;    // allowed deviation from start angle
  const double OK_ANGLE_FOR_STRATGHT_END_MAN = 15.0 * M_PI / 180.0;      // allowed deviation when not last manoeuvre
  const double OK_ANGLE_FOR_STRATGHT_END_MAN_LAST = 30.0 * M_PI / 180.0; // allowed deviation when last manoeuvre
  bool useStraightLine = false;
  double angLim1 = OK_ANGLE_FOR_STRATGHT_START_MAN;
  double angLim2 = OK_ANGLE_FOR_STRATGHT_END_MAN;
  //
  fromPose = ppSeq->getStartPoseV();
  toPose = ppSeq->getEndPoseV();
  relEnd = fromPose.getMapToPosePose(&toPose);
  // test if a straight route could solve the problem
  a1 = atan2(relEnd.y, relEnd.x);
  if (relEnd.x < 0.3)
    angLim1 *= 2.0;
  if (fabs(a1) < angLim1)
  { // may be OK for straight drive, if end heading is not too much off.
    a2 = limitToPi(toPose.h - (fromPose.h + a1));
    if (last or (relEnd.x < 0.3))
      angLim2 = OK_ANGLE_FOR_STRATGHT_END_MAN_LAST;
    useStraightLine = (fabs(a2) < angLim2);
  }
  //
  velOK = 1e3;
  if (not useStraightLine)
  { // use arc-line-arc
    if (fabs(fromPose.getVel()) < toPose.getVel())
      // try using end-speed in turn radius if enough space
      vel1 = toPose.getVel();
    else
      vel1 = fromPose.getVel();
    // desired end velocity
    vel2 = toPose.getVel();
    if (fabs(vel2) < 0.01)
      vel2 = signofd(vel1) * MIN_SPEED;
    relEnd.setVel(vel2);
    //
//     if (maxTurnAcc > 1e-3)
//       minRad = sqr(fromPose.getVel()) / maxTurnAcc;
    // find manoeuvre solution using 2 arcs and a straight part.
    result = relEnd.get2hereALA( &manType, vel1, maxAcc, maxTurnAcc,
                                &rad1, &arc1, &dist, &rad2, &arc2, NULL);
    // starting left?
    left = (manType <= 1); // left-left, left-right, right-left, right-right
  }
  if (useStraightLine or not result)
  { // use straight line (also if arc-line-arc failed)
    left = true;
    manType = 0;
    rad1 = 10.0;
    rad2 = 10.0;
    arc1 = 0.0;
    arc2 = 0.0;
    dist = hypot(relEnd.x, relEnd.y);
    // adjust heading to actual obtained values
    fromPose.h += a1;
    toPose.h = fromPose.h;
    ppSeq->setStartPoseV(fromPose);
    ppSeq->setEndPoseV(toPose);
  }
  //
  // debug
/*  printf("ALA a %d to (%.2f,%.2f,%.1fh) is %.2fr %.2fa %.2fd %.2fr %.2fa\n",
         manType, relEnd.x, relEnd.y, relEnd.h * 180.0/M_PI,
         rad1, arc1, dist, rad2, arc2);*/
  // debug ned
  //
  if (result and not useStraightLine)
  { // calculate the required acceleration to get to
    // end
    vel1 = maxd(fromPose.getVel(), mind(toPose.getVel(), sqrt(rad1 * maxTurnAcc)));
    vel2 = mind(toPose.getVel(), sqrt(rad2 * maxTurnAcc));
    // set initial speed to more than zero to get mooving
    if (fabs(vel1) < MIN_SPEED)
      vel1m = MIN_SPEED;
    else
      vel1m = vel1;
    // may accelerate as fast as possible
    if (vel2 >= vel1)
      acc = maxAcc;
    else
      acc = -maxAcc;
    d = getNeededDist(vel1, vel2, acc);
    if (arc1 > 0.002)
    { // first turn at initial speed
      if ((d > dist) and (acc < 0.0))
      { // need to break much - use arc too
        arc1b = mind(arc1, (d - dist)/rad1); // breaking part
        arc1a = arc1 - arc1b; // non breaking part
        if (arc1a > 0.002)
        { // add a non breaking part
          ma = getNewArc(rad1, arc1a, left, maxAcc, vel1m);
          result = ppSeq->add(ma);
        }
        // breaking part - towards speed vel2
        ma = getNewArc(rad1, arc1b, left, maxAcc, vel2);
        result = ppSeq->add(ma);
        // reduce remaining break distance
        d -= arc1b * rad1;
      }
      else
      { // do not break in arc1
        ma = getNewArc(rad1, arc1, left, maxAcc, vel1m);
        result = ppSeq->add(ma);
      }
    }
    // now straight part
    if ((d < dist) and (acc < 0))
    { // first straight part at initial velocity
      // i.e. break as late as possible
      ma = getNewLine(dist - d, maxAcc, vel1m);
      result = ppSeq->add(ma);
      dist = d;
    }
    if (dist > 0.01)
    { // now the (remaining) straight part ending at exit speed
      if ((d > dist) and (acc < 0.0))
      { // more break than maxAcc is needed, OK to double
        // without retry route with lower initial speed
        acc = maxAcc * d / dist;
        // limit to MAX_BREAKE_ACC
        if (acc > MAX_BREAKE_ACC)
        { // suggest a lower initial velocity
          velOK = vel1 * 0.8;
          acc = MAX_BREAKE_ACC;
        }
      }
      else
        acc = maxAcc;
      ma = getNewLine(dist, acc, vel2);
      result = ppSeq->add(ma);
    }
    // then the final arc
    if (arc2 > 0.001)
    {
      left = (manType == 0) or (manType == 2); // 0:left-left, 1:left-right, 2:right-left, 3:right-right
      ma = getNewArc(rad2, arc2, left, maxAcc, vel2);
      result = ppSeq->add(ma);
    }
    if (fabs(toPose.getVel()) < 0.01)
    { // add a stop man
      ma = getNewStop();
      result = ppSeq->add(ma);
    }
    // suggest nitial speed reduction if
    // too long turns, either almost full circle or
    // almost half circle and big radius - all sign of
    // a major detour, remidi is the same - lower speed.
    if ((arc1 > 1.3 * M_PI) or (arc2 > 1.3 * M_PI) or
         ((arc1 * rad1 > 5.0) and (arc1 > 0.75 * M_PI)) or
         ((arc2 * rad2 > 5.0) and (arc2 > 0.75 * M_PI)))
    { // suggest lower initial speed to requester
      // in an attempt to get rid of loops
      velOK = maxd(fromPose.getVel() * 0.67, MIN_SPEED);
      // debug
/*      printf("+++ too large r1 %.2fm, arc1 %.3frad -- r2 %.2fm, arc2 %.3frad\n",
             rad1, arc1, rad2, arc2);*/
      // debug end
    }
    // debug
    if (not result)
      printf("UManSeq::expandAddManALA: out of available UManoeuvre (%d of %d) objects?\n", ppSeq->getSeqCnt(), ppSeq->getMaxManCnt());
    // debug end
  }
  if (velOK < *okVel)
    *okVel = velOK;
  return result;
}


//////////////////////////////////////////////////////

bool UManSeq::expandAddManLALA(UManPPSeq * ppSeq, bool last,
                              double maxAcc, double maxTurnAcc, double minTurnRad) //
//                              double * okVel)
{
  UPose2pose relEnd;
  int manType;
  double dist, rad1, rad2, arc1, arc2;
  bool left;
  bool result = true;
  UManoeuvre * ma;
//  double minRad = 0.0;
//  double velOK;
  double vel2; // target velocity
  double vel1; // start velocity
//  const double MIN_SPEED = 0.2;
  UPoseV fromPose, toPose;
  double a1, a2;
  const double OK_ANGLE_FOR_STRATGHT_START_MAN = 5.0 * M_PI / 180.0;    // allowed deviation from start angle
  const double OK_ANGLE_FOR_STRATGHT_END_MAN = 15.0 * M_PI / 180.0;      // allowed deviation when not last manoeuvre
  const double OK_ANGLE_FOR_STRATGHT_END_MAN_LAST = 30.0 * M_PI / 180.0; // allowed deviation when last manoeuvre
  bool useStraightLine = false;
  double angLim1 = OK_ANGLE_FOR_STRATGHT_START_MAN;
  double angLim2 = OK_ANGLE_FOR_STRATGHT_END_MAN;
  double aDist, v, bDist;
  //
  fromPose = ppSeq->getStartPoseV();
  toPose = ppSeq->getEndPoseV();
  relEnd = fromPose.getMapToPosePose(&toPose);
  // test if a straight route could solve the problem
  a1 = atan2(relEnd.y, relEnd.x);
  if (relEnd.x < minTurnRad + 0.3)
    angLim1 *= 2.0;
  if (fabs(relEnd.y) < 0.04)
  { // may be OK for straight drive, if end heading is not too much off.
    a2 = relEnd.h;
    if (last or (relEnd.x < minTurnRad + 0.3))
      angLim2 = OK_ANGLE_FOR_STRATGHT_END_MAN_LAST;
    useStraightLine = (fabs(a2) < angLim2);
    // debug - fails
    // useStraightLine = false;
    // debug end
  }
  else if (fabs(a1) < angLim1)
  { //small corrections needed only, allow small turn radius (knowing, that it can
    // not be implemented, but the line drive method will get to the final
    // heading as fast as possible - skipping the turn altogeather
    minTurnRad *= 0.3;
  }
  //
  //  velOK = 1e3;
  if (not useStraightLine)
  { // use arc-line-arc
    vel1 = fromPose.getVel();
    // desired end velocity
    vel2 = toPose.getVel();
    //if (fabs(vel2) < 0.01)
    //  vel2 = signofd(vel1) * MIN_SPEED;
    relEnd.setVel(vel2);
    //
/*    if (maxTurnAcc > 1e-3)
      minRad = sqr(fromPose.getVel()) / maxTurnAcc;*/
    // find manoeuvre solution using 2 arcs and a straight part.
    result = relEnd.get2hereLALA( &manType, vel1, maxAcc, maxTurnAcc, minTurnRad,
                                  &aDist, &v, &rad1, &arc1, &dist, &rad2, &arc2, &bDist);
  }
  if (useStraightLine or not result)
  { // use straight line (also if arc-line-arc failed)
    left = true;
    manType = 0;
    rad1 = 10.0;
    rad2 = 10.0;
    arc1 = 0.0;
    arc2 = 0.0;
    if (relEnd.x < 0.0)
      dist = 0.0;
    else
      dist = hypot(relEnd.x, relEnd.y);
    // adjust heading to actual obtained values
/*    fromPose.h += a1;
    toPose.h = fromPose.h;
    ppSeq->setStartPoseV(fromPose);
    ppSeq->setEndPoseV(toPose);*/
    ma = getNewLine(dist, maxAcc, toPose.getVel());
    result = ppSeq->add(ma);
  }
  //
  // debug
/*  printf("ALA a %d to (%.2f,%.2f,%.1fh) is %.2fr %.2fa %.2fd %.2fr %.2fa\n",
  manType, relEnd.x, relEnd.y, relEnd.h * 180.0/M_PI,
  rad1, arc1, dist, rad2, arc2);*/
  // debug ned
  //
  if (result and not useStraightLine)
  { // make man segments
    if (aDist > 0.001)
    { // add a break or acc line
      ma = getNewLine(aDist, maxAcc, v);
      result = ppSeq->add(ma);
    }
    if (arc1 > 0.001)
    { // first turn at initial speed
      // starting left?
      left = (manType <= 1); // left-left, left-right, right-left, right-right
      ma = getNewArc(rad1, arc1, left, maxAcc, v);
      result = ppSeq->add(ma);
    }
    if (dist > 0.001)
    { // first turn at initial speed
      ma = getNewLine(dist, maxAcc, v);
      result = ppSeq->add(ma);
    }
    if (arc2 > 0.001)
    { // first turn at initial speed
      left = ((manType % 2) == 0); // left-left, left-right, right-left, right-right
      ma = getNewArc(rad2, arc2, left, maxAcc, v);
      result = ppSeq->add(ma);
    }
    if (bDist > 0.001)
    { // final break
      ma = getNewLine(bDist, maxAcc, relEnd.getVel());
      result = ppSeq->add(ma);
    }
    // debug
    if (not result)
      printf("UManSeq::expandAddManALA: out of available UManoeuvre (%d of %d) objects?\n",
             ppSeq->getSeqCnt(), ppSeq->getMaxManCnt());
    // debug end
  }
/*  if (velOK < *okVel)
    *okVel = velOK;*/
  return result;
}

//////////////////////////////////////////////

UManoeuvre * UManSeq::getNewLine(double dist, double acceleration, double targetVel)
{
  UManLine * result;
  //
  result = (UManLine*)requestObj(UManoeuvre::MAN_LINE);
  if (result != NULL)
  {
    result->setAcc(acceleration);
    result->setDistance(dist);
    result->setVel(targetVel);
  }
  return result;
}

//////////////////////////////////////////////

UManoeuvre * UManSeq::getNewArc(double radius, double angle,
                       bool left,
                       double acceleration, double targetVel)
{
  UManArc * result;
  //
  result = (UManArc*)requestObj(UManoeuvre::MAN_ARC);
  if (result != NULL)
  {
    result->setAcc(acceleration);
    result->setTurnRadius(radius);
    if (left)
      result->setTurnAngle(angle);
    else
      result->setTurnAngle(-angle);
    result->setVel(targetVel);
  }
  return result;
}

//////////////////////////////////////////////

UManoeuvre * UManSeq::getNewStop()
{
  UManStop * result;
  result = (UManStop*)requestObj(UManoeuvre::MAN_STOP);
  result->setAcc(0.0);
  result->setVel(0.0);
  return result;
}

//////////////////////////////////////////////

void UManSeq::fprint(FILE * fd, const char * prestr)
{
  int i;
  //
  fprintf(fd, "%s manCnt = %d, reuseCnt=%d, avail=%d\n",
         prestr, p2pCnt, reuseCnt, reusableCnt());
  for (i = 0; i < p2pCnt; i++)
  { // print all subman's
    p2p[i]->fprint(fd, " - ");
  }
}

//////////////////////////////////////////////////

const char * UManSeq::print(const char * prestr, char * buff, const int buffCnt)
{
  int i;
  char * p1 = buff;
  int n = 0;
  //
  snprintf(buff, buffCnt, "%s manCnt = %d, reuseCnt=%d, avail=%d\n",
          prestr, p2pCnt, reuseCnt, reusableCnt());
  for (i = 0; i < p2pCnt; i++)
  { // print all subman's
    n += strlen(p1);
    p1 = &buff[n];
    p2p[i]->print(" - ", p1, buffCnt -n);
  }
  return buff;
}

//////////////////////////////////////////////////

int UManSeq::reusableCnt()
{
  int i;
  int result = 0;
  //
  for (i = 0; i < reuseCnt; i++)
  {
    if (reuse[i] != NULL)
      result++;
  }
  return result;
}

////////////////////////////////////////////////////

UPoseV UManSeq::getEndPoseV()
{
  UPoseV pv;
  if (p2pCnt > 0)
  {
    pv = p2p[p2pCnt-1]->getEndPoseV();
    pv.setVel(p2p[p2pCnt-1]->getEndVel());
  }
  else
    pv.clear();
  return pv;
}

/////////////////////////////////////////////////

UPoseV UManSeq::getStartPoseV()
{
  UPoseV result;
  if (p2pCnt > 0)
    result = p2p[0]->getStartPoseV();
  return result;
}

/////////////////////////////////////////////////

bool UManSeq::addMan(UPoseV fromPose, UPoseV toPose,
                     double maxAcc, double maxTurnAcc, double minTurnRad,
                double * okVel)
{
  bool result = p2pCnt < MAX_PP_MAN_CNT;
  UManPPSeq * ppseq = NULL;
  //
  if (result)
  {
    if (p2p[p2pCnt] == NULL)
      ppseq = new UManPPSeq();
    else
      ppseq = p2p[p2pCnt];
    result = ppseq != NULL;
  }
  if (result)
  {
    p2p[p2pCnt++] = ppseq;
    result = expandMan(fromPose, toPose, true,
                       maxAcc, maxTurnAcc, minTurnRad,
                       ppseq, okVel);
  }
  return result;
}

////////////////////////////////////////////////////

UManPPSeq * UManSeq::getP2P(int idx)
{
  UManPPSeq * seqpp = NULL;
  //
  if ((idx >= 0) and (idx < p2pCnt))
    seqpp = p2p[idx];
  //
  return seqpp;
}

////////////////////////////////////////////////////

UManPPSeq * UManSeq::addP2P()
{
  UManPPSeq * seqpp = NULL;
  //
  if (p2pCnt < MAX_PP_MAN_CNT)
  {
    seqpp = p2p[p2pCnt];
    if (seqpp == NULL)
    {
      seqpp = new UManPPSeq();
      p2p[p2pCnt] = seqpp;
    }
    p2pCnt++;
  }
  //
  return seqpp;
}

///////////////////////////////////////////////////

double UManSeq::getDistanceXYSigned(UPosition pos, int * where,
                             bool posIsRight,
                             double maxDist,
                             UPose * pHit,
                             int * idx, double * t)
{
  UPosition p1, p2;
  UPoseV pv1, pv2;
  UManPPSeq * mpp;
  int i, w = 3, atIdx = -1, wm = 1;
  double d1, d2, d3, dms = 0.0, dm = maxDist, tm = 0.0;
  double tsum = 0.0; // time into manoeuvre
  double outDist = 2.0 * maxDist;
  double atT; // distance into manoeuvre seq.
  UPose hitAt, hm;
  //
  for (i = 0; i < p2pCnt; i++)
  {
    mpp = p2p[i];
    pv1 = mpp->getStartPoseV();
    pv2 = mpp->getEndPoseV();
    // use block distance for rough estimate
    d1 = fabs(pos.x - pv1.x) + fabs(pos.y - pv1.y);
    d2 = fabs(pos.x - pv2.x) + fabs(pos.y - pv2.y);
    d3 = fabs(pv1.x - pv2.x) + fabs(pv1.y - pv2.y);
    outDist += d3;
    if ((outDist > d1) or (outDist > d2))
    { // point may be near enough to be usable - get true distance
      d1 = mpp->getDistanceXYSigned(pos, &w, posIsRight, &hitAt, &atT);
      if ((fabs(d1) < dm) or (i == 0))
      { // d1 is distance to midpoint of route or close end
        // no chance for better matche (unless in a loop)
        atIdx = i;
        dms = d1;
        //if (w != 0)
        // close to an end - ignore sign
        dm = fabs(d1);
        //else
        //  dm = d1;
        wm = w;
        // distance into manoeuvre
        tm = tsum + atT;
        hm = hitAt;
      }
      // accumulate distance
      tsum += mpp->getManDist();
    }
  }
  if (dm >= maxDist)
    // too far away to be relevant.
    wm = 3;
  if (where != NULL)
    *where = wm;
  if (idx != NULL)
    *idx = atIdx;
  if (t != NULL)
    *t = tm;
  if (pHit != NULL)
    *pHit = hm;
  return dms;
}

///////////////////////////////////////////////////
/*getMinDistanceXYSigned(ULineSegment * seg, int * whereOnSeg,
                       UPosition * posOnSeg,
                       bool posIsRight,
                       int * whereOnMan,
                       UPose * poseOnMan)*/
double UManSeq::getMinDistanceXYSigned(ULineSegment * seg,
                                    int * whereOnSeg,
                                    UPosition * posOnSeg,
                                    bool posIsRight,
                                    double maxDist,
                                    int * whereOnMan,
                                    UPose * poseOnMan,
                                    int * idx)
{
  UPosition p1, p2, minPs, pS;
  UPoseV pv1, pv2;
  UManPPSeq * mpp;
  int i, minWm = 3, minWs = 1, atIdx = -1, wS, wM;
  double d1, d2, d3, dms = 1e27, dm = maxDist;
  double outDist;
  UPose hitAt, minPm, pM;
  //
  for (i = 0; i < p2pCnt; i++)
  {
    mpp = p2p[i];
    pv1 = mpp->getStartPoseV();
    pv2 = mpp->getEndPoseV();
    d3 = mpp->getManDist();
    // distance fromeither  end to segment worst case
    d1 = hypot(seg->pos.x - pv1.x, seg->pos.y - pv1.y) - seg->length;
    d2 = hypot(seg->pos.x - pv2.x, seg->pos.y - pv2.y) - seg->length;
    // route length + security distance (maxDist)
    outDist = maxDist + d3;
    // if within reach get minimum distance
    if ((outDist > d1) or (outDist > d2))
    { // point may be near enough to be usable - get true distance
      d1 = mpp->getMinDistanceXYSigned(seg, &wS, &pS, posIsRight, &wM, &pM);
      if ((fabs(d1) < dm) or (i == 0))
      { // d1 is distance to midpoint of route or close end
        // no chance for better matche (unless in a loop)
        atIdx = i;
        dms = d1;
        dm = fabs(d1);
        minWm = wM;
        minPm = pM;
        minWs = wS;
        minPs = pS;
      }
    }
  }
  if (dm >= maxDist)
    // too far away to be relevant.
    minWm = 3;
  if (whereOnSeg != NULL)
    *whereOnSeg = minWs;
  if (whereOnMan != NULL)
    *whereOnMan = minWm;
  if (posOnSeg != NULL)
    *posOnSeg = minPs;
  if (poseOnMan != NULL)
    *poseOnMan = minPm;
  if (idx != NULL)
    *idx = atIdx;
  return dms;
}

//////////////////////////////////////////////////

double UManSeq::getDistance()
{
  double dist = 0.0;
  int i;
  UManPPSeq * mpp;
  //
  for (i = 0; i < p2pCnt; i++)
  {
    mpp = p2p[i];
    dist += mpp->getManDist();
  }
  return dist;
}

//////////////////////////////////////////////////

double UManSeq::getTime()
{
  double dt = 0.0;
  int i;
  UManPPSeq * mpp;
  //
  for (i = 0; i < p2pCnt; i++)
  {
    mpp = p2p[i];
    dt += mpp->getManTime();
  }
  return dt;
}

//////////////////////////////////////////////////

double UManSeq::getDeviationFromDirect()
{
  double var = 0.0;
  int i;
  UManPPSeq * mpp;
  ULineSegment seg;
  UPosition p1;
  //
  if (p2pCnt > 1)
  { // make line segment from robot to exit point
    seg.setFromPoints(p2p[0]->getStartPos(),
                      p2p[p2pCnt - 1]->getEndPos());
    for (i = 0; i < p2pCnt - 1; i++)
    {
      mpp = p2p[i];
      var += seg.getDistanceSq(mpp->getEndPos());
    }
    var /= (p2pCnt - 1);
  }
  return sqrt(var);
}

//////////////////////////////////////////////

bool UManSeq::copy(UManSeq * source)
{
  UManPPSeq * pp1, * pp2;
  int i, j;
  bool result = source != NULL;
  UManoeuvre * man1, *man2;
  //
  if (result)
  { // remove current content
    releaseAllMan();
    for (i = 0; i < source->getP2PCnt(); i++)
    {
      pp1 = source->getP2P(i);
      pp2 = p2p[i];
      if (pp2 == NULL)
      {
        pp2 = new UManPPSeq();
        p2p[i] = pp2;
      }
      //
      pp2->setStartPoseV(pp1->getStartPoseV());
      pp2->setEndPoseV(pp1->getEndPoseV());
      pp2->setManTime(pp1->getManTime());
      pp2->setManDist(pp1->getManDist());
      pp2->setEndVel(pp1->getEndVel());
      //
      for (j = 0; j < pp1->getSeqCnt(); j++)
      {
        man1 = pp1->getMan(j);
        switch (man1->getManType())
        { //   typedef enum MAN_TYPE  {MAN_NONE, MAN_LINE, MAN_ARC, MAN_STOP};
          case UManoeuvre::MAN_LINE:
            man2 = getNewLine(man1->getDistance(), man1->getAcc(), man1->getVel());
            break;
          case UManoeuvre::MAN_ARC:
            man2 = getNewArc(((UManArc*)man1)->getTurnRadius(),
                               ((UManArc*)man1)->getTurnAngle(),
                               man1->getAcc(), man1->getVel());
            break;
          case UManoeuvre::MAN_STOP:
            man2 = getNewStop();
            break;
          default:
            man2 = NULL;
            break;
        }
        if (man2 != NULL)
          pp2->add(man2);
      }
    }
    p2pCnt = source->getP2PCnt();
  }
  return result;
}

////////////////////////////////////////////////////

double UManSeq::getMaxTurnArc()
{
  UManPPSeq * pp;
  int i;
  double maxArc = 0.0;
  double a;
  //
  for (i = 0; i < p2pCnt; i++)
  {
    pp = p2p[i];
    a = pp->getMaxTurnArc();
    if (a > maxArc)
      maxArc = a;
  }
  return maxArc;
}

///////////////////////////////////////////////////////

bool UManSeq::isValid()
{
  int i;
  bool result = false;
  //
  for (i = 0; i < p2pCnt; i++)
  {
    result = p2p[i]->isValid();
    if (not result)
      break;
  }
  return result;
}

//////////////////////////////////////////////

UPoseV UManSeq::getPoseV(double atManTime)
{
  UPoseV pv;
  int i;
  UManPPSeq ** mpp;
  double sec, ssec = 0.0;
  bool isOK = false;
  //
  mpp = p2p;
  for (i = 0; i < p2pCnt; i++)
  {
    sec = (*mpp)->getManTime();
    if (sec + ssec > atManTime)
    { // end time is within this manoeuvre
      pv = (*mpp)->getPoseV(atManTime - ssec);
      isOK = true;
      break;
    }
    // summ used time
    ssec += sec;
    mpp++;
  }
  if (not isOK)
    pv = getEndPoseV();
  return pv;
}

//////////////////////////////////////////////

void UManSeq::removeLastArc()
{
  int i;
  UManoeuvre * mn;
  UManPPSeq * ppsq;
  //
  if (p2pCnt > 0)
  {
    ppsq = p2p[p2pCnt-1];
    for (i = ppsq->getSeqCnt()-1; i > 1; i--)
    {
      mn = ppsq->getMan(i);
      if (mn->getManType() == UManoeuvre::MAN_ARC)
      { // remove arc
        release(mn);
        ppsq->truncate(i);
        break;
      }
      release(mn);
    }
  }
}

//////////////////////////////////////////////////

double UManSeq::getDistanceFromEndPoseLine(double * maxHeading)
{
  double d, dist = 0.0, head = 0.0, h;
  int i;
  UManPPSeq * mpp;
  UPoseV sp, ep;
  U2Dlined line;
  //
  ep = getEndPoseV();
  line.setPH(ep.x, ep.y, ep.h);
  for (i = 0; i < p2pCnt; i++)
  {
    mpp = p2p[i];
    sp = mpp->getStartPoseV();
    d = line.distanceSigned(sp.x, sp.y);
    if (fabs(d) > fabs(dist))
      dist = d;
    h = limitToPi(sp.h - ep.h);
    if (fabs(h) > fabs(head))
      head = h;
  }
  if (maxHeading != NULL)
    *maxHeading = head;
  return dist;
}

/////////////////////////////////////////////////////////

bool UManSeq::expandManDriveon(UPoseV fromPose, UPoseV toPose,
                        double turnRad,
                        UManPPSeq * ppSeq)
{
  bool result = false;
  double v, t, d;
  UPoseV calcEndPose;
  UPose poEnd, poDEnd;;
  double tu1, dr1, tu2;
  UPose2pose relEnd;
  bool left;
  const double ACC = 0.3; // not used much, as manoeuvre is an emulation
  UManoeuvre * ma;
  //
  releaseAll(ppSeq);
  ppSeq->setStartPoseV(fromPose);
  ppSeq->setEndPoseV(toPose);
  //
  relEnd = fromPose.getMapToPosePose(&toPose);
  poEnd = relEnd.get2line(turnRad, &tu1, &dr1, &tu2);
  // add the estimated manoeuvre elements
  if (fabs(tu1) > 0.0001)
  { // add a initial turn
    left = tu1 > 0.0;
    ma = getNewArc(turnRad, fabs(tu1), left, ACC, toPose.vel);
    result = ppSeq->add(ma);
  }
  if (dr1 > 0.02)
  {
    ma = getNewLine(dr1, ACC, toPose.vel);
    result = ppSeq->add(ma);
  }
  if (fabs(tu2) > 0.0001)
  { // first turn at initial speed
    left = (tu2 > 0.0);
    ma = getNewArc(turnRad, fabs(tu2), left, ACC, toPose.vel);
    result = ppSeq->add(ma);
  }
  poDEnd = poEnd.getMapToPosePose(&relEnd);
  if ((poDEnd.x > 0) and (fabs(poDEnd.y) < 0.5))
  { // add a final straight part to end as desired
    ma = getNewLine(poDEnd.x, ACC, toPose.vel);
    result = ppSeq->add(ma);
    poDEnd.x = 0.0;
  }
  //
  if (result)
  { // calculate final velocity and time used
    v = ppSeq->calcEndVel(&t, &d);
    ppSeq->setEndVel(v);
    ppSeq->setManTime(t);
    ppSeq->setManDist(d);
    calcEndPose = ppSeq->calcEndPose(ppSeq->getStartPoseV());
    d = calcEndPose.getDistance(toPose);
    if (poDEnd.x > 0.02 or fabs(poDEnd.y > 0.1))
    {
      printf("****UManSeq::expandManDriveon: NB! discrepancy %g from predicted to expected pose!\n", d);
      fromPose.print(   "\n    From");
      toPose.print(     "\nExpected");
      calcEndPose.print("\nPrediced");
      printf("\n");
      ppSeq->fprint(stdout, ":--");
      result = false;
    }
  }
  else
  { // debug
    //printf("UManSeq::expandMan:   Truncating solution!)\n");
  } // debug end
  //
  return result;
}

/////////////////////////////////////////////////

bool UManSeq::addManDriveon(UPoseV fromPose, UPoseV toPose,
                     double turnRad)
{
  bool result = p2pCnt < MAX_PP_MAN_CNT;
  UManPPSeq * ppseq = NULL;
  //
  if (result)
  {
    if (p2p[p2pCnt] == NULL)
      ppseq = new UManPPSeq();
    else
      ppseq = p2p[p2pCnt];
    result = ppseq != NULL;
  }
  if (result)
  {
    p2p[p2pCnt++] = ppseq;
    result = expandManDriveon(fromPose, toPose, turnRad, ppseq);
  }
  return result;
}

//////////////////////////////////////////////

bool UManSeq::replaceManDriveon(UPoseV midPose, bool fixate,
                         double turnRad,
                         int fromIdx, int toIdx,
                         bool newEndpoint)
{
  UPoseV fromPose;
  UPoseV mPose;
  UPoseV toPose;
  UManPPSeq  *mpp1 = NULL, *mpp2 = NULL;
  bool result = ((p2pCnt + fromIdx - toIdx + 1) < MAX_PP_MAN_CNT);
  int i, d, s;
  //bool lastMan;
  //
  if (result)
  { // ensure that exactly two sequental manoeuvre
    // sequences 'UManPPSeq' are available.
    fromIdx = maxi(0, fromIdx);
    toIdx = mini(p2pCnt-1, toIdx);
    mpp1 = p2p[fromIdx];
    mpp2 = p2p[toIdx];
    fromPose = mpp1->getStartPoseV();
    toPose = mpp2->getEndPoseV();
    if (fromIdx == toIdx)
    { // only one sequence, there must be 2, so add one
      toIdx++;
      if (toIdx < p2pCnt)
      { // copy one seq in from end of list
        mpp2 = p2p[p2pCnt];
        memmove(&p2p[toIdx + 1], &p2p[toIdx], sizeof(UManPPSeq*) * (p2pCnt - toIdx));
        p2p[toIdx] = mpp2;
        //
        if (mpp2 != NULL)
          // the new one should be cleared, but all is replaced
          // except the fixate flag for the last pose.
          mpp2->setLastFixed(false);
      }
      p2pCnt++;
    }
    if ((toIdx - fromIdx) > 1)
    { // two is needed only, so delete the rest
      // debug
      // if (toIdx - fromIdx > 3)
      //   printf("Big chunk huh!\n");
      // debug end
      d = fromIdx + 1;
      s = d + 1;
      for (i = fromIdx + 1; i < toIdx; i++)
      { // move surplus to end of list, one at a time
        mpp2 = p2p[d]; // save surplus
        memmove(&p2p[d], &p2p[s], sizeof(UManPPSeq*) * (p2pCnt - (fromIdx + 2)));
        p2p[--p2pCnt] = mpp2; // insert surplus at end
      }
    }
    toIdx = fromIdx + 1;
    mpp2 = p2p[toIdx];
    if (mpp2 == NULL)
    { // create new if needed
      mpp2 = new UManPPSeq();
      p2p[toIdx] = mpp2;
    }
  }
  i = 0;
  //lastMan = (toIdx == p2pCnt - 1);
  if (result)
  { // now two pose-to-pose sequence objects are in place
    // then just find the needed manoeuvres to get to endpose
    // passing the mid-pose
    result = expandManDriveon(fromPose, midPose, turnRad, mpp1);
    if (not result and not newEndpoint)
    { // we did not reach the destination totally,
      // modify midPose (if not the end)
      mPose = mpp1->calcEndPose(fromPose);
      mpp1->setEndPoseV(mPose);
      result = true;
    }
    mpp1->setLastFixed(fixate); // fixate if in narrow passage
    if (newEndpoint)
    { // new endpoint, so last manoeuvre is not needed
      p2pCnt = fromIdx + 1;
    }
    else
    { // and the second part, using the end velocity from first part
      midPose = mpp1->getEndPoseV(); // gets desired end velocity, not real
      // replace with actual obtained velocity
      midPose.setVel(mpp1->getEndVel());
      result = expandManDriveon(midPose, toPose, turnRad, mpp2);
      mpp2->setFirstFixed(fixate); // fixate if in narrow passage
    }
  }
  return result;
}

