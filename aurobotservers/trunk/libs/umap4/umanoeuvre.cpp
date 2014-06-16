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
#include "umanoeuvre.h"

UManoeuvre::UManoeuvre()
{
  acc = 0.0;
  vel = 0.0;
  manType = MAN_NONE;
}

////////////////////////////////////////

UManoeuvre::~UManoeuvre()
{
}

////////////////////////////////////////

UPose UManoeuvre::getEndPose()
{ // relative end pose
  UPose result(0.0, 0.0, 0.0);
  return result;
}

////////////////////////////////////////

UPoseV UManoeuvre::getEndPoseV(UPoseV startPoseV)
{
  return startPoseV;
}

////////////////////////////////////////

double UManoeuvre::getManTime(double startVel)
{
  double result;
  double ta;
  double sa;
  double d = getDistance();
  double v;
  //
  // find acceleration time and distance
  ta = absd(vel - startVel)/absd(acc);
  sa = 0.5 * acc * sqr(ta);  // s = �at^2
  // is that distance available?
  if (sa < d)
  { // full acceleration plus constant speed at rest
    result = ta;
    if (vel > 0.05)
      result += (getDistance() - sa) / absd(vel);
  }
  else
  { // long time as velocity may be zero and
    // The stationary part do not count
    // if distance is too short, then acceleration can not reach final speed
    v = (vel + startVel)/2.0 + 1e-5;
    if (d / v > ta)
      result = ta;
    else
      result = d/v;
  }
  return result;
}

////////////////////////////////////////

double UManoeuvre::getEndV(UPoseV startPoseV)
{
  return getEndV(startPoseV.getVel());
}

////////////////////////////////////////

double UManoeuvre::getEndV(double startV)
{ // ve� - vs� = 2 a s
  double endv;
  double ta;
  double sa;
  //
  // find acceleration time and distance
  ta = absd(vel - startV)/acc;
  sa = 0.5 * acc * sqr(ta);  // s = �at^2
  if (sa > getDistance())
  { // end vel is not reached
    if (vel > startV)
      // increase
      endv = startV + sqrt(2.0 * getDistance() * acc);
    else
      // decrease
      endv = startV - sqrt(2.0 * getDistance() * acc);
  }
  else
    endv = vel;
  return endv;
}

////////////////////////////////////////

UPoseV UManoeuvre::getPoseV(double atManTime, UPoseV startPoseV, double endV)
{
  return startPoseV;
}

////////////////////////////////////////

void UManoeuvre::fprint(FILE * fd, const char * prestr, double * v)
{
  fprintf(fd, "%s NO Manoeuvre (base object)\n", prestr);
}





////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////

UManStop::UManStop()
{
  manType = MAN_STOP;
}

////////////////////////////////////////

UManStop::~UManStop()
{
}

////////////////////////////////////////

const char * UManStop::print(char * prestr, double * v, char * buff, const int buffCnt)
{
  snprintf(buff, buffCnt, "%s STOP\n", prestr);
  return buff;
}

////////////////////////////////////////

bool UManStop::getSMRCLcmd(char * buf, int bufCnt)
{
  snprintf(buf, bufCnt, "idle");
  return true;
}

/////////////////////////////////////

bool UManStop::getSMRCLcmd2(char * buf, int bufCnt,
            UPoseV * startPose,
            bool * first, double firstDistance,
            int * lineCnt, int lineCntMax,
            double * distSum, double distSumMax)
{
  bool result;
  UPoseV pv;
  //
  result = getSMRCLcmd(buf, bufCnt);
  (*lineCnt)++;
  *first = false;
  //
  return result;
}
