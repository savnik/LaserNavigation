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

//#include "constants.h"

//#include "planner.h"
#include <ugen4/u2dline.h>
#include <urob4/uvarcalc.h>

#include "ucalcseq.h"
//#include "uscanset.h"


UCalcSeq::UCalcSeq()
{
  //poseHist = new UPoseHistUTM();
  doEvaluateFunction = NULL;
  varDriveMode = NULL;
  varReplay = NULL;
  varReplayTime = NULL;
  //laser = NULL;
//  gmkPool = NULL;
//  imgPath = NULL;
  //addSysVars();
}

//////////////////////////////////////////////////////////////

UCalcSeq::~UCalcSeq()
{
/*  if (poseHist != NULL)
    delete poseHist;*/
}

//////////////////////////////////////////////////////////////

int UCalcSeq::addSysVars()
{
  UVarPool * vars;
  //
  vars = getVarPool();
  if (vars != NULL)
  {
    // add fixed values to calculator
    // NB! do not reorder or insert values (just add at end)
    // index may be used elsewhere
/*    vars->addVar("false", 0.0, "d", "Constant false is the same as 0");          //
    vars->addVar("true", 1.0, "d", "Constant true is the same as 1");          //
    vars->addVar("pi", M_PI, "d", "Constant pi is the same as 3.14159...");*/
    // set read-only limit
    //varsConstCnt += 1;
    // road in general
    varDriveMode     = vars->addVar("driveMode", 0.0, "d", "0=idle, 1= go");
    //varLaserTilt     = vars->addVar("laserTilt", -10.0 * M_PI / 180.0, "d", "Current tilt of laserscanner - is this implemented?");
    //
    // GMK
/*    varGmkX = vars->addVar("gmkX", 0.0, "3d", "Position of selected guidemark");
    varGmkY = vars->addVar("gmkY", 0.0, "3d", "");
    varGmkZ = vars->addVar("gmkZ", 0.0, "3d", "");
    varGmkO = vars->addVar("gmkO", 0.0, "rot", "Orientation of selected guidemark");
    varGmkP = vars->addVar("gmkP", 0.0, "rot", "");
    varGmkK = vars->addVar("gmkK", 0.0, "rot", "");
    varGmkTime = vars->addVar("gmkTime", 0.0, "d", "Detection time of selected guidemark");
    varGmkId = vars->addVar("gmkId", 0.0, "d", "ID number of selected guidemark");*/
    // vision - use path from vision
    //
    varReplay     = vars->addVar("replay", 0.0, "d", "Is sequencer in replay mode ('1' is replay)"); // default false
    // simulation time (current time in logfiles, when simulation replay)
    varReplayTime       = vars->addVar("replayTime", 0.0, "d", "Current time index to replay file(s)");
    //
  }
  // add also std math functions
  // addMathMethods();
  //
  return getVarPool()->getVarsCnt(); //varsSysCnt;
}

/////////////////////////////////////////////////////////

bool UCalcSeq::evaluateSequencerFunction(const char * name,
                              const double pars[], int parsCnt,
                              double * value, bool syntaxCheck)
{
  return false;
}

/////////////////////////////////////////////////////////

UTime UCalcSeq::getReplayTime()
{
  UTime result;
  result.setTime(varReplayTime->getValued());
  return result;
}
