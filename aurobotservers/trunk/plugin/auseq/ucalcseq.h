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
#ifndef UCALCSEQ_H
#define UCALCSEQ_H


#include <ugen4/ulock.h>
#include <umap4/upose.h>
#include <urob4/uresposehist.h>

//#include "umucalc.h"
#include "ucalc.h"

//class ULaserScanSet;

/**
Class for symbolic calculation on Natty bumpo variables */
class UCalcSeq : public UCalc
//class UCalcSeq : public UMuCalc
{
public:
  /**
  Constructor */
  UCalcSeq();
  /**
  Destructor */
  virtual ~UCalcSeq();
  /**
  Get the calculated pose in
  calcX, calcY and calcH as a UPose */
  UPose getCalcPose();
  /**
  Add system defined variables for MMR */
  int addSysVars();
  /**
  Get system function online help.
  Returns the help text for function 'index' and
  returns further more the number of help-lines in
  count. */
  const char * getSysFunctionHelp(const int index, int * count);
  /**
  Set odometry and time variable to this value */
  void setOdoPoseTime(UPoseTime value);
  /**
  Set pose and time for the most recent drive update */
  void setDrivePoseTime(UPoseTime value);
  /**
  Set odometry (x,y,h), time and velocity variables to this value */
  void setOdoPoseTimeVel(UPoseTime value, double vel);
  /**
  Set the current velocity */
  void setOdoSpeed(double odoSpeed);
  /**
  Set GPS  (EKF) current position and its update time */
  void setEkfPoseTime(UPoseTime value);
  /**
  Get most recent GPS (EKF) position and update time */
  UPoseTime getEkfPoseTime();
  /**
  Get current timeout time (absolut unix time) */
  UTime getTimeout();
  /**
  Get current time (absolut unix time) */
  UTime getTime();
  /**
  Set new time value - should be set from odometry time */
  inline void setTime(UTime value)
  { varTime->setValued(value.getDecSec(), 0, false); };
  /**
  Set drive mode (0 = idle (do not issue drive commands), 1 = go, ...) */
  inline void setDriveMode(int mode)
  { varDriveMode->setInt(mode, 0); };
  /**
   * Is sysVars in place */
  inline bool isSysVarsDefined()
  { return varReplayTime != NULL; };
  /**
  Set drive mode (0 = idle (do not issue drive commands), 1 = go, ...) */
  inline int getDriveMode()
  { return roundi(varDriveMode->getValued()); };
  /**
  Set simulation stop time. Controls
  data sources (GPS and ODO) to advance to this time, pushing
  data as they go. */
  inline void setReplayTime(UTime replayTime)
  { varReplayTime->setTime(replayTime); };
  /**
  Get simulation stop time.
  Used by other data sources (GPS and ODO), to get matching logfile
  data. */
  UTime getReplayTime();
  /**
  Set simulated flag */
  inline void setReplay(bool value)
  { varReplay->setBool(value); };
  /**
  get simulated flag */
  inline bool getReplay()
  { return varReplay->getValueBool(0); };

protected:
  /**
  Evaluate a sequencer function, if such exist.
  Returns true if it exist.
  If not just 'syntaxCheck' and if not syntax errors are found, then
  the value is returned in 'value' */
  virtual bool evaluateSequencerFunction(const char * name,
                                      const double pars[], int parsCnt,
                                      double * value, bool syntaxCheck);

protected:
  /**
  Evaluate distance from an X,Y position to the line formed
  by a given pose - (X,Y,H).
  Returns distance */
  double distToPoseLineSigned(double fromX, double fromY,
                              double lineX, double lineY, double lineH);

protected:
  /**
  Pose history for historic calculations - some 50 m gets stored */
  //UResPoseHist * poseHist;
  /**
  Laser scan history */
  //ULaserScanSet * laser;

////////////////

protected:
// index of system variables
  /** Number of (read-only) constants - not used */
  //int varsConstCnt;
  /** Number of system defined variables - not used */
  //int varsSysCnt;
  /** index for newest odo update */
  UVariable * varTime;
  /**
  Index for drive mode variable */
  UVariable * varDriveMode;
  /**
  Simulation */
  UVariable * varReplay;
  /**
  Simulation time */
  UVariable * varReplayTime;
  //
public:
  /**
  Function that cal evaluate scripted functions */
  double (*doEvaluateFunction)(const char * functionName,
                                        char ** parameters,
                                        bool * resultValid);

};

#endif
