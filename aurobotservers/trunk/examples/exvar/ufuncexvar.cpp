/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>

#include "ufuncexvar.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncExVar' with your classname, as used in the headerfile */
  return new UFuncExVar();
}

#endif

///////////////////////////////////////////////////

bool UFuncExVar::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 500;
  char reply[MRL];
  bool aLog, doLog;
  bool aReset;
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("MINIVAR");
    sendText("--- MINIVAR is a demonstration plug-in that counts number of calls and note the odometry pose and sets a few matrices\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    sendText(            "update              Update variables (default behaviour)\n");
    sendText(            "reset               Reset variables\n");
    sendText("help       This message\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    aLog = msg->tag.getAttBool("log", &doLog, true);
    aReset = msg->tag.getAttBool("reset", NULL, false);
    //
    // implement command
    if (aLog)
    { // open or close the log
      aLog = logf.openLog(doLog);
      if (doLog)
        snprintf(reply, MRL, "Opened logfile %s (%s)", logf.getLogFileName(), bool2str(aLog));
      else
        snprintf(reply, MRL, "closed logfile %s", logf.getLogFileName());
      // send an information tag back to client
      sendInfo(reply);
    }
    if (aReset)
    {
      resetVars();
      sendInfo("Values reset");
    }
    if (not (aLog or aReset))
    { // default update call
      updateVars();
      // format reply as valid XML tag
      snprintf(reply, MRL, "<minivar cnt=\"%d\" tod=\"%.3f\"/>\n", varUpdateCnt->getInt(0), varTime->getDouble());
      // send reply to client
      sendMsg(reply);
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncExVar::createResources()
{
  // Create global variables - owned by this plug-in.
  // Returns a pointer to the the variable for easy access.
  varLogName = addVar("logname", "exvar", "s", "(r) name of the logfile - is placed in dataPath");
  varUpdateCnt = addVar("updCnt", 0.0, "d", "(r/w) Number of updates");
  varPose = addVar("pose", "0 0 0", "pose", "(r) Odometry pose at last update");
  varTime = addVar("time", 0.0, "t", "(r) Time at last update");
  varAUMatrix = addVar("mau", "1 2 3; 2 1 2; 3 2 1", "m", "(r/w) AU Matrix 3x3");
  varCVMatrix = addVar("mcv", "1 2 3; 2 1 2; 3 2 1", "m", "(r/w) CV Matrix 3x3");
}

////////////////////////////////////////////////////////////////

void UFuncExVar::resetVars()
{
  UVarPool * vp;
  //
  // set CV matrix to a identity 3x3 matrix
  varCVMatrix->setValued("0 0 0; 0 0 0; 0 0 0", 0, true);
  // set AU matrix to a 3x3 matrix
  varAUMatrix->setValued("0 0 0; 0 0 0; 0 0 0", 0, true);
  // set update count
  varUpdateCnt->setInt(0);
  // set time (to now)
  varTime->setTimeNow();
  // reset pose [x, y, h]
  varPose->setValued("0 0 0", 0, true);
  //
  varLogName->setValues(logf.getLogName(), 0, true);
  //
  // reset trip counter B
  vp = getVarPool();
  vp->setGlobalVar("odoPose.tripB", 0.0, false);
}

////////////////////////////////////////////////////////////////

void UFuncExVar::updateVars()
{
  UMatrix4 mCV(3,3); // a sized matrix with maximum 16 elements
  matrix * m1, *m2, *mpp1, *mpp2, *mpp; // an IAU_mat matrix
  bool isOK = true;
  UPose pose1, pose3;
  UPoseTVQ pose2;
  UResPoseHist * odoPose;
  //
  // Update 
  // (old pose)
  pose1 = varPose->getPose();
  // get pointer to odometry pose history
  odoPose = (UResPoseHist*)getStaticResource("odoPose", false, true);
  // get newest pose
  pose2 = odoPose->getNewest();
  //
  if (pose1.getDistance(&pose2) < 0.001)
  { // pose is not updated, so make a random update
    // add a random distance up to 0.2m forward and turn up to 0.02 radians left or right
    pose2.add((double)rand() / (double)RAND_MAX * 0.2, (double)rand() / (double)RAND_MAX * 0.04 - 0.02);
    pose2.t.now();
    // tel the odometry pose history plugin that the pose has changed
    // (second parameter is the update source, negative numbers are plug-ins - else a socket client)
    odoPose->addIfNeeded(pose2, -19);
  }
  // set new pose and pose time
  pose3 = pose2;
  varPose->setPose(&pose3);
  varTime->setTime(pose2.t);
  // add one to update count
  varUpdateCnt->add(1.0);
  //
  // get pose change
  pose3 = pose2 - pose1;
  //
  // this is not very meaningfull, but just to show
  // how to get and set matrix values to and from global variables.
  //
  // openCV matrix example
  isOK = varCVMatrix->getM(&mCV);
  if (not isOK)
    printf("Size mismatch: %s var has size %dx%d but matrix is %dx%d\n",
          varAUMatrix->name, varAUMatrix->rows(), varAUMatrix->cols(), mCV.rows(), mCV.cols());
  // update value using OpenCV matrix
  // so that mCV = mCV + pose3 * pose3'
  // taking pose as a column vector
  mCV = mCV + pose3.asCol3() * pose3.asRow3();
  mCV.print("CV matrix");
  varCVMatrix->setValueM(&mCV);
  //
  // AU-mat example
  // update value using AU matrix
  // so that m2 = m1 + pose3 * pose3'
  // taking pose as a column vector
  m1 = mmake(3, 3);
  m2 = mmake(3, 3);
  // get current value of the matrix
  isOK = varAUMatrix->getM(m1);
  if (not isOK)
    printf("Size mismatch: %s has size %dx%d but should be 3x3\n",
          varAUMatrix->name, varAUMatrix->rows(), varAUMatrix->cols());
  // convert pose to vectors
  mpp1 = mmake(1, 3);
  mpp2 = mmake(3, 1);
  vput(mpp1, 0, pose3.x);
  vput(mpp1, 1, pose3.y);
  vput(mpp1, 2, pose3.h);
  vput(mpp2, 0, pose3.x);
  vput(mpp2, 1, pose3.y);
  vput(mpp2, 2, pose3.h);
  // make vector product
  mpp = mmake(3, 3);
  mmul(mpp, mpp2, mpp1);
  // add to previous value
  madd(m2, m1, mpp);
  // set result to global variable
  varAUMatrix->setValueM(m2);
  // free mat structures
  mfree(m1);
  mfree(m2);
  mfree(mpp);
  mfree(mpp1);
  mfree(mpp2);
  //
  //
}
