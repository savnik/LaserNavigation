/***************************************************************************
 *   Copyright (C) 2013 by DTU (Christian Andersen, Andreas Emborg, m.fl.) *
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


/**
 * Labyrinth game start and end code */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ugen4/ulock.h>
#include <urob4/ulogfile.h>
#include "labyrinthgame.h"
#include "fwguppy.h"
#include "showimage.h"
#include "kalman.h"
#include "track.h"
#include "refgen.h"
#include "tiltcontrol.h"
#include "tiltandcraneif.h"

/// control values
float gXpos, gYpos;
/// is control thread running
bool controlThreadRunning = false;
/// stop control thread
bool stopControlThread = false;
/// thread handle
pthread_t thControl;
/// strat control thread (prototype)
bool startControl();
/// stop control thread
void stopControl();
// restart count - kalman filter restarted
int controllRestartCnt = 0;
// current reference position in track coordinates
float currentRefX, currentRefY;
// current segment in pathPoints
int currentSegmet;
// ball is lost
bool ballLost = false;
// flag to start control - set true, when ball is positioned
bool readyToControl;

const double pixelsPerMeter = GAME_WIDTH_PIXELS / GAME_WIDTH;
  //


/// /////////////////////////////////////////////////////////////////

void * runControl(void * notUsed)
{
  int ballOKCnt = 0;
  int ballNOKCnt = 0;
  float ballX, ballY;
//  float ballXold = 0.0, ballYold = 0.0;
  refgen_parm refGenState; /// state for reference generation
  int segment; /// actual segment on route (towards this point)
  double xlpold, ylpold; /// old reference position
  const int MHL = 5; /// max position history time
  double  ballOldX[MHL], ballOldY[MHL]; /// old ball position
  UTime ballOldTime[MHL];
  UTime lpOldTime;
  double vxlp, vylp; /// x and y velocity in meters per second
//  const double maxMovement = 20; /// pixels
  bool isOK;
//  const float refHastMPS = 0.85; // meter per second
  const float refHastMPS = 0.010; // meter per second
  const float refHastPPS = refHastMPS * GAME_WIDTH_PIXELS / GAME_WIDTH; // ref pixel per second
  ULogFile ctllog;
  //
  if (debugLog)
  {
    ctllog.openLog("control");
//         fprintf(ctllog.getF(), "%ld.%06ld %d %.2f %.2f %.2f %.2f %d %.2f %d %.4f %.4f %d %.6f %.6f %d %d\n",
//                     ballTime.getSec(), ballTime.GetMicrosec(), ballOK,
//                     ballX, ballY, refGenState.xlp, refGenState.ylp,
//                     lastPosIdx, lastPosM,
//                     ballOKCnt, 
//                     xvel, yvel, roundi(tiltScale), 
//                     tiltXangle, tiltYangle,
//                     xi_old, yi_old
    if (ctllog.isOpen())
      fprintf(ctllog.getF(), "# log of control related values\n"
        "#  0 log time in seconds\n"
        "#  1 ball found (0=not)\n"
        "#  2 ball position X (pixels)\n"
        "#  3 ball position Y (pixels)\n"
        "#  4 ball target position X (pixels)\n"
        "#  5 ball target position Y (pixels)\n"
        "#  6 current line segment index (of %d)\n"
        "#  7 target position in current line segment\n"
        "#  8 Consecutive ball OK count\n"
        "#  9 estimated ball velocity X (pixels/sample ?)\n"
        "# 10 estimated ball velocity Y (pixels/sample ?)\n"
        "# 11 scaling factor from tilt angle in radians to control units to controller\n"
        "# 12 tilt angle X (radians)\n"
        "# 13 tilt angle Y (radians)\n"
        "# 14 control value X [0..255] - offset=%d\n"
        "# 15 control value Y [0..255] - offset=%d\n", 
        pathPointsCnt,
        xyBalance[0], xyBalance[1]
      );
  }
  // load parameters for Kalman filter
  LoadKalman();
  refGenState.hast = refHastPPS;
  refGenState.tau = 0.81; // for smoothing corners
  controlThreadRunning = true;
  while (not stopControlThread)
  { // main control thread
    if (readyToControl)
    {
      double xvel = 0;
      double yvel = 0;
      double dt;
      // wait for a ball position to be available
      semBallPosition.wait();
      if (stopControlThread)
        break;
      ballLost = false;
      isOK = ballOK;
      if (isOK)
      { // convert ball image coordinates to frame coordinates
        imageToFrame(ballPosition[1] + frameTop, ballPosition[0], &ballX, &ballY);
        // debug removed
//         if (ballOKCnt > 1)
//         { // may be too far away
//           float ex = ballXold - ballX;
//           float ey = ballYold - ballY;
//           if( hypot(ex, ey) > maxMovement)
//             // wrong ball
//             isOK = false;
//         }
        // debug
      }
      if (not isOK)
      {
        ballOKCnt = 0;
        ballNOKCnt++;
        if (ballNOKCnt > 10)
        { // give up - ball lost
          ballLost = true;
        }
      }
      else
      {
        int nearPnt;
        //float lineX, lineY;
        //
        // 
        ballOKCnt++;
        ballNOKCnt = 0;
        // time since last update
        dt = ballTime - ballOldTime[0];
        if (dt > 0.3 or dt < 0.001)
          // all too old, so assume single step debugging and default interval
          dt = 0.005;

        if (ballOKCnt == 1)
        { // (re)init Kalman control filter
          controllRestartCnt++;
          // initialize both axes
          InitKalman(gkalx, ballX);
          InitKalman(gkaly, ballY);
          //findTrackPart(ballX, ballY, &lineX, &lineY, pathPointsCnt, &nearPnt, pathPoints);
          // initialize reference system
          if (nearPnt > 0)
            nearPnt--;
          segment = nearPnt;
          // start reference system from this ball position
          InitRefGen(&refGenState, pathPoints[segment][0], pathPoints[segment][1], refHastPPS);
          double d = findOnRoutePoint(&refGenState, pathPoints, &segment, pathPointsCnt, ballX, ballY);
          if (d > 0.015)
          { // more than 1.5cm away
            printf("**** failed to find a track near ball - faound one %f m away!\n", d);
            printf("**** - but continues\n");
          }
        }
        /// ///////////// fra run
        // update reference structure and get (possibly) new path segment.
        //bool finished = RefGen(&refGenState, pathPoints, &segment, pathPointsCnt);
        //double carrotDist = 0.015 * pixelsPerMeter; // in pixels
        double advanceDist = refHastPPS * dt; // advance distance for carrot point
        bool finished = RefGen2(&refGenState, pathPoints, &segment, pathPointsCnt, 
                                refGenState.xref, refGenState.yref, advanceDist);
        if (finished)
        { // level game plate
          gXpos = 0.0;
          gYpos = 0.0;
          ballLost = true;
          readyToControl  = false;
        }
        else
        {
          float xlp = refGenState.xlp;
          float ylp = refGenState.ylp;
          double ex = 0.0;
          double ey = 0.0;
          if (ballOKCnt == 1)
          { // just started assume zero velocity
            xlpold = xlp;
            ylpold = ylp;
            lpOldTime = ballTime - 0.001;
            for (int i = 0; i < MHL; i++)
            {
              ballOldX[i] = ballX;
              ballOldY[i] = ballY;
              ballOldTime[i] = ballTime;
            }
          }
          // calculate desired velocity in each direction
          vxlp=(xlp - xlpold) * 25;
          vylp=(ylp - ylpold) * 25;
          // get control value from state controller
          if (true)
          { // find vector from current ball position to carrot point
            double carrotDx = refGenState.xlp - ballX;
            double carrotDy = refGenState.ylp - ballY;
            double carrotDist = hypot(carrotDx, carrotDy);
            if (carrotDist / pixelsPerMeter > 0.03)
              // stop this attempt - the ball has lost sight of the ball
              ballLost = true;
            // calculate reference velocity in each direction (in pixels per second)
            double refVelX = refHastPPS * carrotDx / carrotDist;
            double refVelY = refHastPPS * carrotDy / carrotDist;
            // get time passed since oldest position
            dt = ballTime - ballOldTime[MHL - 1];
            // debug
            if (dt / MHL > 0.3 or dt < 0.001)
              // all too old, so assume single step debugging and default interval
              dt = 0.005 * MHL;
            // debug end
            // calculate measured velocity - over last MHL measurements
            xvel = (ballX - ballOldX[MHL - 1]) / dt;
            yvel = (ballY - ballOldY[MHL - 1]) / dt;
            // get velocity error (in pixels per second
            double errX = refVelX - xvel;
            double errY = refVelY - yvel;
            // calculate control signal - in angle radians
            // proportional controller
            // - from 0 to 0.82m/s (1600 Pix/s) in 0.3 sec requires about 2.4m/s2 acc, or an angle of 0.24 radianc (or 14 degrees)
            double ux = errX * 0.24/1600 * 1;
            double uy = errY * 0.24/1600 * 1;
            // send to control
            tiltXangle = ux;
            tiltYangle = uy;
          }
          else
          { // use kalman controller
            if (gkalx->ni == 1)
            { // use error as input to kalman state controller, as

              ex = xlp - ballX;
              ey = ylp - ballY;
              tiltXangle = KalmanOutput(gkalx, ex, 0.0, 0.0);
              tiltYangle = KalmanOutput(gkaly, ey, 0.0, 0.0);
            }
            else
            { // use also velocity - requires D-matrix to be 1x3
              tiltXangle = KalmanOutput(gkalx, ballX, xlp, vxlp);
              tiltYangle = KalmanOutput(gkaly, ballY, ylp, vylp);
            }
          }          
          xlpold = xlp;
          ylpold = ylp;
          lpOldTime = ballTime;
          for (int i = MHL - 1; i > 0; i--)
          { // delay old ball positions
            ballOldX[i] = ballOldX[i - 1];
            ballOldY[i] = ballOldY[i - 1];
            ballOldTime[i] = ballOldTime[i - 1];
          }
          ballOldX[0] = ballX;
          ballOldY[0] = ballY;
          ballOldTime[0] = ballTime;
//           ballXold = ballX;
//           ballYold = ballY;
          // tell output handler that new values are available
          semOutputCtrl.clearPosts();
          semOutputCtrl.post();

          // report to display the current values
          currentRefX = xlp;
          currentRefY = ylp;
          currentSegmet = segment;
          /* opdater regulator                                            */
          if (gkalx->ni == 1)
          { // B matrix is 5x1, and u is (assumed to be) error 
            OpdatKalman(gkalx, ex, 0.0, 0.0);
            OpdatKalman(gkaly, ey, 0.0, 0.0);
          }
          else
          { // B matrix assumed to be 5x3
            OpdatKalman(gkalx, ballX, xlp, vxlp);
            OpdatKalman(gkaly, ballY, ylp, vylp);
          }
        }
      }
      if (ctllog.isOpen())
      {
        fprintf(ctllog.getF(), "%ld.%06ld %d %.2f %.2f %.2f %.2f %d %.2f %d %.4f %.4f %d %.6f %.6f %d %d\n",
                    ballTime.getSec(), ballTime.GetMicrosec(), ballOK,
                    ballX, ballY, refGenState.xlp, refGenState.ylp,
                    lastPosIdx, lastPosM,
                    ballOKCnt, 
                    xvel, yvel, roundi(tiltScale), 
                    tiltXangle, tiltYangle,
                    xi_old, yi_old
               );
      }
    }
    else
    { // wait a bit
      ballOKCnt = 0;
      Wait(0.1);
    }
  }
  ctllog.closeLog();
  controlThreadRunning = false;
  pthread_exit(NULL);
  return NULL;
}


/// ///////////////////////////////////////////////////////////////////////////

bool startControl()
{
  pthread_attr_t  thAttr;
  int i = 0;
  //
  if (not controlThreadRunning)
  { // start thread
    pthread_attr_init(&thAttr);
    //
    stopControlThread = false;
    // create socket server thread
    if (pthread_create(&thControl, &thAttr, &runControl, NULL) != 0)
      // report error
      perror("control thread");
      // wait for thread to initialize
    while ((not controlThreadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not controlThreadRunning)
    { // failed to start
      printf("startControl: Failed to start thread - in time (5 sec)\n");
    }
    pthread_attr_destroy(&thAttr);
  }
  return controlThreadRunning;
}

/// ///////////////////////////////////////////////////////////////////////////

void stopControl()
{
  if (controlThreadRunning)
  {
    stopControlThread = true;
    if (controlThreadRunning)
    {
      semBallPosition.post();
      pthread_join(thControl, NULL);
    }
    // debug
    printf("stopControl: thread stopped\n");
  }
}


