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
#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
#include <ugen4/ulock.h>
#include <urob4/ulogfile.h>
#include <../libs/ugen4/ulock.h>
#include "labyrinthgame.h"
#include "fwguppy.h"
#include "tiltcontrol.h"
#include "tiltandcraneif.h"
#include "gamesim.h"
#include "showimage.h"
#include <ugen4/utime.h>

/// is interface control  thread running
bool simThreadRunning;
/// stop running interface control  thread
bool stopSimThread;
/// interface control thread handle
pthread_t thSim;


float simBallVelX, simBallVelY; // ball velocity
float simBallPosX, simBallPosY; // ball position
float simTs = 0.005;    // sampleTime
float simTau = 0.05;   // tilt control time constant (1. order model)
float simDelay = 0; // delay in seconds
bool simActivate; // flag for activate simulated ball position
bool simValid; // is simulated position valid - initialized and running
// start control interface to tilt position and crane
bool startSim();
// stop control interface to tilt position and crane
void stopSim();
// validitime for simulated ball position
UTime simBallTime;



/// ////////////////////////////////////////////////////////////

void * runSim(void * notUsed)
{
  enum gameStates { SIM_WAIT, SIM_RUN, SIM_END};
  gameStates simState = SIM_WAIT;
  UTime t;
  float dt;
  float simAngleX = 0, simAngleY = 0;
  const float g = 9.8;
  ULogFile lf;
  //
  simThreadRunning = true;
  t.now();
  lf.setLogName("gamesim");
  if (debugLog)
    lf.openLog();
  if (lf.isLogOpen())
    fprintf(lf.getF(), "# time           | dt | refx | anglex | accx | velx | posx | refy | angley | accy | vely | posy |\n");
  while (not stopSimThread)
  {
    float ay;
    float ax;
    dt = t.getTimePassed();
    // debug for single step
    if (dt > 0.1)
      dt = 0.005;
    t.now();
    switch (simState)
    {
      case SIM_WAIT:
        if (ballOK and readyToControl)
        { // initialize pose
          simBallPosX = ballPosition[0];
          simBallPosY = ballPosition[1] + frameTop;
          simBallVelX = 0;
          simBallVelY = 0;
          simBallTime = ballTime;
          simState = SIM_RUN;
        }
        break;
      case SIM_RUN:
        // get new angle after 1. order dynamics
        simAngleX = (dt * tiltXangle + simTau * simAngleX) / (simTau + dt);
        simAngleY = (dt * tiltYangle + simTau * simAngleY) / (simTau + dt);
        // get new acc in m/s2
        ax = sin(simAngleX) * g;
        // to pixels per seconds squared pix/s2
        ax *= GAME_WIDTH_PIXELS / GAME_WIDTH;
        // new ball velocity in pix/s2
        simBallVelX += ax * dt;
        //
        ay = sin(simAngleY) * g;
        // to pixels per seconds squared p/s2
        ay *= GAME_WIDTH_PIXELS / GAME_WIDTH;
        // new ball velocity in pix/s2
        simBallVelY += ay * dt;
        //
        // get new ball position
        simBallPosX += simBallVelX * dt;
        simBallPosY += simBallVelY * dt;
        simBallTime.add(dt);
        // make log
        if (lf.isLogOpen())
          fprintf(lf.getF(), "%lu.%06lu %.4f "
                            "%.5f %.5f %.5f %6.2f "
                            "%.5f %.5f %.5f %6.2f\n",
                            t.getSec(), t.getMicrosec(), dt,
                            tiltXangle, ax, simBallVelX, simBallPosX,
                            tiltYangle, ay, simBallVelY, simBallPosY);
        simValid = true;
        if (ballLost)
          simState = SIM_END;
        break;
      case SIM_END:
        simValid = false;
        break;
    }
    Wait(simTs);
  }
  lf.closeLog();
  // we are finished here
  simThreadRunning = false;
  pthread_exit(NULL);
  return NULL;
}


/// ///////////////////////////////////////////////////////////////////////////
// start control interface to tilt position and crane
bool startSim()
{
  pthread_attr_t  thAttr;
  int i;
  //
  if (not simThreadRunning)
  { // start thread
    pthread_attr_init(&thAttr);
    //
    stopSimThread = false;
    // create thread for tilt and crane control
    if (pthread_create(&thSim, &thAttr, &runSim, NULL) != 0)
      // report error
      perror("Sim control thread");
    // wait for threads to initialize
    i = 0;
    while ((not simThreadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not simThreadRunning)
    { // failed to start
      printf("startSim: Failed to start thread - in time (5 sec)\n");
    }
    pthread_attr_destroy(&thAttr);
  }
  return simThreadRunning;
}

/// ///////////////////////////////////////////////////////////////////////////
// stop control interface to tilt position and crane
void stopSim()
{
  if (simThreadRunning)
  {
    stopSimThread = true;
    if (simThreadRunning)
      pthread_join(thSim, NULL);
    // debug
    printf("stopSim: thread stopped\n");
  }
}

