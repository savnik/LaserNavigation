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
 * main global variables for labyrinth game */

#ifndef TILTANDCRANEIF_H
#define TILTANDCRANEIF_H

#define GAME_WIDTH 0.248
#define GAME_WIDTH_PIXELS 472.0
/// control values to game
/// Limit to controlled angle
#define ANGLE_LIMIT 0.0713
/// tilt angle in radians
/// limits are approx. 4.1 degrees (+/- 0.0713 radians)
extern float tiltXangle, tiltYangle;
/// semaphore to tell output communication that new values are available
extern USemaphore semOutputCtrl;
/// flag to signal that ball shouldbe loadedwith crane
extern bool loadBallWithCrane;
/// error flag for this unit
extern bool errorTiltAndCraneIf;
/// is ball available for crane
extern bool ballAvailable;
/// is crane swing switch closed
extern bool craneSwitch;
/// is start switch pushed
extern bool startGameSwitch;
/// balance point for game plate to be about newtral 
/// a number between 0 and 255 (should be about 128, 
/// but that woulde require HW modification - not done as of 9/8/2013 /Chr
extern int xyBalance[2];
/// start control interface to tilt position and crane
bool startTcrifCtrl();
/// stop control interface to tilt position and crane
void stopTcrifCtrl();
/// debug string to send to crane-game controller
const int MIOL = 100;
extern char debugIfString[MIOL];
/// time of last status
extern UTime statusTime;
/// is power on to the hardware
extern bool powerOn;
/// old control values - to save a bit of time in IO interface
extern int xi_old, yi_old;

enum controlState { IF_INIT, IF_WAIT, IF_CRANE_INIT, 
                    IF_CRANE_LEFT_TO_BALL, IF_CRANE_DOWN_TO_BALL, IF_CRANE_UP_TO_TEST, 
                    IF_CRANE_BALL_TEST,
                    IF_CRANE_UP, IF_CRANE_LEFT_TO_DROP, IF_CRANE_DOWN_TO_DROP,
                    IF_CRANE_UP_TO_PARK, IF_CRANE_RIGHT_TO_PARK,
                    IF_TILT};
/// 
extern controlState ifState;
/// distance from park position to ball - in stepper units
extern int craneParkToBall;
/// distance from park position to ball drop position - in stepper units
extern int craneParkToDrop;
/// distance down to pick ball in stepper units from park position
extern int craneDownToBall;
/// distance to lift ball to test if got ball in stepper units from park position
extern int craneDownToTest;
/// distance where ball is dropped to game
extern int craneDownToDrop;
/// tilt scale from tilt angle to tilt units
extern float tiltScale;


/// get interface state as string
const char * getControlStateString();


#endif


