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

#ifndef SIMGAME_H
#define SIMGAME_H

extern float simBallVelX, simBallVelY; /// ball velocity
extern float simBallPosX, simBallPosY; /// ball position
extern UTime simBallTime;
extern float simTs;    /// sampleTime
extern float simTau;   /// tilt control time constant (1. order model)
extern float simDelay; /// delay in seconds
extern bool simActivate; /// flag for activate simulated ball position
extern bool simValid;  /// is simulated position valid - initialized and running
/// start control interface to tilt position and crane
bool startSim();
/// stop control interface to tilt position and crane
void stopSim();

#endif


