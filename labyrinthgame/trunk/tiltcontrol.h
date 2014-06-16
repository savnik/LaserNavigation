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

#ifndef TILTCONTROL_H
#define TILTCONTROL_H

/// restart count - kalman filter restarted
extern int controllRestartCnt;
/// current reference position in track coordinates
extern float currentRefX, currentRefY;
/// current segment in pathPoints
extern int currentSegmet;
/// flag to start control - set true, when ball is positioned
extern bool readyToControl;
/// ball is lost
extern bool ballLost;
/// staic value for conversion
extern const double pixelsPerMeter;

/// start control loop for the tilt angle
bool startControl();
/// stop control loop for the tilt angle and wait for it to terminate
void stopControl();

#endif


