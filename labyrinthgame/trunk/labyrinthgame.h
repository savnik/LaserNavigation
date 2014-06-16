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

#ifndef SERIAL_H
#define SERIAL_H

#define MAXPUNKT 200
// points on route
extern int pathPoints[MAXPUNKT][2];
// number of points on route
extern int pathPointsCnt;
// control debug mode:
//    0 : normal operation
//    1 : firewire debug
//    2 : load images for imgRaw and imgFull from disk (do not use camera)
extern int globalDebug;
//extern int *gSema;
/// control semaphore to start a new control cycle after an image is analyzed
extern USemaphore semBallPosition;

enum gameStates { GAME_WAIT_TO_START, GAME_CRANING, GAME_START, GAME_RUN, GAME_OVER};
extern gameStates gameState;

const char * getGameStateString();

/**
 * Convert this image position to frame position
 * \param r is image row number
 * \param c image column
 * \param x is set to x position in frame coordinates 0,0 is top-left x is right y is down
 * \param y is set to y position in framed coordinates */
void imageToFrame(float r, float c, float * x, float * y);

/**
 * Convert this frame position to image position
 * \param x is in frame coordinates 0,0 is top-left cornet of frame x is right y is down
 * \param y is in frame coordinates
 * \param r is set to corresponding image row
 * \param c is set to corresponding image coumn */
void frameToImage(float x, float y, float * r, float * c);
/// save current configuration to ini-file
void saveConfiguation();
/// debug log flag
extern bool debugLog;


#endif


