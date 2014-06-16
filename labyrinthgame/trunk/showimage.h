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
#ifndef SHOWIMAGE_H
#define SHOWIMAGE_H

#include <ugen4/uimage.h>

/// size of image
extern int showImageHeight;
extern int showImageWidth;
extern int showImageClickRoute;
extern int showImageClickFrame;
extern int framePoints[4][2]; 
extern int framePointsCnt;
// from start position to end
const int MRP = 400;
extern int routePoints[MRP][2];
extern int routePointsCnt;
/// image buffer for displayed image
extern UImage * imgBufferDebug;
/// angle of frame top line
extern float frameAngle;


/// framepoints startion top left going CCV
extern int framePoints[4][2];

/** start show image thread
 * \returns false if image could not be shown */
bool startShowImage();
/**
 * Stop image show thread and release all resources */
void stopShowImage();

/**
 * Save route and frame to disk */
void saveFrameAndRoute();


#endif
