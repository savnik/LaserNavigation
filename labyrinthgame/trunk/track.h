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
#ifndef TRACK_H
#define TRACK_H
/** ****************************************************************
  Dato       : 2/7 1990                                            *
 *******************************************************************/


/**
 * Find position on track nearest to this ball position
 * \param x2,y2 is current ball position
 * \param linex,liney is returnd closest position on track,
 * \param itot is (total) number of points in track
 * \param minpt is point (edge) nearest to ball position - point after line
 * \param track is track point array,
 * returns true if found. */
void findTrackPart(float x2, float y2, float * linex, float * liney, int itot, int *minpt, int ref[][2]);

#endif
