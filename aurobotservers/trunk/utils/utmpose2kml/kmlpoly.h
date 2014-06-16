/***************************************************************************
 *   Copyright (C) 2008 by Christian Andersen   *
 *   chrand@mail.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef KMLPOLY_H
#define KMLPOLY_H

#include "poseq.h"

/// zone 32 UTM offset for pometet i Tåstrup
# define OX 708000.0             // UTM mapping offset in x direction
# define OY 6174000.0            // UTM mapping offset in y direction

/**
Class to read a polyline exported by google earth

	@author Christian Andersen <chrand@mail.dk>
*/
class KmlPoly{
public:
  /// constructor
  KmlPoly();
  /// destructor
  ~KmlPoly();
  /// xpat start element handler
  static void startTag(void *userData, const XML_Char *el, const XML_Char **attr);
  /// expat end element handler
  static void endTag(void *userData, const XML_Char *el);
  /// expat text handler
  static void tagText(void *userData, const XML_Char *s, int len);
  /// load simple graph
  bool load(FILE * fptr);
  /** save polyline as route rule file
   * \param fd destination file handle, or NULL is destination is to be generated from source
   * \param offsetX is an additional offset - in X or easting - to compensate for errors in google earth image
   * \param offsetY is an additional offset in the Y (northing) direction.
   * \param inUTM makes the rule use driveUTM rather then driveMap commands
   * \returns true if file is written */
  bool makeRouteRule(FILE * fd, double offsetX, double offsetY, const char * name, bool inUTM);
  /**
   * make additional rule of way-point and addition to map for display and localizer purposes
   * \param fd destination file handle, or NULL is destination is to be generated from source
   * \param offsetX is an additional offset - in X or easting - to compensate for errors in google earth image
   * \param offsetY is an additional offset in the Y (northing) direction.
   *  */
  bool makeRoutePlanRule(FILE * fd, double offsetX, double offsetY,
                         bool isClosed, const char * name, bool inUTM);
  /**
   * modify path to a number of parallel paths, separated by a headline drive.
   * \param rowCnt is the number of paths to generate
   * \param rowSep is distance between each path
   * \param minTurnDiam is minimum turn diameter
   * \param hDist is distance to headline transport line
   * \param useUTM leave result rule in UTM coordinates, rather than map coordinates.
   * returns true if path is planned, and entered into kml-point list */
  bool modifyPath(int rowCnt, double rowSep, double minTurnDiam,
                  double hDist, bool useUTM);
  /** save polyline as MRC drive script - running on kalman as odometry
   * \param fd destination file handle, or NULL is destination is to be generated from source
   * \param offsetX is an additional offset - in X or easting - to compensate for errors in google earth image
   * \param offsetY is an additional offset in the Y (northing) direction.
   * \returns true if file is written */
  bool makeRouteMRC(FILE * fd, double offsetX, double offsetY, const char * name);

public:
  bool isKml;
  bool placemark;
  bool coordinates;
  bool isName;
  static const int bufMaxCnt = 100;
  char buf[bufMaxCnt];
  int bufCnt;
  static const int pktMaxCnt = 10000;
  double pkt[pktMaxCnt];
  int pktCnt;
  static const int kmlnameMaxCnt = 100;
  char kmlname[kmlnameMaxCnt];
  static const int MAX_POSE_CNT = pktMaxCnt / 3;
  PoseQ poses[MAX_POSE_CNT];
  int posesCnt;
};

#endif
