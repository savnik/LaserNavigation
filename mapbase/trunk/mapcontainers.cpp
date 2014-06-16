/** \file mapcontainers.cpp
 *  \ingroup Mapping
 *  \brief Mapcontainers for the graph mapping lib
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2012-01-25 13:38:07 +0100 (Wed, 25 Jan 2012) $
 */
/***************************************************************************
 *                  Copyright 2009 Anders Billesø Beck, DTU                *
 *                       anders.beck@get2net.dk                            *
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
/*********************** Version control information ***********************/
 #define VERSION  "2.0"
 #define REVISION "$Rev: 59 $:"
 #define DATE     "$Date: 2012-01-25 13:38:07 +0100 (Wed, 25 Jan 2012) $:"
/***************************************************************************/

#include <string.h>

#include "mapcontainers.h"

connector::connector() {
}

connector::~connector() {
}

void connector::copy(connector source) {
    id = source.id;
    strncpy(name,source.name,NAMELEN);
    th = source.th;
    x = source.x;
    y = source.y;
	 poseLocal = source.poseLocal;
	 poseMap = source.poseMap;
    autoconnect = source.autoconnect;
    features = source.features; /**< Mapped list of features */
}

node::node() {
}

node::~node() {
}

/** Copy all content from another node to this instance
 *
 **/
void node::copy(node source) {

    strncpy(name,source.name,NAMELEN);
    strncpy(typeName,source.typeName,NAMELEN);
    strncpy(shape,source.shape,NAMELEN);

    width = source.width;
    heigth = source.heigth;
    diameter = source.diameter;
    connectors = source.connectors; //Vectors has = operator
    features = source.features; //So has maps
    x = source.x;
    y = source.y;
    th = source.th;
    parentId = source.parentId;
	 poseMap = source.poseMap;
	 poseLocal = source.poseLocal;
	 pMap = source.pMap;
	 pLocal = source.pLocal;
}

edge::edge() {

    //Initialize some variables
    blocked = false;
    disabled = false;
	 length = 0.0;
    directional = false;
    name[0] = '\0';
    typeName[0] = '\0';
    startNodeName[0] = '\0';
    endNodeName[0] = '\0';
    startConnName[0] = '\0';
    endConnName[0] = '\0';
    end[0] = '\0';
    start[0] = '\0';
}


edge::~edge() {
}

/** Copy all content from another edge to this instance
 *
 **/
void edge::copy(edge source) {

    strncpy(name,source.name,NAMELEN);
    strncpy(typeName,source.typeName,NAMELEN);
    strncpy(startNodeName,source.startNodeName,NAMELEN);
    strncpy(endNodeName,source.endNodeName,NAMELEN);
    strncpy(startConnName,source.startConnName,NAMELEN);
    strncpy(endConnName,source.endConnName,NAMELEN);
    strncpy(end,source.end,NAMELEN);
    strncpy(start,source.start,NAMELEN);
    startNodeIndex  = source.startNodeIndex;
    startConnIndex  = source.startConnIndex;
    endNodeIndex    = source.endNodeIndex;
    endConnIndex    = source.endConnIndex;
    startConnId     = source.startConnId;
    endConnId       = source.endConnId;
    startnode       = source.startnode;
    startconnector   = source.startconnector;
    endnode         = source.endnode;
    endconnector    = source.endconnector;
    length          = source.length;
    blocked         = source.blocked;
    directional     = source.directional;
    features        = source.features; //Maps has a = operator

}
