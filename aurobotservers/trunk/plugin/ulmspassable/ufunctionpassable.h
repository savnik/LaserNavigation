/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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
#ifndef URES_PASS_H
#define URES_PASS_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>
#include <ulms4/ulaserpool.h>

#include "urespassable.h"
#include "uresobstacle.h"
#include "uresroadline.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class UResPoseHist;

/**
* This plugin (or module) converts a laserscan to a set of passable intervals and obstacles,
  assuming that the laser is pointing down looking at the road at a distance of about 2.5-3 m.
* This class is the interface part of the module, the real work is done in the class UResPassable.
* The class also handles interface for obstacles and road data.
@author Christian Andersen
*/
class UFunctionPassable : public UFuncPlugBase
{
public:
    /**
    Constructor */
    UFunctionPassable();
    /**
    Destructor */
    virtual ~UFunctionPassable();
    /**
    Called by the server core. Should return the
    name of function. There should be a first short part separated
    by a space to some additional info (e.g. version and author).
    The returned name is intended as informative to clients and should
    include compile date or version number. */
    //virtual const char * name();
    /**
    Called by the server core when loaded, to get a list of
    keywords (commands) handled by this plugin.
    Return a list of handled functions in
    one string separated by a space.
    e.g. return "COG".
    The functions should be unique on the server. */
    //virtual const char * commandList();
    /**
    Create any resources that this modules needs and add these to the resource pool
    to be integrated in the global variable pool and method sharing. */
    virtual void createResources();
    /**
    List (space separated) of shared resources
    provided by this function.
    Must be an empty string if no resources are to be shared.
    Each resource ID must be no longer than 20 characters long. */
    /*  virtual const char * resourceList()
      { return "pass obst road"; }*/
    /**
    Called by the server core, when a new resource is
    available (or is removed), local pointers to the resource should be
    updated as appropriate. */
    virtual bool setResource(UResBase * resource, bool remove);
    /**
    This function is called by the server core, when it needs a
    resource provided by this plugin.
    Return a pointer to a resource with an ID taht matches this 'resID' ID string.
    The string match should be case sensitive.
    Returns false if the resource faled to be created (e.g. no memory space). */
//  UResBase * getResource(const char * resID);
    /**
    return true if all ressources is available */
//  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
    /**
    Handle incomming command
    (intended for command separation)
    Must return true if the function is handled -
    otherwise the client will get a failed - reply */
    virtual bool handleCommand(UServerInMsg * msg, void * extra);
    /**
    print status to a string buffer */
    virtual const char * print(const char * preString, char * buff, int buffCnt);
    /**
    Handle obstacle requests */
    bool handleObstGet(UServerInMsg * msg, void * extra);
    /**
    Send groups of obstacles to client.
    May send newly updated obstacles only when 'updatesOnly' is true.
     * \param msg is reference to command
     * \param maxCnt is maximum number of obstacle groups (from newest)
     * \param updateOnly sends only updated obstacles
     * \param andFixed sends (also) near fixed obstacles, as retreived from mapbase.
     * \returns true if send. */
    bool sendObstacles(UServerInMsg * msg, int maxCnt, bool updateOnly, bool andFixed);
    /**
     * send polygon to poly plugin - mainly for display in client
     * \param poly is the polygin to send
     * \param name is the name for the polygon in the poly plugin
     * \param coordinateSystem 0=odo 1=utm 2=map
     * \param color is the color character - like MATLAB r=red ....
     * \returns true if successful (else poly pluging is not found) */
    bool polygonToPolyPlugin(UPolygon * poly, char * name, int coordinateSystem, char color);
    /**
     * Delete a polygon (sets cont to zero) - or a wildcard set of polygons.
     * \param name is name of polygon to delete, may hold multiple wildcards (* or ?) to match
     * some or all polygons (regardless of origin.
     * \returns true of poly plugin says OK */
    //bool polygonDelete(char * name);
    /**
    Handle requests for road lines */
    bool handleRoad(UServerInMsg * msg, void * extra);
    /**
    Sent all available road lines */
    bool sendRoadLines(UServerInMsg * msg, int upds, double qual, bool bestOnly);
    /**
     * Make obstacle (all) groups to polygon plug-in.
     * \param maxCnt is the number of obstacle groups that are to be polygon'ed (from newest group).
     * \param newestUpdatesOnly copy polygons with most recent update timestamp only.
     * \param andFixed should the group of fixed obstacles be added too.
     * \returns true if polygon plugin resopnded positively. */
    bool makePoly(int maxCnt, bool newestUpdatesOnly, bool andFixed);

    protected:
    /**
    Laser pool resource pointer (is not created from this class) */
    ULaserPool * lasPool;
    /**
    Pose history resource pointer for odometry pose */
    UResPoseHist * odoHist;
    /**
    passable interval resource pointer */
    UResPassable * pass;
    /**
    Is the resource created locally */
    //bool passLocal;
    /**
    Last scan number from device N */
    unsigned long lastSerial[MAX_LASER_DEVS];
    /**
    Obstacle pool */
    UResObstacle * obsts;
    /**
    Is obstacle pool locally created */
    bool obstsLocal;
    /**
    Road line resource */
    UResRoadLine * roads;
    /**
    Is road resource created locally */
    //bool roadsLocal;

private:
    /**
    Function to get closest distance */
    bool handlePass(UServerInMsg * msg, void * extra);
    /**
    Send full scan with analysis statistics (for display purposes) */
    bool sendFullScan(UServerInMsg * msg, const char * tagName,
                      ULaserDevice * sick,    // sensor structure
                      ULaserScan * scan);      // scan with analysis data
};


#endif

