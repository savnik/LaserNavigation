/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.206.                              *
 *   Contains map database						   *
 *   Edited by Peter Tjell (s032041) & Søren Hansen (s021751)              *
 *   Edited by Anders Billesø Beck (abb)
 * ----------------------------------------------------------------------- *
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
@author Christian Andersen
@editor Peter Tjell / Søren Hansen
@editor Anders Billesø Beck
*/
 
#ifndef URESMAPBASE_H
#define URESMAPBASE_H

#include <cstdlib>

# include <ugen4/uline.h>
# include <umap4/upose.h>
# include <math.h>       
# include <urob4/uresvarpool.h>
# include <ulms4/ulaserpool.h>
# include <urob4/uresposehist.h>
# include <utime.h>
# include <ugen4/ulock.h>

# include <expat.h>
# include <vector>

# include "graphmap.h"
# include "mapline.h"
# include "mappoint.h"

# define BUFFSIZE 1024       // Buffersize used by the XML reader

class UResMapbase : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResMapbase) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResMapbase();
  /**
  Destructor */
  virtual ~UResMapbase();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "mapbase"; };
  /**
  Print status to string preceding the string with this 'preString'.
  The target string is 'buff' with a maximum length of 'buffCnt'. */
  const char * print(const char * preString, char * buff, int buffCnt);
  
  /**
  The varPool has methods, and a call to one of these is needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed).
  * If the call is allowed, but the result is invalid for some reason, the
    return value 'value' can be set to som agreed value, (e.g. 0.0 (false) and 1.0 for true). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct = NULL,
                  int * returnStructCnt = NULL);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.


public: // Public functions
//  virtual bool setResource(UResBase * resource, bool remove);
  /**
   * Load a map with this filename - absolute, or relative to server default path
   * \param filename
   * \returns true if graph is loaded. */
  bool mapload(const char *);
  /**
   * Print graph to console */
  void display(void);
  /** Export the entire map in XML format to this string. */
  string exportMap(void);
  /**
   * Get pointer to the reference node for the map lines */
  node * getMapRefNode();
  /**
   * Get the reference pose for the map (tree) lines.
   * \Returns the pose defined in the node (current_node) referred in the map */
  UPose getMapRefPose();
  /**
   * Add a map-line to the mapdatabase.
   * \param name is the name of the line - must be unique, and is usualy a number.
   * \param startX,startY is the start position of the line - in current-node map coordinates
   * \param endX,endY is the end position of the line - in current-node map coordinates
   * \param radius is half the width of the line, as seen by a laserscanner (about 2 sigma) (sometimes also called 'perimeter'.
   * \param isObstacle if true, then this line is treated as a known obstacle, to avoid and suppress new obstacle detections.
   * Returns true if added. */
  bool addMapLine(const char * name,
                    double startX, double startY,
                    double endX, double endY,
                    double radius, bool isObstacle);

     
public: // Public variables
   //mapline lines[MAP_LENGTH];
   //mappoint points[MAP_LENGTH];
   vector<mapline> maplines;
   vector<mappoint> mappoints;
   int lines_cnt, points_cnt;
   graphmap graphmapper;

protected: // Protected functions:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();

protected: // Protected variables:

private:
   /** Recursive function to generate node informations in XML */
   string recursiveNodeExport(int, int);
   string exportEdge(void);
   string exportFeature(map<string,double>,int);
// Help functions for XML load using expat
  static void end(void *userData, const XML_Char *el);
  static void start(void *userData, const XML_Char *el, const XML_Char **attr);
  void line_parse(const char **);
  void point_parse(const char **);
  void add_startpoint(const char **);
  void add_endpoint(const char **);
  void add_point(const char **);
  char current_node[NAMELEN];
  UVariable *varConnMapPose;
};

#endif  //URESMAPBASE_H
