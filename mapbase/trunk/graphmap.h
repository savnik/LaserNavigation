/** \file graphmap.cpp
 *  \ingroup Mapping
 *  \brief Graphbased map server and route planning lib
 *
 *  Mapping library for interperting XML based graph-maps.
 *
 *  The lib provides functions for planning routes through the graph-map
 *  and for supplying geospacial-informations when navigating through the
 *  map.
 *
 *  \author Anders Billesø Beck
 *  $Rev: 1801 $
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

#include <expat.h>
#include <string>
#include <vector>
#include <stdio.h>

//LibDGL includex
#include "dglib/type.h"
#include "dglib/graph.h"

#include "mapcontainers.h"
#include "ExpatXMLParser.h"

#ifndef _GRAPHMAP_H
#define	_GRAPHMAP_H

//Define namespaces (both std and expatmm)
using namespace std;
using namespace expatmm;


/** Interface class used for callback to the libdgl shortest path clipper
 *
 *  This class must be inherited by the class that wishes to implement
 *  the clipper function. The functions must then overwritten in the classes
 *  that does the implementation
 */
class dglClipperCallback {
public:
    virtual ~dglClipperCallback() 
    {};
    /** Clipper function
     *
     * All node/connector/edge elements associated with the edge are passed into
     * the function to allow full evaluation of the edge.
     *
     * If the function returns true, the edge is discarded in the shortest
     * path calculations
     *
     */
    virtual bool shortestPathClipper (
                                     edge        *currEdge, /**< the edge to drive */
                                     edge        *prevEdge /**< the previous edge */
                                    ) {

       //default implementation always return false
        printf("Base clipper called\n");
       return false;

   }

};

//Graph mapping class. Implements ExpatXMLParser for C++ access to Expat
// (sorry about the virtual functions)
class graphmap : public ExpatXMLParser, public dglClipperCallback {
public:
    /* Graphmap constructor also calls ExpatXMLParser constructor  */
    graphmap();
    virtual ~graphmap();

    int loadMap(const char*);
    int calculateRoute(string origin, string destination, vector<route> *routeToDest);

    //Map element extraction functions
    node*       getNode(size_t);
    node*       getNode(const char*);
    edge*       getEdge(size_t);
    edge*       getEdge(const char*);
    connector*  getConnector(size_t);
    connector*  getConnector(const char*);
    connector*  getUniqueConnector(const char*);
    //Load function to load the clipper callback
    int loadClipper(dglClipperCallback *);

    //Public variables

protected:
   /* Functions to override ExpatXMLParser's functions */
   virtual ssize_t read_block(void);
   virtual void StartElement(const XML_Char *name, const XML_Char **atts);
   virtual void EndElement(const XML_Char *name);

public:

    //Containers for map data
    vector<edge> edges;             /**< Flattened container for all map edges */
    vector<node> nodes;             /**< Flattened container for all map nodes */
    bool maploaded;                 /**< Is any map loaded */
    string  mapname;                /**< Name of the graph map */


private:

    bool        verbose;            /**< Set true for alotta debug printouts */

    //Helper containers for graph map data
    vector<node> nodeTypes;         /**< Container for node types (used for parsing) */
    vector<edge> nodeTypeEdges;     /**< Container for edges defined within nodetypes (used for parsing) */
    vector<edge> edgeTypes;         /**< Container for edge types (used for parsig) */
    map<int,int> nodeidFromConnid;  /**< Index table for converting from connector id to node id */

    //Containers for libdgl
    dglGraph_s  dglgraph;
    dglClipperCallback *clipCallback;   /**< callback pointer for a function to implement the clipper callback */

    string  mapfile;
    string  mapBaseFilename;

    bool    mapChanged;
    //Definitions for each parser pass (unfortunately there is no way to
    //iterate through an enum in c++, so this has to do.
    // (used for paserPass in the struct below)
    static const int EDGETYPES  = 0;
    static const int NODETYPES  = 1;
    static const int NODES      = 2;
    static const int EDGES      = 2;
    static const int FINISHED   = 3;

    enum tag {NODETYPE, EDGETYPE, NODETYPEEDGE, EDGE , NODE, CONN};
    /** Structure for XML parsing information */
    struct  pinfo{
        int parserPass; /**< What pass is the parser performing */
        int passRead;
        int depth;  /**< Depth of XML Parsing */
        int skip;   /**< Skip further elements below this level */
        int found; /**< Is an element fitting the pass found */
        int connCount;
        int nodeDepth;
        int parentId[256];      /**< Supports a depth of 256 tags */
        enum tag tagType[256];  /**< Type of tag currently beeing processed */
        int currentNode;
    } parseInfo;

    string parseNodeName; /**< The dynamic node-name variable for parsing */
    string currentTypeName; /**< The current type-name in parsing */

    //internal processing functions
    int writeDGLGraph(void);
    int createDGLGraph(void);
    int executePlanner(int, int, vector<route> *);
    int executeDglShortestPath(int, int, vector<route> *);
    int getConnId(string);
    int getNodeId(int);
    int getNodeId(string);

    static int  shortestPathClipperGateway(dglGraph_s *,dglSPClipInput_s *,dglSPClipOutput_s *, void *);
    //int         shortestPathClipper(dglGraph_s *,dglSPClipInput_s *,dglSPClipOutput_s *);

    //Internal XML parsing functions
    void    addNode(const XML_Char *name, const XML_Char **atts);
    void    addEdge(const XML_Char *name, const XML_Char **atts);
    void    addConn(const XML_Char *name, const XML_Char **atts);
    void    parseProperties(const XML_Char *name, const XML_Char **atts);
    void    parseDimension(const XML_Char *name, const XML_Char **atts);
    void    featureParser(const XML_Char *name, const XML_Char **atts);
    int     postProcessMap(void);
	 pose		poseTransform(pose local, pose global);

};

#endif	/* _GRAPHMAP_H */

