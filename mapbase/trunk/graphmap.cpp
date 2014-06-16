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
 *  $Rev: 59 $
 *  $Date: 2012-01-25 13:38:07 +0100 (Wed, 25 Jan 2012) $
 */
/***************************************************************************
 *                  Copyright 2009 Anders Billesø Beck, DTU                *
 *                       anbb@teknologisk.dk                            *
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
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>

#include "mapcontainers.h"
#include "graphmap.h"

/** Constructor
 * The graphmap constructor also calls the ExpatXMLParser constructor
 * to intialize XML buffer etc..
 */
graphmap::graphmap() : ExpatXMLParser() {

    //Clear libdgl graph structure
    memset(&dglgraph,0,sizeof(dglGraph_s));
    clipCallback = NULL;
    verbose = false;
    maploaded = false;
    for (int i = 0; i < 256; i++)
    {
      parseInfo.tagType[i] = NODETYPE;
      parseInfo.parentId[i] = 0;
    }
}

graphmap::~graphmap() {
}

/** Load a callback instance of the dglClipperCallback interface
 */
int graphmap::loadClipper(dglClipperCallback *loadClip) {

    if (loadClip != NULL) {
        clipCallback = loadClip;
        return 1;
    } else {
        clipCallback = NULL;
        return -1;
    }

}

/** Load a map from XML file.
 *
 * Loads a XML map and runs the XML parser to create internal mapping
 * representation
 * \returns 1 on success and -1 on error
 *
 */
int graphmap::loadMap(const char*mapfilename) {

    int returnValue = 1;
    size_t dotPlace;

    //Save mapfilename
    mapfile.assign(mapfilename);
    dotPlace = mapfile.find_last_of('.');
    if (dotPlace != mapfile.npos) {
        mapBaseFilename = mapfile.substr(0,dotPlace);

    } else {
        mapBaseFilename = mapfile;
    }
    
    //Reset parseInfo structure
    memset(&parseInfo,0,sizeof(pinfo));

    //Clear map-containing vectors
    nodeTypes.clear();
    nodes.clear();
    nodeTypeEdges.clear();
    edgeTypes.clear();
    edges.clear();

    //Run XML Parser through several passes (parserPass already = 0)
    for (; parseInfo.parserPass < FINISHED; parseInfo.parserPass++) {
        //Run the parser
        
        if (Parse() == false) {
            returnValue = -1;
            break;
        }
        resetParser();
    }

    //postprocess nodes to autogenerate interconnections
    postProcessMap();

    //Mark map as changed so a new libDGL file is generated
    mapChanged = true;
    writeDGLGraph();  //Create the libdgl binary files
    createDGLGraph(); //Create the libdgl internal graph

    maploaded = true;
 
    return returnValue;
}

/** Calculate shortest path to destination
 *
 * This function uses libDGL to calculate the shortest path from the
 * origin to destination input. origin must be given in the format 
 * node.connector, where destination can be either just the node-name
 * or in the node.connector format.
 *
 * The calculated route is placed in a route-vector. It is up to the caller, to
 * create the vector, but this function will clear the vector and allocate 
 * route-objects for each edge in the route.
 *
 * If routeplanning fails, the function returns -1, and the vector will be empty.
 *
 **/

int graphmap::calculateRoute(string origin, string destination, vector<route> *routeToDest) {

    int                 endId, tempId, planStartId;
    enum {CONN, NODE}   endType = CONN;
    //route               *tempRoute;
    struct timeval      tempTime;
    double              startTime, endTime;
    int                 routeLen;

    gettimeofday(&tempTime,NULL);
    startTime = (double)tempTime.tv_sec + (double)tempTime.tv_usec / 1000000.0;

  //Resolve connector Id's (return -1 on error)
  if ((planStartId = getConnId(origin)) < 0) {
      printf("Planning error: Start connector %s not found\n",origin.c_str());
      return -1;
  }
  if ((endId = getConnId(destination)) < 0) {
      endType = NODE;
      if ((endId = getNodeId(destination)) < 0) {
          printf("Planning error: End connector or node not found\n");
         return -1; 
      }
      
  }

    //If destination is input as node, then find closest node
    if (endType == NODE) {
        tempId = endId;
        int tempLen = 0, calcLen;
        for (size_t nConn = 0; nConn < nodes[tempId].connectors.size(); nConn++) {
            //Execute planner on all connectors for the node
            printf("Testing %d\n",nConn);
            calcLen = executeDglShortestPath(planStartId,nodes[tempId].connectors[nConn].id,routeToDest);
            if ((calcLen < tempLen) || (tempLen <= 0)) { //Find shortst dist
                tempLen = calcLen;
                endId = nodes[tempId].connectors[nConn].id;  
            }
        }
    }

    //Then execute planner to create final route
    //routeLen = executePlanner(startId,endId,routeToDest);
    routeLen = executeDglShortestPath(planStartId,endId,routeToDest);

    printf("Planning route from %d to %d dist %d\n",planStartId,endId,routeLen);
    printf("Extracted %d route elements\n",routeToDest->size());

    gettimeofday(&tempTime,NULL);
    endTime = (double)tempTime.tv_sec + (double)tempTime.tv_usec / 1000000.0;

    printf("Time spent routeplanning %5.3f ms\n",(endTime-startTime)*1000);
    return routeLen;
}

int graphmap::executePlanner(int startConn, int endConn, vector<route> *routeToDest) {

    char buf[1024];
    FILE *pipeFile;
    int totalDistance, routeLen;
    route *tempRoute;
    char *retVal;

  //Calcluate path
  sprintf(buf,"./shortest_path -g %s.dgl -f %d -t %d",mapBaseFilename.c_str(),startConn,endConn);
  pipeFile = popen(buf,"r");
  while (!feof(pipeFile)) {
    retVal = fgets(buf,1024,pipeFile);
    if (feof(pipeFile)) break;

    if (strncmp("shortest path:",buf,strlen("shortest path:")) == 0) {

    } else if (strncmp("shortest path report:",buf,strlen("shortest path report:")) == 0) {
        sscanf(buf,"shortest path report: total edges %d - total distance %d",&routeLen,&totalDistance);
        printf("Planning route of %d mm along %d edges\n",totalDistance,routeLen);

        //Clear route-vector if a valid route has been planned
        routeToDest->clear();

    } else if (strncmp("edge",buf,strlen("edge")) == 0) {
      //Create new route element
        tempRoute = new route;

      sscanf(buf,"edge[%d]: from %d to %d - travel cost %d - user edgeid %d - distance from start node %d",
           &tempRoute->id, &tempRoute->startConnId, &tempRoute->endConnId, &tempRoute->edgeLength,
           &tempRoute->edgeId, &tempRoute->totalDist);

      //Copy the edge
      tempRoute->routeEdge.copy(edges[tempRoute->edgeId]);
      //Save copies of start and end nodes and connectors
      for (size_t nNode = 0; nNode < nodes.size(); nNode++) {
          for (size_t nConn = 0; nConn < nodes[nNode].connectors.size(); nConn++) {
              if (nodes[nNode].connectors[nConn].id == tempRoute->startConnId) {
                  tempRoute->startNode.copy(nodes[nNode]);
                  tempRoute->startConn.copy(nodes[nNode].connectors[nConn]);
                  tempRoute->startName = tempRoute->startNode.name;
                  tempRoute->startName.append(".");
                  tempRoute->startName.append(tempRoute->startConn.name);
                  tempRoute->startNodeId = nNode;
                  tempRoute->startConnId = nConn;
              } else if (nodes[nNode].connectors[nConn].id == tempRoute->endConnId) {
                  tempRoute->endNode.copy(nodes[nNode]);
                  tempRoute->endConn.copy(nodes[nNode].connectors[nConn]);
                  tempRoute->endName = tempRoute->endNode.name;
                  tempRoute->endName.append(".");
                  tempRoute->endName.append(tempRoute->endConn.name);
                  tempRoute->endNodeId = nNode;
                  tempRoute->endConnId = nConn;
                  
              }
          }
      }

      routeToDest->push_back(*tempRoute);
      delete tempRoute;


    } else if (strncmp("destination node is unreachable",buf,strlen("destination node is unreachable")) == 0){
      totalDistance = -1;
      printf("Graphplanner: Destination is unreachable\n");
      break;
    }
  }
  pclose(pipeFile);

  return totalDistance;
}

/** Use libdgl to calculate shortest path between two connectors
 *
 */

int graphmap::executeDglShortestPath(int startConn, int endConn, vector<route> *routeToDest) {

    dglSPReport_s * pReport;
    dglSPCache_s    spCache;
    int             nret;
    route           *tempRoute;

    dglInitializeSPCache( & dglgraph, & spCache );
    
    nret = dglShortestPath( & dglgraph , & pReport , startConn , endConn , graphmap::shortestPathClipperGateway, this , &spCache );
    if (nret == 0) {
        routeToDest->clear();
        printf("libdgl: Destination unreachable\n");
        return nret;
    } else if (nret < 0) {
        printf("libdgl ShortestPath error: %s\n",dglStrerror( & dglgraph ));
        routeToDest->clear();
        return nret;
    } else {
        routeToDest->clear();

        

        //Loop through the planned route elements and extract informations to route element class
        for (int i = 0; i < pReport->cArc; i++) {
            tempRoute = new route;

            //Extract informations from pReport struct
            tempRoute->id           = i;
            tempRoute->startConnId  = pReport->pArc[i].nFrom;
            tempRoute->endConnId    = pReport->pArc[i].nTo;
            tempRoute->edgeLength   = dglEdgeGet_Cost(&dglgraph, pReport->pArc[i].pnEdge);
            tempRoute->edgeId       = dglEdgeGet_Id(&dglgraph, pReport->pArc[i].pnEdge);
            tempRoute->totalDist    = pReport->pArc[i].nDistance;

            //Copy the edge
            tempRoute->routeEdge.copy(edges[tempRoute->edgeId]);
            //Save copies of start and end nodes and connectors
            for (size_t nNode = 0; nNode < nodes.size(); nNode++) {
                for (size_t nConn = 0; nConn < nodes[nNode].connectors.size(); nConn++) {
                    if (nodes[nNode].connectors[nConn].id == tempRoute->startConnId) {
                        tempRoute->startNode.copy(nodes[nNode]);
                        tempRoute->startConn.copy(nodes[nNode].connectors[nConn]);
                        tempRoute->startName = tempRoute->startNode.name;
                        tempRoute->startName.append(".");
                        tempRoute->startName.append(tempRoute->startConn.name);
                        tempRoute->startNodeId = nNode;
                        tempRoute->startConnId = nConn;
                    } else if (nodes[nNode].connectors[nConn].id == tempRoute->endConnId) {
                        tempRoute->endNode.copy(nodes[nNode]);
                        tempRoute->endConn.copy(nodes[nNode].connectors[nConn]);
                        tempRoute->endName = tempRoute->endNode.name;
                        tempRoute->endName.append(".");
                        tempRoute->endName.append(tempRoute->endConn.name);
                        tempRoute->endNodeId = nNode;
                        tempRoute->endConnId = nConn;

                    }
                }
            }

            routeToDest->push_back(*tempRoute);
            delete tempRoute;
        }
    }

    return pReport->nDistance; //Return the total distance

}


/** Generate Graph file for libDGL and convert it to binary format
 *
 */
int graphmap::writeDGLGraph(void) {

    string  tempDGLFilename;                    //Create filename.txt
    FILE    *mfp, *pipeFile;                    //Filepointers
    char    buf[1024], tempStr[128];            //Char buffers
    char    *retVal;


    tempDGLFilename = mapBaseFilename + ".txt";
    
    
    //Only generate map, if it has changed
    if (!mapChanged) {
        printf("No changes in map\n");
        return 0;
    } else {
        //Clear change-flag
        mapChanged = 0;
    }

    mfp = fopen(tempDGLFilename.c_str(),"w");
    if (mfp == NULL) {
        printf("Error opening libDGL file: %s for writing",tempDGLFilename.c_str());
        return -1;
    }

  // *** Write ASCII libDGL graph map ***
  //Write standard header stuff into the file
  fprintf(mfp,"#******************************************************************\n");
  fprintf(mfp,"#* This map for libDGL is generated by RobotManager MapModule %s *\n",VERSION);
  fprintf(mfp,"#******************************************************************\n");
  fprintf(mfp,"# GRAPH VERSION AND NODE-ATTRIBUTE SIZE (3*32bit words for X Y and Z in bytes)\n");
  fprintf(mfp,"2 12\n\n");
  fprintf(mfp,"# Edges (ARCS) Connections between nodes\n");
  fprintf(mfp,"#    FROM    TO     COST   ARCID\n");

    //Then process all edges
    for (size_t nEdge = 0; nEdge < edges.size(); nEdge++) {
        //Evaluate if edge is blocked or not
        if (!edges[nEdge].blocked && !edges[nEdge].disabled) {
            //Write edges to file (length in mm for int-resolution)
            fprintf(mfp,"A   %5d %5d %8d %5d",edges[nEdge].startConnId,edges[nEdge].endConnId,
                  (int)(edges[nEdge].length * 1000),nEdge);
            fprintf(mfp,"   #%s.%s -> %s.%s \n",edges[nEdge].startNodeName,edges[nEdge].startConnName, edges[nEdge].endNodeName,edges[nEdge].endConnName);
        } else {
            edges[nEdge].blocked = false;
        }
    }

    //Write node header
    fprintf(mfp,"\n# Node X Y Z coordinates\n#\n");
    fprintf(mfp,"# NODEID       X       Y        Z\n#\n");

    //Print nodes and position (in mm)
    for (size_t nNode = 0; nNode < nodes.size(); nNode++) {
        for (size_t nConn = 0; nConn < nodes[nNode].connectors.size(); nConn++) {
            fprintf(mfp,"N %6d %7d %7d        0",nodes[nNode].connectors[nConn].id, (int)(nodes[nNode].x*1000),(int)(nodes[nNode].y*1000));
            fprintf(mfp," #%s.%s \n",nodes[nNode].name,nodes[nNode].connectors[nConn].name);
        }
    }

    fprintf(mfp,"\n#***  RobotManager done writing %d edges and %d nodes  ***\n",edges.size(),nodes.size());
    fclose(mfp); //Close mapfile


    // *** Create Binary DGL map using cr_from_a program from libDGL ***
    memset(buf,0,1024);
    //Create system command to cr_from_a
    sprintf(tempStr,"./cr_from_a -f %s -g %s.dgl",tempDGLFilename.c_str(),mapBaseFilename.c_str());
    pipeFile = popen(tempStr,"r"); //Execute
    while (!feof(pipeFile)) {
        retVal = fgets(buf,1024,pipeFile);
        if (strcmp("",buf) != 0) {
            printf("Error generating binary graphmap: %s\n",buf);
            return -1;
        }
    }
    pclose(pipeFile);

    return 1;
    
}

/** Create internal libdgl graph representation using libdgl directly
 *
 */
int graphmap::createDGLGraph(void) {

    int		nret;
    const int 	version = 2; //default map-version
    dglInt32_t	nodeid , from , to , cost , edgeid;
    //No idea, what this below is used for... (stolen from dglib example)
    dglInt32_t	opaqueset[ 16 ] = {
		360000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //If the graph is not cleared, release it before use
    if (dglgraph.Version != 0) {
        dglRelease(&dglgraph);
    }
    
    //initialize the libdgl structure
    dglInitialize (& dglgraph ,
                   version,         // Graph version (default 2)
                   0,               // node attributes size
		   0,               // edge attributes size
		   opaqueset);      // opaque graph parameters (No idea what that does?)

    //generate edge cost prioritizing
    dglSet_Options( & dglgraph, DGL_GO_EdgePrioritize_COST );

    //Start by creating nodes (actually connectors)
    for (size_t nNode = 0; nNode < nodes.size(); nNode++) {
        for (size_t nConn = 0; nConn < nodes[nNode].connectors.size(); nConn++) {
            nodeid = nodes[nNode].connectors[nConn].id;
            nret = dglAddNode( & dglgraph , nodeid , NULL, 0 );

            //Error check
            if ( nret < 0 ) {
		printf("dglAddNode error: %s\n" , dglStrerror( & dglgraph ) );
            }
        }
    }

    //Then add all the edges
    for (size_t nEdge = 0; nEdge < edges.size(); nEdge++) {
        from = edges[nEdge].startConnId;
        to = edges[nEdge].endConnId;
        cost = (unsigned int) (edges[nEdge].length * 1000); //length in mm
        edgeid = nEdge;
        nret = dglAddEdge(& dglgraph ,from ,to ,cost ,edgeid);

        //Error check
        if ( nret < 0 ) {
            printf("dglAddNode error: %s\n" , dglStrerror( & dglgraph ) );
        }
    }

    printf("   Graphmap: Created DGL-Graph with %d connectors and %d edges\n",dglGet_NodeCount(&dglgraph),dglGet_EdgeCount(&dglgraph));


    return 1;
}

/** Gateway function used as static accesspoint for the shortest path clipper function
 *
 */
int  graphmap::shortestPathClipperGateway(
						dglGraph_s * 	pgraph ,
						dglSPClipInput_s * pIn ,
						dglSPClipOutput_s * pOut ,
						void * 	pvarg
						) {
    //Typecast the pvarg pointer to graphmap object, and call the
    //real clipper function
    graphmap *gm = (graphmap *) pvarg;
    edge *currEdge = &(gm->edges[dglEdgeGet_Id(pgraph, pIn->pnEdge)]);
    edge *prevEdge;
    //Send a null pointer, if this is the first edge. Otherwise send the edge pointer
    if (pIn->pnPrevEdge == NULL){
       prevEdge = NULL;
    } else {
       prevEdge = &(gm->edges[dglEdgeGet_Id(pgraph, pIn->pnPrevEdge)]); 
    }
    
    //Call the defalut clipper if no callback is loaded, otherwise call the callback
    if (gm->clipCallback == NULL)
       return (int)gm->shortestPathClipper(currEdge,prevEdge);
    else {
        return (int)gm->clipCallback->shortestPathClipper(currEdge,prevEdge);
    }

}


/** Return a pointer to the node matching the index
 *
 */
node* graphmap::getNode(size_t nodeIndex) {

    if (nodeIndex >= nodes.size()) return NULL;
    else {
        return &(nodes[nodeIndex]);
    }

}

/** Return the node matching the name
 *
 */
node* graphmap::getNode(const char* nodeName) {

  const char *p1;
    for (size_t i = 0; i < nodes.size(); i++)
    {
      // debug
      p1 = nodes[i].name;
      // debug end
      if (!strcmp(nodes[i].name,nodeName)) {
          return &(nodes[i]);
      }
    }

    //If no match is found, return null
    return NULL;

}

/** Return a pointer to the edge matching the index
 *
 */
edge* graphmap::getEdge(size_t edgeIndex) {

    if (edgeIndex >= edges.size()) return NULL;
    else {
        return &(edges[edgeIndex]);
    }


}

/** Return a pointer to the edge matching the name
 *
 */
edge* graphmap::getEdge(const char* edgeName) {

    for (size_t i = 0; i < edges.size(); i++) {
        if (!strcmp(edges[i].name,edgeName)) {
            return &(edges[i]);
        }
    }

    //If no match is found, return null
    return NULL;


}

/** Return a pointer to the connector matching the index
 *
 */
connector*  graphmap::getConnector(size_t connIndex) {

    for(size_t i = 0; i < nodes.size(); i++) {
        for (size_t j = 0; j < nodes[i].connectors.size();j++) {
            if ((size_t)nodes[i].connectors[j].id == connIndex) {
                return &(nodes[i].connectors[j]);
            }
        }
    }

    //If no match is found, return null
    return NULL;

}


/** Return a pointer to the connector matching the name
 *
 */
connector*  graphmap::getConnector(const char* connName) {

    string tempName, tempName2;

    for(size_t i = 0; i < nodes.size(); i++) {
        tempName.assign(nodes[i].name);
        tempName.append(".");
        for (size_t j = 0; j < nodes[i].connectors.size();j++) {
            tempName2 = tempName;
            tempName2 += nodes[i].connectors[j].name;
            if (!strcmp(tempName2.c_str(),connName)) {
                return &(nodes[i].connectors[j]);
            }
        }
    }

    //If no match is found, return null
    return NULL;
}

/** Return a pointer to the connector matching the name
 *
 */
connector*  graphmap::getUniqueConnector(const char* uniqueconnName) {

    for(size_t i = 0; i < nodes.size(); i++) {
        for (size_t j = 0; j < nodes[i].connectors.size();j++) {
            if (!strcmp(nodes[i].connectors[j].name,uniqueconnName)) {
                return &(nodes[i].connectors[j]);
            }
        }
    }

    //If no match is found, return null
    return NULL;
}

/** Get connector id
 *
 * This function returns the connector id, based an input string
 * of for format "node.connector" name.
 */
int graphmap::getConnId(string positionName) {

    //Extract node and connector names from input-string
    string nodeName = positionName.substr(0,positionName.find_last_of("."));
    string connName = positionName.substr(positionName.find_last_of(".")+1);
    int returnValue = -1;

    //Search for the connector Id by looping through nodes and connectors
    for (size_t nNode = 0; ((nNode < nodes.size()) && (returnValue < 0)); nNode++) {
        if (!nodeName.compare(nodes[nNode].name)) {
            for (size_t nConn = 0; nConn < nodes[nNode].connectors.size(); nConn++) {
                if (!connName.compare(nodes[nNode].connectors[nConn].name)) {
                    //Save connector Id when the right connector is found
                    returnValue = nodes[nNode].connectors[nConn].id;
                    break;
                }
            }
        }
    }
    return returnValue;
}

/** Get node id
 *
 * This function returns the node-id associated with the input connector Id
 */
int graphmap::getNodeId(int connId) {

    //TODO: Write getNodeId function

    return 1;

}

/** Get node id
 *
 * This function returns the node-id matching the name (overloaded)
 */
int graphmap::getNodeId(string nodeName) {

    int rValue = -1;

    for (size_t nNode = 0; nNode < nodes.size(); nNode++) {
        if (!nodeName.compare(nodes[nNode].name)) {
            rValue = nNode;
            break;
        }
    }

    return rValue;
}

/** Pose transformation function
 *
 * Returned pose is the local pose projected into the
 * global pose coordinate system
 */
pose graphmap::poseTransform(pose local, pose global) {

	pose trans;

	//Transform coordinates
	trans.x = cos(global.thr)*local.x - sin(global.thr)*local.y + global.x;
	trans.y = sin(global.thr)*local.x + cos(global.thr)*local.y + global.y;
	trans.thr = local.thr + global.thr;

	//Limit angle to +/-Pi
	if      (trans.thr > M_PI)  trans.thr -= 2*M_PI;
	else if (trans.thr < -M_PI) trans.thr += 2*M_PI;

	//Calculate angle in degrees
	trans.th = trans.thr * (180.0 / M_PI);

	//Return the new pose
	return trans;

}

/*******************************************************************************
 *                         XML Map Parsing functions                           *
 *******************************************************************************/


/** Read block of XML data
 *
 * This handler functions overwrites the read-block from ExpatXMLParser
 *
 * The XML file is read fully in each block. As the map is read in several
 * passes, this read block increments the pass-counter and reads the full
 * map file until all passes has been finished.
 *
 */
ssize_t graphmap::read_block(void) {

    FILE *fp;
    ssize_t len;
    int xmlFileLen;
    XML_Char *xmlBuf = getBuffer();

    if (verbose) printf("Running parser pass: %d\n",parseInfo.parserPass);

    //Open and read the XML file
    fp = fopen(mapfile.c_str(),"r");
    if(fp == NULL) {
      printf("   Graphmap: Error reading file: %s\n",mapfile.c_str());
      return -1;
    }
    //Get the length of the file
//     fseek(fp,0,SEEK_END);
    xmlFileLen = ftell(fp); //Get position
    fseek(fp,0,SEEK_SET); //Return to start of file

    //Read XML file into buffer
    // memset(xmlBuf,0,xmlFileLen+10);
    len = fread(xmlBuf, 1, getBlockSize(), fp);
    fclose(fp);

    setLastError(XML_ERROR_FINISHED); //Set XML file finished to Expat

    return len; //Return the file size
}


/** XML Start tag handler
 *
 * This handler functions overwrites the default handlers from ExpatXMLParser
 *
 */
void graphmap::StartElement(const XML_Char *name, const XML_Char **atts) {

    node        *tempNode;
    edge        *tempEdge;

 //Increment XML Depth
    parseInfo.depth++;

    //printf("Tag %s in depth %d\n",name,parseInfo.depth);

 //Check for the right tag
  if ((parseInfo.depth > 1) && !parseInfo.found && !parseInfo.skip) {

    //Only parse the right types in each pass through the file
    //First pass detect types
    if ((parseInfo.parserPass == NODETYPES) && (strcmp("nodetype",name) == 0)) {
        parseInfo.found = parseInfo.depth;
        if (verbose) printf("Found nodetype: %s\n",name);
    //Second pass detect nodes
    } else if ((parseInfo.parserPass == EDGETYPES) && (strcmp("edgetype",name) == 0)) {
        parseInfo.found = parseInfo.depth;
        if (verbose) printf("Found edgetype: %s\n",name);
    //Third pass detect nodes
    } else if ((parseInfo.parserPass == NODES) && (strcmp("node",name) == 0)) {
        parseInfo.found = parseInfo.depth;
        if (verbose) printf("Found node: %s\n",name);
    //Fourth pass detect edges
    } else if ((parseInfo.parserPass == EDGES) && (strcmp("edge",name) == 0)) {
        parseInfo.found = parseInfo.depth;
        if (verbose) printf("Found edge: %s\n",name);
    //Skip everything elese, that does not fit the current pass
    } else {
      parseInfo.skip = parseInfo.depth;
      return;
    }
    //Save map name
  } else if ((parseInfo.depth <= 1) && (strcmp("graph",name) == 0)) {
        //find and save mapname
        for(int i = 0; atts[i]; i+=2) if (strcmp("name",atts[i]) == 0) {
            mapname = atts[i+1];
            if((parseInfo.parserPass == NODES) || verbose)
                printf("   Graphmap: Loading graph map: %s\n",mapname.c_str());
        }
        return;
    //Exit if parser is skipping...
  } else if (parseInfo.skip > 0) return;

  //

  /* Tag parser
   * Only tags in the correct context should have passed to here  */
    if (strcmp("nodetype",name) == 0) {
        tempNode = new node();

        //Find type name
        for(int i = 0; atts[i]; i+=2) if (strcmp("name",atts[i]) == 0) {
            strncpy(tempNode->name,"",128); //Clear name on top-nodetype
            strncpy(tempNode->typeName,atts[i+1],128);
            //Clear node-string and prepare it to define nodes through addNode function
            parseNodeName.clear();
            currentTypeName.assign(atts[i+1]);
        }

        //Maintain node-parsing depth counters
        parseInfo.currentNode = nodes.size();
        parseInfo.parentId[parseInfo.depth] = nodeTypes.size();
        parseInfo.tagType[parseInfo.depth] = NODETYPE;
        tempNode->depth = parseInfo.depth;

        //Add node type to the end of the type-vector storage
        nodeTypes.push_back(*tempNode);
        delete tempNode;
        
        if (verbose) printf("Parsed nodetype: %s\n",nodeTypes.back().name);

    } else if (strcmp("edgetype",name) == 0) {
        tempEdge = new edge();

        for(int i = 0; atts[i]; i+=2) if (strcmp("name",atts[i]) == 0) {
            strncpy(tempEdge->typeName,atts[i+1],128);
        }

        //Keep track on depth-counters
        parseInfo.parentId[parseInfo.depth] = edgeTypes.size();
        parseInfo.tagType[parseInfo.depth] = EDGETYPE;

        //Add node type to the end of the type-vector storage
        edgeTypes.push_back(*tempEdge);
        delete tempEdge;

        if (verbose) printf("Added edgetype: %s\n",edgeTypes.back().typeName);

    } else if (strcmp("node",name) == 0) {
        addNode(name,atts); //Add node
    } else if (strcmp("edge",name) == 0) {
        addEdge(name,atts); //Call the edgeparser function
    } else if (strcmp("connector",name) == 0) {
        addConn(name,atts); //Call connector parser function
    } else if (strcmp("properties",name) == 0) { //Parse properties tag for edge/edgetype
        parseProperties(name,atts);
    } else if (strcmp("dimension",name) == 0) { //Parse properties tag for edge/edgetype
        parseDimension(name,atts);
    } else {
        //Tags not parsed above is considered a feature to the given type/edge/node
        featureParser(name,atts);
    }




}

/** XML End tag handler
 *
 * This handler functions overwrites the default handlers from ExpatXMLParser
 *
 */
void graphmap::EndElement(const XML_Char *name) {

    //Decrement XML depth again
    parseInfo.depth--; 
    
    //Stop skipping when depth is back below skip-depth
    if (parseInfo.skip > parseInfo.depth) {
        parseInfo.skip = 0;
    } else if (parseInfo.found > parseInfo.depth) {
        parseInfo.found = 0;
    }

    //Exit end-parser if skipping
    if (parseInfo.skip) return;
    
    //Detect end of "edge" tag
    if (!strcmp("edge",name)) {
        //create reverse path if edge is not directional
        if (((parseInfo.tagType[parseInfo.depth+1] == EDGE) && 
              !edges[parseInfo.parentId[parseInfo.depth+1]].directional) 
              ||
             ((parseInfo.tagType[parseInfo.depth+1] == NODETYPEEDGE) && 
               !nodeTypeEdges[parseInfo.parentId[parseInfo.depth+1]].directional)) {
            
            edge *newEdge = new edge();
            edge *tempEdge = new edge();

            //Copy all informations from edge or nodeTypeEdge
            if (parseInfo.tagType[parseInfo.depth+1] == EDGE) {
               newEdge->copy(edges[parseInfo.parentId[parseInfo.depth+1]]);
               tempEdge->copy(edges[parseInfo.parentId[parseInfo.depth+1]]);
            } else if (parseInfo.tagType[parseInfo.depth+1] == NODETYPEEDGE) {
               newEdge->copy(nodeTypeEdges[parseInfo.parentId[parseInfo.depth+1]]);
               tempEdge->copy(nodeTypeEdges[parseInfo.parentId[parseInfo.depth+1]]);
            }  

            //Invert direction
            strcpy(newEdge->start,tempEdge->end);
            strcpy(newEdge->end,tempEdge->start);


            if (verbose) printf("Created reversed edge from %s to %s\n",newEdge->start,newEdge->end);

            if (parseInfo.tagType[parseInfo.depth+1] == EDGE) {
                edges.push_back(*newEdge);
            } else if (parseInfo.tagType[parseInfo.depth+1] == NODETYPEEDGE) {
                nodeTypeEdges.push_back(*newEdge);
            }
            delete newEdge;
            delete tempEdge;
        }
    } else if (!strcmp("node",name)) {

        //Remove last node-bit of parsenodename when ending node-tag
        if (parseNodeName.find_last_of('.') == parseNodeName.npos) parseNodeName.clear();
        else parseNodeName.erase(parseNodeName.find_last_of('.'));
    }

}

/** Process node tag
 *
 * This function processes a new node, using the hierarchical
 * node-structure.
 **/
void graphmap::addNode(const XML_Char *name, const XML_Char **atts) {

    node *newNode = new node();
    edge *newEdge;
    string foundTypeName;
    string tempName;
    size_t topId = 0;
    double  x = -1,y = -1,th = -1, thr = -1;

    //Assign default values
    newNode->x = 0.0;
    newNode->features["x"] = 0.0;
    newNode->y = 0.0;
    newNode->features["y"] = 0.0;
    newNode->th = 0.0;
    newNode->features["th"] = 0.0;
	 newNode->thr = 0.0;
    newNode->features["thr"] = 0.0;

    //Extract node-name and add it the the <.>-notation name-string
    for(int i = 0; atts[i]; i+=2) {
        if (strcmp("name",atts[i]) == 0) {
            //Add "." to node name, if it is not empty
            if (!parseNodeName.empty()) parseNodeName.append(".");
            parseNodeName.append(atts[i+1]); //Then add the new name
        } if (strcmp("x",atts[i]) == 0) {
            //Parse physical parameters
            x = strtod(atts[i+1],NULL);
        } if (strcmp("y",atts[i]) == 0) {
            //Parse physical parameters
            y = strtod(atts[i+1],NULL);
        } if (strcmp("th",atts[i]) == 0) {
            //Parse physical parameters
            th = strtod(atts[i+1],NULL);
				thr = th * (M_PI / 180);
        } if (strcmp("thr",atts[i]) == 0) {
            //Parse physical parameters
            thr = strtod(atts[i+1],NULL);
				th = thr * (180 / M_PI);

        }

    }

    //Are we parsing types or real-nodes
    if (parseInfo.parserPass == NODETYPES) {

        //Save typename and nodename
        strcpy(newNode->typeName,currentTypeName.c_str());
        strcpy(newNode->name,parseNodeName.c_str());

        //Save physical parameters
        if (x != -1)  {
            newNode->x = x;
            newNode->features["x"] = x;
        }
        if (y != -1) {
            newNode->y = y;
            newNode->features["y"] = y;
        }
        if (th != -1) {
            newNode->th = th;
            newNode->features["th"] = th;
				newNode->thr = thr;
				newNode->features["thr"] = thr;

        }

        //Keep depth-counter up-to-date
        parseInfo.tagType[parseInfo.depth] = NODETYPE;
        parseInfo.parentId[parseInfo.depth] = nodeTypes.size();
        
        nodeTypes.push_back(*newNode);
        if (verbose) printf("Added node: %s to type: %s\n",newNode->name,newNode->typeName);
        delete newNode; //Cleanup

    } else if (parseInfo.parserPass == NODES) {

        //Start by checking for type inheritance for the node
        for(int i = 0; atts[i]; i+=2) {
            if (!strcmp("type",atts[i])) {
                foundTypeName.assign(atts[i+1]); //Save typename
            }
        }

        //Inherit from top-type-node, if any type is defined
        if (!foundTypeName.empty()) {
            for (topId = 0; topId < nodeTypes.size(); topId++) {
                //Search for the type
                if (!strcmp(nodeTypes[topId].typeName,foundTypeName.c_str())) {
                    //Top nodeType is the first match
                    newNode->copy(nodeTypes[topId]);
                    break;
                }
            }
        }

        //Save name
        strcpy(newNode->name,parseNodeName.c_str());
        //Save physical parameters
        if (x != -1)  {
            newNode->x = x;
            newNode->features["x"] = x;
        }
        if (y != -1) {
            newNode->y = y;
            newNode->features["y"] = y;
        }
        if (th != -1) {
            newNode->th = th;
            newNode->features["th"] = th;
				newNode->thr = thr;
				newNode->features["thr"] = thr;

        }

        //Keep depth-counter up-to-date
        parseInfo.tagType[parseInfo.depth] = NODE;
        parseInfo.parentId[parseInfo.depth] = nodes.size();

        if (verbose) printf("Added node: %s (inheritance? : %s)\n",newNode->name,newNode->typeName);
        newNode->features["id"] = nodes.size();

        nodes.push_back(*newNode);
        delete newNode; //Cleanup

        //If a type is detected, then add all sub-nodes and edges from the type
        if (!foundTypeName.empty()) {
            //Transfer all sub-nodes
            for (size_t n = 0; n < nodeTypes.size(); n++) {
                //Find any nodes in type (but NOT the top-one)
                if ((n != topId) && !strcmp(nodeTypes[n].typeName,foundTypeName.c_str())) {
                    newNode = new node();
                    newNode->copy(nodeTypes[n]); //Inherith all from type

                    //Adjust name, by adding the parseNodeName to the type-name
                    tempName.assign(parseNodeName);
                    if (!tempName.empty()) tempName.append(".");
                    tempName.append(nodeTypes[n].name);
                    strcpy(newNode->name,tempName.c_str());
                    tempName.clear();

                    if (verbose) printf("Added node %d : %s from type %s\n",nodes.size(),newNode->name,foundTypeName.c_str());
                    nodes.push_back(*newNode);
                    delete newNode;
                }
            }
            //Then sub-edges
            for (size_t n = 0; n < nodeTypeEdges.size(); n++) {
                if (!strcmp(nodeTypeEdges[n].typeName,foundTypeName.c_str())) {
                    newEdge = new edge();
                    newEdge->copy(nodeTypeEdges[n]); //Inherith all from type

                    //Adjust name, by adding the parseNodeName to the type-name
                    tempName.assign(parseNodeName);
                    if (!tempName.empty()) tempName.append(".");
                    tempName.append(nodeTypeEdges[n].name);
                    strcpy(newEdge->name,tempName.c_str());
                    tempName.clear();

                    //Then adjust start end end-name
                    tempName.assign(parseNodeName);
                    if (!tempName.empty()) tempName.append(".");
                    tempName.append(nodeTypeEdges[n].start);
                    strcpy(newEdge->start,tempName.c_str());
                    tempName.clear();

                    tempName.assign(parseNodeName);
                    if (!tempName.empty()) tempName.append(".");
                    tempName.append(nodeTypeEdges[n].end);
                    strcpy(newEdge->end,tempName.c_str());
                    tempName.clear();

                    if (verbose) printf("Added edge between %s and %s from type %s\n",newEdge->start, newEdge->end,newEdge->typeName);
                    edges.push_back(*newEdge);
                    delete newEdge;
                }
            }
        }


    } //End of node-part

}


/** Process edge tag
 *
 * Processing of an edge-tag is so extensive, that is is moved to a
 * seperate function
 **/
void graphmap::addEdge(const XML_Char *name, const XML_Char **atts) {


       edge     *tempEdge = new edge();
       string   tempName;
       string   foundType;


        //Loop through edge attributes
        for(int i = 0; atts[i]; i+=2) {
            if (!strcmp("type",atts[i])) {
                foundType.assign(atts[i+1]); //Save type, if it is found
            } 
        }

        //Inherith from type, if it is existing
       if (!foundType.empty()) {
           for (size_t n = 0; n < edgeTypes.size(); n++) {
               if (!strcmp(edgeTypes[n].typeName,foundType.c_str())) {
                   tempEdge->copy(edgeTypes[n]); //Copy from type
               }
           }
       }

        //Loop through edge attributes and write them to edge class
        for(int i = 0; atts[i]; i+=2) {
            if (!strcmp("name",atts[i])) {
                strncpy(tempEdge->name,atts[i+1],128);
            } else if (!strcmp("start",atts[i])) {
                strncpy(tempEdge->start,atts[i+1],128);
            }else if (!strcmp("end",atts[i])) {
                strncpy(tempEdge->end,atts[i+1],128);
            } else if (!strcmp("type",atts[i])) {
                foundType.assign(atts[i+1]); //Save type, if it is found
            } else if (!strcmp("length",atts[i])) {
                tempEdge->length = strtod(atts[i+1],NULL);
                tempEdge->features["length"] = tempEdge->length;
            } else if (!strcmp("directional",atts[i])) {
                tempEdge->directional = atoi(atts[i+1]);
                tempEdge->features["directional"] = tempEdge->directional;
            }

        }

       //Add add node-name to edges to fit to depth
       //Start-name
       tempName.assign(parseNodeName);
       if (!tempName.empty()) tempName.append(".");
       tempName.append(tempEdge->start);
       strcpy(tempEdge->start,tempName.c_str());
       tempName.clear();
       //end-name
       tempName.assign(parseNodeName);
       if (!tempName.empty()) tempName.append(".");
       tempName.append(tempEdge->end);
       strcpy(tempEdge->end,tempName.c_str());

        //If edges is placed inside types, they go to nodeTypeEdges
        if (parseInfo.parserPass == NODETYPES) {
            //Keep depth-counter up-to-date
           parseInfo.tagType[parseInfo.depth] = NODETYPEEDGE;
           parseInfo.parentId[parseInfo.depth] = nodeTypeEdges.size();

           //Set typename
           strcpy(tempEdge->typeName,currentTypeName.c_str());

           if (verbose) printf("Parsed NodeTypeEdge from type %s between %s to %s\n",tempEdge->typeName,tempEdge->start,tempEdge->end);

           nodeTypeEdges.push_back(*tempEdge);
           delete tempEdge;


        } else if (parseInfo.parserPass == EDGES) { //Otherwise just add edges

           //Keep depth-counter up-to-date
           parseInfo.tagType[parseInfo.depth] = EDGE;
           parseInfo.parentId[parseInfo.depth] = edges.size();
           tempEdge->features["id"] = edges.size();

           if (verbose) printf("Parsed edge from %s to %s\n",tempEdge->start,tempEdge->end);
           if (verbose && !foundType.empty()) printf("  Edge inherited from type: %s\n",foundType.c_str());
           
           edges.push_back(*tempEdge);
           delete tempEdge;

        }

        
       

}

/** Process connector tag
 *
 * process the connector tag and add it to the associated node or node-type
 **/
void graphmap::addConn(const XML_Char *name, const XML_Char **atts) {

    connector *tempConn = new connector();

    //Initialize default optional values
    tempConn->autoconnect =       0;
    tempConn->th =              0.0;
    tempConn->features["th"] =  0.0;
	 tempConn->thr =             0.0;
    tempConn->features["thr"] =  0.0;
    tempConn->x =               0.0;
    tempConn->features["x"] =   0.0;
    tempConn->y =               0.0;
     tempConn->features["y"] =  0.0;

    //Parse connector attributes
    for(int i = 0; atts[i]; i+=2) {
        if (!strcmp("name",atts[i])) {
            strncpy(tempConn->name,atts[i+1],128);;
        }
        if (!strcmp("th",atts[i])) {
            tempConn->th = strtod(atts[i+1],NULL);
            tempConn->features["th"] = tempConn->th;
				tempConn->thr = tempConn->th * (M_PI / 180);
            tempConn->features["thr"] = tempConn->thr;
        }
		  if (!strcmp("thr",atts[i])) {
            tempConn->thr = strtod(atts[i+1],NULL);
            tempConn->features["thr"] = tempConn->th;
				tempConn->th = tempConn->thr * (180 / M_PI);
            tempConn->features["th"] = tempConn->th;
        }
        if (!strcmp("x",atts[i])) {
            tempConn->x = strtod(atts[i+1],NULL);
            tempConn->features["x"] = tempConn->x;
        }
        if (!strcmp("y",atts[i])) {
            tempConn->y = strtod(atts[i+1],NULL);
            tempConn->features["y"] = tempConn->y;
        }
        if (!strcmp("autoconnect",atts[i])) {
            tempConn->autoconnect = atoi(atts[i+1]);
        }
    }


    //Update depth counters and add connector to either nodetypes or nodes
    if (parseInfo.tagType[parseInfo.depth-1] == NODETYPE) {

        //Save type and connector id (long index as it points to parent-nodetype
        parseInfo.tagType[parseInfo.depth] = CONN;
        parseInfo.parentId[parseInfo.depth] = nodeTypes[parseInfo.parentId[parseInfo.depth-1]].connectors.size();

        //Add connector to parent nodetype
        nodeTypes[parseInfo.parentId[parseInfo.depth-1]].connectors.push_back(*tempConn);

    } else if(parseInfo.tagType[parseInfo.depth-1] == NODE) {

        //Save type and connector id (long index as it points to parent-nodee)
        parseInfo.tagType[parseInfo.depth] = CONN;
        parseInfo.parentId[parseInfo.depth] = nodes[parseInfo.parentId[parseInfo.depth-1]].connectors.size();

        //Add connector to parent nodetype
        nodes[parseInfo.parentId[parseInfo.depth-1]].connectors.push_back(*tempConn);

    } else { //Error check, as connectors is only for nodes and nodetypes!
        printf("Error parsing connector: %s, parent is NOT node or nodetype!\n",tempConn->name);
        delete tempConn;
        return;

    }

    if (verbose) printf("Added connector %s\n",tempConn->name);
    delete tempConn;//Cleanup
}

/** Parser of properties tag
 *
 * The properties is a reserved tag, that assigns certain features to
 * edges.
 *
 * This means that a property is not placed within the mapped features, but
 * is placed directly in class-variables
 *
 **/
void graphmap::parseProperties(const XML_Char *name, const XML_Char **atts) {

    //Loop thrrough the properties attributtes
    for(int i = 0; atts[i]; i+=2) {
        if (!strcmp("length",atts[i])) {
            if (parseInfo.tagType[parseInfo.depth-1] == EDGE) {
                edges[parseInfo.parentId[parseInfo.depth-1]].length = strtod(atts[i+1],NULL);
                if (verbose) printf("Properties Edge %s\n",edges[parseInfo.parentId[parseInfo.depth-1]].typeName);
            } else if (parseInfo.tagType[parseInfo.depth-1] == EDGETYPE) {
                edgeTypes[parseInfo.parentId[parseInfo.depth-1]].length = strtod(atts[i+1],NULL);
                if (verbose) printf("Properties Edgetype %s\n",edgeTypes[parseInfo.parentId[parseInfo.depth-1]].typeName);
            } else if (parseInfo.tagType[parseInfo.depth-1] == NODETYPEEDGE) {
                nodeTypeEdges[parseInfo.parentId[parseInfo.depth-1]].length = strtod(atts[i+1],NULL);
                if (verbose) printf("Properties nodetypedge %s\n",nodeTypeEdges[parseInfo.parentId[parseInfo.depth-1]].typeName);
            }
        } else if (!strcmp("directional",atts[i])) {
            if (parseInfo.tagType[parseInfo.depth-1] == EDGE) {
                edges[parseInfo.parentId[parseInfo.depth-1]].directional = atoi(atts[i+1]);
            } else if (parseInfo.tagType[parseInfo.depth-1] == NODETYPEEDGE) {
                nodeTypeEdges[parseInfo.parentId[parseInfo.depth-1]].directional = atoi(atts[i+1]);
            } else if (parseInfo.tagType[parseInfo.depth-1] == EDGETYPE) {
                edgeTypes[parseInfo.parentId[parseInfo.depth-1]].directional = atoi(atts[i+1]);
            }
        } 
    }

    //Send properties attributes on to feature system (to support extra properties)
    featureParser(name,atts);
}

/** Parser of properties tag
 *
 * The properties is a reserved tag, that assigns certain features to
 * edges.
 *
 * This means that a property is not placed within the mapped features, but
 * is placed directly in class-variables
 *
 **/
void graphmap::parseDimension(const XML_Char *name, const XML_Char **atts) {

    double  tempVal;
    char    tempStr[128];
    //Pass it on to feature parser first
    featureParser(name,atts);

    //Then correct for the string contents of shape
    for (int i = 0; atts[i]; i+=2) {
        //Convert the shape to a double value (3.0 for any other than circle and rect)
       if (!strcmp("shape",atts[i])) {
           if (!strcmp("circle",atts[i+1])) {
               tempVal = 1.0;
           } else if (!strcmp("rect",atts[i+1]))  {
               tempVal = 2.0;
           } else {
               tempVal = 3.0;
           }

           strncpy(tempStr,atts[i+1],128);

           //Update the hashmap to the double value and set the shape string
           switch (parseInfo.tagType[parseInfo.depth-1]) {
                case NODETYPE :
                    nodeTypes[parseInfo.parentId[parseInfo.depth-1]].features["dimension.shape"] = tempVal;
                    strncpy(nodeTypes[parseInfo.parentId[parseInfo.depth-1]].shape,tempStr,128);
                    break;
                case NODE :
                    nodes[parseInfo.parentId[parseInfo.depth-1]].features["dimension.shape"] = tempVal;
                    strncpy(nodes[parseInfo.parentId[parseInfo.depth-1]].shape,tempStr,128);
                    break;

               default: break;
            };
       }
    }
}



/** Parser of edge/node features
 *
 * It is possible to add an arbitray number of features to a given type/node/edge
 *
 * A feature must have the format of <string name, double value>
 * When parsed, features is located in a hashmap, using the tag.attribute syntax
 * i.e. linemarker.color = 0
 */
void graphmap::featureParser(const XML_Char *name, const XML_Char **atts) {

    string *tempName = new string;
    double *tempValue = new double;
    map<string, double> testmap;

    for (int i = 0; atts[i]; i+=2) {

        //Create name and value set
        *tempName = name;
        tempName->append(".");
        tempName->append(atts[i]);
        *tempValue = strtod(atts[i+1],NULL);

        //Add to the correct hashmap
        switch (parseInfo.tagType[parseInfo.depth-1]) {
            case NODETYPE :
                nodeTypes[parseInfo.parentId[parseInfo.depth-1]].features[*tempName] = *tempValue;
                break;
            case EDGETYPE :
                edgeTypes[parseInfo.parentId[parseInfo.depth-1]].features[*tempName] = *tempValue;
                break;
            case NODETYPEEDGE :
                nodeTypeEdges[parseInfo.parentId[parseInfo.depth-1]].features[*tempName] = *tempValue;
                break;
            case NODE :
                nodes[parseInfo.parentId[parseInfo.depth-1]].features[*tempName] = *tempValue;
                break;
            case EDGE :
                edges[parseInfo.parentId[parseInfo.depth-1]].features[*tempName] = *tempValue;
                break;
            case CONN :
                //Select if connector-parent is node or nodeType
                if (parseInfo.tagType[parseInfo.depth-2] == NODETYPE) {
                    nodeTypes[parseInfo.parentId[parseInfo.depth-2]].connectors[parseInfo.parentId[parseInfo.depth-1]].features[*tempName] = *tempValue;
                } else if (parseInfo.tagType[parseInfo.depth-2] == NODE) {
                    nodes[parseInfo.parentId[parseInfo.depth-2]].connectors[parseInfo.parentId[parseInfo.depth-1]].features[*tempName] = *tempValue;
                } 
                break;
            default :
                printf("Error selecting type for feature %s\n",tempName->c_str());
                break;
        };
        if (verbose) printf("Adding/updating feature %s = %4.2f\n",tempName->c_str(),*tempValue);
    }
}


/** Perform postprocessing of map details
 *
 * This post processing of nodes and edges performs error checking on
 * edges and executes the auto-mapping of edges between connectors inside a node
 *
 *
 **/

int graphmap::postProcessMap(void) {

    if (verbose) printf(" *** Postprocessing nodes ***\n");

    edge    *tempEdge;
    int     connId = 0;
    string  endNode, endConn;
    string  startNode, startConn;
    string  tempName;
    bool    startFound, endFound;
    double  angleIn, angleOut, turnAngle, connDist, circRadius;

   //Start by looping throug all connectors, and update the numbering
    for(size_t nNode = 0; nNode < nodes.size(); nNode++) {
        for (size_t i = 0; i < nodes[nNode].connectors.size(); i++) {
            nodes[nNode].connectors[i].id = connId;
            nodes[nNode].connectors[i].features["id"] = connId;
            nodeidFromConnid[connId] = nNode;   //Save connId in table
            connId++;
        }
    }

    //Process all edges, update names and indexes
    for (size_t nEdge = 0; nEdge < edges.size(); nEdge++) {
        startFound = false;
        endFound = false;
        edges[nEdge].disabled = false;
        //Extract node and connector names
        tempName.assign(edges[nEdge].start);
        startNode = tempName.substr(0,tempName.find_last_of('.')); //Grab until last '.'
        startConn = tempName.substr(tempName.find_last_of('.')+1); //Grab the rest
        tempName.assign(edges[nEdge].end);
        endNode = tempName.substr(0,tempName.find_last_of('.')); //Grab until last '.'
        endConn = tempName.substr(tempName.find_last_of('.')+1); //Grab the rest
        //Loop through nodes and connectors and extract start and end details
        for(size_t nNode = 0; nNode < nodes.size(); nNode++) {
            for (size_t nConn = 0; nConn < nodes[nNode].connectors.size(); nConn++) {
                if ((!strcmp(startNode.c_str(),nodes[nNode].name)) &&
                    (!strcmp(startConn.c_str(),nodes[nNode].connectors[nConn].name))) {
                    edges[nEdge].startConnId = nodes[nNode].connectors[nConn].id;
                    edges[nEdge].startConnIndex = nConn;
                    edges[nEdge].startNodeIndex = nNode;
                    strcpy(edges[nEdge].startNodeName,startNode.c_str());
                    strcpy(edges[nEdge].startConnName,startConn.c_str());
                    edges[nEdge].startnode = &(nodes[nNode]);
                    edges[nEdge].startconnector = &(nodes[nNode].connectors[nConn]);
                    if (verbose) printf("Edge from N:%s.%s ",startNode.c_str(),startConn.c_str());
                    startFound = true;
                }
                if ((!strcmp(endNode.c_str(),nodes[nNode].name)) &&
                    (!strcmp(endConn.c_str(),nodes[nNode].connectors[nConn].name))) {
                    edges[nEdge].endConnId = nodes[nNode].connectors[nConn].id;
                    edges[nEdge].endConnIndex = nConn;
                    edges[nEdge].endNodeIndex = nNode;
                    strcpy(edges[nEdge].endNodeName,endNode.c_str());
                    strcpy(edges[nEdge].endConnName,endConn.c_str());
                    edges[nEdge].endnode = &(nodes[nNode]);
                    edges[nEdge].endconnector = &(nodes[nNode].connectors[nConn]);
                    if (verbose) printf("to N %s.%s\n",endNode.c_str(),endConn.c_str());
                    endFound = true;
                }
            }
        }

        //Check if both start and end-point has been found among nodes/connectors
        if (!startFound) {
            printf("Map postprocessing error: Startpoint %s was not found\n",edges[nEdge].start);
            edges[nEdge].disabled = true;
        }
        if (!endFound) {
            printf("Map postprocessing error: End %s was not found\n",edges[nEdge].end);
            edges[nEdge].disabled = true;
        }
    }

    //Finally create connections between all connectors that has the autoconnect
    //attribute set
    for(size_t nNode = 0; nNode < nodes.size(); nNode++) {
        for (size_t i = 0; i < nodes[nNode].connectors.size(); i++) {
            for (size_t j = 0; j < nodes[nNode].connectors.size(); j++) {
                //Skip connection between the same connector or if autoconnect is not activated
                if ((i != j) && nodes[nNode].connectors[i].autoconnect) {
                    //Search for already existing connections
                    for(size_t k = 0; k < edges.size(); k++) {
                        if (!strcmp(nodes[nNode].name,edges[k].startNodeName) &&
                            !strcmp(nodes[nNode].connectors[i].name,edges[k].startConnName) &&
                            !strcmp(nodes[nNode].name,edges[k].endNodeName) &&
                            !strcmp(nodes[nNode].connectors[i].name,edges[k].endConnName)) {
                            if (verbose) printf("Existing connection from %s.%s to %s.%s\n",edges[k].startNodeName,edges[k].startConnName,edges[k].endNodeName,edges[k].endConnName);
                            break;
                        }
                    }
                    tempEdge = new edge;

                    strcpy(tempEdge->startNodeName,nodes[nNode].name);
                    strcpy(tempEdge->endNodeName,nodes[nNode].name);
                    strcpy(tempEdge->startConnName,nodes[nNode].connectors[i].name);
                    strcpy(tempEdge->endConnName,nodes[nNode].connectors[j].name);
                    snprintf(tempEdge->start,NAMELEN,"%s.%s",tempEdge->startNodeName,tempEdge->startConnName);
                    snprintf(tempEdge->end,NAMELEN,"%s.%s",tempEdge->endNodeName,tempEdge->endConnName);
                    tempEdge->startNodeIndex = nNode;
                    tempEdge->startConnIndex = i;
                    tempEdge->startConnId = nodes[nNode].connectors[i].id;
                    tempEdge->endConnIndex = j;
                    tempEdge->endNodeIndex = nNode;
                    tempEdge->endConnId = nodes[nNode].connectors[j].id;
                    tempEdge->startnode = &(nodes[nNode]);
                    tempEdge->endnode   = &(nodes[nNode]);
                    tempEdge->startconnector = &(nodes[nNode].connectors[i]);
                    tempEdge->endconnector   = &(nodes[nNode].connectors[j]);

                    //Calculate angle between connectors
                    angleIn     = nodes[nNode].connectors[i].features["th"];
                    angleOut    = nodes[nNode].connectors[j].features["th"];
                    if (angleIn < angleOut)  angleIn += 360.0;
                    turnAngle = fabs((angleOut - angleIn) + 180.0);

                    //Calculate the distance between the connector coordinates sqrt((x2-x1)^2+(y2-y1)^2)
                    connDist = sqrt(
                                pow((nodes[nNode].connectors[j].features["x"]-nodes[nNode].connectors[i].features["x"]),2.0)
                                +pow((nodes[nNode].connectors[j].features["y"]-nodes[nNode].connectors[i].features["y"]),2.0));

                    //Calculate circle arc distance between nodes, if angle is larger than +/- 5.0 deg
                    if (turnAngle > 5.0) {
                        //Calculate the radius of the circle spanning between the two points
                        circRadius = (connDist/2)/(sin((turnAngle/2)*(M_PI/180.0)));
                        //Calculate length of circle arc
                        tempEdge->length = (circRadius*M_PI) * (turnAngle/180.0);
                    } else {
                        //Just take the straight distance between points if the angle is less than 5 degrees
                        tempEdge->length = connDist;
                    }
                    //Save length in feature table
                    tempEdge->features["properties.length"] = tempEdge->length;

                    if (verbose) printf("Created edge from %s.%s to %s.%s length %f\n",tempEdge->startNodeName,
                                    tempEdge->startConnName,tempEdge->endNodeName,tempEdge->endConnName,tempEdge->length);

                    edges.push_back(*tempEdge);
                    delete tempEdge;

                }
            }
        }
    }

	 //Loop through all nodes and connectors and update the local and map poses
	 bool found = true;
	 int depth = 0, pId;
	 string nName, pName;
	 //Iterate through the depth levels of the nodes, to make sure that the parent node
	 //is transformed before its childs.
	 for(int nDepth = 0; found == true; nDepth++) {
		 found = false;
		for(size_t nNode = 0; nNode < nodes.size(); nNode++) {
			//Extract the number of '.' in the name to get depth
			depth = 0;
			for(int i = 0; nodes[nNode].name[i] != '\0';i++)
				if (nodes[nNode].name[i] == '.') depth++;
			
			//Transform coordinates if depth is matched
			if(depth == nDepth) {
				found = true; //Mark that a node was found at this level
				if (depth == 0) { //Special zero case
					nodes[nNode].poseLocal.x = nodes[nNode].x;
					nodes[nNode].poseLocal.y = nodes[nNode].y;
					nodes[nNode].poseLocal.th = nodes[nNode].th;
					nodes[nNode].poseLocal.thr= nodes[nNode].thr;
					nodes[nNode].pLocal.x = nodes[nNode].x;
					nodes[nNode].pLocal.y = nodes[nNode].y;
					nodes[nNode].pLocal.h = nodes[nNode].thr;
					nodes[nNode].poseMap.x  = 0.0;
					nodes[nNode].poseMap.y  = 0.0;
					nodes[nNode].poseMap.th = 0.0;
					nodes[nNode].poseMap.thr= 0.0;
				} else {
					nodes[nNode].poseLocal.x = nodes[nNode].x;
					nodes[nNode].poseLocal.y = nodes[nNode].y;
					nodes[nNode].poseLocal.th = nodes[nNode].th;
					nodes[nNode].poseLocal.thr= nodes[nNode].thr;
					nodes[nNode].pLocal.x = nodes[nNode].x;
					nodes[nNode].pLocal.y = nodes[nNode].y;
					nodes[nNode].pLocal.h = nodes[nNode].thr;
					//Get the parent ID from name
					nName = nodes[nNode].name;
					pName = nName.substr(0,nName.find_last_of("."));
					pId   = getNodeId(pName);
					//Transform from the parent's mappose

					if (pId >= 0) {
						nodes[nNode].poseMap = poseTransform(nodes[nNode].poseLocal, nodes[pId].poseMap);
						//Add to feature map
						nodes[nNode].features["mappose.x"] = nodes[nNode].poseMap.x;
						nodes[nNode].features["mappose.y"] = nodes[nNode].poseMap.y;
						nodes[nNode].features["mappose.th"] = nodes[nNode].poseMap.th;
						nodes[nNode].features["mappose.thr"] = nodes[nNode].poseMap.thr;
						nodes[nNode].pMap.x = nodes[nNode].poseMap.x;
						nodes[nNode].pMap.y = nodes[nNode].poseMap.y;
						nodes[nNode].pMap.h = nodes[nNode].poseMap.thr;
						if (verbose) printf("Transforming %s %d as child of %s %d\n",nodes[nNode].name,nNode,pName.c_str(),pId);
					}
				}
				if (verbose) printf("Transformed: %s from (%3.2f,%3.2f,%3.2f) to (%3.2f,%3.2f,%3.2f)\n",nodes[nNode].name,
									nodes[nNode].poseLocal.x,nodes[nNode].poseLocal.y,nodes[nNode].poseLocal.th,
							      nodes[nNode].poseMap.x,nodes[nNode].poseMap.y,nodes[nNode].poseMap.th);
						  
				//Also transform all connector poses
				for (size_t nConn = 0; nConn < nodes[nNode].connectors.size(); nConn++) {
					nodes[nNode].connectors[nConn].poseLocal.x = nodes[nNode].connectors[nConn].x;
					nodes[nNode].connectors[nConn].poseLocal.y = nodes[nNode].connectors[nConn].y;
					nodes[nNode].connectors[nConn].poseLocal.th = nodes[nNode].connectors[nConn].th;
					nodes[nNode].connectors[nConn].poseLocal.thr = nodes[nNode].connectors[nConn].thr;
					nodes[nNode].connectors[nConn].pLocal.x = nodes[nNode].connectors[nConn].x;
					nodes[nNode].connectors[nConn].pLocal.y = nodes[nNode].connectors[nConn].y;
					nodes[nNode].connectors[nConn].pLocal.h = nodes[nNode].connectors[nConn].thr;
					//Then transform from the parents mappose
					nodes[nNode].connectors[nConn].poseMap = poseTransform(nodes[nNode].connectors[nConn].poseLocal, nodes[nNode].poseMap);
					//Add to feature map
					nodes[nNode].connectors[nConn].features["mappose.x"] = nodes[nNode].connectors[nConn].poseMap.x;
					nodes[nNode].connectors[nConn].features["mappose.y"] = nodes[nNode].connectors[nConn].poseMap.y;
					nodes[nNode].connectors[nConn].features["mappose.th"] = nodes[nNode].connectors[nConn].poseMap.th;
					nodes[nNode].connectors[nConn].features["mappose.thr"] = nodes[nNode].connectors[nConn].poseMap.thr;
					nodes[nNode].connectors[nConn].pMap.x = nodes[nNode].connectors[nConn].poseMap.x;
					nodes[nNode].connectors[nConn].pMap.y = nodes[nNode].connectors[nConn].poseMap.y;
					nodes[nNode].connectors[nConn].pMap.h = nodes[nNode].connectors[nConn].poseMap.thr;


					if (verbose) printf("   Conn: %s transformed from (%3.2f,%3.2f,%3.2f) to (%3.2f,%3.2f,%3.2f)\n",nodes[nNode].connectors[nConn].name,
							  nodes[nNode].connectors[nConn].poseLocal.x,nodes[nNode].connectors[nConn].poseLocal.y,nodes[nNode].connectors[nConn].poseLocal.th,
							  nodes[nNode].connectors[nConn].poseMap.x,nodes[nNode].connectors[nConn].poseMap.y,nodes[nNode].connectors[nConn].poseMap.th);
				}
			}
	   }
	 }

	 //Loop through all edges again and calculate lengths if they have not been set in map
	 double len;
	 connector *connSt, *connEn;
	 for (size_t nEdge = 0; nEdge < edges.size(); nEdge++) {
		 //Only if the length has not been assigned
		 if (edges[nEdge].length == 0.0) {
		    //Get start and end connectors
			 connSt = getConnector(edges[nEdge].startConnId);
			 connEn = getConnector(edges[nEdge].endConnId);
			 len = sqrt( pow(connSt->poseMap.x - connEn->poseMap.x,2) + pow(connSt->poseMap.y - connEn->poseMap.y,2));
			 edges[nEdge].length = len;
			 edges[nEdge].features["properties.length"] = len;
			 if (verbose) printf("   Adding length to edge %d to %f\n",nEdge,len);
		 }
	 }

	 if (verbose) printf("Done postprocessing!\n");
    
    return 1;
}



