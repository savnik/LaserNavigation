/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.408+                              *
 *   Contains map database						   *
 *   Edited by Peter Tjell (s032041) & Søren Hansen (s021751)              *
 *   Edited by Anders Billesø Beck (s021786)
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

#include <map>
#include <string>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/ucmdexe.h>
#include <urob4/ulogfile.h>
# include <urob4/uvarcalc.h>
# include <urob4/uresposehist.h>
# include <umap4/upose.h>
# include <ugen4/uline.h>
# include <urob4/uresvarpool.h>

# include "uresmapbase.h"

UResMapbase::UResMapbase()
{ // these first two lines is needed
  // to save the ID and version number
  setResID("mapbase", 10);
  setDescription("Database for maprelated information.", false);
  createBaseVar();
  resVersion = getResVersion();
  // other local initializations
  lines_cnt = 0;
  points_cnt = 0;
  strncpy(current_node, "UNKNOWN_NODE", NAMELEN);
}

UResMapbase::~UResMapbase()
{
  /* Close */
  printf(" --  UResMapbase closed  --\n");
}

void UResMapbase::createBaseVar()
{
    //Methods to access
    addMethod("edge",      "ds", "Get feature value from [s] extracted from the edge id [d]");
    addMethod("edge",      "ss", "Get feature value from [s] extracted from the edge name [s]. Syntax: (edgename,featurename)");
    addMethod("node",      "ds", "Get feature value from [s] extracted from the node id [d]");
    addMethod("node",      "ss", "Get feature value from [s] extracted from the node name [s]. Syntax: (nodename,featurename)");
    addMethod("connector", "ds", "Get feature value from [s] extracted from the connector id [d]");
    addMethod("connector", "ss", "Get feature value from [s] extracted from the connector name [s]. "
        "Syntax: (connname,featurename) ");
    addMethod("connectorGlobal", "ss", "Get feature value from [s] extracted from the connector with the globally unique name [s]."
        "No parent-nodes are needed. Syntax: (Unique-connname,featurename) ");
	 addMethod("getConnMapPose","s","Get the mappose of a connector (returns in var connMapPose)");
	 varConnMapPose  = addVar("connMapPose", "0.0 0.0 0.0", "pose", "Return value of Connetor mappose");
    addMethod("addMapLine", "sdddddd", "Add a mapline to current reference node. "
        "Parameters are addMapLine(\"name\", startX, startY, endX, endY, radius, isObstacle). "
        "If isObstacle==0, then line is for localizer only. "
        "Coordinates are in map coordinates (see graph for position of node).");

}

/** Methods avaliable through the varpool (i.e. avaliable from within the rulebased */
bool UResMapbase::methodCall(const char *name, const char *paramOrder, char **strings, const double *pars, double *value, UDataBase **returnStruct, int *returnStructCnt)
{

  double    retValue = 0.0;
  int       id;
  bool      result = true;
  char      *paramStr = NULL;
  edge      *tempEdge = NULL;
  node      *tempNode = NULL;
  connector *tempConn = NULL;

  //Get feature details from edge
  if (strcasecmp(name, "edge") == 0)
  {
    //Get the edge id and extract the edgege
    if (strcmp(paramOrder, "ds") == 0) {
        id = (int) pars[0];
        tempEdge = graphmapper.getEdge(id);
        paramStr = strings[0];
    } else if (strcmp(paramOrder, "ss") == 0) {
        tempEdge = graphmapper.getEdge(strings[0]);
        paramStr = strings[1];
    }
    if (tempEdge != NULL) {
        retValue = -1.0;
        if (tempEdge->features.count(paramStr) == 0) {
            retValue = -1.0;
        } else {
            retValue = tempEdge->features[paramStr];
        }
    } else retValue = -1.0;
  }
  //Get feature details from node
  else if (strcasecmp(name, "node") == 0)
  {
    //Get the edge id and extract the edge
    if (strcmp(paramOrder, "ds") == 0) {
        id = (int) pars[0];
        tempNode = graphmapper.getNode(id);
        paramStr = strings[0];
    } else if (strcmp(paramOrder, "ss") == 0) {
        tempNode = graphmapper.getNode(strings[0]);
        paramStr = strings[1];
    }
    if (tempNode != NULL) {
        if (tempNode->features.count(paramStr) == 0) {
            retValue = -1.0;
        } else {
            retValue = tempNode->features[paramStr];
        }
      } else retValue = -2.0;
  }
  //Get feature details from connector
  else if (strcasecmp(name, "connector") == 0)
  {
    //Get the connector id and extract the connector
    if (strcmp(paramOrder, "ds") == 0) {
        id = (int) pars[0];
        tempConn = graphmapper.getConnector(id);
        paramStr = strings[0];
    } else if (strcmp(paramOrder, "ss") == 0) {
        tempConn = graphmapper.getConnector(strings[0]);
        paramStr = strings[1];
    }
    if (tempConn != NULL) {
        if (tempConn->features.count(paramStr) == 0) {
            retValue = -1.0;
        } else {
            retValue = tempConn->features[paramStr];
        }
    } else {
	retValue = -2.0;
      
    }
  }
  //Get feature details from connectorGlobal
  else if (strcasecmp(name, "connectorGlobal") == 0)
  {
    //Get the connector id and extract the connector
    if (strcmp(paramOrder, "ss") == 0) {
        tempConn = graphmapper.getUniqueConnector(strings[0]);
        paramStr = strings[1];
    }
    if (tempConn != NULL) {
        if (tempConn->features.count(paramStr) == 0) {
            retValue = -1.0;
        } else {
            retValue = tempConn->features[paramStr];
        }
    } else {
	retValue = -2.0;
      
    }
  }
    //Get feature details from connector
  else if (strcasecmp(name, "getConnMapPose") == 0)
  {
    //Get the connector id and extract the connector
    if (strcmp(paramOrder, "s") == 0) {
        tempConn = graphmapper.getConnector(strings[0]);
    }
    if (tempConn != NULL) {
		 UPose cPose(tempConn->poseMap.x,tempConn->poseMap.y,tempConn->poseMap.thr);
		 *returnStructCnt = 1;
		 varConnMapPose->setPose(&cPose);

	 } else {
		 result = false;
    }
  }
  else if ((strcasecmp(name, "addMapLine") == 0) and (strcmp(paramOrder, "sdddddd") == 0))
  { // add a map line to current node
    addMapLine(strings[0], pars[0], pars[1], pars[2], pars[3], pars[4], pars[5] > 0.5);
    retValue = 1;
  }

    // return result - if a location is provided
  if (value != NULL)
    *value = retValue;
    // it is good practice to set count of returned structures to 0
  if (returnStructCnt != NULL)
    *returnStructCnt = 0;
  return result;
}

const char * UResMapbase::print(const char * preString, char * buff, int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  UPose pose;
  snprintf(buff, buffCnt, "%s has %d nodes, %d edges, %d lines, %d points\n",
            preString,
            graphmapper.nodes.size(), graphmapper.edges.size(),
            lines_cnt, points_cnt);
  n = strlen(p1);
  p1 = &buff[n];
  pose = getMapRefPose();
  snprintf(p1, buffCnt - n, "current node = '%s' at (%.2fx, %.2fy, %.4fh)\n", current_node, pose.x, pose.y, pose.h);
  return buff;
}

///////////////////////////////////////////////////////

bool UResMapbase::mapload(const char *filename)
{
  char buffer[BUFFSIZE];
  FILE *fptr;
  int done;
  int len;

  printf("You are trying to load the file \"%s\"\n", filename);


  /*  This should be the way XML maps are loaded using EXPAT: */

  /* Reset mapcounters: */
  lines_cnt = 0;
  points_cnt = 0;

  XML_Parser p = XML_ParserCreate(NULL);
  if (! p)
  {
    fprintf(stderr, "Couldn't allocate memory for parser\n");
    return false;
  }
  XML_SetUserData(p, this);
  XML_SetElementHandler(p, UResMapbase::start, UResMapbase::end);

  fptr = fopen(filename, "r");
  if(fptr == NULL)
  {
    printf("Mapbase load unable to open file %s\n", filename);
    return false;
  }

  do
  {
    len = fread(buffer, 1, BUFFSIZE, fptr);
    printf("Read length = %d\n", len);

    if (ferror(fptr))
    {
      fprintf(stderr, "Read error\n");
      return false;
    }
    done = feof(fptr);

    if (! XML_Parse(p, buffer, len, done))
    {
      fprintf(stderr, "Parse error at line %d:\n%s\n", (int) XML_GetCurrentLineNumber(p), XML_ErrorString(XML_GetErrorCode(p)));
      fclose(fptr);
      return false;
    }
  } while(!done);

  XML_ParserFree(p);
  printf("Mapbase loaded %i lines and %i points successfully.\n", lines_cnt, points_cnt);
  fclose(fptr);
  return true;
}


void UResMapbase::display(void)
{
  int sz;

  printf("Contains: %d lines and %d points\n", lines_cnt, points_cnt);

  printf("\n------------------- MAP ------------------\n");
  sz = (int) maplines.size();
  for(int i = 0; i < sz; i++)
  {
    printf("Map line %d %s:\n", i, maplines[i].number);
    maplines[i].display();
    printf("--------------------------------------------\n");
  }

  sz = (int) mappoints.size();
  for(int i = 0; i < sz; i++)
  {
    printf("Map point %d:\n", i);
    mappoints[i].display();
    printf("--------------------------------------------\n");
  }
}


/* Export map to XML */
string UResMapbase::exportMap(void) {

    string xmlBuf;
    const int STRLEN = 1024;
    char tempStr[STRLEN];

    //Create return string
    if (graphmapper.maploaded) {
        snprintf(tempStr,STRLEN,"<map name=\"%s\">\n",graphmapper.mapname.c_str());
        xmlBuf = tempStr;

        //Iterate through the top level nodes and call the recursive export function
        for (size_t i = 0; i < graphmapper.nodes.size(); i++) {
            //Top-level does not have a '.' in the name
            if (strchr(graphmapper.nodes[i].name,'.') == NULL) {
                xmlBuf += recursiveNodeExport(i,1);
            }
        }
        
        //Export all the edges (use their notation in global context)
        xmlBuf += exportEdge();

        //Close the map
        xmlBuf += "</map>\n";
    }
    //If no graphmap is loaded, just export feature-map (if it is loaded)
    else {
        xmlBuf = "<map warning=\"No graphmap loaded\"/>\n";
    }

    return xmlBuf;

}

/* Recursive function to iterate down through the node tree and create XML */
string UResMapbase::recursiveNodeExport(int nodeId, int level) {

    string      blanks; //Blanks holder to make the proper indentation of the XML
    string      xmlBuf;
    string      nodeName;
    const int   STRLEN = 1024;
    char        tempStr[STRLEN];
    node        *no = &(graphmapper.nodes[nodeId]);

    for (int i = 0; i < level; i++) blanks += "  "; //Two whitespaces pr level

    //Extract the last part of the node name (after the last '.')
    nodeName = no->name;
    if (nodeName.find_last_of('.') != nodeName.npos) {
        nodeName = nodeName.substr(nodeName.find_last_of('.')+1);
    }

    //Start by generating node tag
    snprintf(tempStr,STRLEN,"%s<node name=\"%s\" x=\"%f\" y=\"%f\" th=\"%f\" thr=\"%f\">\n",
            blanks.c_str(),nodeName.c_str(),no->x,no->y,no->th,no->thr);
    xmlBuf += tempStr;

	 //Add the mappose (Now in feature map)
	 /*snprintf(tempStr,STRLEN,"%s  <mappose x=\"%f\" y=\"%f\" th=\"%f\" thr=\"%f\">\n",
            blanks.c_str(),no->poseMap.x,no->poseMap.y,no->poseMap.th,no->poseMap.thr);
	 xmlBuf += tempStr;*/

    //Add the node-features
    xmlBuf += exportFeature(no->features,level+1);

    //Loop through the connectors
    for (size_t nConn = 0; nConn < no->connectors.size(); nConn++) {
        snprintf(tempStr,STRLEN,"%s  <connector name=\"%s\" x=\"%f\" y=\"%f\" th=\"%f\" thr=\"%f\">\n",
            blanks.c_str(),no->connectors[nConn].name,no->connectors[nConn].x,
            no->connectors[nConn].y,no->connectors[nConn].th,no->connectors[nConn].thr);
        xmlBuf += tempStr;

		  //Add the mappose (Now in feature map)
		  /*snprintf(tempStr,STRLEN,"%s    <mappose x=\"%f\" y=\"%f\" th=\"%f\" thr=\"%f\">\n",
            blanks.c_str(),no->connectors[nConn].poseMap.x,no->connectors[nConn].poseMap.y,
				no->connectors[nConn].poseMap.th,no->connectors[nConn].poseMap.thr);
		  xmlBuf += tempStr;*/

        //Add the features
        xmlBuf += exportFeature(no->connectors[nConn].features,level+2);

        //Close the connector tag
        xmlBuf += blanks;
        xmlBuf += "  "; //Extra indentation
        xmlBuf += "</connector>\n";
    }

    //TODO: Add output from Sørens feature map to xmlBuf
    for(size_t cnt = 0; cnt < maplines.size(); cnt++)
    {
      if(strcmp(maplines[cnt].nodename, no->name) == 0)
        maplines[cnt].xml_export(&xmlBuf, tempStr, blanks.c_str());
    }
    for(size_t cnt = 0; cnt < mappoints.size(); cnt++)
    {
      if(strcmp(mappoints[cnt].nodename, no->name) == 0)
        mappoints[cnt].xml_export(&xmlBuf, tempStr, blanks.c_str());
    }

    //Recursive part: Search for nodes with "thisname"."subname"
    for (size_t nNode = 0; nNode < graphmapper.nodes.size(); nNode++) {
          //Skip this node
        if (nNode != (size_t)nodeId) {
            //Test if there is any nodes with a matching name (just the first part)
            if (!strncmp(no->name,graphmapper.nodes[nNode].name,strlen(no->name))) {
               //Then test if there is any '.'s left in the name (meaning the node is
               //two levels below this node
                if (strchr(&(graphmapper.nodes[nNode].name[strlen(no->name)+1]),'.') == NULL) {
                    //Call this function recursively
                    xmlBuf += recursiveNodeExport(nNode,level + 1);
                }
            }
        }

    }

    //Close the node tag
    xmlBuf += blanks;
    xmlBuf += "</node>\n";

    //Return the XML string
    return xmlBuf;

}


/* Export all edges */
string UResMapbase::exportEdge(void) {

    string xmlBuf;
    const int   STRLEN = 256;
    char        tmpStr[STRLEN];

    //Iterate through all the edges
    for (size_t i = 0; i < graphmapper.edges.size(); i++) {
        snprintf(tmpStr,STRLEN,"  <edge start=\"%s\" end=\"%s\" length=\"%f\">\n",
            graphmapper.edges[i].start,graphmapper.edges[i].end,graphmapper.edges[i].length);
        xmlBuf += tmpStr;

        //Export the feature map
        xmlBuf += exportFeature(graphmapper.edges[i].features,2);

        //Close the edge tag
        xmlBuf += "  </edge>\n";

    }

    return xmlBuf;

}


string UResMapbase::exportFeature(map<string,double> featureMap,int level) {

   map<string,double>::iterator fIt;
   string      blanks; //Blanks holder to make the proper indentation of the XML
   string      featTag, featParam;
   const int   STRLEN = 1024;
   char        tempStr[STRLEN];
   string      xmlBuf;

   //Do noting, if feature map is empty
   if (featureMap.empty()) return xmlBuf;

   for (int i = 0; i < level; i++) blanks += "  "; //Two whitespaces pr level

    //Loop through feature table (sorry! One line pr feature so far..)
    for (fIt = featureMap.begin(); fIt != featureMap.end(); fIt++) {
        featTag = (*fIt).first;
        //Check if a '.' exist in the feature name, otherwise skip it..
        if (featTag.find_first_of('.') != featTag.npos) {
            //Devide into two text strings for converting to
            // properties.length = <properties length="">
            featParam = featTag.substr(featTag.find_first_of('.')+1);
            featTag = featTag.substr(0,featTag.find_first_of('.'));
            snprintf(tempStr,STRLEN,"%s<%s %s=\"%f\"/>\n",
                    blanks.c_str(),featTag.c_str(),featParam.c_str(),(*fIt).second);
            xmlBuf += tempStr;
        }
    }

   return xmlBuf;

}



// Expat XML helper functions:
//
//

/* XML starting tag */
void UResMapbase::start(void *userData, const XML_Char *el, const XML_Char **attr)
{
  UResMapbase *mapbase = (UResMapbase *) userData;
  int i;

  if(strcmp(el, "map") == 0)
  {
    for(i = 0; attr[i]; i+= 2)
    {
      if(strcmp(attr[i], "nodename") == 0)
        strncpy(mapbase->current_node, attr[i+1], NAMELEN);
    }
  }
  else if(strcmp(el, "line") == 0)
    mapbase->line_parse(attr);
  else if(strcmp(el, "start") == 0)
    mapbase->add_startpoint(attr);
  else if(strcmp(el, "end") == 0)
    mapbase->add_endpoint(attr);
  else if(strcmp(el, "point") == 0)
    mapbase->point_parse(attr);
  else if(strcmp(el, "value") == 0)
    mapbase->add_point(attr);
}

/* XML ending tag */
void UResMapbase::end(void *userData, const XML_Char *el)
{
  //UResMapbase *mapbase = (UResMapbase *) userData;

  //Depth--;

  if(strcmp(el, "map") == 0)
  {
    //strncpy(mapbase->current_node, "UNKNOWN_NODE", NAMELEN);
  }
} 

/* Starts parsing a line element from the file to the map */
void UResMapbase::line_parse(const char **attr)
{
  int i;

  lines_cnt++;
  mapline *parseline = new mapline;

  for(i = 0; attr[i]; i+= 2)
  {
    if(strcmp(attr[i], "perimeter") == 0)
      parseline->perimeter = atof(attr[i+1]);
    if(strcmp(attr[i], "isObstacle") == 0)
      parseline->isObstacle = strtol(attr[i+1], NULL, 0);
    if(strcmp(attr[i], "number") == 0)
      strncpy(parseline->number, attr[i+1], mapline::MNL);
  }
  strncpy(parseline->nodename, current_node, NAMELEN);
  maplines.push_back(*parseline);
  delete parseline;
}

/* Starts parsing a point element from the file to the map */
void UResMapbase::point_parse(const char **attr)
{
  int i;

  points_cnt++;
  mappoint *parsepoint = new mappoint();

  for(i = 0; attr[i]; i+= 2)
  {
    if(strcmp(attr[i], "perimeter") == 0)
      parsepoint->perimeter = atof(attr[i+1]);
  }
  strncpy(parsepoint->nodename, current_node, NAMELEN);
  mappoints.push_back(*parsepoint);
  delete parsepoint;
}


/* Stores the starting points */
void UResMapbase::add_startpoint(const char **attr)
{
  int i;

  for(i = 0; attr[i]; i += 2)
  {
    if(strcmp(attr[i], "x") == 0)
      maplines.back().x_s = atof(attr[i+1]);
    else if(strcmp(attr[i], "y") == 0)
      maplines.back().y_s = atof(attr[i+1]);
  }
}

/* Stores the ending points */
void UResMapbase::add_endpoint(const char **attr)
{
  int i;

  for(i = 0; attr[i]; i += 2)
  {
    if(strcmp(attr[i], "x") == 0)
      maplines.back().x_e = atof(attr[i+1]);
    else if(strcmp(attr[i], "y") == 0)
      maplines.back().y_e = atof(attr[i+1]);
  }
  maplines.back().fill_ABC();
}

/* Stores points */
void UResMapbase::add_point(const char **attr)
{
  int i;

  for(i = 0; attr[i]; i += 2)
  {
    if(strcmp(attr[i], "x") == 0)
      mappoints.back().x = atof(attr[i+1]);
    else if(strcmp(attr[i], "y") == 0)
      mappoints.back().y = atof(attr[i+1]);
  }
}

/////////////////////////////////////////////////

node * UResMapbase::getMapRefNode()
{
  node * nd;
  nd = graphmapper.getNode(current_node);
  return nd;
}

/////////////////////////////////////////////////

UPose UResMapbase::getMapRefPose()
{
  node * nd;
  UPose result;
  //
  nd = graphmapper.getNode(current_node);
  if (nd != NULL)
    result = nd->getOrigin();
  else
    result.clear();
  //
  return result;
}
//
//
bool UResMapbase::addMapLine(const char * name,
                              double startX, double startY,
                              double endX, double endY,
                              double radius, bool isObstacle)
{
  mapline *line = new mapline;
  //
  strncpy(line->nodename, current_node, NAMELEN);
  strncpy(line->number, name, mapline::MNL);
  line->x_s = startX;
  line->y_s = startY;
  line->x_e = endX;
  line->y_e = endY;
  line->isObstacle = isObstacle;
  line->perimeter = radius;
  line->fill_ABC();
  maplines.push_back(*line);
  delete line;
  return true;
}
