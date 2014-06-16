/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
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
#include "graph.h"


bool Edge::set(Node * parent, const char * start, const char * end, double lng)
{
  char * p2;
  bool result;
    // start name - remove connector part
  strncpy(name1, start, MNL);
  p2 = strrchr(name1, '.');
  result = (p2 != NULL);
  if (result)
  { // must have a connector name
    *p2++ = '\0';
    strncpy(conn1, p2, MCL);
  }
    // end name - remove connector part
  strncpy(name2, end, MNL);
  p2 = strrchr(name2, '.');
  result &= (p2 != NULL);
  if (result)
  {
    *p2++ = '\0';
    strncpy(conn2, p2, MCL);
  }
  length = lng;
  return result;
}

/////////////////////////////////////////////

bool Edge::setNodes(Node * parent)
{ // set relations only
  const char * p1;
  char * p2;
  Node * n;
  char b[MNL];
    // start name - remove connector part
  strncpy(b, name1, MNL);
  p2 = b;
  p1 = p2;
  n = parent;
  while (p2 != NULL)
  {
    p1 = strsep(&p2, ".");
    n = n->getChild(p1);
  }
  node1 = n;
    // end name - remove connector part
  strncpy(b, name2, MNL);
  p2 = b;
  p1 = p2;
  n = parent;
  while (p2 != NULL)
  {
    p1 = strsep(&p2, ".");
    n = n->getChild(p1);
  }
  node2 = n;
  return node1 != NULL and node2 != NULL;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

bool Node::connectEdges()
{
  bool result = true;
  Edge * e = edge;
  Node * nn;
  //
  while (e != NULL)
  {
    result = e->setNodes(this);
    if (not result)
      printf("Edge %s-%s in node %s failed to find refered nodes\n", e->name1, e->name2, name);
    e = e->next;
  }
  // and all the children too
  nn = child;
  while (nn != NULL)
  {
    nn->connectEdges();
    nn = nn->sister;
  }
  return result;
}

/////////////////////////////////////////////////

PoseQ Node::getMapPose(const char * connector)
{
  PoseQ conPose;
  if (cons == NULL)
    conPose.clear();
  else
    conPose = cons->getPose(connector);
  // convert to map coordinates
  conPose = getMapPose(conPose);
  return conPose;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

Graph::Graph()
{
  ntypCnt = 0;
  currentNode = &rootNode;
  currentType = NULL;
}

/////////////////////////////////////////////////

void Graph::startGraphTag(void *userData, const XML_Char *el, const XML_Char **attr)
{
  int i;
  Node * nn;
  Graph * gr = (Graph *) userData;
  PoseQ pose;
  const char * s = NULL;
  const char * ty = NULL;
  double len;
  //
  if(strcmp(el, "graph") == 0)
  {
    for(i = 0; attr[i] != NULL; i+= 2)
    { // get max width of a tree - from center out
      if(strcmp(attr[i], "name") == 0)
        strncpy(gr->name, attr[i+1], MNL);
    }
  }
  else if (strcmp(el, "nodetype") == 0)
  {
    if (gr->ntypCnt < MNT)
    {
      gr->currentType = &gr->ntyp[gr->ntypCnt];
      for(i = 0; attr[i] != NULL; i+= 2)
      { // get max width of a tree - from center out
        if(strcmp(attr[i], "name") == 0)
          gr->currentType->setName(attr[i+1]);
      }
    }
  }
  else if (gr->currentType != NULL and  strcmp(el, "dimension") == 0)
  {
    for(i = 0; attr[i] != NULL; i+= 2)
    { // get max width of a tree - from center out
      if(strcmp(attr[i], "radius") == 0)
        gr->currentType->radius = strtod(attr[i+1], NULL);
    }
  }
  else if (gr->currentType != NULL and  strcmp(el, "connector") == 0)
  {
    pose.clear();
    for(i = 0; attr[i]; i += 2)
    {
      if(strcmp(attr[i], "x") == 0)
        pose.x = atof(attr[i+1]);
      else if(strcmp(attr[i], "y") == 0)
        pose.y = atof(attr[i+1]);
      else if(strcmp(attr[i], "th") == 0)
        pose.h = atof(attr[i+1]) * M_PI / 180.0;
      else if(strcmp(attr[i], "name") == 0)
        s = attr[i+1];
    }
    gr->currentType->addConnector(&pose, s);
  }
  else if(strcmp(el, "node") == 0)
  {
    nn = new Node();
    pose.clear();
    ty = "none";
    for(i = 0; attr[i]; i += 2)
    {
      if(strcmp(attr[i], "x") == 0)
        pose.x = atof(attr[i+1]);
      else if(strcmp(attr[i], "y") == 0)
        pose.y = atof(attr[i+1]);
      else if(strcmp(attr[i], "th") == 0)
        pose.h = atof(attr[i+1]) * M_PI / 180.0;
      else if(strcmp(attr[i], "name") == 0)
        s = attr[i+1];
      else if(strcmp(attr[i], "type") == 0)
        ty = attr[i+1];
    }
    nn->set(pose, s);
    gr->addChild(nn, ty);
  }
  else if(strcmp(el, "edge") == 0)
  {
    len = 1.0;
    for(i = 0; attr[i]; i += 2)
    {
      if(strcmp(attr[i], "start") == 0)
        s = attr[i+1];
      else if(strcmp(attr[i], "end") == 0)
        ty = attr[i+1];
      else if(strcmp(attr[i], "length") == 0)
        len = atof(attr[i+1]);
    }
    gr->currentNode->addEdge(s, ty, len);
  }
}

/////////////////////////////////////////////

void Graph::endGraphTag(void *userData, const XML_Char *el)
{
  Graph * gr = (Graph *) userData;

  if (strcmp(el, "nodetype") == 0)
  {
    gr->currentType = NULL;
    gr->ntypCnt++;
  }
  else if (strcmp(el, "node") == 0)
  {
    gr->currentNode = gr->currentNode->parent;
  }
}

////////////////////////////////////////////

bool Graph::graphload(FILE * fptr)
{
  const int BUFFSIZE = 2000;
  char buffer[BUFFSIZE];
  int done;
  int len;
  bool result;

  XML_Parser p = XML_ParserCreate(NULL);
  if (! p)
  {
    fprintf(stderr, "Couldn't allocate memory for parser\n");
    return false;
  }
  XML_SetUserData(p, this);
  XML_SetElementHandler(p, Graph::startGraphTag, Graph::endGraphTag);

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
  result = rootNode.connectEdges();
  return result;
}

