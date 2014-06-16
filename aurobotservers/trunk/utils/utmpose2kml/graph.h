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
#ifndef GRAPH_H
#define GRAPH_H

#include "poseq.h"
/**
	@author Christian Andersen <chrand@mail.dk>
*/
class Nodetype
{
  public:
  // max connectors in a node
    static const int MCC = 10;
    PoseQ con[MCC];
  /// connector name length
    static const int MCNL = 10;
  /// connector name
    char cn[MCC][MCNL];
  /// number of connectors
    int conCnt;
  /// type name
    static const int MNL = 100;
    char name[MNL];
  /// radius > 0.0 if circle
    double radius;
  /// square if w and h > 0.0
  /// and the x,y coordinate of the bottom left corner
  /// oriented like the node pose
    double w, h, blx, bly;

public:
  /// constructor
  Nodetype()
  {
    conCnt = 0;
    name[0] = '\0';
    radius = 0.0;
    blx = 0.0;
    bly = 0.0;
    w = 0.0;
    h = 0.0;
  }
  /**
   * Set name */
  void setName(const char * toName)
  {
    strncpy(name, toName, MNL);
  };
  /// add a new connector to nodetype
  void addConnector(PoseQ * newCon, const char * conName)
  {
    if (conCnt < MCC)
    {
      con[conCnt] = *newCon;
      strncpy(cn[conCnt], conName, MCNL);
      conCnt++;
    }
  }
  // get pose for this connector
  PoseQ getPose(const char * name)
  {
    int i;
    PoseQ p;
    p.set(radius, 0.0, 0.0);
    //
    for (i = 0; i < conCnt; i++)
    {
      if (strcmp(cn[i], name) == 0)
      { // assumes nodetype is a circle
        // square is not handled
        p = con[i].getPoseToMapPose(p);
      }
    }
    return p;
  }
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class Node;

class Edge
{
public:
  /**
   * assumed length - not used */
  double length;
  static const int MCL = 10;
  static const int MNL = 200;
  /// name of start node 
  char name1[MNL];
  /// connector name in start node
  char conn1[MCL];
  /// name of end node 
  char name2[MNL];
  /// connector name in end node
  char conn2[MCL];
  /// start node
  Node * node1;
  /// end node
  Node * node2;
  /// next edge in edge list
  Edge * next;
public:
  Edge()
  {
    node1 = NULL;
    node2 = NULL;
    next = NULL;
  };
  ~Edge()
  {
    if (next != NULL)
    {
      delete next;
      next = NULL;
    }
  };
  /// set node
  bool set(Node * thisNode, const char * start, const char * end, double lng);
  /// make connections to nodes
  bool setNodes(Node * parent);
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class Node
{
public:
  /// position of node in parent node
  PoseQ pose;
  /// parent node
  Node * parent;
  /// sister node
  Node * sister;
  /// child node
  Node * child;
  /// node type
  Nodetype * cons;
  /// edge list
  Edge * edge;
  static const int MNL = 100;
  char name[MNL];
    //
public:
  /// constructor
  Node()
  {
    parent = NULL;
    sister = NULL;
    child = NULL;
    edge = NULL;
    name[0] = '\0';
  };
  ~Node()
  {
    remove();
  }
  /// remove graph
  void remove()
  {
    if (child != NULL)
      delete child;
    child = NULL;
    if (sister != NULL)
      delete sister;
    sister = NULL;
  }
  /// add child node
  void addChild(Node * theChild, Nodetype * theType)
  {
    theChild->sister = child;
    child = theChild;
    child->parent = this;
    child->cons = theType;
  }
  /// add edge
  void addEdge(const char * start, const char * end, double lng)
  {
    Edge * e;
    e = new Edge();
    e->set(this, start, end, lng);
    e->next = edge;
    edge = e;
  }

  /// set node
  void set(PoseQ origin, const char * nodeName)
  {
    pose = origin;
    strncpy(name, nodeName, MNL);
  }
  /// get pose in map coordinates
  PoseQ getMapPose(PoseQ localPose)
  {
    PoseQ result;
    //
    result = pose.getPoseToMapPose(localPose);
    if (parent != NULL)
      // also parents coordinates
      result = parent->getMapPose(result);
    //
    return result;
  }
  /// get the map pose for this connector 
  PoseQ getMapPose(const char * connector);
  /// get a pointer to the node with this name
  Node * getChild(const char * ofName)
  {
    Node * n = child;
    while (n != NULL)
    {
      if (strcmp(n->name, ofName) == 0)
        break;
      n = n->sister;
    }
    return n;
  }
  /// connect edges to nodes
  bool connectEdges();
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class Graph
{
public:
  /// parent graph node
  Node rootNode;
  static const int MNT = 20;
  /// assumes all node types are unique in name
  Nodetype ntyp[MNT];
  /// number of defined node types
  int ntypCnt;
  /// maximum length of name
  static const int MNL = 100;
  /// name of the graph
  char name[MNL];
private:
  Node * currentNode;
  Nodetype * currentType;
public:
  /// constructor
  Graph();
  /// xpat start element handler
  static void startGraphTag(void *userData, const XML_Char *el, const XML_Char **attr);
  /// expat end element handler
  static void endGraphTag(void *userData, const XML_Char *el);
  /// load simple graph
  bool graphload(FILE * fptr);
  /// find node type from name
  Nodetype * findNodeType(const char * name)
  {
    int i;
    Nodetype * nt = NULL;
    for (i = 0; i < ntypCnt; i++)
    {
      if (strcmp(ntyp[i].name, name) == 0)
      {
        nt = &ntyp[i];
        break;
      }
    }
    return nt;
  }
  /// addChild to current node and set current node to this child
  void addChild(Node * newChild, const char * nodetype)
  {
    Nodetype * nt;
    nt = findNodeType(nodetype);
    currentNode->addChild(newChild, nt);
    currentNode = currentNode->child;
  };
};

///////////////////////////////////////////////


#endif
