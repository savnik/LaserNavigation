/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
 *   jca@oersted.dtu.dk                                                    *
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
#include <urob4/usmltag.h>
#include "ufuncgraphplan.h"

#include <iostream>
#include <string.h>
#include "robotinc/planners.h"


#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncNear' with your classname */
  return new UFuncPlan();
}
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


graphtype G;
graphnode nodes[MAX_GRAPH_NODES];
posetype route[MAX_GRAPH_NODES];
int routestart;

UFuncPlan::UFuncPlan()
{ // initialization of variables in class - as needed
  setCommand("findroute resetplanner getpoint addpoint addcon calculatecost", "plan", "auPlan (" __DATE__ " " __TIME__ " by Nils A. Andersen)");
  nodePoly = NULL;
}


UFuncPlan::~UFuncPlan()
{ // possibly remove allocated variables here - if needed
}


void UFuncPlan::createBaseVar()
{
  varPointsInMap = addVar("nodePoints", 0.0, "d", "(r) number of nodes in map");
  varConnectionsInMap = addVar("connectionPoints", 0.0, "d", "(r) number of connection lines defined");;
  varPointsInPlan = addVar("planPoints", "0 0 0", "d", "(r) number of points in found path plan, its cost, and last fetched point");
  varMakePolygons = addVar("makePolylines", 1.0, "d", "(rw) make polygons of nodes, connections and path plan");
  varPlanTime = addVar("planTime", 0.0, "t", "(r) time when plan were calculated (or reset)");
}

///////////////////////////////////////////////////

bool UFuncPlan::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if(msg->tag.isTagA("calculatecost"))
    result = handlecalculatecost(msg);
  else if (msg->tag.isTagA("findroute"))
    result = handleFindRoute(msg);
  else if (msg->tag.isTagA("getpoint"))
    result = handleGetPoint(msg);
  else if (msg->tag.isTagA("addpoint"))
    result = handleAddPoint(msg);
  else if (msg->tag.isTagA("addcon"))
    result = handleAddCon(msg);
  else if (msg->tag.isTagA("resetplanner"))
    result = handleResetPlan(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFuncPlan::handleResetPlan(UServerInMsg * msg) {
  int i;
  if (msg->tag.getAttBool("help", NULL))
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "resetplanner");
    sendText(msg, "Removes all nodes and connections in map\n");
    sendHelpDone(msg);
    return true;
  }
  G.Nnodes=MAX_GRAPH_NODES - 1;
  G.nodes=nodes;
  G.slisthead=-1;
  for (i=0;i<G.Nnodes;i++){
    G.nodes[i].initialized=0;
    G.nodes[i].conlist[0]=-1;
  }
  if (varMakePolygons->getBool())
  {
    if (nodePoly != NULL)
      callVS("poly.del", "plannerNodes");
    // delete also all connections
    callVS("poly.del", "plannerCon*");
    callVS("poly.del", "plannerPath");
  }
  // set global variables
  varPointsInPlan->setInt(0, 0);
  varPointsInPlan->setInt(0, 1);
  varPointsInMap->setInt(0, 0);
  varConnectionsInMap->setInt(0, 0);
  varPlanTime->setTimeNow();
  return true;
}

///////////////////////////////////////////////////

bool UFuncPlan::handlecalculatecost(UServerInMsg * msg) 
{  
  if (msg->tag.getAttBool("help", NULL))
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "calculate cost");
    sendText(msg, "Calculate route cost based on distance (replaces optional cost in addcon)\n");
    sendHelpDone(msg);
    return true;
  }
  calculatecost(&G);
  return true;
}

///////////////////////////////////////////////////

bool UFuncPlan::handleFindRoute(UServerInMsg * msg) {
  const int MVL = 50;
  char val[MVL];
  double cost;
  double startx;
  double starty;
  double endx;
  double endy;
  double id;
  bool startxavailable = false;
  bool startyavailable = false;
  bool endxavailable = false;
  bool endyavailable = false;
  const int MRL = 500;
  char reply[MRL];
  //
  if (msg->tag.getAttBool("help", NULL))
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "findroute");
    sendText(msg, "finds the route between start and end position\n");
    sendText(msg, "startx=SX starty=SY    is start position of route (e.g. current position in map coordinates)\n");
    sendText(msg, "endx=EX   endy=SY      is desired end position (in map coordinates)\n");
    sendText(msg, "id=N                   optional sequence number N returned to MRC\n");
    sendText(msg, "Returnes (on success):\n");
    sendText(msg, "laser l4=routestart    number of points on route (and index to start point)\n");
    sendText(msg, "laser l5=cost          Cost value of route (distance)\n");
    sendText(msg, "laser l9=N             sequence number (N)\n");
    sendText(msg, "If rute not found, then nothing is returned\n");
    sendHelpDone(msg);
    return true;
  }
  id=0;
  if (msg->tag.getAttValue("startx", val, MVL)){
      startx = strtod(val, NULL); startxavailable=true;}
  if (msg->tag.getAttValue("starty", val, MVL)){
      starty = strtod(val, NULL); startyavailable=true;}
  if (msg->tag.getAttValue("endx", val, MVL)){
      endx = strtod(val, NULL); endxavailable=true;}
  if (msg->tag.getAttValue("endy", val, MVL)){
      endy = strtod(val, NULL); endyavailable=true;}
  if (msg->tag.getAttValue("id", val, MVL)){
      id= strtod(val, NULL);}

  //
  if(startxavailable && startyavailable && endxavailable && endyavailable) {
    cost=findroutexy(&G,startx,starty,endx,endy,route,&routestart);
    snprintf(reply, MRL, "<laser l4=\"%d\" l5=\"%g\"  l9=\"%g\" />\n",
          routestart,cost,id);
    printf("<laser l4=\"%d\" l5=\"%g\"  l9=\"%g\" />\n",
          routestart,cost,id);
    // debug
    printf("from %.2fx %.2fy to %.2fx %.2fy\n", startx, starty, endx, endy);
    for (int i=routestart; i >= 0; i--)
    {
      UPose p(route[i].x, route[i].y, route[i].th);
      printf("route p=%d %.2fx %.2fy %.1fdeg\n", i, p.x, p.y, p.h * 180.0/M_PI);
    }
    printf("cost %.2fm\n", cost);
    // debug end
    varPlanTime->setTimeNow();
    varPointsInPlan->setInt(routestart, 0);
    varPointsInPlan->setDouble(cost, 1);
    if (varMakePolygons->getBool()) 
    {
      UPolygon po;// = new UPolygon(3);
      po.setSize(routestart+3);
      po.add(startx, starty);
      for (int i=routestart; i >= 0; i--)
        po.add(route[i].x, route[i].y);
      po.add(endx, endy);
      po.setAsPolyline();
      po.setColor("r2dd");
      callVSCD("poly.setPolygon", "plannerPath", &po, 2);
    }
    // send this string as the reply to the client
    sendMsg(msg, reply);
    return true;
  } else {
    cout<<"inappropriate arguments for findroute\n";
    sendWarning("inappropriate arguments for findroute");
    return false;
  }
}

///////////////////////////////////////////////////

bool UFuncPlan::handleGetPoint(UServerInMsg * msg) {
  const int MVL = 50;
  char val[MVL];
  const int MRL = 500;
  char reply[MRL];
  int p;
  double id;
  double pavailable=false;
  if (msg->tag.getAttBool("help", NULL))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("getPoint");
    sendText("Get point (pose) for found point in calculated route\n");
    sendText("parameters:\n");
    sendText("p=M      is point number on route\n");
    sendText("id=N     is a sequence number returned to MRC\n");
    sendText("Returns (if M is valid):\n");
    sendText("laser l4=M             is point number (M) on route\n");
    sendText("laser l5=X l6=y l7=th  is destination pose for this point\n");
    sendText("laser l9=N             is sequence number (N)\n");
    sendHelpDone();
    return true;
  }
  id=0;
  if (msg->tag.getAttValue("p", val, MVL)){
      p = (int)strtod(val, NULL); pavailable=true;}
  if (msg->tag.getAttValue("id", val, MVL)){
      id= strtod(val, NULL);}
  if(pavailable ) {
    if (p>=0 && p <=routestart){
      snprintf(reply, MRL, "<laser l4=\"%d\" l5=\"%g\"  l6=\"%g\" l7=\"%g\" l9=\"%g\" />\n",
                           p,route[p].x,route[p].y,route[p].th,id);
      // send this string as the reply to the client
      sendMsg(msg, reply);
      if (varMakePolygons->getBool()) 
      {
        UPolygon po;// = new UPolygon(3);
        po.setSize(3);
        po.add(route[p].x, route[p].y);
        po.add(route[p].x, route[p].y);
        po.setAsPolyline();
        po.setColor("b9od");
        callVSCD("poly.setPolygon", "plannerCurrentPoint", &po, 2);
      }
      // show status
      varPointsInPlan->setInt(p, 2);
    }
    return true;
  } else {
    cout<<"inappropriate arguments for getpoint\n";
    sendWarning("inappropriate arguments for getpoint");
    return false;
  }
}

///////////////////////////////////////////////////

bool UFuncPlan::handleAddPoint(UServerInMsg * msg) {
  int pno;
  double x,y;
  double pavailable=false,xavailable=false,yavailable=false;
  if (msg->tag.getAttBool("help", NULL))
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "addpoint");
    sendText(msg, "--- addpoint parameters:\n");
    sendText(msg, "pno=N        is a reference number for the node.\n");
    sendText(msg, "x=X y=Y      is a X,Y position of the node.\n");
    sendHelpDone(msg);
    return true;
  }
  pavailable = msg->tag.getAttInteger("pno", &pno, -1);
  xavailable = msg->tag.getAttDouble("x", &x);
  yavailable = msg->tag.getAttDouble("y", &y);
//   if (msg->tag.getAttValue("pno", val, MVL)){
//       pno = (int)strtod(val, NULL); pavailable=true;}
//   if (msg->tag.getAttValue("x", val, MVL)){
//       x= strtod(val, NULL);xavailable=true;}
//   if (msg->tag.getAttValue("y", val, MVL)){
//       y= strtod(val, NULL);yavailable=true;}
  if(pavailable && xavailable && yavailable && pno> 0 && pno <G.Nnodes-1){
    addp(&G,pno,x,y);
    if (varMakePolygons->getBool())
    {
      if (nodePoly == NULL)
      {
        nodePoly = new UPolygon();
        nodePoly->setSize(MAX_GRAPH_NODES);
        nodePoly->setAsPolyline();
        nodePoly->setColor("n5o ");
      }
      nodePoly->add(x, y);
      callVSCD("poly.setPolygon", "plannerNodes", nodePoly, 2);
    }
    return true;
  } else {
    printf("inappropriate arguments for addpoint\n");
    sendWarning("inappropriate arguments for addpoint");
    return false;
  }
}

///////////////////////////////////////////////////

bool UFuncPlan::handleAddCon(UServerInMsg * msg) {
  int pno1,pno2;
  double cost=0;
  double pavailable=false,pno2available=false;
  //
  if (msg->tag.getAttBool("help", NULL))
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "addcon");
    sendText(msg, "Add a usable connection between node points:\n");
    sendText(msg, "pno1=A pno2=B    marks a valid route from node A to B\n");
    sendText(msg, "cost=D           optional cost associated with connection (e.g. distance)\n");
    sendHelpDone(msg);
    return true;
  }
  pavailable = msg->tag.getAttInteger("pno1", &pno1);
  pno2available = msg->tag.getAttInteger("pno2", &pno2);
  msg->tag.getAttDouble("cost", &cost);
  if(pavailable && pno2available && pno1 >0 && pno1 < G.Nnodes-1 && pno2>0 && pno2 < G.Nnodes-1) {
    addcon(&G,pno1,pno2,cost);
    if (varMakePolygons->getBool()) 
    {
      UPolygon po;// = new UPolygon(3);
      const int MSL=45;
      char s[MSL];
      graphnode * gn1, *gn2;
      po.setSize(3);
      gn1 = &nodes[pno1];
      gn2 = &nodes[pno2];
      if (gn1->initialized and gn2->initialized)
      {
        po.add(gn1->x, gn1->y);
        po.add(gn2->x, gn2->y);
        po.setAsPolyline();
        if (pno1 > pno2)
          // try to show both ways (r or green)
          po.setColor("r1od");
        else
          po.setColor("g1od");
        snprintf(s, MSL, "plannerCon%03d%03d", pno1, pno2);
        callVSCD("poly.setPolygon", s, &po, 2);
      }
      else
      {
        printf("either node %d (%s) or node %d (%s) is not initialized\n",
               pno1, bool2str(gn1->initialized), pno2, bool2str(gn2->initialized));
      }
    }
    return true;
  } else {
    cout<<"inappropriate arguments for addcon  "<< pno1 <<"  "<< pno2 << "\n ";
    sendWarning("inappropriate arguments for addcon");
    return false;
  }
}

// //////////////////////////////////////////////////////
// 
// bool UFuncPlan::callVS(const char * function, const char * stringParam)
// {
//   int n;
//   UVariable * par[3];
//   UVariable vs;
//   UVariable vr;
//   UDataBase *dbr;
//   //
//   vs.setValues(stringParam, 0, true);
//   par[0] = &vs;
//   dbr = &vr;
//   return callGlobalV(function, "s", par, &dbr, &n);
// }
// 
// 
// ///////////////////////////////////////////
// 
// void UFuncPlan::callVSCD(const char * function, const char * strPar, UDataBase * data, int cooSys)
// {
//   int n;
//   UVariable vs;
//   UVariable vCoo;
//   UVariable * par[3] = {&vs, (UVariable *) data, &vCoo} ;
//   UVariable vr; // return value from function
//   UDataBase *dbr; 
//   bool isOK;
//   // put values into parameter list
//   vs.setValues(strPar, 0, true);
//   vCoo.setInt(cooSys);
//   vr.setDouble(-1.0, 0, true);
//   dbr = &vr;
//   isOK = callGlobalV(function, "scd", par, &dbr, &n);
//   if (not isOK or not vr.getBool())
//     printf("UFuncPlan::callVSCD: failed to call '%s(%s, data, %d)'\n", function, strPar, cooSys);
//   //printf("UFuncPlan::callVSCD: debug1\n");
// }
  
  