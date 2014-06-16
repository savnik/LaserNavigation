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
#include "ufuncnear.h"
#include <urob4/uresposehist.h>

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncNear' with your classname */
  return new UFuncNear();
}
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

bool UFuncNear::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  const int MVL = 30;
  char value[MVL];
  ULaserData * data;
  UResPoseHist * odo;
  UPoseTime pose;
  ULaserDevice * dev;
  //
  int i;
  double r;
  double minRange; // min range in meter
  double minAngle = 0.0; // degrees
  double d1 = 0.25, d2, h;
  bool gotHeading = false;
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  gotHeading = msg->tag.getAttDouble("heading", &d2, 5.0);
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "NEAR");
    sendText(msg, "--- available NEAR options\n");
    sendText(msg, "help            This message\n");
    sendText(msg, "fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
    sendText(msg, "device=N        Laser device to use (see: SCANGET help)\n");
    sendText(msg, "heading=B       Get historic heading back from 0.25m to B meters from current position\n");
    sendText(msg, "see also: SCANGET and SCANSET\n");
    sendHelpDone(msg);
  }
  else
  { // do some action and send a reply
    data = getScan(msg, (ULaserData*)extra, false, &dev);
    odo = (UResPoseHist *) getStaticResource("odoPose", true);
    if (gotHeading)
    {
      if (odo != NULL)
      {
        pose = odo->getNewest();
        h = odo->getHistHeading(d1, d2, &pose, NULL);
        snprintf(reply, MRL, "<near histHeading=\"%g\"/>\n", h);
        sendMsg(reply);
      }
      else
        sendWarning("no odometry pose history");
    }
    //
    if (data->isValid())
    { // make analysis for closest measurement
      minRange = 1000.0;
      for (i = 0; i < data->getRangeCnt(); i++)
      { // range are stored as an integer in current units
        r = data->getRangeMeter(i);
        if ((r < minRange) and (r >= 0.020))
        { // less than 20 units is a flag value for URG scanner
          minRange = r;
          minAngle = data->getAngleDeg(i);
        }
      }
      /**
      "Normal" XML reply format */
/*      snprintf(reply, MRL, "<%s range=\"%g\" azimuth=\"%g\" x=\"%g\" y=\"%g\" today=\"true\"/>\n",
               msg->tag.getTagName(), minRange, minAngle,
               cos(minAngle * M_PI / 180.0) * minRange,
               sin(minAngle * M_PI / 180.0) * minRange);*/
      /**
      SMRDEMO reply format */
      snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" />\n",
               minRange, minAngle /* in degrees */,
               cos(minAngle * M_PI / 180.0) * minRange,
               sin(minAngle * M_PI / 180.0) * minRange);
      // send this string as the reply to the client
      sendMsg(reply);
      //
      // paint line from robot to nearpoint as polygon (if polygon plugin is loaded
      if (getStaticResource("poly", false, false) != NULL and odo != NULL)
      {
        odo = (UResPoseHist *) getStaticResource("odoPose", true);
        if (odo != NULL)
        {
          UPosition mp(cos(minAngle * M_PI / 180.0) * minRange,
                       sin(minAngle * M_PI / 180.0) * minRange);
          // get scanner pose on robot
          UPosRot scannerPose = dev->getDevicePose();
          // get near position in robot coordinates
          UMatrix4 m4 = scannerPose.getRtoMMatrix();
          mp = m4 * mp;
          // get pose of robot
          pose = odo->getPoseAtTime(data->getScanTime());
          // convert near position to odometry coordinates
          mp = pose.getPoseToMap(mp);
          sendToPolyPlugin("nearpoint", pose.x, pose.y, mp.x, mp.y);
        }
      }
    }
    else
      sendWarning("No scandata available");
  }
  // return true if the function is handled with a positive result
  return true;
}

#include <ugen4/upolygon.h>

void UFuncNear::sendToPolyPlugin(const char * lineName, double x1, double y1, double x2, double y2)
{
  int n;
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UVariable vCoo;
  UDataBase * db, *dbr;
  bool isOK;
  UPolygon40 poly;
  // delete old value - if any
  vs.setValues(lineName, 0, true);
  par[0] = &vs;
  dbr = &vr;
  isOK = callGlobalV("poly.del", "s", par, &dbr, &n);
  // set polygon coordinate system to odometry (0=odo, 1=map, 3=utm)
  vCoo.setDouble(0.0);
  par[2] = &vCoo;
  poly.add(x1, y1);
  poly.add(x2, y2);
  poly.color[0] = 'r'; // color
  poly.color[1] = '2'; // two pixels wide
  poly.color[2] = '-'; // no marker
  poly.color[3] = 'd'; // default line style

  db = &poly;
  par[1] = (UVariable *) db;
  isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
  if (not isOK)
    printf("UFuncNear::sendToPolyPlugin: polygon plug-in not found\n");
}

