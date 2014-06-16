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
#include "ufuncmrcobst.h"

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncNear' with your classname */
  return new UFuncMRCobst();
}
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
#define SMRWIDTH 0.4
bool UFuncMRCobst::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  const int MVL = 30;
  char value[MVL];
  ULaserData * data;
  //
  int i;
  double r;
  double minRange; // min range in meter
  // double minAngle = 0.0; // degrees
  double x,y,d,d1,robotwidth;
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "mrcobst");
    sendText("--- available MRCOBST options\n");
    sendText("width=W         width of sensitive box\n");
    sendText("help            This message\n");
    sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
    sendText("device=N        Laser device to use (see: SCANGET help)\n");
    sendText("see also: SCANGET and SCANSET\n");
    sendHelpDone();
  }
  else
  { // do some action and send a reply
   bool gotwidth = msg->tag.getAttValue("width", value, MVL);
    if (gotwidth) {
        robotwidth=strtod(value, NULL);   
    }
    else {
        robotwidth=SMRWIDTH;
    }
 
    data = getScan(msg, (ULaserData*)extra);
    //
    if (data->isValid())
    { // make analysis for closest measurement
     
      minRange = 1000;
      d=1000;
      for (i = 0; i < data->getRangeCnt(); i++)
      { // range are stored as an integer in current units
        double th;
	r = data->getRangeMeter(i);
        if (r >= 0.020)
        { // less than 20 units is a flag value for URG scanner
          th = data->getAngleRad(i);
	  x=r*cos(th);
	  y=r*sin(th);
	  d1=fabs(y)-robotwidth/2;
	  if (d1<0){
	    d=x;
	  }
	  else {
	    if (d1 > x)
	      d=d1;
	    else
	      d=x;
	      
	    d=1000;  
	 }
	 if (d < minRange)
	   minRange=d; 
	  
	  
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
      snprintf(reply, MRL, "<laser  l3=\"%g\" />\n", minRange);
      // send this string as the reply to the client
      sendMsg(msg, reply);
    }
    else
      sendWarning(msg, "No scandata available");
  }
  // return true if the function is handled with a positive result
  return true;
}

