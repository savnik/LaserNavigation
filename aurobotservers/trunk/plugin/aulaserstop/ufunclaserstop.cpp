/***************************************************************************
 *   Copyright (C) 2014 by DTU (Peter J. Savnik S113556)                   *
 *   s113556@student.dtu.dk                                                *
 *                                                                         *
 *   				                                            *
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

#ifdef OPENCV2
#include <legacy/compat.hpp>
#endif

#include <urob4/usmltag.h>
#include <cstdlib>
#include <ucam4/ufunctioncambase.h>

/**
Laser scanner plugin - stop robot in something is infront of the robot
*/


class UFuncLaserStop : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
 
 public:
   
   /** CONSTRUCTOR */
   UFuncLaserStop()
   {
     setCommand("laserstop", "laserstop", "Laser based stop function (compiled " __DATE__ " " __TIME__ ")");
     // create global variables
     createBaseVar();
     // initialize local variables
     
   };
   
   /** Destructor - to delete the resource (etc) when finished */
   virtual ~UFuncLaserStop()
   { // possibly remove allocated variables here - if needed
   }
 
   /**
   Handle incomming command
   Must return true if the function is handled -
   otherwise the client will get a 'failed' reply */
   virtual bool handleCommand(UServerInMsg * msg, void * extra)
   { // handle command(s) send to this plug-in
     // check for parameters - one parameter is tested for - 'help'
     // the help value is ignored, e.g. if help="bark", then
     // the value "bark" will be in the 'helpValue' string.
     bool ask4help;
     const int MVL = 50;
     char val[MVL];
     int camDevice = -1;
     bool debug = true; // default is debug on
     
     ask4help = msg->tag.getAttValue("help", val, MVL);
     if (not ask4help)
     { // get all other parameters
       msg->tag.getAttValueInt("device", &camDevice);
       //gotImg = msg->tag.getAttValueInt("img", &imgPoolNum);
       msg->tag.getAttValueBool("debug", &debug, true);
       //msg->tag.getAttValueBool("smrcl", &smrcl, true);
       //msg->tag.getAttValueBool("blue", &gotBlue, true);
     }
     // ask4help = false, if no 'help' option were available.
     if (ask4help)
     { // create the reply in XML-like (html - like) format
       sendHelpStart("LaserStop");
       sendText("--- available Laser Stop options\n");
       sendText("device=X          Use this laser device\n");
       sendText("debug=false       More images and print on server console (def=true)\n");
       sendText("smrcl             Format the reply for MRC (<vision vis1=\"x.x\" vis2=\"y.y\" .../>\n");
       sendText("help              This message\n");
       sendHelpDone();
       sendInfo("done");
       
     } 
     
     return true;
   }
 
 protected:
  
  
};
