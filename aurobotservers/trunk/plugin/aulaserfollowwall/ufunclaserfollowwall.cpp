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

// #include <stdio.h>

#include <urob4/usmltag.h>
#include <cstdlib>
#include <ulms4/ufunclaserbase.h>
//#include <ucam4/ufunctioncambase.h>

/**
Laser scanner plugin - follow wall
*/




class UFuncLaserStop : public UFuncLaserBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
 
 public:
   
   /** CONSTRUCTOR */
   UFuncLaserStop()
   {
     setCommand("laserfollowwall", "laserfollowwall", "Laser based follow wall function (compiled " __DATE__ " " __TIME__ ")");
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
     bool result = true;
     ULaserData * data;
     ULaserDevice * lasDev; // pointer to laser device
     
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
      printf("test3\n");
      sendHelpStart("Laser wall follow");
      sendText("--- available options\n");
      sendText("device=X          Use this camera - for position and parameters\n");
      sendText("help              This message\n");
      sendHelpDone();
      sendInfo("done");
      result = true;
     } 
     // Handle all other commands
     else
     {
       // Print out data
       if(msg->tag.getAttValue("dataprint", NULL, 0))
       {
	 ULaserDevice * lasDev; // laser device
         data = getScan(msg, (ULaserData*)extra, false, &lasDev);
         if (data->isValid())
	 {
	   printf("MAX Scan: %d\n",data->getRangeCnt());
	   int i;
	   for(i=0;i<data->getRangeCnt();i=i+1){
	     if(data->getRangeMeter(i)>=1) printf("ID: %d \t%f \t%f \n",i,data->getRangeMeter(i),data->getAngleDeg(i));
	   }
	 }
	 else
	 {
	   sendWarning(msg, "No scandata available");
	 }
       }
       
       // test stop condition
       else if(msg->tag.getAttValue("test", NULL, 0))
       {
	 ULaserDevice * lasDev; // laser device
         data = getScan(msg, (ULaserData*)extra, false, &lasDev);
         
	 if (data->isValid())
	 {
	  printf("MAX Scan: %d\n",data->getRangeCnt());
	  // loop that run for all messurements
	  int i;
	  for(i = 0; i < data->getRangeCnt(); i = i+1)
	  {
	    // Filter out the angles of interrest
	    if((data->getAngleDeg(i) > -30) && (data->getAngleDeg(i) < 30))
	    {
	      printf("ID: %d \t%f \t%f \n",i,data->getRangeMeter(i),data->getAngleDeg(i));
	    }
	  }
	 }
       }   
       
       // MRC
       else if(msg->tag.getAttValue("mrc", NULL, 0))
       {
	 ULaserDevice * lasDev; // laser device
         data = getScan(msg, (ULaserData*)extra, false, &lasDev);
         int objectDetected = 0;
	 
	 if (data->isValid())
	 {
	   
	  // loop that run for all messurements
	  int i;
	  for(i = 0; i < data->getRangeCnt(); i = i+1)
	  {
	    // Filter out the angles of interrest
	    if((data->getAngleDeg(i) >= -30) && (data->getAngleDeg(i) <= 30))
	    {
	      // 0.25 stop if obstacel is closer than 0.25m
	      // remove samples that are closer than 0.06m due to datasheet presision.
	      // Remove samples that are out of index (0).
	      if((data->getRangeMeter(i)<0.25) && (data->getRangeMeter(i) > 0.06)){
		objectDetected = 1;
		//printf("ID: %d \t%f \t%f \n",i,data->getRangeMeter(i),data->getAngleDeg(i));
	      }
	    }
	  }
	  
	  if(objectDetected == 1){
	    sendText("Object in front of SMR!\n");
	    sendMsg("<laser l1=\"1\"/>\n");
	  }
	  else{
	    sendMsg("<laser l1=\"0\"/>\n");
	  }
	 }
	 else{
	   sendWarning(msg, "No scandata available!");
	 }
       }   
       
       else{
	result = false;
       }
     } // end of handle all other commands
     
     return result;
   }
   
   
 
 protected:
  
  
};


#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
  { // called by server to create an object of this type
    /** replace 'UFuncBall' with your classname, as used in the headerfile */
    return new UFuncLaserStop();
  }
#endif