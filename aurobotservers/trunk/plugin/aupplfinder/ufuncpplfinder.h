/***************************************************************************
 *   Copyright (C) 2011 by Mikkel Viager and DTU                           *
 *   s072103@student.dtu.dk                                                *
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
 ***************************************************************************/
#ifndef UFUNC_NEARGET_H
#define UFUNC_NEARGET_H

#include <cstdlib>

#include <ulms4/ufunclaserbase.h>
#include <urob4/uresposehist.h>
#include "Leg.h"
#include <ugen4/upolygon.h>

////////////////////////////////////////////////////////

/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/

class UFuncPpl : public UFuncLaserBase
{
public:
  /**
  Constructor */
  UFuncPpl()
  { 
      //create option vars
      varCertaintyMax = addVar("certaintyMax", 5.0, "d", "maximum certainty status the legs can attain");
      varMaxDist = addVar("maxDist", 0.09, "d", "maximum distance difference on two scans allowed for leg to be verified as \"same\"");

      //create vars
      varLegsInView = addVar("legsInView", 0.0, "d", "the number of legs currently in view");
      varPplInView = addVar("pplInView", 0.0, "d", "the number of people currently in view");
      varClosestLegDist = addVar("closestLegDist", 0.0, "d", "distance in meters to the closest leg");
      varClosestLegID = addVar("closestLegID", 0.0, "d", "the ID number of the closest leg");
      varBestLegID = addVar("bestLegID", 0.0, "d", "the ID number of the leg with best certainty");

      //create methods
      addMethod("legInfo", "d", "return array of info for given leg (ID). On the form [ID, timestamp, Xmean, Ymean, Certainty, Certainty_max]");
      addMethod("testLegInfo", "d", "print array of info for given leg (ID). On the form [ID, timestamp, Xmean, Ymean, Certainty, Certainty_max]");

      // set the command (or commands) handled by this plugin
        setCommand( "pplfinder", "pplFinder", "Detects and provides info on people, based on laser scan data");

        UDataDouble d;
        d.setVal(0);
        for (int i = 0; i != 6; i++)
            legInfo.push_back(d);

  }
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  bool compareResultLegs(std::vector<Leg> * prev, std::vector<Leg> * curr);
  bool drawPolygons(int polygons, int option, std::vector<Leg> * legs);
  bool createPolygon(Leg * leg, UPolygon * poly);
  bool robotToOdoCooTransf(std::vector<std::vector<UPosition> > *  points, UResPoseHist * odoPose, UPosition * offset);
  bool calculateVars();
  bool updateVars();
  bool methodCall(const char * name, const char * paramOrder,
                       UVariable ** params,
                       UDataBase ** returnStruct,
                       int * returnStructCnt);
  bool methodCall(const char * name, const char * paramOrder,
            char ** strings, const double * doubles,
            double * value,
            UDataBase ** returnStruct,
            int * returnStructCnt);

private:
    std::vector<Leg> prevLegsInView;
    std::vector<Leg> currLegsInView;
    std::vector<UDataDouble> legInfo;
    
    UResPoseHist * odoPose;
    int ID;                         //next available ID

    // Parameters and parameter pointers (set in .ini file)

    UVariable * varCertaintyMax;
    int certainty_max;              //max value of certaincy

    UVariable * varMaxDist;
    double max_dist;

    // vars and varPointers

    UVariable * varLegsInView;
    double legsInView;
    
    UVariable * varPplInView;
    double pplInView;

    UVariable * varClosestLegDist;
    double closestLegDist;

    UVariable * varClosestLegID;
    double closestLegID;

    UVariable * varClosestLegPos;
    double closestLegPos[2];

    UVariable * varBestLegID;
    double bestLegID;

};

#endif

