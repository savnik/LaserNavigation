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

#include "ufuncpplfinder.h"
#include "zimino_pDetection.h"
#include "urespplfinder.h"

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncNear' with your classname */
  return new UFuncPpl();
}
#endif

///////////////////////////////////////////////////

bool UFuncPpl::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const bool DEBUG = false;
  certainty_max = varCertaintyMax->getInt(0);
  max_dist = varMaxDist->getDouble(0); // max acceptable distance a leg can move bewteen to consecutive scans
  
  const double MIN_LASER_HEIGHT = 0.10; // in meters
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  const int MVL = 30;
  char value[MVL];
  double * pdvalue;
  double dvalue;
  pdvalue = &dvalue;
  ULaserData * data;
  //UResPoseHist * odoPose;
  UPoseTime pose;
  ULaserPool * lasPool;
  UPosition lasPlacementOffset;
  //
  //int i;
  //double r;
  //double minRange; // min range in meter
  //double minAngle = 0.0; // degrees
  //double d1 = 0.25, d2, h;

  //bool gotHeading = false;

  // vvvvvvvvvvvvvvvvvvvvvvvvvvvv
  std::vector<std::vector<UPosition> > * zLegsAsPoints = new std::vector<std::vector<UPosition> >; //vectors containing vectors of Upositions
  std::vector<Leg> * pPrevLegsInView = &prevLegsInView;
  std::vector<Leg> * pCurrLegsInView = &currLegsInView;

  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  //gotHeading = msg->tag.getAttDouble("heading", &d2, 5.0);
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "pplfinder");
    sendText(msg, "--- available pplfinder options ---\n");
    sendText(msg, "help            This message\n");
    sendText(msg, "run             Run pplfinder; analyze current laserscan, and update all\n");
    sendText(msg, "                 variables and history.                                  \n");
    sendText(msg, "                (Must be run every time you want to update. This is the \n");
    sendText(msg, "                 command you want to \"push\").                         \n");
    sendText(msg, "nearest         Return information for the nearest leg in XML-like format\n");
    sendText(msg, "                 for SMRCL, with the variables;                         \n");
    sendText(msg, "                l0 = <ID>             (the unique ID-number for this leg)\n");
    sendText(msg, "                l1 = <scanID>         (the laser scan ID-number)\n");
    sendText(msg, "                l2 = <timestamp>      (time since plugin load)\n");
    sendText(msg, "                l3 = <Xmean>          (mean X value for points defining this leg)\n");
    sendText(msg, "                l4 = <Ymean>          (mean Y value for points defining this leg)\n");
    sendText(msg, "                l5 = <certainty>      (rises +1 every time the leg is re-discovered)\n");
    sendText(msg, "                                      (falls -1 every time the leg is not re-discovered)\n");
    sendText(msg, "                                      (when this reaches 0, the leg is deleted)\n");
    sendText(msg, "                l6 = <Certainty_max>  (maximum value the certainty can reach)\n");
    sendText(msg, "legInfo=n       Return information on the leg with ID = n (for SMRCL, the\n");
    sendText(msg, "                 same format as for \"nearest\")                          \n");
    sendText(msg, "testLegInfo=n   Write information on the leg with ID = n to the console \n");
    sendText(msg, "                 for testing purposes.                                   \n");
    sendText(msg, "see also: SCANGET and SCANSET\n");
    sendText(msg, "\n");
    sendText(msg, "--- available variables ---\n");
    sendText(msg, "The command \"var pplfinder\" can be used to see help for the variables\n");
    sendText(msg, "You can get/set the variables with the command \"var pplfinder.<variableName>\"\n");
    sendText(msg, "\n");
    sendText(msg, "--- plugin-to-plugin interface ---\n");
    sendText(msg, "Both callGlobal and callGlobalV responses are available with the commands;\n");
    sendText(msg, "\"var call=pplfinder.legInfo(n)\"      (where n is the ID for the desired leg)\n");
    sendText(msg, "\"var call=pplfinder.testLegInfo(n)\"  (where n is the ID for the desired leg)\n");
    sendText(msg, "\n");
    
    sendHelpDone(msg);
  }
  else
  { // do some action and send a reply

      //to get info on placement of laser-scanner
      lasPool = (ULaserPool *)getStaticResource("lasPool", false);
      if(lasPool != NULL)
      {
          lasPlacementOffset = lasPool->getDefDevice()->getDevicePos();
          if(lasPlacementOffset.z < MIN_LASER_HEIGHT)
              printf("WARNING: Your Laser scanner is placed at %fm height, which is too low!\n"
                      "You should place the laser scanner at least %gm above the ground!\n"
                      ,lasPlacementOffset.z, MIN_LASER_HEIGHT);
      }

    data = getScan(msg, (ULaserData*)extra);

      odoPose = (UResPoseHist *) getStaticResource("odoPose", true);
      if (odoPose != NULL)
      {
        pose = odoPose->getNewest();
        //h = odoPose->getHistHeading(d1, d2, &pose, NULL);
        //snprintf(reply, MRL, "<pplfinder histHeading=\"%g\"/>\n", h);
        //sendMsg(reply);
      }
      else
        sendWarning("no odometry pose history");
    //
    if (data->isValid())
    { 
      // vvvvvvvvvvvvvvvvvvvvvvvvvvvv

      if (msg->tag.getAttValue("run", value, MVL))
      {

          //Update prevLegsInView, and clear (make ready) currLegsInView;
          prevLegsInView = currLegsInView;
          currLegsInView.clear();
          
          //printf("RUNNING ZIMINO's PEOPLE_DETECTION\n");

          zLegsAsPoints->clear();
          handlePeopleDetection (data, zLegsAsPoints);
          //transfer from robot to odometry coordinates
          robotToOdoCooTransf(zLegsAsPoints, odoPose, &lasPlacementOffset);

          std::vector<std::vector<UPosition> >::size_type i;
          double time = (double) data->getScanTime().GetDecSec();
          long scanID = data->getSerial();

          for ( i = 0; i != zLegsAsPoints->size(); i++)
          {
              Leg* tempLeg = new Leg( ID+i, scanID, time, (*zLegsAsPoints)[i], DEBUG);
              //Create a new leg with the given ID, time, data and decide if we want debug output
              pCurrLegsInView->push_back((*tempLeg));
          }

          if(ID < 2147483000) //take care of overflow (very unlikely)
              ID += i;
          else
              ID = 0;

          //compare previous and current legs viewed, and store result in currLegsInView
          compareResultLegs(pPrevLegsInView, pCurrLegsInView);

          if(drawPolygons(pCurrLegsInView->size(), 1, pCurrLegsInView)&& DEBUG)
              printf("createPolygons have been successfully run for %d Leg(s)...!\n",pCurrLegsInView->size());
          
      }
      else if(msg->tag.getAttDouble("legInfo", pdvalue, 1.0))
      {         
          UDataBase *pd[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
          UVariable *par;

          UVariable uvar;
          uvar.setValued(*pdvalue); // pass on the variable
          par = &uvar;

          int n = 6;

          bool isOK = callGlobalV("pplfinder.legInfo", "d", &par, pd, &n);

          if(n > 5 && isOK)
          {
               snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       ((UDataDouble*) pd[0])->getVal(),  // Leg ID
                       ((UDataDouble*) pd[1])->getVal(),  // Scan ID
                       ((UDataDouble*) pd[2])->getVal(),  // Timestamp
                       ((UDataDouble*) pd[3])->getVal(),  // Xmean
                       ((UDataDouble*) pd[4])->getVal(),  // Ymean
                       ((UDataDouble*) pd[5])->getVal(),  // Certainty
                       (double) certainty_max);           // Certainty_max
              
          }
          else
          {
              snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       -1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0);
          }
          // send this string as the reply to the client
          sendMsg(reply);
          
      }else if(msg->tag.getAttDouble("nearest", pdvalue, 1.0))
      {
          UDataBase *pd[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
          UVariable *par;

          UVariable uvar;
          uvar.setValued(closestLegID); // pass on the variable
          par = &uvar;

          int n = 6;

          bool isOK = callGlobalV("pplfinder.legInfo", "d", &par, pd, &n);

          if(n > 5 && isOK)
          {
               snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       ((UDataDouble*) pd[0])->getVal(),  // Leg ID
                       ((UDataDouble*) pd[1])->getVal(),  // Scan ID
                       ((UDataDouble*) pd[2])->getVal(),  // Timestamp
                       ((UDataDouble*) pd[3])->getVal(),  // Xmean
                       ((UDataDouble*) pd[4])->getVal(),  // Ymean
                       ((UDataDouble*) pd[5])->getVal(),  // Certainty
                       (double) certainty_max);           // Certainty_max

          }
          else
          {
              snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       -1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0);
          }
          // send this string as the reply to the client
          sendMsg(reply);
      }else if(msg->tag.getAttDouble("best", pdvalue, 1.0))
      {
          UDataBase *pd[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
          UVariable *par;

          UVariable uvar;
          uvar.setValued(bestLegID); // pass on the variable
          par = &uvar;

          int n = 6;

          bool isOK = callGlobalV("pplfinder.legInfo", "d", &par, pd, &n);

          if(n > 5 && isOK)
          {
               snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       ((UDataDouble*) pd[0])->getVal(),  // Leg ID
                       ((UDataDouble*) pd[1])->getVal(),  // Scan ID
                       ((UDataDouble*) pd[2])->getVal(),  // Timestamp
                       ((UDataDouble*) pd[3])->getVal(),  // Xmean
                       ((UDataDouble*) pd[4])->getVal(),  // Ymean
                       ((UDataDouble*) pd[5])->getVal(),  // Certainty
                       (double) certainty_max);           // Certainty_max

          }
          else
          {
              snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       -1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0);
          }
          // send this string as the reply to the client
          sendMsg(reply);
      }else if(msg->tag.getAttDouble("testLegInfo", pdvalue, 1.0))
      {
          UDataBase *pd[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
          UVariable *par;

          UVariable uvar;
          uvar.setValued(*pdvalue); // pass on the variable
          par = &uvar;

          int n = 6;

          bool isOK = callGlobalV("pplfinder.legInfo", "d", &par, pd, &n);


          if(n > 5 && isOK)
          {
               snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       ((UDataDouble*) pd[0])->getVal(),  // Leg ID
                       ((UDataDouble*) pd[1])->getVal(),  // Scan ID
                       ((UDataDouble*) pd[2])->getVal(),  // Timestamp
                       ((UDataDouble*) pd[3])->getVal(),  // Xmean
                       ((UDataDouble*) pd[4])->getVal(),  // Ymean
                       ((UDataDouble*) pd[5])->getVal(),  // Certainty
                       (double) certainty_max);           // Certainty_max
               callGlobalV("pplfinder.testLegInfo", "d", &par, pd, &n);
          }
          else
          {
              snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6 =\"%g\" />\n",
                       -1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0);
          }
          // send this string as the reply to the client
          sendMsg(reply);

      }

      else
        sendDebug ("Command not handled (by me)");
      //return result;
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    }
    else
      sendWarning(msg, "No scandata available");
  }
  
  // return true if the function is handled with a positive result
  return true;
}

//compare previous and current legs viewed, and store result in currLegsInView
bool UFuncPpl::compareResultLegs(std::vector<Leg> * prev, std::vector<Leg> * curr)
{
    //printf("running compare %d times with %d prev. legs\n", curr->size(), prev->size());
    std::vector<Leg>::size_type i,j;
    for (i = 0 ; i!=curr->size(); i++)
    {
        //printf("\nTest on current leg no. %d\n",i);
        int bestMatchAt = -1;              //invalid vector position

        double bestMatchDist = 100.0;      //initial not acceptable distance
        std::vector<Leg>::iterator iter = prev->begin();
        std::vector<Leg>::iterator iterBestMatchAt;

        //go through all legs in
        for (j = 0 ; j != prev->size(); j++ )
        {
            //find distance between this and leg no. i
            double distDiff;

            anyCooPhytagoras((*prev)[j].getXmean(), (*curr)[i].getXmean(),
                    (*prev)[j].getYmean(), (*curr)[i].getYmean(), &distDiff);
            //see if this has a closer distance
            //printf("distdiff = %f\n",distDiff);

            if(distDiff < bestMatchDist)
            {
                bestMatchAt = j;
                iterBestMatchAt = iter;
                bestMatchDist = distDiff;

                iter++;
            }  

        }
        //printf("best match diff: %f\n", bestMatchDist);
        if(bestMatchDist < max_dist && bestMatchAt != -1)
        {
            //printf("found an acceptable match for leg no. %d \n",i);
            
                    //do something with trajectory?
            (*curr)[i].setID((*prev)[bestMatchAt].getID());
            (*curr)[i].setPosHist((*prev)[bestMatchAt].getPosHist());     //save history
            (*curr)[i].addCertainty((*prev)[bestMatchAt].getCertainty(), certainty_max); //add saved certainty
            (*curr)[i].setColor((*prev)[bestMatchAt].getColor());    //save color
            if((*curr)[i].getCertainty() == certainty_max)
                (*curr)[i].setColorChar('k');
            else
                (*curr)[i].setColorChar('b');
            //printf("Detected leg now has certainty %d\n",(*curr)[i].getCertainty());
            
            //remove to indicate that this has been used already
                prev->erase(iterBestMatchAt);
        }else
        {
            //new leg found
        }

    }

    //go through all legs from previously, that was not matched
    for (i = 0 ; i!=prev->size(); i++)
    {
        //printf("Size of prev: %d\n", prev->size());
        (*prev)[i].addCertainty(-1, certainty_max);        //subtract one from certainty

        if((*prev)[i].getCertainty() > 0)   //if leg has a build-up certaincy
        {
            //This leg has been successfully matched before
            //the inability to detect the leg must have been an error.
            //include in current legs, as if this was detected
            curr->push_back((*prev)[i]);
            //printf("Saved undetected Leg (ID %d)! certainty now at %d\n", (*prev)[i].getID(),(*prev)[i].getCertainty());
            //printf("Position was: Xmean = %f, Ymean = %f\n",(*prev)[i].getXmean(), (*prev)[i].getYmean());
        }
    }


    calculateVars();
    updateVars();
    return true;
}


//adapted from copyCellPolys in uresavoid.cpp
bool UFuncPpl::drawPolygons(int polygons, int option, std::vector<Leg> * legs)
{
UResBase * pres;
  int i, n;
  const int MSL = 30;
  char s[MSL];
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UVariable vCoo;
  UDataBase * db, *dbr;
  bool isOK;
  UPolygon40 poly;
  //
  pres = getStaticResource("poly", false, false);
  if (pres != NULL && polygons > 0)
  { // delete old footprint polygons in polygon resource
    snprintf(s, MSL, "legPoly.*");
    vs.setValues(s, 0, true);
    par[0] = &vs;
    dbr = &vr;
    isOK = callGlobalV("poly.del", "s", par, &dbr, &n);
    //
    // set polygon coordinate system to odometry (0=odo, 1=utm, 3=map)
    vCoo.setDouble(0.0);
    par[2] = &vCoo;
    if (option == 1)
    { // all cells

      for (i = -1; i < polygons; i++)
      {
          if(i == -1){
                //create starting point polygon
                //workaround for first polygon offset. Makes count start at 1
                poly.clear();
                poly.add(0, 0, 0.0);
                poly.color[0] = 'w';
          }else
                isOK = createPolygon(&(*legs)[i] ,&poly); //avcg->getCellPoly(i, &poly);
        if (isOK)
        {
          snprintf(s, MSL, "legPoly.%03d", i);
          vs.setValues(s, 0, true);
          db = &poly;
          par[1] = (UVariable *) db;
          isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
          if ((not isOK and i == 0) or not vr.getBool())
            printf("UResAvoid::copyCellPolys: failed to 'poly.setPoly(%s, %d)'\n", s, i);
        } else
            printf("createPolygon failed for polygon no. %d!\n",i);
      }
    }
  
  } else
      printf("failed on getStaticResource, or polygon number invalid!\n");

    return true;
}

//take info in Leg object, and set options in given poly
bool UFuncPpl::createPolygon(Leg * leg, UPolygon * poly)
{
    poly->clear();
    poly->add(leg->getXmean(), leg->getYmean(), 0.0);
    poly->color[0] = (*leg->getColorChar(0));
    poly->color[1] = (*leg->getColorChar(1));
    poly->color[2] = (*leg->getColorChar(2));
    poly->color[3] = (*leg->getColorChar(3));

    return true;

}

//correct given points in robot coordinates to odometry coordinates
bool UFuncPpl::robotToOdoCooTransf(std::vector<std::vector<UPosition> > *  points, UResPoseHist * odoPose, UPosition * offset)
{

    UPose p = odoPose->getNewest();
    double totX,totY;
    double theta = p.h;
    double odoX = p.x;
    double odoY = p.y;
    double offX = offset->x;
    double offY = offset->y;


    std::vector<std::vector<UPosition> >::iterator iter1= points->begin();
    std::vector<UPosition>::iterator iter2;

    for(iter1 = points->begin(); iter1 != points->end(); iter1++)
    {
        for(iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
        {
            totX = iter2->x + offX;
            totY = iter2->y + offY;
            //translation and rotation
            iter2->x = (totX*cos(theta) - totY*sin(theta))+odoX;
            iter2->y = (totX*sin(theta) + totY*cos(theta))+odoY;
        }
    }
    //printf("successfuly translated and rotated\n");
    return true;
}

bool UFuncPpl::calculateVars()
{
    // legsInView
    legsInView = currLegsInView.size();

    // pplInView


    // closestLegDist closestLegID bestLegID
    double closest_dist = 4.0;
    int closest_ID = -1;
    int best_certainty = 0;
    int best_ID = -1;

    UPose p = odoPose->getNewest();
    double odoX = p.x;
    double odoY = p.y;

    std::vector<Leg>::iterator iter;
    
    for (iter = currLegsInView.begin(); iter!=currLegsInView.end(); iter++)
    {
        double legX = iter->getXmean();
        double legY = iter->getYmean();
        double distDiff;

        anyCooPhytagoras(odoX, legX, odoY, legY, &distDiff);
        
        if(distDiff < closest_dist)
        {
            closest_dist = distDiff;
            closest_ID = iter->getID();
        }

        if(iter->getCertainty() > best_certainty)
        {
            best_certainty = iter->getCertainty();
            best_ID = iter->getID();
        }


    }

    if(closest_ID != -1)
    {
        closestLegDist = closest_dist;
        closestLegID = closest_ID;
    }else
        printf("Error finding closest leg. Data not updated");

    if(best_ID != -1)
    {
        bestLegID = best_ID;
    }else
        printf("Error finding best leg. Data not updated");

    return true;
}

bool UFuncPpl::updateVars()
{
    varLegsInView->setValued(legsInView);
    varPplInView->setValued(pplInView);
    varClosestLegDist->setValued(closestLegDist);
    varClosestLegID->setValued(closestLegID);
    varBestLegID->setValued(bestLegID);
    return true;
}

//For use with callGlobalV
bool UFuncPpl::methodCall(const char * name, const char * paramOrder,
                       UVariable ** params,
                       UDataBase ** returnStruct,
                       int * returnStructCnt)
{
  bool result = true;

  // evaluate standard functions
  if ((strcasecmp(name, "legInfo") == 0) and (strcmp(paramOrder, "d") == 0))
  {

      legInfo[0].setVal(0);
      legInfo[1].setVal(0);
      legInfo[2].setVal(0);
      legInfo[3].setVal(0);
      legInfo[4].setVal(0);
      legInfo[5].setVal(0);

      std::vector<Leg>::iterator iter;
        
        for(iter = currLegsInView.begin(); iter != currLegsInView.end(); iter++)
        {
            if (iter->getID() == (int) (*params)->getValued())
            {
                legInfo[0].setVal(iter->getID());           //ID
                legInfo[1].setVal( (double) iter->getScanID()); //scanID
                legInfo[2].setVal(iter->getTimeStamp());    //Timestamp
                legInfo[3].setVal(iter->getXmean());        //Xmean
                legInfo[4].setVal(iter->getYmean());        //Ymean
                legInfo[5].setVal(iter->getCertainty());    //Certainty
            }
        }

      if((int) legInfo[2].getVal() == 0)
      {
          printf("ERROR: a leg with the given ID is not available.\n");
          legInfo[0].setVal(-1);
          legInfo[1].setVal(-1);
          legInfo[2].setVal(-1);
          legInfo[3].setVal(-1);
          legInfo[4].setVal(-1);
          legInfo[5].setVal(-1);
      }

      if(*returnStructCnt > 5)
      {
          returnStruct[0] = &legInfo[0];
          returnStruct[1] = &legInfo[1];
          returnStruct[2] = &legInfo[2];
          returnStruct[3] = &legInfo[3];
          returnStruct[4] = &legInfo[4];
          returnStruct[5] = &legInfo[5];
      }

      //*returnStructCnt = legInfo.size();
  }
  else if((strcasecmp(name, "testLegInfo") == 0) and (strcmp(paramOrder, "d") == 0))
  {
      UDataBase *pd[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
      UVariable *par;

      UVariable uvar;
      uvar.setValued((*params)->getValued()); // pass on the variable(s)
      par = &uvar;

      int n = 6;

      bool isOK = callGlobalV("pplfinder.legInfo", "d", &par, pd, &n);

      if(n > 5 && isOK)
      {
            printf("reply[0] ID = %f\n",((UDataDouble*) pd[0])->getVal());
            printf("reply[1] scanID = %f\n",((UDataDouble*) pd[1])->getVal());
            printf("reply[2] Timestamp = %f\n",((UDataDouble*) pd[2])->getVal());
            printf("reply[3] Xmean = %f\n",((UDataDouble*) pd[3])->getVal());
            printf("reply[4] Ymean = %f\n",((UDataDouble*) pd[4])->getVal());
            printf("reply[5] Certainty = %f\n",((UDataDouble*) pd[5])->getVal());
      }

      *returnStructCnt = 6;
  }
  else
    result = false;
  return result;
}

//For use with callGlobal
bool UFuncPpl::methodCall(const char * name, const char * paramOrder,
            char ** strings, const double * doubles,
            double * value,
            UDataBase ** returnStruct,
            int * returnStructCnt)
{
    bool result = true;

  // evaluate standard functions
  if ((strcasecmp(name, "legInfo") == 0) and (strcmp(paramOrder, "d") == 0))
  {

      legInfo[0].setVal(0);
      legInfo[1].setVal(0);
      legInfo[2].setVal(0);
      legInfo[3].setVal(0);
      legInfo[4].setVal(0);
      legInfo[5].setVal(0);

      std::vector<Leg>::iterator iter;

        for(iter = currLegsInView.begin(); iter != currLegsInView.end(); iter++)
        {
            if (iter->getID() == *doubles)
            {
                legInfo[0].setVal(iter->getID());           //ID
                legInfo[1].setVal( (double) iter->getScanID()); //scanID
                legInfo[2].setVal(iter->getTimeStamp());    //Timestamp
                legInfo[3].setVal(iter->getXmean());        //Xmean
                legInfo[4].setVal(iter->getYmean());        //Ymean
                legInfo[5].setVal(iter->getCertainty());    //Certainty
            }
        }

      if((int) legInfo[2].getVal() == 0)
      {
          printf("ERROR: a leg with the given ID is not available. Returning [-1,-1,-1,-1,-1]\n");
          legInfo[0].setVal(-1);
          legInfo[1].setVal(-1);
          legInfo[2].setVal(-1);
          legInfo[3].setVal(-1);
          legInfo[4].setVal(-1);
          legInfo[5].setVal(-1);
      }

      if(*returnStructCnt > 5)
      {
          returnStruct[0] = &legInfo[0];
          returnStruct[1] = &legInfo[1];
          returnStruct[2] = &legInfo[2];
          returnStruct[3] = &legInfo[3];
          returnStruct[4] = &legInfo[4];
          returnStruct[5] = &legInfo[5];
      }
      *value = 1;
      *returnStructCnt = legInfo.size();
  }
  else if((strcasecmp(name, "testLegInfo") == 0) and (strcmp(paramOrder, "d") == 0))
  {
      UDataBase *pd[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
      UVariable *params;
      
      UVariable uvar;
      uvar.setValued(*doubles); // pass on the variable(s)
      params = &uvar;

      int n = 6;


      bool isOK = callGlobalV("pplfinder.legInfo", "d", &params, pd, &n);

      if(n > 5 && isOK)
      {
            printf("reply[0] ID = %f\n",((UDataDouble*) pd[0])->getVal());
            printf("reply[1] scanID = %f\n",((UDataDouble*) pd[1])->getVal());
            printf("reply[2] Timestamp = %f\n",((UDataDouble*) pd[2])->getVal());
            printf("reply[3] Xmean = %f\n",((UDataDouble*) pd[3])->getVal());
            printf("reply[4] Ymean = %f\n",((UDataDouble*) pd[4])->getVal());
            printf("reply[5] Certainty = %f\n",((UDataDouble*) pd[5])->getVal());
      }



      *returnStructCnt = 6;
      *value = 2;
  }
  else
    result = false;
  return result;
}
