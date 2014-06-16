/****************************************************************************
 *   This file originates from the pDetection ulmsserver-plugin         *
 *   by DMZ - Daniel Muhle-Zimino,s011359@student.dtu.dk                    *
 *   The below is part of the original plugin, modified to create c++       *
 *   objects, instead of writing data to files as it did originally.        *
 *   Modifications by: MV - Mikkel Viager, s072103@student.dtu.dk           *
 ****************************************************************************/

#include "zimino_pDetection.h"
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*  The function is given a set of laserscan-data,
    and creates objects for each leg detected.      */

bool handlePeopleDetection (ULaserData * pushData, std::vector<std::vector<UPosition> > * zLegsAsPoints)
{
  //saves two files in the given directories if true
  bool DEBUG = false;
  if(DEBUG) printf("zimino:pDetection is in DEBUG-mode (a lot of output on console)");

  // DEBUG
  char saveDir[100] = "/home/mikkel/Dropbox/Bachelor/zimino_scanfiles/";
  char saveFile[100] = "/home/mikkel/Dropbox/Bachelor/zimino_scanfiles/";
  // DEBUG END

  ULaserData *data = pushData;

  int r, i, j, k;
  int minR, minR1;		// range in range units
  int counter1 = 0;
  int cou1 = 0;
  int cou2 = 0;
  const int Laserprecision = 181;
  int clausterCount, tjek;
  int testCount = 0;
  int count = 0;
  int testNum;
  int tester = 0;
  int sorted = 0;
  int finalCount = 0;

  // int finalKlyngeAnt[Laserprecision];
  int longestDistNum = 0;
  int klyngeLength[Laserprecision];
  int finalKlyngeSeperator[Laserprecision];
  double smallestMiddleVal;
  double shortest1 = 0;
  double shortest2 = 0;
  double maxMeasureDist = 0;
  double maxDistClausters = 0;
  int finalKlynge[Laserprecision];
  int counterAll = 0;
  double pPIFR[999];		// max registreringer i personindex kan

  // volde probs
  double pPIFAngle[999];	// max registreringer i

  // personindex kan volde probs
  double pPIFRAll[999];
  double pPIFAngleAll[999];
  double personRange = 0.0;
  double personTestDist = 0.0;
  double minDistCal = 0;
  double maxDistCal = 0;
  double xCoord, yCoord, xPrevCoord, yPrevCoord;
  double aTempCal = 0.0;
  double bTempCal = 0.0;
  double klyngeX[Laserprecision][Laserprecision];
  double klyngeY[Laserprecision][Laserprecision];
  double klyngeRange[Laserprecision][Laserprecision];
  double aCal2[181];
  double bCal2[181];
  double shortest = 4.0;
  double sumXY, sumX, sumY, sumXX;
  double longestDist = 0.0;
 
  double tempDist;
  unsigned long serial;
  float closestMeasurement = 1000;

  // saveFile = "All";
  // SIKRE AT KLYNGE ER T�MT
  // Ensure the variable for cluster-mesurements are empty 
  for (i = 0; i < Laserprecision; i++)
  {
    for (j = 0; j < Laserprecision; j++)
    {
      klyngeX[i][j] = 0;
      klyngeX[i][j] = 0;
      klyngeRange[i][j] = 0;
    }
  }
  
  //DEBUG
  // Files to write to
  FILE *FileThirdSort = NULL;
  FILE *FileUnsorted = NULL;
  if(DEBUG){
      // The stringconcatination ensures that the files are
      // created/overwritten in the correct path
      FileThirdSort = fopen (strcat (saveDir, "3sort.txt"), "w");
      strcpy (saveDir, saveFile);
      FileUnsorted = fopen (strcat (saveDir, "unsorted.txt"), "w");
      strcat (saveFile, "default");
      strcat (saveFile, ".txt");
  }
  //DEBUG END

    // Ensures that scandata is valid
    if (data->isValid ())
    {

      // make analysis for closest measurement
      minR = 100000;
      minR1 = 100000;
      serial = data->getSerial ();

      // gets data scanned
      for (i = 0; i < data->getRangeCnt (); i++)
      {
	// range are stored as an integer in current units
	r = *data->getRange (i);
	// test if the object is the closer than the given max
	// distance to object set
	if (r < minR)
	{
	  count = count + 1;
	}
	// Person scan tests up to 3 meters from robot
	personTestDist = 3.00;

	// Converts data to meter-format
	switch (data->getUnit ())
	{
	  case 0:
	    personRange = double (r) * 0.01;

	    break;		// cm
	  case 1:
	    personRange = double (r) * 0.001;

	    break;		// mm
	  case 2:
	    personRange = double (r) * 0.1;

	    break;		// 10cm
	  default:
	    personRange = 20.0;
	}
	// Finding the closest object to robot
	if (personRange < closestMeasurement)
	{
	  closestMeasurement = personRange;
	}
	// ////////////
	// Data about pPIF (possible Person In Front)
	// PossiblepersonInFront Range
	// Saves all data in this variable
	pPIFRAll[counterAll] = personRange;

	// PossiblePersonInFront Angle
	pPIFAngleAll[counterAll] = data->getAngleDeg (i);

	// Updating counter 
	counterAll++;

	// Saves data that lies between reasonable range from
	// laserscanner in this variable 
	if ((personRange < personTestDist) and (r >= 20))
	{
	  // PossiblepersonInFront Range
	  pPIFR[counter1] = personRange;
	  // PossiblepersonInFront Angle
	  pPIFAngle[counter1] = data->getAngleDeg (i);
	  // counter update
	  counter1++;
	}
      }

      for (i = 0; i < counter1; i++)
      {
	if (i > 0)
	{
	  // Transform data inside the reasonable range to x and 
	  // 
	  // y-coordinates
	  xCoord = cos (pPIFAngle[i] * M_PI / 180.0) * pPIFR[i];
	  yCoord = sin (pPIFAngle[i] * M_PI / 180.0) * pPIFR[i];
	  xPrevCoord = cos (pPIFAngle[i - 1] * M_PI / 180.0) * pPIFR[i - 1];
	  yPrevCoord = sin (pPIFAngle[i - 1] * M_PI / 180.0) * pPIFR[i - 1];
	  aTempCal = 0.0;
	  bTempCal = 0.0;

	  // The resonable distance between the legs is decided
	  // from the the distance to the mesurements (0.0175
	  // meters in each side pr. meter to the robot plus a
	  // bufferdistance of 2centimeters, see calculations)
	  maxMeasureDist = ((0.0175 + 0.0175) * pPIFR[i] + 0.02);

	  // test two related mesurements
	  if (sqrt
	      (pow ((xCoord - xPrevCoord), 2) +
	       pow ((yCoord - yPrevCoord), 2)) < maxMeasureDist)
	  {

	    // Inserts first element in cluster
	    if (cou2 == 0)
	    {
	      // Converted to real x and y-coordinates
	      klyngeX[cou1][cou2] = -yPrevCoord;
	      klyngeY[cou1][cou2] = xPrevCoord;
	      klyngeRange[cou1][cou2] = pPIFR[i - 1];
	      cou2++;
	    }
	    // Inserts current element in cluster 
	    klyngeX[cou1][cou2] = -yCoord;
	    klyngeY[cou1][cou2] = xCoord;
	    klyngeRange[cou1][cou2] = pPIFR[i];
	    cou2++;
	  }
	  // Does not belong to this cluster, and create a new
	  // one.
	  else if (cou2 > 0)
	  {
	    // The length of cluster
	    klyngeLength[cou1] = cou2;
	    cou1++;
	    cou2 = 0;
	  }
	}
      }

      // Close last cluster
      if (cou2 > 0)
      {
	// The length of cluster
	klyngeLength[cou1] = cou2;
	cou1++;
	cou2 = 0;
      }
      // Logistics calculation of linear regression
      clausterCount = 0;
      testCount = 0;
      sorted = 0;


      // //// TJEK OP P� DENNE ////// 
      // // Skal nok ikke bruges s� rettelse udelades 
      while (klyngeX[clausterCount][0] != 0 or klyngeY[clausterCount][0] != 0)
      {
	if (klyngeX[clausterCount][2] != 0 or klyngeY[clausterCount][2] != 0)	// nr 
	{
	  sumXY = 0;
	  sumX = 0;
	  sumY = 0;
	  sumXX = 0;

	  while (klyngeX[clausterCount][testCount] !=
		 0 and klyngeY[clausterCount][testCount] != 0)
	  {
	    sumXY +=
	      klyngeX[clausterCount][testCount] * klyngeY[clausterCount][testCount];
	    sumX += klyngeX[clausterCount][testCount];
	    sumY += klyngeY[clausterCount][testCount];
	    sumXX += pow (klyngeX[clausterCount][testCount], 2);

	    testCount++;
	  }

	  aCal2[clausterCount] =
	    ((testCount * sumXY) - (sumX * sumY)) / ((testCount * sumXX) - pow (sumX, 2));
	  bCal2[clausterCount] = (sumY - (aCal2[clausterCount] * sumX)) / testCount;
	  testCount = 0;
	  clausterCount++;
	}
	else
	{
	  sorted++;
	  clausterCount++;
	}
      }

      for (i = 0; i < cou1; i++)
      {
	if (klyngeLength[i] > 2)
	{
	  // Minumum distance of leg-width mesured. Once again
	  // the variable of 0.0175meters is used to to
	  // calculate the buffer-distance and a standard
	  // leg-with of 8centimeters
	  minDistCal = 0.08 - klyngeRange[i][0] * 0.0175 * 2;
	  maxDistCal = 0.35;
	  tjek = 0;
	  // Absolut shortest leg-width possible
	  shortest = 4.0;

	  // testing parameters for the comming tests 
	  bool test1 = false;
	  bool test2 = false;
	  bool test3 = false;
	  bool test4 = false;

	  // This switch makes for special-cases for specified
	  // cluster-length. For example if clusters with 3 or 4 
	  // 
	  // mesurements should be tested different than others
	  // because of the few mesurements.. 
	  switch (klyngeLength[i])
	  {
	    default:
	      double buf = 0.005 * klyngeRange[i][(int) floor (klyngeLength[i] / 2)];

	      // minimum width of cluster in meters
	      if (sqrt
		  (pow
		   ((klyngeX[i][klyngeLength[i] - 1] -
		     klyngeX[i][0]),
		    2) + pow ((klyngeY[i][klyngeLength[i] - 1] -
			       klyngeY[i][0]), 2)) > minDistCal)
	      {
		test2 = true;
	      }
	      // maximum width of cluster
	      if (sqrt
		  (pow
		   ((klyngeX[i][klyngeLength[i] - 1] -
		     klyngeX[i][0]),
		    2) + pow ((klyngeY[i][klyngeLength[i] - 1] -
			       klyngeY[i][0]), 2)) < maxDistCal)
	      {
		test3 = true;
	      }
	      // cluster-width in mesurements is an equal number
	      if (klyngeLength[i] % 2 == 0)
	      {
		if (((klyngeRange[i]
		      [(int) klyngeLength[i] / 2 - 1]) <
		     klyngeRange[i][0]
		     and (klyngeRange[i]
			  [(int) klyngeLength[i] / 2 - 1] <
			  klyngeRange[i][klyngeLength[i] - 1]))
		    or ((klyngeRange[i]
			 [(int) klyngeLength[i] / 2]) <
			klyngeRange[i][0]
			and (klyngeRange[i]
			     [(int) klyngeLength[i] / 2] <
			     klyngeRange[i][klyngeLength[i] - 1])))
		{
		  test1 = true;
		}
		else
		{}
	      }
	      else
	      {
		// hvis kun 3 ekstra tjek p� afstanden mellem
		// punkterne

		// If cluster consist of 3 mesurements, a
		// further check on the distance between
		// mesurements is done
		if (klyngeLength[i] == 3)
		{

		  if ((klyngeRange[i]
		       [(int) floor (klyngeLength[i] / 2)] -
		       buf < klyngeRange[i][0])
		      and (klyngeRange[i]
			   [(int) floor (klyngeLength[i] / 2)]
			   - buf < klyngeRange[i][klyngeLength[i] - 1]))
		  {
		    test1 = true;
		  }
		  else
		  {}
		}
		// if width is not equal then it defines which 
		// 
		// mesurements in the middle is closest
		// related in meters (decides the middle of a
		// possible leg)
		else
		{
		  if (sqrt
		      (pow
		       (klyngeX[i][klyngeLength[i] - 1] -
			klyngeX[i][0],
			2) + pow (klyngeY[i][klyngeLength[i] -
					     1] - klyngeY[i][0], 2)) < 0.22)
		  {
		    for (j = -1; j <= 1; j++)
		    {
		      if (klyngeRange[i]
			  [(int) floor (klyngeLength[i] / 2) + j] < shortest)
			shortest = klyngeRange[i][(int) floor (klyngeLength[i] / 2)];
		    }
		    if ((shortest <
			 klyngeRange[i][0]) and (shortest <
						 klyngeRange[i][klyngeLength[i] - 1]))
		    {
		      test1 = true;
		    }
		    else
		    {}
		  }
		  else
		  {}
		}

	      }

	      // If the previous test sucseeded and cluster
	      // consist of more than 6 mesurements it could be
	      // a possible pair of legs
	      if (test1 == false and klyngeLength[i] > 6)
	      {	
		for (j = 1; j < klyngeLength[i] - 1 - 1; j++)
		{

		  tempDist =
		    sqrt (pow
			  ((klyngeX[i][j] -
			    klyngeX[i][j + 1]),
			   2) + pow ((klyngeY[i][j] - klyngeY[i][j + 1]), 2));

		  
		  // divides a possible leg-pair where the
		  // distance between the measurements i
		  // largest. 
		  if (tempDist > longestDist)
		  {
		    // divide between j and j+1
		    longestDist = tempDist;
		    longestDistNum = j;
		  }
		}
		// Resets distance-variables to a nonpossible
		// large distance
		shortest1 = 4.0;
		shortest2 = 4.0;
		tester = 0;
		
		// test if the one of the divided clusters is
		// smaller than 3 mesurements or 
		if (klyngeLength[i] - longestDistNum < 3)
		  tester++;;
		if (longestDistNum < 3)
		  tester++;
		// if one side, of the cluster dived, length
		// equal 3 mesurements
		if (longestDistNum == 3)
		{
		  // Test if middlepoints of cluster is
		  // closer to robot. The described curved
		  // line earlier.
		  if (klyngeRange[i][1] <
		      klyngeRange[i][0] and klyngeRange[i][1] < klyngeRange[i][2])
		    tester++;		  
		}
		// if the other side, of the cluster dived,
		// length equal 3 mesurements
		if (klyngeLength[i] - longestDistNum == 3)
		{
		  // Test if middlepoints of cluster is
		  // closer to robot. The described curved
		  // line earlier.
		  if (klyngeRange[i][klyngeLength[i] - 2] <
		      klyngeRange[i][longestDistNum +
				     1] and
		      klyngeRange[i][klyngeLength[i] - 2] <
		      klyngeRange[i][klyngeLength[i] - 1])
		    tester++;
		}
		// test if the first divided cluster have a
		// width larger than three
		if (longestDistNum > 3)
		{
		  // forloop that test every value i cluster 
		  // 
		  // except the first and last regarding
		  // curved regression
		  smallestMiddleVal = 4.0;
		  for (k = 1; k < longestDistNum - 1; k++)
		  {
		    if (klyngeRange[i][k] < smallestMiddleVal)
		      smallestMiddleVal = klyngeRange[i][k];
		  }
		  // test up on the closest middleval in the 
		  // 
		  // cluster 
		  if (smallestMiddleVal <
		      klyngeRange[i][0] and smallestMiddleVal <
		      klyngeRange[i][longestDistNum - 1])
		    tester++;
		}
		// test if the second divided cluster have a
		// width larger than three
		if (klyngeLength[i] - longestDistNum > 3)
		{
		  smallestMiddleVal = 4.0;
		  // forloop that test every value i cluster 
		  // 
		  // except the first and last regarding
		  // curved regression

		  for (k = longestDistNum + 2; k < klyngeLength[i] - 1; k++)
		  {
		    if (klyngeRange[i][k] < smallestMiddleVal)
		      smallestMiddleVal = klyngeRange[i][k];
		  }
		  // test up on the closest middleval in the 
		  // 
		  // cluster 
		  if (smallestMiddleVal <
		      klyngeRange[i][longestDistNum +
				     1] and smallestMiddleVal <
		      klyngeRange[i][klyngeLength[i] - 1])
		    tester++;
		}

		if (tester == 2)
		{
		  test4 = true;
		}
		else
		{}


		}
	      // n�dtest tjekker op p� de to punkter ved siden
	      // af


	      break;
	  }
	  
	  // 
	  if (test3 == true and test2 == true)	// and
	  {
	    // saves internal the number and amount of
	    // measurements to the final decision about
	    // person-detection
	    // if the first test is passed directly insertion 
	    if (test1 == true)
	    {
	      finalKlynge[finalCount] = i;
	      finalCount++;
	    }
	    // if the first test is passed directly insertion 
	    else if (test4 == true)	// test4==true
	    {

	      // I KNOW::::::::: EASY FIX WITH FOLLOWUP evt
	      // ren inds�ttelse i fil p� dette tidspunkt
	      // eller endda hurtigere
	      finalKlynge[finalCount] = i;
	      finalKlyngeSeperator[finalCount] = 1;	// longestDistNum; 
	      finalCount++;
	    }
	    
	    testNum = 1;
	    // skud p� maks benbredde og min benbredde 10 og
	    // 40cm (et henholdsvis 2 ben 
	    // desuden antages det at et ben er rundt dvs
	    // robotten registrerer en person som buet
	  }
	}
      }

      // HUSK AT �NDRE 180 TILBAGE TIL LASERPRECISION 
      int z;

      //DEBUG
      if(DEBUG){
          // Save data for every degree mesured
          for (z = 0; z < 180; z++)
          {
            // Saves data fullfilling all tests
            fprintf (FileUnsorted, "%g	%g \n",
                     -sin (pPIFAngleAll[z] * M_PI / 180.0) *
                     pPIFRAll[z], cos (pPIFAngleAll[z] * M_PI / 180.0) * pPIFRAll[z]);
          }
      }
      //DEBUG END
      
      maxDistClausters = 0.30;	// fixed size

      for (i = 0; i < finalCount; i++)
      {
	// test the clusters op against each other from left
	// against right
	if (finalKlyngeSeperator[i] == 0 and i < finalCount - 1)
	{
	 
	  for (k = i + 1; k < finalCount; k++)
	  {
	  // Test of the distance between the to clusters
		if (sqrt (pow ((klyngeX[finalKlynge[i]]
			    [klyngeLength[finalKlynge[i]] - 1] -
			    klyngeX[finalKlynge[k]][0]),
			   2) + pow ((klyngeY[finalKlynge[i]]
				      [klyngeLength
				       [finalKlynge[i]] - 1] -
				      klyngeY[finalKlynge[k]][0]), 2)) < maxDistClausters)
	    {
	    }
	  }
	}
      }

      //Added by Mikkel Viager, s072103@student.dtu.dk
      
      //convert saved data to Upositions, and save to the pointvector in the legvector
  
      std::vector<UPosition> tempVec;
      UPosition pos;

      for (i = 0; i < finalCount; i++)
      {
            (*zLegsAsPoints).push_back(tempVec);
            if(DEBUG)printf("\n Leg no. %d\n",i);

            for (j = 0; j < klyngeLength[finalKlynge[i]]; j++)
            {
                //transfer to odometry coordinates
                double odoX = klyngeY[finalKlynge[i]][j];
                double odoY = -klyngeX[finalKlynge[i]][j];

              pos.clear();
              pos.add(odoX,odoY,0);
              (*zLegsAsPoints)[i].push_back(pos); //push to the i'th leg
              if(DEBUG)(*zLegsAsPoints)[i][j].print("point:");
            }         
            
      }

      // end added


      //DEBUG
      if(DEBUG){

          for (i = 0; i < finalCount; i++)
          {
            for (j = 0; j < klyngeLength[finalKlynge[i]]; j++)
            {
                //transfer to odometry coordinates
                double odoX = klyngeY[finalKlynge[i]][j];
                double odoY = -klyngeX[finalKlynge[i]][j];

                // Printes all final elements
              fprintf (FileThirdSort, "%g	%g	%i \n", odoX, odoY, i);
            }

          }
          // Closing files
          fclose (FileThirdSort);
          fclose (FileUnsorted);
      }//DEBUG END
    }
    else{
        printf("ERROR: SCAN DATA IN ZIMINO-FUNC NOT VALID!");
    }
  return true;
}
