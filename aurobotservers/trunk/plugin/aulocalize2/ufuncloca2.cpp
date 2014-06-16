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
#include "ufuncloca2.h"
#include <utils/localizationutils.h>

#include <iostream>
#include <string.h>
//#include <mhf/iau_ukf_eigen.hpp>
//#include <boost/foreach.hpp>

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
	/** replace 'UFuncNear' with your classname */
	return new UFuncLoca2();
}
#endif

/////////////////////////////////////////////////////////////////

UFuncLoca2::UFuncLoca2() { // initialization of variables in class - as needed
	setCommand(
			"loca",
			"loca",
			"auLocalize (" __DATE__ " " __TIME__ " by Enis BAYRAMOGLU and jca)");
	//setResID("localize", 915);
	createBaseVar();
	//
	poseIndex = 0;
	lastOdoPose.t.valid = false;
	matchMiss = 100;

	//Initialize the trans x,y,th variables for returning to MRC
	transX = 0.0;
	transY = 0.0;
	transTh = 0.0;
  for (int i = 0; i < MML; i++)
  {
    matchList[i] = NULL;
  }
  matchList[0] = new ULineMatch();
//   matchList[0]->delH_delP = &matchDelH_delPs[0];
//   matchList[0]->lineDifCov = &matchlineDifCovs[0];
  matchListCnt = 1;
  for (int i = 0; i < MML; i++)
  {
    laserMatches[i] = NULL;
  }
  laserMatchesCnt = 0;
  //
  innoLog.setLogName("loca");
  innoLog.openLog();
  fprintf(innoLog.getF(), "time, mapX, mapY, mapH, covXX, covYY, covHH, innovX, innovY, innovH, odoDist, innovDist, usedLines\n");
}

UFuncLoca2::~UFuncLoca2() { // possibly remove allocated variables here - if needed
  // clean up
  for (int i = 0; i < MML; i++)
  { // delete map match list
    if (matchList[i] == NULL)
      break;
    else
    {
      delete matchList[i];
      matchList[i] = NULL;
    }
  }
  for (int i = 0; i < MML; i++)
  { // delete laser matches
    if (laserMatches[i] == NULL)
      break;
    else
    {
      delete laserMatches[i];
      laserMatches[i] = NULL;
    }
  }
  for (int i = 0; i < MAX_LAS_CNT; i++)
  {
    if (laserLines[i] != NULL)
    {
      delete laserLines[i];
      laserLines[i] = NULL;
    }
    else
      break;
  }
  innoLog.closeLog();
}

void UFuncLoca2::createBaseVar() {
	varOdoB = addVar("wheelBase", 0.26, "d",
			"(rw) (odoB) Wheel base for assumed differential drive robot");
	/// distance varaince for right wheel each moved meter
	varKR
	= addVar("sdmRight", sqrt(0.003), "d",
			"(rw) (kR) Variance each driven meter - as sd of err each m (right)");
	/// distance varaince for left wheel each moved meter
	varKL = addVar("sdmLeft", sqrt(0.003), "d",
			"(rw) (kL) Variance each driven meter - as sd (left)");
// 	varMaxXYVariance = addVar("maxXYVariance", 0.05, "d",
// 			"(rw) The maximum variance of each hypothesis along the x and y dimensions");
// 	varMaxThVariance = addVar("maxThVariance", 0.05, "d",
// 			"(rw) The maximum variance of each hypothesis along the theta dimension");
	varLaserAlpha = addVar("laserAlphaVariance", 0.001, "d",
			"(rw) The assumed noise variance over the laser alpha reading (of lines)");
	varLaserR = addVar("laserRVariance", 0.0004, "d",
			"(rw) The assumed noise variance over the laser R reading (of lines)");
  varMahaDistNoCorr = addVar("noCorrMaha", 1.1, "d", "(rw) Mahalanobi distance cost, when discarding a correlation (joint match)");
	varDefinedLines = addVar("lines", 0.0, "d",
			"(r) Number lines defined in localizer");
	varMinOverlap	= addVar("minOverlap", 0.1, "d",
			"(r/w) minimum overlap between laser line and map line");
	varCovar = addVar("covar", "0 0 0; 0 0 0; 0 0 0", "m2",
			"(r) Current covariance martix (x, y, th)");
	varCovarA = addVar("errElipse", "0 0", "d",
			"(r) Error ellipse - major [m]; minor [m]; degrees");
  varCovarV = addVar("errElipseV", "0 0; 0 0", "d",
      "(r) Error ellipse vectors - x,y in rows");
	varUpdates = addVar("hit", 0.0, "d",
			"(r) Number of successful updates (total)");
	varFailed = addVar("mis", 0.0, "d",
			"(r) Number of scans with no update since last update");
  varOffsetValue = addVar("odoOffset", "0 0; 0 0", "m2", "(r(w)) calculated offset (distance and heading) to remove innovation [Pd, Ph; Id, Ih] P=proportional (open loop), I integral for closed loop");
  varOffsetGain = addVarA("odoOffsetGain", "0.01, 0.01", "m2", "(rw) gain for calculation of offset [P-lowpass-gain Integral-gain] if Integral is > 0 then I value is implemented");
}


///////////////////////////////////////////////////

bool UFuncLoca2::setResource(UResBase * resource, bool remove) { // load resource as provided by the server (or other plugins)
	bool result = true;

	if (resource->isA(UResPoseHist::getOdoPoseID())) { // pointer to server the resource that this plugin can provide too
		// but as there might be more plugins that can provide the same resource
		// use the provided
		if (remove)
			// the resource is unloaded, so reference must be removed
			poseHist = NULL;
		else if (poseHist != (UResPoseHist *) resource)
			// resource is new or is moved, save the new reference
			poseHist = (UResPoseHist *) resource;
		else
			// reference is not used
			result = false;
	}

	// other resource types may be needed by base function.
	result = UFunctionBase::setResource(resource, remove);
	return result;
}

// bool UFuncLoca2::handleCommand(UServerInMsg * msg, void * extra) { // message is unhandled
//   bool result = false;
//   // Test for the handled commands, and call a function to do the job
//   // the tagname is not case sensitive - see the library documentation for
//   // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
//   if (msg->tag.isTagA("localize"))
//     result = handleLocalize(msg, extra);
//   else if (msg->tag.isTagA("addline"))
//     result = handleAddLine(msg);
//   else if (msg->tag.isTagA("setinitpose"))
//     result = handleSetInitPose(msg);
//   else if (msg->tag.isTagA("setinitcov"))
//     result = handleSetInitCov(msg);
//   else if (msg->tag.isTagA("resetlocalizer"))
//     result = handleResetLocalizer(msg);
//   else
//     sendDebug(msg, "Command not handled (by me)");
//   return result;
// }

bool UFuncLoca2::handleCommand(UServerInMsg * msg, void * extra)
{ // handle localization commands
  const int MRL = 200;
  char reply[MRL];
  // first priority is on-line help
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("LOCA - localizer2");
    sendText("LOCA is an extension to the basic kalman localizer (aulocalize by Enis).\n");
    sendText("LOCA uses the aulobst plugin to detect lines from a laserscan.\n");
    sendText("The polygon plugin (poly) may hold the map lines, and relevant parts of the polygon lines are then\n");
    sendText("  loaded to LOCA using the 'loca addline polyline=mapWall*' or similar command.\n");
    sendText("Set variance for laser and robot using using VAR LOCA.xxx.\n");
    sendText("The localize need initial pose (in map coordinates) and covariance to work.\n");
    sendText("Before localize the lobst plugin must have analyzed the laserscan (e.g. 'lobst make')\n");
    sendText("Use 'loca localize' to make a localization.\n");
    sendText("The localized position (in map oordinates) are used to maintain the maopose plugin (if loaded)\n");
    sendText("The localizer try to use the set of correlations between laser lines and map that are most\n");
    sendText("  joint compatible.\n");
    sendText("NB! the LOCA localizer is not finished yet 8/8/2013 /JCA\n");
    sendText("\n");
    sendText("--- LOCA localizer options:\n");
    sendText(" reset                        Resets most of localizer and take next data as first data\n");
    sendText(" resetMap                     Removes map\n");
    sendText(" setInitPose x=X y=Y th=Th    Set the initial robot pose in map coordinates (X,Y,Th) [m,m,rad]\n");
    sendText(" setInitCov  Cx=X Cy=Y Cth=Th Set the initial covariance NB (X,Y,Th) must be squared values [m², m²,rad²]\n");
    sendText(" addline startx=X1 starty=Y1 endx=X2 endy=Y2   Adds limited line from [X1,Y1] to X2,Y2]\n");
    sendText(" addline alpha=R r=R                           Adds unlimited line in aplha-r notation\n");
    sendText(" addline polyLine=name        Add line segments from polygon plug-in with 'name', name may use wildcard e.g. 'map?Line*' (is case sensitive)\n");
    sendText(" list                         list loaded lines (no localize performed)\n");
    sendText(" list poly=prename            Copy lines to polygons - polygon names will be 'prename.' + line name\n");
    sendText(" silent[=false]               print less to server console\n");
    sendText(" getonly                      Return MRC laservariables only (no localize performed)\n");
    sendText(" localize                     Do the localization based on default laser scanner or pushed data\n");
    sendText("--- \n");
    sendText("See also 'VAR LOCA' for other setup parameters\n");
    sendHelpDone(msg);
    return true;
  }
  else
  {
    msg->tag.getAttBool("silent", &silent, true);
    if (msg->tag.getAttBool("reset", NULL))
    { // reset and remove map
      lastOdoPose.t.valid = false;
      poseIndex = 0;
      sendInfo("reset");
    }
    if (msg->tag.getAttBool("resetMap", NULL))
    { // reset and remove map
      lineList.clear();
      varDefinedLines->setDouble(0.0);
      sendInfo("map reset");
    }
    if (msg->tag.getAttBool("setInitPose", NULL))
    { // reset and remove map
      double x,y,th;
      bool xavailable, yavailable, thavailable;
      xavailable = msg->tag.getAttDouble("x", &x);
      yavailable = msg->tag.getAttDouble("y", &y);
      thavailable = msg->tag.getAttDouble("th", &th);
      if (xavailable && yavailable && thavailable)
      { // set pose
        pose << x, y, th;
        updateStatus(x, y, th, lastOdoPose.t);
        sendInfo("pose set");
      } else {
        sendWarning("inappropriate arguments for setinitpose");
      }
    }
    if (msg->tag.getAttBool("setInitCov", NULL))
    {
      double cx,cy,cth;
      bool cxavailable, cyavailable, cthavailable;
      cxavailable = msg->tag.getAttDouble("cx", &cx);
      cyavailable = msg->tag.getAttDouble("cy", &cy);
      cthavailable = msg->tag.getAttDouble("cth", &cth);
      if (cxavailable && cyavailable && cthavailable)
      {
        poseCov << cx, 0, 0, 0, cy, 0, 0, 0, cth;
        //GaussianHypothesis<3> initDist(pose,poseCov,1);
        //Matrix<double, 3, 1> maxVariances;
        //maxVariances << varMaxXYVariance->getValued(), varMaxXYVariance->getValued(), varMaxThVariance->getValued();
        //GaussianHypothesis<3>::list splittedList;
        //poseDist.split(initDist,splittedList,maxVariances,table);
        //poseDist.GHlist.splice_after(poseDist.GHlist.begin(),splittedList);
        updateCovStatus();
        sendInfo("covariance matrix set");
      } else {
        sendWarning("inappropriate arguments for setinitcov");
      }
    }
    if (msg->tag.getAttBool("addline", NULL))
    {
      int n;
      if (handleAddLine(msg, &n))
      {
        varDefinedLines->setDouble((double) lineList.size());
        if (not silent)
        {
          snprintf(reply, MRL, "added %d line(s), now defined %u lines", n, lineList.size());
          sendInfo(reply);
        }
      }
      else
        sendInfo("inappropriate arguments for addline");
    }
    if (msg->tag.getAttBool("list", NULL))
    {
      list<LEL_ARLine>::iterator itWrld;
      int n;
      //
      const int MPL = 32;
      char polyName[MPL];
      char polyPreName[MPL];
      UResPoly * resPoly = (UResPoly *) getStaticResource("poly", false, false);
      UPolyItem * poly;
      bool toPoly = msg->tag.getAttValue("poly", polyPreName, MPL);
      UPosition p1, p2, p2old;
      if (toPoly)
      { // give polygon name a default pre-name
        if (strlen(polyPreName) < 2)
          snprintf(polyPreName, MPL, "loca");
        else if (strlen(polyPreName) > 10)
          // terminate if too long
          polyPreName[10] = '\0';
      }
      else
      { // send pretekst
        snprintf(reply, MRL, "List of loaded %d localizer line(s)",
            (int) lineList.size());
        sendHelpStart(reply);
      }
      n = 0;
      poly = NULL;
      itWrld = lineList.end();
      while (true) // (itWrld = lineList.end(); itWrld != lineList.begin(); itWrld--)
      {
        itWrld--;
        if (toPoly)
        { // make polygon line
          p1.set(itWrld->p1[0], itWrld->p1[1], 0.2);
          p2.set(itWrld->p2[0], itWrld->p2[1], 0.2);
          if (p2old.dist(p1) > 0.01)
            poly = NULL;
          if (poly == NULL)
          {
            snprintf(polyName, MPL, "%s.%s", polyPreName, itWrld->name);
            poly = resPoly->getItem(polyName);
            if (poly != NULL)
              poly->clear();
            else
              poly = resPoly->add(polyName);
            poly->cooSys = 2;
            strncpy(poly->color, "d1-d", 5);
          }
          if (poly->getPointsCnt() == 0)
            poly->add(p1);
          poly->add(p2);
          p2old = p2;
        }
        else
        {
          if (itWrld->limited)
          { // copy to string
            snprintf(
                reply,
                MRL,
                "   %d a=%8.5frad r=%7.4fm (%7.3fx,%7.3fy to %7.3fx,%7.3fy) %s\n",
                n, itWrld->alpha, itWrld->r, itWrld->p1[0],
                itWrld->p1[1], itWrld->p2[0], itWrld->p2[1],
                itWrld->name);
            //printf("%s", reply);
          } else
            snprintf(reply, MRL, "   %d a=%8.5frad r=%7.4fm (unlimited)\n",
                n, itWrld->alpha, itWrld->r);
          sendText(reply);
        }
        n++;
        if (itWrld == lineList.begin())
          break;
      }
      if (not silent)
        sendHelpDone();
    }
    if (msg->tag.getAttBool("getonly", NULL))
    { // send origin only
      snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n",
          transX, transY, transTh);
      // send this string as the reply to the client
      sendMsg(msg, reply);
    }
    else if (msg->tag.getAttBool("localize", NULL))
    {
      if (handleLocalize(msg, extra))
        sendInfo("localized");
      else
        sendWarning("localize failed");
    }
  }
  return true;
}


//////////////////////////////////////////////////////////////////////////

bool UFuncLoca2::handleAddLine(UServerInMsg * msg, int * added) {
  const int MVL = 50;
  char val[MVL];
  double alpha=NAN;
  bool alphaavailable = false;
  double r=NAN;
  bool ravailable = false;
  double startx=NAN;
  double starty=NAN;
  double endx=NAN;
  double endy=NAN;
  bool startxavailable = false;
  bool startyavailable = false;
  bool endxavailable = false;
  bool endyavailable = false;
  char name[LEL_ARLine::MNL] = "noname";
  bool result = false;
  int n = 0;
  //
  alphaavailable = msg->tag.getAttDouble("alpha", &alpha);
  ravailable = msg->tag.getAttDouble("r", &r);
  startxavailable = msg->tag.getAttDouble("startx", &startx);
  startyavailable =  msg->tag.getAttDouble("starty", &starty, MVL);
  endxavailable =  msg->tag.getAttDouble("endx", &endx, MVL);
  endyavailable =  msg->tag.getAttDouble("endy", &endy, MVL);
  msg->tag.getAttValue("name", name, LEL_ARLine::MNL);
  //
  if (alphaavailable && ravailable) {
    LEL_ARLine newLine(alpha, r);
    lineList.push_front(newLine);
    result = true;
    n++;
  } else if (startxavailable && startyavailable && endxavailable
      && endyavailable) {
    LEL_ARLine newLine(startx, starty, endx, endy, name);
    lineList.push_front(newLine);
    result = true;
    n++;
  }
  else if (msg->tag.getAttValue("polyLine", val, MVL))
  { // 'val' is now the name of the lines to added
    UResPoly * resPoly = (UResPoly *) getStaticResource("poly", false, false);
    UPolyItem * poly;
    int p1 = 0, p2;
    const int MNL = 32;
    char s[MNL];
    //
    if ((resPoly != NULL) and (strlen(val) > 0))
    poly = resPoly->getNext(p1, &p2, val);
    while (poly != NULL)
    {
      UPosition * pos1 = poly->getPoints();
      UPosition pos2;
      for (int i = 0; i < poly->getPointsCnt() - 1; i++)
      { // line name in localizer uses polygon name and add a index number
        snprintf(s, MNL, "%s_%03d", poly->name, i);
        pos2 = pos1[1];
        if (pos1->dist(pos2) > 0.2)
        { // must have some length to be usefull
          LEL_ARLine newLine(pos1->x, pos1->y, pos2.x, pos2.y, s);
          lineList.push_front(newLine);
          n++;
        }
        pos1++;
      }
      p1 = p2;
      poly = resPoly->getNext(p1, &p2, val);
    }
    result = true;
  }
  if (result)
    *added = n;
  return result;
}

///////////////////////////////////////////////////

void UFuncLoca2::projectToLaser(LEL_ARLine worldLine,
                            Matrix<double,3, 1> & pose,
                            Matrix<double,3, 3> & poseCov,
                            LEL_ARLine &projLine,
                            Matrix<double,2, 2> &lineCov,
                            Matrix<double,2, 3> & delH_delP)
{
  double posex = pose(0, 0);
  double posey = pose(1, 0);
  double poseTh = pose(2, 0);
  double alpha = worldLine.alpha - poseTh;
  projLine.alpha = alpha - lasPose.Kappa;
  projLine.r = worldLine.r -
              (posex * cos(worldLine.alpha) +  posey * sin(worldLine.alpha)) -
              (lasPose.x * cos(alpha) + lasPose.y * sin(alpha));
  if (worldLine.limited)
  {
    projLine.limited = true;
    double lsrX = posex + lasPose.x * cos(poseTh) - lasPose.y * sin(poseTh);
    double lsrY = posey + lasPose.x * sin(poseTh) + lasPose.y * cos(poseTh);
    double ll = worldLine.positionAlongLine(lsrX, lsrY);
    projLine.lb = worldLine.lb - ll;
    projLine.le = worldLine.le - ll;
    if (projLine.lb > projLine.le)
      // make sure le > lb
      fswap(&projLine.lb, &projLine.le);
  }
  delH_delP << 0, 0, -1,
              -cos(worldLine.alpha),
              -sin(worldLine.alpha),
              -lasPose.x * sin(alpha) + lasPose.y * cos(alpha);
  lineCov = delH_delP * poseCov * delH_delP.transpose();
}

/////////////////////////////////////////////////

void UFuncLoca2::updateStatus(double mapX, double mapY, double mapTh,
    UTime time) {
  UPoseTVQ poseTVQ(mapX, mapY, mapTh, time, 0.0, 0.0);
  UResPoseHist * mappose;
  const int LOCALIZE_ID = 15;

  poseTVQ.q = 1.0 - fmin(matchMiss / 100.0, 1.0);
  varFailed->setDouble(matchMiss);
  //
  mappose = (UResPoseHist *) getStaticResource("mapPose", true);
  if (mappose != NULL)
    mappose->addIfNeeded(poseTVQ, LOCALIZE_ID);
  //
  updateCovStatus();
}

//////////////////////////////////////////////////////

void UFuncLoca2::updateCovStatus()
{
  UMatrix4 mc, mv(2, 2), ma;
  bool isCompl;
  //
  varCovar->setValueM(poseCov.rows(), poseCov.cols(), &poseCov(0, 0));
  mc.setMat(poseCov.rows(), poseCov.cols(), &poseCov(0, 0));
  // mc.print("Covar as matrix");
  // get eigenvector/eigenvalues for x,y part of matrix
  ma = mc.eig2x2(&isCompl, &mv);
  // set covariance vector
  varCovarV->setValueM(mv.rows(), mv.cols(), mv.getData());
  // expand values to include heading deviation
  ma.setSize(3, 1);
  // convert to deviation values and degrees
  ma.set(sqrt(ma.get(0, 0)), sqrt(ma.get(1, 0)), sqrt(mc.get(2, 2)) * 180.0
      / M_PI);
  // set to global variable
  varCovarA->setValueM(ma.rows(), ma.cols(), ma.getData());
  //
}

//////////////////////////////////////////////////////////////////////

bool UFuncLoca2::handleLocalize(UServerInMsg * msg, void * extra)
{ // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  int matchCnt = 0;
  bool isOK = false;
  UPoseTime uNewPose;
  UPose updVec;
  double dist;
  // use 2D line segments from  aulobst plugin
  laserLinesCnt = MAX_LAS_CNT;
  isOK = callGlobalV("lobst.getRansacLines", "", NULL, laserLines, &laserLinesCnt);
  if (not isOK)
    printf("failed to contact the aulobst plugin for laser lines\n");
  else if (laserLinesCnt < 2)
  {
    printf("no laser lines found by aulobst - forgot to ask aulobst to generate lines ('lobst make')?\n");
    isOK = false;
  }
  else
  { // there is lines, so attempt update
    /// get robot pose at laser time
    uNewPose = *(UPoseTime*) laserLines[0];
    updVec = uNewPose - lastOdoPose;
    dist = hypot(updVec.x, updVec.y); // uNewPose.getDistance(&lastOdoPose);
    if (not silent)
    { // print pre-update pose
      printf("Pose preupd %7.3fx, %7.3fy, %7.4fth,  cov diag %9.5fxx, %9.5fyy %9.5fhh - moved %.3f m\n",
          pose(0,0), pose(1,0), pose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2), dist);

    }
    /// prepare match array with laser lines
//     for (int i = 0; i < laserLinesCnt - 1; i++)
//     {
//       if (laserMatches[i] == NULL)
//         laserMatches[i] = new ULaserMatches();
//       laserMatches[i]->laserLine = (U2Dseg *) laserLines[i + 1];
//       laserMatches[i]->mapLinesCnt = 0;
//     }
    /// update since last localize
    updateDisplacement(lastOdoPose.t, pose, poseCov, uNewPose.t,
        poseHist, silent, varOdoB->getValued(),
        varKR->getValued(), varKL->getValued());
    lastOdoPose = uNewPose;
    /// correlate map lines and laser lines - in laser coordinates
    correlateLines(uNewPose);
    //
    isOK = matchListCnt > 0;
    if (not silent)
      printf("loca2 laser and map correlated, found %d possible matches to %d laser lines in %d map lines\n", matchListCnt, laserLinesCnt - 1, lineList.size());
  }
  if (isOK)
  { // now, try all combinations and find best joint match
    // uses matchList to find and mark compatible matches
    // sets the match->best to the map line that is the best joint match
    //    - else if not compatible then best is set to mapLineCnt
    markBestJointMatches();
    isOK = laserMatchesCnt > 0;
    if (not silent)
      printf("loca finished joint match selection - and marked best match for each laser line\n");
  }
  // update kalman filter with new correlated lines
  if (true and isOK)
  { // single updates - update kalman filter after each match - bad for wrong correlations
    if (not silent)
      printf("======================== single match update ===============================\n");
    Matrix<double,3, 1> newPose = pose;
    Matrix<double,3, 1> preUpdPose = pose;
    //ULaserMatches * lm;
    double devThreshold = sqr(3.0);
    char * p1 = reply;
    int n = 0;
    reply[0] = '\0';
    if (not silent)
      printf("loca found best matches (in %d lines):\n", laserMatchesCnt);
    for (int i = 0; i < laserMatchesCnt; i++)
    { // update for all laser lines that has a best match
      ULaserMatches * lm = laserMatches[i];
      if (lm->best < lm->mapLinesCnt)
      { // it has a best match (not a non-correlated laser line)
        // get the best match
        ULineMatch * me = lm->mapLines[lm->best];
        snprintf(p1, MRL - (p1 - reply), ", %s", me->mapLine.name);
        n += strlen(p1);
        p1 = &reply[n];
        if (me->valid and me->minMahDist < devThreshold)
        { // update with measurement
          Matrix<double,3, 1> poseDif;
          Matrix<double,2, 1> lineDif;
          Matrix<double,2, 1> v;
          Matrix<double,3, 2> K;
          //
          poseDif = newPose - pose;
          lineDif << me->difA, me->difR;
          v = lineDif - (me->delH_delP * poseDif);
          K = poseCov * me->delH_delP.transpose() * me->lineDifCovAR.inverse();
          newPose += K * v;
          poseCov -= K * me->lineDifCovAR * K.transpose();
          matchCnt++;
          if (not silent)
          {
            printf("PoseDif %.3fx, %.3fy, %.3frad\n", poseDif(0,0), poseDif(1,0), poseDif(2,0));
            printf("LineDif %.3fa, %.3fr\n", lineDif(0,0), lineDif(1,0));
            printf("arldCov %.6f, %.6f \n", me->lineDifCovAR(0,0), me->lineDifCovAR(0,1));
            printf("arldCov %.6f, %.6f \n", me->lineDifCovAR(1,0), me->lineDifCovAR(1,1));
            printf("   dHdP %.6f, %.6f %.6f\n", me->delH_delP(0,0), me->delH_delP(0,1), me->delH_delP(0,2));
            printf("   dHdP %.6f, %.6f %.6f\n", me->delH_delP(1,0), me->delH_delP(1,1), me->delH_delP(1,2));
            printf("      v %.3fx, %.3fy\n", v(0,0), v(1,0));
            printf("      k %.6f, %.6f \n", K(0,0), K(0,1));
            printf("      k %.6f, %.6f \n", K(1,0), K(1,1));
            printf("      k %.6f, %.6f \n", K(2,0), K(2,1));
            printf("poseCov %.6f, %.6f %.6f\n", poseCov(0,0), poseCov(0,1), poseCov(0,2));
            printf("poseCov %.6f, %.6f %.6f\n", poseCov(1,0), poseCov(1,1), poseCov(1,2));
            printf("poseCov %.6f, %.6f %.6f\n", poseCov(2,0), poseCov(2,1), poseCov(2,2));
            printf("Pose after line %d  %7.3fx, %7.3fy, %7.4fth,  cov diag %9.5fxx, %9.5fyy %9.5fhh\n",
                  i, newPose(0,0), newPose(1,0), newPose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2));
          }
        }
        pose = newPose;
      }
    }
    int upds = varUpdates->getInt();
    UPose innovation(newPose(0,0) - preUpdPose(0,0), newPose(1,0) - preUpdPose(1,0), limitToPi(newPose(2,0)- preUpdPose(2.0)));
    double innoDist = updVec.x * innovation.x + updVec.y * innovation.y;
    double offsetGain = varOffsetGain->getDouble(0);
    double offsetDistP = 0,  offsetDistI;
    double offsetHP = 0, offsetHI;
    if (upds > 2)
    { // skip the first updates, as start position may be inaccurate
      if (offsetGain > 1e-6 and dist > 1e-3)
      { // proportional gain is > 0.0 and we have moved more than a mm
        if (upds < 1/offsetGain)
          offsetGain = 1.0 / upds;
        offsetDistP = varOffsetValue->getDouble(0) * (1 - offsetGain) + innoDist/dist * offsetGain;
        offsetHP = varOffsetValue->getDouble(1) * (1 - offsetGain) + innovation.h/dist * offsetGain;
        // limit size of offset
        offsetHP = limitd(offsetHP, -0.2, 0.2);
        offsetDistP = limitd(offsetDistP, -0.2, 0.2);
        // save value
        varOffsetValue->setValued(offsetDistP, 0);
        varOffsetValue->setValued(offsetHP, 1);
      }
      offsetGain = varOffsetGain->getDouble(1);
      if (offsetGain > 1e-6 and dist > 1e-3)
      { // integral gain is > 0 and we have moved
        offsetDistI = varOffsetValue->getDouble(2) + innoDist/dist * offsetGain;
        offsetHI = varOffsetValue->getDouble(3) + innovation.h/dist * offsetGain;
        // limit size of offset
        offsetHI = limitd(offsetHI, -0.2, 0.2);
        offsetDistI = limitd(offsetDistI, -0.2, 0.2);
        // save value
        varOffsetValue->setValued(offsetDistI, 2);
        varOffsetValue->setValued(offsetHI, 3);
      }
    }
    if (innoLog.isOpen())
    {
      fprintf(innoLog.getF(), "%ld.%06ld %.2f %.2f %.2f %.5f %.5f %.5f %.3f %.3f %.5f %.3f %.3f %.4f %.4f %.4f %.4f '%s'\n", uNewPose.t.getSec(), uNewPose.t.GetMicrosec(),
              newPose(0,0), newPose(1,0), newPose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2),
              innovation.x, innovation.y, innovation.h,
              dist, hypot(newPose(0,0) - preUpdPose(0,0), newPose(1,0) - preUpdPose(1,0)),
              offsetDistP, offsetHP, offsetDistI, offsetHI,
              &reply[2]
             );
    }
  }
  if (not silent)
  {
    printf("Pose after  %7.3fx, %7.3fy, %7.4fth,  cov diag %9.5fxx, %9.5fyy %9.5fhh*\n",
          pose(0,0), pose(1,0), pose(2,0), poseCov(0,0), poseCov(1,1), poseCov(2,2));
  }
  // implement new pose and covariance
  if (isOK)
  {
//    cout.setf(std::ios_base::fixed);
    //UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());
    UPose trans;
    double poseX = pose(0, 0);
    double poseY = pose(1, 0);
    double poseTh = pose(2, 0);
    //
    trans.h = uNewPose.h - poseTh;
    trans.x = uNewPose.x - (cos(trans.h) * poseX - sin(trans.h)
        * poseY);
    trans.y = uNewPose.y - (sin(trans.h) * poseX + cos(trans.h)
        * poseY);
    //
    if (matchCnt > 0)
      matchMiss = 0;
    else
      matchMiss += 1;
    if (not silent)
      printf("Localizer: got %d line matches\n", matchCnt);
    if (matchCnt)
      varUpdates->add(1.0);
    //
    // update map-pose and global data
    updateStatus(poseX, poseY, poseTh, uNewPose.t);
    //
    //Save the trans variables to class-variables
    transX = trans.x;
    transY = trans.y;
    transTh = trans.h;
    /**
    SMRDEMO reply format */
    if (msg->client >= 0) { // send to real client only, not a push or other non-real
      snprintf(reply, MRL,
          "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n", trans.x,
          trans.y, trans.h);

      // send this string as the reply to the client
      sendMsg(msg, reply);
    }
  }
  if (not silent)
  {
    if (isOK)
      sendInfo("localizer2 done updating");
    else
      sendWarning("localizer2 failed to update with laser lines");
  }
  // return true if the function is handled with a positive result
  silent = false;
  return isOK;
}

/////////////////////////////////////////////////////////////////////////

int UFuncLoca2::correlateLines(UPoseTime uNewPose)
{
  ULineMatch * match;
  const double devThreshold = sqr(3.0);
  list<LEL_GFLine>::iterator itLas;
  UResPoly * resPoly = (UResPoly *) getStaticResource("poly", false, false);
  UPolyItem * poly;
  double minOverlap = varMinOverlap->getDouble();
  //
  matchListCnt = 1;
  match = matchList[0];
  int mapLineCnt =0;
  // correlate laser lines (in GFLL) with map lines (in lineList)
  list<LEL_ARLine>::iterator itWrld;
  //Matrix<double,3, 1> newPose = pose;
  double varAlpha = varLaserAlpha->getValued();
  double varR = varLaserR->getValued();
  if (not silent)
    printf("Correlation of all map-lines (%d) with laser lines (%d)\n", lineList.size(), laserLinesCnt - 1);
  // find best laser line correlation for each map line
  for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++)
  { // for all map lines
    if (true)
    {
      poly = resPoly->getItem(itWrld->name);
      if (poly != NULL)
      {
        poly->lock();
        // remove old polygon - set to one point
        poly->setPointsCnt(1);
        poly->setColor("b1-d");
        poly->setUpdated();
        poly->unlock();
        resPoly->gotNewData();
      }
    }
    LEL_ARLine projLine;
    Matrix<double,2, 3> delH_delP;
    Matrix<double,2, 2> lineDifCov;
    UPosition pos1, pos2; //, pe1, pe2;
    double angleDif;
    double rDif;
    projectToLaser(*itWrld, pose, poseCov, projLine, lineDifCov,
        delH_delP);
    lineDifCov(0, 0) += varAlpha;
    lineDifCov(1, 1) += varR;
    //match->minMahDist = devThreshold;
    match->valid = false;
    //debug
    // match->minMahDist = sqr(3.0);
    // debug end
    LEL_GFLine closestLine;

    for (int itLas = 0; itLas < laserLinesCnt - 1; itLas++)
    { // test all laser lines (for this map-line)
      U2Dseg * lasSeg = (U2Dseg*) laserLines[itLas + 1];
      double a,r;
      double mahDist;
      lasSeg->getARLine(&a, &r);
      angleDif = limitToPi(a - projLine.alpha);
      if (fabs(angleDif) > M_PI / 2)
      {
        rDif = -r - projLine.r;
        // line may point in opposite direction - and that is OK
        if (angleDif > 0)
          angleDif -= M_PI;
        else
          angleDif += M_PI;
      }
      else
        rDif = r - projLine.r;
      Matrix<double,2, 1> lineDif; // in a,r terms
      lineDif << angleDif, rDif;
      mahDist = lineDif.transpose() * lineDifCov.inverse() * lineDif;
      double overlap = 0.0;
      if (mahDist < devThreshold and fabs(angleDif) < 0.5)
      { // match is OK, and angle is within manageable range
        if (projLine.limited)
        { // count number of measurements that is within map line length.
          double t1, t2;
          double c1 = projLine.lb;
          double c2 = projLine.le;
          U2Dpos oe = lasSeg->getOtherEnd();
          t1 = projLine.positionAlongLine(lasSeg->x, lasSeg->y);
          t2 = projLine.positionAlongLine(oe.x, oe.y);
          if (t1 > t2)
            // make sure t2 > t1 (as projLine.le > projLine.lb)
            fswap(&t1, &t2);
          if (t1 > c1)
            c1 = t1;
          if (t2 < c2)
            c2 = t2;
          if (c2 > c1)
            // overlap only if c2 > c1 still
            overlap = c2 - c1;
          if (overlap < minOverlap)
          { // not enough coverage - most is outside map line
            // cout<<"Match ruled out due to line limits mismatch\n";
            if (not silent)
              printf("-- match     laserLine %3d to mapLine %3d (%s) but discarded (at %.2fx,%.2fy) lack of overlap (is %.2fm < %.2fm)\n", itLas, mapLineCnt, itWrld->name,
                     lasSeg->x, lasSeg->y, overlap, minOverlap);
            continue;
          }
        }
        // candidate correlation
        match->minMahDist = mahDist;
        match->delH_delP = delH_delP;
        //match->lineDifMin = lineDif;
        match->lineDifCovAR = lineDifCov;
        match->laserLine = lasSeg;
        match->laserLineMatch = itLas;
        match->mapLine = *itWrld;
        match->pe1 = pos1;
        match->pe2 = pos2;
        match->difR = rDif;
        match->difA = angleDif;
        match->valid = true;
        if (not silent) // and match->minMahDist < 1e3)
        { // print line correlation result
          char c = ' ';
          if (match->minMahDist < devThreshold)
            c = '*';
          if (match->minMahDist < devThreshold * 2)
            // print also near matches, that just missed correlation
            match->print(silent, c, mapLineCnt, matchListCnt, overlap);
          //
        }
        if (resPoly != NULL and match->minMahDist < devThreshold)
        {
          //UPose lp = uNewPose.getPoseToMapPose(UPose(lasPose.x, lasPose.y, lasPose.Kappa));
          UResPoseHist * mappose;
          mappose = (UResPoseHist *) getStaticResource("mapPose", true);
          UPoseTime odoo = mappose->getOdoPoseOrigin();
          match->addToPoly(resPoly, uNewPose, odoo);
        }
        if (match->minMahDist < devThreshold)
        { // calculate inverse to save time in joint match algorithm
          match->lineDifCovARinv = match->lineDifCovAR.inverse();
          //
          // save matches, to postpone update to all is matched
          // add lo matches for this laser line
//           ULaserMatches * lm = laserMatches[match->laserLineMatch];
//           if (lm->mapLinesCnt < lm->MLM)
//           { // space for more matches
//             //lm->laserLine = match->laserLine;
//             lm->mapLines[lm->mapLinesCnt] = match;
//             lm->mapLinesCnt++;
//           }
//           else
//             printf("Loca **** error too many maplines (>%d) near this laser line %d\n", lm->MLM, match->laserLineMatch);
          //
          if (matchListCnt < MML)
          { // prepare next match structure
            if (matchList[matchListCnt] == NULL)
              matchList[matchListCnt] = new ULineMatch();
            match = matchList[matchListCnt];
            match->minMahDist=1e3;
            match->valid = false;
            // - used by later joint match test
            match->setExpectedPose(&pose, &poseCov);
            matchListCnt++;
          }
          else
            printf("Loca **** error too many matches between laser lines and map lines (>%d\n", MML);
        }
      }
    }
    mapLineCnt++;
  }
  if (matchListCnt > 0 and not matchList[matchListCnt - 1]->valid)
  { // remove last empty match structure
    matchListCnt--;
  }
  return matchListCnt;
}


///////////////////////////////////////////////////////////////////

// int UFuncLoca2::correlateLines_old(list<LEL_GFLine> GFLL, UPoseTime uNewPose)
// {
//   ULineMatch * match;
//   double devThreshold = 3 * 3;
//   list<LEL_GFLine>::iterator itLas;
//   UResPoly * resPoly = (UResPoly *) getStaticResource("poly", false, false);
//   UPolyItem * poly;
//   double pointsInThreshold = varPointsInThreshold->getDouble();
//   //
//   matchListCnt = 1;
//   match = matchList[0];
//   int mapLineCnt =0;
//   // correlate laser lines (in GFLL) with map lines (in lineList)
//   list<LEL_ARLine>::iterator itWrld;
//   //Matrix<double,3, 1> newPose = pose;
//   double varAlpha = varLaserAlpha->getValued();
//   double varR = varLaserR->getValued();
//   if (not silent)
//     printf("Correlation of all map-lines (%d( with laser lines (%d)\n", lineList.size(), GFLL.size());
//   // find best laser line correlation for each map line
//   for (itWrld = lineList.begin(); itWrld != lineList.end(); itWrld++)
//   { // for all map lines
//     if (not silent)
//     {
//       poly = resPoly->getItem(itWrld->name);
//       if (poly != NULL)
//       {
//         poly->lock();
//         // remove old polygon - set to one point
//         poly->setPointsCnt(1);
//         poly->setColor("b1dd");
//         poly->setUpdated();
//         poly->unlock();
//         resPoly->gotNewData();
//       }
//     }
//     LEL_ARLine projLine;
//     Matrix<double,2, 3> delH_delP;
//     Matrix<double,2, 2> lineDifCov;
//     UPosition pos1, pos2; //, pe1, pe2;
//     double lmin, lmax; //, difA, difR;
//     double angleDif;
//     projectToLaser(*itWrld, pose, poseCov, projLine, lineDifCov,
//         delH_delP);
//     lineDifCov(0, 0) += varAlpha;
//     lineDifCov(1, 1) += varR;
//     //match->minMahDist = devThreshold;
//     match->valid = false;
//     //debug
//     match->minMahDist = 10;
//     // debug end
//     LEL_GFLine closestLine;
//     int laserLine = 0;
//     for (itLas = GFLL.begin(); itLas != GFLL.end(); itLas++)
//     { // test all laser lines (for this map-line)
//       LEL_ARLine line = (*itLas).toARLine();
//       angleDif = fmod(line.alpha - projLine.alpha + 3
//           * M_PI, 2 * M_PI) - M_PI;
//       double rDif;
//       if (fabs(angleDif) > M_PI / 2)
//       { // absolute angle difference
//         angleDif = fmod(angleDif + 2 * M_PI, 2 * M_PI) - M_PI;
//         rDif = -line.r - projLine.r;
//       } else
//         rDif = line.r - projLine.r;
//       Matrix<double,2, 1> lineDif; // in a,r terms
//       lineDif << angleDif, rDif;
//       double mahDist = lineDif.transpose()
//               * lineDifCov.inverse() * lineDif;
//       int pointsIn = 0;
//       if (mahDist < match->minMahDist and angleDif < 0.7)
//       { // match is OK, and angle is within manageable range
//         if (projLine.limited)
//         { // count number of measurements that is within map line length.
//           for (int p = 0; p < itLas->edgeCount; p++) {
//             double x = itLas->edgesX[p];
//             double y = itLas->edgesY[p];
//             double lp = projLine.positionAlongLine(x, y);
//             if (p == 0)
//             {
//               lmin = lp;
//               lmax = lp;
//               pos1.set(x,y);
//               pos2.set(x,y);
//             }
//             else if (lp < lmin)
//             {
//               lmin = lp;
//               pos1.set(x,y);
//             }
//             else if (lp > lmax)
//             {
//               lmax = lp;
//               pos2.set(x,y);
//             }
//             if (projLine.lb < lp and lp < projLine.le)
//               pointsIn++;
//           }
//           if (pointsIn < pointsInThreshold * itLas->edgeCount)
//           { // not enough coverage - most is outside map line
//             // cout<<"Match ruled out due to line limits mismatch\n";
//             if (not silent)
//               printf("-- match discarded with laser line %d and %s (at %.2fx,%.2fy) lack of measurements (%d) inside map line\n", laserLine, itWrld->name, itWrld->p1[0], itWrld->p1[0], pointsIn);
//             continue;
//           }
//         }
//         // best candidate so far
//         match->minMahDist = mahDist;
//         match->delH_delP = delH_delP;
//         //match->lineDifMin = lineDif;
//         match->lineDifCovAR = lineDifCov;
//         match->laserLine = *itLas;
//         match->laserLineMatch = laserLine;
//         match->mapLine = *itWrld;
//         match->pe1 = pos1;
//         match->pe2 = pos2;
//         match->difR = rDif;
//         match->difA = angleDif;
//         match->valid = true;
//         if (not silent and match->minMahDist < 1e3)
//         { // print line correlation result
//           char c = ' ';
//           if (match->minMahDist < devThreshold)
//             c = '*';
//           if (match->minMahDist < devThreshold * 2)
//             // print also near matches, that just missed correlation
//             match->print(silent, c, mapLineCnt, matchListCnt, pointsIn);
//           //
//           if (resPoly != NULL and match->minMahDist < devThreshold)
//           {
//             UPose lp = uNewPose.getPoseToMapPose(UPose(lasPose.x, lasPose.y, lasPose.Kappa));
//             UResPoseHist * mappose;
//             mappose = (UResPoseHist *) getStaticResource("mapPose", true);
//             UPoseTime odoo = mappose->getOdoPoseOrigin();
//             match->addToPoly(resPoly, lp, odoo);
//           }
//         }
//         if (match->minMahDist < devThreshold)
//         { // calculate inverse to save time in joint match algorithm
//           match->lineDifCovARinv = match->lineDifCovAR.inverse();
//           //
//           // save matches, to postpone update to all is matched
//           // add lo matches for this laser line
//           ULaserMatches * lm = laserMatches[match->laserLineMatch];
//           if (lm->mapLinesCnt < lm->MLM)
//           { // space for more matches
//             //lm->laserLine = match->laserLine;
//             lm->mapLines[lm->mapLinesCnt] = match;
//             lm->mapLinesCnt++;
//           }
//           else
//             printf("Loca **** error too many maplines (>%d) near this laser line %d\n", lm->MLM, match->laserLineMatch);
//           //
//           if (matchListCnt < MML)
//           { // prepare next match structure
//             if (matchList[matchListCnt] == NULL)
//               matchList[matchListCnt] = new ULineMatch();
//             match = matchList[matchListCnt];
//             match->minMahDist=1e3;
//             match->valid = false;
//             // - used by later joint match test
//             match->setExpectedPose(&pose, &poseCov);
//             matchListCnt++;
//           }
//           else
//             printf("Loca **** error too many matches between laser lines and map lines (>%d\n", MML);
//         }
//       }
//       laserLine++;
// //     if (not silent and match->minMahDist < 1e3)
// //     { // print line correlation result
// //       char c = ' ';
// //       if (match->minMahDist < devThreshold)
// //         c = '*';
// //       if (match->minMahDist < devThreshold * 2)
// //         // print also near matches, that just missed correlation
// //         match->print(silent, c, matchID);
// //       //
// //       if (resPoly != NULL and match->minMahDist < devThreshold)
// //       {
// //         UPose lp = uNewPose.getPoseToMapPose(UPose(lasPose.x, lasPose.y, lasPose.Kappa));
// //         UResPoseHist * mappose;
// //         mappose = (UResPoseHist *) getStaticResource("mapPose", true);
// //         UPoseTime odoo = mappose->getOdoPoseOrigin();
// //         match->addToPoly(resPoly, lp, odoo);
// //       }
// //     }
// //     if (match->minMahDist < devThreshold)
// //     { // calculate inverse to save time in joint match algorithm
// //       match->lineDifCovARinv = match->lineDifCovAR.inverse();
// //       //
// //       // save matches, to postpone update to all is matched
// //       // add lo matches for this laser line
// //       ULaserMatches * lm = laserMatches[match->laserLineMatch];
// //       if (lm->mapLinesCnt < lm->MLM)
// //       { // space for more matches
// //         //lm->laserLine = match->laserLine;
// //         lm->mapLines[lm->mapLinesCnt] = match;
// //         lm->mapLinesCnt++;
// //       }
// //       else
// //         printf("Loca **** error too many maplines (>%d) near this laser line %d\n", lm->MLM, match->laserLineMatch);
// //       //
// //       if (matchListCnt < MML)
// //       { // prepare next match structure
// //         if (matchList[matchListCnt] == NULL)
// //           matchList[matchListCnt] = new ULineMatch();
// //         match = matchList[matchListCnt];
// //         match->minMahDist=1e3;
// //         match->valid = false;
// //         // - used by later joint match test
// //         match->setExpectedPose(&pose, &poseCov);
// //         matchListCnt++;
// //       }
// //       else
// //         printf("Loca **** error too many matches between laser lines and map lines (>%d\n", MML);
//     }
//     mapLineCnt++;
//   }
//   if (matchListCnt > 0 and not matchList[matchListCnt - 1]->valid)
//   { // remove last empty match structure
//     matchListCnt--;
//   }
//   return matchListCnt;
// }

//////////////////////////////////////////////////////////

class ULocaMatchStat
{
public:
  /** list of laser lines and map-matches for each of these */
  ULaserMatches ** lasMchs;
  /** number of laser lines */
  int lasMchsCnt;
  /** best match squared difference */
  double minDist2;
  /** average mahalanobi distance for any correlation used in best joint set */
  double mahaScore;
  /** pose from last update */
  Matrix<double,3, 1> poseOrg;
  /** covariance from last update */
  Matrix<double,3, 3> poseCovOrg;
  /** no-match Mahalanobi distance - suared */
  double noMatchVariance;
  /** allow print to console or not */
  bool silent;
  /** seed correlation laser line */
  int seedLaserLine;
  /** seed map-line correlation for this laser line */
  int seedMapMatch;
  /** number of used correlations in this joint test */
  int usedCorrelationsCnt;
  /** number of skipped correlations (no correlations is better) in this joint test */
  int skippedCorrelationsCnt;
  /** best matched map line array */
  static const int MAX_LAS_LINES = 200;
  /** test set of correlations */
  int testSet[MAX_LAS_LINES];
  /** best set of correlations */
  int best[MAX_LAS_LINES];
  /// uncertainty for line detect
  double varAlpha; // = varLaserAlpha->getValued();
  /// uncertainty for line detect
  double varR; // = varLaserR->getValued();
  /// replacement mahalanobi distance cost for not using correlation
  double mahaDistForNotUsedCorrelation;

public:
  /** init */
  void init(ULaserMatches ** laserLines,
            int laserLinesCnt,
            double noMatchMaha,
            double varianceAlpha,
            double varianceRadius,
            double mahaDistWhenNotUsedCorrelation,
            bool beSilent)
  {
    mahaDistForNotUsedCorrelation = mahaDistWhenNotUsedCorrelation;
    lasMchs = laserLines;
    lasMchsCnt = laserLinesCnt;
    noMatchVariance = noMatchMaha;
    silent = beSilent;
    seedLaserLine = -1;
    seedMapMatch = -1;
    clear();
    varAlpha = varianceAlpha;
    varR = varianceRadius;
  }
  void clear()
  {
    minDist2 = 1e5;
    mahaScore = 3.0;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * recursive call */
//   bool testMatchesNot(int llCnt, ULineMatch * prevMatch)
//   {
//     bool result = false; // default is not a best match
//     ULaserMatches * lm = lasMchs[llCnt];
//     for (int i = 0; i <= lm->mapLinesCnt; i++)
//     {
//       ULineMatch * ma = lm->mapLines[i];
//       double inovMaha;
//       if (i < lm->mapLinesCnt)
//       { // this is not the empty hypothesis
//         Matrix<double,3, 1> poseDif;
//         Matrix<double,2, 1> lineDif;
//         Matrix<double,2, 1> v;
//         Matrix<double,3, 2> K;
//         //
//         if (prevMatch == NULL)
//         { // no previous, so use line dif direct
//           lineDif << ma->difA, ma->difR;
//           v = lineDif;
//           K = poseCovOrg * ma->delH_delP.transpose() * ma->lineDifCovARinv;
//           ma->inov = K * v;
//           inovMaha = ma->inov.transpose() * poseCovOrg.inverse() * ma->inov;
//           if (isnan(inovMaha) or inovMaha < 0)
//           { // should this be possible? -- it happens, but why?
//             if (inovMaha < 0)
//             {
//               inovMaha = -inovMaha;
//               printf("-- gave negative mahalanobi distance??\n");
//             }
//             else
//             {
//               inovMaha = 1;
//               printf("-- inverse() returned false\n");
//             }
//           }
//           ma->inovMaha = inovMaha;
//           ma->newPose1 = poseOrg + ma->inov;
//           ma->poseCov1 = poseCovOrg - K * ma->lineDifCovAR * K.transpose();
//         }
//         else
//         { // there is a previous update, so combine covariance with this
//           poseDif = prevMatch->newPose1 - poseOrg;
//           lineDif << ma->difA, ma->difR;
//           // adjust line-dif relative to updated pose
//           v = lineDif - (ma->delH_delP * poseDif);
//           // calculate update gain
//           K = prevMatch->poseCov1 * ma->delH_delP.transpose() * ma->lineDifCovARinv;
//           ma->inov = K * v;
//           inovMaha = ma->inov.transpose() * prevMatch->poseCov1.inverse() * ma->inov;
//           if (isnan(inovMaha) or inovMaha < 0)
//           { // should this be possible? -- it happens, but why?
//             if (inovMaha < 0)
//             {
//               inovMaha = -inovMaha;
//               printf("-- gave negative mahalanobi distance??\n");
//             }
//             else
//             {
//               inovMaha = 1;
//               printf("-- inverse() returned false\n");
//             }
//           }
//           // forward the worst distance
//           if (inovMaha > prevMatch->inovMaha)
//             ma->inovMaha = inovMaha;
//           else
//             ma->inovMaha = prevMatch->inovMaha;
//           // update pose and covar so far
//           ma->newPose1 = prevMatch->newPose1 + K * v;
//           ma->poseCov1 = prevMatch->poseCov1 - K * ma->lineDifCovAR * K.transpose();
//         }
//         if (not silent)
//           printf("laser line %d - match %d/%d (%s) gives maha-dist %.4f sigma (%f var)\n",
//                  llCnt, i, lm->mapLinesCnt, ma->mapLine.name, sqrt(inovMaha), inovMaha);
//       }
//       else
//       { // a no-match case
//         if (not silent)
//           printf("laser line %d - match %d/%d (no match for this line)\n", llCnt, i, lm->mapLinesCnt);
//       }
//       if (ma == NULL)
//         ma = prevMatch;
//       if (i == lm->mapLinesCnt and ma != NULL and i > 0)
//       { // this is an empty hopothesis, so apply no-match inovation distance
//         if (noMatchVariance > prevMatch->inovMaha)
//           ma->inovMaha = noMatchVariance;
//       }
//       if (llCnt + 1 < lasMchsCnt)
//       { // there is more matches - add those too
//         if (testMatchesNot(llCnt + 1, ma))
//         { // this laser-map match is a (p.t.) best match
//           result = true;
//           lm->best = i;
//         }
//       }
//       else if (ma != NULL)
//       { // not a total empty hypothesis
//         // no more matches, so evaluate
//         if (ma->inovMaha < maxMaha)
//         { // this is best so far
//           maxMaha = ma->inovMaha;
//           lm->best = i;
//           result = true;
//         }
//         if (not silent)
//           printf("--- combination has worst mahaDist of %.3f\n", sqrt(maxMaha));
//       }
//     }
//     return result;
//   }

  /**
   * recursive call -
   * - version that assumes first call is seed correlation */
  bool testMatches(int llCnt, ULineMatch * prevMatch)
  {
    bool result = false; // default is not a best match
    ULaserMatches * lm = lasMchs[llCnt];
    int mc;
    if (llCnt == seedLaserLine)
      mc = 0; // seed laser line has no alternatives
    else
      mc = lm->mapLinesCnt; // try al alternative correlations for laser line, including no-correlation
    for (int i = 0; i <= mc; i++)
    {
      ULineMatch * ma = NULL;
      double inovMaha;
      if (i < lm->mapLinesCnt)
      { // this is not the empty hypothesis
        Matrix<double,3, 1> poseDif;
        Matrix<double,2, 1> lineDif;
        Matrix<double,2, 1> v;
        Matrix<double,3, 2> K;
        Matrix<double,3, 3> tmpCov;
        Matrix<double,2, 2> tmpCovK;
        Matrix<double,3, 3> tmpCovIno;
        //
        if (prevMatch == NULL)
        { // initial seed match
          i = seedMapMatch;
          ma = lm->mapLines[seedMapMatch];
          lineDif << ma->difA, ma->difR;
          v = lineDif;
          //
          tmpCovK = ma->delH_delP * poseCovOrg * ma->delH_delP.transpose();
          tmpCovK(0, 0) += varAlpha;
          tmpCovK(1, 1) += varR;
          K = poseCovOrg * ma->delH_delP.transpose() * tmpCovK.inverse();
          //K = poseCovOrg * ma->delH_delP.transpose() * ma->lineDifCovARinv;
          ma->inov = K * v;
          inovMaha = ma->inov.transpose() * poseCovOrg.inverse() * ma->inov;
          if (isnan(inovMaha) or inovMaha < 0)
          { // should this be possible? -- it happens, but why?
            if (inovMaha < 0)
            {
              printf("-- gave negative mahalanobi distance %f ??\n", inovMaha);
              inovMaha = -inovMaha;
            }
            else
            {
              inovMaha = 1;
              printf("-- inverse() must have returned false\n");
            }
          }
          ma->inovMaha = inovMaha;
          ma->newPose1 = poseOrg + ma->inov;
          tmpCovIno = K * ma->delH_delP * poseCovOrg;
          //tmpCovIno = K * ma->lineDifCovAR * K.transpose();
          tmpCov = poseCovOrg - tmpCovIno;
        }
        else
        { // there is a previous update, so combine covariance with this
          ma = lm->mapLines[i];
          poseDif = prevMatch->newPose1 - poseOrg;
          lineDif << ma->difA, ma->difR;
          // adjust line-dif relative to updated pose
          v = lineDif - (ma->delH_delP * poseDif);
          // calculate update gain
          tmpCovK = ma->delH_delP * prevMatch->poseCov1 * ma->delH_delP.transpose();
          tmpCovK(0, 0) += varAlpha;
          tmpCovK(1, 1) += varR;
          K = prevMatch->poseCov1 * ma->delH_delP.transpose() * tmpCovK.inverse();
          //K = prevMatch->poseCov1 * ma->delH_delP.transpose() * ma->lineDifCovARinv;
          ma->inov = K * v;
          inovMaha = ma->inov.transpose() * prevMatch->poseCov1.inverse() * ma->inov;
          if (isnan(inovMaha) or inovMaha < 0)
          { // should this be possible? -- it happens, but why?
            if (inovMaha < 0)
            {
              printf("-- gave negative mahalanobi distance %f ??\n", inovMaha);
              inovMaha = -inovMaha;
            }
            else
            {
              inovMaha = 1;
              printf("-- inverse() has returned false\n");
            }
          }
          // forward the worst distance
//           if (inovMaha > prevMatch->inovMaha)
//             ma->inovMaha = inovMaha;
//           else
//             ma->inovMaha = prevMatch->inovMaha;
          ma->inovMaha = prevMatch->inovMaha + inovMaha;
          // update pose and covar so far
          ma->newPose1 = prevMatch->newPose1 + K * v;
          tmpCovIno = K * ma->delH_delP * prevMatch->poseCov1;
          //tmpCovIno = K * ma->lineDifCovAR * K.transpose();
          tmpCov = prevMatch->poseCov1 - tmpCovIno;
        }
        // make symmetric covariance
        ma->poseCov1 = tmpCov + tmpCov.transpose();
        ma->poseCov1 *= 0.5;
        //
        if (not silent)
          printf("laser line %d - match %d/%d (%s) gives maha-dist %.4f sigma\n",
                 llCnt, i, lm->mapLinesCnt, ma->mapLine.name, sqrt(inovMaha));
      }
      else
      { // a no-match case
        if (lm->mapLinesCnt > 0)
        { // skipped a line - but punish the mean innovation
          if (not silent)
            printf("laser line %d - match %d/%d (no match for this line)\n", llCnt, i, lm->mapLinesCnt);
        }
      }
      if (ma == NULL)
        // no match for this line, so previous is inherited
        ma = prevMatch;
      // find next laser-line correlations
      int nxt;
      if (prevMatch == NULL)
      { // we are at the seed line, so next is 0 or 1
        if (seedLaserLine == 0)
          nxt = 1; // seed is same as first, so goto 1
        else if (prevMatch == NULL)
          nxt = 0;
      }
      else
        // use next
        nxt = llCnt + 1;
      if (nxt == seedLaserLine)
        // next is seed line, so skip that
        nxt++;
      // savethis correlation in the test set array
      testSet[llCnt] = i;
      if (nxt < lasMchsCnt)
      { // there are more matches - add those too
        result = testMatches(nxt, ma);
//         { // this laser-to-map match is part of a (p.t.) best match
//           result = true;
//           best[llCnt] = i;
//         }
      }
      else
      { // no more matches, so evaluate
        //static const double MahaDistForNotUsedCorrelation = sqr(0.6);
        int skipped = 0;
        int got = 0;
        double score;
        for (int j = 0; j < lasMchsCnt - 1; j++)
        {
          int mlc = lasMchs[j]->mapLinesCnt;
          if (mlc > 0)
          {
            if (testSet[j] >= mlc)
              skipped++;
            else
              got++;
          }
        }
        if (i >= lm->mapLinesCnt)
          skipped++;
        else
          got++;
        score = (ma->inovMaha + mahaDistForNotUsedCorrelation * skipped) / (got + skipped);
        result = score < mahaScore;
        if (result)
        { // this is best combination of correlations so far,
          // so mark as best, and do so for the rest too (by return true)
          mahaScore = score;
          // transfer test set to best set array
          for (int n = 0; n < lasMchsCnt; n++)
            best[n] = testSet[n];
          usedCorrelationsCnt = got;
          skippedCorrelationsCnt = skipped;
        }
        if (not silent)
          printf("             - maha=%f, best maha is %f, (best result %s)\n", sqrt(score), sqrt(mahaScore), bool2str(result));
      }
    }
    return result;
  }
};


///////////////////////////////////////////////////////////

// void UFuncLoca2::markBestJointMatchesNot()
// {
//   ULocaMatchStat lmd;
//   int mahaMeanCnt = 0;
//   bool found;
//   int m = 0;
//   // make list of laser lines, each with a list of correlated map lines
//   laserMatchesCnt = 0;
//   for (int i = 0; i < matchListCnt; i++)
//   {
//     ULineMatch * match = matchList[i];
//     int line = match->laserLineMatch;
//     if (line >= laserMatchesCnt)
//     {
//       for (int j = laserMatchesCnt; j <= line; j++)
//       {
//         if (laserMatches[j] == NULL)
//           laserMatches[j] = new ULaserMatches();
//         else
//           // reuse and reset map line count
//           laserMatches[j]->mapLinesCnt = 0;
//         // get pointer to laser line segment (for easy access) - for debug print only
//         // laserMatches[j]->laserLine = (U2Dseg*) laserLines[line + 1];
//       }
//       laserMatchesCnt = line + 1;
//     }
//     ULaserMatches * lasMatch = laserMatches[line];
//     lasMatch->mapLines[lasMatch->mapLinesCnt] = match;
//     if (lasMatch->mapLinesCnt + 1 < lasMatch->MLM)
//       lasMatch->mapLinesCnt++;
//     else
//       printf("loca::markBestJointMatches: to many map lines match this laser line (more than %d) - ignored one matched map line\n", lasMatch->MLM);
//     m++;
//   }
//   // mark best match for each line as first guess
//   // first guess is used if no more than 1 laserline with a match is found
//   if (not silent)
//     printf("--------- Laser line match list (has %d matches)\n", m);
//   mahaMean = 0;
//   for (int i = 0; i < laserMatchesCnt; i++)
//   {
//     double mind = 1e5;
//     ULaserMatches * lmi = laserMatches[i];
//     for (int j = 0; j < lmi->mapLinesCnt; j++)
//     {
//       ULineMatch * lm = lmi->mapLines[j];
//       double d = sqr(lm->difA) + sqr(lm->difR);
//       if (d < mind)
//         lmi->best = j;
//     }
//     if (not silent)
//     { // get laser line segment
//       U2Dseg * ls = (U2Dseg*) laserLines[i + 1];
//       printf("laser line %d from %6.2fx,%6.2fy to %6.2fx,%6.2fy (%.1fm) has %d matches [", i,
//            ls->getFirstEnd().x, ls->getFirstEnd().y,
//            ls->getOtherEnd().x, ls->getOtherEnd().y,
//            ls->length, lmi->mapLinesCnt);
//       for (int n = 0; n < lmi->mapLinesCnt; n++)
//         printf("%s ", lmi->mapLines[n]->mapLine.name);
//       printf("]\n");
//     }
//     if (lmi->mapLinesCnt > 1)
//     {
//       for (int j=0; j < lmi->mapLinesCnt; j++)
//         mahaMean += lmi->mapLines[j]->minMahDist;
//       mahaMeanCnt += lmi->mapLinesCnt;
//     }
//   }
//   if (laserMatchesCnt > 0)
//   {
//     if (mahaMeanCnt == 0)
//       mahaMean = 1.0;
//     else
//       mahaMean /= mahaMeanCnt;
//     if (not silent)
//       printf("mean match SD = %.4f (using %d values), used as value for no-match correlation\n", sqrt(mahaMean), mahaMeanCnt);
//     //
//     lmd.init(laserMatches, laserMatchesCnt, mahaMean, silent);
//     lmd.poseOrg = &pose;
//     lmd.poseCovOrg = &poseCov;
//     if (not silent)
//       printf("--------- best joint match iteration\n");
//     found = lmd.testMatches(0, NULL);
//     printf("--------- \n");
//     if (found)
//     { // there should always be a best set of matches
//       ULaserMatches * lm;
//       if (not silent)
//         printf("loca found best matches (in %d lines):\n", laserMatchesCnt);
//       for (int i = 0; i < laserMatchesCnt; i++)
//       {
//         lm = laserMatches[i];
//         if (not silent)
//         { // debug print
//           if (lm->best < lm->mapLinesCnt)
//             printf("  laser line %d has best match %d is %s\n", i, lm->best, lm->mapLines[lm->best]->mapLine.name);
//           else
//             printf("  laser line %d has no best match\n", i);
//         }
//       }
//     }
//   }
//   if (not silent)
//     printf("finished joint match test\n");
// }

///////////////////////////////////////////////////////

/**
 * Test innovation distance for all correlations  based on
 * one of the correlations are right.
 * The series of best correlations are then used - some may be false
 * - First build laser line list with all possible map correlations
 * - then seed an update bast on one of these correlations, and
 * - for each seed, test all other correlations
 * - what should be out? more than 1 sigma mahalanobi?
 * - what about no correlation as alternative, even if within 1 sigma
 */
void UFuncLoca2::markBestJointMatches()
{
  //bool found;
  int m = 0;
  // make list of laser lines, each with a list of correlated map lines
  laserMatchesCnt = 0;
  for (int i = 0; i < matchListCnt; i++)
  {
    ULineMatch * match = matchList[i];
    int line = match->laserLineMatch;
    if (line >= laserMatchesCnt)
    {
      for (int j = laserMatchesCnt; j <= line; j++)
      {
        if (laserMatches[j] == NULL)
          laserMatches[j] = new ULaserMatches();
        else
          // reuse and reset map line count
          laserMatches[j]->mapLinesCnt = 0;
        // get pointer to laser line segment (for easy access) - for debug print only
        // laserMatches[j]->laserLine = (U2Dseg*) laserLines[line + 1];
      }
      laserMatchesCnt = line + 1;
    }
    ULaserMatches * lasMatch = laserMatches[line];
    lasMatch->mapLines[lasMatch->mapLinesCnt] = match;
    if (lasMatch->mapLinesCnt + 1 < lasMatch->MLM)
      lasMatch->mapLinesCnt++;
    else
      printf("loca::markBestJointMatches: to many map lines match this laser line (more than %d) - ignored one matched map line\n", lasMatch->MLM);
    m++;
  }
  //
  if (not silent)
  {
    printf("--------- Laser line match list (has %d matches)\n", m);
    for (int i = 0; i < laserMatchesCnt; i++)
    { // get laser line segment
      ULaserMatches * lmi = laserMatches[i];
      U2Dseg * ls = (U2Dseg*) laserLines[i + 1];
      printf("laser line %d from %6.2fx,%6.2fy to %6.2fx,%6.2fy (%.1fm) has %d matches [", i,
          ls->getFirstEnd().x, ls->getFirstEnd().y,
          ls->getOtherEnd().x, ls->getOtherEnd().y,
          ls->length, lmi->mapLinesCnt);
      for (int n = 0; n < lmi->mapLinesCnt; n++)
        printf("%s ", lmi->mapLines[n]->mapLine.name);
      printf("]\n");
    }
  }
  if (laserMatchesCnt > 0)
  { // test with all correlations as beeing right
    ULocaMatchStat * lmd;
    ULaserMatches * lma;
    int jm = 0;
    int bestSeedSet = 0;
    double bestResult = 100.0;
    for (int c = 0; c < laserMatchesCnt; c++)
    {
      lma = laserMatches[c];
      for (int m = 0; m < lma->mapLinesCnt; m++)
      {
        if (jointMatch[jm] == NULL)
          jointMatch[jm] = new ULocaMatchStat();
        lmd = jointMatch[jm];
        // init this joint match with this seed.
        lmd->init(laserMatches, laserMatchesCnt, 0.0,
                  varLaserAlpha->getDouble(),
                  varLaserR->getDouble(),
                  varMahaDistNoCorr->getDouble(),
                  silent);
        lmd->seedLaserLine = c;
        lmd->seedMapMatch = m;
        // reset best list
        for (int i=0; i < laserMatchesCnt; i++)
          lmd->best[i] = 0;
        // set seed correlation as best - omitted in test
        lmd->best[c] = m;
        //
        if (not silent)
          printf("=========\n"
                 "========= start seed is line %d with map line %d ===============\n", c, m);
        // test all correlation combinations with this seed
        lmd->poseOrg = pose;
        //lmd->poseCovOrg = &poseCov;
        lmd->poseCovOrg.setZero(3, 3);// = &poseCov;
        // initialize to ensure that seed correlation is trusted reasonably much
        lmd->poseCovOrg(0,0) = sqr(0.1);  // x
        lmd->poseCovOrg(1,1) = sqr(0.1);  // y
        lmd->poseCovOrg(2,2) = sqr(0.05);  // heading
        if (not silent)
          printf("--------- best joint match iteration case for laser line %d\n", c);
        // test all combinations with this seed
        lmd->testMatches(c, NULL);
        // count result
//         if (not silent)
//           printf(
//                  "========= seed line %d and map line %d, result: %5.3fmaha, %dcorr %dmis ===============\n",
//                  c, m, sqrt(lmd->mahaScore), lmd->usedCorrelationsCnt, lmd->skippedCorrelationsCnt);
        //if (found)
        //{ // value good correlations and few skipped (deemed false) correlations
          //double result = lmd->mahaScore + lmd->skippedCorrelationsCnt * 0.1;
        if (lmd->mahaScore < bestResult)
        {
          bestSeedSet = jm;
          bestResult = lmd->mahaScore;
        }
        //}
//         else
//           printf("UFuncLoca2::markBestJointMatches !!!! logic error \n");
        if (not silent)
        {
          printf("--- combination series (%d used %d skipped) has best avg mahaDist of %.3f - best=%s\n",
                 lmd->usedCorrelationsCnt, lmd->skippedCorrelationsCnt,
                 sqrt(lmd->mahaScore), bool2str(bestSeedSet == jm));
        }
        jm++;
      }
    }
    if (not silent)
      printf("--------- \n");
    if (jm >= 0)
    { // there should always be a best set of matches
      ULocaMatchStat * lms = jointMatch[bestSeedSet];
      ULaserMatches * lm;
      if (not silent)
        printf("loca found best match is set %d (in %d lines):\n", bestSeedSet, laserMatchesCnt);
      for (int i = 0; i < laserMatchesCnt; i++)
      {
        lm = laserMatches[i];
        lm->best = lms->best[i];
        if (not silent)
        { // debug print
          if (lm->best < lm->mapLinesCnt)
            printf("  laser line %d has best match %d is %s\n", i,
                   lm->best, lm->mapLines[lm->best]->mapLine.name);
          else if (lm->mapLinesCnt > 0)
            printf("  laser line %d has no best match\n", i);
        }
      }
      if (not silent)
        printf("in total %d correlations, %d skipped, worst mahalanobi dist %5.3f\n",
               lms->usedCorrelationsCnt, lms->skippedCorrelationsCnt, sqrt(lms->mahaScore));
    }
  }
  if (not silent)
    printf("finished joint match test\n");
}

UPose interpolatePoses(UPoseTime uPoseTime1, UPoseTime uPoseTime2, UTime timeOfPose) {
    double distFromBeginning = (timeOfPose-uPoseTime1.t) / (uPoseTime2.t-uPoseTime1.t);
    UPose P;
    UPose Pb = uPoseTime1.getPose();
    UPose Pe = uPoseTime2.getPose();
    if (fabs(Pb.h - Pe.h) > M_PI/2.0)
    {
      if (Pb.h > Pe.h)
        Pe.h += 2.0 * M_PI;
      else
        Pb.h += 2.0 * M_PI;
    }
    P.x = Pb.x*(1-distFromBeginning) + Pe.x*(distFromBeginning);
    P.y = Pb.y*(1-distFromBeginning) + Pe.y*(distFromBeginning);
    P.h = Pb.h*(1-distFromBeginning) + Pe.h*(distFromBeginning);
    P.h = limitToPi(P.h);
    return P;
}


void UFuncLoca2::updateDisplacement(UTime &timeOfPose, Matrix<double,3,1> &pose,
                                    Matrix<double,3,3> &poseCov,
                                    UTime scanTime,
                                    UResPoseHist  *poseHist,
                                    int silent, double odoB, double KR, double KL) {
  UPoseTVQ uPoseTime1;
  UPoseTVQ uPoseTime2;
  poseHist->getPoseNearTime(timeOfPose,&uPoseTime1,&uPoseTime2);
//  if(silent==0)
//    printf("Last Image Time:%f\n", timeOfPose.getDecSec());
//  if(silent<2)
//    printf("Image Time:%f\n", scanTime.getDecSec());
  if(not silent) {
    printf("Pose Time %.3f to %.3f %.3f secs\n",timeOfPose.getDecSec(), scanTime.getDecSec(), scanTime - timeOfPose);
  }

  UPose P1;
  if(uPoseTime1.t <= timeOfPose && uPoseTime2.t >= timeOfPose) {
    P1 = interpolatePoses(uPoseTime1, uPoseTime2, timeOfPose);
  } else {
    P1 = uPoseTime1.getPose();
    timeOfPose = uPoseTime1.t;
  }


  while (1) {
    if(timeOfPose == scanTime)
      break;
    if (poseHist->getNewest(0).t==timeOfPose) {
      if (not silent)
        cout << "\tpose list end\n";
      break;
    }
    if (not poseHist->getPoseNearTime(timeOfPose+0.001,&uPoseTime1,&uPoseTime2))
      break;
    UPose P2;
      //cout << "\tPose Time Diff:\n";// << uPoseTime2.t.getDecSec()- scanTime << "\n";
    if (uPoseTime2.t > (scanTime + 0.001)) {
      P2 = interpolatePoses(uPoseTime1, uPoseTime2, scanTime);
      timeOfPose = scanTime;
//      if (not silent)
//        cout << "\tTime at scan reached\n";
    } else {
      P2 = uPoseTime2.getPose();
      timeOfPose = uPoseTime2.t;
    }
    double D, L, delSr, delSl;
    D = fmod(P2.h - P1.h + 3 * M_PI, 2 * M_PI) - M_PI;
    double xDif = (P2.x - P1.x);
    double yDif = (P2.y - P1.y);
    double dist = hypot(xDif, yDif);
    // scale with odo offset (scale)
    if (dist > 1e-4)
    { // scale with offset in speed
      double scale = 1 - varOffsetValue->getDouble(2) / dist;
      xDif *= scale;
      yDif *= scale;
    }
    // signed distance
    L = sqrt( xDif * xDif +
              yDif * yDif) * ((cos(P1.h) * xDif + sin(P1.h) * yDif) * 2 - 1);
    // this looks like an error - replaced with the one above / chr 8/9/2013
//     L = sqrt( xDif * xDif +
//               yDif * yDif) * ((cos(P1.h) * xDif + sin(P1.h) * yDif>0) * 2 - 1);
    //cout << "\tD:" << D << " L:" << L << "\n";

    delSr = L + odoB * D / 2;
    delSl = L - odoB * D / 2;

    double th_ = pose(2,0) + D / 2;

    double thd = limitToPi(P2.h-P1.h);
    if (dist > 1e-4)
      // scale with fixed offset to turn rate
      thd = limitToPi(thd * (1 - varOffsetValue->getDouble(3)));
    double xd =  cos(P1.h) * (xDif) + sin(P1.h) * (yDif);
    double yd = -sin(P1.h) * (xDif) + cos(P1.h) * (yDif);
    double thl = pose(2,0);
    // save old pose
    Matrix<double,3,1> poseOld = pose;;
    // update to new pose, xd forward and yd left of current pose
    Matrix<double,3,1> poseDif;
    poseDif << cos(thl)*xd - sin(thl)*yd,
               sin(thl)*xd + cos(thl)*yd,
               thd;
    pose += poseDif;
    pose(2,0) = limitToPi(pose(2,0));
    // calculate new covariance
    Matrix<double,3,2> delPnew_delY;
    delPnew_delY << cos(th_)/2-L*sin(th_)/(2*odoB),    cos(th_)/2+L*sin(th_)/(2*odoB),
                    sin(th_)/2+L*cos(th_)/(2*odoB),    sin(th_)/2-L*cos(th_)/(2*odoB),
                    1/odoB,                            -1/odoB;

    Matrix<double,3,3> delPnew_delPold;
    delPnew_delPold<<  1,    0,      -L*sin(th_),
                      0,    1,      L*cos(th_),
                      0,    0,      1;

    Matrix<double,2,2> covU;

    covU << sqr(KR) * fabs(delSr),    0,
            0                   ,    sqr(KL) * fabs(delSl);

    //printf("Robot %gbase l=%g R=%g\b", odoB, varKL->getValued(), varKR->getValued());

    //covOut = delPnew_delPold*covIn*delPnew_delPold'+delPnew_delY*covU*delPnew_delY';*/

    poseCov << delPnew_delY *covU * delPnew_delY.transpose()
            +delPnew_delPold * poseCov * delPnew_delPold.transpose();

    if (not silent)
      printf(" - position update to %.3f: %.3fm %.5frad odo (%.3fx %.3fy %.4fr->%.3fx %.3fy %.4fr moved %.3fm dir %.5fh) "
                "map (%.3fx %.3fy %.4fr->%.3fx %.3fy %.4fr moved %.3fm dir %.5fh)\n",
                 timeOfPose.getDecSec(),
                 L, thd, P1.x, P1.y, P1.h, P2.x, P2.y, P2.h, hypot(P2.y - P1.y, P2.x - P1.x), atan2(P2.y - P1.y, P2.x - P1.x),
                 poseOld(0,0), poseOld(1,0), poseOld(2,0), pose(0,0), pose(1,0), pose(2,0),
                 hypot(pose(1,0) - poseOld(1,0), pose(0,0) - poseOld(0,0)),
                 atan2(pose(1,0) - poseOld(1,0), pose(0,0) - poseOld(0,0))
            );
    P1 = P2;
  }

  if (not silent)
    cout << "updated displacement\n";
}
