/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include <iostream>
using namespace std;
#include <limits>

#include <urob4/usmltag.h>

#include "ufunccamlocalize.h"
#include <LEL_utilities.h>
#include <utils/localizationutils.h>

//#include <iau_ukf.hpp>

#ifdef LIBRARY_OPEN_NEEDED

void libraryOpen(void)
{ // called when server opens this plugin (i.e. calls dlopen())
	printf("Loaded LineResource plugin\n");
	// add code as needed
	// for global logfiles etc ...
}

///////////////////////////////////////////////////

void libraryClose(void)
{ // called when server unloads this plugin (i.e. calls dlclose())
	printf("Unloaded LineResource plugin\n");
	// add code as needed
}

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
		{ // create an object of this type
	return new UFuncCamLocalize();
		}

///////////////////////////////////////////////////

void deleteFunc(UFunctionBase * p)
{ // delete the object
	delete p;
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncCamLocalize::UFuncCamLocalize(): table("kl1e-2table")
{ // initialization of variables in class - as needed
	prepareTables();
	createBaseVar();
	lastScanTime.setTime(-1);
	poseIndex = 0;
}

bool UFuncCamLocalize::handleResetLM(UServerInMsg * msg)
{
	return true;
}

///////////////////////////////////////////////////

UFuncCamLocalize::~UFuncCamLocalize()
{ // possibly remove allocated variables here - if needed
	freeTables();

	freeLines();
}

void UFuncCamLocalize::createBaseVar()
{
	varOdoB = addVar("wheelBase", 0.26, "d", "(rw) (odoB) wheel base for assumed differential drive robot");
	/// distance varaince for right wheel each moved meter
	varKR = addVar("sdmRight", sqrt(0.003), "d", "(rw) (kR) variance each driven meter - as sd of err each m (right)");
	/// distance varaince for left wheel each moved meter
	varKL = addVar("sdmLeft", sqrt(0.003), "d", "(rw) (kL) variance each driven meter - as sd (left)");
	//varDefinedLines = addVar("lines", 0.0, "d", "(r) Number lines defined in localizer");
	/// The camera tilt, positive means camera looking up
	varCameraTilt = addVar("cameraTilt", 0.0, "d", "(rw) (cameraTilt) The camera tilt in radians, positive means camera looking up");
	/// The x coordinate of the camera in the robot frame
	varCameraX = addVar("cameraX", 0.0, "d", "(rw) (cameraX) The x coordinate of the camera in the robot frame");
	/// The y coordinate of the camera in the robot frame
	varCameraY = addVar("cameraY", 0.0, "d", "(rw) (cameraY) The y coordinate of the camera in the robot frame");
	/// The z coordinate of the camera in the robot frame
	varCameraZ = addVar("cameraZ", 0.40, "d", "(rw) (cameraZ) The z coordinate of the camera in the robot frame");
	/// The x pixel coordinate of the image center
	varCx = addVar("cx", 159.5, "d", "(rw) (cx) The x pixel coordinate of the image center");
	/// The y pixel coordinate of the image center
	varCy = addVar("cy", 119.5, "d", "(rw) (cy) The y pixel coordinate of the image center");
	/// The camera focal distance
	varFp = addVar("focalDistance", 350, "d", "(rw) (fp) The camera focal distance");

	varMaxXYVariance = addVar("maxXYVariance", 0.05, "d",
			"(rw) The maximum variance of each hypothesis along the x and y dimensions");
	varMaxThVariance = addVar("maxThVariance", 0.05, "d",
			"(rw) The maximum variance of each hypothesis along the theta dimension");
	/// The minimum image line strength
	varLineStr = addVar("minLineStr", 100, "d", "(rw) The minimum pixel support for an image line to be accepted");
	// Default Silence Level
	varDefaultSilence = addVar("defaultSilence", 2, "d", "(rw) The default silence level");


	//  varPointsInThreshold = addVar("pointsIn", 0.1 , "d", "(r/w) number of points supporting line that must be inside line segment");
	//  varCovar = addVar("covar", "0 0 0; 0 0 0; 0 0 0", "m2", "(r) Current covariance martix (x, y, th)");
	//  varCovarV = addVar("covarV", "0 0; 0 0", "d", "(r) covariance ellipse vectors - x,y in rows");
	//  varCovarA = addVar("covarA", "0 0", "d", "(r) covariance ellipse length - major; minor; degrees (1-sigma)");
	//  varUpdates = addVar("hit", 0.0, "d", "(r) Number of successful updates (total)");
	//  varFailed = addVar("mis", 0.0, "d", "(r) Number of scans with no update since last update");
}

void UFuncCamLocalize::freeLines(){
	vector<worldLine3d>::iterator itlines;
	for(itlines=worldLines.begin() ; itlines != worldLines.end(); itlines++)
		delete (*itlines).name;

	worldLines.clear();
}

///////////////////////////////////////////////////

const char * UFuncCamLocalize::name()
{
	return "auNavVision (" __DATE__ " " __TIME__ " by Enis BAYRAMOGLU)";
}

///////////////////////////////////////////////////

const char * UFuncCamLocalize::commandList()
{ // space separated list og command keywords handled by this plugin
	return "resetlocalizer localize addline setinitpose setinitcov";
}

///////////////////////////////////////////////////

bool UFuncCamLocalize::setResource(UResBase * resource, bool remove)
{ // load resource as provided by the server (or other plugins)
	bool result = true;

	if (resource->isA(UResPoseHist::getOdoPoseID()))
	{ // pointer to server the resource that this plugin can provide too
		// but as there might be more plugins that can provide the same resource
		// use the provided
		if (remove)
			// the resource is unloaded, so reference must be removed
			poseHist = NULL;
		else if (poseHist != (UResPoseHist *)resource)
			// resource is new or is moved, save the new reference
			poseHist = (UResPoseHist *)resource;
		else
			// reference is not used
			result = false;
	}

	// other resource types may be needed by base function.
	result = UFunctionCamBase::setResource(resource, remove);
	return result;
}


///////////////////////////////////////////////////

bool UFuncCamLocalize::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
	bool result = false;
	// Test for the handled commands, and call a function to do the job
	// the tagname is not case sensitive - see the library documentation for
	// 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
	if (msg->tag.isTagA("localize"))
		result = handleLocalize(msg,(UImage *) extra);
	else if (msg->tag.isTagA("addline"))
		result = handleAddLine(msg);
	else if (msg->tag.isTagA("setinitpose"))
		result = handleSetInitPose(msg);
	else if (msg->tag.isTagA("setinitcov"))
		result = handleSetInitCov(msg);
	else if (msg->tag.isTagA("resetlocalizer"))
		result = handleResetLocalizer();
	else if (msg->tag.isTagA("outputdist"))
		result = handleOutputDist(msg);
	else if (msg->tag.isTagA("settable"))
		result = handleSetTable(msg);
	else
		sendDebug(msg, "Command not handled (by me)");
	return result;
}

bool UFuncCamLocalize::handleOutputDist(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL]="dist";
	bool ask4help;
	//
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "OUTPUTDIST");
		sendText(msg, "--- Description of outputdist command ---\n");
		sendText(msg," Write the current distribution to the given file name.\n");
		sendText(msg," Example:\n");
		sendText(msg," outputdist filename=dist\n");
		sendHelpDone(msg);
		return true;
	}
	if (msg->tag.getAttValue("filename", val, MVL)) {
		ofstream the_file;
		the_file.open(val);
		the_file<<poseDist;
		the_file.close();
	}
	return true;
}

bool UFuncCamLocalize::handleSetTable(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL]="dist";
	bool ask4help;
	//
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help) { // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "SETTABLE");
		sendText(msg, "--- Description of settable command ---\n");
		sendText(msg," Set the splitting table to be used.\n");
		sendText(msg," Example:\n");
		sendText(msg," settable filename=dist\n");
		sendHelpDone(msg);
		return true;
	}
	if (msg->tag.getAttValue("filename", val, MVL)) {
		table = SplitTable<1>(val);
	}
	return true;
}

////////////////////////////////////////////////////////////

//const char * UFuncCamLocalize::print(const char * preString, char * buff, int buffCnt)
//{
//    snprintf(buff, buffCnt, "%s I don't need a resource\n",
//                  preString);
//    return buff;
//}

bool UFuncCamLocalize::handleResetLocalizer() {
	freeLines();
	return true;

}

bool UFuncCamLocalize::handleSetInitPose(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL];  double x;
	double xavailable=false;
	double y;
	double yavailable=false;
	double th;
	double thavailable=false;
	bool ask4help;
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help)
	{ // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "SETINITPOSE");
		sendText(msg, "--- setinitpose options:\n");
		sendText(msg, " setinitpose accepts no options\n\n");
		sendText(msg, "--- Description of setinitcov command ---\n");
		sendText(msg, " set the initial robot pose to be used by localize.\n\n");
		sendText(msg, "--- Usage ---\n");
		sendText(msg, " setinitpose has to be called with all tree pose parameters as follows:\n");
		sendText(msg, " setinitcov x='robot x position(in meters)' y='robot y position' th='robot heading'\n");
		sendHelpDone(msg);
		return true;
	}
	if (msg->tag.getAttValue("x", val, MVL)){
		x = strtod(val, NULL); xavailable=true;}
	if (msg->tag.getAttValue("y", val, MVL)){
		y = strtod(val, NULL); yavailable=true;}
	if (msg->tag.getAttValue("th", val, MVL)){
		th = strtod(val, NULL); thavailable=true;}

	if(xavailable && yavailable && thavailable) {
		pose << x,y,th;
		return true;
	} else {
		cout<<"inappropriate arguments for setinitpose";
		return false;
	}
}
bool UFuncCamLocalize::handleSetInitCov(UServerInMsg * msg) {
	const int MVL = 50;
	char val[MVL];
	double Cx;
	double Cxavailable=false;
	double Cy;
	double Cyavailable=false;
	double Cth;
	double Cthavailable=false;
	bool ask4help;
	ask4help = msg->tag.getAttValue("help", val, MVL);
	if (ask4help)
	{ // create the reply in XML-like (html - like) format
		sendHelpStart(msg, "SETINITCOV");
		sendText(msg, "--- setinitcov options:\n");
		sendText(msg, " setinitcov accepts no options\n\n");
		sendText(msg, "--- Description of setinitcov command ---\n");
		sendText(msg, " set the covariance of the initial robot pose to be used by localize.\n\n");
		sendText(msg, "--- Usage ---\n");
		sendText(msg, " setinitcov has to be called with all tree covariance parameters as follows:\n");
		sendText(msg, "setinitcov Cx='variance of x(in meters)' Cy='variance of y' Cth='variance of heading'\n");
		sendHelpDone(msg);
		return true;
	}

	if (msg->tag.getAttValue("Cx", val, MVL)){
		Cx = strtod(val, NULL); Cxavailable=true;}
	if (msg->tag.getAttValue("Cy", val, MVL)){
		Cy = strtod(val, NULL); Cyavailable=true;}
	if (msg->tag.getAttValue("Cth", val, MVL)){
		Cth = strtod(val, NULL); Cthavailable=true;}

	if(Cxavailable && Cyavailable && Cthavailable) {
		poseCov << Cx,0,0,0,Cy,0,0,0,Cth;
		GaussianHypothesis<3> initDist(pose,poseCov,1);
		Matrix<double, 3, 1> maxVariances;
		maxVariances << varMaxXYVariance->getValued(), varMaxXYVariance->getValued(), varMaxThVariance->getValued();
		GaussianHypothesis<3>::list splittedList;
		poseDist.split(initDist,splittedList,maxVariances,table);
		poseDist.GHlist.splice_after(poseDist.GHlist.begin(),splittedList);
		return true;
	} else {
		cout<<"inappropriate arguments for setinitcov";
		return false;
	}
}

bool UFuncCamLocalize::handleLocalize(UServerInMsg * msg, UImage* pushImg)
{
	// send a reply back to the client requested the 'bark'
	const int MRL = 2000;
	char reply[MRL];
	bool ask4help;
	const int MVL = 50;
	char val[MVL];
	bool gotDevice = false;
	int camDevice = -1;
	bool gotImg = false;
	int imgPoolNum = -1;
	UImage * img = pushImg;
	USmlTag tag;
	UCamPush * cam = NULL;
	bool result;
	bool debug = true; // default is debug on
	float p11, p12, p13, p22, p23, p33;
	bool p11available=false, p12available=false, p13available=false,
			p22available=false, p23available=false, p33available=false, psavailable=false;
	bool saveimage = false;
	// check for parameters - one parameter is tested for - 'help'
	// the help value is ignored, e.g. if help="bark", then
	// the value "bark" will be in the 'helpValue' string.
	ask4help = msg->tag.getAttValue("help", val, MVL);
	int silent = (int)varDefaultSilence->getValued();
	msg->tag.getAttInteger("silent", &silent, 0);
	if (not ask4help)
	{ // get all other parameters
		if (msg->tag.getAttValue("device", val, MVL))
		{
			gotDevice = true;
			camDevice = strtol(val, NULL, 0);
		}
		if (msg->tag.getAttValue("img", val, MVL))
		{
			gotImg = true;
			imgPoolNum = strtol(val, NULL, 0);
		}
		if (msg->tag.getAttValue("debug", val, MVL))
			debug = str2bool2(val, true);

		if (msg->tag.getAttValue("saveimage", val, MVL))
			saveimage = true;

		if (msg->tag.getAttValue("p11", val, MVL)){
			p11 = strtol(val, NULL, 0); p11available=true;}
		if (msg->tag.getAttValue("p12", val, MVL)){
			p12 = strtol(val, NULL, 0); p12available=true;}
		if (msg->tag.getAttValue("p13", val, MVL)){
			p13 = strtol(val, NULL, 0); p13available=true;}
		if (msg->tag.getAttValue("p22", val, MVL)){
			p22 = strtol(val, NULL, 0); p22available=true;}
		if (msg->tag.getAttValue("p23", val, MVL)){
			p23 = strtol(val, NULL, 0); p23available=true;}
		if (msg->tag.getAttValue("p33", val, MVL)){
			p33 = strtol(val, NULL, 0); p33available=true;}
	}
	if(p11available&&p12available&&p13available&&p22available&&p23available&&p33available)
		psavailable=true;
	// ask4help = false, if no 'help' option were available.
	if (ask4help)
	{ // create the reply in XML-like (html - like) format
		sendMsg( msg, "<help subject=\"HOUGH\">\n");
		sendText( msg, "--- available GPS options\n");
		sendText(msg, "device=X          Use this camera - for position and parameters\n");
		sendText(msg, "img=X             Get image from image pool - else take new image\n");
		sendText(msg, "debug=false       More images and print on server console (def=true)\n");
		sendText(msg, "help              This message\n");
		sendMsg( msg, "</help>\n");
		sendInfo(msg, "done");
		result = true;
	}
	else
	{ // resource is available, so make a reply
		if (gotImg)
		{ // take image from image pool
			img = imgPool->getImage(imgPoolNum, false);
			result = (img != NULL);
			/*      if (result and camDevice < 0)
        //take image device from image
        camDevice = img->camDevice;
      if (result)
      {
        cam = camPool->getCam(camDevice);
        result = (cam != NULL);
      }*/
		}
		else if (img != NULL)
		{ // we have an image, so just camera is needed
			camDevice = img->camDevice;
			cam = camPool->getCam(camDevice);
			result = (cam != NULL);
		}
		else
		{ // get new image from a camera and store in first imagepool number for ball images
			img = imgPool->getImage(UIMGPOOL_NAVVISION_BASE, true);
			result =getCamAndRawImage(&cam,        // result camera           out
					&img,        // result image            out
					&camDevice,  // camera device number    in-out
					NULL,        // pushed image (YUV)      in
					"",0);         // camera position name    in
			if (result)
				result = (img != NULL);
		}
		// camera and image is now available
		// time to kick some ass
		if (result)
		{
			UTime scanTime = img->imgTime; // getImgUpdateTime();
			UPose uNewPose = poseHist->getPoseAtTime(scanTime);



			if (lastScanTime.getDecSec()==-1)
			{
				if (silent < 2)
					cout << "processingFirstScan\n";
			}
			else {
				updateDisplacement(lastScanTime, pose, poseCov, scanTime, poseHist,
						silent, varOdoB->getValued(), varKR->getValued(), varKL->getValued());
//				UPoseTVQ uPoseTime1;
//				UPoseTVQ uPoseTime2;
//				poseHist->getPoseNearTime(lastScanTime,&uPoseTime1,&uPoseTime2);
//				UPose P1 = uPoseTime1.getPose();
//				if(silent==0)
//					printf("Last Image Time:%f\n", lastScanTime.getDecSec());
//				if(silent<2)
//					printf("Image Time:%f\n", scanTime.getDecSec());
//				if(silent==0) {
//					printf("Pose Time1:%f\n",uPoseTime1.t.getDecSec());
//					printf("Pose Time2:%f\n",uPoseTime2.t.getDecSec());
//				}
//				while (1) {
//					if (poseHist->getNewest(0).t==uPoseTime1.t) {
//						if (silent == 0)
//							cout << "\tpose list end\n";
//						break;
//					}
//					if (not poseHist->getPoseNearTime(uPoseTime1.t+0.001,&uPoseTime1,&uPoseTime2))
//						break;
//						UPose P2 = uPoseTime2.getPose();
//						//cout << "\tPose Time Diff:\n";// << uPoseTime2.t.getDecSec()- scanTime << "\n";
//					if (uPoseTime2.t > (scanTime + 0.001)) {
//						if (silent == 0)
//							cout << "\tTime at scan reached\n";
//						break;
//					}
//					double D, L, delSr, delSl;
//					D = fmod(P2.h - P1.h + 3 * M_PI, 2 * M_PI) - M_PI;
//					double xDif = (P2.x - P1.x);
//					double yDif = (P2.y - P1.y);
//					L = sqrt( xDif * xDif +
//							yDif * yDif)*((cos(P1.h)*xDif+sin(P1.h)*yDif>0)*2-1);
//					//cout << "\tD:" << D << " L:" << L << "\n";
//
//					double odoB = varOdoB->getValued();
//					delSr = L + odoB*D/2;
//					delSl = L - odoB*D/2;
//
//					double th_ = pose(2,0)+D/2;
//
//					double thd = P2.h-P1.h;
//					double xd = cos(P1.h)*(P2.x-P1.x)+sin(P1.h)*(P2.y-P1.y);
//					double yd = -sin(P1.h)*(P2.x-P1.x)+cos(P1.h)*(P2.y-P1.y);
//					double thl = pose(2,0);
//
//					Matrix<double,3,1> poseDif;
//					poseDif << cos(thl)*xd-sin(thl)*yd,sin(thl)*xd+cos(thl)*yd,thd;
//					pose+=poseDif;
//
//					Matrix<double,3,2> delPnew_delY;
//					delPnew_delY<<cos(th_)/2-L*sin(th_)/(2*odoB),    cos(th_)/2+L*sin(th_)/(2*odoB),
//								  sin(th_)/2+L*cos(th_)/(2*odoB),    sin(th_)/2-L*cos(th_)/(2*odoB),
//								  1/odoB,                        	-1/odoB;
//
//					Matrix<double,3,3> delPnew_delPold;
//					delPnew_delPold<<  1,    0,      -L*sin(th_),
//									   0,    1,      L*cos(th_),
//									   0,    0,      1;
//
//					Matrix<double,2,2> covU;
//					covU<<  sqr(varKR->getValued()) * fabs(delSr),    0,
//							0                                    ,    sqr(varKL->getValued()) * fabs(delSl);
//
//					//printf("Robot %gbase l=%g R=%g\b", odoB, varKL->getValued(), varKR->getValued());
//
//					//covOut = delPnew_delPold*covIn*delPnew_delPold'+delPnew_delY*covU*delPnew_delY';*/
//
//					poseCov = delPnew_delY *covU * delPnew_delY.transpose()
//								  +delPnew_delPold * poseCov * delPnew_delPold.transpose();
//
//					uPoseTime1 = uPoseTime2;
//					P1 = P2;
//					poseIndex++;
//					if(silent==0)
//						cout<<"-*";
//				}
//
//				if (silent < 2)
//					cout << "\tupdated displacement\n";
			}
			if(silent<2)
				cout<<"\n\n---------------Starting new image----------------\n";

			CvMat* dx,*dy;
			UImage * imgBW = imgPool->getImage(UIMGPOOL_NAVVISION_BASE,true,img->getHeight(),img->getWidth(),1,8);
			UImage * edges = imgPool->getImage(UIMGPOOL_NAVVISION_BASE+1,true,img->getHeight(),img->getWidth(),1,8);
			UImage * processedImg = imgPool->getImage(UIMGPOOL_NAVVISION_BASE+2,true,img->getHeight(),img->getWidth(),img->getChannels(),img->getDepth());
			list<LEL_GroupedLine> GLL;

			IplImage * IplProcessedImg = processedImg->getIplImage();
			cvCopy( img->getIplImage(), IplProcessedImg, NULL );

			img->toBW(imgBW);

			IplImage * imgIpl = imgBW->getIplImage();

			dy = cvCreateMat( imgIpl->height, imgIpl->width, CV_16SC1 );
			dx = cvCreateMat( imgIpl->height, imgIpl->width, CV_16SC1 );
			cvSobel( imgIpl, dx, 1, 0, 3 );
			cvSobel( imgIpl, dy, 0, 1, 3 );

			cvModCanny(dx,dy,edges->getIplImage(),200,60);

			hough(edges->cvArr(), GLL);
			if(silent==0)
				cout << "Number of image lines: "<< GLL.size() << "\n";

			list<LEL_GroupedLine *> GLPL, matchedLines;
			//vector<int> matchIndices;
			for(list<LEL_GroupedLine>::iterator it=GLL.begin();it!=GLL.end();it++){
				if(it->edgeCount>varLineStr->getValued())
					GLPL.push_front(&(*it));
			}


			arrangeGLines(IplProcessedImg,GLPL,25,CV_RGB(255,255,255),1);

			if(silent<2)
				cout << "Number of considered lines: "<< GLPL.size() << "\n";

			double devThreshold = 3*3;
			//double pointsInThreshold = 0.1;

			if(silent<2) {
				cout<<"\nprinting pose and poseCov before measurement update\n";
				cout<<pose;
				cout<<poseCov;
			}
			Matrix<double,3,1> newPose = pose;

			//double inc = 0.00001;

			for(unsigned int i=0;i<worldLines.size();i++) {
				Matrix<double,2,1> projLine;
				Matrix<double,2,2> projLineCov;

				lineRange range = lineInImage(pose,worldLines[i]);

				iau_ukf::unscentedtransform(pose,poseCov,projLine,projLineCov,worldLines[i],*this,&UFuncCamLocalize::projectToImage);

				// compliance test


//				projectToImage(linePoints[i],lineVecs[i],pose,poseCov,projLine,lineDifCov,delH_delP,delH_delPTrans);


				Matrix<double,2,2> lineDifCov = projLineCov;

				Matrix<double,2,2> noiseCov; noiseCov<<varAlpha,0,0,varR;

				lineDifCov+=noiseCov;

				if(silent==0) {
					cout<<"map line: "<< worldLines[i].name <<" alpha: " <<projLine(0,0) << " line r: " << projLine(1,0) << " lineCov:\n"<<lineDifCov;
					cout<<"line range:"<<range.lb<<" -> "<<range.le<<"\n";
				} else	if(silent==1) {
					cout<<"\n"<<worldLines[i].name<<"; a:"<<projLine(0,0)<<" r:"<<projLine(1,0)<<" range:"<<range.lb<<"->"<<range.le<<"::  ";
				}

				if(isnan(range.lb)) {
					if(silent<2)
						cout<<" this line is behind the camera\n";
					continue;
				}

				/*          mput(pose,0,0,mget(pose,0,0)+inc);

          projectToImage(linePoints[i],lineVecs[i],pose,poseCov,projLineInc,lineCov,delH_delP,delH_delPTrans);

          cout<<"delH_delP:\n";
          mprint(delH_delP);

          cout<<"calculated derivatives:\n" <<(projLineInc.alpha-projLine.alpha)/inc<<", " << (projLineInc.r-projLine.r)/inc <<"\n";*/
				double minMahDist = devThreshold;
				//LEL_GFLine closestLine;
				LEL_GroupedLine* matchedLine;
				Matrix<double,2,1> lineDifMin; Matrix<double,2,2> lineDifCovMin;
				for ( list<LEL_GroupedLine*>::iterator itLas=GLPL.begin() ; itLas != GLPL.end(); itLas++ ) {
					if(silent==0)
						cout<<"\timage line alpha: "<<(*itLas)->Th<<" line r:" << (*itLas)->R << " lb:"<<(*itLas)->lb << " le:"<<(*itLas)->le;
					else if(silent==1)
						cout<<(*itLas)->Th<<","<<(*itLas)->R;
					double angleDif = fmod((*itLas)->Th-projLine(0,0)+3*M_PI,2*M_PI)-M_PI;
					double rDif;
					if(fabs(angleDif)>M_PI/2) {
						angleDif = fmod(angleDif+2*M_PI,2*M_PI)-M_PI;
						rDif = -(*itLas)->R-projLine(1,0);
					} else
						rDif = (*itLas)->R-projLine(1,0);
					Matrix<double,2,1> lineDif;
					lineDif<<angleDif, rDif;
					double mahDist = lineDif.transpose()*lineDifCov.inverse()*lineDif;
					if(silent==0)
						cout<<"\tmah dist: "<< mahDist;
					if(range.lb>(*itLas)->le || range.le<(*itLas)->lb) {
						if(silent==0)
							cout<<" (X-limits)\n";
						else if(silent==1)
							cout<<"(X);";
						continue;
					}
					if(silent==0)cout<<"\n";else if(silent==1)cout<<";";
					if(mahDist<minMahDist) {
						matchedLine = (*itLas);
						minMahDist = mahDist;
						lineDifMin=lineDif;
						lineDifCovMin = lineDifCov;
						//closestLine = *itLas;
					}
				}
				if(minMahDist<devThreshold) {
		            		cout<<"Localizer: Line match to " << worldLines[i].name << "\n";
		 			// We have a match!
					matchedLines.push_front(matchedLine);
					Matrix<double,2,1> meas; meas << matchedLine->Th,matchedLine->R;
					iau_ukf::update(pose,poseCov,meas,noiseCov,worldLines[i],*this,&UFuncCamLocalize::projectionError);

				}

			}

			arrangeGLines(IplProcessedImg,matchedLines,25,CV_RGB(255,100,0),1);

			cout<<"\nprinting pose and poseCov after measurement update\n";
			cout<<pose;
			cout<<poseCov;

			lastScanTime = scanTime;

			UPose poseAtScan = poseHist->getPoseAtTime(scanTime);

			UPose trans;

			double poseX = pose(0,0);
			double poseY = pose(1,0);
			double poseTh = pose(2,0);

			trans.h = poseAtScan.h-poseTh;
			trans.x = poseAtScan.x-(cos(trans.h)*poseX-sin(trans.h)*poseY);
			trans.y = poseAtScan.y-(sin(trans.h)*poseX+cos(trans.h)*poseY);

			/**
		    SMRDEMO reply format */
			if (msg->client >= 0)
			{ // send to real client only, not a push in the .ini file
				snprintf(reply, MRL, "<vision vis0=\"%g\" vis1=\"%g\" vis2=\"%g\" />\n",
						trans.x, trans.y, trans.h);

				// send this string as the reply to the client
				sendMsg(msg, reply);
			}

		}
		else
		{
			snprintf(reply, MRL, "failed, got image %s, got camera %d %s\n",
					bool2str(img != NULL), camDevice, bool2str(cam != NULL));
			sendWarning(msg, reply);
		}
	}
	// return true if the function is handled with a positive result
	return result;
}

lineRange UFuncCamLocalize::lineInImage(Matrix<double,3,1> pose, worldLine3d worldLine) {
	double posex = pose(0,0);
	double posey = pose(1,0);
	double poseTh = pose(2,0);

	double cameraTilt = varCameraTilt->getValued();
	double cameraX = varCameraX->getValued();
	double cameraY = varCameraY->getValued();
	double cameraZ = varCameraZ->getValued();

	double cx = varCx->getValued();
	double cy = varCy->getValued();

	double fp = varFp->getValued();

	Matrix<double,3,3,true> KCam;KCam<< fp, 0 , cx,
									    0,  fp, cy,
									    0,  0,  1.0;

	Matrix<double,3,3> R; R << sin(poseTh),                  -cos(poseTh),                   0,
							   sin(cameraTilt)*cos(poseTh),   sin(cameraTilt)*sin(poseTh),   -cos(cameraTilt),
							   cos(cameraTilt)*cos(poseTh),   cos(cameraTilt)*sin(poseTh),   sin(cameraTilt);

	Matrix<double,3,1> cameraPosition; cameraPosition << posex+cos(poseTh)*cameraX-sin(poseTh)*cameraY,
													     posey+sin(poseTh)*cameraX+cos(poseTh)*cameraY,
													     cameraZ;

	Matrix<double,3,1> cameraSPoint = R*(worldLine.SPoint - cameraPosition);
	Matrix<double,3,1> imageSPoint = KCam*cameraSPoint;
	Matrix<double,3,1> cameraEPoint = R*(worldLine.EPoint - cameraPosition);
	Matrix<double,3,1> imageEPoint = KCam*cameraEPoint;

	Matrix<double,3,1> imageVec = KCam * R * worldLine.Vec;

	double iPx = imageSPoint(0,0), iPy = imageSPoint(1,0), iPz = imageSPoint(2,0);
	double iVx = imageVec(0,0)  , iVy = imageVec(1,0)  , iVz = imageVec(2,0);

	double iDx = iVx*iPz-iVz*iPx;
	double iDy = iVy*iPz-iVz*iPy;

	//cout<<"iDx:"<<iDx<<" iDy:"<<iDy<<"\n";

	double alpha_ = atan2(iDy,iDx);
	double alpha = alpha_+M_PI/2;

	//double l = sqrt(iDx*iDx+iDy*iDy);

	double r = (cos(alpha)*iPx+sin(alpha)*iPy)/iPz;//(iVx*iPy-iVy*iPx)/l;

	double pSx = imageSPoint(0,0)/imageSPoint(2,0);
	double pSy = imageSPoint(1,0)/imageSPoint(2,0);

	double pEx = imageEPoint(0,0)/imageEPoint(2,0);
	double pEy = imageEPoint(1,0)/imageEPoint(2,0);

	double lS = sin(alpha)*pSx - cos(alpha)*pSy;
	double lE = sin(alpha)*pEx - cos(alpha)*pEy;

	if(cameraSPoint(2,0)<0) {
		if(cameraEPoint(2,0)<0) {
			return lineRange(numeric_limits<double>::quiet_NaN(),0);
		}
		else {
			lS = ((lE>lS)?1:-1)*numeric_limits<double>::infinity();
		}
	} else {
		if(cameraEPoint(2,0)<0) {
			lE = ((lE<lS)?1:-1)*numeric_limits<double>::infinity();
		}
	}

	if(r<0){
		r=-r;
		alpha= fmod(alpha+2*M_PI,2*M_PI)-M_PI;
		lS=-lS; lE=-lE;
	} else
		alpha= fmod(alpha+3*M_PI,2*M_PI)-M_PI;

	if(lS>lE) swap(lS,lE);

	return lineRange(lS,lE);
}

Matrix<double,2,1> UFuncCamLocalize::measerreq(Matrix<double,3,1> state, Matrix<double,2,1> noise, Matrix<double,2,1> meas, Matrix<double,3,1> auxin) {
	return Matrix<double,2,1>();
}

Matrix<double,2,1> UFuncCamLocalize::projectionError( Matrix<double,3,1> pose, Matrix<double,2,1> noise, Matrix<double,2,1> meas, worldLine3d line) {
	Matrix<double,2,1> projLine = projectToImage(pose,line) + noise;
	double angleDif = fmod(meas(0,0)-projLine(0,0)+3*M_PI,2*M_PI)-M_PI;
	double rDif;
	if(fabs(angleDif)>M_PI/2) {
		angleDif = fmod(angleDif+2*M_PI,2*M_PI)-M_PI;
		rDif = -meas(1,0)-projLine(1,0);
	} else
		rDif = meas(1,0)-projLine(1,0);
	Matrix<double,2,1> lineDif;
	lineDif<<angleDif, rDif;
	return lineDif;
}

Matrix<double,2,1> UFuncCamLocalize::projectToImage( Matrix<double,3,1> pose, worldLine3d line) {
	Matrix<double,2,1> projLine;

	double posex = pose(0,0);
	double posey = pose(1,0);
	double poseTh = pose(2,0);

	double cameraTilt = varCameraTilt->getValued();
	double cameraX = varCameraX->getValued();
	double cameraY = varCameraY->getValued();
	double cameraZ = varCameraZ->getValued();

	double cx = varCx->getValued();
	double cy = varCy->getValued();

	double fp = varFp->getValued();

	Matrix<double,3,3,true> KCam;KCam<< fp,  0,  cx,
										 0, fp,  cy,
										 0,  0, 1.0;

	Matrix<double,3,3> R; R << sin(poseTh),                  -cos(poseTh),                   0,
							   sin(cameraTilt)*cos(poseTh),   sin(cameraTilt)*sin(poseTh),   -cos(cameraTilt),
							   cos(cameraTilt)*cos(poseTh),   cos(cameraTilt)*sin(poseTh),   sin(cameraTilt);

	Matrix<double,3,1> cameraPosition; cameraPosition << posex+cos(poseTh)*cameraX-sin(poseTh)*cameraY,
													     posey+sin(poseTh)*cameraX+cos(poseTh)*cameraY,
													     cameraZ;

	Matrix<double,3,1> cameraPoint = R*(line.SPoint - cameraPosition);
	Matrix<double,3,1> imagePoint = KCam*cameraPoint;
	Matrix<double,3,1> imageVec = KCam * R * line.Vec;

	double iPx = imagePoint(0,0), iPy = imagePoint(1,0), iPz = imagePoint(2,0);
	double iVx = imageVec(0,0)  , iVy = imageVec(1,0)  , iVz = imageVec(2,0);

	double iDx = iVx*iPz-iVz*iPx;
	double iDy = iVy*iPz-iVz*iPy;

	//cout<<"iDx:"<<iDx<<" iDy:"<<iDy<<"\n";

	double alpha_ = atan2(iDy,iDx);
	double alpha = alpha_+M_PI/2;

	//double l = sqrt(iDx*iDx+iDy*iDy);

	double r = (cos(alpha)*iPx+sin(alpha)*iPy)/iPz;//(iVx*iPy-iVy*iPx)/l;

	if(r<0){
		r=-r;
		alpha= fmod(alpha+2*M_PI,2*M_PI)-M_PI;
	} else
		alpha= fmod(alpha+3*M_PI,2*M_PI)-M_PI;

	//cout<<"r:" <<r<<" alpha:"<<alpha<<" x:"<<posex<<" y:"<<posey<< " th:"<<poseTh<<"\n";

	projLine(0,0) = alpha;
	projLine(1,0) = r;

	return projLine;

}

bool UFuncCamLocalize::handleAddLine(UServerInMsg * msg)
{
  const int MVL = 50;
  char val[MVL];
  double startx;
  double starty;
  double startz;
  double endx;
  double endy;
  double endz;
  bool startxavailable = false;
  bool startyavailable = false;
  bool startzavailable = false;
  bool endxavailable = false;
  bool endyavailable = false;
  bool endzavailable = false;
  bool ask4help;
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "ADDLINE");
    sendText(msg, "--- addline options:\n");
    sendText(msg, " help                  this help\n");
    sendText(msg, "--- Description of addline command ---\n");
    sendText(msg, " This command is used to construct the line map to be used by the localize command. One line is added per call. ");
    sendText(msg, "Lines could be infinite or finite length. localize will not match finite lines to the extracted lines that lie outside their finite range..\n\n");
    sendText(msg, "--- Usage ---\n");
    sendText(msg, " addline direct is called with 2 alternative ways, which have different meanings.\n");
    sendText(msg, " The first way adds an infinitely long line described by its distance from the origin (r) and the angle of its normal(alpha).\n");
    sendText(msg, " addline alpha='the angle of the line normal' r='the line distance to the origin'\n");
    sendText(msg, " The second way adds a finite length line described by its starting and end points.\n");
    sendText(msg, " addline startx='startx' starty='starty' endx='endx' endy='endy'\n");
    sendHelpDone(msg);
    return true;
  }
  if (msg->tag.getAttValue("startx", val, MVL)){
    startx = strtod(val, NULL); startxavailable=true;}
  if (msg->tag.getAttValue("starty", val, MVL)){
    starty = strtod(val, NULL); startyavailable=true;}
  if (msg->tag.getAttValue("startz", val, MVL)){
    startz = strtod(val, NULL); startzavailable=true;}
  if (msg->tag.getAttValue("endx", val, MVL)){
    endx = strtod(val, NULL); endxavailable=true;}
  if (msg->tag.getAttValue("endy", val, MVL)){
    endy = strtod(val, NULL); endyavailable=true;}
  if (msg->tag.getAttValue("endz", val, MVL)){
    endz = strtod(val, NULL); endzavailable=true;}
  
  if(startxavailable && startyavailable && startzavailable && endxavailable && endyavailable && endzavailable) {
    char * name = new char[LEL_ARLine::MNL];
    strcpy(name,"noname");
    msg->tag.getAttValue("name", name, LEL_ARLine::MNL);

    Matrix<double,3,1> newLineSPoint;
    Matrix<double,3,1> newLineEPoint;
    Matrix<double,3,1> newLineVec;

    double deltaX = endx-startx;
    double deltaY = endy-starty;
    double deltaZ = endz-startz;
    double lineLength = sqrt(deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ);

    newLineSPoint << startx, starty, startz;
    newLineEPoint << endx, endy, endz;

    newLineVec << deltaX/lineLength, deltaY/lineLength, deltaZ/lineLength;
    worldLine3d newline(newLineSPoint, newLineEPoint,newLineVec,name);
    worldLines.push_back(newline);

    return true;
  } else {
    cout<<"inappropriate arguments for addline";
    return false;
  }
}

///////////////////////////////////////////////////////

void UFuncCamLocalize::addLine(UPosition p1, UPosition p2)
{
  char * name = new char[LEL_ARLine::MNL];
  strcpy(name,"noname");
  msg->tag.getAttValue("name", name, LEL_ARLine::MNL);

  Matrix<double,3,1> newLineSPoint;
  Matrix<double,3,1> newLineEPoint;
  Matrix<double,3,1> newLineVec;

  double deltaX = p2.x-p1.x;
  double deltaY = p2.y-p1.y;
  double deltaZ = p2.z-p1.z;
  double lineLength = sqrt(deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ);

  newLineSPoint << p1.x, p1.y, p1.z;
  newLineEPoint << p2.x, p2.y, p2.z;

  newLineVec << deltaX/lineLength, deltaY/lineLength, deltaZ/lineLength;
  worldLine3d newline(newLineSPoint, newLineEPoint,newLineVec,name);
  worldLines.push_back(newline);
}


/*void UFuncCamLocalize::projectToImage(Matrix<double,3,1>linePoint, Matrix<double,3,1> lineVec, matrix *pose, matrix *poseCov, LEL_ARLine &projLine, matrix *lineCov, matrix *delH_delP,matrix *delH_delPTrans) {
	double posex = mget(pose,0,0);
	double posey = mget(pose,1,0);
	double poseTh = mget(pose,2,0);

	double cameraTilt = varCameraTilt->getValued();
	double cameraX = varCameraX->getValued();
	double cameraY = varCameraY->getValued();
	double cameraZ = varCameraZ->getValued();

	double cx = varCx->getValued();
	double cy = varCy->getValued();

	double fp = varFp->getValued();

	Matrix<double,3,3,true> KCam;KCam= fp, 0 , cx,
							  0, fp, cy,
							  0,  0, 1.0;

	Matrix<double,3,3> R; R = -sin(poseTh),                  -cos(poseTh),                   0,
						 sin(cameraTilt)*cos(poseTh),   -sin(cameraTilt)*sin(poseTh),   -cos(cameraTilt),
						 cos(cameraTilt)*cos(poseTh),   -cos(cameraTilt)*sin(poseTh),   sin(cameraTilt);

	Matrix<double,3,1> cameraPosition; cameraPosition = posex+cos(poseTh)*cameraX-sin(poseTh)*cameraY,
												   posey+sin(poseTh)*cameraX+cos(poseTh)*cameraY,
												   cameraZ;

	Matrix<double,3,1> cameraPoint = R*(linePoint - cameraPosition);
	Matrix<double,3,1> imagePoint = KCam*cameraPoint;
	Matrix<double,3,1> imageVec = KCam * R * lineVec;

	double iPx = imagePoint(0,0), iPy = imagePoint(1,0), iPz = imagePoint(2,0);
	double iVx = imageVec(0,0)  , iVy = imageVec(1,0)  , iVz = imageVec(2,0);

	double iDx = iVx*iPz-iVz*iPx;
	double iDy = iVy*iPz-iVz*iPy;

	double alpha_ = atan2(iDy,iDx);
	double alpha = alpha_+M_PI/2;

	//double l = sqrt(iDx*iDx+iDy*iDy);

	double r = cos(alpha)*iPx+sin(alpha)*iPy;//(iVx*iPy-iVy*iPx)/l;

	projLine.alpha = alpha;
	projLine.r = r;

	// dH_dPose = dLine_diP*diP_dPose + dLine_diV*diV_dPose
	// dLine_diP
	// dLine_diV
	// diP_dPose = diP_dRPw*dRPw_dPose
	// diV_dPose = diV_dRVw*dRVw_dPose
	// diP_dRPw = K
	// diV_dRVw = K
	// dRVw_dPose
	// dRPw_dPose
	double PWx = linePoint(0,0);	double PWy = linePoint(1,0);

	mput(dRPw_dPose,0,0,sin(poseTh));     mput(dRPw_dPose,1,0,-sin(cameraTilt)*cos(poseTh));      mput(dRPw_dPose,2,0,-cos(cameraTilt)*cos(poseTh));

	mput(dRPw_dPose,0,1,cos(poseTh));     mput(dRPw_dPose,1,1, sin(cameraTilt)*sin(poseTh));      mput(dRPw_dPose,2,1, cos(cameraTilt)*sin(poseTh));

	mput(dRPw_dPose,0,2,-4.0*cos(poseTh)*sin(poseTh)*cameraY+sin(poseTh)*PWy-sin(poseTh)*posey-cos(poseTh)*PWx+cos(poseTh)*posex-2.0*cameraX+4.0*pow(cos(poseTh),2.0)*cameraX);
	mput(dRPw_dPose,1,2,sin(cameraTilt)*(-sin(poseTh)*PWx+sin(poseTh)*posex+4.0*sin(poseTh)*cos(poseTh)*cameraX-cos(poseTh)*PWy+cos(poseTh)*posey-2.0*cameraY+4.0*pow(cos(poseTh),2.0)*cameraY));
	mput(dRPw_dPose,2,2,cos(cameraTilt)*(-sin(poseTh)*PWx+sin(poseTh)*posex+4.0*sin(poseTh)*cos(poseTh)*cameraX-cos(poseTh)*PWy+cos(poseTh)*posey-2.0*cameraY+4.0*pow(cos(poseTh),2.0)*cameraY));

	double VWx = lineVec(0,0);	double VWy = lineVec(1,0);

	double dRVw_dPoseArr[] = {0, 0, -cos(poseTh)*VWx+sin(poseTh)*VWy,
			0, 0, -sin(cameraTilt)*(sin(poseTh)*VWx+cos(poseTh)*VWy),
			0, 0, -cos(cameraTilt)*(sin(poseTh)*VWx+cos(poseTh)*VWy)};

	array2mat(dRVw_dPose,dRVw_dPoseArr,9);

	double auxVar1= sqrt(pow(iVx*iVx*iPz*iPz-2.0*iVx*iPz*iVz*iPx+iVz*iVz*iPx*iPx+iVy*iVy*iPz*iPz-2.0*iVy*iPz*iVz*iPy+iVz*iVz*iPy*iPy,3.0));

	mput(dLine_diP,0,0, -iVz*(-iVy*iPz+iVz*iPy)/(iVx*iVx*iPz*iPz-2.0*iVx*iPz*iVz*iPx+iVz*iVz*iPx*iPx+iVy*iVy*iPz*iPz-2.0*iVy*iPz*iVz*iPy+iVz*iVz*iPy*iPy));
	mput(dLine_diP,1,0, (-iVy*iPz+iVz*iPy)*(-iPy*iVz*iVy-iPx*iVz*iVx+iVy*iVy*iPz+iVx*iVx*iPz)/auxVar1);
	mput(dLine_diP,0,1, -iVz*(iVx*iPz-iVz*iPx)/(iVx*iVx*iPz*iPz-2.0*iVx*iPz*iVz*iPx+iVz*iVz*iPx*iPx+iVy*iVy*iPz*iPz-2.0*iVy*iPz*iVz*iPy+iVz*iVz*iPy*iPy));
	mput(dLine_diP,1,1, (iVx*iPz-iVz*iPx)*(-iPy*iVz*iVy-iPx*iVz*iVx+iVy*iVy*iPz+iVx*iVx*iPz)/auxVar1);
	mput(dLine_diP,0,2, iVz*(iVx*iPy-iVy*iPx)/(iVx*iVx*iPz*iPz-2.0*iVx*iPz*iVz*iPx+iVz*iVz*iPx*iPx+iVy*iVy*iPz*iPz-2.0*iVy*iPz*iVz*iPy+iVz*iVz*iPy*iPy));
	mput(dLine_diP,1,2, -(iVx*iPy-iVy*iPx)*(-iPy*iVz*iVy-iPx*iVz*iVx+iVy*iVy*iPz+iVx*iVx*iPz)/auxVar1);


	mput(dLine_diV,0,0, iPz*(-iVy*iPz+iVz*iPy)/(iVx*iVx*iPz*iPz-2.0*iVx*iPz*iVz*iPx+iVz*iVz*iPx*iPx+iVy*iVy*iPz*iPz-2.0*iVy*iPz*iVz*iPy+iVz*iVz*iPy*iPy));
	mput(dLine_diV,1,0, -(-iVy*iPz+iVz*iPy)*(-iVz*iPx*iPx-iVz*iPy*iPy+iVy*iPz*iPy+iVx*iPz*iPx)/auxVar1);
	mput(dLine_diV,0,1, (iVx*iPz-iVz*iPx)*iPz/(iVx*iVx*iPz*iPz-2.0*iVx*iPz*iVz*iPx+iVz*iVz*iPx*iPx+iVy*iVy*iPz*iPz-2.0*iVy*iPz*iVz*iPy+iVz*iVz*iPy*iPy));
	mput(dLine_diV,1,1, -(iVx*iPz-iVz*iPx)*(-iVz*iPx*iPx-iVz*iPy*iPy+iVy*iPz*iPy+iVx*iPz*iPx)/auxVar1);
	mput(dLine_diV,0,2, -iPz*(iVx*iPy-iVy*iPx)/(iVx*iVx*iPz*iPz-2.0*iVx*iPz*iVz*iPx+iVz*iVz*iPx*iPx+iVy*iVy*iPz*iPz-2.0*iVy*iPz*iVz*iPy+iVz*iVz*iPy*iPy));
	mput(dLine_diV,1,2, (iVx*iPy-iVy*iPx)*(-iVz*iPx*iPx-iVz*iPy*iPy+iVy*iPz*iPy+iVx*iPz*iPx)/auxVar1);

	mmul(diP_dPose,KCam,dRPw_dPose);
	mmul(diV_dPose,KCam,dRVw_dPose);

	mmul(dH_dPose1,dLine_diP,diP_dPose);
	mmul(dH_dPose2,dLine_diV,diV_dPose);

	madd(delH_delP,dH_dPose1, dH_dPose2);
	mtrans(delH_delPTrans,delH_delP);

	mmul(cov1,delH_delP,poseCov);
	mmul(lineCov,cov1,delH_delPTrans);


//	if(worldLine.limited) {
//		projLine.limited = true;
//		double lsrX = posex+lasPose.x*cos(poseTh)-lasPose.y*sin(poseTh);
//		double lsrY = posey+lasPose.x*sin(poseTh)+lasPose.y*cos(poseTh);
//		double ll = worldLine.positionAlongLine(lsrX,lsrY);
//		projLine.lb = worldLine.lb-ll;
//		projLine.le = worldLine.le-ll;
//
//	}
//
//	double delH_delPArr[] = {0, 0, -1,
//							 -cos(worldLine.alpha), -sin(worldLine.alpha), -lasPose.x*sin(alpha)+lasPose.y*cos(alpha)};
//	array2mat(delH_delP,delH_delPArr,6);
//	mtrans(delH_delPTrans,delH_delP);
//
//	mmul(cov1,delH_delP,poseCov);
//	mmul(lineCov,cov1,delH_delPTrans);
}*/


