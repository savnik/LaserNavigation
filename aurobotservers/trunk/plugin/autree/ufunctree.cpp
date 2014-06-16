/***************************************************************************
 *   Copyright (C) 2010 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>
#include <urob4/uimagepool.h>
#include <vector>

#include "/usr/include/opencv/cv.h"
#include "/usr/include/opencv/highgui.h"
#include <iostream>

#include "ufunctree.h"


#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncTree' with your classname, as used in the headerfile */
  return new UFuncTree();
}

#endif

UFuncTree::~UFuncTree()
{
  if (mx1 != NULL)
  {
    cvReleaseStereoBMState(&BMState);
    cvReleaseMat( &mx1 );
    cvReleaseMat( &my1 );
    cvReleaseMat( &mx2 );
    cvReleaseMat( &my2 );
    cvReleaseMat( &img1rd );
    cvReleaseMat( &img2rd );
    cvReleaseImage(&img1);
    cvReleaseImage(&img2);
  }
}

///////////////////////////////////////////////////

bool UFuncTree::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 500;
  char reply[MRL];
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("tree");
    sendText("--- TREE is an implementation of openCV to detect trees\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    sendText(            "do                  Find trees\n");
    sendText(            "init                Init should be called if calibration values are changed (not needed at start)\n");
    sendText(            "silent              do not send a reply to this command\n");
    sendText("help       This message\n");
    sendText("----\n");
    sendText("see also: stereoPush for event handling of 3d cloud\n");
    sendText("see also: 'var stereo' for stereo generation parameters\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    bool doLog = false;
    bool aLog = msg->tag.getAttBool("log", &doLog, true);
    bool anUpdate = msg->tag.getAttBool("do", NULL, false);
    bool silent = msg->tag.getAttBool("silent", NULL);
    // implement command
    if (aLog)
    { // open or close the log
      aLog = logf.openLog(doLog);
      if (doLog)
        snprintf(reply, MRL, "Opened logfile %s (%s)", logf.getLogFileName(), bool2str(aLog));
      else
        snprintf(reply, MRL, "closed logfile %s", logf.getLogFileName());
      // send an information tag back to client
      if (not silent)
        sendInfo(reply);
    }
    if (anUpdate)
    { // default update call
      bool isOK = false;
      //if (varImagesLR != NULL)
      isOK = processImages();
      // format reply as valid XML tag
      if (not silent)
      {
        if (isOK)
        { // OK result
          varUpdateCnt->add(1.0);
          snprintf(reply, MRL, "<tree isOK=\"%s\" cnt=\"%d\" tod=\"%.3f\"/>\n",
                bool2str(isOK), varUpdateCnt->getInt(0), varTime->getDouble());
          sendMsg(reply);
        }
        else
        { // failed reply
          snprintf(reply, MRL, "Trees not find - %s", whyString);
          sendWarning(reply);
        }
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncTree::createResources()
{
  // Create global variables - owned by this plug-in.
  // Returns a pointer to the the variable for easy access.
  varUpdateCnt = addVar("updCnt", 0.0, "d", "(r) Number of updates");
  varTime = addVar("time", 0.0, "t", "(r) Time at last update");
  varImageTree = addVarA("imageDisp", "18", "d", "(rw) Image pool number for camera image");
}


/**
 * OpenCV equivalent of Matlab's bwareaopen.
 * image must be 8 bits, 1 channel, black and white (objects)
 * with values 0 and 255 respectively
 */

int bwareaopen(IplImage* image, int size)
{
  CvMemStorage *storage;
  CvSeq *contour;
  IplImage *input;
  double area;
  int count=0;
  
  if (image == NULL || size == 0)
    return 0;

  input = cvCloneImage(image);
  storage = cvCreateMemStorage(0);

  cvFindContours(input, storage, &contour, sizeof (CvContour),
                 CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

  while(contour)
  {
    area = cvContourArea(contour, CV_WHOLE_SEQ, 1);

    if (-size <= area && area <= 0)
    {
      // removes white dots
      cvDrawContours(image, contour, CV_RGB(0,0,0), CV_RGB(0,0,0),
                     -1, CV_FILLED, 8, cvPoint(0, 0));

      count=count+1;
    }
    else if (0 < area && area <= size)
    {
      // fills in black holes
      cvDrawContours(image, contour, CV_RGB(0xff,0xff,0xff),
                     CV_RGB(0xff,0xff,0xff), -1, CV_FILLED, 8, cvPoint(0,0));
    }
    contour = contour->h_next;
  }

  cvReleaseMemStorage(&storage);
  cvReleaseImage(&input);
  
  return count;
}


// Create a HSV image from the RGB image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated HSV image.
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB)
{
	float fR, fG, fB;
	float fH, fS, fV;
	const float FLOAT_TO_BYTE = 255.0f;
	const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;
	// Create a blank HSV image
	IplImage *imageHSV = cvCreateImage(cvGetSize(imageRGB), 8, 3);
	if (!imageHSV || imageRGB->depth != 8 || imageRGB->nChannels != 3) {
		//printf("ERROR in convertImageRGBtoHSV()! Bad input image.\n");
		exit(1);
	}

	int h = imageRGB->height;		// Pixel height.
	int w = imageRGB->width;		// Pixel width.
	int rowSizeRGB = imageRGB->widthStep;	// Size of row in bytes, including extra padding.
	char *imRGB = imageRGB->imageData;	// Pointer to the start of the image pixels.
	int rowSizeHSV = imageHSV->widthStep;	// Size of row in bytes, including extra padding.
	char *imHSV = imageHSV->imageData;	// Pointer to the start of the image pixels.
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			// Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
			uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
			int bB = *(uchar*)(pRGB+0);	// Blue component
			int bG = *(uchar*)(pRGB+1);	// Green component
			int bR = *(uchar*)(pRGB+2);	// Red component

			// Convert from 8-bit integers to floats.
			fR = bR * BYTE_TO_FLOAT;
			fG = bG * BYTE_TO_FLOAT;
			fB = bB * BYTE_TO_FLOAT;

			// Convert from RGB to HSV, using float ranges 0.0 to 1.0.
			float fDelta;
			float fMin, fMax;
			int iMax;
			// Get the min and max, but use integer comparisons for slight speedup.
			if (bB < bG) {
				if (bB < bR) {
					fMin = fB;
					if (bR > bG) {
						iMax = bR;
						fMax = fR;
					}
					else {
						iMax = bG;
						fMax = fG;
					}
				}
				else {
					fMin = fR;
					fMax = fG;
					iMax = bG;
				}
			}
			else {
				if (bG < bR) {
					fMin = fG;
					if (bB > bR) {
						fMax = fB;
						iMax = bB;
					}
					else {
						fMax = fR;
						iMax = bR;
					}
				}
				else {
					fMin = fR;
					fMax = fB;
					iMax = bB;
				}
			}
			fDelta = fMax - fMin;
			fV = fMax;				// Value (Brightness).
			if (iMax != 0) {			// Make sure its not pure black.
				fS = fDelta / fMax;		// Saturation.
				float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
				if (iMax == bR) {		// between yellow and magenta.
					fH = (fG - fB) * ANGLE_TO_UNIT;
				}
				else if (iMax == bG) {		// between cyan and yellow.
					fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
				}
				else {				// between magenta and cyan.
					fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
				}
				// Wrap outlier Hues around the circle.
				if (fH < 0.0f)
					fH += 1.0f;
				if (fH >= 1.0f)
					fH -= 1.0f;
			}
			else {
				// color is pure Black.
				fS = 0;
				fH = 0;	// undefined hue
			}

			// Convert from floats to 8-bit integers.
			int bH = (int)(0.5f + fH * 255.0f);
			int bS = (int)(0.5f + fS * 255.0f);
			int bV = (int)(0.5f + fV * 255.0f);

			// Clip the values to make sure it fits within the 8bits.
			if (bH > 255)
				bH = 255;
			if (bH < 0)
				bH = 0;
			if (bS > 255)
				bS = 255;
			if (bS < 0)
				bS = 0;
			if (bV > 255)
				bV = 255;
			if (bV < 0)
				bV = 0;

			// Set the HSV pixel components.
			uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
			*(pHSV+0) = bH;		// H component
			*(pHSV+1) = bS;		// S component
			*(pHSV+2) = bV;		// V component
		}
	}
	return imageHSV;
}


////////////////////////////////////////////////////////////////
bool UFuncTree::processImages()
{
  UImagePool * imgPool = (UImagePool*) getStaticResource("imgPool",false);
  UImage * img = imgPool->getImage(varImageTree->getInt(), true);
  IplImage *imghsv = cvCreateImage(cvSize((int)img->width(),(int)img->height()),(int)img->getDepth(),(int)img->getChannels());
  IplImage *img_gray = cvCreateImage(cvSize((int)img->width(),(int)img->height()),IPL_DEPTH_8U,1);//img->nChannels);
  IplImage *Br=cvCreateImage(cvGetSize(img_gray),IPL_DEPTH_8U,1);
  IplImage *Brneg=cvCreateImage(cvGetSize(img_gray),IPL_DEPTH_8U,1);
  
  cvCvtColor(img->getIplImage(),img_gray,CV_RGB2GRAY);
  cvThreshold(img_gray,Br,128,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
  // convert to HSV
  imghsv=convertImageRGBtoHSV(img->getIplImage());
 
  int w,h,i;
  CvScalar color;
  // create image with only points between the two threshold defined
  for (h=0; h<imghsv->height; h++)
  {
    for (w=0; w<imghsv->width;w++)
    {
	color = cvGet2D(imghsv, h, w);
	i=h*imghsv->width+w;
	Br->imageData[i]=255*(color.val[0]/255<0.16 and color.val[0]/255>0.05);
	Brneg->imageData[i]=~Br->imageData[i];	 
    }
  }
  // area removing
  bwareaopen(Br,100);
  
  // Create Memory for contour-search
  CvMemStorage* B = cvCreateMemStorage(0);
  CvMemStorage* stor = cvCreateMemStorage(0);
  CvMemStorage* appstor = cvCreateMemStorage(0);
  //pointer to the first contour
  CvSeq* cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
  CvSeq* contnew = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
  CvSeq* appcont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
  //findContours
  cvFindContours(Br,B,&cont,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE, cvPoint(0,0));
  //cout << "number of objects: " << cont->elem_size << "total: " << cont->total << endl;
  IplImage *dst=cvCreateImage(cvGetSize(Br),IPL_DEPTH_8U,1);
  IplImage *appdst=cvCreateImage(cvGetSize(Br),IPL_DEPTH_8U,1);
  IplImage *original=cvCreateImage(cvGetSize(Br),IPL_DEPTH_8U,1);
  float xsum = 0.0;
  float ysum = 0.0;
  float area = 0.0;
  float centMassX;
  float centMassY;
    
  int max_x, min_x, max_y, min_y;
  int hor, ver;
  int tree=0;
  // loop over all contours
  for(;cont;cont = cont->h_next)
  {
	// size of pointArray and polygon
	int point_cnt = cont->total;
	//cout << "point_cnt: " << point_cnt;

	// no small contours
	if (point_cnt<150){
	    continue;
	}
		
	// Allocate memory for contour point set.
	CvPoint* PointArray = (CvPoint*)malloc( point_cnt*sizeof(CvPoint) );
		
	// Get contour point set.
	cvCvtSeqToArray(cont, PointArray, CV_WHOLE_SEQ);
	max_x=0;max_y=0;
	min_x=img->width();min_y=img->height();
	for (int i=0; i<point_cnt; i++){ 
		if (PointArray[i].x>max_x){
		    max_x=PointArray[i].x;
		}
		if (PointArray[i].x<min_x){
		    min_x=PointArray[i].x;
		}
		if (PointArray[i].y>max_y){
		    max_y=PointArray[i].y;
		}
		if (PointArray[i].y<min_y){
		    min_y=PointArray[i].y;
		}
			
		int h_value = int(i*3.0/point_cnt*0)%100;

		cvLine(original, PointArray[i%point_cnt], PointArray[(i+1)%point_cnt],cvScalar(h_value, 255, 255), 4 );
	}
	//cout << " max_x :" << max_x << " min_x :" << min_x << " max_y :" << max_y << " min_y :" << min_y << endl;
	hor = max_x-min_x; ver = max_y-min_y;
	if (ver>hor){
	    contnew=cont;
	    CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
	    cvDrawContours( dst, contnew, color, color, -1, 1, 8 );
	    appcont=cvApproxPoly(contnew,sizeof(CvContour),appstor,CV_POLY_APPROX_DP,3,1);
	    cvDrawContours( appdst, appcont, color, color, -1, 1, 8 );
	    xsum = 0.0;
	    ysum = 0.0;
	    area = 0.0;
	    CvPoint* PointArrayCent = (CvPoint*)malloc( appcont->total*sizeof(CvPoint) );
	    for(int i = 0; i < appcont->total - 1; i++) {
		cvCvtSeqToArray(appcont, PointArrayCent, CV_WHOLE_SEQ);
		xsum +=PointArrayCent[i].x;
		ysum +=PointArrayCent[i].y;
		area++;
	    }
	    tree=tree+1;  
	    centMassX= xsum/area;
	    centMassY= ysum/area;
	    //center of mass representation
	    cvLine(appdst, cvPoint(centMassX,centMassY-10), cvPoint(centMassX,centMassY+10), cvScalar(100,100,100), 1);
	    cvLine(appdst, cvPoint(centMassX-10,centMassY), cvPoint(centMassX+10,centMassY), cvScalar(100,100,100), 1);
	}
  }
  
  printf("Found trees in image : %d",tree);
  
  cvReleaseImage(&imghsv);
  cvReleaseImage(&Br);
  cvReleaseImage(&dst);
  cvReleaseImage(&appdst);
  cvReleaseImage(&img_gray);
    
  return true;
  
}   

void UFuncTree::callGotNewDataWithObject()
{
  UDataBase * data = &cloud3d;
  gotNewData(data);
}


