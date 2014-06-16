/***************************************************************************
 *   Copyright (C) 2006 by Christian   *
 *   chrand@mail.dk   *
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
#ifndef URESDISP_H
#define URESDISP_H

//#include <SDL.h>
#include <pthread.h>

#include <urob4/uimgpush.h>
#include <urob4/uresvarpool.h>

//#define USE_HIGHGUI
#include "unavpaint.h"

class UResPoseHist;
class UImagePool;


/**
Dispalyed window information */
class UHighGuiWindowHandle
{
public:
  /**
  Constructor */
  UHighGuiWindowHandle();
  /**
  Constructor */
  ~UHighGuiWindowHandle();
  /**
  Set image */
  bool setImage(UImage * toImg, int imgPoolNum);
  /**
  Get pointer to image */
  inline UImage * getImage()
  { return img; };
  /**
  Get image pool number */
  inline int getImageNum()
  { return imgNum; };
  /**
  Show this image (and save a copy)
  shows image if the used flag is 0.
  if shown, the use flag is set to 1 (true);
  \param useHighGui if true, then image is actually shown using highgui - window,
  this is in conflict with gstreamer, so may be disabled.
  \Returns true if shown/updated (also set if highgui is not used). */
  bool showImage(bool useHighGui);
  /**
  Remove window with this name */
  void removeImage();
  /**
  Is it time to redisplay this image */
  bool isTime(UTime dueTime);

protected:
  /**
  Maximum length of openCV window name */
  static const int MAX_CV_WINDOW_NAME_SIZE = 60;
  /**
  String that identifies the window to openCV highgui */
  char cvWndName[MAX_CV_WINDOW_NAME_SIZE];
  /**
  image source to display */
  UImage * img;
  /**
  image pool number */
  int imgNum;

};

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

/**
Display of laserdata using the HIGHGUI library

	@author Christian <chrand@mail.dk>
*/
class UResDisp : public UResVarPool
{
public:
  /**
  Constructor */
  UResDisp()
  {
    setResID("disp", 200);
    UResDispInit();
    globalThresholdMin = 40;
    globalThresholdMax = 200;
    imgBuf = NULL;
  };
  /**
  Destructor */
  ~UResDisp();
  /**
   * Initialize class */
  void UResDispInit();
  /**
    Create basic polygon values */
  void createBaseVar();
  /**
    Fixed name of this resource type */
    static const char * getResID()
    { return "disp"; };
  /**
    Fixed varsion number for this resource type.
    Should follow release version, i.e. version 1.28 gives number 128.
    Should be incremented only when there is change to this class
    definition, i.e new or changed functions or variables. */
    static int getResVersion()
    { return 173; };
  /**
    Print status for this resource */
    virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual void snprint(const char * preString, char * buff, int buffCnt)
  { print(preString, buff, buffCnt); };
  /**
    Is this resource missing any base ressources */
    bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
    Set ressource as needed (probably not used by this resource) */
    bool setResource(UResBase * resource, bool remove);
    /**
    The varPool has methods, and a call to one of these are needed.
    Do the call now and return (a double sized) result in 'value' and
    return true if the method call is allowed. */
    virtual bool methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt);

  /**
  Repaint loop */
  void run();
  /**
  Start repaint loop */
  bool start();
  /**
  Stop repaint loop */
  void stop(bool andWait);
  /**
  Repaint images in the image pool */
  void doRepaintImages();
  /**
  Load this image into the image pool */
  bool loadImgToPool(const char * imgName, const int imgPoolNum);
  /**
  Get top-view painter */
  inline UNavPaint * getNavPaint()
  { return &navPaint; };
  /**
  Set navigation data flag to update image */
  inline void setNewDataNav()
  { newDataNav = true; };
  /**
  See if new images has emerged in image pool that is not on the display list */
  void test4NewImages();
  /**
   * Set UV image source */
  void setUVSource(int uvImg)
  { imgSourceUV = uvImg; };
  /**
   * Set UV image source */
  void setCROMASource(int cromaImg)
  { imgSourceCROMA = cromaImg; };
  /**
   * Set UV/CROMA analysis intensity minimum */
  void setIntensMin(int val)
  { globalThresholdMin = val; };
  /**
   * Set UV/CROMA analysis intensity maximum */
  void setIntensMax(int val)
  { globalThresholdMax = val; };
  /**
   * set automatic herenow flag */
  void setAutoHereNow(bool value)
  { varAutoHereNow->setValued(value); };
  /**
   * set automatic herenow flag */
  void setUvRedisplay(bool value);
  /**
  Try capture an image using openCV highGUI
  \param camIdx is the camera device number to use */
//  void capture(int camIdx);
  /**
  Set the paint bold flag
  \param bold true or false */
  void paintBold(bool bold);
  /**
  Set robot type.
  \param robname is the type name of the robot - smr, mmr or hako */
  void setRobot(const char * robname);
  /**
  Set number of range rings */
  void setRangeRingCnt(int value);
  /**
  Set scale (height of display in meters */
  void setScale(double value);
  /**
  Set position of robot on display - in meters from bottom position
  \param value is a string with pose values as a string, p.t.
  the first cvalue is used only. */
  void setRobotPose(char * value);
  /**
  Rescale based on Ctrl-mouse click or shift-ctrl click or ctrl - square */
  void mouseRescale(bool shiftKey);

  
protected:
  /**
  Display this image, and redisplay, when updated */
  void addImagePoolImg(int imgPoolNum);
  /**
  Remove this image, as it is about to be resized */
  void removeImagePoolImg(int imgPoolNum);
  /**
  Paint an UV image from the source image in the variable 'imgSourceUV' */
  void showUVImage(bool asCROMA);
  /**
  Classify mouse sample and the-rest
  into two clusters */
  bool doFuzzyClassify(UImage * imgsrc);
  /**
  Paint pixels within  'clust' into image
  in blue */
  void paintClustPixels(bool isUV,
                        UImage * imgsrc,
                        UImage * imgUV);
  /**
  Paint cluster covariance ellipse
  in UV image, if not UV, then cromatisity */
  void paintClustEllipse(bool isUV,
                         UMatrix * mV,
                         UMatrix * mQ,
                         UImage * img,
                         int clust);
  /**
  Get cluster number for this pixel.
  Must know the number of classes the
  pixels are divided into. */
  int getPixelClass(UPixel yuv, int clustCnt, bool isCROMA);
  /**
  Convert this BGR pixel to a cromatisity only pixel */
  UPixel getCroma(UPixel * rgb, int sourceFormat);

protected:
  /**
  Pointer to a pose history module with current robot position */
  UResPoseHist * poseHist;
  /**
  Pointer to a pose history module with current robot position in GPS (UTM) coordinates */
  UResPoseHist * poseUtm;
  /**
  Pointer to a pose history module with current robot position in map coordinates */
  UResPoseHist * poseMap;
  /**
  Pointer to a (root) var-pool with access to all sub-structures */
  UResVarPool * varRoot;
  /**
  Pointer to image pool resource */
  UImagePool * imgPool;
  /**
  Time of newest data update of image in image pool */
  UTime newDataImgPoolAt;
  /**
  New data received to image pool */
  bool newDataImgPool;
  /**
  Time of newest data update of data for laser plane display */
  UTime newDataLaserAt;
  /**
  New data received - laser data */
  bool newDataLaser;
  /**
  Time of newest data update of data for laser plane display */
  UTime newDataNavAt;
  /**
  New data received - navigation data */
  bool newDataNav;
  /**
  Index to variable with thread running value */
  UVariable * varRunning;
  /**
  main laser navigation image number in image pool */
  UVariable * varNavImg;
  /// should coordinate systems be alligned at robot always
  UVariable * varAutoHereNow;
  /// used by the UV image and cromaticity image to limit colour display
  UVariable * varUVCROMAminY;
  /// used by the UV image and cromaticity image to limit colour display
  UVariable * varUVCROMAmaxY;
  /// paint bold
  UVariable * varBold;
  /// number of range rings to paint (each 1m)
  UVariable * varRangeRings;
  /// should HIGHGUI be used to display images (conflict with gstream)
  UVariable * varUseHighGUI;
  /**
   * allowed number of highgui displayed images */
  static const int UPD_LIST_MAX_CNT = 100;
  /**
  Array of relations between openCV images and SDL draw surfaces */
  UHighGuiWindowHandle updList[UPD_LIST_MAX_CNT];
  /**
  Number of used entries in the image update list */
  int updListCnt;
  /**
  Image source for UV/CROMA image */
  int imgSourceUV;
  /**
  image reference for CROMATICITY analysis */
  int imgSourceCROMA;
  /// colour analysis intensity limits
  int  globalThresholdMin;
  /// colour analysis intensity limits
  int  globalThresholdMax;
  /**
  Display sync lock */
  ULock dispSync;
  /**
  Painter for navigation image */
  UNavPaint navPaint;

private:
  /**
  Stop flag for display thread */
  bool stopFlag;
  /**
  Thread handle for main loop. */
  pthread_t  thDisp;

private:
  /**
   * Image buffer if UV or croma image is not in an appropriate format */
  UImage * imgBuf;

};

#endif
