/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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
#ifndef UCLIENTFUNCIMGGUI_H
#define UCLIENTFUNCIMGGUI_H

#include <urob4/uclientfuncimage.h>
#include <ugen4/ufuzzysplit.h>

/**
Max length of character strings used to describe a
openCV highgui window */
#define MAX_CV_WINDOW_NAME_SIZE MAX_IMG_NAME_SIZE
/**
Number of window titles */
#define MAX_WINDOWS 60

/**
Dispalyed window information */
class UHighGuiWindowHandle
{
public:
  /**
  Constructor */
  UHighGuiWindowHandle();
  /**
  Set image */
  bool setImage(UImage * toImg);
  /**
  Get pointer to image */
  inline UImage * getImage()
    { return img; };
  /**
  Show this image (and save a copy)
  shows image, if imgTime is different than existing image,
  of if force is true.
  Returns true is shown. */
  bool showImage(UImage * toImg, bool force);
  /**
  Create an empty image of max size.
  Returns a pointer to the created image. */
  UImage * makeImage(const char * name);

public:
  /**
  String that identifies the window to openCV highgui */
  char cvWndName[MAX_CV_WINDOW_NAME_SIZE];
private:
  /**
  Copy of image to display */
  UImage * img;
};

/**
Show images from image server

@author Christian Andersen
*/
class UClientFuncImgGui : public UClientFuncImage
{
public:
  enum ColorDisplayType {CDT_NONE, CDT_YUV, CDT_CROMA};
  /**
  Constructor */
  UClientFuncImgGui();
  /**
  Destructor */
  ~UClientFuncImgGui();
  /**
  time to update GUI */
  virtual void doTimeTick();
  /**
  Close all images and release resources */
  void clear();
 /**
  Set the image to be displayed for color analysis */
  void setUVSource(const char * captName,
                 ColorDisplayType type);
  /**
  Get handle to highgui image with this caption part.
  Default image is first, if noone else.
  Result may be NULL if no image defined. */
  UHighGuiWindowHandle * getImageSource(const char * capt);
  /**
  Save image with this string part in caption */
  bool saveImage(const char * capt, const char * noshow);
  /**
  Get pointer to one of the windows
  with this caption */
  UImage * getImage(const char * capt);
  /**
  Set flag, that determines if images are to be converted to max res */
  inline void setToMaxRes(bool value)
  { imgToMax = value; };
  /**
  Set line for amplitude analysis */
  inline void setFocalLine(int line)
  {  focalLine = line;};
  /**
  Set width of mask, where sample for
  fuzzy-classifier is taken.
  The other class is taken over the full image */
  void setFuzzyWidth(int value);
  /**
  Set number of iterations for fuzzy classifier
  0=seed area only */
  inline void setFuzzyIter(int value)
  { fuzzyIter = value; };
  /**
  Set number of classes to cklassify into.
  2 = 2x mouse, 3=2xmouse + background
  (all other values are illegal) */
  inline void setFuzzyClasses(int value)
  { fuzzyClasses = value; };
  /**
  Set number of classes to cklassify into.
  2 = 2x mouse, 3=2xmouse + background
  (all other values are illegal) */
  void setFuzzyLine(int row, int col);
  /**
  Set pointer to laser scanner path results 
  - may be used to paint in camera image */
  void showImage(UImage * img)
  { imgRefr = img; };
  
protected:
  /**
  Called when a new image in RGB format
  (openCv format) is available. */
  virtual void gotNewImage(UImage * img, int poolNum, USmlTag * tag);
  /**
  Show UV image from default source and
  default (global) threshold */
  void showUVImage();
  /**
  Paint color intensity line on top of image for
  'line' (2/3 width) */
  void showLine(UImage * imgs, UImage * imgd, int line);
  /**
  Show largest intensi-jump from one pixel to the next
  over the full image (every 3rd line) */
  void showFocusNumber(
              UImage * imgs, // source image
              UImage * imgd // destination image
              );
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
  int getPixelClass(UPixel yuv, int clustCnt);
  /**
  Convert this BGR pixel to a cromatisity only pixel */
  UPixel getCroma(UPixel * rgb, int sourceFormat);

private:
  /**
  Get the handle for this image name */
  UHighGuiWindowHandle * getWndName(char * name, bool mayCreate);

private:
  /**
  openCV variables to show window */
  UHighGuiWindowHandle * window[MAX_WINDOWS];
  /**
  Number of windows created */
  int windowCnt;
  /**
  Image to use for UV display */
  unsigned int imgSourceUV;
  /**
  Image that displays the UV colour values */
  UImage * imgUV;
  /**
  Copy all images to max resolution */
  bool imgToMax;
  /**
  Show focus line curve */
  int focalLine;
  /**
  Show fuzzy classifier */
  int fuzzyWidth;
  /**
  Number of fuzzy classifier classes */
  int fuzzyClasses;
  /**
  Number of fuzzy classifier iterations */
  int fuzzyIter;
  /**
  Fuzzy seed from fuzzyLine in image
  split at fuzzyCol position. */
  int fuzzyLine;
  /**
  Fuzzy seed from 'fuzzyLine' in image
  split at 'fuzzyCol' position. */
  int fuzzyCol;
  /**
  Fuzzy classifier */
  UFuzzySplit * fuzzy;
  /**
  Image to redisplay */
  UImage * imgRefr;
  /**
  Save images if match caption */
  bool saveOnMatch;
  /**
  Display image also if it was saved */
  bool saveOnMatchAndShow;
  /**
  Match string length */
  static const int MAX_MATCH_STR_LNG = 100;
  /**
  Caption match string */
  char saveOnMatchStr[MAX_MATCH_STR_LNG];
};

#endif
