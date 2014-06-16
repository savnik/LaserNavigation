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
#ifndef UIMAGEANA_H
#define UIMAGEANA_H

#include <ugen4/uimage.h>
#include <ugen4/uimage2.h>
#include <urob4/uimagepool.h>

/**
From openCV coltour function */
#define MAX_POLYGON_POINTS 1000
/**
For homemade outline function */
//#define MAX_OUTLINE_POINTS 1400
/**
Number  of possible ouline pixels */
#define MAX_POLYGON_OUTLINE_POINTS 10000

/**
A factor that convert a intensity difference (r+g+b) to a red cromaticity
variation to reduce effect of shadows.
rc = rc + (1.5*256 - (r+g+b)) * SHADOW_COMPENSATION_FACTOR */
//#define SHADOW_COMPENSATION_FACTOR 0.066
#define SHADOW_COMPENSATION_FACTOR 0.06

/**
Class to store information on area outline in an image */
class UPixelLinked
{
public:
  /**
  Constructor */
  UPixelLinked();
  //~UPixelLinked();
  /**
  Set ref position from a previous and a directions.
  height and width is needed to test for leagal values
  Returns true if new position is valid */
  bool getNext(UPixelLinked * from, int direction,
               int height, int width);
  /**
  Assignment operator */
  UPixelLinked operator= (UPixelLinked v);

public:
  /**
  Pixel row */
  int r;
  /**
  Pixel col */
  int c;
  /**
  Direction to next
  0 is positive x direction.
  1 is next direction clockwise (south). */
  int nx;
  /**
  Previous direction */
  int px;
};


/**
Class for image analysis.
Initially color statistics only.

@author Christian Andersen
*/
class UImageAna{
public:
  /**
  Constructor */
  UImageAna();
  /**
  Destructor */
  ~UImageAna();
  /**
  Set storage pointer */
  inline void setStorage(CvMemStorage * mem)
     { storage = mem;};
  /**
  Set image */
  inline void setImage(UImage * toImg) { img = toImg;};
  /**
  Get image name - static for test purpose only */
  const char * getImageName2(int i);
  /**
  Get image name from a 'ls *.bmp >imgListLog' list in file 'list' in
  subdirectory 'subdir' in the imagePath.
  The n'th filename in the list is extracted
  and stored in 'buff'. The buffer 'buffer' has space for
  buffCnt characters. If no name is found, then
  the returned string is empty, and NULL is returned.
  else a pointer to the filename is returned.
  The filename must be a *.bmp file
  and the '.bmp' is removed from the filename.
  There is no path (imagePath nor subdir) in the filename. */
  char * getImageName(
                  const char * list, const char * subdir,
                  int n,
                  char * buff, int buffLng);
  /**
  Find road as area that matches this start area.
  Paint result in result image if in debug mode. */
  bool findRoad(unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UImage * resultImg, bool debug,
                int imgNum, char * imgN);
  /**
  Find road as area that matches this start area.
  Paint result in result image if in debug mode. */
  bool findRoadCroma(unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UImage * resultImg, bool debug,
                int imgNum, char * imgN,
                int pct);
  /**
  Find road by filtering alone */
  bool findRoadFill(unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UImage * resi, bool debug,
                int camDev, char * imgName,
                int pct, bool lookingLeft);
  /**
  Find road using new outline search algorithem */
  bool findRoadPoly(
                bool debug, int camDev,
                char * imgN, double ballance, double limit,
                double lookingLeft, int x1, int y1, int x2, int y2,
                bool colorCorrect);
  /**
  Get estimated floor polygon */
  inline CvPoint * getFloorPolygon() { return floorArea; } ;
  /**
  Get number of vertexes in floor polygon .*/
  inline int getFloorPolygonCnt() { return floorAreaCnt; };
  /**
  Get pointer to number of vertexes in floor polygon .*/
  inline int * getFloorPolygonCntP() { return &floorAreaCnt; };
  /**
  Get boolean flag array to indicate that the vertex is part of an obstacle. */
  inline bool * getObstacleFlags() { return obst; };
  /**
  Get the cromaticity SD from the seed area. */
  inline double getCromaTraceSD() { return sqrt(cromaTrace); };
  /**
  Test image for illumination sanity
  Returns true if statistics seems OK */
  bool imageUsable(UImage * img);
  /**
  Find contour of image 'img' from a seed point
  by traversing from seedpoint clockwise until seedpoint
  is again reached.
  NB! must start at an bottom or right edge, as first test is
  down and there must be no way to loop back to start point clockwise.
  Finds an outline of area, but discards islands, that is only connected to
  "mainland" with one pixel.
  Bufer size should be at least 4000 points for a 320x240 image.
  Returns true if images are valid */
  bool imageContour2(UImage * img, UImage * imgC,
                             unsigned int row, unsigned int col,
                             UPixelLinked * outlineBuffer,
                             int * outlineBufferCnt,
                             double diff, double diffa);
  /**
  Same as above, but using Mahalonobis distance */
  bool imageContourMaha(UImage * img, UImage * imgC,
                        unsigned int row, unsigned int col,
                        UPixelLinked * outlineBuffer,
                        int * outlineBufferCnt,
                        UMatrix4 * averageSeed, UMatrix4 * covarianceInv,
                        double diff, double diffa);
  /**
  Same as above, but using Mahalonobis cromaticity distance
  Ballance is range [0..1] - default = 0.5.
  limit range 2.0 (no road) to 0.0 (all road) .*/
  bool imageContourMahaCroma(UImage * img, UImage * imgC,
                        unsigned int row, unsigned int col,
                        int x1, int y1, int x2, int y2, int maxH,
                        UPixelLinked * outlineBuffer,
                        int * outlineBufferCnt,
                        UMatrix4 * averageCroma, UMatrix4 * covarCromaInv,
                        UMatrix4 * averageRgb, UMatrix4 * covarRgbInv,
                        double ballance, double limit);
  /*
  bool imageContour(UImage * img, UImage * imgC,
                    unsigned int row, unsigned int col,
                    UPixelLinked * outlineBuffer,
                    int * outlineBufferCnt,
                    int diff, int diffa);
  */

  /**
  Reduce vertex count using a Douglas-Peuker reduction method.
  Method from:
  http://geometryalgorithms.com/Archive/algorithm_0205 */
  bool polygonReduce(UPixelLinked * outlineBuffer,
                     int * outlineBufferCnt,
                     double tollerance,
                     int topLine);
  /**
  Remove closed appendix areas */
  bool polygonReduce2(
      UPixelLinked * outlineBuffer,  // source and result pixel list
      int * outlineBufferCnt,
      double tollerance, // at bottom
      int topLine);       // "zero" tollerance at this line
  /**
  Finds area of a polygon.
  The area is signed - positive if clockwise.
  Calculated as A = 0.5 * sum(a(i)) over all i in [0, n-1],
  where a(i) = x(i) * y(i-1) - x(i-1) * y(i), where the
  first element (i=0) uses the last element (n-1) i.s.f. (i-1).
  http://homepages.borland.com/efg2lab/Graphics/PolygonArea.htm. */
  double polygonArea(CvPoint * poly, int elements);
  /**
  Set image pool */
  inline void setImagePool(UImagePool * imagePool)
    { imgPool = imagePool; };

private:
  /**
  Clear openCv storeage area */
  //void clearStorage();
  /**
  Get average and covariance matrix for this part of
  image.
  Returns true if image is valid. */
  bool getAvgVar(UImage * img,
                unsigned int x, unsigned int y,
                unsigned int w, unsigned int h,
                UMatrix4 * mE, UMatrix4 * mC);
  /**
  Compare pixels for intensity.
  Returns positive if p1 > p2, and 0 if equal,
  and negative if p2 > p1. */
  int pixCmp(UPixel * p1, UPixel * p2);
  /**
  Order this number of pixels after intensity in all channels (try),
  Returns the center element.
  NB! no range checks */
  UPixel * pixSort(UPixel *** ps, int pixCnt);
  /**
  Filter image based on row number */
  bool distFilt(UImage * simg, UImage * dimg, // source and destination image
        unsigned int maxW, unsigned int minW, // width of filter in pixels
        unsigned int minR, unsigned int maxR);// min and max row to filter
  /**
  Median filter with variable mask width and maximum mask height.
  Mask height must be smaller than or equal to 5 (od number).
  Mask width must be smaller than or equal to 40. */
  bool distFiltH(UImage * simg, UImage * dimg, // source and destination image
              unsigned int maxW, unsigned int minW,           // width of filter in pixels
              unsigned int minR, unsigned int maxR,           // min and max row to filter
              unsigned int maxMaskHeight); // max 5
  /**
  Find contour of maksked area.
  and paint result in destination image. */
  int findContour(UImage * imgMask, UImage * dest,
                     CvPoint * flp, int flpMax,
                     bool * tooHigh, bool * tooSmall, bool lookingLeft);
  /**
  Find contour of area, using own routine, and reduction
  using Douglas-Peucker.
  LookingLeft decides seed-point 1.0 seed is full right, -1.0 seed is full left.
  Parameters are pct1 is dynamic edge size and pct2 is absolute from reference point.
  Resulting polygon is returned in floorArea with floorAreaCnt
  points. floorOutline buffer is used during calculation and
  holds the same information on return.
  too-small is set trus if outline is less that 100 pixels.
  Returns true if a proper area is found. */
  int findContourPoly(UImage * imgM, UImage * imgC,
                     bool * tooSmall,
                     double lookingLeft,
                     double pct1, double pct2);
  /**
  Find contour polygon using cromaticity */
  int findContourPolyCroma(UImage * imgM, UImage * imgC, UImage * imgCroma,
                           bool * tooSmall, double lookingLeft,
                           int x1, int y1, int x2, int y2, int maxH,
                           double ballance, double limit);
  /**
  Count edges in contour sequebnce */
  int countContourEdges(CvSeq * contours);
  /**
  Find square limits of found contour
  returns the point cont, and
  if limit pointers != NULL the left, right, top and
  bottol limits of the points in the sequence. */
  int countContourLimits(CvSeq * contours, int * left,
      int * right, int * top, int * bottom, bool lookingLeft);
  /**
  Find the most distant vertex from the line between these endpoints
  and recursively find all vertexes in between.
  Adds the index to the found verteces to the vertex list.
  This needs to be sorted at the end. */
  void findVertex(const UPixelLinked * end1,
                  const UPixelLinked * end2,
                  const int count,
                  const int startNum,
                  int ** vertexList,
                  int * vertexListCnt,
                  const int listLength,
                  const float toll,
                  const int topLine);
  /**
  Get road probability value for pixel at (r, c).
  If useShadow then a compensation for shadow is attempted. */
  double getRoadProbability(int r, int c,
                            UImage * img, UImage * imgC,
                            //UPixel * pixE, UPixel * pix1,
                            UMatrix4 * vECroma, 
                            UMatrix4 * averageCroma, UMatrix4 * covarCromaInv,
                            //UMatrix4 * averageRgb, UMatrix4 * covarRgbInv,
                            double ballance,   // floating criteria
                            double averageY, bool useShadow);  // absolute criteria
  /**
  Paint base image with road detection */
  bool paintRoadImage(int seedR, int seedC,
                      UImage * img, // source
                      UImage * imgC, // destination
                      UMatrix4 * vCroma, // average croma value
                      UMatrix4 * mCromaInv,// inverse covariance matrix
                      UMatrix4 * vRgb, // average RGB value
                      UMatrix4 * mRgbInv, // iinverse RGB covariance
                      double dyn, // edge probability factor 1
                      double abs // edge probability factor 2
                      );
        
protected:
  /**
  Find the distance weighted (sqr) distance from this pixel value (RGB)
  using the provided covariance matrix as weight (Mahalonibis)
  Average must be a 1x3 vector and covariance must be 3x3 vector. */
  double getMahalonobisDist(UPixel * pix, UMatrix4 * average, UMatrix4 * covarianceInv);
  /**
  Find the distance weighted (sqr) distance from this pixel value (CROMATICITY)
  using the provided covariance matrix as weight (Mahalonibis).
  Average must be a 1x2 vector and covariance must be 2x2 vector.
  If useShadow then the red value is shadow compensated. */
  double getMahalonobisDistCroma(UPixel * pix, UMatrix4 * average, UMatrix4 * covarianceInv,
                                 double averageY, bool useShadow);

private:
  /**
  Image to be analized */
  UImage * img;
  /**
  Storage used by openCV */
  CvMemStorage * storage;
  /**
  Array of CvPoint, that holds the the floor contour */
  CvPoint * floorArea;
  /**
  Number of points in contour */
  int floorAreaCnt;
  /**
  Array of points and directions in floor outline */
  UPixelLinked * floorOutline;
  /**
  Count of elements in outline */
  int floorOutlineCnt;
  /**
  List of obstacle vertex. If two connected vertwx are non obstacle
  the the line between are not a detected obstacle, if on vertex is an
  obstacle, then it mustbe assumed to be so. */
  bool * obst;
  /**
  Reference to image pool */
  UImagePool * imgPool;
  /**
  Seed area cromaticity standard deviation */
  double cromaTrace;
};

#endif
