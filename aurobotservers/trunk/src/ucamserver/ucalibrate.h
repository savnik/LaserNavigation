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

#ifndef UCALIBRATE_H
#define UCALIBRATE_H

#include <ugen4/uimage.h>
#include <ucam4/ucammount.h>
#include <urob4/uimagepool.h>

#include "ucamparest.h"
#include "ubarcode.h"
//#include "ucamera.h"

/**
Size of calibration array */
#define MAX_CALIBRATION_COMPONENTS 1500
/**
Size of candidate list for ordering corner elements (8 true corners). */
#define MAX_BEST_ARRAY_SIZE 12
/**
Barcode square frame size (cells in frame) */
#define MAX_SIDE_LENGTH 12
/**
Number of cornermarks in a barcode frame.
or 100 point frames - as these are more demanding */
//#define MAX_FRAME_POSITIONS (MAX_SIDE_LENGTH * 4 - 4)
#define MAX_FRAME_POSITIONS 100
/**
Barcode code cells in each direction for one frame cell. */
#define MAX_CELL_FACTOR 4
/**
Code cells in each direction - including frame. */
#define MAX_CELL_COUNT   (MAX_CELL_FACTOR * MAX_SIDE_LENGTH)
/**
Saved set of calibration points from barcode frame. */
#define MAX_CALIBRATION_SETS_SAVED 20
/**
Sides of guidemarks must be parallel within a sertain limit.
When this lomit is decreased more perspective is allowed,
about 3.0 allows app. 60 degree angle, 2.0 allows app. 80 deg. */
#define NON_PARALLEL_FACTOR 2.0
/**
Relation between nearest neighbor one way and the nearest
neighbor the other way.
A factor if 1.8 do not allow much perspective, 3.2 allows
a fair bit. */
#define MAX_NEIGHBOR_DISTANCE 5.0
/**
Side size of grid to find missing squares in center of grid */
#define MAX_CALIB_GRID_SIZE 50

/**
Max number of bytes in a barcode (guidemark) code */
#define MAX_BARCODE_LENGTH 64
/**
Do not look for more than this number of
barcodes (GMKs) in one single image */
#define MAX_CODES_IN_ONE_IMAGE 16

/**
Max image size for guidemark use */
#define MAX_IMAGE_HEIGHT 1200
#define MAX_IMAGE_WIDTH 1920

/**
Image pool numbers to use */
#define UCAM_IMS_CALIB_RAW      21
#define UCAM_IMS_CALIB_FILTER   22
#define UCAM_IMS_CALIB_CORNER   23
#define UCAM_IMS_CALIB_GRID     24

//////////////////////////////////////////////////////////////

/**
Save this message in logfile if significance is less than logLevel.
Print also to console (debug feature) if verbose messages.
Timestamp is optional, if not used, then Now() is used in logfile.*/
extern void toLogGmk(const char * info, int significance, UTime * timestamp);



/**
Class for x,y statistics, of distance to calibration
components.
Sum and squared sum can be accumulated in x,y and vx, vy and n increased.
After ending the sum the statistics mean van be evaluated in x,y,
the variance in vx, vy and standard deviation in sx, sy. */
class UCalibXYStat
{
public:
  /**
  Position, offset or average (x,y) */
  float x, y;   // position, offset (or average)
  /**
  Variance */
  float vx, vy; // variance or std.deviation
  /**
  Number of samples in structure */
  int n;        // number of samples
  /**
  Flag to mark valid direction */
  bool flag;    // flag used to mark valid direction
  /**
  Last ID in a linear row */
  int endId;    // last ID in a linear row.
  /**
  Reference to the oother item */
  int id;  // reference to other item
  /**
  Distance to other item */
  float d; // distance to other id.
  /**
  Constructor, just resets values */
  UCalibXYStat();
  /**
  Clear just clear values */
  void clear();
  /**
  Add a new set of values.
  returns the number of samples, or -1 if error.
  Reports error if justSum is false. */
  int add(float x, float y);
  /**
  Convert to statistics - mean and variance.
  returns n if valid else -1 */
  int FromSumToStat(void);
};

//////////////////////////////////////////////////////////////

class UCalibrationMarkSet;

/**
Class to hold calibration points found in an image */
class UCalibrationComponent
{
public:
 /**
 pixel position from 0 to image width -1 */
 float x, y;
 /**
 number of points in centroid calculation (0 == invalid) */
 int w;
 /**
 row position in chess grid */
 int row, col;
 /**
 real position of this corner on chart in meter from
 bottom left. */
 float rx, ry;
 /**
 Source of this component - wich filter */
 int filter;
 /**
 Link and relation to neighbor in 8 directions.
 Directions are  (N=0,NE=1,E=2,SE=3,S=4,SW=5,W=6,NW=7) */
 UCalibXYStat NSEW[8];
 /**
 Clear component */
 void clear();
 /**
 Clear relations to neighbors only */
 void clearNSEW();
 /**
 Get block distance to another component.
 as the greater of either x or y distance.
 Returns the distance */
 float GetMaxBlockDist(UCalibrationComponent * other,
                           float * dx, float * dy);
 /**
 Get offset point, where neighbor in direction head
 most likely are found.
 returns position offset in dx and dy */
 void GetDefaultOffset(
           int head, /* heading (0..7) */ int MinSep, /* minimum */
           float * dx, float * dy);

  /**
  Get distance to another component. */
  float GetDist(UCalibrationComponent * other);
  /**
  Is the other component in the right direction compared to this one?
  returns true if so. */
  bool IsInRightQuadrant(
                      UCalibrationComponent * other,
                      int head);
  /**
  Set row and column number from the
  neighbor opposit head direction. e.g
  if heading is east then column is set to one more than c
  and row kept the same. */
  void SetRc(int r, int c, int head);
  /**
  Paint line to neighbors for this component */
  void PaintNeighbors(UImage * image,
                      UPixel pix,
                      bool halfWay,
                      UCalibrationComponent * ccHits,
                      float scale = 1.0);
};

/////////////////////////////////////////////////////////////

/**
Minor set of calibration components used during calibration point
isolation */
class UCalibBestArrayElement
{
public:
  /**
  ID of element */
  int id;
  /**
  Distance */
  float d;
  /** Distance in x only */
  float dx;
  /** Distance in y only */
  float dy;
  /** is valid */
  bool valid;
};

/**
Array of a few candidates for associating elements */
class UCalibBestArray
{
public:
  /**
  Best (nearest) candidates for further processing */
  UCalibBestArrayElement items[MAX_BEST_ARRAY_SIZE];
  /**
  Number of valid items in best array. */
  int count; // number of valid items
  /**
  Add a candidate in distance order. */
  void AddCandidate(int candidate, float d /* distance */,
                                   float dx, float dy);
  /**
  Limit candidates using distribution of
  nearest candidates. Cot the list if too far / too near. */
  void DoLimitCandidates(float MaxNeighborDistanceFactor);
};

//////////////////////////////////////////////////////////////

/* forward declaration */
class UCalibrationMark;



/**
Class to hold calibration points found in an image */
class UCalibrationComponents
{
public:
  /**
  Constructor */
  UCalibrationComponents();
  /**
  Clear any existing components */
  void clear();
  /**
  Save this message in logfile if significance is less than logLevel.
  Print also to console (debug feature) if verbose messages.
  Timestamp is optional, if not used, then Now() is used in logfile.*/
  inline void toLog(const char * info, int significance, UTime * timestamp = NULL)
  { toLogGmk(info, significance, timestamp); };
  /**
  Pack components, so that any non valid (w==0) ones are removed */
  int Pack();
  /**
  Enumereate pixels in an image with intensity \> 0.
  The method combines pixels using a connectivity of either
  'connectivity' = 4, 8, 12 or 20. Each separate component
  are numbered and added to the component array.
  returns true cim is an image. */
  bool EnumerateCalibrationDetections(UImage * cim,
                                int connectivity);
  /**
  Enumereate pixels in an image with intensity \> 0.
  Each of the 2x2 quadrants are treaded seperately, i.e. not combined
  with corner of opposite type.
  The method combines pixels using a connectivity of either
  'connectivity' = 4, 8, 12 or 20. Each separate component
  are numbered and added to the component array.
  returns true cim is an image. */
  bool EnumerateCalibrationDetections4(UImage * cim,
                                int connectivity);
  /**
  Paints components in hits[] array on this image using the
  base color in pix. Pix is in YUV format, and intensity are
  set from weight (hits[].w). <br>
  Returns 0. */
  int PaintHitsInImage(UImage * image, UPixel pix);
  /**
  Paint real chart positions in image where
  they should be if calibration was OK.
  Saves corner positions to in matlab format if matlabFilename != NULL.
  Return 0. */
  int PaintChartInImage(UImage * image,
                        UPixel pix,
                        UPosition * chartPos,
                        UCamPar * cm,
                        const char * matlabFilename);
  /**
  Paints components in grid[row][col] array on this image using the
  color in pix. Pix is in YUV format <br>
  Paint in lines connecting rows and cols when consecutive
  Returns 0. */
  int PaintGridInImage(UImage * image, UPixel pix);
  /**
  Test for loose components at the edge of others, and
  merge components if centers are within 'Dist' pixels.
  Returns number if valid components (\<= HitCount)  */
  int doMergeNearHits(float dist);
  /**
  In many cases some background elements looks like corners
  but most will be smaller or bigger than the desired ones,
  so remove all outside +/- 2 standard deviations (plus a bit)
  of all corners.
  Returns number of corners left. */
  int DoSizeLimit(void);
  /**
  This routine validates and orders the found cornerpoints
  assumed to be within the test following pattern
   1 2 3 4 5 6 7 8 9 (9 cols x 11 rows - 8 = 91 corners)
   x x x x x x x x x  1
   x x x x x x x x x  2
   x x x x x x x x x  3
   x x x x x     x x  4
   x x x x x     x x  5   Painted on A4 with
   x x x x x x x x x  6   2cm between corners
   x x     x x x x x  7   center pattern
   x x     x x x x x  8   used for
   x x x x o o x x x  9   edge detection
   x x x x o o x x x  10
   x x x x x x x x x  11
  Method
   - first find candidate neighbors in 8 directions
     this will fail at edges, where some directions will
     find a wrong candidate
   - Check for direction of neighbors and fix
   - Renumber remaining candidates in a row-column grid
     starting with a candidate with 8 neighbors, and trust primarily
     neighbors with 8 or 7 neighbors for continued numbering
     edge candidates will only be numbered from a 7 or 8 neighbor
     candidate. Unnumbered candidates will be dropped.
   - Check that numbering is consistent.
   - Find center pattern and renumber to correct row and column.
  Adds corner position oncalibration chart to data set (using 'stride')
  Returns number of corners.  */
//  int DoOrderHits(UImage * image, float stride);
  /**
  Find nearest neighbors for barcode detection.
  The neighbors must be nearer than 'maxDistance' (in pixels).
  If 'extraImages' is true, then neighbor lines gets drawn in
  'destination' image in top of a faded copy of 'source' image.
  Returns true if at least one neighbor is found.  */
  bool findClosestNeighbors(
               float maxDistance,
               bool extraImages,
               UImage * source,
               UImage * destination);
  /**
  Order hits from neighbors for a 2D barcode image detection */
  bool doOrderHits4(int size,  // barcode size to look for
             int * k0, int * k1, int * k2, // corners
             int * k0dk1, int * k0dk2, int * k1dB, int * k2dB,
             bool extraImages,
             UImage * img, int pixFactor); // directions
  /**
  Finds grid positions ordered from the 3 cornerpoints ht, ht1, ht2,
  and the 4 direction direction statistics htT0k1, htTok2, k1ToB, ht2ToB.
  The positions are saved as a 'UCalibrationMarkSet' with corresponding
  chart positions and pixel positions.
  If 'alignFrame' is true then the pixel positions are moved
  to be on a straight line with the other on the same frame side.
  Returns true if successful. */
  bool doMakeCodeGrid(
              int k0, int k1, int k2,  // three corners
              int k0dk1, int k0dk2, int k1dB, int k2dB, // direction to other
              int size,     // expected barcode frame side size
              int sideFactor,  // code cells for each frame side (2)
              float stride,  // block side size in meter
              UCalibrationMarkSet * markSet, // frame positions
              bool alignFrame, // rectify frame to align with straight lines
              bool saveToFile); // save grid-square to matlab file
  /**
  Find intensity value in each code grid cell.
  Source image is needed for average intensity along border.
  Paints result to 'destination' if 'extraImage' is true.
  Destination image is only needed if used.
  Returns true if successful. */
  bool doFindCodeCellIntensity(
                    int size, int sideFactor,
                    UImage * source,
                    int * borderIntensity,
                    bool extraImages,
                    UImage * destination, // for extra images
                    int codeNumber);      // if more than one code in image
  /**
  Extract barcode value to string.
  The size and side factor defines the barcode frame.
  The code is returned in the code[] array one (hex) number in
  each array element (up to 'maxCodeLng').
  The last 8 code values are converted to a long integer
  (assumed 4 bytes) to be used for fast value.
  Returns true if code is captured. <br>
  The index corner has index (m='indexM',n='indexN') in
  the mCode[m][n] array (with pixel position mCode[m][n].x,mCode[m][n].y) */
  bool doFindCodeValue(
              int size, int sideFactor,
              int code[], int * codeLng, int maxCodeLng,
              unsigned long * intCode,
              bool debug,
              int splitIntensity,
              int * indexM, // index corner
              int * indexN);
  /**
  Save grid coordinates to a file for verification of
  correct numbering ('txt' file). */
  int SaveCalibPointsSquare(char * filename);
  /**
  Save selected hit information to file */
  int SaveAllHits(char * filename);
  /**
  Find maxR and maxC as limits for grid positions. */
  int FindGridLimits();
  /**
  Save calibration coordinates in MATLAB format, as
  needed by camera calibratution routine:
  p = [chart x (left), y (up), z (0.0), image col (left), row (down), 0, 0, 1; ...]; <br>
  ... <br>
  chart x (left), y (up), z (0.0), image col (left), row (down), 0, 0, 1]; <br>
  filename should end on '.m', as a MATLAB file.
  Returns 0 if OK, else -1. */
  int SaveCalibCoordinates(char * filename);
  /**
  Find position in image using coordinates in found grid.
  Returns found positions in ix and iy, but not the corresponding
  position in chart coorfinates
  returns 0 if
  succesfull, and -1 if not.  */
  UCalibrationMark FindImagePosition(int sm, int sn);

private:
  /**
  Find candidate neighbors in 8 directions for one candidate. */
  void DoFindNSEW(int candidate);
  /**
  Find up to 8 closest neighbors to this point all within the distance limit 'limit'.
  Returns true if at least one neighbor is found */
  bool DoFindNSEW4(int candidate, float limit, FILE * fl = NULL);
  /**
  Find a list of close neighbors to this candidate and
  plase then in the best array. */
  void GetClosestItems(int candidate, UCalibBestArray * best,
                       float distanceLimitFactor,
                       FILE * fl = NULL);
  /**
  Find the best candidate in a given direction from
  this list of up to 12 candidates. */
  void DoFindNSEWHead(int candidate, int head, UCalibBestArray * best);
  /**
  Validate that candidate in each heading is in the right direction. */
  void DoValidateCandidates(int candidate, UCalibBestArray * best);
  /**
  An invalid neighbor candidate is found, so remove it and try
  to find a new. Used by DoValidateCandidates only. */
  void DoRemoveAndRetry(int candidate,  int head,
                       UCalibBestArray * best);
  /**
  Set row and column for all neighbors without one, assuming this
  candidate has a number. Continues numbering using breast first
  and setting the numbers using 'SetRowColFromNeighbor(n)' to ensure
  best consistency, when there is conflicts.
  Returns 0. */
  int SetRowColForNeighbors(int candidate);
  /**
  Give this item a row/col number based on neighbor oppinion */
  int SetRowColFromNeighbor(int candidate);
  /**
  Look for center cross by folding a cross mask over grid area.
  Returns number of valid positions in grid, or -1 if no
  center was found.
  The candidates are then marked with position in grid relative to
  center of chart, using GRID_STRIDE spacing between markers in meters. */
  int  DoFindCenterCross(float strideL, float strideW);
  /**
  Fold the center mask with 4 blocks missing over found grid and
  Return score */
  int CenterCrossScore66(int row, int col);
  /**
  Fold the center mask with 2 blocks missing over found grid and
  Return score */
  int CenterCrossScore55(int row, int col, bool inverse);
  /**
  Find length and number of hops for this corner point.
  Explores direction to all neighbors, and stores distance and number of hops
  in the NSEW part of the structure. */
  int DoFindRowLength4(int candidate, int * direction, int size);
  /**
  Search if candidate 'id' has a neighbor, in distance 'dd' and
  dirextion 'dx', 'dy' - all within 1/4 of 'dd' margin.
  If so increase hop-count 'n' and total distance 'vx' and 'vy'.
  Function is called recursively.
  Stop recursion, when n has reached 'stopSize'. */
  void findNextInThisDirection4(
                  int id,   // candidate
                  float dx, // this far in x direction
                  float dy, // and this far in y direction
                  float dd, // at this distance
                  int * ID,   // id of last in this direction
                  float * vx, // add x-dist here if found
                  float * vy, // add y-dist here if found
                  int   *  n, // increase this count if found
                  int * next, // save route
                  int stopSize); // do not go further than this count
  /**
  Test if the square formed by the 3 corners are too flat to be
  a usable square.
  Returns true if size is usable.
  @todo There should be a parameter to decide what is usable, but
  for now a fixed value of 14 pixels wide is used. */
  bool testForFlatness(UCalibrationComponent * ht,
                       UCalibrationComponent * ht1,
                       UCalibrationComponent * ht2);

public:
  /** Temporary structure used when calculating center of
      gravity of cornetr position. */
  struct sPos
    {
      /** corner position in pixels of cell (left) */
      float px;
      /** corner position in pixels of cell (top) */
      float py;
      /** sum of intensity */
      int v;
      /** summed count of cells */
      int n;
    };
  /** Temporary structure used when calculating center of
      gravity of cornetr position. */
  typedef sPos UsPos;

protected:
  /** Grid of corner positions in a barcode frame grid */
  UsPos mCode[MAX_CELL_COUNT + MAX_CELL_FACTOR][MAX_CELL_COUNT + MAX_CELL_FACTOR];

public:
  /**
  List of calibration components */
  UCalibrationComponent hits[MAX_CALIBRATION_COMPONENTS];
  /** Number of used entries in 'hits[]' */
  int hitCount;
  /**
  hits ordered in grid */
  UCalibrationComponent * grid[MAX_CALIB_GRID_SIZE][MAX_CALIB_GRID_SIZE]; // orded hits
  /**
  Max rov when grid is filled. -1 if not found */
  int maxR;
  /**
  Max column when grid is filled. -1 if not found */
  int maxC;
  /** center (5x5) or 1,1 cm (6x6) position */
  int centR, centC;
  /** found mask: 0 = 6x6, 1= 5x5 portrait, 2= 5x5 landscape */
  int maskN;
};

//////////////////////////////////////////////////////////////

/**
This class holds one calibration mark, that is position of corner
in image pixels and position in real 2D coordinates on the flat barcode
surface.
Valid flag must be marked/checked. */
class UCalibrationMark
{
public:
  /**
  Constructor */
  UCalibrationMark();
  /**
  Copy entry from a UCalibrationComponent */
  void copy(UCalibrationComponent * source);
  /**
  Set all data
  and make valid. */
  void setMark(int frameRow, int frameCol,
               float imageX, float imageY,
               float realX, float realY);
public:
  /** not found marks are marked as invalid */
  bool valid;
  /** image coordinates */
  float ix, iy;
  /** chart coordinates relative to center */
  float rx, ry;
  /** row and column number on the border */
  int row, col;
};

/////////////////////////////////////////////////////////

/**
Class to hold positions of all frame corners in a barcode frame. These
corners are used as calibration marks.
Calibration marks are used to:
- Calculate camera position and orientation if barcode position is known.
- Calculate barcode position if camera position is known.
- Calculate camera intrinsic parameters. */
class UCalibrationMarkSet
{
public:
  /**
  Constructor */
  UCalibrationMarkSet();
  /**
  Set all data as invalid. */
  void clear();
  /**
  Save this message in logfile if significance is less than logLevel.
  Print also to console (debug feature) if verbose messages.
  Timestamp is optional, if not used, then Now() is used in logfile.*/
  inline void toLog(const char * info, int significance, UTime * timestamp = NULL)
  { toLogGmk(info, significance, timestamp); };
  /**
  Get finishing error distance change for x,y,z */
  inline double getErrDist() { return errDist;};
  /**
  Get finishing error angle change - combination of omega, phi and kappa */
  inline double getErrRot() { return errRot;};
  /**
  Get finishing error in pixels (absolute) */
  inline double getErrPix() { return errEpix;};
  /**
  Set evaluation stop limits in distance and angle.
  Iteration stops if error change between two
  iterations is below thise limits. */
  inline void setStopLimit(double distance, double angle)
  {
    errDistStopLimit = distance;
    errRotStopLimit = angle;
  }
  /**
  Get current error distance that stops iteration
  (in connection with angle stop limit). */
  double getStopLimitDistance() {return errDistStopLimit;};
  /**
  Get current error angle that stops iteration
  (in connection with distance stop limit). */
  double getStopLimitAngle() {return errRotStopLimit;};
  /**
  Try use the calibration marks to calculate new position
  and rotation parameters. <br>
  Takes a parameter string of the form "xyzOPK" for all
  6 parameters or a subset, e.g. "xy" to evaluate an estimate
  for x and y only. The 'chartP' must be the known position
  of the calibration chart. The chart must be vertical and facing
  the camera. the iteration continues for up to 50 loops
  or until distance error is less than 'errStopLimit'. <br>
  Returns 0 if seccessfull, and the new parameters in
  posCal and rotCal (and calValid will be true, and
  loops be the number of iterations to find solution). */
  bool evaluatePosRot(
                    const char * params,
                    UCamPar * camp,
                    UPosition * oldPos,
                    URotation * oldRot,
                    UPosition * chartP,
                    bool makeLogfile);
  /**
  Estimats a barcode position and rotation relative to the camera.
  If the initial position and rotation is known this is used as the
  initial vale.
  If no initial or expected position is given (set to NULL), then
  a serach will be performed in all 4 quadrants (+/- Phi and +/- Omega)
  to ensure that the real optimum is found.
  The stop limit in distance and rotation is the 3D change in position
  in last iteration, if both these are below the stop limits then the
  parameter estimation stops. <br>
  If 'adjRadial' then pixel positions is assumed distorted and is
  adjusted acccording to value of K1, K2 and head point found in
  camera parameters 'camp'. <br>
  The actual work is done in 'evaluateChartPosRot(-,-,-,-,logfile)'*/
  bool evaluateChartPosRot(
                    UPosition * initialChartPos,
                    URotation * initialChartRot,
                    double stopLimDist,
                    double stopLimRot,
                    UCamPar * camp,
                    bool adjRadial,
                    bool makeLogfile);
  /**
  Performs the above functions and report iteration results in
  file f, if this is != NULL.  */
  bool evaluateChartPosRot(
                    UPosition * initialChartPos,
                    URotation * initialChartRot,
                    double stopLimDist,
                    double stopLimRot,
                    UCamPar * camp,
                    bool adjRadial,
                    FILE * f);
  /**
  Get distance from expected pixel position to actual pixel position.<br>
  Guidemark corner is at (rx,ry)[meter] in this mark, and is converted
  to pixel coordinates using pos- and rotBarcode and camera info
  to get to pixel coordinates.
  From this difference is calculated. <br>
  NB! expects that pos- and rotBarcode is in camera coordinates. <br>
  Returns distance in pixels and calculated position in (cx,cy).
  */
  double getErrorInPixels(UCalibrationMark * thisMark,
                          float * cx, float * cy,
                          UCamPar * camp,
                          bool radErrAdj);
  /**
  Calculate average pixel position error by transferring real chart position
  to image pixel position using the values in the markSet and
  the transformations in the following values, i.e.:
    Xpix = b * P * Pcam * Ocam * Kcam * Tcam * Tbar^-1 * Pbar' * Obar' * Kbar' * Xbar. <br>
  If 'markDist' is 1, then all points are used else only [0, markDist, ... n* markDist],
  for n = 1,2,3... until no more marks.
  b and P is taken from cam->mb and cam->mP as mItoP. <br>
  if 'adjRadial' then radial correction is applied to pixel coordinates. <br>
  Returns -1.0 if error. */
  double geterrorInPixels(UPosition * chartPos,// assume this barcode position
                          URotation * chartRot,// assume this barcode rotation
                          UPosition * camPos,  // assume this camera position
                          URotation * camRot,  // assume this camera rotation
                          int markDist,         // use every 'markDist' mark
                          UCamPar * camp,
                          double * variance,   // return also variance
                          bool adjRadial);
  /**
  Show calibration status as info. */
  bool info(int cam);
  /**
  Set position of each frame position in a fw x fw squared frame.
  The side size is 'stride' meter. and an array of X and Y positions
  are provided as parameter for side A, B, C, D. The side order
  is in 'order[]'. if the corresponding entry in 'fwd[]' is 't' (true)
  array element [0] is the first in clockwise direction;
  if false, then element [fw] is the first.  */
  bool setFramePositions(int fw,  // frame width
                 float stride, char order[], char fwd[],
                 float AX[], float AY[], // A
                 float BX[], float BY[], // B
                 float CX[], float CY[], // C
                 float DX[], float DY[] // D
                 );
  /**
  Save mark positions to file
  If no filename is provided (is NULL) then a
  name is constructed 'image999calib.m' on data path with 999 beeing the
  provided image number. */
  bool saveMarksToFile(const char filename[],
                       bool radialErrorRemoved,
                       UCamPar * cam,
                       int imageNumber);
private:
  /**
  Set values in B matrix and K vector before least square calculation
  for rows r and r+1 for the parameters mentioned in params from: <br>
  camera constants headpoint hx and hy and focus length c, <br>
  initial estimate of camera position and rotation (camPos0 and camRot0), <br>
  one calibration mark with object and pixel position (pkt), <br>
  The base position of the chart, NB chart must be perpendicular to robot
  main Z axis e.g. parallel to world Y and X axis. <br>
  Returns true if focus length is > 0. */
  bool setCameraPosRowSet(const char * params,
              UMatrixBig * B, UMatrixBig * K, int r,
              double hx, double hy, double c,
              UPosition * camPos0, URotation * camRot0,
              UCalibrationMark * pkt, UPosition * chartPos);
  /**
  Set values in B matrix and K vector before least square calculation
  for rows r and r+1 for the parameters mentioned in params from: <br>
  camera constants headpoint hx and hy and focus length c, <br>
  initial estimate of barchart position and rotation (chartPos0 and chartRot0), <br>
  one calibration mark with object and pixel position (pkt), <br>
  Returns true if focus length is > 0. */
  bool setBarcodePosRowSet(
              UMatrixBig * B, UMatrixBig * K, int r,
              UCamPar * camp, bool adjRadial,
              UPosition * chartPos0, URotation * chartRot0,
              UCalibrationMark * pkt);
  /**
  Solve a set of equations by least square adjustment.
  B * R = K, where R is the unknown. <br>
  Returns true if solved */
  //bool solveLeastSquare(UMatrixBig * B, UVectorBig * R, UVectorBig * K);
public:
  // ID of set
  /** barcode number  or image number */
  unsigned long  barcode;    //
  /** number of dataset for this barcode chart */
  int barcodeSetNum;
  /** holds mark positions at frame of barcode */
  UCalibrationMark mark[MAX_FRAME_POSITIONS];
  /** number of valid points in mark array */
  int markCnt;
  /** size of one pixel relative to full resolution */
  float pixSize;
  /** is marks valid */
  bool valid;
  /** calibrated camera position */
  UPosition posCal;
  /**  calibrated camera rotation */
  URotation rotCal;
  /** is calibration valid */
  bool calValid;
  /**  is results implemented to camera */
  bool implemented;
  //
  /** Position of barcode chart relative to robot */
  UPosition posBarcode;
  /** Rotation of barcode chart at position. */
  URotation rotBarcode;
  /** is barcode chart position valid */
  bool barcodeValid;
  //
private:
  //
  /** end error in position (m) */
  double errDist;
  /** end error in rotation (radians) */
  double errRot;
  /** remaining average error distance in pixels */
  double errEpix;
  /** number of iteration loops */
  int loops;
  /**
  Max error limit, when estimation position error.
  When estimation error is less, then stop
  iterations. (in meters) */
  double errDistStopLimit;
  /**
  Max error limit, when estimation rotation error.
  When estimation error is less, then stop
  iterations. (in radians) */
  double errRotStopLimit;
};

/////////////////////////////////////////////////////////


/**
Class to hold calibration mark sets for a number of images (or for a number
of barcode frames). For each frame the position of all frame positions are saved
for calibrtion purposes or for calculation of camera or barcode position.
*/
class UCalibrationMarkSets
{
public:
  /**
  Constructor */
  UCalibrationMarkSets();
  /**
  Destructor */
  ~UCalibrationMarkSets() {};
  /**
  Get pointer to calibration set.
  Returns NULL if not within range present count. */
  UCalibrationMarkSet * getSet(int setNumber);
  /**
  Get number of (valid) saved sets */
  int getSavedSets();
  /**
  Get pointer to camera parameters used in estimation process. */
  inline UCamPar * getCamPar() { return &estCamPar;};
  /**
  initialize flags. */
  void clear();
  /**
  Save this message in logfile if significance is less than logLevel.
  Print also to console (debug feature) if verbose messages.
  Timestamp is optional, if not used, then Now() is used in logfile.*/
  inline void toLog(const char * info, int significance, UTime * timestamp = NULL)
  { toLogGmk(info, significance, timestamp); };
  /**
  Get calibration chart position */
  inline UPosition getCalibrationChartPosition() { return chartPos;};
  /**
  Set new calibration chart position */
  inline void setCalibrationChartPosition(UPosition newChartPos)
    { chartPos = newChartPos;};
  /**
  Set camera parameters */
  inline void setCameraParameters(float hx, float hy,
                  float k1, float k2,  float f,
                  float resFactor,
                  char * name)
  {
    estCamPar.setCameraParameters(hx, hy, k1, k2, f, resFactor);
  }
  /**
  Set camera parameters from copy */
  inline void setCameraParameters(UCamPar * camPar)
  {
    estCamPar.setCameraParameters(camPar);
  }
  /**
  Get new calibration mark set - if any more space. */
  UCalibrationMarkSet * getNewSet();
  /**
  Returns count of data sets with this barcode value. */
  int countSetsWith(unsigned int codeValue);
  /**
  Get a pointer to the n'th set with this barcode value.
  first set with this code has number 0.
  Returns NULL if it do not exist. */
  UCalibrationMarkSet * getThisSet(unsigned int codeValue, int nth);
  /**
  Save this barcode position set for later camera parameter
  estimation purpose.
  If no more space for sets or set is not valid, then return false. */
  bool saveNewSet(UCalibrationMarkSet * frameSet);
  /**
  Calculate camera internal parameters.
  Assume that each set has an associated chart position.
  Make an parameter least square estimate based
  in 4 corners in each (of 5) dataset. With the new estimate
  make a new position estimate for each set,
  then verify error size, to exit if error is minimum.
  REturns false if no result. */
  bool doEstimateCameraParameters(
                 UCamPar * oldEstimate,
                 bool debug);
  /** Depricated method to find camera parameters (test all positions) */
  bool doEstimateCameraParametersCarpet(
                 UCamPar * oldEstimate,
                 bool debug);
  /** Do binary search in an attempt to find parameters, */
  bool doEstimateCameraParametersBinary(
                 UCamPar * oldEstimate,
                 bool debug,
                 unsigned long setsToUse,
                 bool alsoFocalLength, bool alsoK1, bool alsoK2);
private:
  /**
  Evaluate position and rotation of each guidemark with this set
  of parameters and return the average pixel error. */
  float evaluatePixError(UCalibrationMarkSet * cset[],
                         unsigned int sets_used,
                         const float hx, const float hy,
                         const float k1, const float k2,
                         const float c,
                         float pixSize);
  /**
  Estimate k2 parameter within the provided limits using
  a binary search.
  Returns pixel error on best estimate and estimated parameter in
  'k2e' parameter. */
  float evaluatek2PixError(
                       UCalibrationMarkSet * cset[],
                       unsigned int sets_used,
                       const float hx, const float hy,
                       const float k1,
                       const float k2min, float k2max, float * k2e, int steps,
                       const float c,
                       float pixSize, bool alsoK2);
  /**
  Estimate k1 and k2 parameters within the provided limits using
  binary search.
  Returns pixel error on best estimate and estimated parameters in
  'k1e' and 'k2e' parameter. */
  float evaluatek1PixError(
                       UCalibrationMarkSet * cset[],
                       unsigned int sets_used,
                       const float hx, const float hy,
                       const float k1min, float k1max, float * k1e, int steps1,
                       const float k2min, float k2max, float * k2e, int steps2,
                       const float c,
                       float pixSize, bool alsoK1, bool alsoK2);

private:
  /**
  One set of calibration marks for each (possible) camera device. */
  UCalibrationMarkSet set[MAX_CALIBRATION_SETS_SAVED];
  int setCnt;  // number of sets used
  /**
  Known position of the calibration chart. for camera position
  evaluation
  Set from configuration file, or manually. */
  UPosition chartPos;
  /**
  Estimated camera internal parameters. */
  UCamParEst estCamPar;
};

/**
  This class encapsulates functions and methods used
  for calibration purposes.
  *@author Christian Andersen
  */
class UCalibrate
{
public:
  /**
  Constructor */
  UCalibrate();
  /**
  Destructor */
  ~UCalibrate();
  /**
  Save this message in logfile if significance is less than logLevel.
  Print also to console (debug feature) if verbose messages.
  Timestamp is optional, if not used, then Now() is used in logfile.*/
  inline void toLog(const char * info, int significance, UTime * timestamp = NULL)
  { toLogGmk(info, significance, timestamp); };
  /**
  Create or assign buffer images for guidemark/calibration process */
  bool allocateBufferImages(UImagePool * imgPool);
  /**
  Buffer space is allocated on heap.
  This function returns true if all went well. */
  bool isBufferImagesAvailable();
  /**
  Clear data store for frame corner positions. */
//  void clearParamData();
  /**
  Find barcodes in this image.
  Both horizontal and diagonal may be used simultaniously
  \param useVertFilt use filters for mostly vertical guidemarks
  \param useDiagFilt use filters for mostly diagonal guidemarks
  \Returns true if at least one is found. */
  bool findGmk(UCamMounted * cam,
               UImage * img,
               int frameSizeBlocks,
               int codeBlockFactor, // normally 2
               float sideStride, // size of a block (meter)
               int maxCodesToLookFor,
               bool findCode,
               bool findPosition,
               bool findRotation,
               float chartCenterHeight,
               bool findChartPos,
               bool nearGMK, bool useVertFilt, bool useDiagFilt,
               bool extraImages);
  /**
  Find calibration points in this source image with 100 corners,and stores the result
  calibration marks in the cc structure if successfull.
  if name provided, then points are saved in textfile with
  this name in the default image path with a .txt and .m extension.
  Returns number of found points (100 if all calibration points were found). */
/*  int Find100CalibrationPoints(
                      UImage * image, // (raw) image with points
                      char * name,
                      bool debug,
                      float stride);  // side size of squares (e.g. 0.025 m)*/
  /**
  Show calibration status as info messages. */
  bool ShowStatus( int setNumber);
  /**
  Compares four pixels from the filtered
  source image (fim) around a center cell.
  If there is sufficient intensity difference
  in the four corners a corner is found, and the pixel
  is marked in the destination image (cim).
  No corner is black, red is a white upper left and
  cyan is black upper left.          www---bbb    ---bbb---
  'GuardBand' is pixels from center  www---bbb    ---bbb---
  cell to corner cell                www---bbb    ---bbb---
  - default is 3 and is useually OK  ---------    www---www
  - (for filter mask 3x3)            ----x----    www-x-www
  - as illustrated to the right      ---------    www---www
  - (center cell is marked x)        bbb---www    ---bbb---
  - ("-" is a don't care pixel)      bbb---www    ---bbb---
  'IntensDiff' is minimum            bbb---www    ---bbb---
  difference in black and white intensity.
  'AverageDiff' is a maksimum limit of how far away in
  intensity average the sum of all four blobs may be away from
  image average at all corner points (in intensity * 4 units),
  this eliminates stray corner points outside chart.
  The function finds also diagonal corners in the same way
  Returns 0 if sucessfull.  */
  bool MarkCornerDetections4
                  (UImage * fim, // source image BLUR summed as needed
                   UImage * cim, // corner marked image
                   //int * avgCornerInt,
                   int GuardBandH,  // = 3 horizontal/vertical
                   int GuardBandD,  // = 3 diagonal
                   int IntensDiffH, // = 90 horizontal-vertical
                   int IntensDiffD, // = 50 diagonal
                   bool assumeBlackIsZero);
  /**
  Save found information, stored in grid array as calibration set for
  camera. */
  int SaveCalibrationMarks();

  /**
  Save the frame positions for this barcode frame.
  save under 'codeValue' number. If 'codeNum' is
  -1 then new entry is created else the entry with this
  'codeNum' and same 'codeValue' will be overwritten.
  if it do not exist, a new entry is created.
  All calculated values gets invalid.
  Returns true if entry created.
  Camera positioen relative to robot is in 'camera' (used set) or
  in this set for a just evaluated value.
  The default chart position is in 'calcamset.chartPos'.
  Position of chart relative to robot (and in robot coordinates)
  is in this set. Rotation of chart is in this set. */
  bool saveCodeFrame(unsigned long codeValue, int codeNum, float strideStride);
  /**
  Set camera parameters */
  inline void setCameraParameters(float hx, float hy,
                  float k1, float k2,  float f,
                  float resFactor,
                  char * name)
  {
    calcamset.setCameraParameters(hx, hy, k1, k2, f, resFactor, name);
  }
  /**
  Set camera parameters from copy */
  inline void setCameraParameters(UCamPar * camPar)
  {
    calcamset.setCameraParameters(camPar);
  }
  /**
  Find camera position assuming barcode chart is
  not rotated relative to camera orientation.
  if 'findChartPos' then estimate chart both rotation and position,
  defult is just rotation and z-distance (addition to
  expected chart position).
  Returns true if found succesfully.
  The result is returned in the dataset structure as
  posCal and rotCal, with the calValid if the data is valid.
  If 'debug' is true, then some extra logfiles will be produced
  in result or image path with filename based on image number. */
  bool findCameraPosition(UCamMounted * cam,
                          UCalibrationMarkSet * dataset,
                          bool findPosition, // for both camera rotation and position
                          float chartCenterHeight, // for camera orientation
                          bool debug); // make logfiles etc.
  /**
  Find position of barcode chart in this dataset.
  The position (x,y,z) and the orientation of the chart
  (Omega, Pgi and Kappa) is estimated. The accuracy depends on the pixel
  position accuracy in the dataset.
  Returns true if found.
  The found position is returned in the dataset structure as
  posBarcode and rotBarcode, with the barcodeValid if the data is valid.  */
  bool findBarcodeChartPosition(UCamMounted * cam,
                                UCalibrationMarkSet * dataset,
                                UCalibrationMark * indexCorner,
                                int gmkBlocks,      // blocks in frame (typical 7)
                                float gmkBlockSize, // size of one block (typical 0.02)
                                UImage * img,       // debug image
                                float pixFactor,    // size of debug image relative to source
                                bool removeRadialError, // if not removed already
                                bool debug          // more debug messages etc.
                               );
  /**
  Get number of found guidemarks */
  inline int getGmksCnt()
  { return gmksCnt; };
  /**
  Get a specific guidemark.
  NB! no index range check */
  inline UBarcode * getGmk(int index)
  { return &gmks[index]; };

public:
  /**
  Raw image to be used as basis for calibration.
  (copy from camera or saved image) */
  UImage * pRawImage;
  /**
  Averaged image after low pass. */
  UImage * pFiltImage;
  /**
  Image with detected corners marked.
  Last image stage before corner pixels are combined to components in cc. */
  UImage * pCornerImage;
  /**
  Image used both to lines to neighbor and to code-cell lines
  and histogram paint */
  UImage * pGridImage;
  /**
  Calibration components as extracted from pCornetImage. */
  UCalibrationComponents cc;
  /**
  Calibrate information sets (calcamset) stores one set of points per camera
  device.*/
  UCalibrationMarkSets calcamset;

protected:
  /**
  Found barcodes in last image */
  UBarcode gmks[MAX_CODES_IN_ONE_IMAGE];
  /**
  Valid number of barcodes */
  int gmksCnt;
  /**
  Working frameset, with latest information on frame */
  UCalibrationMarkSet frameSet;

};

////////////////////////////////////////////////////

#endif
