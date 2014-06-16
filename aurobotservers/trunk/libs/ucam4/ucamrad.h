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

#ifndef UCAMRAD_H
#define UCAMRAD_H

#include <ugen4/ucampar.h>
#include <urob4/uvarpool.h>
//#include "ucamdevice.h"
#include "uimagelog.h"
#include "ucampantiltdevice.h"

/**
Radial error values is valid for lower right part of image
and includes correcion values valid for image headpoint offset
of up to 50 pixels. */
#define MAX_IMAGE_WIDTH_RAD_SQUARE  (UIMAGE_MAX_WIDTH  / 2 + 50)
#define MAX_IMAGE_HEIGHT_RAD_SQUARE (UIMAGE_MAX_HEIGHT / 2 + 50)
/**
Class to hold a pixel offset during radial error removal.
Is used in UCamera structure for fast error removal. <br>
The source pixel for this position is to be found
with this offset in the original image. */
class UXYoffset
{
public:
  short int dx;
  short int dy;
};

/**
Class that includes camera functtions to remove radial error in the raw images.
The calss is based on both the camera device 'UCamDevice' and camera
parameters 'UCamPar'.
  *@author Christian Andersen
  */
class UCamRad : public UCamPanTiltDevice
{
public:
  /**
  Constructor */
  UCamRad(UCamDevBase * device);
  /**
  Destructor */
  virtual ~UCamRad();
  /**
  Get pointer to camera parameters */
  UCamPar * getCamPar() {return &par;};
  /**
  Overwrite get Image Snapshot to allow removal of radial error
  if such removal is possible and set as default.
  returns true if image were captured and
  radial error status were removed or not as requetsed.
  Result is always in URawImage YUV42 or Bayer RGGB format.
  If 'image' == NULL then an image is captured, and discarded. */
  bool getImageSnapshot(UImage * image); //, bool remRadErr);
  /**
  Make textfile with radial correction values.
  Saves pixel offset values for the last used resolution. */
  void saveRadialCorrectionMatrix(char * filename);
  /**
  Removal of radial error is done in according to
  calibration information in the referenced camera.
  Removal follows the guidelines in Image Analysis,
  Vision and Computer Graphics DTU/IMM publication
  by Jens Michael Carstensen Lyngby 2001, page 75.  <br>
  focus length, Head-point (hx and hy) and
  correction factors (k1 and k2)
  are always in 640x480 resolution. */
  bool setRadialErrorMatrix();
  /**
  Remove radial error from this image.
  uses openCV cvRemap(..) function to remove error.
  Resolution of destination will be as source.
  Uses pre-calculated correction matrix, if image size or rectification parameters has
  changed, then the rectification matrix is recalculated.
  \param source is source image
  \param destination is destination image - both must exist at call.
  \returns true if rectified. */
  bool removeRadialError(UImage * source, UImage * destination);
  /** Should radial error be removed by default when
      auto repeat is turned on.
      Is set when capturing an image */
  inline bool getDefaultRemRadErr() { return defaultRemRadErr;};
  /** Should radial error be removed by default when
      auto repeat is turned on.
      Is set when capturing an image */
  inline void setDefaultRemRadErr(bool value)
                          { defaultRemRadErr = value; };
  /** Default color format if
      auto repeat is turned on.
      is set only when capturing an HDR image set. */
  inline int getDefaultColFormat() { return defaultColFormat;};
  /**
  Set camera internal parameters and redo the matrix for
  radial error removal and for conversion in camera coordinates
  (pixel to 3D and reverse).
  Parameters are expected to be in scale 1, i.e. 640x480 image size
  focal length, hx, hy (headpoint) are all in pixels and
  pixSizeFactor is pixel size relative to 640x480,
  i.e. 320x240 gives a pixSizeFactor of 2.0.
  Returns true if successful. */
  bool setCameraParameters(float hx, float hy,
                  float k1, float k2,  float focalLng);
  /**
  Create locally maintained variables - if any */
  virtual void createVars();
  /**
  Update lens parameters and setup distortion correction matrix - as used by openCV.
  and as used by the old correction scheme - in the UCamPar structure.
  Parameters are assumed to be loaded into the corresponding clobal variables
  \param height is the actual image height in pixels.
  \param width is the actual width of image in pixels. */
  void updateLensParams(const int height, const int width);
                  
protected:
  /**
  A call to see if anyone need the image logged.
  \param image is the newly captured image.
  \returns true if used. */
  virtual inline bool logImage(UImage * img)
  { return false; };

protected:
  /**
  Remove radial error for one pixel plane -
  either Y, U or V.
  A source pointer to the top-left pixel must be
  provided for source and destination, and the size of the images.
  The decimal factor is the factor in the radial error matrix
  that desides the decimal point. At full resolution this
  will normally be 10, at half resolution 20 and so on.
  For YUV the U and V the height, width and pixel size must
  be given in half resolution and further 'halfRes' must
  be true */
  bool removeRadialErrorOnePlane(unsigned char ps[],
                                 unsigned char pd[],
                                 unsigned int height,
                                 unsigned int width,
                                 float pixSize,
                                 bool halfRes);
  /**
  Removes radial error based on offset array in full resolution
  as produced by 'setRadialErrorMatrix()'. Image pointers
  must be buffers of 'UPixel' ordered one row at a time, so that
  the next row follows imidiately after the first.
  A source pointer to the top-left pixel must be
  provided for source and destination, and the size of the images.
  The decimal factor is the factor in the radial error matrix
  that desides the decimal point. A pixel size must be provide for
  correct scaling of offset matrix values. */
  bool removeRadialErrorPixels(UPixel ps[],
                               UPixel pd[],
                               unsigned int height,
                               unsigned int width,
                               float pixSize);
  /**
  Calculate where to fetch a pixel in the distorted
  image (raw) to this corrected image position relative
  to this {x,y} position (+ is right and down).
  The returned result {X,Y} position is in 16 bit
  accuracy with fixed decimal point, i.e. multiplied
  with 'resultDecimalFactor' (probably 10).
  The decimal part should be used to interpolate
  between pixels in the raw image. */
  UXYoffset getRadialU2DOffsetInt(int x, int y);
  /**
  Same as above, but makes float calculations
  and return change in coordinates
  to dx, dy. to get the corresponding position
  in the raw image. */
  bool getRadialU2DOffset(float xu, float yu,
                  float * dxd, float * dyd);
  /**
  Set the parameters derived from resolution, i.e.
  conversion matrix, radial error valeues etc.*/
  virtual void imageSizeChanged(double iResFactor);
  /**
  Has focal length, headpoint or lens parameters changed - i.e. must
  rectification matrix be re-calculated.
  \returns true if any of these parameters has changed. */
  bool lensParChanged();

public:
  /**
   * structure for variables for this camera */
  UVarPool * varBaseStruct;
  
private:
  /**
  Camera parameters as local object, as it is used during estimation etc. */
  UCamPar par;
  /** Radial error removal table */
  UXYoffset radialOffset[MAX_IMAGE_HEIGHT_RAD_SQUARE][MAX_IMAGE_WIDTH_RAD_SQUARE];
  /**
  Is the radial error table calculated */
  bool radialOffsetValid;
  /** Should radial error be removed by default when
      auto repeat is turned on. */
  bool defaultRemRadErr;
  /** Default color format if
      auto repeat is turned on. */
  int defaultColFormat;
  /// pointer to intrinsic camera parameters (focal-length and head-point)
  UVariable * varIntrinsic;
  /// pointer to lens distortion parameters radial (k1, k2, k3) and tangential - ccd-chip not parallel to lens (p1, p2))
  UVariable * varDistortion;
  // Build the undistort map that we will use for all subsequent frames
  IplImage * mapx; //= cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
  IplImage * mapy; // = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
  //  cvInitUndistortMap( intrinsic, distortion, mapx, mapy );

};

#endif

