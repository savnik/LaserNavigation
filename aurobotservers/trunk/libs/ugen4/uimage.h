/***************************************************************************
 *   Copyright (C) 2006-2011 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
 *
 *  $Rev: 1834 $
 *  $Id: uimage.h 1834 2012-02-19 13:44:53Z jca $
 *
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

#ifndef UIMAGE_H
#define UIMAGE_H

#include <stdint.h>
#include "ucommon.h"
#include "utime.h"
#include "u3d.h"
#include "umatrix.h"
#include "conf.h"
#include "ulock.h"

/**
If no highgui is installed */
#define NO_OPENCV_HIGHGUI
/**
If no libpng is available - then no save/loadPNG() */
//#define IMAGE_NO_PNG
/**
if no zlib is available - then no pack/unpackZLIB() */
//#define IMAGE_NO_ZLIB

const int UIMAGE_MAX_HEIGHT = 576;
const int UIMAGE_MAX_WIDTH = 852;

/**
Class to hold a virtual image (of 1 pixel)
with all the functionality of an image. <br>
Should only be used as UImage640 or similar
enherited classes with larger data structure. <br>
The class do not use virtual methods as the
class is used in shared memory, where the virtual
method pointer stack is uninitialized (the class
is not initialized by a constructor call) */
class UImage : public ULock
{
public:
  /**
  Constructor - resets data */
  UImage();
  /**
  Destructor */
  virtual ~UImage();
  /**
  Initialize image data area.
  This buffer area must be at least iHeight * iWidth * channels, as
  pixels are always in unsigned bye format. */
  bool initImage(const unsigned int iHeight, const unsigned int iWidth,
               char * data, const unsigned int bufferSize,
               const int channels = 3, const int depth = 8);
  /**
   * Initialize image size. Pixel depth is always unsigned byte.
   * \param height is height of image
   * \param width is width of image (height * width * channels should be less
   * than 800*600*3 = 1440000 (bytes)
   * \param channels may be 1,2,3,4 - default is 3 (usually RGB).
   * \param depth should be 8 (bits) if channels > 1, else 16 is OK
   * \param colorType is a string with color type - valid types are:
   "BW", "RGB", "BGR", "YUV". "RGBA", "BGRA", "GRAY", "BW16S", "BGGR", "RGGB", "YUV420"
   * \returns true if changes as requested */
  bool setSize(const unsigned int iHeight, const unsigned int iWidth,
               const int channels /* = 3 */, const int depth /* = 8 */,
               const char * colorType = NULL);
  /**
  Resize image - and rearrange image buffer as needed
   * \returns true if possible */
  bool resize(const unsigned int iHeight, const unsigned int iWidth,
              const int channels = 3, const int depth = 8);
  /**
  Resize image buffer (if needed).
  Allocates more memory image buffer (realloc) if new size is larger than current buffer.
  if image is smaller than buffer no action is taken.
  \param iHeight is the new image height.
  \param iWidth is the new image width.
  \param channels is the number of colour channels (1=BW, 3 or 4 if colour)
  \param depth is number of bits per channel - usually 8 (is truncated down to bytes, i.e. 8 and 16 is allowed only).
  \returns true if buffer can hold specified image. */
  bool resizeBuffer(const unsigned int iHeight, const unsigned int iWidth,
                    const int channels , const int depth );
  /**
  Change size only, i.e. keep depth, channels and color-mode.
  Returns true if space for new size.
  Do not change image content, and
  any existing image will be folded differently */
  bool setSizeOnly(const unsigned int iHeight,
                  const unsigned int iWidth);
  /**
  Set Region of Interest for image processing
  using filter and other methods. */
  inline void setROI(int x, int y, int w, int h)
  {
    cvSetImageROI(&img, cvRect(x,y,w,h));
  }
  /**
  Set size to the largest image that the buffer can hold
  when the width/height relation is 4/3.
  Returns true if this is possible (buffer can hold an image of 4x3 pixels) */
  bool setMaxSize43();
  /**
  Set name and extension inside image meta data.
  to the attributes 'name' and 'saveType'.
  If no name is provided and no name exist already
  then a name is created using the image number on the form
  img99999.
  If no type is provided, and non existing,
  then the type is set to 'png'.
  On return there is a valid name and extension in
  the name and saveType fields. */
  void setNameAndExt(const char * basename, const char * ext = NULL);
  /**
  Set image name used in save and in caption in UClient */
  inline void setName(const char * imageName)
  {
    strncpy(name, imageName, MAX_IMG_NAME_SIZE);
    name[MAX_IMG_NAME_SIZE - 1] = '\0';
  }
  /**
  Set image as valid image without any camera and with max size.
  Primarily used for planar view images, where no camera
  projection is needed. */
  bool GetNewNonCameraImage();
  /**
  Get a pixel value in right color format from RGB source */
  UPixel pixRGB(unsigned char r,
                unsigned char g,
                unsigned char b);
  /**
  Get a pixel value in right color format from YUV source */
  UPixel pixYUV(unsigned char y,
                unsigned char u,
                unsigned char v);
  /**
   * Get a pointer to an image line of UPixels
   * \param line is row number starting at 0 counting down in image (less than height())
   * \returns pointer to first pixel in line */
  inline UPixel * getLine(unsigned int line)
  { return getPixRef(line, 0); };
  /**
   * Get a pointer to a UPixel
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \returns pointer to this pixel. */
  inline UPixel * getPixRef(unsigned int line,
                        unsigned int column)
  { return (UPixel *) cvPtr2D(cvArr(), line, column); };
  /**
   * Get a pointer to a pixel, - for use with 8-bit BW images
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \returns pointer to this pixel. */
  inline char * getCharRef(unsigned int line,
                              unsigned int column)
  { return (char *) cvPtr2D(cvArr(), line, column); };
  /**
   * Get a pointer to a pixel, - for use with unsigned 8-bit BW images
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \returns pointer to this pixel. */
  inline unsigned char * getUCharRef(unsigned int line,
                             unsigned int column)
  { return cvPtr2D(cvArr(), line, column); };
  /**
   * Get a pointer to a pixel, - for use with 32-bit color images
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \returns pointer to this pixel. */
  inline int * getIntRef(unsigned int line,
                             unsigned int column)
  { return (int *) cvPtr2D(cvArr(), line, column); };
  /**
  Get a UPixel at this position.
  Line count positive down and column counts positiove right.
  Returns pixel in current color format (YUV420 returns in YUV) and
  BW returns in YUV too. */
  UPixel getPix(unsigned int line,
                unsigned int column);
  /**
  Get a UPixel at this position.
  Line count positive down and column counts positiove right.
  Returns pixel in desired color format (YUV420 returns in YUV) and
  BW returns in YUV too. */
  UPixel getPix(unsigned int line,
                unsigned int column,
                int inColFormat);
  /**
  Get pixel as integer.
  The pixel is coded as 0xRRGGBB 3x 8 bit if in RGB format
  or in general 0xP1P2P3 for the 3 pixel planes in 3x8 bit color.
  \param line is the image line - zero is left, positive right.
  \param column is the image column - zero is top line, positive down.
  \returns pixel packed into integer. */
  int getPixInt(unsigned int line,
                unsigned int column);

  /**
   * Set a pixel to this value.
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \param pix is the value to be set at this point */
  inline void setPix(unsigned int line, unsigned int column,
              UPixel pix)
  { *getPixRef(line, column) = pix; };
  /**
   * Set a pixel to this value (8-bit (char)).
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \param pix is the value to be set at this point */
  inline void setPixChar(unsigned int line, unsigned int column,
                     char pix)
  { *getCharRef(line, column) = pix; };
  /**
   * Set a pixel to this value (8-bit unsigned).
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \param pix is the value to be set at this point */
  inline void setPixUChar(unsigned int line, unsigned int column,
                     unsigned char pix)
  { *getUCharRef(line, column) = pix; };
  /**
   * Set a pixel to this value (32-bit int).
   * \param line is row number starting at 0 counting down in image (less than height())
   * \param column is coloumn starting at 0 counting positive right (less than width())
   * \param pix is the value to be set at this point */
  inline void setPixInt(unsigned int line, unsigned int column,
                     int pix)
  { *getIntRef(line, column) = pix; };
  /**
  Is this pixel position in image range */
  inline bool inRange (int line, int column)
  { return (line < img.height) and (column < img.width) and
      (line >= 0) and (column >= 0); };
  /**
  Paints a line across the image at position pos for axis (in x,y,z)
  given the limits min and max for both axis and axis specifier sw
  colour is 1:Minor,2:Major,3:Zero, 4,4++ dotted in major colour
  if sw = 'z' then axis='x' means width (right positive)
               and axis='y' means height (up positive) */
  void gridLine(float Pos, char axis,
                  float minH, float maxH,
                  float minW, float maxW,
                  char sw = 'y',  int Color = 1);
  /**
  Paint a free line in image from position 1 to position 2
  in specified color, width and perspective as indicated */
  void lineInGrid(UPosition pos1,
                       UPosition pos2,
                       float minH, float maxH,
                       float minW, float maxW,
                       char sw, UPixel * rgb,
                       UMatrix4 * mA = NULL,
                       int lineWidth = 1);
  /**
  Paints a series of grid lines along one coordinate (axis)
  axis specified as above. 'major' is major line distance,
  'minor' is minor line distance.
  sw: // y => x-z plot => axis: x or z - z as width
         z => y-x plot => axis: y or x - x as width
         x => y-z plot => axis: y or z - z as width */
  void gridLines(float major, float minor,
                       char axis,
                       float minH, float maxH,
                       float minW, float maxW, char sw);
  /**
  Paint one pixel in mixtuer of provided color and
  same pixel in this image or provided image */
  void setPixel(int x1, int y1, UPixel rgb,
                        UImage * image = NULL,
                        int procent = 0);
  /**
  Tone full image towards provided color */
  void tone(UPixel * rgb, int procent);
  inline void tone(UPixel rgb, int procent)
    { tone(&rgb, procent); };
  /**
  Set colot type.
  The following types are recognized:
  "BW", "RGB", "BGR", "YUV". "RGBA", "BGRA", "GRAY", "BW16S", "BGGR", "RGGB", "YUV420", "YUV422"
  Returns ordinal value.
  The image data is left unchanged. */
  int setColorType(const char *);
  /**
  Convert color type string to integer value.
  Color format is not case sensitive.
  Does not change anything in image.
  \param col is string representation BW, gray, RGB, BGR, BW16s, RGGB BGGR BGRA RGBA YUV YUV420.
  \returns the ordinal value. */
  static int toColFormatInt(const char * col);

  /**
   * Set image color format to one of
   * PIX_PLANES_BW ,
   * PIX_PLANES_RGB,
   * PIX_PLANES_BGR,
   * PIX_PLANES_YUV,
   * PIX_PLANES_YUV420,
   * PIX_PLANES_YUV422,
   * PIX_PLANES_BW16S. (16 bit signed integer per pixel),
   * PIX_PLANES_BGGR (Bayer coded 1 plane image)
   * PIX_PLANES_RGGB (Bayer coded 1 plane image)
   * PIX_PLANES_BGRA (32 bit interger per pixel (1 plane image in openCV))
   * PIX_PLANES_RGBA (32 bit interger per pixel (1 plane image in openCV))
  Returns true if one of these formats.
  There is no check to prohibit a BW image to be
  changed to RBG or the other way. The
  image data is unchanged.  */
  bool setColorType(int colorFormat);
  /**
  Get the color type as integer value */
  inline int getColorType() { return colFormat;};
  /**
  Get image comour format as string
  \returns pointer to constant string with format name. */
  const char * getColorTypeString();
  /**
  Is color format RGB */
  inline bool isRGB()
      { return colFormat == PIX_PLANES_RGB;};
  /**
  Is color format BGR */
  inline bool isBGR()
      { return colFormat == PIX_PLANES_BGR;};
  /**
  Is color format Bayer */
  inline bool isBayer()
      { return colFormat == PIX_PLANES_BGGR or
               colFormat == PIX_PLANES_RGGB or
               colFormat == PIX_PLANES_GBRG or
               colFormat == PIX_PLANES_GRBG;};
  /**
  Is color format RGB */
  inline bool isYUV()
      { return colFormat == PIX_PLANES_YUV;};
  /**
  Is color format BW */
  inline bool isBW()
      { return colFormat == PIX_PLANES_BW or colFormat == PIX_PLANES_BAYER;};
  /**
  Is 16 bit signed BW image */
  inline bool isBW16s()
      { return colFormat == PIX_PLANES_BW16S;};
  /**
  Is 16 bit signed BW image */
  inline bool isBW16u()
      { return colFormat == PIX_PLANES_BW16U;};
  /**
  Is YUV 420 planar format */
  inline bool isYUV420()
  { return colFormat == PIX_PLANES_YUV420;};
  /**
  Is YUV 422 planar format */
  inline bool isYUV422()
  { return colFormat == PIX_PLANES_YUV422;};
  /**
  Is image valid (configured) */
  inline bool isValid() { return valid;};
  /**
  Mark the image as updated, i.e. increase the
  used number, so that debug users can see the
  updated image.
  Further set the update time almost the same purpose.
  The 'used' count is a bit depricated. */
  inline void updated()
    { imgUpdated(); };
  /**
  Convert this image to BW format.
  Changes the image format to one channel
  If the source is 16bit (BW16s) signed BW image, then
  the result is limited to +/- 128 and is
  shifted to range 0 to 255. */
  bool toBW(UImage * dest = NULL);
  /**
  Convert from YUV format to RGB format.
  \param dest is the destination image - may be the same image.
  \Returns true if converted */
  bool toRGB(UImage * dest = NULL);
  /**
  Convert from YUV format to RGB format.
  NB! can not convert a BW image.
  Returns true if converted */
  bool toYUV(UImage * dest = NULL);
  /**
  Convert to BGR (native opencv format)
  NB! can not convert a BW image.
  Returns true if converted. */
  bool toBGR(UImage * dest = NULL);
  /**
  Conver image to an image, where the intensity is removed
  and replaced with BGR values, where the sum is always 256
  (+/- 3)
  Returns true if converted. */
  bool toCromaBGR(UImage * dest = NULL);
  /**
  Increase colour diversity by applying this gain to the colour
  channels, ie multiply U=(U-128)*gain and V(V-128)*gain.
  Saturation limits to 255 and 0.
  gain may have one decimal, as the value is calculated as integer - the 
  used operation is U = ((U-128) * int(gain*8)) >> 3)*/
  bool colourSaturate(double gain);
  inline bool colorSaturate(double gain)
  { return colourSaturate(gain); };
  /**
  Invert intensity for image, by inverting the Y channal. If
  The image is RGB then it is converted before the operation and
  back after the operation.
  returns 0; */
  bool invertY();
  /**
  clears image to this value amd makes image valid  */
  void clear(UPixel rgb);
  /**
  Clears the image to white, and makes the image valid */
  void clear(void);
  /**
  Clears all bytes in image to this value. */
  void clear(int v);
  /**
  Clears or sets all UImage propreties to a default value */
  //void clearAll(void);
  /**
  Copy just image meta data, that is name, time, position and color format,
  position etc., but not the bitmap itself.
  if 'andSize', then the image size and color format is changed to that of
  the source image.
  A full copy is implemented as a copyMeta() followed by a copyJustImage(). */
  bool copyMeta(UImage * source, bool andSize);
  /**
  Copy image rgb data, width and height, leaving other data unchanged */
  bool copyJustImage(UImage * source);
  /**
  Make a copy of source image, but scanle size down by the integer
  factor provided.
  The scale down uses center pixel in new image.
  Reurns true if possible. */
  bool copyScaleDown(UImage * source, int factor);
  /**
  Copy source image to this image and scale this image with factor.
  The scaling uses nearest neighbour method (no interpolation)
  \param source is the image to copy
  \param factor is the scale factor - 0.5 reduces to half size, 3 expands each axis by a factor 3.
  \returns true if scaling succeded. */
  bool copyAndScale(UImage * source, double factor);
  /**
  Resample source image to destination image, so that it fills to max size.
  The copy is not optimized, and there is no anti-aliasing, and at
  decreasing size the nearest pixel is used only. <br>
  Some image parameters are copied as well,
  i.e.: time, name, number, intensity, isRGB, cam.
  Returns false if source data is not valid */
  bool copyToMaxRes(UImage * source);
  /**
  Copy image all image data. <br>
  Returns false if source data is not valid or resolution is incomatible. */
  bool copy(UImage * source);
  /**
  Get image width in pixels. */
  inline unsigned int width() { return img.width;} ;
  /**
  Get image height in pixels. */
  inline unsigned int height() { return img.height;} ;
  /**
  Get image width in pixels. */
  inline unsigned int getWidth() { return img.width;} ;
  /**
  Get image height in pixels. */
  inline unsigned int getHeight() { return img.height;} ;
  /**
  Get pointer to image data */
  inline UPixel * getData() { return (UPixel *)img.imageData;};
  /**
  Get array structure recognized by opencv */
  inline CvArr * cvArr() { return &img; };
  /**
  get pointer to openCV image header.
  This is used in connection with image functions other than
  the simple byte conversions. */
  inline IplImage * getIplImage() { return &img; };
  /**
  */
//  cv::Mat mat()
//  { return cv::Mat v(img.height, img.width, CV_8UC3, img.imgData)};

  /**
  Get buffer size in bytes */
  inline unsigned int maxBytes() { return bufferBytes;};
  /**
  Get buffer size in bytes */
  inline unsigned int maxPixels()
  {
    return bufferBytes/img.nChannels/img.depth* 8;
  };
  /**
  Test to see if the image can hold an image of this size */
  inline bool isTooBig(const unsigned int w,
                       const unsigned int h)
  {
    return ((w * h) > maxPixels());
  }
#ifndef NO_OPENCV_HIGHGUI
  /**
  Save image to file.
  The actual filename is combined
  by concatinating the three provided strings
  in the normal way:
  path + '/' + name + '.' + type.
  If type is NULL, then type is assumed included in name.
  if path is NULL then path is assumed included in name.
  NB! color format is assumed to be be either BW or BGR before the call.
  Returns true is saved.
  */
  bool save(const char * name, const char * type, const char * path);
  /**
  Load image from file.
  If name is not provided (==NULL), then the internal name is used.
  If type or path is not provided (==NULL) then the missing parts are assumed included
  in name. <br>
  The file is first loaded to a dynamically allocated storage and
  the copied to this image buffer if size permits.
  Image structure must be valid ('valid' == true). <br>
  If size is too big then top-left part (present image size) is loaded. <br>
  Returns true if image is loaded.
  image 'name' is saved as image name if image or part of image is loaded.
  Other image meta data (position, time, radial-err, cam, imageNumber and used is not touched. */
  bool load(const char * name, const char * type, const char * path);
#endif
  /**
  Save image in BMP format (do not require opencv highgui */
  bool saveBMP(const char filename[]);
  /**
  Saves the image in Windows BMP format with more separate name components.
  There is always added an ".bmp" extension.
   e.g. saveBMP("/home/chr", "img0045", 7, "Gaus3x3"); is equvivalent with
        saveBMP("/home/chr/img0045-07Gaus3x3.bmp");
  The image name is set to the same name excluding the path.
  If the integer part 'num' is -1, then the "-07" part of the name is omitted */
  bool saveBMP(const char path[], const char basename[],
               const int num,  const char nameAdd[]);
  /**
  Load image from BMP file (do not require opencv highgui.
  Takes a fully qualified filename.
  Returns true if loaded.
  Returns false if image is not found, too big of in other ways not qualified. */
  bool loadBMP(const char filename[]);
  /**
  Load an image in Windows BMP format with more separate name components.
  There is always added an ".bmp" extension.
    e.g. saveBMP("/home/chr", "img0045", 7, "Gaus3x3"); is equvivalent with
    //      saveBMP("/home/chr/img0045-07Gaus3x3.bmp");
  * The image name is set to the same name excluding the path.
  * If the integer part 'num' is -1, then the "-07" part of the name is omitted */
  bool loadBMP(const char path[], const char basename[],
               const int num,  const char nameAdd[]);
  /**
  * Load an RGB image in png format (no larger than buffer allows).
  * Filename is assumed to be fully qualified. */
  bool loadPNG(const char * filename);
  /**
  * Save image in PNG format.
  * Filename is assumed to be fully qualified. */
  bool savePNG(const char * filename);
  /**
  * Save image as space separated integer values line by line.
  * Filename is assumed to be fully qualified. */
  bool saveTxt(const char * filename);
  /**
  Pack image data using Z-lib.
  Returns the number of bytes used in the buffer.
  Returns -1 packing failed.
  Will fail if ZLIB is not available */
  int packZLIB(char * buffer, int bufferSize);
  /**
  Unpack image data
  original image height and width must be provided along
  with color type - see setColorType().
  The buffer must hold the packed image to unpack (assumed packed using packZLIB).
  bufferCnt is the length of the bufferdata to use. */
  bool unpackZLIB(int rows, int cols,
                  const char * colFormat,
                  char * buffer, int bufferCnt);
  /**
  Save image meta data to file.
  Returns true if saved. */
  bool saveMeta(const char * name, const char * type, const char * path);
  /**
  Get number of color planes (channels) */
  inline int getChannels() { return img.nChannels;};
  /**
  Get number of bits in each color plane - in each channel */
  inline int getDepth() { return img.depth;};
  /**
  Get number of bytes in image */
  inline unsigned int imgBytes()
  {
    int b = width() * height() * getChannels() * getDepth() / 8;
    if (colFormat == PIX_PLANES_YUV420)
      b /= 2;
    if (colFormat == PIX_PLANES_YUV422)
      b = (b * 2)/3;
    return b;
  }
  /**
  Get number of bytes in image */
  inline unsigned int getDataSize()
  {
    return imgBytes();
  }
  /**
   * \brief Get number of bytes in image buffer.
   * This is the total size of the allocated image buffer. This may be larger than the
   * the number of bytes actually used by the image, and allows thus change in image size or format
   * without allocating new buffer space.
   * \returns number of bytes in reserver buffer (unsigned integer)  */
  inline unsigned int getBufferSize()
  { return maxBytes();};
  /**
  Edge filter the image to the destination image.
  if 'absoluteValue' the the result is the absolute value of
  the sum of found edges. Else the found signed sum is
  added to the source image - as a contour enhancement.
  The reducFactor is used to divide the found value, so
  a factor 4 gives an average value of the 4 edge filtrers. */
  void edgeSobel(UImage * destination, bool absoluteValue = true, int reducFactor = 1);
  /**
  Get edge pixel in area up to 3 pixels left of the
  3 rows of pixels provided in these pointers.
  The center is assumed to be p2[1].
  Returns the Sobel value of this pixel. */
  UPixel sobel(UPixel * p1, UPixel * p2, UPixel * p3);
  /**
  Get edge pixel in area up to 3 pixels left of the
  3 rows of pixels provided in these pointers.
  The center is assumed to be p2[1].
  Returns a signed Sobel value added to the pixRef pixel value. */
  UPixel sobel(UPixel * p1, UPixel * p2, UPixel * p3, UPixel * pixRef, int reducFactor);
  /**
  Compare pixels for intensity (r+g+b).
  Returns positive if p1 > p2, and 0 if equal,
  and negative if p2 > p1. */
  int pixCmp(UPixel * p1, UPixel * p2);
  /**
  Order this number of pixels after intensity in all channels,
  Uses pixCmp(...) to when sorting pixels.
  Returns the center element.
  NB! no range checks */
  UPixel * pixSort(UPixel *** ps, int pixCnt);
  /**
  Do median filter for a part of an image.
  The part from (row1, col1) to (row2,col2)
  is filtered - row1,col1 must be top-left corner.
  at edges the edge pixel is reused as needed.
  The row and column row2 and col2 is not inclusive
  in the filtered area.
  Returns true if destination image is valid and
  mask size is within handled size (fH <= 11 and
  (fH * fW) < 200. */
  bool median(UImage * dimg, // destination image
              unsigned int row1, unsigned int col1,  // top left of area
              unsigned int row2, unsigned int col2,  // bottol right of area
              unsigned int fW, unsigned int fH     // filter width and height
              );
  /**
  Shift image this number of rows up or down.
  The shift is down if rows is positive.
  If byteFill is zero or positive the
  now empty part of the image is filled
  with this value. */
  void shiftRows(int rows, int byteFill = -1);
  /**
  Shift image this number of columns left or right.
  The shift is right if cols is positive.
  If byteFill is zero or positive the
  now empty part of the image is filled
  with this value. */
  void shiftCols(int cols, int byteFill = -1);
  /**
  Paint horizontal line in one or more of the image planes
  from column 'c1' to column 'c2'. paint with
  value p1, p2, p3 in the 3 planes, if a plane is
  -1 (negative) then plane is not changed. */
  void hLine(unsigned int line,
             unsigned int c1, unsigned int c2,
             int p1, int p2, int p3);
  /**
  Paint vertical line in one or more of the image planes
  from row 'r1' to row 'r2'. paint with
  value p1, p2, p3 in the 3 planes, if a plane is
  -1 (negative) then plane is not changed. */
  void vLine(unsigned int col,
             unsigned int r1, unsigned int r2,
             int p1, int p2, int p3);
  /**
  Set four intensity pixels and one set of color pixels
  The line and column will be truncated to even numbers.
  NB! works correct if format is YUV420 only! */
  bool setQuadPix(unsigned int line, unsigned int column,
                  unsigned char I11, // top left
                  unsigned char I12, // bottom left
                  unsigned char I21, // top right
                  unsigned char I22, // botom right
                  unsigned char U,
                  unsigned char V);
  /**
  Get the 4 intensity values and the related color information.
  The line and column is for the full image resolution, but
  is truncated so that the LSB is  not used - i.e.
  getQuadPix(5,5, ...) gets same values as getQuadPix(4,4, ...)
  Returns true if in range - otherwise no data.
  NB! works correct if format is YUV420 only!  */
  bool getQuadPix(unsigned int line, unsigned int column,
                  unsigned char * I11, // top left
                  unsigned char * I12, // bottom left
                  unsigned char * I21, // top right
                  unsigned char * I22, // botom right
                  unsigned char * U,
                  unsigned char * V);
  /**
  Get line array for intensity (full resolution)
  Returns NULL if out of range.
  Returns pointer to array intensity bytes lpY[width].
  NB! works correct if format is YUV420 only!  */
  unsigned char * getYline(unsigned int line);
  /**
  Get line array for U-intensity at half resolution.
  line has range 0 to (height / 2 - 1).
  Returns NULL if out of range.
  Returns pointer to array of bytes lpU[line * width / 2].
  NB! works correct if format is YUV420 only!  */
  unsigned char * getUline(unsigned int line);
  /**
  Get line array for V-intensity at half resolution.
  line has range 0 to (height / 2 - 1).
  Returns NULL if out of range.
  Returns pointer to array of bytes lpV[line * width / 2].
  NB! works correct if format is YUV420 only!  */
  unsigned char * getVline(unsigned int line);
  /**
  Image is updated, and if any push commands is associated, then these
  are executed */
  virtual void imgUpdated();
  /**
  Get image update time.
  The image update time is set by a call to 'imgUpdate' and
  is always in current processor time - replay or not. */
  inline UTime getUpdatedTime()
  { return imgUpdateTime; };
  /**
  Get image timestamp time (when the image was taken).
  The image update time is set when image was first captured. */
  inline UTime getImageTime()
  { return imgTime; };
  /**
  Downscale image to half size, if
  image is YUV420 or one of the bayer formats, then the resulting image is in BGR format.
  Does nothing if format is YUV.
  \returns true */
  bool toHalf();
  /**
   * Paint simple grid aligned with x and y axis.
   * The origin is at (r0,c0) row and column number, a tick every meter
   * and stronger ticks regularly.
   * Origin lines are in blue, simple ticks week gray and strong tics a bit darker.
   * Designed for white background.
   * \param r0,c0 the grid origin in pixels
   * \param pixPerMeter number of pixels per meter (one tick per meter)
   * \param strongTicEvery strong tick interval (e.g. 10 for every 10 meter)
   */
  void paintGridAligned(const int r0, const int c0, const double pixPerM,
                                const int strongTicEvery);
  /**
  Make this image a rainbow RGB image from the BW image provided.
  0 towards 255 in gray becomes white->red->green->mag->dark->black.
  \param gray the gray sourceimage.
  \returns true if success. */
  bool toRaibow(UImage * gray);
  /**
  Make this image a rainbow RGB image from the BW image provided.
  255 towards 0 in gray becomes white->red->green->mag->dark->black.
  \param gray the gray sourceimage.
  \returns true if success. */
  bool toRaibowInv(UImage * gray);
  /**
  save a BW16S format as RGB - no data loss */
  void moveBW16ToRGB(UImage * dest);
  /**
  Get address of convert buffer pointer - for fixed allocated convert buffer space.
  The buffer is an integrated part of the UImage structure, and needs to be allocated,
  when first used. It will be deallocated when image is deallocated (deleted).
  \returns adress of convert buffer pointer inside image structure. */
  UImage ** getConvertBuffer()
  { return &convertBuffer; };


protected:
  /**
  Convert image format YUV420 (raw from webcam) to one
  of the other formats
  \param toColFormat is the new color format.
  \param dest is the destination image, may be same image (and is, if dest== NULL).
  \returns true if OK
  */
  bool moveYUV420To(int toColFormat, UImage * dest);
  /**
  Convert image format RGGB Bayer (raw from ieee1394 cam) to one
  of the other formats
  \param toColFormat is the new color format.
  \param dest is the destination image, may be same image (and is, if dest== NULL).
  \returns true if OK */
  bool moveBGGRTo(int toColFormat, UImage * dest);
  /**
  Move BGGR to BW - using the green values only, and narest neighbour.
  \param dest is the destination image, may be same image (and is, if dest== NULL).
  \returns true */
  bool moveBGGRtoBW(UImage * dest);
  /**
  Move BGGR to RGB (24 bit) using openCV.
  \param dest is the destination image, may be same image (and is, if dest== NULL).
  \returns true if image buffer has space for BGR image
   -as this is 3 times the size of the original image. */
  bool moveBGGRtoRGB(UImage * dest);
  /**
  Move BGGR to BGR (24 bit) - using openCV.
  \param dest is the destination image, may be same image (and is, if dest== NULL).
  \returns true if image buffer has space for BGR image
   -as this is 3 times the size of the original image. */
  bool moveBGGRtoBGR(UImage * dest);
  /**
  Downsample TYV420 coded image to half size in BGR format */
  void moveYUV420ToHalf();
  /**
  Downsample Bayer BGGR image to half size */
  void moveBGGRToHalfBGR();
  /**
  Downsample BW image to half size */
  void moveBWToHalf();
  /**
  Downsample any UPixel based image (RGB, BGR, YUV) to half size */
  void movePixToHalf();

public:
  /**
   * Move a source YUV422 image buffer of known size to an BW image.
   * \param source is a buffer with the source values
   * \param h is the source image height in rows
   * \param w is source image width
   * \returns true if successfull */
  bool yuv422toBW(uint8_t * source, int h, int w);
  /**
   * Move a source YUV422 image buffer of known size to an YUV image.
   * NB source buffer must not be the same as the image buffer in this image
   * \param source is a buffer with the source values
   * \param h is the source image height in rows
   * \param w is source image width
   * \returns true if successfull */
  bool yuv422toYUV(uint8_t * source, int h, int w);
  /**
   * Move a source YUV422 image buffer of known size to an YUV image in half resolution.
   * NB source buffer must not be the same as the image buffer in this image
   * \param source is a buffer with the source values
   * \param h is the source image height in rows
   * \param w is source image width
   * \returns true if successfull */
  bool yuv422toYUVhalf(uint8_t * source, int h, int w);
  /**
   * Move a source YUV422 image buffer of known size to an RGB image.
   * NB source buffer must not be the same as the image buffer in this image
   * \param source is a buffer with the source values
   * \param h is the source image height in rows
   * \param w is source image width
   * \returns true if successfull */
  bool yuv422toRGB(uint8_t * source, int h, int w);
  /**
   * Move a source YUV422 image buffer of known size to an BGR image.
   * NB source buffer must not be the same as the image buffer in this image
   * \param source is a buffer with the source values
   * \param h is the source image height in rows
   * \param w is source image width
   * \returns true if successfull */
  bool yuv422toBGR(uint8_t * source, int h, int w);
  /**
   * Move a source YUV422 image buffer of known size to an RGB image in half resolution.
   * \param source is a buffer with the source values
   * \param h is the source image height in rows
   * \param w is source image width
   * \returns true if successfull */
  bool yuv422toRGBhalf(uint8_t * source, int h, int w);
  /**
   * Move a source YUV422 image buffer of known size to an BGR image in half resolution.
   * \param source is a buffer with the source values
   * \param h is the source image height in rows
   * \param w is source image width
   * \returns true if successfull */
  bool yuv422toBGRhalf(uint8_t * source, int h, int w);

  
protected:
  /**
  Image structure from opencv library */
  IplImage img;
  /**
  Size of allocated buffer to img.imageBuffer */
  unsigned int bufferBytes;
  /**
  flag for color format. possible formats are
  PIX_PLANES_BW ,
  PIX_PLANES_RGB,
  PIX_PLANES_BGR,
  PIX_PLANES_YUV.
  PIX_PLANES_YUV420, 
  ... */
  int colFormat;
  /**
  first value of Y-pixels (YUV 420 planar only) */
  unsigned char * py; //
  /**
  first value of U-pixels (YUV 420 planar only) */
  unsigned char * pu; //
  /**
  first value of V pixels (YUV 420 planar only) */
  unsigned char * pv; //
  /**
  additional timestamp for other purposes than when image was taken */
  UTime imgUpdateTime;

public:
  /**
  Image is usable (buffers valid) */
  bool valid;
  /**
  Flag used by some processes to avoid
  transmitting an image twice.
  Is set to 0 by a imgUpdated() call, and by a image create or copy operation. */
  unsigned int used;
  /**
  Source number - used for processed
  images only (default value is image pool number).
  Value is not handled by copy functions. */
  int source;
  /**
  Flag to avoid that radial error is removed twice. */
  bool radialErrorRemoved;
  /**
  Estimated time image were taken */
  UTime imgTime;
  /**
  Pointer to camera that took the image. */
  void * cam;
  /**
  Image name (base name - if any) */
  char name[MAX_IMG_NAME_SIZE];
  /**
  Image saved using this file type e.g. png or bmp etc. */
  char saveType[MAX_EXT_SIZE];
  /**
  Position of robot (or camera) at capture time. */
  UPosition pos;
  /**
  Orientation of robot (or camera) at image capture time */
  URotation rot;
  /**
  Image serial number - from this camera */
  unsigned long imageNumber;
  /**
  Camera device number */
  int camDevice;
  /**
  High dynamic range set number.
  0 is a normal image not part of a set.
  1 is a dark reference image (minimum shutter time)
  2.. is images with increasing shutter time and/or gain,
  so hdrSet 2 should not be saturated and hdrSet = 3 shuld
  be usabel in saddows. */
  unsigned int hdrSet;
  /**
  Read delay is an indication of the validity of the image
  timestamp. If the delay is wery short (< 5ms) it is
  likely that the image were on stock before the timestamping
  and therefore older than could be expected from the
  timestamp. */
  float readDelay;

private:
  /** private format conversion buffer - created if needed */
  UImage * convertBuffer;
};


#endif
