/** *************************************************************************
 *   Copyright (C) 2007-2008 by DTU (Christian Andersen)
 *   rse@elektro.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef URES_SVS_H
#define URES_SVS_H

#include <cstdlib>

#include <ugen4/uposrot.h>
#include <ugen4/uimg3dpoint.h>

#include <urob4/uresvarpool.h>
#include <urob4/uimagepool.h>
#include <urob4/ulogfile.h>

#include <svs/svsclass.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Class with 3D point found by stereo calculation */
class USvs3Dpoints : public UImg3Dpoints
{
public:
    /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { // not needed an no data is added, so should be
    // fully compatible with 'img3d' type
    return "svs3d";
  };
  /**
   * Add a 3d point to the point list
   * \param r is the image row number (0..239 typically)
   * \param c image column (0..319 typically)
   * \param w is the image width in pixels
   * \param src3d pointer to a valid 3D point (in svs coordinates)
   * \param img pointer to the image pixel array (RGB colour coded into 32 bit integer) */
  void add3d(int r, int c, int w, svs3Dpoint * src3d, unsigned long * img)
  {
    UImg3Dpoint p;
    p.pos.set(src3d->Z, -src3d->X, -src3d->Y);
    p.row = r;
    p.column = c;
    if (img != NULL)
      p.pixLeft = img[r * w + c];
    add(&p);
  };
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Class that holds the proparitary image set from Videre Design an in addition
 * a timestamp and serial number */
class USvsImageSet : public ULock, public UDataBase
{
public:
  /**
   * Constructor */
  USvsImageSet()
  {
    si = NULL;
    serial = 0;
  }

public:
  /**
   * Newest stereo image set - should not be used unless the
   * dataSiLock is locked. */
  svsStereoImage * si;  // this holds the current frame set
  /**
   * Detection time for this set of images */
  UTime imageTime;
  /**
   * Image serial number */
  unsigned int serial;
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

/**
 * Stereo image capture and disparity processing class.
 * The class is build on the SVS library from Videre design.
 * A thread is running requesting new images at the rate available from the camera
 * At every new image set the set is offered for potential svsPush commands. or
 * for manual commands.
 * A number of parameters are available as global variables.
*/
class UResSVS : public UResVarPool, public UServerPush
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLine) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResSVS();
  /**
  Destructor */
  virtual ~UResSVS();
  /**
  print status to a string buffer */
  //virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
   * Create any resources that this modules needs
   * This method is called after the resource is registred by the server. */
  virtual void createResources();

  /**
  \brief open or close fixed logfile
  \param close closes the logfile if false, else open */
  void openLogs(bool doOpen)
  {
    logImg.openLog(doOpen);
    logHoz.openLog(doOpen);
    logFran.openLog(doOpen);
  };
  /**
  \brief ask if logfile is open
  \returns true if open */
  inline bool isImgLogOpen()
  { return logImg.isLogOpen(); };
  /**
  \brief ask if horizon logfile is open
  \returns true if open */
  inline bool isHozLogOpen()
  { return logHoz.isLogOpen(); };
  /**
  \brief ask if exposure control logfile is open
  \returns true if open */
  inline bool isExposureLogOpen()
  { return logFran.isLogOpen(); };
  /**
  \do some stereo action */
  //bool doStereoAction(bool doPoolImages);
  /**
   * \brief Make stereo disparity calculation on this image set
   * The disparity filter settings are taken from global variables
   * \Returns true if successful */
  bool doDisparityCalculation(USvsImageSet * si, bool doPoolImages);
  /**
   * Set reference time for image timestamp */
  void setImageTimeRef(UTime ref)
  {
    printf("ref-time-diff %.1f msec\n", ref - timeRef);
    timeRef = ref; };
  /**
    * Run the receice loop for the stereo data source, to allow push.
    * This call do not return until the threadStop flag is set true. */
  void run();
  /**
   * Called by server core when it is time to execute push commands for this object.
   * The method will call 'gotNewData(void * pushData)' */
  virtual void callGotNewDataWithObject();
  /**
   * \breif Get a fresh image set to do some stereo calculation on */
  USvsImageSet * getImageSet(USvsImageSet * pushIsi, bool toImgPool, bool doReplay);
  ///  open camera devices and start streaming
  bool startStreaming();
  /**
   * Stop streaming (if started) and close device */
  void stopStreaming();
  /**
   * get name of logfile */
  const char * getImgLogName();
  /**
   * Get horizon line estimate logfile name */
  const char * getHozLogName();
  /**
   * Get exposure control logfile name */
  const char * getExposureLogName();
  /**
   * Write the 3D result to a (matlab) file */
  bool to3Dfile(USvsImageSet * si);
  /**
   * Set replay value */
  bool getReplay()
  { return replay; };
  /** get replay stream name */
  const char * getReplayImgBaseName()
  { return replayImgBaseName; };
  /** get current replay frame number */
  int getReplayFrame()
  { return replayFrameSerial; };
  /**
   * Set shutter in the range from -1 (auto) and 0 to 100 */
  void setShutter(int value);
  /**
   * Get shutter in the range from 0 to 100? */
  int getShutter();
  /**
   * Is shutter set to auto.
   * \returns true if auto setting */
  bool isShutterAuto();
  /**
   * Set shutter in the range from -1 (auto) and 0 to 100 */
  void setGain(int value);
  /**
   * Get shutter in the range from 0 to 100? */
  int getGain();
  /**
   * Is shutter set to auto.
   * \returns true if auto setting */
  bool isGainAuto();
  /**
   * Set calibration file */
  void setSvsCalibFile(const char * name)
  { strncpy(svsCalib, name, MAX_FILENAME_SIZE); };
  /**
   * Set imageset subdir (for replay only) */
  void setImageSetSubdir(const char * subdir)
  { strncpy(imgSetSubdir, subdir, MAX_FILENAME_SIZE); };
  /**
   * Get the filename of the calibration file */
  const char * getSvsCalibFile()
  { return svsCalib; };
  /**
   * Get imageset subdir (for replay only) */
  const char * getImageSetSubdir()
  { return imgSetSubdir; };
  /**
   * Replay one step in the logfile and update available image in file structure.
   * \returns false if no more steps area available in logfile */
  virtual bool replayStep();
  /**
   * 'Manually initiated' replay of N singles from the logfile.
   * This will trigger other replay resources to advance up to just before this time.
   * Returns true if N steps available. */
  bool replayStep(int steps);
  /**
  is replay file open */
  inline bool isReplayFileOpen()
  { return (replayFile != NULL); };
  /**
  Get current line number in replay logfile */
  inline int getReplayLogLine()
  { return replayLogLine; };
  /**
   * Set replay flag.
   * \param value is the new value for the replay flag.
   * If replay flag is changed to true, the logfile is attempted opened.
   * If successfull, the current pose history is cleared and
   * the first record read and ready to be implemented implemented (by a further step(1)).
   * If new value is false, then the replay file is closed (if not already).
   * The replay flag is set to the new value in any case.
   * \Returns true if replay file is in the state suggested by the value */
  bool setReplay(bool value);
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param strings is an array of string pointers for the string type parameters (may be NULL if not used)
   * \param doubles is an array with double typed parameters (may be NULL if not used)
   * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true
   *              (2.0 for implicit stop if a controll call from mission sequencer).
   * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);
  /**
   * Get sensor pose */
  UPosRot getSensorPose();
  /**
   * Get sensor pose */
  void setSensorPose(UPosRot * newPose);
  /**
   * Do some har coded calibration stuff */
//  bool doFranCode(int doJust);
  
protected:
  /**
  Start read thread
  \return Returns true if the read thread started. */
  bool start();
  /**
   * Stop read thread - and wait for thread join
   * \param andWait is ignored - always waiting */
  virtual void stop(bool andWait);
  /**
  Create global variables for this resiource */
  void createBaseVar();
  /// copy extractions parameters from global to extract function
  void getParams();
  /**
   * \brief Copy image size, format and pixels to image
   * \param img is the destination image (from image pool).
   * \param height is the image height
   * \param width  is the image width
   * \param color  is colour (bayer-coded)
   * \param source is the buffer with pixel values
   * \returns true if image is copied */
  bool toImagePool(UImage * img, int height, int width, int format,
                   const unsigned char * source);
  /**
   * \brief Get image set from camera hardware.
   * \param ssi pointer to resulting image set data structure
   * \returns true if image was captured. */
  bool getImageSetFromHardware(USvsImageSet * ssi);
  /**
   * \brief Get image set from replay - svs stream file.
   * The image tests the replayFrameNumber and the replay time, if
   * any of these are ahead of current frame, then a new image set
   * is loaded from the stream.
   * Frame number is used to single step based on images.
   * Time is set to image stream time if frame number triggered the read.
   * \param ssi pointer to resulting image set data structure
   * \param lastSerial is the last image serial number used
   * \returns true if images are available and loaded into structure. */
  bool getImageSetFromStream(USvsImageSet * ssi, unsigned int lastSerial);
  /**
   * \brief Get image set from image pool.
   * The image pool numbers are determined by 'useImgPoolSource' (left) and
   * 'useImgPoolSource+1' (right) image.
   * Time is set to image pool time.
   * \param ssi pointer to resulting image set data structure
   * \returns true if images are available and loaded into structure. */
  bool getImageSetFromImgPool(USvsImageSet * ssi);
  /**
   * Do fram exposure time control - calculate based on provided horizon line
   * \param ssi is the stereo image set to use
   * \param hozLine is the horizon line in the image (horizontal)
   * \param debugImage paint results into debug image in image pool
   * \returns the exposure value to be implemented */
  int doFranExposureControl(USvsImageSet * ssi, const int hozLine, bool debugImage);
  /**
   * Set value of shutter and video gain to global variables (read only) */
  void setGainAndShutterVar();
  /**
   * Estimate horizon line */
  //int doHorizonEstimate(USvsImageSet * ssi);
  /**
   * Estimate horizon line using blue over green as limit
   * \param ssi is the stereo image source structure
   * \param debugImage if true, then a sky filter image  (image 83) is generated
   * \returns estimated horizon line */
  int doHorizonEstimateBG(USvsImageSet * ssi, bool debugImage);
  /**
   * Estimate horizon line using blue another method (JCA invension) */
  //int doHorizonEstimate3(USvsImageSet * ssi);
  /**
   * Paint a histogram with baseline horizontal and elements extending up
   * \param img image to paint in
   * \param bin array of values
   * \param binCnt number of values in bin
   * \param binMax maximum value in histogram
   * \param baseline low value of histogram - in pixels
   * \param height height of histogram - in pixels
   * \param left left edge of histogram - in pixels
   * \param elemWidth width of histogram elements - in pixels
   * */
  void paintHistogramH(UImage * img, const int bin[], const int binCnt,
                       double binMax, int baseline, int height, int left, int elemWidth);
  /**
   * Paint a histogram with baseline to the left and element extending to the right
   * \param img image to paint in
   * \param bin array of values
   * \param binCnt number of values in bin
   * \param binMax maximum value in histogram
   * \param baseline low value of histogram - in pixels
   * \param height height of histogram - in pixels
   * \param top edge of histogram - in pixels
   * \param elemWidth width of histogram elements - in pixels
   * */
  void paintHistogramV(UImage * img, const int bin[], const int binCnt,
                       double binMax, int baseline, int height, int top, int elemWidth);
  /**
   * save current horizonvalues to this file - a debug function */
  void saveToHozFile(FILE * logfile, unsigned int serial);
  /**
   * save current horizonvalues to this file - a debug function */
  void saveToExpFile(FILE * logfile, unsigned int serial,
                     const int oldShutter,
                     const int newControl,
                     const double newGain,
                     const double errLinear,
                     const double errSaturated,
                     const double bin255,
                     const int controlMode);

public:
  /// Is streaming (iso mode) started
  bool isStreaming;
  /**
   * Should console messages be verbose */
  bool verbose;
  /**
   * intensity distribution below horizon */
  int bin[256];
  /**
   * Maximum intensity in image rows for horizon estimation */
  int binh[480]; // highest intensity value in image row

protected:
  /// logfile handle
  ULogFile logImg;
  /// logfile for intensity control
  ULogFile logFran;
  /// logfile for horizon estimate
  ULogFile logHoz;
  /// logfile for horizon estimate
  ULogFile logTime;
  /// path to calibration file
  char svsCalib[MAX_FILENAME_SIZE];
  /// stereo camera access object (svs library)
  svsVideoImages * videoObject; // for live video source
  /// stereo processing object (svs library)
  svsStereoProcess *processObject; // for computing disparity image
  /**
   * Read a stereo fileset from file set - structure */
  svsFileImages * fileObject;
  /**
   * stereo image set - possibly rectified for lens errors */
  USvsImageSet dataSi;
  /**
   * reference time for stereo images, this value
   * is to be added to the acqTime from image set to get
   * tetection time. */
  UTime timeRef;
  /// later adjustment of time ref for more accurate timestamp
  double timeRefAdj;
  /**
   * is real camera hardware to be used - else replay from stream or image pool. */
  bool useRealCamera;
  /**
   * use stream replay from svs-library format */
//  bool useReplay;
  /**
   * replay time, either set from a svs step command or set by others
   * expecting svs to replay images up to this time.  */
  UTime useReplayTime;
  /**
   * Take images from image-pool */
  int useImgPoolSource;
  /// replay stream filename (current image base filename)
  char replayImgBaseName[MAX_FILENAME_LENGTH];
  /// Image set subdir, used during replay only.
  char imgSetSubdir[MAX_FILENAME_LENGTH];
  /// replay frame number
  unsigned int replayFrameSerial;
  /// replay file handle
  FILE * replayFile;
  /**
  Replay file path - full path */
//  char replayPath[MAX_FILENAME_SIZE];
  /**
  Maximum length of a line in the logfile */
  static const int MAX_LOG_LINE_LENGTH = 200;
  /**
  Buffer for latest line (but not used) line from logfile */
  char replayLine[MAX_LOG_LINE_LENGTH];
  /**
  Replay time of last used data from logfile */
//  UTime replayTimeNow;
  /**
  Replay time of next data from in logfile (line buffer) */
//  UTime replayTimeNext;
  /**
  Current line number in logfile */
  int replayLogLine;
  /// current image number
  unsigned long serial;
  /// fran control source serial
  unsigned long serialFran;
  /// maximum delay of shutter
  static const int MAX_CTRL_DELAY = 6;
  /// valid control shutter for image
  int exposureCtrl[MAX_CTRL_DELAY];
  /// exposure calculation time
  double exposureCalcTime;
  /// 3D point cloud resulting from the process
  USvs3Dpoints * points3D;
  /// new value for gain
  double newGain;

  ///  Maximum disparity (determines minimum range)
  UVariable * varMaxDisp;
  /// index to Size of correlation window 5x5 to 21x21 - uneven
  UVariable * varCorrSize;
  /// index to Texture filter setting 0 to ???
  UVariable * varFiltTexture;
  /// index to Uniqueness filter setting
  UVariable * varFiltUnique;
  /// index to sprecle filter size (0 to ???)
  UVariable * varFiltSprec;
  /// index to sprecle filter difference to neighbor (0 to ???)
  UVariable * varFiltDiff;
  /// index to image number for left image
  UVariable * varImgLeft;
  /// index to image number for right image
  UVariable * varImgRight;
  /// index to image number for disparity image
  UVariable * varImgDisparity;
  /// index to image size 320 or 640
  UVariable * varWidth;
  /// index to image frame rate
  UVariable * varRate;
  /// index to image serial number
  UVariable * varImgSerial;
  /// index to current shutter value
  UVariable * varShutter;
  /// index to current gain value
  UVariable * varGain;
  /// index to delay from image shutter to driver timestamp
  UVariable * varDelay;
  /// index to additional delay from log timestamp - during replay
  UVariable * varDelayReplay;
  /// index to current Fran control value (shutter)
  UVariable * varFranExp;
  /// index to horizon value
  UVariable * varHorizon;
  /// number of bins that should be counted as saturation
  UVariable * varSaturationBins;
  /// index to fran reference intensity
  UVariable * varRefIntensity;
  /// index to fran reference intensity
  UVariable * varRefIntensityBand;
  /// index to gain for Fran intensity control
  UVariable * varFranGain;
  /// index to control delay from shutter setting to image reaction
  UVariable * varControlDelay;
  /// index to flag for automatic horizon estimate
  UVariable * varFranHoz;
  /// index to horizon line estimated by the Fran method (auto or not)
  UVariable * varHorizonEst;
  /// index to mean intensity below horizon
  UVariable * varMeanIntensity;
  /// index to 6D position of sensor
  UVariable * varPose6d;
  /// index to flag for intensity histogram display
  UVariable * varMakeHistogram;
  /// index to Green limit for horizon estimate
  UVariable * varHozGreenLimit;
  /// index to lowest allowed horizon limit, limit to this value
  UVariable * varHorizonLow;
  /// the the blus sky threshold as a percent
  UVariable * varHorizonBluePct;
  /// index to reference shutter value (for video gain controller)
  UVariable * varRefShutter;
  /// index to gain for video gain I-controller
  UVariable * varRefShutterIGain;
  /// index to maximum value for automatic gain control
  UVariable * varRefVideoGainMax;
private:
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
  bool threadRunning;
  /**
  Should thread stop - terminate */
  bool threadStop;

};

#endif

