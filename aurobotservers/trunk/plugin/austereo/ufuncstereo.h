/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/
#ifndef UFUNC_AUSTEREO_H
#define UFUNC_AUSTEREO_H

#include <ugen4/uimg3dpoint.h>
#include <urob4/ufuncplugbase.h>
//#include "pseudocolor.h"
#ifdef OPENCV2
#include <calib3d/calib3d.hpp>
#endif
/**
 * This stereo plugin takes two unprocessed (raw) images and produce a diaparity image and a 3D cloud, based on a calibration values and correlation parameters in the global variables.
@author Christian Andersen
*/
class UFuncStereo : public UFuncPlugBasePush
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncStereo) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncStereo()
  { // command list and version text
    setCommand("stereo stereoPush", "stereo", "Plug-in to implement openCV stereo");
    logf.setLogName("stereo");
//#if (CV_MAJOR_VERSION >= 1)
    BMState = NULL;
    varImagesLR = NULL;
    strncpy(whyString, "unknown", MWL);
//#endif
    imageSize.width = 0;
    imageSize.height = 0;
    mx1 = NULL;
    img1 = NULL;
    img2 = NULL;
  }
//#if (CV_MAJOR_VERSION >= 1)
  /**
  Destructor */
  ~UFuncStereo();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  Called from push implementor to get the push object.
  should call to 'gotNewData(object)' with the available event object.
  if no object is available (anymore), then call with a NULL pointer. */
  virtual void callGotNewDataWithObject();
  //
private:
  /**
  Reset global variables */
  //void resetVars();
  /**
  produce disparity image.
  \returns true if successful - i.e. source images available*/
  bool processImages();
  /**
  Initialize the needed stereo structures and parameters
  \param paramChanged marks that som parameter intrinsic, distortion or fundamental matrix has changed, and
  thus rectification values must be recalculated - otherwise the existing values are used. */
  void initializeStereo(bool paramChanged);
  /**
  Handle stereo-push commands */
  bool handleStereo(UServerInMsg * msg);

private:

  static const int MWL = 100;
  /**
  Short explanation to why processing failed */
  char whyString[MWL];
  /**
  Pointers to "own" global variables. */
  UVariable * varUpdateCnt;
  UVariable * varPoseOnRobot;
  UVariable * varTime;
  /**
  Stereo calibrate values */
  UVariable * varImageSize;
  UVariable * varIntrinsicLeft;
  UVariable * varDistortionLeft;
  UVariable * varIntrinsicRight;
  UVariable * varDistortionRight;
  UVariable * varFundamental;
  UVariable * varEssential;
  UVariable * varLRrotation;
  UVariable * varLRtranslate;
  /**
  Stereo processing parameters */
  UVariable * varImagesLR;
  UVariable * varImageDisp;
  UVariable * varDebug;
  UVariable * varPreFilterSize;
  UVariable * varPreFilterCap;
  UVariable * varSADWindowSize;
  UVariable * varMinDisparity;
  UVariable * varNumberOfDisparities;
  UVariable * varTextureThreshold;
  UVariable * varUniquenessRatio;
  UVariable * varTrySmallerWindow;
  //UVariable * varSpecleWindowSize;
  //UVariable * varSpecleRange;
  UVariable * varQ;

  /**
  Stereo calculation structures */
  CvStereoBMState * BMState;
//#endif
  /**
  Logfile */
  ULogFile logf;
  /**
  debug structure for colour disparity image */
  //Pseudocolor pscolor;
  /**
  values for image rectification and reallignment for image 1 */
  CvMat *mx1, *my1;
  CvMat *mx2, *my2;
  /**
  Image buffer for left and right image in BW - if not debug image */
  CvMat *img1rd, *img2rd; //, *disp16;
  UMatrix umatQ;
  /**
  Image buffer for disparity image - in 16bit gray format */
  //CvMat* disp;
  /** expected size of images */
  CvSize imageSize;
  /**
  Image buffers for source image in BW */
  UImage * img1;
  UImage * img2;
  /**
  Array with 3d points */
  UImg3Dpoints cloud3d;
};


#endif

