/***************************************************************************
 *   Copyright (C) 2011 by DTU (Kristian Villien)                          *
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
 *
 * $Date: 2011-07-04 17:03:31 +0200 (Mon, 04 Jul 2011) $
 * $Id: ufunctof.h 1579 2011-07-04 15:03:31Z jca $
 ***************************************************************************/

#ifndef UFUNC_AUTOF_H
#define UFUNC_AUTOF_H

#include <stdint.h>

#include <ugen4/uimg3dpoint.h>
#include <urob4/ufuncplugbase.h>
#include <ucam4/uimagelog.h>

#include <ugen4/ulock.h> // include the TCP/IP port thingy


// time stamp structure used in image header
struct HeaderTimeStamp
{
float Seconds;
float Useconds;
};
//image header structure
struct ImageHeaderInformation {
/** @brief Imagedata size in Bytes without header */
float DataSize;
/** @brief Size of the header */
float HeaderSize;
/** @brief type of image cf. IMAGE_HEADER::ImageTypes */
float ImageType;
/** @brief consecutive version number */
float Version;
/** @brief single or double integration */
float SamplingMode;
/** @brief illu status 0,1,2,3 bit coded */
float IlluMode;
/** @brief frequency mode cf. ModulationFrequency */
float FrequencyMode;
/** @brief unambiguous range of current frequency */
float UnambiguousRange;
/** @brief time needed by image evaluation [ms] */
float EvaluationTime;
/** @brief first integration time single sampling mode [ms] */
float IntegrationTime_Exp0;
/** @brief second integration time double sampling mode [ms] */
float IntegrationTime_Exp1;
/** @brief timestamp */
HeaderTimeStamp TimeStamp;
/** @brief median filter status */
float MedianFilter;
/** @brief mean filter status */
float MeanFilter;
float internal_a[4];
//float internal_a2;
//float internal_a3;
//float internal_a4;
/** @brief displays if image is valid or not */
float ValidImage;
float ErrorCode;
float internal_b[3];
//float internal_b2;
//float internal_b3;
/** @brief configured trigger mode */
float CurrentTriggerMode;
float internal_c[4];
//float internal_c2;
//float internal_c3;
//float internal_c4;
/** @brief Inter Frame Mute time*/
float IfmTime;
float internal_d[64];
/*float internal_d2;
float internal_d3;
float internal_d4;
float internal_d5;
float internal_d6;
float internal_d7;
float internal_d8;
float internal_d9;
float internal_d10;
float internal_d11;
float internal_d12;
float internal_d13;
float internal_d14;
float internal_d15;
float internal_d16;
float internal_d17;
float internal_d18;
float internal_d19;
float internal_d20;
float internal_d21;
float internal_d22;
float internal_d23;
float internal_d24;
float internal_d25;
float internal_d26;
float internal_d27;
float internal_d28;
float internal_d29;
float internal_d30;
float internal_d31;
float internal_d32;
float internal_d33;
float internal_d34;
float internal_d35;
float internal_d36;
float internal_d37;
float internal_d38;
float internal_d39;
float internal_d40;
float internal_d41;
float internal_d42;
float internal_d43;
float internal_d44;
float internal_d45;
float internal_d46;
float internal_d47;
float internal_d48;
float internal_d49;
float internal_d50;
float internal_d51;
float internal_d52;
float internal_d53;
float internal_d54;
float internal_d55;
float internal_d56;
float internal_d57;
float internal_d58;
float internal_d59;
float internal_d60;
float internal_d61;
float internal_d62;
float internal_d63;
float internal_d64;*/
/*picture data*/
float Data[50*64];
};

// the possible image types
enum ImageTypes
{
INVALID_IMAGE = 0,
DISTANCE_IMAGE,
INTERNAL_DATA_A,
AMPLITUDE_IMAGE,
INTERNAL_DATA_B,
NORMAL_X_IMAGE,
NORMAL_Y_IMAGE,
NORMAL_Z_IMAGE,
KARTESIAN_X_IMAGE,
KARTESIAN_Y_IMAGE,
KARTESIAN_Z_IMAGE,
INTERNAL_DATA_C,
SEGMENTATION_IMAGE
};

//the different commands







/**
 * This is a simple example plugin, that manipulates global variables, both of its own, and those of other modules.
@author Christian Andersen
*/
class UFuncTOF : public UFuncPlugBasePush
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncKinect) followed by
  // a descriptive extension for this specific plug-in
public:
  /**
  Constructor */
  UFuncTOF()
  { // command list and version text
    setCommand("tof", "tof", "3D Time Of Flight camera");
    init();
  }
  /**
  Destructor */
  ~UFuncTOF();
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
  Initialize other variables */
  void init();
  /**
  produce disparity image.
  \returns true if successful - i.e. source images available*/
  bool processImages(ImageHeaderInformation *Image);
  /**
   * check if setting is up to data, if not, update it
   */
  bool processSetting(void);
  /**
   * reads all the settings from the camera
   */
  bool getAllSetting(void);  
  /**
   * call XML-RPC command and related functions
   */
  bool callXMLRPC(int command);
  bool ParseResponse(char* Response, char* Result);
  bool GenerateRequest(int type,char* Request);
  char* GetNextValue(char* reply);
  bool WaitForReply(char* Reply,int timeout, int limit);
  /**
  Handle stereo-push commands */
  bool handleTOF(UServerInMsg * msg);
  /// start read thread
  bool start();
  /** stop read thread
  \param andWait waits until thread is terminated, when false, then the call returns
  when the stop flag is set (i.e. immidiately). */
  void stop(bool andWait);


  /**
  Open or close camera stream */

public:
  /**
  Run receive thread */
  void runSetting();
  void runData();
private:
// variables used for settings
  char CameraName[15];
  char CameraVersion[8];
  int ModulationFrequencySetting;
  int DoubleSampling;
  int IntegrationTime1;
  int IntegrationTime2;
  int FrameMuteTime;
// free running
  int FreeRunning;
  int FreeRunningValue;
// image filters
  int NumberOfImagesAverage;
  int MeanFilter;
  int MedianFilter;

// variables used to pass internal data.
  volatile bool TriggerImage;
  volatile int ImagesMissing;

//time stamp of last image
  HeaderTimeStamp LastTime;
  UTime CurrentImageTime;
  UTime ImageRequestTime;
  /**
  Pointers to "own" global variables. */
  UVariable * varIsOpen;
// get images U-variables
  UVariable * varGetDistance;
  UVariable * varGetIntensity;
  UVariable * varGetNormalX;
  UVariable * varGetNormalY;
  UVariable * varGetNormalZ;
  UVariable * varGetKartesianX;
  UVariable * varGetKartesianY;
  UVariable * varGetKartesianZ;
//setting U-variables
  UVariable * varModulationFrequency;
  UVariable * varDoubleSampling;
  UVariable * varIntegrationTime1;
  UVariable * varIntegrationTime2;
  UVariable * varFrameMuteTime;
  UVariable * varFreeRunning;
  UVariable * varNumberOfImagesAverage;
  UVariable * varMeanFilter;
  UVariable * varMedianFilter;

  //image variables
  UVariable * varCamDeviceNum;
  UVariable * varFramerate;
  UVariable * varUpdateCnt;
  //debug
  UVariable * varExportData;
  
  /**
  Stereo processing parameters */
  /*UVariable * varImagesC3D;
  UVariable * varCamDeviceNum;
  UVariable * varDebug;
  UVariable * varIsOpen;
  UVariable * varFramerate;
  UVariable * varUseEveryCol;
  UVariable * varUseEveryDep;
  UVariable * varImgIR;
  UVariable * varTiltDeg;
  UVariable * varLed;
  UVariable * varAcc;
  UVariable * varAccRate;
  UVariable * varAccLogN;
  UVariable * varImageLogN;
  UVariable * varAccInterval;
  UVariable * varUseColor;
  UVariable * varUseDepth;
  */

  /// thread runnung flag
  bool threadRunningSetting;
  bool threadRunningData;
  /// stop thread flag
  bool threadStop;

  /**
  Thread handle for frame read thread. */
  pthread_t threadHandleData;
  pthread_t threadHandleSetting;

  /*network ports*/
  UClientPort DataPort;
  UClientPort SettingPort;
//  UClientPort DataPort;
//  UClientPort SettingPort;
};


#endif

