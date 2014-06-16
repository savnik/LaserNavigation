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
#ifndef UIMAGELOG_H
#define UIMAGELOG_H

#include <stdio.h>

#include <ugen4/ulock.h>
#include <ugen4/uimage.h>
#include <urob4/ulogfile.h>

class UCamMounted;

/**
Function to save and maintain an imagelogfile.
The file is to be used for simulated replay of MMRD including cameraserver.

@author Christian Andersen
*/
class UImageLog : public ULLogFile
{
public:
  /**
  Constructor */
  UImageLog();
  /**
  Destructor */
  ~UImageLog();
  /**
  Open logfile - and start logging */
  bool openLogging(const char * name);
  /**
  Close logfie - and stop logging */
  void closeLogging();
  /**
  Open (or close) logfile.
  An open will start a fresh empty file.
  \param doOpen if true opens a fresh logfile, if false closes the logfile (if open)
  \param returns true if logfile logfile operation is successful. */
  bool openLog(bool doOpen);
  /**
  Log this image - if logfile is defined and open.
  Image itself is not saved is 'saveImg'==false. 
  else image is saved in '.bmp' or '.png' format, as available.
  \param img is the image to be saved
  \param cam is the camera device - with camera pose.
  \returns true if image is saved (if requested) and image log entry is created. */
  bool logImage(UImage * img, UCamMounted * cam);
  /**
  Is logging open */
/*  inline bool isOpen()
  { return logi.isOpen(); };*/
  /**
  Get log filename */
/*  inline const char * getLogName()
  { return logi.getLogName(); };*/
  /**
  Get log filename */
/*  inline const char * getLogFileName()
  { return logi.getLogFileName(); };*/
  /**
  set log filename */
/*  inline void setLogName(const char * name)
  { logi.setLogName(name); };*/
  /**
  Is logging to save the image too? */
  inline bool isLogSaveImg()
  { return saveImg;};
  /**
  Set the save image too flag */
  inline void setSaveImg(bool value)
  { saveImg = value; };
  /**
  Set the image format to PNG (else BMP) */
  inline void setPng(bool value)
  { inPNG = value; };
  /**
  Set the image format to PNG (else BMP) */
  inline bool isPng()
  { return inPNG; };
  /**
  Save info tekst to log */
  void toLog(const char * txt);
  
private:
  /**
  Logfile for continued logging of images */
//  ULogFile logi;
  /**
  Buffer image for RGB conversion */
  //UImage * imgBuf;
  /**
  Save images in PNG format (if supported) */
  bool inPNG;
  /**
  Save image in addition to logging and making filename */
  bool saveImg;
  /**
  Filename length */
  static const int MFNL = MAX_FILENAME_LENGTH;
  /**
  Filename for image log */
  //char fn[MFNL];
};

#endif
