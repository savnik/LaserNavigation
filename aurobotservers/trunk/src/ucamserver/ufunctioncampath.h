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
#ifndef UFUNCTIONCAMPATH_H
#define UFUNCTIONCAMPATH_H

#include <ucam4/ufunctioncambase.h>

#include "uimageana.h"
#include "uimgproj.h"
#include "uprobgrid.h"

#define MAX_LIST_FILENAME_SIZE 100
/**
server function to extract free path from images
Version tag $Id: ufunctioncampath.h 1573 2011-07-02 16:20:48Z jca $
@author Christian Andersen
*/
class UFunctionCamPath : public UFunctionCamBase
{
public:
  /**
  Constructor */
  UFunctionCamPath();
  /**
  Destructor */
  virtual ~UFunctionCamPath();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
  // virtual const char * name();
  /**
  Returns list of handled functions */
  // virtual const char * commandList();
  /**
  Handle camera and image commands */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  Set logfile to use for path limit analyze */
  void setLog(FILE * logFd)
  { logPath = logFd; };
  
protected:
  /**
  Function to do the path analysis */
  bool handlePathGetCommand(UServerInMsg * msg, URawImage * imgBase);
  /**
  The find path function.
  Finds a polygon from a seed point,
  using an relative and an absolute paramaeter */
  bool findPath(UServerInMsg * msg, // reply info
                UImage * pushImg,  // image to analize
                UCamPush * cam,    // related camera
                UPose robPose,     // robot pose at image time
                bool push,        // send more info (and images)
                double lookingLeft, // in range 1.0 (full left) to -1.0 (full right)
                int x1, int y1,    // top-left of seed area for road
                int x2, int y2,   // bot right of seed area for road
                bool colorCorrect,
                bool testBalLim   // try different values around bal and lim values
               );
  /**
  Initialize needed structures */
  bool initializeFindPath();
  /**
  Get image name and decode device, time, camera, hdrMode */
  bool getImageName(int imgNumber,      // image number in listfile
               const char * subdir,     // subdir for images
               const char * imgFileLog, // filenaem of image file list
               char * imgFile,          // image filename
               int imgFileLng,          // image filename buffer length
               UTime * imgTime,         // image time (from image name)
               int * camDeviceNum,      // alternative to posName
               char * posName,           // alternative to camDeviceNum
               unsigned long * imageNumber // image number
               );
  /**
  Get robot pose at this time */
  bool getRobotPose(
               const char * subdir,     // subdir for images
               const char * odoFileLog, // filenaem of image file list
               UTime imgTime,         // image time (from image name)
               UPose * robPose
               );

private:
  /**
  Image analysis object */
  UImageAna * ana;
  /**
  Probability grid object */
  UProbGrid * grid;
  /**
  Projection object */
  UImgProj * imp;
  /**
  Storage used by openCV */
  CvMemStorage * cvStorage;
  /**
  is all objects initialized */
  bool initialized;
  /**
  Name of file with saved images for testing */
  char imgFileLog[MAX_LIST_FILENAME_SIZE];
  /**
  Name of file with logged odometry information
  in format: \n
  tomeofday              x        y       h \n
  1099404780.178155 -0.000921 0.000003 -0.004755 \n
  1099404780.238847 -0.001036 0.000003 -0.004756 \n
  1099404780.328159 -0.001094 0.000003 -0.004993 \n
   */
  //char odoFileLog[MAX_LIST_FILENAME_SIZE];
  /**
  Subdirectory relative to imagePath, where
  the logfiles and the images are stored. */
  char imgFileSubdir[MAX_LIST_FILENAME_SIZE];
  /**
  ballance between croma and edge -- default = 0.5.
  Ballance = 0.0 => croma criteria only.
  Ballance = 1.0 => edge alone. */
  double ballance; // 
  /**
  Absolute treshold for free path edge finding
  relative to seed point. */
  double limit; // = 40; // absolute edge size
  /**
  Last used image number from file, to allow
  use of push of next image number. */
  int imgFileNum;
  /**
  Logfile for path analyze */
  FILE * logPath;
};

#endif
