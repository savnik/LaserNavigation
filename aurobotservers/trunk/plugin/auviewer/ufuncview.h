/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#ifndef UFUNC_AUVIEW_H
#define UFUNC_AUVIEW_H

#include <urob4/usmltag.h>
#include <cstdlib>
#include <urob4/ufuncplugbase.h>
#include "unavview.h"

///////////////////////////////////////////////////

void * startUFuncViewThread(void * obj);

/**
 * 3D viewer - from PCL library
@author Christian Andersen
*/
class UFuncView : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncView()
  {
    setCommand("view", "view", "PCL viewer (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    createBaseVar();
    start();
  };
  /**
  Destructor - to delete the resource (etc) when finished */
  virtual ~UFuncView();
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
   * Start viewer thread */
  bool start();
  /**
   * Stop viewer thread */
  void stop(bool andWait);
  /**
  Set ressource as needed (probably not used by this resource) */
  bool setResource(UResBase * resource, bool remove);
  /**
  The varPool has methods, and a call to one of these are needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                                char ** strings, const double * pars,
                                double * value,
                                UDataBase ** returnStruct,
                                int * returnStructCnt);

  /**
   * Create viewer */
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

public:
  /**
   * thread that updated the view */
  void run();

protected:
  /**
   * The viewer */
  pcl::visualization::PCLVisualizer * viewer;
  /**
  Set position of robot on display - in meters from bottom position
  \param value is a string with pose values as a string, p.t.
  the first cvalue is used only. */
  void setRobotPose(char * value);
  /**
  Set the paint bold flag
  \param bold true or false */
  void paintBold(bool bold);
  /**
  Set number of range rings */
  void setRangeRingCnt(int value);
  /**
  Set robot type.
  \param robname is the type name of the robot - smr, mmr or hako */
  void setRobot(const char * robname);

protected:
  /**
  Pointer to a pose history module with current robot position */
  UResPoseHist * poseHist;
  /**
  Pointer to a pose history module with current robot position in GPS (UTM) coordinates */
  UResPoseHist * poseUtm;
  /**
  Pointer to a pose history module with current robot position in map coordinates */
  UResPoseHist * poseMap;
  /**
  Pointer to a (root) var-pool with access to all sub-structures */
  UResVarPool * varRoot;
  /**
  Pointer to image pool resource */
  UImagePool * imgPool;
  /**
  Time of newest data update of image in image pool */
  UTime newDataImgPoolAt;
  /**
  New data received to image pool */
  bool newDataImgPool;
  /**
  Time of newest data update of data for laser plane display */
  UTime newDataLaserAt;
  /**
  New data received - laser data */
  bool newDataLaser;
  /**
  Time of newest data update of data for laser plane display */
  UTime newDataNavAt;
  /**
  New data received - navigation data */
  bool newDataNav;
  /**
  Index to variable with thread running value */
  UVariable * varRunning;
  /// should coordinate systems be alligned at robot always
  UVariable * varAutoHereNow;
  /// paint bold
  UVariable * varBold;
  /// number of range rings to paint (each 1m)
  UVariable * varRangeRings;
  /// Default position of view camera
  UVariable * varResetViewPos;
  /// Default focus position (when reset)
  UVariable * varResetViewFocus;
  /// reset to default view camera position
  UVariable * varResetView;
  /// pan automatically [0]=1 active, [1]=pan angle (degrees), [2]=deg/sec
  UVariable * varAutoPanView;
  /// PCL viewer spin time value [1..100] approximately in ms - if available CPU power
  UVariable * varPCLspinTime;
  /// try to make view follow robot
  UVariable * varFollowRobot;
  /**
   * allowed number of highgui displayed images */
  static const int UPD_LIST_MAX_CNT = 100;
  /**
  Number of used entries in the image update list */
  int updListCnt;
  /**
  Display sync lock */
  ULock dispSync;
  /**
  Painter for navigation image */
  UNavView navPaint;
  /**
  Flag for changing scale on nav display */
  bool globalNavRedisplay;
  /**
   * Reset view - look at the robot from behind */
  bool resetView;

  //
private:
  /// pointer to limiting red values redMin, redMax, greenMin, greenMax
  UVariable * varPoolImg;
  /// thread runnung flag
  bool threadRunning;
  /// stop thread flag
  bool threadStop;
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
   * save viewed image as png-file if true */
  bool saveAsPng;
  /**
   * save as PNG file using this name */
  char saveAsPngName[MAX_FILENAME_SIZE];
};

#endif
