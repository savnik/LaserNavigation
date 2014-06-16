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
#ifndef UFUNC_BALL_FINDER_H
#define UFUNC_BALL_FINDER_H

#include <cstdlib>

#include <ucam4/ufunctioncambase.h>
#include <urob4/uresposehist.h>
#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <umap4/uposev.h>
//#include <iau_mat.h>
//#include <smartmat.hpp>


#include <LEL_commons.h>
#include <LEL_hough.h>

#include <list>
#include <vector>

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <mhf/MultiHypDist.hpp>

const int UIMGPOOL_NAVVISION_BASE = 20;
const int MRS = 1000000; // Maximum image report size
const int MNI = 1000; // Maximum number of images

const float varAlpha = 0.0001;
const float varR = 4;

class worldLine3d;
class lineRange;

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Example plugin that demonstrates a plogin that provides a resource.
A similar example plugin is available that uses the shared resource.

The shared resource provides the simple functionality in the form of a line.
The resource provides functions to calculate the length of the line.


@author Christian Andersen
*/
class UFuncCamLocalize : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncCamLocalize();
  /**
  Destructor */
  virtual ~UFuncCamLocalize();
  /**
  Called by the server core. Should return the
  name of function. There should be a first short part separated
  by a space to some additional info (e.g. version and author).
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name();
  /**
  Called by the server core when loaded, to get a list of
  keywords (commands) handled by this plugin.
  Return a list of handled functions in
  one string separated by a space.
  e.g. return "ball".
  The functions should be unique on the server. */
  virtual const char * commandList();
  /**
  List (space separated) of shared resources
  provided by this function.
  Must be an empty string if no resources are to be shared.
  Each resource ID must be no longer than 20 characters long. */
  virtual const char * resourceList()
  { return "odoPose"; }
  /**
  Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  This function is called by the server core, when it needs a
  resource provided by this plugin.
  Return a pointer to a resource with an ID taht matches this 'resID' ID string.
  The string match should be case sensitive.
  Returns false if the resource faled to be created (e.g. no memory space). */
  //UResBase * getResource(const char * resID);
  /**
  return true if all ressources is available */
  //virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  print status to a string buffer */
//  virtual const char * print(const char * preString, char * buff, int buffCnt);

protected:

  UResPoseHist * poseHist;
  UTime lastScanTime;
  int poseIndex;


  Matrix<double,3,1> pose;
  Matrix<double,3,3> poseCov;
  MultiHypDist<3> poseDist;
  SplitTable<1> table;

  std::vector<worldLine3d> worldLines;

  // Matrices

//  /// number of points that must be inside line segment to correlate
//  UVariable * varPointsInThreshold;
//  /// value of current covariance matrix
//  UVariable * varCovar;
//  /// eigenvectors of x,y part of current covariance matrix
//  UVariable * varCovarV;
//  /// eigenvalues of x,y part of current covariance matrix
//  UVariable * varCovarA;
//  /// number of successful match updates
//  UVariable * varUpdates;
//  /// number of failed matches since last successful match
//  UVariable * varFailed;
//  /// number of defined lines in localizer
//  UVariable * varDefinedLines;
//  /// robot base (differential drive?)
  UVariable * varOdoB; // = 0.26;
  /// distance varaince for right wheel each moved meter
  UVariable * varKR; // = 0.0003;
  /// distance varaince for left wheel each moved meter
  UVariable * varKL; // = 0.0003;
  /// The camera tilt, positive means camera looking up
  UVariable * varCameraTilt; // = 0.0003;
  /// The x coordinate of the camera in the robot frame
  UVariable * varCameraX; // = 0.0003;
  /// The y coordinate of the camera in the robot frame
  UVariable * varCameraY; // = 0.0003;
  /// The z coordinate of the camera in the robot frame
  UVariable * varCameraZ; // = 0.0003;
  /// The x pixel coordinate of the image center
  UVariable * varCx; // = 0.0003;
  /// The y pixel coordinate of the image center
  UVariable * varCy; // = 0.0003;
  /// The camera focal distance
  UVariable * varFp; // = 0.0003;
  /// Minimum image line strength
  UVariable * varLineStr;
  /// Default silence level
  UVariable * varDefaultSilence;

  UVariable * varMaxXYVariance; // = 0.05;
  UVariable * varMaxThVariance; // = 0.05;

private:
  /**
  Function to handle hough transform calls */
  bool handleLocalize(UServerInMsg * msg, UImage* pushImg);
  bool handleAddLine(UServerInMsg * msg);
  bool handleResetLM(UServerInMsg * msg);
  bool handleSetInitCov(UServerInMsg * msg);
  bool handleSetInitPose(UServerInMsg * msg);
  bool handleOutputDist(UServerInMsg * msg);
  bool handleSetTable(UServerInMsg * msg);

  bool handleResetLocalizer();
  void freeLines();
  Matrix<double,2,1> measerreq(Matrix<double,3,1> state, Matrix<double,2,1> noise, Matrix<double,2,1> meas, Matrix<double,3,1> auxin);
//  void projectToImage(Matrix<double,3,1> linePoint, Matrix<double,3,1> lineVec, matrix * pose, matrix * poseCov, LEL_ARLine &projLine, matrix * lineCov, matrix * delH_delP,matrix * delH_delPTrans);
  Matrix<double,2,1> projectToImage( Matrix<double,3,1> pose, worldLine3d line);
  Matrix<double,2,1> projectionError( Matrix<double,3,1> pose, Matrix<double,2,1> noise, Matrix<double,2,1> meas, worldLine3d line);
  lineRange lineInImage(Matrix<double,3,1> pose, worldLine3d worldLine);

  void createBaseVar();
};

class worldLine3d {
public:
	Matrix<double,3,1> SPoint;
	Matrix<double,3,1> EPoint;
	Matrix<double,3,1> Vec;
	char * name;

	worldLine3d(Matrix<double,3,1> lineSPoint, Matrix<double,3,1> lineEPoint, Matrix<double,3,1> lineVec, char * name): SPoint(lineSPoint), EPoint(lineEPoint), Vec(lineVec), name(name){}
};

class lineRange {
public:
	double lb;
	double le;
	lineRange(double lb, double le): lb(lb), le(le){};
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/** called when server makes a dlopen() (loads plugin into memory) */
void libraryOpen();

/** called when server makes a dlclose() (unloads plugin from memory) */
void libraryClose();

/**
Needed for correct loading and linking of library */
void __attribute__ ((constructor)) libraryOpen(void);
void __attribute__ ((destructor)) libraryClose(void);
/**
Allows server to create object(s) of this class */
extern "C" UFunctionBase * createFunc();
/**
... and to destroy the created object(s) */
extern "C" void deleteFunc(UFunctionBase * p);



#endif

