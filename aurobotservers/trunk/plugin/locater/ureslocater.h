/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.206.                              *
 *   Does laserbased localisation for a robot running in an orchard        *
 *   environment.                                                          *
 *   Edited by Peter Tjell (s032041) & Søren Hansen (s021751)              *
 * ----------------------------------------------------------------------- *
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
/**
@author Christian Andersen
@editor Peter Tjell / Søren Hansen
*/

#ifndef URESLOCATER_H
#define URESLOCATER_H

#include <cstdlib>
//# include <math.h>
# include <expat.h>

# include <ugen4/uline.h>
# include <umap4/upose.h>
# include <urob4/uresvarpool.h>
# include <ulms4/ulaserpool.h>
# include <auef/auextractfeatures.h>
# include <urob4/uresposehist.h>
# include <ugen4/utime.h>
# include <ugen4/ulock.h>
#include <ugen4/umatrix.h>
#include <urob4/ulogfile.h>

# include <uresmapbase.h>
# include <iau_mat.h>

# include "linreg.h"
# include "tools.h"

class UPolygon;

//# define LOG_LENGTH 100000     // Logarray size
//# define SCAN_POINTS 512     // Maximal number of points in a laserscan
# define OX 708000.0             // UTM offset in x direction
# define OY 6174000.0            // UTM offset in y direction
# define URESLOCATER_ID_OK (-13)    // ID of locater resource, when OK
# define URESLOCATER_ID_SET (-14)    // ID of locater resource, when just set
# define MAPLENGTH 500


//typedef struct map_type
//{
////    x12 struct array with fields:
//    double x[2];      /* Max and min x values */
//    double y[2];      /* Max and min y values */
//    double offset_x;  /* x offset */
//    double offset_y;  /* y offset */
//    char zone[4];     /* zone (not implemented) */
//    double A;         /* lineparameters A, B, C */
//    double B;
//    double C;
//};
/*
  // Help functions for XML load using expat
  void end(void *userData, const XML_Char *el);
  void start(void *userData, const XML_Char *el, const XML_Char **attr);
  void row_parse(const char **);
  void add_startpoint(const char **);
  void add_endpoint(const char **);
  void add_lineparams(const char **);
  void add_offset(const char **);
  void transport_parse(const char **);
*/
  // Internal function used in various calculations
  void extract_control(UPose, UPose, double, double *, double *);
  /**
   * Correlates laser scan points to each of the map lines in mapBase.
   * The result is delivered in the array parameters.
   * \param rd is the laserscan measurements
   * \param x_ matrix, that ???
   * \param cnt_array number of measurements associated to eaxh map point
   * \param x_array,y_array is the associated measurements to each map line
   * \param cnte_array number of measurements associated to eaxh map point
   * \param xe_array,ye_array is the associated measurements to each map line, assuming endless lines
   * \param ls_offset is forward position of laserscanner relative to robot origin (x-offset)
   * \param UResMapbase is a pointer to the map plugin with map line information */
/*  void match_points(RangeData rd, matrix *x_,
                    int *cnt_array,
                    double x_array[][POINTS_MAX],
                    double y_array[][POINTS_MAX],
                    int *cnte_array,
                    double xe_array[][POINTS_MAX],
                    double ye_array[][POINTS_MAX],
                    double ls_offset, UResMapbase *);*/
  //void mahalanobis_calc(matrix *x_, double x_array[][POINTS_MAX], double y_array[][POINTS_MAX], int *cnt_array, UResMapbase *);

/**
 * Class used by the locater to save one set of results from associations from one mapped line, */
class ULocaterLineResult
{
public:
  /// the expected distance from line to robot - signed and perpendicular to the line.
  double distMap;
  /// the measured signed distance to the robot - perpendicular to the line
  double distMeas;
  /// angle difference from mapped to measured angle
  double angleDelta;
  /// the related map line
  mapline * line;
  /// is the line end detection valid
  bool lineEndOK;
  /// offset from expected line end offset parallel with line.
  double lineEndDelta;
  /// form factor of associated detection cloud - along line
  double length;
  /// form factor of associated detection cloud - across line
  double width;
  /// number of measurements for this line
  int hitCnt;
};

/**
 * Resource to process lasterscans and detectable apriori mapped lines.
 */
class UResLocater : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLocater) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResLocater();
  /**
  Destructor */
  virtual ~UResLocater();
  /**
  The varPool has methods, and a call to one of these is needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed).
  * If the call is allowed, but the result is invalid for some reason, the
    return value 'value' can be set to som agreed value, (e.g. 0.0 (false) and 1.0 for true). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct = NULL,
                  int * returnStructCnt = NULL);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.


public: // Public functions
//  virtual bool setResource(UResBase * resource, bool remove);
  void retrieveData(ULaserData * data, RangeData *rd);

  /* Log handling */
  void openlog();
  void closelog();

  /* Prints variables */
  void varprint();

  /* EKF function */
  void locate(ULaserData * pushData, bool silent);

  /* Set parameters used in filter and navigation */
  void set_state(char *, double);
  void set_utmstate();
  void set_poselast();
  /**
   * Should module keep mapPose from localizer.
   * If keepMapPose=true, then locater updates sets the mapPose.
   * If keepMapPose=false then mapPose can be set bu the utmToMap locater function */
  bool keepMapPose()
  { return varKeepMapPose->getBool(); };
  /**
   * Set the map pose from utmPose - if mapPose is not maintained by locater update
   * \returns true if mapPose is updated by call. */
  bool setUtmPoseToMapPose();


public: // Public variables
  unsigned long serial;
  /** Flag indicating whether updates are allowed or not: */
  bool update_allow;

  /** Noise variables for control signals */
  double E_A, E_SA, E_L;

  /** Robot parameters */
  double ls_offset, axle_dist;

  /** Measurement noise intensities */
  double var_d, var_phi, var_t;

  /** Update limits - number of correlated points */
  int min_points;
  /** Update limits - angle limit */
  double max_angle_deviation;

  /** Path to mapfile */
  char *mapfile;

private:
  /** array of measurement points for each of the lines in the map - for line estimate */
  double x_array[MAPLENGTH][POINTS_MAX];
  double y_array[MAPLENGTH][POINTS_MAX];
  /** number of points for each map line */
  int cnt_array[MAPLENGTH];

  /** array of measurement points for each of the lines in the map - without line end test
   * for line end test */
  double xe_array[MAPLENGTH][POINTS_MAX];
  double ye_array[MAPLENGTH][POINTS_MAX];
  int cnte_array[MAPLENGTH];

  /// matrix declarations
  /// updated covariance maintained by averaged update function
  UMatrix4 mPu;  // updated covaiance
  /// updated locater state maintained by averaged update function
  UMatrix4 mXu;
  /* Matrices used in EKF calculation: Handles */
  matrix  *eye3;
  /// state and estimate state (twice) matrix (3x1)
  matrix *x, *x_, *x_up;
  /// starte and estimated covariance matrix
  matrix *P, *P_;
  /// odometry based state update
  matrix *x_odo;
  /// odometry relation to covariance
  matrix *df_x, *df_x_trans, *df_u, *df_u_trans;
  /// temporary elements of P during odometry update
  matrix *P_1, *P_2, *P_3, *P_4;
  /// measurement vector Z={d; th} Z3 = {d, th, t}
  matrix *z, *z3;
  /// connection matrix between measurement and state variables
  matrix *H, *H_trans, *H3, *H3_trans;
  /// state noise matrix
  matrix *Q;
  /// measurement error
  matrix *R, *R3;
  /// measurement change
  matrix *y_hat, *y3_hat;
  /// temporary matrices
  matrix *tmp, *tmp23, *tmp32, *tmp33, *tmp22, *tmp31; //, *KL, *K3L;
  /// sensitiviti matrix for kalman gain calculation
  matrix *S, *S_inv, *S3, *S3_inv;
  /// kalman gain
  matrix *K, *K3;
  /// detection results
  /// maximum number of line detections in one scan
  static const int MAX_LOC_DECS = 20;
  /// detected lines
  ULocaterLineResult locDecs[MAX_LOC_DECS];
  /// number of detected lines
  int locDecsCnt;

protected: // Protected functions:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
   * allocate space for matrices */
  void make_matrix();
  /**
   * initialize matrices */
  void init_matrix();
  /**
   * get quality of localization */
  float quality();
  /**
   * update robot state in map coordinates
   * based on odometry movement
   * \param poseScan pose at current laser scan time.
   */
  void odoPredict(UPose * poseScan);
  /**
   * update state based on locater measurements
   * of distance from row and row angle
   * \param dist_meas is the measured distance to row
   * \param dist_pred the expected distance
   * \param ang_dev difference between measured and expected angle
   * \param line is the mapped row line
   */
  void correctionUpdate(double dist_meas, double dist_pred,
                        double ang_dev,
                        mapline * line);
  /**
   * update state based on locater measurements
   * of distance from row and row angle
   * \param dist_meas is the measured distance to row
   * \param dist_pred the expected distance
   * \param ang_dev difference between measured and expected angle
   * \param rowEndDev is the difference is row end position
   * \param line is the mapped row line
   */
  void correctionUpdate3(double dist_meas, double dist_pred,
                        double ang_dev,
                        double rowEndDev,
                        mapline * line);
  /**
   * estimate a row end locationcompared to current pose.
   * \param mPose is current robot pose (in map coordinates)
   * \param lPose is the current laser pose (in map coordinates)
   * \param maxRange is max range of laser scanner (twice that distance is searched)
   * \param lineIdx is the index to the row to test.
   * \param line is the associated map line
   * \param rend_dev is the measured deviation from old row end
   * \param lineName is the descriptive name of the line
   * \param scantime for logging
   * \returns true if a row end is estimated */
  bool rowEndDetect(UPose mPose, UPose lPose, double maxRange,
                    int lineIdx, mapline * line,
                    double * rend_dev,
                    const char * lineName, UTime scanTime);
  /**
   * Implement the detections of mapped lines.
   * NB! method to add measurements - to an average - is simplified!!
   * no alternative solution is found - yet.
   * \param updateKalman skould kalman matrices be updated (else just log)
   * \param logUpd pointer to logfile to use
   * \param updTime is the timestamp for the source data
   * \param serial is the serial number of the laserscan - for log only
   * \returns true if updates are usefull. */
  bool doLocatorUpdates(bool updateKalman, ULogFile * logUpd, UTime updTime,
                        UPose pose_scan, unsigned int serial);
  /**
   * Correlates laser scan points to each of the map lines in mapBase.
   * The result is delivered in the array parameters.
   * \param rd is the laserscan measurements
   * \param x_ matrix, that ???
   * \param cnt_array number of measurements associated to eaxh map point
   * \param x_array,y_array is the associated measurements to each map line
   * \param cnte_array number of measurements associated to eaxh map point
   * \param xe_array,ye_array is the associated measurements to each map line, assuming endless lines
   * \param ls_offset is forward position of laserscanner relative to robot origin (x-offset)
   * \param UResMapbase is a pointer to the map plugin with map line information */
  void match_points(RangeData rd, matrix *x_,
/*                    int *cnt_array,
                    double x_array[][POINTS_MAX],
                    double y_array[][POINTS_MAX],
                    int *cnte_array,
                    double xe_array[][POINTS_MAX],
                    double ye_array[][POINTS_MAX],*/
                    double ls_offset, UResMapbase *);
  /**
  Make a public polygon from the points used in linear regression calculation.
  \param reg is the linear regression points and result.
  \param mLine is the mapped line - for polygon naming
  */
  void makeRowPolygon(regtype reg, mapline mLine);
  /**
  Send this polygon to poly plug-in
  \param poly is the polygon (or polyline to send.
  \param name is the name of the polygon
  \param coordinatesystem is 0=odo, 1=utm, 2=map coordinate system
  \returns true if sucessfull. */
  bool polygonToPolyPlugin(UPolygon * poly, char * name, int coordinateSystem);


protected: // Protected variables:

  /* Last pose used */
  UPose pose_last;

  /* Variables in varpool */
  UVariable *varkalmpose;
  /*UVariable *varXk, *varYk, *varTk;
  UVariable *varXrel, *varYrel, *varTrel; */
  UVariable *varDist;
  UVariable *varUpdcnt;
  /// flag to allow update of mapPose
  UVariable * varKeepMapPose;
  /// flag to allow row-end to be used
  UVariable * varUseRowEnd;
  /// distance inside coverage before row end detect - when expected out of range.
  UVariable * varRowEndIn1;
  /// distance inside coverage before row end detect - when expected in range
  UVariable * varRowEndIn2;
  // max gap in a row detect to be valid (meter)
  UVariable * varRowEndMaxGap;
  /// use averaged update of locater, rather than each line individually
  UVariable * varUseAvgUpdate;
  /// maximum distance to a row used for line end detection
  UVariable * varRowToRobotMaxDist;
  /// max usable distance movement for row end detection - should be covariance dependent
  UVariable * varRowEndMaxDist;
  /// scale to multiply driven distance - to compensate for soft ground and tire variations
  UVariable * varOdoDistScale;
  UVariable * varRowExtraLog;
  UVariable * varRowWidthFactor;
  UVariable * varMaxAngleDeviation;

  /* Logdata array */
/*  double log_data[LOG_LENGTH][11];
  double log_lines[LOG_LENGTH][7];*/
//   int log_cnt;
//   int log_linecnt;

  /* Target variables */
  double xtarget, ytarget, ttarget;
  /**
   * logfiles */
  ULogFile fp, testfid;
  /// logfile for updates - matlab style
  ULogFile logUpd;
  /// logfile for updates - matlab style
  ULogFile logEnd;
  /**
   * is the startup pose initiated OK
   * This is to avoid the map pose to be initialized from a non-valid GPS pose. */
  bool initPoseOK;
private:
  /** temporary polygon for debug */
  UPolygon *poly400,  *poly40;
};

#endif  //URESLOCATER_H
