/***************************************************************************
 *   Copyright (C) 2011 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *   $Rev: 1870 $
 *   $Id: ufunclocalize.h 1870 2012-03-16 14:41:22Z eba $
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UFUNC_LOCA2_H
#define UFUNC_LOCA2_H

#include <cstdlib>
#include <list>

using namespace std;

#include <ulms4/ufunclaserbase.h>
#include <urob4/uresposehist.h>
#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <umap4/uposev.h>
#include "../aupoly/urespoly.h"
#include <LEL_ransac.h>
#include <iau_mat.h>
#include <eigen3/Eigen/Dense>
// #include <mhf/MultiHypDist.h>
// #include <mhf/SplitTable.h>
using namespace Eigen;
class ULineMatch;
class ULaserMatches;
class ULocaMatchStat;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


/**
 * Localizer based on Enis implementation of kalman localizer filter
 * and - in unknown state - a multi hypothesis filter.
 * This version is modified for more consensus based correlation by
 * @author Christian Andersen
*/
class UFuncLoca2 : public UFuncLaserBase
{
public:
  /**
  Constructor */
  UFuncLoca2();
  /**
  Destructor */
  virtual ~UFuncLoca2();

  /**
  Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  virtual bool setResource(UResBase * resource, bool remove);

  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
  /**
  Update module status (global variables)
  \param mapX is the localized position in X
  \param mapY is the localized position in Y
  \param mapTh is the localized position in radians
  \param time is the valid time for the position
  \param match was the last update a match. */
  void updateStatus(double mapX, double mapY, double mapTh, UTime time);
  /**
  Update covariance status in global variables. */
  void updateCovStatus();
  /**
   * Correlate laser lines and map lines using 2D-line segment laser lines.
   * \param uNewPose is robot pose at time of data lines
   * \returns number of matches */
  int correlateLines(UPoseTime uNewPose);
  /**
   * Correlate laser lines and map lines - using old LEL_GFline laser line list
   * \param GFLL is detected lines in laser scanner coordinates
   * \param uNewPose is robot pose at time of data lines
   * \returns number of matches */
  int correlateLines_old(list<LEL_GFLine> GFLL, UPoseTime uNewPose);
  /**
   * Use list of laser line matches to mark best match for each of the candidate correlations */
  void markBestJointMatches();
//  void markBestJointMatchesNot();

  void updateDisplacement(UTime &timeOfPose,
                          Matrix<double,3,1> &pose,
                          Matrix<double,3,3> &poseCov,
                          UTime scanTime, UResPoseHist  *poseHist,
                          int silent,
                          double odoB, double KR, double KL);


protected:
  list<LEL_ARLine> lineList;
  Matrix<double,3,1> pose;
  Matrix<double,3,3> poseCov;
//   MultiHypDist<3> poseDist;
//   SplitTable<1> table;

  UResPoseHist * poseHist;
  int poseIndex;
  UPoseTime lastOdoPose;
  int matchMiss;
  UPosRot lasPose;
  double transX, transY, transTh;

  /// number of points that must be inside line segment to correlate
  UVariable * varMinOverlap;
  /// value of current covariance matrix
  UVariable * varCovar;
  /// eigenvectors of x,y part of current covariance matrix
  UVariable * varCovarV;
  /// eigenvalues of x,y part of current covariance matrix
  UVariable * varCovarA;
  /// number of successful match updates
  UVariable * varUpdates;
  /// number of failed matches since last successful match
  UVariable * varFailed;
  /// number of defined lines in localizer
  UVariable * varDefinedLines;
  /// robot base (differential drive?)
  UVariable * varOdoB; // = 0.26;
  /// distance varaince for right wheel each moved meter
  UVariable * varKR; // = 0.0003;
  /// distance varaince for left wheel each moved meter
  UVariable * varKL; // = 0.0003;
  /// The maximum allowed variances for each hypothesis
  UVariable * varMaxXYVariance; // = 0.05;
  UVariable * varMaxThVariance; // = 0.05;
  /// The noise covariances for laser readings
  UVariable * varLaserAlpha; // = 0.001;
  UVariable * varLaserR; // = 0.0004;
  /// mahalanobi distance cost, when using no correlation
  UVariable * varMahaDistNoCorr;
  /// automatic calculated offset to odometry updates to get a bias-free update - distance and heading
  UVariable * varOffsetValue;
  /// gain for filtering offset to odometry, default is 0.01 (100 updates)
  UVariable * varOffsetGain;
  /// list of valid matches (Mahalanobi distance less than some limit)
  static const int MML = 200;
  ULineMatch * matchList[MML];
  /// number of entries in match list
  int matchListCnt;
  /// the mean Mahalanobi distance in all matches in matchList,
  /// to be used as distance value for no-match test.
  double mahaMean;

  ULaserMatches * laserMatches[MML];
  int laserMatchesCnt;
//   Matrix<double,2, 3> matchDelH_delPs[MML];
// //  Matrix<double,2, 1> lineDifMin;
//   Matrix<double,2, 2> matchlineDifCovs[MML];
  /** maximum number of laser lines in one svan */
  static const int MAX_LAS_CNT = 200;
  /** laser line list - first element is robot pose at scantime (UPoseTime*) the rest is 2D segments (U2Dseg*) */
  UDataBase * laserLines[MAX_LAS_CNT];
  /** number of valid elements in laser line list */
  int laserLinesCnt;

  /** array of best joint correlations */
  static const int MJM = 50;
  ULocaMatchStat * jointMatch[MJM];
  /**
   * Innovation logfile */
  ULogFile innoLog;

private:
  //bool handleOdoposeUpdate();
  bool handleLocalize(UServerInMsg * msg, void * extra);
  /**
   * Handle addition of map lines.
   * \param msg is a struct with further parameters
   * \param added is number of lines added.
   * \returns true if no parameter error */
  bool handleAddLine(UServerInMsg * msg, int * added);
  /**
   * Add a detectable line from 3D position p1 to 3D position p2 */
  void addLine(UPosition p1, UPosition p2);
//   bool handleSetInitPose(UServerInMsg * msg);
//   bool handleSetInitCov(UServerInMsg * msg);
//   bool handleResetLocalizer(UServerInMsg * msg);
  void projectToLaser(LEL_ARLine worldLine, Matrix<double,3,1> & pose, Matrix<double,3,3> & poseCov, LEL_ARLine &projLine, Matrix<double,2,2> &lineCov, Matrix<double,2,3> & delH_delP);

  //Matrix<double,2,1> projectToLaserUKF(Matrix<double,3,1> pose, Matrix<double,2,1> noise, Matrix<double,5,1> auxin);
  /**
  Create local variables for manipulating parameters */
  void createBaseVar();


public:
  // this EIGEN_... line is needed to allow Eigen matrix operations to use SSE instruction
  // as these require 128 bit alignment of created matrices
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


class ULineMatch
{
public:
  /// distance between line and map line is SD units (squared)
  double minMahDist;
  Matrix<double,2, 3> delH_delP;
  Matrix<double,2, 2> lineDifCovAR;
  Matrix<double,2, 2> lineDifCovARinv;
  /** original mahalanobi distance from first pose */
  Matrix<double,2, 1> mahaDistXY;
  /** update pose and cov used in joint update calc */
  Matrix<double,3, 3> poseCov1;
  Matrix<double,3, 1> newPose1;
  /** update inovation */
  Matrix<double,3, 1> inov;
  /** mahalanobi distance of the inovation */
  double inovMaha;
  U2Dseg * laserLine; /// back reference to laser line
  LEL_ARLine mapLine; /// map line matched
  UPosition pe1;
  UPosition pe2;
  /** distance to expected position of line */
  double difR;
  double difA;
  /** is it a valid match */
  bool valid;
  /** index to laser line match structure */
  int laserLineMatch;
  /** center sum of distance to expected position - for calculation of best joint match */
  double sumA, sumR;
  /** sum squared */
  double sumA2, sumR2;
  /** number of valus in sum */
  int sumCnt;

public:
  // this EIGEN_... line is needed to allow Eigen matrix operations to use SSE instruction
  // as these require 128 bit alignment of created matrices
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  ULineMatch()
  {
//     delH_delP = NULL;
//     lineDifCov = NULL;
    minMahDist = 1e3;
    valid = false;
  };
  void print(bool silent, char c, int mapLineCnt, int matchCnt, double overlap)
  {
    pe1 = laserLine->getFirstEnd();
    pe2 = laserLine->getOtherEnd();
    if (not silent)
      printf("-- match %3d laserLine %3d to mapLine %3d (%s) overlap %6.2fm (laser %6.2fx,%6.2fy to %6.2fx,%6.2fy) <-> (map %.2fx,%.2fy to %.2fx,%.2fy) a-dif %.4f, R-dif %.4f, maha dist %.3f %c\n",
                  matchCnt, laserLineMatch, mapLineCnt,  mapLine.name, overlap,
                  pe1.x, pe1.y, pe2.x, pe2.y,
                  mapLine.p1[0], mapLine.p1[1], mapLine.p2[0], mapLine.p2[1], difA, difR, sqrt(minMahDist), c);
  };

  /**
   * Calculate robot pose and pose covariance, if this update is used only */
  void setExpectedPose(Matrix<double,3, 1> * pose, Matrix<double,3, 3> * poseCov)
  {
    Matrix<double,3, 1> poseDif;
    Matrix<double,2, 1> lineDif;
    Matrix<double,3, 2> K;
    //
    lineDif << difA, difR;
    //v = lineDif - (me->delH_delP * poseDif);
    lineDifCovARinv = lineDifCovAR.inverse();
//     K = poseCov * delH_delP.transpose()  * lineDifCovARinv;
//     newPose1 = pose + K * lineDif;
//     poseCov1 -= K * lineDifCovAR * K.transpose();
  }


//   void convToXY()
//   { // not used
//     Matrix<double,2,2> dHdPXY;
//     dHdPXY << delH_delP(0,0), delH_delP(0,1), delH_delP(1,0), delH_delP(1,1);
//     lineDifCovXY = dHdPXY.transpose() * lineDifCovAR * dHdPXY;
//   }

  void addToPoly(UResPoly * resPoly, UPose LaserPose, UPoseTime mapOdoOrigin)
  {
    UPolyItem * poly;
    UPosition p1, p2, m1, m2;
    if (resPoly != NULL)
    {
      poly = resPoly->getItem(mapLine.name);
      if (poly == NULL)
        poly = resPoly->add(mapLine.name);
      if (poly != NULL)
      {
        poly->lock();
        poly->clear();
        // add map line
//         poly->add(mapLine.p1[0], mapLine.p1[1]);
//         poly->add(mapLine.p2[0], mapLine.p2[1]);
        // to center line
        // get map line in map coordinates
        m1.set(mapLine.p1[0], mapLine.p1[1], 0.0);
        m2.set(mapLine.p2[0], mapLine.p2[1], 0.0);
        // convert to odometry
        m1 = mapOdoOrigin.getMapToPose(m1);
        m2 = mapOdoOrigin.getMapToPose(m2);
        // convert laser line to odometry coordinates too
        p1 = LaserPose.getPoseToMap(laserLine->x, laserLine->y);
        p2 = LaserPose.getPoseToMap(laserLine->getOtherEnd());
        poly->add((m1.x + m2.x)/2.0, (m1.y + m2.y)/2.0);
        // add laser line.
        poly->add((p1.x + p2.x)/2.0, (p1.y + p2.y)/2.0);
        poly->add(p1);
        poly->add(p2);
        poly->cooSys = 0; // odometry
        poly->setColor("r1dd");
        poly->setAsPolyline();
        poly->setUpdated();
        resPoly->gotNewData();
        poly->unlock();
      }
    }
  };
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class ULaserMatches
{
public:
  /** max mp lines per laser line */
  static const int MLM = 5;
  /** array of map lines that match this laser line */
  ULineMatch * mapLines[MLM];
  /**  pointer to laser line */
  // U2Dseg * laserLine;
  /** number of matches found */
  int mapLinesCnt;
  /** source laser line - back reference */
  // LEL_GFLine laserLine; // back reference to laser line
  /** index to map line that best matches laser line */
  int best;

public:
  /** constructor */
  ULaserMatches()
  { // clear matches
    mapLinesCnt = 0;
    for (int i = 0; i < MLM; i++)
      mapLines[i] = NULL;
  }
};


///////////////////////////////////////////////////////////////////////////

#endif

