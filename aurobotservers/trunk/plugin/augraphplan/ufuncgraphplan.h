/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                             *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
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
#ifndef UFUNC_PLAN_H
#define UFUNC_PLAN_H

#include <cstdlib>
#include <list>

using namespace std;

#include <urob4/ufuncplugbase.h>
#include <urob4/uresposehist.h>
#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <umap4/uposev.h>
#include <iau_mat.h>
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#define MAX_GRAPH_NODES 100

/**
 * Class to implement graph-based planner, implemented in
 * library compiled with MRC code
 * @author Nils A Andersen
 * modified by Christian Andersen (added helptext and comments)
*/
class UFuncPlan : public UFuncPlugBase
{
public:
  /**
  Constructor */
  UFuncPlan();
  /**
  Destructor */
  virtual ~UFuncPlan();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing. */
  virtual void createResources()
  {
    createBaseVar();
  };

protected:

//   matrix * pose;
//   matrix * poseCov;
//   int poseIndex;
//   UTime lastScanTime;
// 
//   UPosRot lasPose;
// 
//   // The matrices used:
//   matrix *poseDif, *delPnew_delY, *delPnew_delYTrans, *lineDifCov, *lineDifCovMin, *lineDifCovInv, *lineDifCovMinInv, *lineDif, *lineDifMin, *lineDif1,
//   *lineDifTrans, *mahDist, *delH_delP, *delH_delPMin, *delH_delPTrans, *delH_delPMinTrans, *newPose, *v, *K, *K1, *KTrans, *poseCovDif, *poseCovDif1,
//   *delPnew_delPold, *delPnew_delPoldTrans, *covU, *covU1, *covU2, *covIn1, *covIn2, *cov1;

  /// number of points that must be inside line segment to correlate



private:
  bool handleOdoposeUpdate();

  bool handleFindRoute(UServerInMsg * msg);
  bool handleGetPoint(UServerInMsg * msg);
  bool handleResetPlan(UServerInMsg * msg);
  bool handleAddPoint(UServerInMsg * msg);
  bool handleAddCon(UServerInMsg * msg);
  bool handlecalculatecost(UServerInMsg * msg);
  /**
  Create local variables for manipulating parameters */
  void createBaseVar();
  
  /// index to global variables in this plugin
  UVariable * varPointsInMap;
  UVariable * varConnectionsInMap;
  UVariable * varPointsInPlan;
  UVariable * varMakePolygons;
  UVariable * varPlanTime;
  /// polygon (pointlist) for nodes
  UPolygon * nodePoly;
//   /**
//    * Call other plugin with a single string parameter.
//    * \param function is full name of function to call.
//    * \param stringParam is the single string parameter used in the call 
//    * \returns true if function exist (plugin is loaded) */
//   bool callVS(const char * function, const char * stringParam);
//   /**
//    * Call other plugin with a string, data-struct and integer parameter.
//    * \param function is full name of function to call.
//    * \param strPar is the single string parameter used in the call 
//    * \param data   is the data needed by the call. 
//    * \param coosys is an integer as the last parameter (often coordinate system of the data 0=odo,1=utm,2=map) 
//    * \returns true if function exist (plugin is loaded) */
//   void callVSCD(const char * function, const char * strPar, UDataBase * data, int cooSys);
};


#endif

