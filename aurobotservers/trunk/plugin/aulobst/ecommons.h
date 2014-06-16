/*
 * LEL_commons.h
 *
 *  Created on: 17/12/2009
 *      Author: eba
 */

#ifndef E_COMMONS_H_
#define E_COMMONS_H_

struct LEL_GFParams
{ /// The maximum distance between a line and a point to include the point in the line support
  double distThreshold; //=0.01;
  /// measurements this close to a line is regarded a false measurement, and not used for obstacle generation.
  double lineEatDist;
  /// pointCount/30; // The minimum number of points to constitute a line
  int minLineSupport; //=10;
  ///pointCount/10; // The minimum number of points to continue iterations
  int minNoOfPoints; //=40;
  /// The maximum number of line extraction iterations
  int lineIter; // = 50;
  /// The number of random pairs drawn to find a line candidate
  int sampleIter;
  /// potentially split the line if distance freak larger than this
  double splitDist;
  /// do not split lines, 1 = split lines at split distances
  int enableSplit;
  /// minimum count of a split part of line, (if not split, i.e. enableSplit=false)
  int minSplitCnt; // 

  // constructor function
  inline LEL_GFParams()
  { // default parameters
    distThreshold=0.01;
    lineEatDist = 0.05;
    minLineSupport = 10;
    minNoOfPoints = 40;
    lineIter = 50;
    sampleIter = 20;
    splitDist = 0.8;
    enableSplit = 0;
  }
};

struct GFLine
{ // General form line described by Ax + By + C = 0
//  public:
    double A;
    double B;
    double C;
    int edgeCount;
    double startX;
    double startY;
    double endX;
    double endY;


    inline GFLine(){};
    inline GFLine(double Ain, double Bin, double Cin, int edgeCountin=0)
    {
      A=Ain;
      B=Bin;
      C=Cin;
      edgeCount=edgeCountin;
    }
    // full constructor
    inline GFLine(double Ain, double Bin, double Cin,
                      double lb, double le, int edgeCountin=0)
    {
      A=Ain;
      B=Bin;
      C=Cin;

      startX = lb*B-C*A;
      startY = -lb*A-C*B;
      endX = le*B-C*A;
      endY = -le*A-C*B;

      edgeCount=edgeCountin;
    }
    /**
     set new values for line */
    inline void set(double Ain, double Bin, double Cin, int edgeCountin=0)
    {
      A=Ain;
      B=Bin;
      C=Cin;
      edgeCount=edgeCountin;
    }
    /**
    Set endpoints from on-line positions
    \param lb start position on line
    \param le end position on line
    */
    inline void setEndpoints(double lb, double le)
    {
      startX = lb*B-C*A;
      startY = -lb*A-C*B;
      endX = le*B-C*A;
      endY = -le*A-C*B;
    }
};

#endif /* LEL_COMMONS_H_ */
