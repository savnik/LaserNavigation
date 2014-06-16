/*
 * LEL_commons.h
 *
 *  Created on: 17/12/2009
 *      Author: eba
 */

#ifndef LEL_COMMONS_H_
#define LEL_COMMONS_H_

#include <string.h>
#include <math.h>

struct LEL_ARLine { // A line described by cos(alpha)x + sin(alpha)y - r = 0
  /** angle of line from origo to closest point on line (radians) */
  double alpha;
  /** distance from line to origo (meters) */
  double r;
  /** line is start-end limited */
  bool limited;
  /** relative start position on line */
  double lb;
  /** relative end position on line */
  double le;
  /** start point if limited */
  double p1[2];
  /** end point if limited */
  double p2[2];
  /** max length of line name */
  static const int MNL = 32;
  /** line name */
  char name[MNL];

  /**
   * Get relative position on the line closest to this point.
   * \returns a relative position on line, can only be used to compare to other relative positions (in meters)
   * and to get position on line */
  inline double positionAlongLine(double x, double y) {return x*sin(alpha)-y*cos(alpha);}
  /** get the X position on the line from a relative position */
  inline double pointX(double pointAlongLine) {return r*cos(alpha)-pointAlongLine*sin(alpha);}
  /** get the Y position on the line from a relative position */
  inline double pointY(double pointAlongLine) {return r*sin(alpha)+pointAlongLine*cos(alpha);}
  /** constructor - initialize */
  inline LEL_ARLine() {
    limited = false;
  }

  /** setline as simpel angle (A), distance (R) line (
   * line is thus not limited. */
  inline LEL_ARLine(double alphaIn, double rIn) {
    alpha = alphaIn;
    r = rIn;
    limited = false;
  }

  /*inline LEL_ARLine(double alphaIn, double rIn, double lbIn, double leIn) {
    alpha = alphaIn;
    r = rIn;
    limited = true;
    lb = lbIn;
    le = leIn;
  }*/
  /** set line from start - end psitions.
   * Makes the line limited.
   * \param linename is a max 32 character name - is copied, and may not be NULL.
   */
  inline LEL_ARLine(double startX, double startY, double endX, double endY, const char * linename) {
    float ex=startX+endX;
    float ey=startY+endY;
    float ex2=startX*startX+endX*endX;
    float ey2=startY*startY+endY*endY;
    float exy=startX*startY+endX*endY;
    p1[0] = startX;
    p1[1] = startY;
    p2[0] = endX;
    p2[1] = endY;
    strncpy(name, linename, MNL);

    alpha =atan2(2*ex*ey-4*exy,ex*ex-ey*ey-2*(ex2-ey2))/2;
    r =(ex/(float)2)*cos(alpha)+(ey/(float)2)*sin(alpha);

    limited = true;

    if(r<0) {
      r=-r;
      if(alpha<=0) alpha=alpha+M_PI;
      else alpha=alpha-M_PI;
    }

    lb= positionAlongLine(startX,startY);
    le= positionAlongLine(endX,endY);
    if(lb>le) {
      double tmp = le;
      le = lb;
      lb= tmp;
    }
  }
};

struct LEL_GFLine { // General form line described by Ax + By + C = 0
	double A;
	double B;
	double C;
	int edgeCount;
	double * edgesX;
	double * edgesY;
	double startX;
	double startY;
	double endX;
	double endY;


	inline LEL_GFLine(): edgesX(NULL), edgesY(NULL){};
	inline LEL_GFLine(double Ain, double Bin, double Cin, int edgeCountin=0) : edgesX(NULL), edgesY(NULL) {
		A=Ain;
		B=Bin;
		C=Cin;
		edgeCount=edgeCountin;
	}

	inline LEL_GFLine(double Ain, double Bin, double Cin, double lb, double le, int edgeCountin=0) : edgesX(NULL), edgesY(NULL) {
		A=Ain;
		B=Bin;
		C=Cin;

		startX = lb*B-C*A;
		startY = -lb*A-C*B;
		endX = le*B-C*A;
		endY = -le*A-C*B;

		edgeCount=edgeCountin;
	}

	inline LEL_ARLine toARLine() {
		double alpha = atan2(B,A);
		double r = -C/sqrt(A*A+B*B);
		if(r<0) {
			r=-r;
			if(alpha<=0) alpha=alpha+M_PI;
			else alpha=alpha-M_PI;
		}
		LEL_ARLine result(alpha,r);
		return result;
	}
};

#endif /* LEL_COMMONS_H_ */
