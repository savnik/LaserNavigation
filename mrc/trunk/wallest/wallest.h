/***************************************************************************
                          regression.h  -  description
                             -------------------
    begin                : Tue Dec 16 2003
    copyright            : (C) 2003 by Jose Fernandez,,,
    email                : ex04@little
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

 #include <stdio.h>
 #include <stdlib.h>
 #include <math.h>
 #define MAX_REGRESSION_SIZE 100

 /* defaut values for initialize */
 #define ROBOT_LENGTH 0.22 /* longitudinal distance from the odometry center of the robot to the sensor (metres)*/
 #define HALF_ROBOT_WIDTH 0.14 /* lateral distance from the odometry center of the robot to the sensor (metres)*/
 #define SAMPLE_DISTANCE_TIME (1/6.0) /*sample time of the distance sensors (seconds)*/
 #define TOLERANCE_FOR_HOLE 0.03 /* distance for considering a meassurement to belong to a hole or noise */
 #define TOLERANCE_FOR_WALL 0.07 /* distance for considering a meassurement to belong to the wall if we are driving parallel to the wall */
 #define POINTS_FOR_HOLE 3 /* necessary number of missed points to detect a hole */
 #define POINTS_FOR_WALL 3 /* necessary number of good points to detecta a wall */
 #define MINIMUN_REGRESSION_POINTS 7 /* minimum number of points necessary for a regression line (m/s)*/
 #define RELIANT_DISTANCE 0.08 /* minimun distance that the robot has to move to consider the line reliant */
 

enum{WE_RIGHT,WE_LEFT};
enum{WE_OFF,WE_ON};

typedef
  struct{
  /* user input */
  int side; /* LE_LEFT,LE_RIGHT */
	double line_distance;
	double input_max_hole_dist;
  double input_min_wall_after_hole;

  /* enviroment input */
  float odox, odoy, odoth; /* odometry information */
	float fwddist;
  float measured_distancel; /* meassured distance by the IR sensor we want to use */
  float measured_distancer; /* meassured distance by the IR sensor we want to use */
  int new_distance_valuel; /* 1 if the meassured distance value is a new one */
  int new_distance_valuer; /* 1 if the meassured distance value is a new one */
  double target_speed;
  int status; /* LE_ON,LE_OFF */



  /* default values */
  //unsigned int max_regression_size; /*  */
  int points_for_hole;
  int points_for_wall;
  float tolerance_for_hole;
  float tolerance_for_wall;
  float robot_length;
  float half_robot_width;
  float max_hole_dist;
  float min_wall_after_hole;
  float reliant_distance;

  /* memory */
  int length; /* used length of the regression, only internal use to change this value use input_length and reset */
  float xdata[MAX_REGRESSION_SIZE], ydata[MAX_REGRESSION_SIZE]; /* for storing regression data */
  double sx, sy, sx2, sy2, pxy; /* temporal values*/
  int initialized_part, next_position; /*Internal counters*/
  float x, y; /* new x and y values for the regression */
  int add_data; /* 0 if x and y information mustn't be used as new values */
  float error_distance; /* distance from the last point in the wall x ,y to the regression line */
  int good_points; /* number of consecutive detected points with a distance to the estimated regression line lower than TOLERANCE_FOR_WALL while driving paralel to a hole */
  int missed_points; /* number of consecutive detected points with a distance to the estimated regression line greater than TOLERANCE_FOR_HOLE while driving paralel to the wall */
  float lastdist; /* last forward distance to store last hole detection or wall detection */
  int uncondup; /* uncondicional update of regression data */
	int previous_status;



  /* output */
  float a, b, c; /* parametres of the implicit line to return ax + by + c = 0*/
  float angle; /* angle of the regression line*/
  float wall_distance; /* distance from the robot to the regression line*/
  int reliant; /* at least two different points are needed to have a line,
     if there are less than two points reliant will take the value 0 and the output
     of the regression line shouldn't be considered */
  int holes; /* number of holes, if it is running through a hole it will also include that one */
  int is_wall; /* 1 if we are running through a wall 0 if we are in a hole */


/*debug information*/
FILE *gendeb,*regx,*regy; //files to store information used for making the plots.
}wallesttype;

void wall_estimation(wallesttype *regr);


