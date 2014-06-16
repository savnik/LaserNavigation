// $Id: commandreader.h,v 1.1.1.1 2005/11/22 21:39:58 naa Exp $
#ifndef COMMANDREADER_H
#define COMMANDREADER_H

#define MAXCHARS 1000
#define MAXLINES 1000
#define SPEEDFACTOR 1
#define COMMANDARRAYWIDTH 6

/******************************************************
 * NOTE!
 * These enums should NOT be here!
 * They have been taken from smrh2.c to assist in the
 * development of the reader.
 ******************************************************/
enum condition {
  INTERSECTION,   /* Line sensor detects intersection */
  CROSSING_BLACK,       /* Line sensor detects a crossing */
  CROSSING_WHITE,
  ODO,            /* Relative odometer  */
  ODO_CALC,       /* Odo + ir */
  REL_ANGLE, 
  ABS_ANGLE,
  END_WALL,
  BLACK_LINE_FOUND,
  WHITE_LINE_STOP,
  BLACK_LINE_STOP,
  OBSTACLE, 
  NOLINE, 
  TIME, 
  ALWAYS,
  IN_DOCK,
  NONE,
  BATTERY_OK,
  RTN,
  CHG_STATUS
};

/* Calculate-commands */
enum command {
  STORE_ODO,
  CALC_ODO_DIST,
  STORE_IR,
  CALC_TOTAL_DIST,
  SET_REFTHETA_TO_ODO,
  SET_ODO_TO_STORED_ODO,
  RESET_ODO,
  START_MIS_LOG,
  STOP_MIS_LOG,
  SAMPLE_DATA_ON,
  SAMPLE_DATA_OFF
};

/* States */
enum state {
  FLW_BLACK_LINE, 
  MOVE, 
  FLW_WHITE_LINE,
  FLW_LEFT,
  FLW_RIGHT,
  FLW_WALL,
  TURN,         /* PARAM1 determines direction of turn */
  CALCULATE,
  WAIT,
  CHG_ROUTE,
  RETURN,
  JMPIF,
  STOP,
  MSG
};

void commandreader_readfile(char *filename, int route[][6],int maxlines, int *lines);
void commandreader_print(int route[][6], int n);

#endif
