// $Id: commandreader.c,v 1.1.1.1 2005/11/22 21:39:58 naa Exp $
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commandreader.h"

int commandreader_getstate(char *token) {

  if (strcmp(token,"FLW_BLACK_LINE") == 0)
    return FLW_BLACK_LINE;
  else if (strcmp(token,"MOVE") == 0)
    return MOVE;
  else if (strcmp(token,"FLW_WHITE_LINE") == 0)
    return FLW_WHITE_LINE;
  else if (strcmp(token,"FLW_LEFT") == 0)
    return FLW_LEFT;
  else if (strcmp(token,"FLW_RIGHT") == 0)
    return FLW_RIGHT;
  else if (strcmp(token,"FLW_WALL") == 0)
    return FLW_WALL;
  else if (strcmp(token,"TURN") == 0)
    return TURN;
  else if (strcmp(token,"CALCULATE") == 0)
    return CALCULATE;
  else if (strcmp(token,"WAIT") == 0)
    return WAIT;
  else if (strcmp(token,"CHG_ROUTE") == 0)
    return CHG_ROUTE;
  else if (strcmp(token,"RETURN") == 0)
    return RETURN;
  else if (strcmp(token,"JMPIF") == 0)
    return JMPIF;
  else if (strcmp(token,"STOP") == 0)
    return STOP;
  else if (strcmp(token,"MSG") == 0)
    return MSG;
  else {
    fprintf(stderr,"Error: invalid state encountered: %s\n",token);
    exit(1);
  }

  return -1;
}

int commandreader_needcondition(enum state state) {
  switch(state) {
  case STOP: case RETURN: case CHG_ROUTE:
    return 0;
  default:
    return 1;
  }
}

int commandreader_getcondition(char *token, enum state state) {

  if (strcmp(token,"INTERSECTION") == 0)
    return INTERSECTION;
  else if (strcmp(token,"CROSSING_BLACK") == 0)
    return CROSSING_BLACK;
  else if (strcmp(token,"CROSSING_WHITE") == 0)
    return CROSSING_WHITE;
  else if (strcmp(token,"ODO") == 0)
    return ODO;
  else if (strcmp(token,"ODO_CALC") == 0)
    return ODO_CALC;
  else if (strcmp(token,"REL_ANGLE") == 0)
    return REL_ANGLE;
  else if (strcmp(token,"ABS_ANGLE") == 0)
    return ABS_ANGLE;
  else if (strcmp(token,"END_WALL") == 0)
    return END_WALL;
  else if (strcmp(token,"BLACK_LINE_FOUND") == 0)
    return BLACK_LINE_FOUND;
  else if (strcmp(token,"WHITE_LINE_STOP") == 0)
    return WHITE_LINE_STOP;
  else if (strcmp(token,"BLACK_LINE_STOP") == 0)
    return BLACK_LINE_STOP;
  else if (strcmp(token,"OBSTACLE") == 0)
    return OBSTACLE;
  else if (strcmp(token,"NOLINE") == 0)
    return NOLINE;
  else if (strcmp(token,"TIME") == 0)
    return TIME;
  else if (strcmp(token,"ALWAYS") == 0)
    return ALWAYS;
  else if (strcmp(token,"IN_DOCK") == 0)
    return IN_DOCK;
  else if (strcmp(token,"NONE") == 0)
    return NONE;
  else if (strcmp(token,"BATTERY_OK") == 0)
    return BATTERY_OK;
  else if (strcmp(token,"RTN") == 0)
    return RTN;
  else if (strcmp(token,"CHG_STATUS") == 0)
    return CHG_STATUS;
  else if (commandreader_needcondition(state)) {
    fprintf(stderr,"Error: invalid condition found: %s\n",token);
    exit(1);
  }
  return -1;
}

int commandreader_getspeed(char *token) {

  if (token)
    return SPEEDFACTOR * atof(token);
  else
    return -1;

}

int commandreader_getparameter(char *token) {

  if (token)
    return atof(token);
  else
    return -1;
}

void commandreader_toupper(char *c) {
  while (*c) {
    if ((*c > 96) && (*c < 123))
      *c -= 32;
    c++;
  }
}

void commandreader_scanline(char *line, int route[][6],
			    int *lines) {
  enum condition condition;
  enum state state;
  char *token;
  int speed, parameter;
  char delimiters[] = " \t\n";

  commandreader_toupper(line);

  token = strsep(&line,delimiters);
  state = commandreader_getstate(token);
  route[*lines][0] = state;

  token = strsep(&line,delimiters);
  condition = commandreader_getcondition(token,state);
  route[*lines][1] = condition;

  token = strsep(&line,delimiters);
  speed = commandreader_getspeed(token);
  route[*lines][2] = speed;

  token = strsep(&line,delimiters);
  parameter = commandreader_getparameter(token);
  route[*lines][3] = parameter;

  printf("Scanned: %d , %d , %d , %d\n",state,condition,speed,parameter);

  route[*lines][4] = 6000;
}

void commandreader_readfile(char *filename, int route[][6],
			    int maxlines, int *lines) {
  char line[MAXCHARS];
  FILE *fp;

  if (!(fp = fopen(filename,"rb"))) {
    fprintf(stderr,"Error: unable to open file '%s'\n",filename);
    exit(1);
  }

  *lines = 0;
  while (fgets(line,MAXCHARS,fp)) {
    if (*lines >= maxlines) {
      fprintf(stderr,"commandreader_readfile: error: route array too small\n");
      exit(1);
    }
    commandreader_scanline(line, route, lines);
    (*lines)++;
  }
}

void commandreader_print(int route[][6], int n) {
  int i,j;

  for (i = 0; i < n; i++) {
    for (j = 0; j < 6; j++) {
      printf("%03d ",route[i][j]);
    }
    printf("\n");
  }

}
