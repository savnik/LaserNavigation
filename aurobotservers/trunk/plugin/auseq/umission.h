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
#ifndef UMISSION_H
#define UMISSION_H

#include <stdio.h>

#include <urob4/uresvarpool.h>
#include <ugen4/udatabase.h>

#include "ucalc.h"

/**
Name length of a variable */
#define MAX_VARIABLE_NAME_SIZE 32
/**
Maximum length of an error */
#define MAX_ERROR_SIZE 200

/**
 * \brief base class for mission lines 
 * */
class UMisBase : public UDataBase
{
  public:
  /**
   * Constructor */
    UMisBase();
  /**
   * Destructor */
    virtual ~UMisBase();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misBase";
  };
  /**
   * Set pointer to local calculator */
  void setCalc(UCalc * newCalculator);

protected:
  /**
   * Calculator to use by this instance, including 
   * position of local variables */
  UCalc * calc;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

typedef enum ELineType {
  EL_UNKNOWN,
  EL_ASSIGN,
  EL_DRIVE_STATEMENT,
  EL_SEGMENT,
  EL_DO_SEGMENT,
  EL_LABEL,
  EL_FLOW_IF, 
  EL_FLOW_LOOP, 
  EL_FLOW_GOTO,
  EL_PRINT,
  EL_REMARK
};


/**
 * Class that holds the base for a mission or a mission segment  */
class UMisLine : public UMisBase
{
public:
  /**
   * Constructor */
  UMisLine();
  /**
   * DEstructor */
  virtual ~UMisLine();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misLine";
  };

protected:
  /**
   * Line type */
  ELineType type;
  
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


/**
 * Class that holds the base for a mission or a mission segment  */
class UMisSegment : public UMisLine , public UCalc
{
public:
  /**
   * Constructor */
  UMisSegment();
  /**
   * DEstructor */
  virtual ~UMisSegment();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misSegment";
  };
  /**
   * Add a line to this segment */
  bool addLine(UMisLine * newLine);
  
protected:
  /**
   * Name of segment */
  char name[MAX_VARIABLE_NAME_SIZE];
  /**
   * Number of lines possible in a segment */
  static const int MAX_LINE_CNT_IN_SEGMENT = 500;
  /**
   * Sequence of lines in this segment */
  UMisLine * lines[MAX_LINE_CNT_IN_SEGMENT];
  /**
   * valid lines in segment */
  int linesCnt;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds the a mission  */
class UMission : public UMisSegment
{
public:
  /**
   * Constructor */
  UMission();
  /**
   * DEstructor */
  virtual ~UMission();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "mission";
  };
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds the a mission  */
class UMisDoSeg : public UMisSegment
{
public:
  /**
   * Constructor */
  UMisDoSeg();
  /**
   * DEstructor */
  virtual ~UMisDoSeg();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misDoSeg";
  };
};


#endif
