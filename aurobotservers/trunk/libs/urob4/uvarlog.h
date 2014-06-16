/***************************************************************************
 *   Copyright (C) 2011 by DTU Christian Andersen                          *
 *   chrand@elektro.dtu.dk                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UVAR_LOG_H
#define UVAR_LOG_H

#include <ctype.h>

#include <ugen4/udatabase.h>
#include <ugen4/ulock.h>
#include <ugen4/uposrot.h>
#include <umap4/upose.h>
#include <iau_mat.h>

#include "ulogfile.h"
#include "ureplay.h"

class UVarLog;
/**
Class to extend functionality of a UVariable with functions for available (fixed size) history and logfile
functionality */
class UVarHist : public ULogFile, public UReplay
{
public:
  /**
   * Constructor */
  UVarHist(UVarLog * parent);
  /**
   * Destructor */
  ~UVarHist();
  /**
  Set history buffer size - if size is changed, then history is lost.
  \param  histRowCnt is the number of history rows to be allocated.
  \param histColCnt is the number of data elements (columns) in each hirtory element
  \returns true if allocated. */
  bool setHistSize(int histRowCnt, int histColCnt);
  /**
  Data has been updated - make history.
  \param data is a pointer to curreent data (after change).
  \param updTime is time associated with update.
  \param isDouble if true then data is a double buffer, else a char buffer. */
  void changed(double * data, bool isDouble, UTime updTime);
  /**
  Save newest data to logfile as doubles (if open)
  \param idx is the history index to log. */
  void logDataDouble(int idx);
  /**
  Get update time of newest element */
  inline UTime getUpdTime()
  { return histTime[rowNext]; };
  /**
  Get update time of indexed element */
  inline UTime getUpdTime(int idx)
  {
    int n = rowNext - idx;
    if (n < 0 and rowCnt == rowMaxCnt)
      n += rowMaxCnt;
    if (n < 0)
    { // return time zero
      UTime t;
      return t;
    }
    else
      return histTime[n];
  };
  /**
  Get data pointer of indexed element
  \param idx is the index to a historic value. 0 is newest*/
  inline const double * getData(int idx)
  {
    if (idx < rowCnt)
    { // an available value
      int n = rowNext - idx;
      if (n < 0)
        n += rowMaxCnt;
      return &histData[n * colCnt];
    }
    else
      return NULL;
  };
  /**
  Make log filename buffer for this name.
  \param preName is the name of the struct.
  \param varName is the name of the variable. */
  void makeLogName(const char * preName, const char * varName);
  /**
  Log all elements in in-memory history. */
  void logAll();
  /**
  Get time series as a matrix with all comumns a maximum of rows.
  The first row is the newest (history) data (not current data).
  the buffer matrix will be resized (if needed) to fit data size.
  \param buffer is the matrix that is to be filled with the data.
  \param maxRows is the maximum size of the data. */
  UMatrix * getTimeSeries(UMatrix * buffer, int maxRows);
  /**
  Get a time series vector from current history
  \param buffer is a result buffer for the values
  \param bufferCnt is the size of the buffer (maximum this number of elements will be loaded into buffer.
  \param element is the element (history column) to use, 0=first element.
    if element is -1, then the update time in decimal seconds is returned.
    if -2 then time is relative to newest update time.
  \param interval is the interval of the samples taken, 2 means use every second measurement only
  \returns number of elements returned (may be less than or equal to bufferCnt dependent of available data and history buffer size.
  \returns 0 if no history is available. */
  int getTimeVector(double * buffer, int bufferCnt, int element, int interval = 1);
  /**
  Get a time series vector from current history with specifiet update rate and end time.
  if update rate is not possible, then average rate is returned.
  The method is to skip recorded updates until specified dt is passed, then use and increase to next expected time
  \param buffer is a result buffer for the values
  \param bufferCnt is the size of the buffer (maximum this number of elements will be loaded into buffer.
  \param element is the element (history column) to use, 0=first element.
    if -1 then the update time value (in decimal seconds) is returned.
    if -2 then time is relative to newest update time.
  \param tot is the end time where the vector should end.
  \param interval is interval of the samples taken, 1 is all, 2 is every second ....
  \param gotdt is the (average) sample interval of saved data [sec].
  \returns number of elements returned (may be less than or equal to bufferCnt dependent of available data and history buffer size.
  \returns 0 if no history is available. */
  int getVectorToTime(double * buffer, int bufferCnt, int element, UTime tot, int interval, double * gotdt);
  /**
  Get some statistics from the time series
  \param element is the element (history column) to use, 0=first element.
  \param toTime is the end time where the vector should end.
  \param mean is set to  the mean value in the specified time series (unchanged if no data) - may be NULL
  \param min is set to the minimum value in the specified time series (unchanged if no data) - may be NULL
  \param max is set to the maximum value in the time series (unchanged if no data) - may be NULL.
  \param sampleInterval is the interval between samples to test (default is 1)
  \returns number of elements used - 0 if no data. */
  int getMeanMinMax(int element, UTime toTime,
                    double * mean, double * min, double * max,
                    int sampleInterval = 1);
  /**
  Get actual sample rate, when looking this number of samples back.
  Function just looks at the oldest and newest element an number of history elements.
  \param element is the (max) number of time series elements to look back for average sample rate.
  \returns sample rate in Hz, or -1.0 if there is not at least 2 elements in the time series. */
  double getSampleRate(int element);

protected:
  /**
  Decode this replay line, that is assumed to be valid.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
  virtual bool decodeReplayLine(char * line);
  
public:
  double * histData;
  /// size of each history data row (each row may be a matrix in its own right)
  int colCnt;
  /// number of history elements in buffer
  int rowMaxCnt;
  /// time associated with each history element
  UTime * histTime;
  /// time of first update not yet in update history
  UTime tUpd1;
  /// number of elements used.
  int rowCnt;
  /// next element to fill next
  int rowNext;
  /// min update time
  double minUpdTime;
  /// name of fully qualified variable (also base name of logfile (less path and extension))
  char * varLogName;
  /// max length of name
  int varLogNameMaxCnt;
  /// pointer to parent variable - to allow replay.
  UVarLog * varParent;
};


/**
A variable history maintainer */
class UVarLog
{
public:
  /**
   * Constructor */
  UVarLog();
  /**
   * Destructor */
  virtual ~UVarLog();
  /**
  Has a log history been initialized */
  inline bool hasHist() const
  { return hist != NULL; };
  /**
  Make time series history for this variable
  \param varCnt is the number of variable to be allocated for on-line history.
  \param elementCnt is number of elements in each history entry.
  \param maxUpdateRate is the maximum update rewate for the time series (must be greater than 1e-6)'
  \returns true if created. */
  bool makeHist(int histCnt, int elementCnt, double maxUpdateRate);
  /**
  Set maksimum update rate for time series
  \param maxUpdateRate is the maximum update rewate for the time series (must be greater than 1e-6)'
  \returns true if set. */
  bool setMaxUpdateRate(double maxUpdateRate);
  /**
  This variable is updated */
  inline void changed(double * data, bool isDouble = true, UTime * updTime = NULL)
  {
    if (hist == NULL)
      return;
    if (updTime == NULL)
    { // use current time
      UTime t;
      t.now();
      hist->changed(data, isDouble, t);
    }
    else
      hist->changed(data, isDouble, *updTime);
  };
  /**
  Get update time of newest element */
  inline UTime getUpdTime()
  {
    UTime t;
    if (hist != NULL)
      t = hist->getUpdTime();
    return t;
  };
  /**
  Open log
  \param doOpen if true: opens replay file and set replay status - requires name and resource parent to work.
                if false: closes replay file and stops replay.
  \param varParent is the variable owner of this history structure (mainly used during replay)
  \param preName is the full name of the structure that holds the variable (may be empty) (supposed to end in a '.'), this will be part of the filename.
  \param varName is the variable name and is added to the replay path, the pre-name and extended with a .log to form the full logfile name.
  \returns true if log is open. */
  bool openLog(bool doOpen, const char * preName = NULL, const char * varName = NULL);
  /**
  Open log
  \param doOpen if true: opens replay file and set replay status - requires name and resource parent to work.
                if false: closes replay file and stops replay.
  \param varParent is the variable owner of this history structure (mainly used during replay)
  \param preName is the full name of the structure that holds the variable (may be empty) (supposed to end in a '.'), this will be part of the filename.
  \param varName is the variable name and is added to the replay path, the pre-name and extended with a .log to form the full logfile name.
  \param parent is the resource replay parent (that will be informed on steps).
  \returns true if logfile is open. */
  bool setReplay(bool doOpen, const char * preName, const char * varName, UReplay * replayParent);
  /**
  Get logfile name (full path) */
  const char * getLogFilename()
  {
    if (hist == NULL)
      return "none";
    else
      return hist->ULogFile::getLogFileName();
  };
  /**
  Decode this replay line, that is assumed to be valid.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
  virtual bool decodeReplayLine(char * line)
  {
    return true;
  };
  /**
   * Get number of elements */
  virtual int getElementCnt2() const
    { return 0; };
  /**
  Get time series as a matrix with all comumns a maximum of rows.
  The first row is the newest (history) data (not current data).
  the buffer matrix will be resized (if needed) to fit data size.
  \param buffer is the matrix that is to be filled with the data.
  \param maxRows is the maximum size of the data. */
  UMatrix * getTimeSeries(UMatrix * buffer, int maxRows);
  /**
  Get a time series vector from current history
  \param buffer is a result buffer for the values
  \param bufferCnt is the size of the buffer (maximum this number of elements will be loaded into buffer.
  \param element is the element (history column) to use, 0=first element.
  \returns number of elements returned (may be less than or equal to bufferCnt dependent of available data and history buffer size.
  \returns 0 if no history is available. */
  int getTimeVector(double * buffer, int bufferCnt, int element);
  /**
  Get a time series vector from current history with specifiet update rate and end time.
  if update rate is not possible, then average rate is returned.
  The method is to skip recorded updates until specified dt is passed, then use and increase to next expected time
  \param buffer is a result buffer for the values
  \param bufferCnt is the size of the buffer (maximum this number of elements will be loaded into buffer.
  \param element is the element (history column) to use, 0=first element.
  \param tot is the end time where the vector should end.
  \param interval is interval of the samples taken, 1 is all, 2 is every second.
  \param gotdt is the (average) sample rate of saved data.
  \returns number of elements returned (may be less than or equal to bufferCnt dependent of available data and history buffer size.
  \returns 0 if no history is available. */
  int getVectorToTime(double * buffer, int bufferCnt, int element, UTime tot,
                             int interval, double * gotdt);
  /**
  Get some statistics from the time series
  \param element is the element (history column) to use, 0=first element.
  \param toTime is the end time where the vector should end.
  \param mean is set to  the mean value in the specified time series (unchanged if no data) - may be NULL
  \param min is set to the minimum value in the specified time series (unchanged if no data) - may be NULL
  \param max is set to the maximum value in the time series (unchanged if no data) - may be NULL.
  \param sampleInterval is the interval between samples to test (default is 1)
  \returns number of elements used - 0 if no data. */
  int getMeanMinMax(int element, UTime toTime,
                    double * mean, double * min, double * max,
                    int sampleInterval = 1);
  /**
  Get number of rows in the time series.
  \returns the number of history rows, excluding the newest, that is used for temporary values. */
  int getHistoryRows()
  {
    if (hist != NULL)
      return hist->rowCnt - 1;
    else
      return 0;
  };

public:
  /**
  Print status for this structure */
  //virtual void snprint(const char * preString, char * buff, const int buffCnt);
  UVarHist * hist;
};

#endif
