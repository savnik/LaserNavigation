/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   chrand@mail.dk   *
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

#include <ugen4/umatrix.h>

#include "uvarlog.h"
#include <iau_mat.h>
#include "uvariable.h"
////////////////////////////////////////////////

UVarHist::UVarHist(UVarLog * parent)
{
  histData = NULL;
  colCnt = 0;
  rowMaxCnt = 0;
  histTime = NULL;
  rowCnt = 0;
  rowNext = 0;
  minUpdTime = 0.005; // set to 5 ms - should be OK in most cases
  varLogName = NULL;
  varParent = parent;
}

////////////////////////////////////////////////

UVarHist::~UVarHist()
{
  if (rowMaxCnt > 0)
  { // log the last entry
    logDataDouble(rowNext);
    // remove data
    rowMaxCnt = 0;
    rowCnt = 0;
    rowNext = 0;
    // release allocated memory
    free(histData);
    free(histTime);
  }
}

//////////////////////////////////////////////////

bool UVarHist::setHistSize(int histRowCnt, int histColCnt)
{
  rowCnt = 0;
  rowNext = 0;
  if (histRowCnt * histColCnt > 0)
  {
    histData = (double *) realloc(histData, sizeof(double) * histRowCnt * histColCnt);
    histTime = (UTime *) realloc(histTime, sizeof(UTime) * histRowCnt);
    histTime[0].clear();
  }
  else if (rowMaxCnt > 0)
  {
    free(histData);
    free(histTime);
    histData = NULL;
    histTime = NULL;
  }
  // set size of time series matrix
  if (histData != NULL and histTime != NULL)
    rowMaxCnt = histRowCnt;
  else
    rowMaxCnt = 0;
  colCnt = histColCnt;
  return rowMaxCnt == histRowCnt;
}

////////////////////////////////////////////////

void UVarHist::changed(double* data, bool isDouble, UTime updTime)
{
  double dt = updTime - tUpd1;
  // double based
  if (isDouble and rowNext < rowMaxCnt)
  { // there is space available - save data
    if (fabs(dt) > minUpdTime)
    { // time to log last version of the data
      /// @todo if time goes backwards, it is likely to be old data
      /// replayed, this is seen as start of a new time series, and
      /// NOT put sorted into the history. - this is to allow
      /// switch from live to replay (and back) 
      if (rowCnt > 0)
      {
        logDataDouble(rowNext);
        rowNext = (rowNext + 1) % rowMaxCnt;
      }
      if (rowNext >= rowCnt)
        rowCnt = rowNext + 1;
      tUpd1 = updTime;
    }
    // update history row with newest data
    memcpy(&histData[rowNext * colCnt], data, sizeof(double) * colCnt);
    // and the associated update time
    histTime[rowNext] = updTime;
  }
  // string based variable
  if (isLogOpen() and not isDouble)
  { // strings are logged as is
    if (fabs(dt) > minUpdTime)
    { // no history for strings
      toLog((const char*) data);
      tUpd1 = updTime;
    }
  }
}

////////////////////////////////////////////////

void UVarHist::logDataDouble(int idx)
{
  if (isLogOpen())
  {
    double * data = &histData[idx * colCnt];
    UTime * t = & histTime[idx];
    // save timestamp
    fprintf(getF(), "%lu.%06lu", t->getSec(), t->getMicrosec());
    // save data elements
    for (int i = 0; i < colCnt; i++)
      fprintf(getF(), " %.10g", data[i]);
    // end line
    fprintf(getF(), "\n");
    // remenber to flush if requested
    if (logFlush)
      fflush(getF());
  }
}

////////////////////////////////////////////////

void UVarHist::makeLogName(const char * preName , const char * varName )
{
  if (varLogNameMaxCnt < (int)strlen(preName) + (int)strlen(varName) + 2)
  {
    varLogNameMaxCnt = strlen(preName) + strlen(varName) + 2;
    varLogName = (char*)realloc(varLogName, varLogNameMaxCnt);
  }
  if (varLogName != NULL)
  {
    snprintf(varLogName, varLogNameMaxCnt, "%s%s", preName, varName);
    setLogName(varLogName);
  }
}

////////////////////////////////////////////////

void UVarHist::logAll()
{
  int idx = rowNext - rowCnt;
  //
  if (idx < 0)
    // when folded do not use oldest (as this is used as buffer for current changes
    idx = (idx + rowMaxCnt + 1) % rowMaxCnt;
  while (idx != rowNext)
  {
    logDataDouble(idx);
    idx = (idx + 1) % rowMaxCnt;
  }
}

//////////////////////////////////////////////////////

bool UVarHist::decodeReplayLine(char * line)
{
  if (varParent != NULL)
    return varParent->decodeReplayLine(line);
  else
    return true;
}

////////////////////////////////////////////////

UMatrix * UVarHist::getTimeSeries(UMatrix * buffer, int maxRows)
{
  if (rowCnt > 1)
  {
    int rws = mini(rowCnt - 1, maxRows);
    if ((int)buffer->rows() != rws or (int)buffer->cols() != colCnt)
      buffer->setSize(rws, colCnt);
    for (int i=1; i <= rws; i++)
    {
      const double * val = getData(i);
      buffer->setRow(i-1, colCnt, val);
    }
  }
  return buffer;
}

////////////////////////////////////////////////

int UVarHist::getTimeVector(double* buffer, int bufferCnt, int element, int interval)
{
  double * bp = buffer;
  int rws = 0;
  if (rowCnt > 1 and element < colCnt)
  { // limit to a legal column
    int c = maxi(0, element);
    if (interval < 1)
      interval = 1;
    // find number of values to fetch
    rws = mini((rowCnt - 1)/interval, bufferCnt);
    for (int i = 1; i <= rws; i++)
    { // save the column into the buffer
      if (element < 0)
        *bp = getUpdTime(i * interval).getDecSec();
      else
        *bp = getData(i * interval)[c];
      bp++;
    }
  }
  // return number of fetched values.
  return rws;  
}

////////////////////////////////////////////////

int UVarHist::getVectorToTime(double * buffer,
                              int bufferCnt,
                              int element,
                              UTime tot,
                              int interval,
                              double * gotdt)
{
  double * bp = buffer;
  int i = 0;
  UTime st;
  //
  if (rowCnt > 1 and element < colCnt)
  { // limit to a legal column
    int c = maxi(0, element);
    // find number of values to fetch
    if (interval < 1)
      interval = 1;
    int rws = mini((rowCnt - 1)/interval, bufferCnt);
    for (i = 1; i <= rws; i++)
    { // save the column into the buffer
      UTime t = getUpdTime(i * interval);
      if (t < tot)
        break;
      if (element < 0)
      {
        if (element == -1)
          *bp = getUpdTime(i * interval).getDecSec();
        else
          // get time relative to newest
          *bp = getUpdTime() - getUpdTime(i * interval);
      }
      else
        *bp = getData(i * interval)[c];
      bp++;
    }
    i--;
    if (i > 1 and gotdt != NULL)
      *gotdt = (getUpdTime(1) - getUpdTime(i * interval)) / (i - 1);
  }
  // return number of fetched values.
  return i;
}

////////////////////////////////////////////////////

int UVarHist::getMeanMinMax(int element, UTime toTime,
                            double * mean, double * min, double * max,
                            int sampleInterval)
{
  double v, mn, mx, mi;
  int i = 0;
  UTime st;
  //
  if (rowCnt > 1 and element < colCnt)
  { // limit to a legal column
    int c = maxi(0, element);
    // find number of values to fetch
    int rws = rowCnt - 1;
    mn = getData(1)[c];
    mx = mn;
    mi = mn;
    for (i = 2; i <= rws; i += sampleInterval)
    { // test if data is too old
      if (getUpdTime(i) < toTime)
        break;
      // get value
      v = getData(i)[c];
      mn += v;
      if (v > mx)
        mx = v;
      if (v < mi)
        mi = v;
    }
    i--;
    if (mean != NULL)
      *mean = mn / i;
    if (min != NULL)
      *min = mi;
    if (max != NULL)
      *max = mx;
  }
  // return number of fetched values.
  return i;
}


////////////////////////////////////////////////

double UVarHist::getSampleRate(int element)
{
  int rw = mini(element, rowCnt - 1);
  UTime t1, t2;
  double ur = -1.0;
  if (rw > 1)
  {
    t1 = getUpdTime(1);
    t2 = getUpdTime(rw);
    ur = (rw - 1)/(t1 - t2);
  }
  // return number of fetched values.
  return ur;
}

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
//////////////////////////////////////////////

UVarLog::UVarLog()
{
  hist = NULL;
}

//////////////////////////////////////////////

UVarLog::~UVarLog()
{
  if (hist != NULL)
  {
    delete hist;
  }
}

//////////////////////////////////////////////

bool UVarLog::makeHist(int histCnt, int elementCnt, double maxUpdateRate)
{
  if (hist == NULL)
    hist = new UVarHist(this);
  if (hist != NULL and histCnt > 0)
  {
    hist->setHistSize(histCnt, elementCnt);
    if (maxUpdateRate > 1e-6);
      hist->minUpdTime = 1.0 / maxUpdateRate;
  }
  return hist != NULL;
}

////////////////////////////////////////////////

bool UVarLog::setMaxUpdateRate(double maxUpdateRate)
{
  bool result = hist != NULL;
  if (result)
  { // has time series
    result = maxUpdateRate > 1e-6;
    if (result)
      // update frequency is realistic
      hist->minUpdTime = 1.0 / maxUpdateRate;
  }
  return result;
}


////////////////////////////////////////////////

bool UVarLog::openLog(bool doOpen, const char * preName, const char * name)
{
  bool result = false;
  if (not hasHist())
    makeHist(1, getElementCnt2(), 200.0);
  if (hasHist())
  { // open or close log as needed
    bool hasUnloggedHist = hist->rowCnt > 0 and not hist->isLogOpen();
    if (preName != NULL and doOpen)
      hist->makeLogName(preName, name);
    hist->openLog(doOpen);
    // when log is opened and there is unloged data, then put history into log.
    if (hasUnloggedHist and hist->isLogOpen())
      hist->logAll();
    result = hist->isLogOpen();
  }
  return result;
};

///////////////////////////////////////////////////

bool UVarLog::setReplay(bool doOpen, const char * preName, const char * varName, UReplay * parent)
{
  if (not hasHist())
    makeHist(1, getElementCnt2(), 200.0);
  if (hasHist())
  {
    if (doOpen)
    {
      hist->replaySetBaseFileName(varName, preName);
      hist->setParent(parent);
    }
    hist->setReplay(doOpen);
    return hist->isReplayFileOpen();
  }
  else
    return false;
}

////////////////////////////////////////////////////////////

UMatrix * UVarLog::getTimeSeries(UMatrix * buffer, int maxRows)
{
  if (hasHist())
    hist->getTimeSeries(buffer, maxRows);
  return buffer;
}

////////////////////////////////////////////////////////////

int UVarLog::getTimeVector(double * buffer, int bufferCnt, int element)
{
  int n = 0;
  if (hasHist())
    n = hist->getTimeVector(buffer, bufferCnt, element);
  return n;
}

////////////////////////////////////////////////////////////

int UVarLog::getVectorToTime(double * buffer, int bufferCnt,
                             int element, UTime tot,
                             int interval,
                             double * gotdt)
{
  int n = 0;
  if (hasHist())
    n = hist->getVectorToTime(buffer, bufferCnt, element, tot, interval, gotdt);
  return n;
}

///////////////////////////////////////////////////////////////////

int UVarLog::getMeanMinMax(int element, UTime toTime,
                           double * mean, double * min, double * max,
                           int sampleInterval)
{
  int n = 0;
  if (hasHist())
    n = hist->getMeanMinMax(element, toTime, mean, min, max, sampleInterval);
  return n;
}

