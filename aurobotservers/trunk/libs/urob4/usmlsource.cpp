/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen                          *
 *   jca@elektro.dtu.dk                                                    *
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

#include <string.h>
#include <ctype.h>

#include <ugen4/ucommon.h>
#include "usmltag.h"

#include "usmlsource.h"

USmlSource::USmlSource()
{
  ifName = NULL;
  dataCnt = 0;
  dataNext = data;
  gotFirstTag = false;
  errorBuffer = NULL;
  errorBufferCnt = 0;
  tagStartAtStartOfLineOnly = false;
  limitedMaxDataLength = MAX_CLIENT_RX_BUFFER;
}

/////////////////////////////////////

USmlSource::~USmlSource()
{
}

/////////////////////////////////////

// bool USmlSource::getNextTag(USmlTag * tag, int msTimeout,
//                               USmlTagIn * failEndTag,
//                               char * beforeTagBuffer,
//                               int * beforeTagCnt)
// {
//   printf("Error: USmlSource::getNextTag is an interface function\n");
//   return false;
// }
//
// /////////////////////////////////////
//
//
// bool USmlSource::skipToEndTag(USmlTagIn * tag, int msTimeout)
// {
//   printf("Error: USmlSource::skipToEndTag is an interface function\n");
//   return false;
// }
//
// /////////////////////////////////////
//
// bool USmlSource::skipNBytes(int n,  int msTimeout)
// {
//   printf("Error: USmlSource::skipNBytes is an interface function\n");
//   return false;
// }
//
// /////////////////////////////////////
//
// int USmlSource::getNBytes(char * buffer, int n, int msTimeout)
// {
//   printf("Error: USmlSource::getNBytes is an interface function\n");
//   return 0;
// }


bool USmlSource::skipNBytes(int n, int msTimeout)
{
  bool result = true;
  int missing;
  int mc = 0;
  // first remove old data
  if (dataNext != data)
  {
    dataCnt -= (dataNext - data);
    memmove(data, dataNext, dataCnt);
    dataNext = data;
  }
  // test to see if data is missing to do the skip
  if (n > dataCnt)
  {
    missing = n - dataCnt;
    //
    while (missing > 0)
    { //
      dataCnt = getMoreData(data, mini(MAX_CLIENT_RX_BUFFER, missing), msTimeout);
      if (dataCnt <= 0)
      { // timeout, so cancel the operation
        if (doVerboseMessages())
          printf("UClientFuncBase::skipnBytes: timeout cansel\n");
        mc++;
        if (mc > 10)
        {
          missing = 0;
          result = false;
          break;
        }
        Wait(0.005);
      }
      else
        mc = 0;
      missing -= dataCnt;
    }
    dataCnt = 0;
  }
  else if (n < dataCnt)
  { // move the remaining to start of buffer
    dataCnt -= n;
    memmove(data, &data[n], dataCnt);
  }
  else
    // all data just used
    dataCnt = 0;
  return result;
}

////////////////////////////////////////////////////////////////////

bool USmlSource::skipToEndTag(USmlTagIn * tag, int msTimeout)
{
  USmlTag tagEnd;
  bool endFound = false;
  int n;
  const int MVL = 30;
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  int binSize;
  bool result;
  const double secTimeout = msTimeout/1000.0;
  UTime t;
  //
  t.Now();
/*  if (doVerboseMessages())
    tag->print("");*/
  result = tag->isAStartTag();
  while (result and not endFound)
  {
    result = getNextTag(&tagEnd, msTimeout);
    if (not result)
      // timeout - no more data to get, so
      // cancel operation and drop the rest
      break;
    // may be start of a binary message - and this may be
    // special (if format is real binary)
    if (tagEnd.isTagA("bin"))
    { // binary needs special treatment
      binSize = 0;
      while (tagEnd.getNextAttribute(att, val, MVL))
      { // look for size
        if (strcasecmp(att, "size") == 0)
          n = sscanf(val, "%d", &binSize);
      }
      if (n > 0)
        result = skipNBytes(binSize, msTimeout);
      else
        break;
      if (not result)
        // no more data -- cancel operation
        break;
/*      if (doVerboseMessages() and result)
        printf("USmlSource::skipToEndTag: Skipped %d bytes binary data\n", binSize);*/
    }
/*    if (doVerboseMessages())
    {
      if (false)
        printf("USmlSource::skipToEndTag: skipped '%s'\n",
               tagEnd.getTagName());
      else
        // just show skipped message
        tagEnd.print("");
    }*/
    if (tagEnd.isAStartTag())
      skipToEndTag(&tagEnd, msTimeout);
    // test for finished
    if (tagEnd.isTagAnEnd(tag->getTagName()))
      // end tag is found - stop
      break;
    if (t.getTimePassed() > secTimeout)
    { // assume garbage and discard the rest
      clearRxBuffer();
      break;
    }
  }
  if (doVerboseMessages() and not result)
    printf("USmlSource::skipToEndTag: timeout cansel\n");
  return result;
}

////////////////////////////////////////////////////////////////////

bool USmlSource::getToEndTag(USmlTagIn * tag, char * buffer, const int bufferSize, int msTimeout, USmlTagIn * lastTag)
{
  USmlTag tagEnd;
  bool endFound = false;
  bool result;
  const double secTimeout = msTimeout/1000.0;
  UTime t;
  char * p1 = buffer;
  int n = 0, m;
  //
  t.Now();
/*  if (doVerboseMessages())
    tag->print("");*/
  result = tag->isAStartTag();
  while (result and not endFound)
  { // allow space for termination zero
    m = bufferSize - n - 1;
    result = getNextTag(&tagEnd, msTimeout, NULL, p1, &m);
    p1[m] = '\0';
    if (not result)
      // timeout - no more data to get, so
      // cancel operation and drop the rest
      break;
    // test for finished
    if (tagEnd.isTagAnEnd(tag->getTagName()))
    { // end tag is found - stop
      endFound = true;
      break;
    }
    if (t.getTimePassed() > secTimeout)
    { // assume garbage and discard the rest
      clearRxBuffer();
      break;
    }
    // there is more, so
    // prepare where to insert more data
    n += strlen(p1);
    p1 = &buffer[n];
    // add tag itself
    strncpy(p1, tagEnd.getTagStart(), bufferSize - n);
    // there may be more
    n += strlen(p1);
    p1 = &buffer[n];
    //
    if (tagEnd.isAStartTag())
    {
      getToEndTag(&tagEnd, p1, bufferSize - n, msTimeout, &tagEnd);
      n += strlen(p1);
      p1 = &buffer[n];
      // include the end tag too
      strncpy(p1, tagEnd.getTagStart(), bufferSize - n);
      // there may be more
      n += strlen(p1);
      p1 = &buffer[n];
    }
    if ((n < 1) and doVerboseMessages())
      printf("USmlSource::getToEndTag: buffer overflow (>%d bytes) - discarding the rest\n", bufferSize);
  }
  if (doVerboseMessages() and not result)
    printf("USmlSource::getToEndTag: timeout cansel\n");
  return endFound;
}

///////////////////////////////////////////////////////////////

int USmlSource::getNBytes(char * buffer, int n, int msTimeout)
{
  int missing;
  // first remove old data
  if (dataNext != data)
  {
    dataCnt -= (dataNext - data);
    memmove(data, dataNext, dataCnt);
    dataNext = data;
  }
  // test to see if data is missing to do the skip
  missing = n - dataCnt;
  // move first part to buffer
  if (dataCnt > 0)
    memmove(buffer, dataNext, mini(n, dataCnt));
  //
  if (n > dataCnt)
  { // get the rest directly from the line
    dataCnt = getMoreData(&buffer[dataCnt], missing, msTimeout);
    // no unused data left
    missing -= dataCnt; // should be zero
    dataCnt = 0;
    //
    if (false and doVerboseMessages() and (missing != 0))
      printf("UClientFuncBase::getNBytes: timeout got %d/%d\n",
             n - missing, n);
  }
  else if (n < dataCnt)
  { // move the remaining data to start of buffer
    dataCnt -= n;
    memmove(data, &data[n], dataCnt);
    missing = 0;
  }
  else
    // no data left
    dataCnt = 0;
  return n - missing;
}

///////////////////////////////////////////////////////////////

void USmlSource::clearRxBuffer()
{
  dataCnt = 0;
  dataNext = data;
}

///////////////////////////////////////////////////////////////

bool USmlSource::getNextTag(USmlTag * tag, int msTimeout,
                                USmlTagIn * failEndTag,
                                char * beforeTagBuffer, int * beforeTagCnt)
{
  bool result = false;
  int got, n, m;
  int beforeBuffer = 0;
  bool bufferFull = false;
  const char * p1;
  //
  if (beforeTagCnt != NULL)
  {
    beforeBuffer = *beforeTagCnt;
    *beforeTagCnt = 0;
  }
  if (errorBuffer != NULL)
    errorBuffer[0] = '\0';
  // try to find a tag in the remaining data
  while (not result)
  { // until a full tag is found (including tag end), or a timeout occured
    if (dataNext != data)
    { // there is old data - remove
      dataCnt -= (dataNext - data);
      if (absi(dataCnt) > MAX_CLIENT_RX_BUFFER)
      {
        result = false;
        printf("USmlSource::getNextTag:"
            " *** OOPS probably a bad case of tramp\n");
        printf("Last tag is %d long: '%s'\n", tag->getTagCnt(), tag->getTagStart());
        printf("Data is %d long: '%s'\n", strlen(data), data);
        //break;
      }
      memmove(data, dataNext, dataCnt);
      dataNext = data;
    }
    data[dataCnt] = '\0';
    // find start tag
    dataNext = strchr(data, '<');
    result = (dataNext != NULL);
    if (result and tagStartAtStartOfLineOnly)
    { // test for non white space before dataNext
      // if so, then ignore this start of tag - is likely a part of a statement
      p1 = data;
      while (isspace(*p1) and (p1 < dataNext))
        p1++;
      if (*p1 != '<')
      {
        result = false;
        syntaxError("Stray '<', replaced by less than XML character '&lt;'");
      }
    }
    // tag message is needed?
    if ((beforeTagBuffer != NULL) and (dataCnt > 0))
    { // save a bufferfull of received data
      n = dataCnt;
      if (result)
        n = dataNext - data;
      // move (additional) m bytes data to buffer
      m = mini(n, beforeBuffer - *beforeTagCnt);
      memmove(&beforeTagBuffer[*beforeTagCnt], data, m);
      *beforeTagCnt += m;
      if (n > m)
      { // more data available than space in buffer
        // return a buffer-full and wait with the rest
        dataNext = &data[m];
        tag->setValid(false);
        bufferFull = true;
      }
    }
    if (not bufferFull)
    {
      if (not result)
      { // disregard the lot if not wanted by beforeTagBuffer
        if (not gotFirstTag and (dataCnt > 0))
        { // may be a non-tag communication - print before discard
          data[dataCnt] = '\0';
          printf("%s", data);
        }
        dataCnt = 0;
        dataNext = data;
      }
      else
      { // test for full tag available
        tag->setTag(dataNext, dataCnt - (dataNext - data));
        result = tag->isTagEndFound();
        if (result)
          gotFirstTag = true;
      }
    }
    // -- need data?
    if (bufferFull or not isSourceAvailable())
      // no more data is available or buffer is full - stop
      break;
    if (not result)
    { // get more data, as no full tag is found
      got = getMoreData(&data[dataCnt], MAX_CLIENT_RX_BUFFER - dataCnt,
                             msTimeout);
      if (got == 0)
      { // timeout - no more data to get, so
        // cancel operation and drop the rest
        dataNext = data;
        if (not gotFirstTag and (dataCnt > 0))
        { // may be a non-tag communication - just print
          data[dataCnt] = '\0';
          printf("%s", data);
        }
        dataCnt = 0;
        break;
      }
      else if (got < 0)
      { // no more data (end of file)
        break;
      }
      else
        dataCnt += got;
    }
  }
  if (result and tag->isValid())
  { // move next to passed the read tags '>'
    dataNext = tag->getToEnd();
    dataNext++; // advance to past '>' character.
    if (dataNext < data)
    {
      printf("USmlSource::getNextTag:"
          " *** OOPS probably a bad case of tramp\n");
      // force protection error here
      //data[-1000000] = '!';
    }
    tag->setCnn(this);
  }
  // test also for specific end tag, and return false if found
  if (result and failEndTag != NULL)
    result = not tag->isTagAnEnd(failEndTag->getTagName());
  // return result
  return (result or bufferFull) and isSourceAvailable();
}

///////////////////////////////////////////////////////

int USmlSource::getMoreData(char * buffer, int bufferSize, int pollTimeoutMs)
{
  printf("Error, USmlSource::getMoreData is an interface funcrtion\n");
  return -1;
}

//////////////////////////////////////////////////////

bool USmlSource::doVerboseMessages()
{
  return false;
}

//////////////////////////////////////////////////////
bool USmlSource::isSourceAvailable()
{
  return false;
}

///////////////////////////////////////////////////////

bool USmlSource::outputData(const char * message)
{
  printf("USmlSource::outputData: not send: '%s'\n", message);
  return false;
}

///////////////////////////////////////////////////////

void USmlSource::syntaxError(const char * message)
{
  const int MSL = 200;
  char s[MSL];
  //
  snprintf(s, MSL, "Syntax error: %s", message);
  if (isVerbose())
    printf("%s\n", s);
  if (errorBuffer != NULL)
  {
    strncat(errorBuffer, s, errorBufferCnt);
  }
}

////////////////////////////////////////////////////////

void USmlSource::setErrorBuffer(char * buf, const int bufCnt)
{
  errorBuffer = buf;
  errorBufferCnt = bufCnt;
}

