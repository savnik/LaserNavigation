/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   jca@elektro.dtu.dk   *
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
#ifndef USMLSOURCE_H
#define USMLSOURCE_H

#include <stdio.h>

/**
Longest tag send to client, i.e. from '<' to '>' (not from <...> to </...>)
(both included), but a longer buffer may be an advantage
for skipping large binary blocks. */
#define MAX_CLIENT_RX_BUFFER 10000

class USmlTag;
class USmlTagIn;

/**
Interface class for a SML (XML) source

	@author Christian Andersen <chrand@mail.dk>

Class that handles input from (any) source and packs into SML (XML)
 */
class USmlSource
{
  public:
  /**
   * Constructor */
    USmlSource();
  /**
   * Constructor */
    virtual ~USmlSource();
  /**
   * Got an unusable message - just skip to other end.
   * \param tag should point to the start tag, of which the content is unwanted.
   * \param timeout is in miliseconds, when waiting for new data.
   * \Returns true when relevant end-tag is removed from the stream or false when a
   * period of 'msTimeout' has passed with no data.*/
  virtual bool skipToEndTag(USmlTagIn * tag, int msTimeout = 100);
  /**
   * Get all data to a buffer until the matching end tag.
   * \param tag the start tag from where to start (start tag is not included)
   * \param buffer is where to store the data, the buffer string is zero terminated.
   * \param bufferSize number of bytes available in buffer
   * \param msTimeout maximum waittime between data bursts (if from socket)
   * \param lastTag may be a pointer to a tag, if the end tag (or the last) is of interest
   * \returns true if the end tag is found. */
  bool getToEndTag(USmlTagIn * tag, char * buffer, const int bufferSize,
                   int msTimeout = 100, USmlTagIn * lastTag = NULL);
  /**
   *  Skip N bytes (from a binary message).
   * \Returns true if n bytes skipped.
   * \Returns false if timeout or source is down. */
  virtual bool skipNBytes(int n,  int msTimeout = 100);
  /**
    Get the next n bytes loaded into the provided buffer.
    N may be of any length (that can be received within the timeout
    period).
    \Returns the number of bytes loaded into the buffer.
    If source is invalid or no data is available zero is returned. */
  virtual int getNBytes(char * buffer, int n, int msTimeout = 100);
  /**
   * Read the next available tag into the provided buffer.
   * \param tag, The found tag is loaded into the 'tag' buffer.
   * \param beforeTagBuffer is (if != NULL) filled with the data before the next tag.
   * \param beforeTagCnt is on entry the buffer size. The number of bytes
   * filled into the buffer returned into 'beforeTagCnt'.
    If there is more data before the next tag than the space in 'beforeTagBuffer' then
    the function returns with 'beforeTagBuffer' equal to the buffer size and
    the function returnes true.
    Note the buffer is NOT nul-terminated (to allow binary data), so the user
    must add a nul at the 'beforeTagCnt' position if the result is used as a string.
    \param failEndTag The search is also stopped if an end-tag is found matching the 'failEndTag'
    (makes the function return false).
    \Returns true if tag is valid (and the beforeTagBuffer has no overflow). */
  virtual bool getNextTag(USmlTag * tag, int msTimeout = 100,
                    USmlTagIn * failEndTag = NULL,
                    char * beforeTagBuffer = NULL,
                    int * beforeTagCnt = NULL);
  /**
    * Should print to console be verbose. */
  bool isVerbose()
  { return doVerboseMessages(); };
  /**
    * Output a reply to the source (if possible, e.g. a socket connection) */
  virtual bool outputData(const char * message);
  /**
    * A syntax error has occured with the provided message
    * \param message is a description of the error type. */
  virtual void syntaxError(const char * message);
  /**
    * Set error buffer
    * Set a (long) character buffer where to save e.g. syntax error for later display
    * \param buf pointer to character array area (set to NULL if buffer is not valid)
    * \param bufCnt size of buffer area */
  void setErrorBuffer(char * buf, const int bufCnt);
  /**
   * Get error buffer pointer */
  const char * getErrorBuffer()
  { return errorBuffer;};
  /**
   * Is there an error text */
  bool isErrorText()
  {
    if (errorBuffer != NULL)
      return errorBuffer[0] != '\0';
    else
      return false;
  };
  /**
    * Get current line number from the source
    * \returns -1 if line numbers are unavalable */
  virtual int getLineNumber()
  { return -1; };
  /**
   * Set current line number from the source
   * \param newLineNumber may be used to set a new series of line numbers */
  virtual void setLineNumber(int newLineNumber)
  { };
  /**
    * Get source name pointer */
  virtual const char * getSourceName()
  { return "noname"; };
  /**
   * Allow tagStartAt start of line only */
  void allowTagAtStartOfLineOnly()
  { tagStartAtStartOfLineOnly = true; };
  /**
   * Is tha data source (still) open.
   * \returns true if open */
  virtual bool isSourceAvailable();
  /**
  Get receive data buffer */
  char * getRxDataBuffer()
  { return data; };
  /**
  Get size of rx data buffer */
  int getRxDataBufferCnt()
  { return limitedMaxDataLength; };
  /**
  Get size of rx data buffer */
  int setRxDataBufferCnt(int newMaxLength)
  {
    limitedMaxDataLength = mini(newMaxLength, MAX_CLIENT_RX_BUFFER); 
    return limitedMaxDataLength; };

protected:
  /**
    * Get more data, if more data is available */
  virtual int getMoreData(char * buffer, int bufferSize, int pollTimeoutMs);
  /**
    * Should additional messaves be printed to console */
  virtual bool doVerboseMessages();

  /**
  Discard all data in buffer -- assumed garbage, e.g server reply error */
  void clearRxBuffer();

public:
  /**
    Pointer to interface name - to be used in data source tracking (mostly for debug and status) */
  const char * ifName;

protected:
  /**
    Valid data in data buffer - relative to buffer start */
  int dataCnt;
  /**
    Flag for receptionof the first tag */
  bool gotFirstTag;
  /** buffer for syntax error - must be set by caller*/
  char * errorBuffer;
  /** size of buffer for syntax error */
  int errorBufferCnt;
  /** ignore tag start if not at start of line (may be preceded by whitespace only) */
  bool tagStartAtStartOfLineOnly;

private:
  /**
    Pointer to next unused data */
  const char * dataNext;
  /**
    Buffer for new data from socket,
    plus space for a zero when searching for '<' and '>' */
  char data[MAX_CLIENT_RX_BUFFER + 1];
  /**
  Used number of bytes (maximum in one read) of the data buffer.
  This may be set by the user to less than the full buffer.
  Mostly used for binary fixed length messages. */
  int limitedMaxDataLength;
};

#endif
