/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen                          *
 *   chrand@mail.dk                                                        *
 *        vvvvvv                                                           *
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
#ifndef USMLSTRING_H
#define USMLSTRING_H

#include "usmlsource.h"

/**
An sml stream based on a stringh source

	@author Christian Andersen <chrand@mail.dk>
*/
class USmlString : public USmlSource
{
public:
  USmlString();

  ~USmlString();
  /**
   * Set the source string.
   * \param editString is the source string, ordered in lines separated by '\n'
   * \returns true */
  bool setSourceString(const char * editString);
  /**
   * A syntax error has occured with the provided message
   * \param message is a description of the error type. */
  virtual void syntaxError(const char * message);
  /**
   * Set if the console messages should be verbose */
  void setVerbose(bool value)
  { verbose = value; };
  /**
   * Get current line number from the source - first line is 1
   * \returns -1 if line numbers are unavalable (file not open) */
  virtual int getLineNumber()
  { return line; };
  /**
   * Set current line number from the source
   * \param newLineNumber may be used to set a new series of line numbers */
  virtual void setLineNumber(int newLineNumber)
  { line = newLineNumber; };
  /**
   * Is tha data source (still) open, and is ther more unread data.
   * \returns true if more data is available */
  virtual bool isSourceAvailable();

protected:
  /**
   * Get more data, if more data is available */
  virtual int getMoreData(char * buffer, int bufferSize, int pollTimeoutMs);
  /**
   * Should additional messages be printed to console */
  virtual bool doVerboseMessages();


protected:
  /** pointer to XML lines to be added */
  const char * fullSource;
  /**
   * Current position in source */
  const char * source;
  /** Should more debug data be output to console */
  bool verbose;
  /** current line number */
  int line;
};

#endif
