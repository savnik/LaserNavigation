/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen                          *
 *   jca@elektro                                                           *
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
#ifndef USMLFILE_H
#define USMLFILE_H

#include <ugen4/conf.h>
#include "usmlsource.h"

/**
File source for SML (XML) source

	@author Christian Andersen <chrand@mail.dk>
*/
class USmlFile : virtual public USmlSource
{
public:
  /**
   * Constructor */
  USmlFile();
  /**
   * Destructor */
  ~USmlFile();

  /**
   * Open file. If file is open, then old file will be closed first. */
  bool openSmlFile(const char * filename);
  /** close SML (XML) file */
  void closeSmlFile();
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
   * Get source name (filename) pointer */
  virtual const char * getSourceName()
  { return fn; };

protected:
  /**
   * Get more data, if more data is available */
  virtual int getMoreData(char * buffer, int bufferSize, int pollTimeoutMs);
  /**
   * Should additional messaves be printed to console */
  virtual bool doVerboseMessages();
  /**
   * Is tha data source (still) open.
   * \returns true if open */
  virtual bool isSourceAvailable();


protected:
  /** file handle for XML type file */
  FILE * fh;
  /** file name */
  char fn[MAX_FILENAME_LENGTH];
  /** Should more debug data be output to console */
  bool verbose;
  /** current line number */
  int line;
};

#endif
