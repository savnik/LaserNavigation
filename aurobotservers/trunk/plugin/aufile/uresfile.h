/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
 *   jca@elektro.dtu.dk                                                    *
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

#ifndef URESFILE_H
#define URESFILE_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarcalc.h>


class UFileItem
{
public:
  /**
   * Constructor */
  UFileItem();
  /**
   * Destructor */
  ~UFileItem();
  /**
   * Add some text to the file */
  bool add(const char * text);
  /**
   * Set filename, NB must be a qualified filename and subdirs are not allowed
   * The global variable file.fileDir controls the directory, where files are placed. */
  void setName(const char * newName);
  /**
   * is the file open */
  bool isOpen();
  /**
   * Close the file (if open).
   * \returns true if the file was closed or false if file was not open */
  bool closeFile();
  /**
   * Open the file for write (by calls to add("text"))
   * If file is open already, then it is closed.
   * \param fileDir is the directory where to put the file (may be "." or "./" for current dir)
   * \param newName if not NULL, then the file name is changed to the newName before opening.
   * \returns true if file was created. */
  bool openFile(const char * fileDir, const char * newName);


private:
  /**
   * File handle - used when file is open */
  FILE * fh;
public:
  /**
   * MAX filename length */
  static const int MFNL = MAX_FILENAME_LENGTH;
  /**
   * File name in current directory */
  char filename[MFNL];
  /**
   * Client thet owns this file handle. */
  int client;
};

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendents) as shown.

@author Christian Andersen
*/
class UResFile : public UResVarPool, public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResFile) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResFile()
  { // set name and version
    setResID("file", 675);
    UResFileInit();
  };
  /**
  Destructor */
  virtual ~UResFile();
  /**
   * Initialize resource */
  void UResFileInit();
  
public:
  // Public functions that this resource provides for all users.
  /**
   * Delete a file
   * \param name is the name of the file - in current directory
   * \returns true if deleted poly, false if not existed. */
  bool del(const char * name);
  /**
   * Add some text to the file
   * \param client is the client trying to add some text to a file.
   * \param text is the name text to add, is assumed to be coded with XML escapes, that will be removed.
   * \returns true if added, false if no file is open. */
  bool add(const int client, const char * text);
  /**
   * Get list of current files in the directory
   * \param preStr is a short string, that is added at the start of the buffer.
   * \param buff is the buffer, where to write the list.
   * \param buffCnt is the size of the buffer
   * \returns a pointer to buff */
  const char * getList(const char * preStr, char * buff, const int buffCnt);
  /**
   * Get name of directory */
  const char * getDirName();
  /**
   * Open a new file
   * \param client the client number of the client creating the file
   * \param newName is the new name of the file
   * \returns true if the file was opened successfully */
  bool newFile(const int client, const char * newName);
  /**
   * close a file (after all text is added)
   * \param client is the owner of the file (one file can be open per client only)
   * \returns true if file was open and is now closed, false if file was not open. */
  bool closeFile(const int client);
  /**
   * Get full filename for the item into provided buffer
   * \param int client if the file owner
   * \param preStr is a string to be placed before the filename
   * \param buff is the buffer, where to place the filename.
   * \param buffCnt is the length of the provided buffer
   * \returns a pointer to the buffer. */
  const char * getFilename(const int client, const char * preStr, char * buff, const int buffCnt);
  /**
   * Get filename (no path) for the item
   * \param int client if the file owner
   * \returns a pointer to the filename. */
  const char * getFilename(const int client);
  /**
   * Set global variable to the current number of open files.
   * \returns the number of open files. */
  int setOpenFilesCnt();


protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
   * Get the open file item for this client.
   * \param client  is the client number (from message).
   * \param mayCreate if true and no file itemexist for the client, then it is created.
   * \returns a pointer to the file item, or NULL if none exist (or not created). */
  UFileItem * getItem(const int client, bool mayCreate);

public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /// maximum number of files
  static const int MAX_FILE_CNT = 100;
  /// array of poly handles
  UFileItem * openFiles[MAX_FILE_CNT];
  /// Number of polys allocated
  int openFilesCnt;
  //
  /// directory for the files
  UVariable * varFileDir;
  /// number of open files
  UVariable * varOpenFiles;
};

#endif

