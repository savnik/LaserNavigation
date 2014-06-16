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

#ifndef URESRHDIF_H
#define URESRHDIF_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarpool.h>
extern "C"
{
#include <rhd/rhd.h>
}

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendents) as shown.

@author Christian Andersen
*/
class UResRhdIf : public UResVarPool, public UServerPush
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResRhdIf) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResRhdIf()
  { // set name and version
    setResID("rhd", 485);
    UResRhdIfInit();
  };
  /**
  Destructor */
  virtual ~UResRhdIf();
  /**
   * Initialize resource */
  void UResRhdIfInit();

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
   * Get current host name */
  const char * getHost()
  { return varHost->getValues();};
  /**
   * Get current port number
   * \return the current selected port number (connected or not) */
  int getPort()
  { return varPort->getInt(); };
  /**
   * Is RDH connected
   * \returns true if connected, else false. */
  bool isConnected()
  { return varConnected->getBool(); };
  /**
   * Connect or stop connection to RHD
   * \param value if true, try to connect, if connected already, then do nothing
   * \param value if false, then stop connection (hang-up) if not already stopped.
   * \param keep try to reconnect if connection is lost
   * \returns true if successful */
  bool tryConnect(bool aConnect, bool keep);
  /**
   * Set host name
   * \param hostName is a c-string with new host name */
  void setHost(const char * hostName)
  { varHost->setValues(hostName, 0, true); };
  /**
   * Set host name
   * \param hostName is a c-string with new host name */
  void setPort(int port)
  { varPort->setInt(port); };
  /**
   * Run the reed loop to maintain data from RHD
   * Is called from a C function, and therefore public */
  void run();
  /**
   * Set write request flag (for next connect)
   * \param value if true, then a write request will be made. */
  void setWriteable(bool value)
  { varWriteAccess->setBool(value); };
  /**
   * is write request flag (for next connect) set - not the same as write granted.
   * \returns true, if write request flag is set. */
  bool isWriteRequested()
  { return varWriteAccess->getBool(); };
  /**
   * is write request granted.
   * \returns true, if write granted flag is set. */
  bool isWriteGranted()
  { return varWriteGranted->getBool(); };

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
  Start read thread
  \return Returns true if the read thread started. */
  bool start();
  /**
   * Stop read thread - and wait for thread join
   * \param andWait wait for thread to finish (ignored, waits always)
  */
  void stop(bool andWait);

public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
  bool threadRunning;
  /**
  Should thread stop - terminate */
  bool threadStop;
  /// Number of keeps allocated
  int keepsCnt;
  /// sample time of the RHD
  UVariable * varSampleTime;
  /// current host name
  UVariable * varHost;
  /// current port number
  UVariable * varPort;
  /// is RHD connected */
  UVariable * varConnected;
  /// should connection be reestablished if not connected
  UVariable * varKeep;
  /// max number of RHD variables
  static const int MAX_VARS_CNT = 500;
  /// array of variables for RHD variables
  UVariable * vars[MAX_VARS_CNT];
  /** number of used pointers in varRHD */
  int varsCnt;
  UVariable * varWriteAccess;
  UVariable * varWriteGranted;
};

#endif

