/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
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
#ifndef USEQLOOP_H
#define USEQLOOP_H

#include "usequencer.h"

//class SRMain;
//class UCmdExe;

/**
MMR specific add-on to sequencer

@author Christian Andersen
*/
class USeqLoop : public USequencer
{
public:
  /**
  Constructor */
  USeqLoop();
  /**
  Destructor */
  ~USeqLoop();
  /**
  Reset to no plan, ready to start, when a cmd line
  gets available */
  void reset();
  /**
  Reset to no plan using reset(), but
  using a locked state of sequencer */
  void clear();
  /**
  Start thread, that services the planned commands.
  Returns true if started. */
  bool start();
  /**
  Stop plan execution */
  inline void stop()
  { terminate = true; };
  /**
  Run plan execution */
  void run();
  /**
  Get last loop start time */
  inline UTime getLoopStartTime()
  { return loopStartTime; };
  /**
  Is sequencer thread running flag set */
  inline bool isRunning()
  { return running; };
  /**
  Is sequencer idle - waiting for a stop criteria, or out of lines */
  inline bool isIdle()
  { return idle; };
  /**
  Is called when sequencer has active stuff to do */
  virtual inline void sequencerUpdate()
  {};



protected:
  /**
  Handle to main server resource */
  //SRMain * srObj;
  /**
  Handle to camera server connection */
  /**
  Is sequencer idle - waiting for a stop criteria */
  bool idle;
  /**
  Connection to server for access to clients */
  //UCmdExe * cmdexe;

protected:
  /**
  terminate control loop.
  Should be false at all times, except at shutdown. */
  bool terminate;
  /**
  Is the thread running */
  bool running;
  /**
  Starttime of last loop */
  UTime loopStartTime;
  /**
  Nominel sample time for running a execution loop */
  double sampleTime;
  /**
  Line analyzed, with stopcondirtions set for testing (if drive command) */
  int lineParsed;
  /**
  Thread handle for loop-thread */
  pthread_t threadLoop;
};

#endif
