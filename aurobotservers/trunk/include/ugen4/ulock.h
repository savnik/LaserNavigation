/***************************************************************************
 *   Copyright (C) 2006 by DTU                                             *
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
#ifndef ULOCK_H
#define ULOCK_H

#include <semaphore.h>
#include <pthread.h>
#include <errno.h>
#include "utime.h"

/**
Just a pthread mutex lock packed in a class,
to be used e.g. when another class needs lock protection.
@author Christian Andersen */
class ULock{
public:
  /**
  Constructor */
  ULock();
  /**
  Destructor */
  ~ULock();
  /**
  Initialize the lock to fast mutex. */
  inline void lockInit()
    { // init to default (fast) mutes
      // with no error check (not much)
      pthread_mutex_init(&mLock, NULL);
    };
  /**
  semaphore lock for this image. */
  inline bool lock()
    { return (pthread_mutex_lock(&mLock) == 0); };
  /**
  semaphore to wait for a post signal. */
  inline bool wait()
  { return (pthread_mutex_lock(&mLock) == 0); };
  /**
  semaphore unlock for this image. */
  inline void unlock()
    {  pthread_mutex_unlock(&mLock);};
  /**
  semaphore to post this real time flag. */
  inline void post()
  {  pthread_mutex_unlock(&mLock);};
  /**
  semaphore unlock for this image.
  \returns true if the resource is locked (else returns false). */
  inline bool tryLock()
    { return (pthread_mutex_trylock(&mLock) == 0); };
  /**
  semaphore check for a post.
  \returns true if the post is accepted, and false if nothing is posted. */
  inline bool tryWait()
  { return (pthread_mutex_trylock(&mLock) == 0); };

private:
  /** lock, when reading or writing */
  pthread_mutex_t mLock; // pthread_mutex_lock

};


/**
 * A small class that encapsulates semaphores
 * The semaphore is initialized as posted with one value,
 * i.e. the first call to wait will be sucessfull immidiately.
 * If this is undesired, then call a trywait() (or wait()) to clear */
class USemaphore
{
private:
  /// the semaphore itself
  sem_t sem;
public:
  /** constructor */
  inline USemaphore()
  { // initialized  as posted with one value,
    sem_init(&sem, 0, 1);
  };
  inline ~USemaphore()
  {
    sem_destroy(&sem);
  }
  /**
   * Wait for another thread sends a post
   * \returns true when the semaphore is posted.
   * \returns false if interrupted by a signal (e.g. ctrl-C) */
  inline bool wait()
  {
    return sem_wait(&sem) == 0;
  }
  /**
   * Tests if the semaphore is already postet, 
   * \returns true, if semaphore is posted, and then the post is cleared.
   * \returns false if not posted yet. */
  inline bool trywait()
  {
    return sem_trywait(&sem) == 0;
  }
  /**
   * Wait until semaphore is posted ot until this absolute time is passed
   * \param until is the timelimit for the wait.
   * \returns 0 on error, 1 on semaphore is posted, 2 if semaphore reached timeout */
  inline bool wait(UTime * until)
  {
    timespec t = {until->time.tv_sec, until->time.tv_usec};
    int err = sem_timedwait(&sem, &t);
    if (err == 0)
      return 1;
    else
    {
      if (errno == ETIMEDOUT)
        return 2;
      else
        // other erro, e.g. interrupt by signal (e.g. ctrl-C)
        return 0;
    }
  }
  /**
   * clears all posted values, leaving the semaphore un-posted */
  inline void clearPosts()
  { // removes posts until trywait returns false
    while (trywait())
      ;
  }
  /**
   * post to this semaphore - cam release a waiting thread
   * If you may have more posts than waits, then it may be a good idea to clear 
   * any unused posts - using clearPosts() - before posting.
   * \returns true, except for extreem cases of overflow of posts. */
  bool post()
  {
    return sem_post(&sem) == 0;
  }
};

#endif
