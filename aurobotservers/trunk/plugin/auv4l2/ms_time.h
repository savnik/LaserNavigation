/*******************************************************************************#
#           guvcview              http://guvcview.berlios.de                    #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/
#ifndef MS_TIME_H
#define MS_TIME_H

//#include <glib.h>
#include "defs.h"

#ifndef G_NSEC_PER_SEC
#define G_NSEC_PER_SEC 1000000000
#endif

/*time in miliseconds*/
DWORD ms_time (void);
/*time in microseconds*/
UINT64 us_time(void);
/*time in nanoseconds (real time for benchmark)*/
UINT64 ns_time (void);
/*MONOTONIC CLOCK in nano sec for time stamps*/
UINT64 ns_time_monotonic();

/*sleep for given time in ms*/
void sleep_ms(int ms_time);

/*wait on cond by sleeping for n_loops of sleep_ms ms */
/*(test (var == val) every loop)                      */
int wait_ms(gboolean* var, gboolean val, pthread_mutex_t *mutex, int ms_time, int n_loops);
#endif

