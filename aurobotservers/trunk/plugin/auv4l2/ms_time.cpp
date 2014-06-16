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


#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include "ms_time.h"
#include <pthread.h>

/*------------------------------ get time ------------------------------------*/
/*in miliseconds*/
DWORD ms_time (void)
{
	struct timeval tod;
	gettimeofday(&tod, NULL);
	DWORD mst = (DWORD) tod.tv_sec * 1000 + (DWORD) tod.tv_usec / 1000;
	return (mst);
}
/*in microseconds*/
UINT64 us_time(void)
{
    struct timeval tod;
    gettimeofday(&tod, NULL);
	UINT64 ust = (UINT64) tod.tv_sec * 1000000 + (UINT64) tod.tv_usec;
	return (ust);
}

/*REAL TIME CLOCK*/
/*in nanoseconds*/
UINT64 ns_time (void)
{
	static struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return ((UINT64) ts.tv_sec * G_NSEC_PER_SEC + (UINT64) ts.tv_nsec);
}

/*MONOTONIC CLOCK*/
/*in nanosec*/
UINT64 ns_time_monotonic()
{
	static struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ((UINT64) ts.tv_sec * G_NSEC_PER_SEC + (UINT64) ts.tv_nsec);
}

//sleep for given time in ms
void sleep_ms(int ms_time)
{
	UINT64 sleep_us = ms_time *1000; /*convert to microseconds*/
	usleep( sleep_us );/*sleep for sleep_ms ms*/
}

/*wait on cond by sleeping for n_loops of sleep_ms ms (test var==val every loop)*/
/*return remaining number of loops (if 0 then a stall occurred)              */
int wait_ms(gboolean* var, gboolean val, pthread_mutex_t *mutex, int ms_time, int n_loops)
{
	int n=n_loops;
	pthread_mutex_lock(mutex); // g_mutex_lock(mutex);
		while( (*var!=val) && ( n > 0 ) ) /*wait at max (n_loops*sleep_ms) ms */
		{
			pthread_mutex_unlock(mutex); // g_mutex_unlock(mutex);
			n--;
			sleep_ms( ms_time );/*sleep for sleep_ms ms*/
			pthread_mutex_unlock(mutex); // g_mutex_lock(mutex);
		};
	pthread_mutex_unlock(mutex); //g_mutex_unlock(mutex);
	return (n);
}

