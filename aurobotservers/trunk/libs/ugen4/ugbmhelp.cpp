 /***************************************************************************
//  ugbmhelp.cpp  
    -------------------
    Christian Andersen  
		chrand@mail.dk
 ***************************************************************************/
// Function:
// Conversion to/from windows 24bit bitmap
// from http://www.nyangau.fsnet.co.uk
//
// History:
// chr 1 apr 2002 Modified from General Bitmap Module
//

/*

gbmhelp.c - Helpers for GBM file I/O stuff

*/

/*...sincludes:0:*/
#include <stdio.h>
#include <ctype.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#define LINUX
#if defined(AIX) || defined(LINUX) || defined(MAC)
#include <unistd.h>
#else
#include <io.h>
#endif
#include <fcntl.h>
#ifdef MAC
#include <types.h>
#include <stat.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#endif
#include "ugbm.h"
#include "ugbmhelp.h"

/*...vgbm\46\h:0:*/
/*...e*/

/*...sgbm_same:0:*/
bool gbm_same(const char *s1, const char *s2, int n)
	{
	for ( ; n--; s1++, s2++ )
		if ( tolower(*s1) != tolower(*s2) )
			return false;
	return true;
	}
/*...e*/
/*...sgbm_find_word:0:*/
const char *gbm_find_word(const char *str, const char *substr)
	{
	char buf[100+1], *s;
	int  len = strlen(substr);

	for ( s  = strtok(strcpy(buf, str), " \t,");
	      s != NULL;
	      s  = strtok(NULL, " \t,") )
		if ( gbm_same(s, substr, len) && s[len] == '\0' )
			{
			int inx = s - buf;
			return str + inx;
				/* Avoid referencing buf in the final return.
				   lcc and a Mac compiler see the buf, and then
				   warn about possibly returning the address
				   of an automatic variable! */
			}
	return NULL;
	}
/*...e*/
/*...sgbm_find_word_prefix:0:*/
const char *gbm_find_word_prefix(const char *str, const char *substr)
	{
	char buf[100+1], *s;
	int  len = strlen(substr);

	for ( s  = strtok(strcpy(buf, str), " \t,");
	      s != NULL;
	      s  = strtok(NULL, " \t,") )
		if ( gbm_same(s, substr, len) )
			{
			int inx = s - buf;
			return str + inx;
				/* Avoid referencing buf in the final return.
				   lcc and a Mac compiler see the buf, and then
				   warn about possibly returning the address
				   of an automatic variable! */
			}
	return NULL;
	}
/*...e*/
/*...sgbm_file_\42\:0:*/
/* Looking at this, you might think that the gbm_file_* function pointers
   could be made to point straight at the regular read,write etc..
   If we do this then we get into problems with different calling conventions
   (for example read is _Optlink under C-Set++ on OS/2), and also where
   function arguments differ (the length field to read is unsigned on OS/2).
   This simplest thing to do is simply to use the following veneers. */

static int def_open(const char *fn, int mode)
	{ return open(fn, mode); }
static int def_create(const char *fn, int mode)
#ifdef MAC
	{ return open(fn, O_CREAT|O_TRUNC|mode); }
		/* S_IREAD and S_IWRITE won't exist on the Mac until MacOS/X */
#else
	{ return open(fn, O_CREAT|O_TRUNC|mode, S_IREAD|S_IWRITE); }
#endif
static void def_close(int fd)
	{ close(fd); }
static long def_lseek(int fd, long pos, int whence)
	{ return lseek(fd, pos, whence); }
static int def_read(int fd, void *buf, int len)
	{ return read(fd, buf, len); }
static int def_write(int fd, const void *buf, int len)
#ifdef MAC
	/* Prototype for write is missing a 'const' */
	{ return write(fd, (void *) buf, len); }
#else
	{ return write(fd, buf, len); }
#endif

int  (*gbm_file_open  )(const char *fn, int mode)         = def_open  ;
int  (*gbm_file_create)(const char *fn, int mode)         = def_create;
void (*gbm_file_close )(int fd)                           = def_close ;
long (*gbm_file_lseek )(int fd, long pos, int whence)     = def_lseek ;
int  (*gbm_file_read  )(int fd, void *buf, int len)       = def_read  ;
int  (*gbm_file_write )(int fd, const void *buf, int len) = def_write ;
/*...e*/
/*...sreading ahead:0:*/

AHEAD *gbm_create_ahead(int fd)
	{
  AHEAD *ahead;

	if ( (ahead = (AHEAD*) malloc((size_t) sizeof(AHEAD))) == NULL )
		return NULL;

	ahead->inx = 0;
	ahead->cnt = 0;
	ahead->fd  = fd;

	return ahead;
	}

void gbm_destroy_ahead(AHEAD *ahead)
	{
	delete ahead;
	}

int gbm_read_ahead(AHEAD *ahead)
	{
	if ( ahead->inx >= ahead->cnt )
		{
		ahead->cnt = gbm_file_read(ahead->fd, (char *) ahead->buf, AHEAD_BUF);
		if ( ahead->cnt <= 0 )
			return -1;
		ahead->inx = 0;
		}
	return (int) (unsigned int) ahead->buf[ahead->inx++];
	}
/*...e*/



UgbmHelp::UgbmHelp(){
}
UgbmHelp::~UgbmHelp(){
}
