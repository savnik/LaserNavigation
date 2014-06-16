#if 0
#include <stdio.h>
#endif
#include <string.h>

#include "command_store.h"

#define CS_STORAGE_SIZE 3000
/* The number of characters available in database. */

static char cs_buf[CS_STORAGE_SIZE] = {0}; /* The storage area. */
char * first_free_ptr = cs_buf; /* The pointer to the first free location. */

void CS_reset(void)
{
  unsigned long i = 0;
  
  /* Clear the storage area - this could be done using Std. C Library function as well. */
  for (i=0; i < CS_STORAGE_SIZE; i++) {
    cs_buf[i] = 0;
  }

  /* Reset the pointer to next free location to the start of the area. */
  first_free_ptr = cs_buf;
}

const char * CS_store(const char * xml_attr_cmd_str)
{
  const char * last_char = &(cs_buf[CS_STORAGE_SIZE - 1]); /* Address of last char in storage area. */
  char * dest_ptr = first_free_ptr;
  const char * source_ptr = xml_attr_cmd_str;
  int done = 0;
  int copy_error = 0;
  const char * rp = NULL;

  if (xml_attr_cmd_str) {
    /* It was not a NULL pointer which was passed. Thus start copying. */

    while (!done) {
      if (*source_ptr) {
	/* End of string not yet seen. Thus check for free space in destination. */
	if (dest_ptr < last_char) {
	  /* Still free space in the storage. Thus append character. */
	  const char ch = *source_ptr;

	  /* Check for MATLAB string literal marker. */
	  if ('\'' == ch) {
	    /* Yes, MATLAB string literal marker found, replace it with '"'. */
	    *dest_ptr = '"';
	  } else {
	    /* No, it was not a MATLAB style string literal marker, copy as is. */
	    *dest_ptr = ch;
	  }

	  /* Prepare for next iteration - advance pointers. */
	  source_ptr++;
	  dest_ptr++;
	} else {
	  /* Storage full! */
	  done = 1;
	  copy_error = 1;
	}
      } else {
	/* Terminating zero in source string encountered. Thus last iteration of loop. */
	done = 1;

	/* Test for space in the destination (could be "<=" instead of "<"). */
	if (dest_ptr < last_char) {
	  /* Yes, there is still room for a terminating zero. */
	  *dest_ptr = 0; /* Terminate the string ... */
	  dest_ptr++; /* Advance destination pointer ... */

	  rp = first_free_ptr; /* Return value */
	  first_free_ptr = dest_ptr; /* Memory is now "allocated". */
	} else {
	  /* No, no space for terminating zero. */
	  copy_error = 1;
	}
      }
    }
  } else {
    /* NULL pointer passed as argument, thus return NULL pointer. */
    rp = NULL;
  }

  if (copy_error) {
    /* Some error occurred during copying ... */
    rp = NULL;

    /* Returning a NULL pointer might be changed to a pointer to some
     * kind of stop command in the SMR-CL syntax. */
  }

#if 0
  fprintf(stderr, "Index %i, cs:%p, r:%p, f:%p, e:%p\n", rp-cs_buf, cs_buf, rp, first_free_ptr, last_char);
#endif

  return rp;
}

