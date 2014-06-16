#ifndef LOGTIME_HDR
#define LOGTIME_HDR

/* Functions to convert calendar time to floating point value, suitable for the log module. */

double LT_gettime_fp(void);
/* Returns a floating point value in the format:
 *   YYYYMMDD.HHMMSS
 * which is the resolution provided by "time"
 */

double LT_clock_fp(void);
/* Returns the number of seconds since start of computer (using the "clock" function). */

#endif /* LOGTIME_HDR */
