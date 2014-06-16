#include <time.h>

#include "logtime.h"

double LT_gettime_fp(void)
{
  const time_t current_time = time(NULL);
  const struct tm * const bdt_ptr = gmtime(&current_time);
  double time_fp = 0.0;

  if (bdt_ptr) {
    const struct tm bdt = *bdt_ptr;
    
    time_fp =
      10000.0 * (bdt.tm_year + 1900) +
      100.0 * (bdt.tm_mon + 1) +
      bdt.tm_mday +
      0.01 * bdt.tm_hour +
      0.0001 * bdt.tm_min +
      0.000001 * bdt.tm_sec;
  }

  return time_fp;
}

double LT_clock_fp(void)
{
  const double clock_fp = (clock() * 1.0) / (CLOCKS_PER_SEC);

  return clock_fp;
}
