#ifndef XFIGREADER_H_
#define XFIGREADER_H_

#include "polyline.h"

polylinestype xfr_init(char *filename);
void xfr_print(polylinetype p);
void xfr_printlines(polylinestype lines);
void xfr_freelines(polylinestype *lines);

#endif
