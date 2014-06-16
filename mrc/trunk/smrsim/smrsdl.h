#ifndef _SMRSDL_H_
#define _SMRSDL_H_

#include <SDL/SDL.h>
#include "smrsim.h"

int smrsdl_init(int width, int height, double xmin, double xmax, double ymin, double ymax);
void smrsdl_drawpixel(simtype *smr);
void smrsdl_drawline(simtype *smr);
void smrsdl_update(void);
void smrsdl_waitforkeypress();

#endif

