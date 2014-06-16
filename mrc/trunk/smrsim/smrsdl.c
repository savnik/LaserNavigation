#include <stdlib.h>
#include <SDL/SDL_endian.h> /* Used for the endian-dependent 24 bpp mode */

#include "smrsdl.h"

// Our main drawing surface
SDL_Surface *screen;

// Variables for doing window-viewport transformations
double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
double xratio = 0.0, yratio = 0.0; 
int width = 0, height = 0;

// Function for initialising drawing area and calculating ratios
int smrsdl_init(int w, int h, double x_min, double x_max, double y_min, double y_max) {
  int flags = 0;

  width = w;
  height = h;

  xmin = x_min;
  xmax = x_max;
  ymin = y_min;
  ymax = y_max;

  xratio = width / (xmax - xmin);
  yratio = - height / (ymax - ymin);

  // Initialize SDL and set video mode
  if ( SDL_Init(SDL_INIT_AUDIO|SDL_INIT_VIDEO) < 0 ) {
    fprintf(stderr, "Unable to init SDL: %s\n", SDL_GetError());
    return 0;
  }
  atexit(SDL_Quit);

  flags |= SDL_SWSURFACE;
  //  flags |= SDL_OPENGL;
  //  flags |= SDL_DOUBLEBUF;
  screen = SDL_SetVideoMode(width, height, 0, SDL_SWSURFACE);
  if ( screen == NULL ) {
    fprintf(stderr, "Unable to SetVideoMode to %dx%d : %s\n", width, height, SDL_GetError());
    return 0;
  }
  return 1;
}

/* DrawPixel function taken from "SDL introduction" document:
 * http://www.libsdl.org/ */
void DrawPixel(SDL_Surface *surface, int x, int y, Uint8 R, Uint8 G, Uint8 B) {
  Uint32 color = SDL_MapRGB(surface->format, R, G, B);

  if ( SDL_MUSTLOCK(surface) ) {
    if ( SDL_LockSurface(surface) < 0 ) {
      return;
    }
  }
  switch (surface->format->BytesPerPixel) {
  case 1: { // Assuming 8-bpp
    Uint8 *bufp;

    bufp = (Uint8 *)surface->pixels + y*surface->pitch + x;
    *bufp = color;
  }
    break;

  case 2: { // Probably 15-bpp or 16-bpp
    Uint16 *bufp;

    bufp = (Uint16 *)surface->pixels + y*surface->pitch/2 + x;
    *bufp = color;
  }
    break;

  case 3: { // Slow 24-bpp mode, usually not used
    Uint8 *bufp;

    bufp = (Uint8 *)surface->pixels + y*surface->pitch + x * 3;
    if(SDL_BYTEORDER == SDL_LIL_ENDIAN) {
      bufp[0] = color;
      bufp[1] = color >> 8;
      bufp[2] = color >> 16;
    } else {
      bufp[2] = color;
      bufp[1] = color >> 8;
      bufp[0] = color >> 16;
    }
  }
    break;

  case 4: { // Probably 32-bpp
    Uint32 *bufp;

    bufp = (Uint32 *)surface->pixels + y*surface->pitch/4 + x;
    *bufp = color;
  }
    break;
  }
  if ( SDL_MUSTLOCK(surface) ) {
    SDL_UnlockSurface(surface);
  }
}

/* DrawLine code taken from the SDL_draw API by Mario Palomo, Jose de la Huerga
 * and Pepe Gonzalez: http://sdl-draw.sourceforge.net/ */
void DrawLine(SDL_Surface *surface, Sint16 x1, Sint16 y1, Sint16 x2, Sint16 y2) {

  Sint16 x = x1;
  Sint16 y = y1;
  Sint16 dy = y2 - y1;
  Sint16 dx = x2 - x1;

  Sint16 G, DeltaG1, DeltaG2, minG, maxG;
  Sint16 swap;
  Sint16 inc = 1;

  DrawPixel(screen,x,y,255,255,255);

  if (abs(dy) < abs(dx)) { /* -1 < ramp < 1 */
      if (dx < 0) {
        dx = -dx;
        dy = -dy;

        swap = y2;
        y2 = y1;
        y1 = swap;

        swap = x2;
        x2 = x1;
        x1 = swap;
      }
      x = x1;
      y = y1;
      if (dy < 0) {
        dy = -dy;
        inc = -1;
      }

      G = 2 * dy - dx;
      DeltaG1 = 2 * (dy - dx);
      DeltaG2 = 2 * dy;

      while (x++ < x2) {
        if (G > 0) { G += DeltaG1; y += inc; }
        else  G += DeltaG2;

	DrawPixel(screen,x,y,255,255,255);
      }/*while*/

  } else { /* ramp < -1 or ramp > 1 */
      if (dy < 0) {
        dx = -dx;
        dy = -dy;

        swap = y2;
        y2 = y1;
        y1 = swap;

        swap = x2;
        x2 = x1;
        x1 = swap;
      }
      if (dx < 0) {
        dx = -dx;
        inc = -1;
      }
      x = x1;
      y = y1;

      G = 2 * dx - dy;
      minG = maxG = G;
      DeltaG1 = 2 * (dx - dy);
      DeltaG2 = 2 * dx;

      while (y++ < y2) {
        if (G > 0) { G += DeltaG1; x += inc; }
        else  G += DeltaG2;

	DrawPixel(screen,x,y,255,255,255);
      }/*while*/

  }/*if*/
}

// Draw the given smr as a single pixel on the screen
void smrsdl_drawpixel(simtype *smr) {
  DrawPixel(screen, smr->pose.x*xratio + width/2, smr->pose.y * yratio + height/2, 255, 255, 255);
  //  SDL_UpdateRect(screen, 0, 0, width, height);
}

// Draw a line from the last pose (initial [0,0,0]) to the current
void smrsdl_drawline(simtype *smr) {
  static double xx = 0, yy = 0.0, theta = 0;

  DrawLine(screen, xx*xratio + width/2, yy * yratio + height/2,
	   smr->pose.x*xratio + width/2, smr->pose.y * yratio + height/2);

  xx = smr->pose.x;
  yy = smr->pose.y;
  theta = smr->pose.th;

  //  SDL_UpdateRect(screen, (smr->pose.x>xx)?smr->pose.x:xx, (smr->pose.y>yy)?smr->pose.y:yy,
  //		 abs(smr->pose.x-xx), abs(smr->pose.y-yy));
}

void smrsdl_update(void) {
  SDL_UpdateRect(screen, 0, 0, width, height);
}

// Simple "wait until a key is pressed" function
void smrsdl_waitforkeypress() {
  SDL_Event event;

  SDL_UpdateRect(screen, 0, 0, width, height);

  while (1) {
    SDL_WaitEvent(&event);
    if (event.type == SDL_KEYDOWN)
      return;
  }
}

