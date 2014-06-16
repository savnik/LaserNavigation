// $Id: polyline.c,v 1.1.1.1 2005/11/22 21:39:58 naa Exp $
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "polyline.h"

static void polyline_doublepoints(polylinetype line) {
  double *tmpx,*tmpy,*tmpex,*tmpey,*tmpl,*tmpc;
  int oldmaxpoints = line->maxpoints;

  tmpx = (double *) malloc(line->maxpoints * sizeof(double));
  tmpy = (double *) malloc(line->maxpoints * sizeof(double));
  tmpex = (double *) malloc(line->maxpoints * sizeof(double));
  tmpey = (double *) malloc(line->maxpoints * sizeof(double));
  tmpl = (double *) malloc(line->maxpoints * sizeof(double));
  tmpc = (double *) malloc(line->maxpoints * sizeof(double));
  
  memcpy(tmpx,line->x,line->maxpoints * sizeof(double));
  free(line->x);

  memcpy(tmpy,line->y,line->maxpoints * sizeof(double));
  free(line->y);

  memcpy(tmpex,line->ex,line->maxpoints * sizeof(double));
  free(line->ex);

  memcpy(tmpey,line->ey,line->maxpoints * sizeof(double));
  free(line->ey);

  memcpy(tmpl,line->l,line->maxpoints * sizeof(double));
  free(line->l);

  memcpy(tmpc,line->c,line->maxpoints * sizeof(double));
  free(line->c);

  line->maxpoints *= 2;

  line->x = (double *) malloc(line->maxpoints * sizeof(double));
  line->y = (double *) malloc(line->maxpoints * sizeof(double));

  line->ex = (double *) malloc(line->maxpoints * sizeof(double));
  line->ey = (double *) malloc(line->maxpoints * sizeof(double));

  line->l = (double *) malloc(line->maxpoints * sizeof(double));
  line->c = (double *) malloc(line->maxpoints * sizeof(double));

  if ((line->x == NULL) || (line->y == NULL) || (tmpx == NULL) || (tmpy == NULL) ||
	  (line->ex == NULL) || (line->ey == NULL) || (tmpex == NULL) || (tmpey == NULL) ||
	  (line->l == NULL) || (line->c == NULL) || (tmpl == NULL) || (tmpc == NULL)) {
    perror("polyline_doublepoints: malloc returned NULL. Exiting.");
    exit(1);
  }
  memcpy(line->y,tmpy,oldmaxpoints * sizeof(double));
  memcpy(line->x,tmpx,oldmaxpoints * sizeof(double));

  memcpy(line->ey,tmpey,oldmaxpoints * sizeof(double));
  memcpy(line->ex,tmpex,oldmaxpoints * sizeof(double));

  memcpy(line->l,tmpl,oldmaxpoints * sizeof(double));
  memcpy(line->c,tmpc,oldmaxpoints * sizeof(double));

  free(tmpx);
  free(tmpy);
  free(tmpex);
  free(tmpey);
  free(tmpl);
  free(tmpc);
}

void polyline_addpoint(polylinetype *p, double x, double y) {
  int c;

  if (*p == NULL) {
    *p = (polylinetype) malloc(sizeof(struct polyline));
    (*p)->numpoints = 0;
    (*p)->maxpoints = 1;
    (*p)->subtype = PLT_NONE;
    (*p)->x = (double *) malloc(sizeof(double));
    (*p)->y = (double *) malloc(sizeof(double));
    (*p)->ex = (double *) malloc(sizeof(double));
    (*p)->ey = (double *) malloc(sizeof(double));
    (*p)->l = (double *) malloc(sizeof(double));
    (*p)->c = (double *) malloc(sizeof(double));
  }

  if ( (fabs( x - (*p)->x[(*p)->numpoints-1] ) < 1e-8) &&
	   (fabs( y - (*p)->y[(*p)->numpoints-1] ) < 1e-8)) {
	return;
  }

  (*p)->x[(*p)->numpoints] = x;
  (*p)->y[(*p)->numpoints] = y;

  if ( 0 < (c = (*p)->numpoints) ) {
	(*p)->l[c] = ((*p)->l[c-1] = sqrt( ( (*p)->x[c] - (*p)->x[c-1] ) * ( (*p)->x[c] - (*p)->x[c-1] ) +
					   ( (*p)->y[c] - (*p)->y[c-1] ) * ( (*p)->y[c] - (*p)->y[c-1] ) ));
	(*p)->ex[c] = ((*p)->ex[c-1] = ( (*p)->x[c] - (*p)->x[c-1] ) / (*p)->l[c-1]);
	  (*p)->ey[c] = ((*p)->ey[c-1] = ( (*p)->y[c] - (*p)->y[c-1] ) / (*p)->l[c-1]);
	(*p)->c[c] = ((*p)->c[c-1] = (*p)->ey[c-1] * (*p)->x[c-1] - (*p)->ex[c-1] * (*p)->y[c-1]);
  }

  (*p)->numpoints++;
  if ((*p)->numpoints == (*p)->maxpoints) {
    polyline_doublepoints(*p);
  }

}

static void polyline_doublenumlines(polylinestype lines) {
  int oldmaxlines = lines->maxlines;
  polylinetype *oldlines = (polylinetype *) malloc(oldmaxlines * sizeof(polylinetype));

  memcpy(oldlines, lines->lines, oldmaxlines * sizeof(polylinetype));
  
  free(lines->lines);
  lines->maxlines = 2 * oldmaxlines;
  lines->lines = (polylinetype *) malloc(lines->maxlines * sizeof(polylinetype));

  memcpy(lines->lines, oldlines, oldmaxlines * sizeof(polylinetype));

  for ( ; oldmaxlines < lines->maxlines; oldmaxlines++) {
    lines->lines[oldmaxlines] = NULL;
  }

  free(oldlines);
}

void polyline_addline(polylinestype lines, polylinetype line) {
  lines->lines[lines->numlines++] = line;
  if (lines->numlines == lines->maxlines) {
    polyline_doublenumlines(lines);
  }
}

static char *plt2char(enum polylinetype t) {
  switch(t) {
  case PLT_NONE:
    return "PLT_NONE";
    break;
  case PLT_POLYLINE:
    return "PLT_POLYLINE";
    break;
  case PLT_BOX:
    return "PLT_BOX";
    break;
  case PLT_POLYGON:
    return "PLT_POLYGON";
    break;
  case PLT_ARCBOX:
    return "PLT_ARCBOX";
    break;
  case PLT_PICTUREBOX:
    return "PLT_PICTUREBOX";
    break;
  default:
    return "Unknown Subtype";
    break;
  }
}

static void polyline_printline(polylinetype p) {
  int i;
  if (!p)
    return;
  printf("Polyline type %s with %d of %d points used\n",plt2char(p->subtype),p->numpoints,p->maxpoints);
  for (i = 0; i < p->numpoints; i++) {
    printf("( %f , %f , %f , %f , %f , %f )\n",p->x[i],p->y[i], p->ex[i],
		   p->ey[i], p->l[i], p->c[i]);
  }
}

void polyline_print(polylinestype lines) {
  int i;
  if (!lines)
    return;

  for (i = 0; i < lines->numlines; i++) {
    polyline_printline(lines->lines[i]);
    printf("\n");
  }
}

polylinestype polyline_new() {
  polylinestype lines = (polylinestype) malloc(sizeof(struct polylines));
  int i;

  lines->numlines = 0;
  lines->maxlines = 1;
  lines->lines = (polylinetype *) malloc(lines->maxlines * sizeof(polylinetype));
  for (i = 0; i < lines->maxlines; i++)
    lines->lines[i] = NULL;

  return lines;
}

static void polyline_freeline(polylinetype *line) {
  if (*line) {
    free((*line)->x);
    free((*line)->y);
    free((*line)->ex);
    free((*line)->ey);
    free((*line)->l);
    free((*line)->c);
    free(*line);
    *line = NULL;
  }
}

void polyline_free(polylinestype *lines) {
  int i;

  // Free each individual line
  for (i = 0; i < (*lines)->maxlines; i++) {
    polyline_freeline(&((*lines)->lines[i]));
    free((*lines)->lines[i]);
    (*lines)->lines[i] = NULL;
  }

  // Free pointer to polylines
  free((*lines)->lines);
  (*lines)->lines = NULL;

  // Free entire polylines struct
  free(*lines);
  *lines = NULL;
}
