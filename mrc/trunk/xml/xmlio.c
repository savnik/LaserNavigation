#include <stdio.h>

#include <stdlib.h>
#include <string.h>

#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "xmlio.h"



/* Initialise xml_in structure.
 *  buflen: Number of bytes in input buffer. Must be at least as large as the largest
 *          possible tag.
 * attrmax: Max number of tag attributes. Extra attributes will be discarded.
 */
struct xml_in *xml_in_init(int buflen, int attrmax)
{
  struct xml_in *p;

  if ((p = malloc(sizeof(struct xml_in))) == 0) return 0;
  if (((p->buf = malloc(buflen)) == 0) ||
      ((p->attr = malloc(sizeof(struct xml_attr) * attrmax)) == 0)) {
    free(p);
    return 0;
  }
  p->a = 0;
  p->n = 0;
  p->empty = 0;

  p->next = 0;
  p->free = 0;
  p->buflen = buflen;
  p->attrmax = attrmax;

  return p;
}

/* Read characters from file descriptor and add to input buffer
 * Return value from read() is returned.
 *  >0: number of bytes read
 *   0: end of stream
 *  -1: error. errno == EAGAIN implies non blocking, no data available.
 */
int xml_in_fd(struct xml_in *p, int fd)
{
  int n = read(fd, p->buf + p->free, p->buflen - p->free);
  if (n > 0) p->free += n;

  return n;
}


#define ENTITIES 5

static struct {
  char c;
  char *s;
} entity[ENTITIES] = {
  {'<',  "lt;"  },
  {'>',  "gt;"  },
  {'&',  "amp;" },
  {'\'', "apos;"},
  {'"',  "quot;"},
};

#define MAYBE   -1
#define NOMATCH -2

static int match(char *s, int n)
{
  char *p, *q;
  int i;

  for (i = 0; i < ENTITIES; i++) {
    
    p = s;
    q = entity[i].s;
    do {
      if (*p++ != *q++) goto mismatch;
      if (*q == '\0') return i;
    } while (p < s + n);
    return MAYBE;

  mismatch: /* is goto evil? */;
  }
  return NOMATCH;
}

      
/*
 * (*from) points to a string, "&....", length n chars.
 * The function copies the converted char to (*to), incrementing
 * both (*from) and (*to) as it goes.
 * If the match function above reports NOMATCH, just the '&' is copied.
 * If the match function above reports MAYBE, the extra chars are also
 * copied, without incrementing (*to). The number of extra characters
 * copied is returned.
 */
static int unquote_char(char **to, char **from, int n)
{
  int i;

  printf("unquote_char: %d bytes\n", n);

  switch (i = match(*from + 1, n - 1)) {
  case NOMATCH: 
    printf(" nomatch\n");
    *(*to)++ = *(*from)++;
    return 0;
  case MAYBE:
    printf(" maybe\n");
    memcpy(*to, *from, n);
    return n;
  default:
    printf(" match\n");
    (*from) += strlen(entity[i].s) + 1;
    *(*to)++ = entity[i].c;
    return 0;
  }
}


static int unquote_move(char **to, char **from, int n)
{
  char *p = memchr(*from, '&', n);
  if (p != 0) n = p - *from;
  memmove(*to, *from, n);
  (*to) += n;
  (*from) += n;

  return p == 0;
}
  

int xml_unquote(char *s, int *n)
{
  char *from, *to;
  int m = 0;

   if ((from = to = memchr(s, '&', *n))) {
    for (;;) {
      if ((m = unquote_char(&to, &from, s + *n - from))) break;
      if (unquote_move(&to, &from, s + *n - from)) break;
    }
    *n = to - s;
  }
  return m;
}

#define WHITE " \t\r\n"



int xml_in_nibble(struct xml_in *p)
{
  int n;
  char *lt, *gt;

  if (p->empty) {
    p->empty = 0;
    p->n = 0;
    return XML_IN_TAG_END;
  }

  memmove(p->buf, p->buf + p->next, p->free - p->next);
  p->free -= p->next;
  p->next = 0;

  if (p->free == 0) return XML_IN_NONE;

  /* printf("looking at %d bytes\n", p->free); */
  
  lt = memchr(p->buf, '<', p->free);
  if (lt == p->buf) {
    gt = memchr(p->buf, '>', p->free);
    if (gt == 0) return XML_IN_NONE;

    p->next = gt - p->buf + 1;

    /* element from lt to gt */
  
    lt++;
    gt[0] = 0;
    if (lt[0] == '/') {
      /* end tag */
      lt++;
      n = strcspn(lt, WHITE);
      p->a = lt;
      lt[n] = 0;
      return XML_IN_TAG_END;
    } else {
      if (gt[-1] == '/') {
	/* empty tag */
	gt--;
	gt[0] = 0;
	p->empty = 1;
      }
      /* open or empty tag */
      p->a = lt;
      lt += strcspn(lt, WHITE);
      if (*lt) *lt++ = 0;

      p->n = 0;

      while ((p->n < p->attrmax) &&
	     (*(lt += strspn(lt, WHITE)))) {  /* skip whitespace */

	p->attr[p->n].name = lt;
	lt += strcspn(lt, WHITE "=");
	n = strspn(lt, WHITE);

	if (lt[n] == '=') {
	  /* attribute value */
	  lt[0] = 0;
	  lt += n + 1;
	  lt += strspn(lt, WHITE);

	  if ((lt[0] == '\'') || (lt[0] == '"')) {
	    char s[2] = {lt[0], 0}; 
	    /* quoted value */
	    lt++;
	    p->attr[p->n].value = lt;
	    lt += strcspn(lt, s);
	    if (*lt) *lt++ = 0;
	  } else {
	    /* unquoted value */
	    p->attr[p->n].value = lt;
	    lt += strcspn(lt, WHITE);
	    if (*lt) *lt++ = 0;
	  }
	} else {
	  /* missing attribute value */
	  lt[0] = 0;
	  lt += n;
	  p->attr[p->n].value = 0;
	}
	p->n++;
      } 
      return XML_IN_TAG_START;
    }
  } else {
    if (lt) {
      /* text followed by tag */
      int n1 = lt - p->buf;
      int n2 = xml_unquote(p->buf, &n1);
      p->n = n1 + n2;
      p->next = lt - p->buf;
    } else {
      /* unterminated text */
      int n1 = p->free;
      int n2 = xml_unquote(p->buf, &n1);
      p->n = n1;
      p->next = n1;
      p->free = n1 + n2;
    }
    p->a = p->buf;

    return p->n ? XML_IN_TEXT : XML_IN_NONE;
  }
}

int getdouble(double *val,char *name,struct xml_in *x){
    int i;
    for(i=0;i<x->n;i++){
      if (strcmp(name,x->attr[i].name)==0){
        sscanf(x->attr[i].value,"%lf",val);
	return 1;
      }
    }
    return 0;
}    
int gethex(double *val,char *name,struct xml_in *x){
    int i,val1;
    for(i=0;i<x->n;i++){
      if (strcmp(name,x->attr[i].name)==0){
        sscanf(x->attr[i].value,"%x",&val1);
	*val=val1;
	return 1;
      }
    } 
    return 0;
}

