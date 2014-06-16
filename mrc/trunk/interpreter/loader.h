#ifndef LOADER_H
#define LOADER_H

typedef struct {
          char * name;
          char *pline;
          char * start,*end;
          char *labels[1000];
          int  nlabels;
          }plantype;

typedef struct {
               char * mem;
               char * pmem;
           }memtype;
char * findlabel(char *label,plantype *p);
int loadplan(char * fname,plantype * p, memtype *pm);
#endif
