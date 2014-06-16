#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "loader.h"

int loadplan(char * fname,plantype * p, memtype *pm){
    char buf[256],*bp;
    FILE *fp;
    fp=fopen(fname,"r");
    if (!fp) return(1); // file not found
    strcpy(pm->pmem,fname);
    p->name=pm->pmem;
    p->nlabels=0;
    pm->pmem+=strlen(p->name)+1;
    p->start=pm->pmem;
    while (fgets(buf,256,fp)!=NULL){
      bp=buf;
      while (*bp==' ' || *bp=='\t')bp++;
      if (strncmp(bp,"label",5)==0) {
        p->nlabels++;
        p->labels[p->nlabels-1]=pm->pmem;
      }
      strcpy(pm->pmem,bp);
      pm->pmem+=strlen(bp)+1;
    }
    p->end=pm->pmem;
    return 0;
}

char * findlabel(char *label,plantype *p){
   int i,n,len;
   char *bp,*s;
   len=strlen(label);
   for (i=0;i<p->nlabels;i++){ 
     bp=p->labels[i];
     bp+=5;
     while (*bp!='\0' && *bp!='"')bp++;
     if (*bp=='"'){
        bp++;
        s=bp;n=0;
        while( isalnum(*bp)){bp++;n++;}
        if (n == len && strncmp(s,label,n)==0) break;
     }
   }
   if(i==p->nlabels)
     return NULL;
   else
     return p->labels[i];
}
      
