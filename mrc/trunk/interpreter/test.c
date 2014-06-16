#include <stdio.h>
#include <string.h>
#include "loader.h"
char planmem[100000];



int main(int argc,char **argv){
  plantype plan;
  memtype planm;
  char label[80];
  char *pline;
  if (argc==1) return (1);
  planm.mem=planmem;
  planm.pmem=planmem;
  loadplan(argv[1],&plan,&planm);
  printf("Plan: %s \n",plan.name);
  pline=plan.start;
 

 pline=plan.start;
  while (pline  != plan.end) {
    printf("%s",pline);
    pline+=strlen(pline)+1;
  }
  printf("nlabels %d\n",plan.nlabels);
  printf("%s",plan.labels[0]);
  printf("%s",plan.labels[1]);
  getchar();
  while(1){
    printf("Enter label ");
    scanf("%s",label);
    pline=findlabel(label,&plan);
    if (pline) 
      printf("%s",pline);
   else
      printf("label not found\n");
  }

}


 