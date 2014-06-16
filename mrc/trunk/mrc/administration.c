#include <string.h>
#include <stdio.h>
#include "administration.h"


int addpar(parameterlisttype *list,char * name,int partype,int vartype,void *parptr){
      int i;
      i=list->N;
      if (i <list->Nmax){
        list->list[i].parname=name;
	list->list[i].partype=partype;
	list->list[i].vartype=vartype;
	list->list[i].parptr=parptr;
	list->N++;
	return 0;
      }
      else
        return 1;
}




parameterelem * findpar(char * module, char * function, char * par){
 int i=0,found=0,j=0,k;
  while(!found && (modulelist[i].functionlist!=0)){
    if ( strcmp(module,modulelist[i].modulename)==0){ 
      found=1;
    }
    else
     i++;
   }
  if (!found){
    printf("module not found\n");
    return 0;
    }
  else{
    found=0;
    while(!found && (modulelist[i].functionlist[j].parlist!=0)){
      if ( strcmp(function,modulelist[i].functionlist[j].functionname)==0){
        found=1;
      }
      else
       j++;
    }
  }
  if (!found) {
    printf("paramlist not found\n");
    return 0;
    }
  else {
    found=0;
    for (k=0;k< modulelist[i].functionlist[j].parlist->N;k++){
      if ( strcmp(par,modulelist[i].functionlist[j].parlist->list[k].parname)==0){ 
        found=1;
        break;
      }
    }
  }
  if (found) 
    return &modulelist[i].functionlist[j].parlist->list[k];
  else
    return 0;
}
