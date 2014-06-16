#include <stdio.h>





int isarray(char * res, char * inp){
int i,notarray,index;
i=0;
notarray=1;
index=-1;
while (inp[i]!=0 && notarray){
  if (inp[i]=='[' ){
    notarray=0;
    index=atoi(&inp[i+1]);
  }  
  else {
    res[i]=inp[i];
    i++;
  }
}  
res[i]=0;
return index;
}
 
int main(void){
char inp[256],res[256];
int i;
inp[0]=1;
while (inp[0]!=0){
  printf("Enter string \n");
  gets(inp);
  i=isarray(res,inp);
  printf("%d %s \n",i,res);
}  
return 0;
}
