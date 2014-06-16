#include <stdio.h>

char buf[256];
int main(void){
  
  while ( gets(buf)!= NULL) {
    system(buf);
  }
}   