#include <string.h>
#include "stream2line.h"


void outputline(stream2line_type *p){
  p->lb[p->linein][p->cp]='\0';
  p->linein++;
  if(p->linein== S2L_NLINES)
    p->linein=0;
  p->nlines++;
  p->cp=0;
}
		
		

void stream2line(stream2line_type *p){
int i;

for (i=0;i<p->Nib;i++){
  switch (p->state){
  case RECLINE:
              if (p->ib[i]== '\r'){
                 outputline(p);	        
		 p->state=AFTERCR;
	      }
	      else if (p->ib[i]== '\n'){
	        outputline(p);
		p->state=AFTERLF;
	      }
	      else {
	        p->lb[p->linein][p->cp]=p->ib[i];
		p->cp++;
	      }
	      break;
 
  case AFTERCR:
              if (p->ib[i] == '\r')
	        outputline(p);
	      else if (p->ib[i]=='\n')
	        p->state=RECLINE;
	      else{
	        p->lb[p->linein][p->cp]=p->ib[i];
		p->cp++;
		p->state=RECLINE;	
              }
	      break;
	      
  case AFTERLF:
              if (p->ib[i] == '\n')
	        outputline(p);
	      else if (p->ib[i]=='\r')
	        p->state=RECLINE;
	      else{
	        p->lb[p->linein][p->cp]=p->ib[i];
		p->cp++;	
		p->state=RECLINE;
	      }	
              break;	            		
  } 
}
}
                          
void stream2lineinit(stream2line_type *p){
  p->Nib=0;
  p->linein=0;
  p->lineout=0;
  p->nlines=0;
  p->cp=0;
  p->state=RECLINE;
}
 
 void getnextline(char *line,stream2line_type * p) {
    if (p->nlines){
        strcpy(line,p->lb[p->lineout]);
	p->nlines--;
	p->lineout++;
	if (p->lineout>=S2L_NLINES)
	  p->lineout=0;
    }
    else
      line[0]=0;
}
 
 
 
 
 
