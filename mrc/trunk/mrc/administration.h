
#ifndef ADMINISTRATION_H
#define ADMINISTRATION_H

#define NAMESIZE 80


typedef
struct{      
        char * parname;
	int partype;
	int vartype;
	void * parptr;
	}parameterelem;

typedef 

struct{
        int Nmax;
	int N;
        parameterelem *list ; 
} parameterlisttype;

typedef
  struct{
       char * functionname;
       parameterlisttype  *parlist;
        }functionlistelem;
      
typedef 
struct{
       char * modulename;
       functionlistelem  *functionlist;
       }modulelistelem;
       
		
extern functionlistelem motioncontrolfunclist[];			
extern modulelistelem modulelist[];

parameterelem * findpar(char* module,char * function, char * par);
int addpar(parameterlisttype *list,char * name,int partype,int vartype,void *parptr);

enum {ADMIN_par,ADMIN_inp,ADMIN_int,ADMIN_double};	
#endif //ADMINISTRATION_H
