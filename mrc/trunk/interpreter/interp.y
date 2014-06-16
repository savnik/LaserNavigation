
     %{
     #include <math.h>  /* For math functions, cos(), sin(), etc. */
     #include <stdio.h>
     #include "interp.h"  /* Contains definition of `symrec'        */
     #include <string.h>
     #include "mobilerobot.h"
     int yylex(void);
     void setvel(double v);
     void setacc(double v);
     void printans(void);
     void * getparlistptr(void);
     int iffunc(double,char *);
     int parok(symrec *);
     void switchfunc(int cas);
     void pushcode(codeel);
     int npar=0,ncond=0;
     double conditions[100],*conditionsptr;
     double constants[10000],*constptr=constants;
     char *bp;  
     char *condstart;
     char newvarname[256];
     symboltable symbols,sysvar;
     tabletype varpooltable;
     double varpool[VARPOOLSIZE];
  
     symrec * putsym (char *sym_name, int sym_type,symboltable *tab);

      extern int verbose;
       
     #if (CPI)
     codeel codestack[256];
     #endif
     int codep,watch=0;
     codeel codet;
    
     
     %}
     %union {
     double     val;  /* For returning numbers.                   */
     symrec  *tptr;   /* For returning symbol-table pointers      */
     double  *cptr;
     char    *csptr;
     }
     
     %token        EQ
     %token	   NEQ
     %token        GTE
     %token        LTE
     %token        VEL
     %token        ACC
     %token        NEWVAR
     %token <tptr> VAR FNCT FNCT2 CMD /* Variable and Function            */
     %token <cptr> NUM
     %token <csptr> STRING
     %token        IF
     %token        EVAL
     %token        SWITCH
     %token	   CASE
     %token	   ENDSWITCH
     %token        ERR
     %type  <val>  exp



     %right '='

     %left  '|'
     %left  '&'
     %left   EQ
     %left   NEQ
     %left   '>'
     %left   '<'
     %left   GTE
     %left   LTE
     %left   '-' '+'
     %left   '*' '/'
     %left   NEG     /* Negation--unary minus */
     %right  '^'    /* Exponentiation        */

     %%

     input:   /* empty */
             | input line
     ;

     line:
              '\n'               {return(1);}
             | CMD parlist vehiclestate  '\n' {if (parok($1))return $1->value.cmdptr(getparlistptr() ); else return 2;}
             | CMD parlist vehiclestate condstart condlist '\n' {if (parok($1)) return $1->value.cmdptr(getparlistptr());else return 2;}
             | ':' condlist '\n'          {return(0);}
             | EVAL explist '\n'          {if (verbose) printans();return(1);}
             | IF '(' exp ')' STRING      {return iffunc($3,$5);}
             | VAR '=' exp                {  $1->value.pvar[0] = $3;      }
	     | VAR '[' exp ']' '=' exp    {if ((int)($3) >-1 && (int)$3 < $1->varsize) $1->value.pvar[(int)($3)] = $6;   }
	     | NEWVAR '=' exp		  {symrec *s ;s=putsym(newvarname,VAR,&symbols);if (s==NULL) return(2);else 
	     				  {s->value.pvar[0] = $3;return 1;}} 
	     | SWITCH '(' exp ')'	  { switchfunc($3); return 1; }
	     | CASE   NUM		  { intp_switch.status=SW_AFTERCASE; return 1;}
	     | ENDSWITCH		  { intp_switch.status=SW_ENDED;return 1}
             | error '\n'                 { yyerrok;        return(2) }
     ;
     
     condstart: ':'	{if (CPI) codep=0;}
     ;

     explist:
              exp                 {pushans($1);}
             | explist ';' exp    {pushans($3);}
     ;

     parlist:
             | parlist NUM	  { pushvarptr(NUMPAR,$2);}
	     | parlist '-' NUM    { pushvarptr(NUMPAR,$3); *$3=-*$3;}
             | parlist VAR        { pushvarptr(VARPAR,($2->value.pvar));}
             | parlist STRING     { pushvarptr(STRINGPAR,$2);}
     ;
     vehiclestate:
             | vehiclestate VEL NUM          {setvel(*$3);}
             | vehiclestate VEL '-' NUM      {setvel(-*$4);}
             | vehiclestate VEL  VAR         {setvel($3->value.pvar[0]);}
             | vehiclestate ACC NUM          {setacc(*$3);}
             | vehiclestate ACC  VAR         {setacc($3->value.pvar[0]);}

    ;

    condlist :
              '(' exp ')'                  {pushcond($2);
	                                   if(CPI){codet.typ=OPTYP;codet.val.op=O_COND;pushcode(codet);}}
              | condlist '|' '(' exp ')'   {pushcond($4);
	      				    if(CPI){codet.typ=OPTYP;codet.val.op=O_COND;pushcode(codet);}}
    ;
     exp:      NUM                { $$ = *$1; if(CPI){
                  		    codet.typ=NUMTYP;codet.val.cst=*$1;pushcode(codet);}}                    
             | VAR                { $$ = $1->value.pvar[0]; if(CPI){
                  		   codet.typ=VARTYP;codet.val.tp=$1;pushcode(codet);}}     
	     | VAR '[' exp ']'    { if ((int)($3) >-1 && (int)$3 < $1->varsize) $$=$1->value.pvar[(int)($3)];}			      
             | FNCT '(' exp ')'   { $$ = (*($1->value.fnctptr))($3); }
             | FNCT2 '(' exp ',' exp ')'   { $$ = (*($1->value.fnct2ptr))($3,$5); }
             | exp  '|' exp       { $$ = $1 || $3;                    }
             | exp  '&' exp       { $$ = $1 && $3;
	                            if(CPI){codet.typ=OPTYP;codet.val.op=O_AND;pushcode(codet);}}
             | exp  EQ exp        { $$ = $1 == $3;               }
             | exp  NEQ exp       { $$ = $1 != $3;               }
             | exp  '>' exp       { $$ = $1 > $3;
	                          if(CPI){codet.typ=OPTYP;codet.val.op=O_GT;pushcode(codet);}}
             | exp  '<' exp       { $$ = $1 < $3; 
	                          if(CPI){codet.typ=OPTYP;codet.val.op=O_LT;pushcode(codet);}}
             | exp LTE exp        { $$ = $1 <= $3;          }
             | exp GTE exp        { $$ = $1 >= $3;          }
             | exp '+' exp        { $$ = $1 + $3;                    }
             | exp '-' exp        { $$ = $1 - $3;                    }
             | exp '*' exp        { $$ = $1 * $3;                    }
             | exp '/' exp        { $$ = $1 / $3;                    }
             | '-' exp  %prec NEG { $$ = -$2;                        }
             | exp '^' exp        { $$ = pow ($1, $3);               }
             | '(' exp ')'        { $$ = $2;                         }
     ;
     /* End of grammar */
     %%



       /* The symbol table: a chain of `struct symrec'.     */
//     extern symrec *sym_table;
 #include "loader.h"
  #define NSYSVARS	300
  #define NSYMBOLS	10000


     symrec * getsym (const char *sym_name, symboltable tab);
//   extern char buf[80];
    
   symrec symtab1[NSYMBOLS];
   symrec sysvartab[NSYSVARS];
   symbolptrtabtype logvar,streamvar;
   varptrtabtype logvar1;
   char stringtab[100000],*stringtabptr=stringtab;
   char conststringtab[10000],*conststringtabptr=conststringtab;
   double varstack[100],*pvarstack=varstack;
   paramtype varptrstack[100],*pvarptrstack=varptrstack;
   double anstab[100];
   int n_ans=0;
  
   plantype *currentplan;
   symrec * condptr;
   double * pcmdtime,condpar0;
   
   char *conststringtabptr_b;
   paramtype *pvarptrstack_b;
   double *conditionsptr_b,cond_bvar;
   symrec cond_b,*condptr_b=&cond_b;
   int npar_b;
   int ncond_b;
   int nans_b;
   double *constptr_b;
   char *condstart_b;
   int (*defaultcond_b)(void); 
   extern int (*defaultcond)(void); 
   double *funcres[50];
   struct{
       int p;
       char *stack[1000];
       }programlinestack;
   
   void pushprogramline(char *line){
    programlinestack.stack[programlinestack.p]=line;
    programlinestack.p++;
    }
    
    char *popprogramline(void){   
    programlinestack.p--;
    return programlinestack.stack[programlinestack.p];
 
    }

   
    void switchfunc(int cas){
      if (cas < 1){
        intp_switch.actualcase=0;
	intp_switch.status=SW_INCASE;
      }
      else {
        intp_switch.actualcase=cas;
	intp_switch.status=SW_STARTED;
      }
    }

    double * putinternvar(char * name,int len){
     symrec * s;
       s = putsym (name, VAR, &sysvar);
       varpooltable.N+=3*len-1;
       s->varsize=len;       
       return (s->value.pvar);
    }

     
     void setcurrentplan(plantype *p){
        currentplan=p;
     }

     void * getparlistptr(void){
       return (void *) varptrstack;
     }
    
     int getncond(void){
        return ncond;
     }     

     int getcondres(void){
       int i=0;
       while (i< ncond){
         if (conditions[i]) break;
         i++;
       }
       if (i== ncond)
         return 0;
       else
         return i+1;
     }

    double getwatchres(void){
       if (ncond >0)
         return conditions[0];
       else     
          return 0;
    }
    
    int parok(symrec *p){
       if (npar >=(p->parstring[0]-48) && (npar <=(p->parstring[1]-48)||((p->parstring[1]==57)&& (npar<50))))
         return(1);
       else{
         printf("Expecting between %c  and %c parameters\n",p->parstring[0],p->parstring[1]);
         return(0);
       }
     } 


     void saveinterpcontext(void){
       conststringtabptr_b=conststringtabptr;
       pvarptrstack_b=pvarptrstack;
       conditionsptr_b=conditionsptr;      
       condptr_b->value.pvar[0]=condptr->value.pvar[0];
       npar_b=npar;
       ncond_b=ncond;
       nans_b=n_ans;
       constptr_b=constptr;
       condstart_b=condstart;
       defaultcond_b=defaultcond;
     }

     void loadinterpcontext(void){
       conststringtabptr=conststringtabptr_b;
       pvarptrstack=pvarptrstack_b;
       conditionsptr=conditionsptr_b;
       condptr->value.pvar[0]=condptr_b->value.pvar[0];
       npar=npar_b;
       ncond=ncond_b;
       n_ans=nans_b;
       constptr=constptr_b;
       condstart=condstart_b;
       defaultcond=defaultcond_b;
     }
 
     void newcmd(void){
       conststringtabptr=conststringtab;
       pvarptrstack=varptrstack;
       conditionsptr=&conditions[0];
       condptr->value.pvar[0]=getcondres();
       npar=0;
       ncond=0;
       n_ans=0;
       codep=0;
       constptr=constants;
       condstart=NULL;
       
     }
     
     void initinterp(void){
        watchtab.max=WATCHTABSIZE;
        watchtab.p=0;
     }
     
     void calccond(void){
       ncond=0;
       bp=condstart;
       constptr=constants;
       conditionsptr=&conditions[0];
     }

     void pushcond(double v){
       *conditionsptr++=v;
       ncond++;  
     }

     void pushvar(double v){
       *pvarstack++=v;
       npar++;  
     }

     void pushvarptr(int typ,void *p ){
       pvarptrstack->pt=p;
       pvarptrstack->typ=typ;
       pvarptrstack++;
       npar++;  
     }

     void pushans(double ans){
       n_ans++;
       anstab[n_ans-1]=ans;
     }
     
     void printans(void){
       int i;
       for (i=0;i<n_ans;i++)
         printf("%lf  ",anstab[i]);
       printf("\n");
     }	 

     #if (CPI)
     void pushcode(codeel c){
       codestack[codep]=c;
       codep++;
     }
     #endif

     void
     yyerror (const char *s)  /* Called by yyparse on error */
     {
       //printf ("yyerror %s\n", s);
     }
     
     #if (CPI)
     #include "avrsmrfunctions.c"
     #else
     #include "smrfunctions.c"
     #endif
     
     double intp_rem(double a,double b){
       b=fabs(b);
       while (a > b) a-=b;
       while (a < 0) a+=b;
       return a;
     }
      
     double normalizeanglerad(double a){
        normalizeangle(&a);
	return a;
     }
     
     double normalizeangledeg(double a){
        a=a/180.0*M_PI;
	normalizeangle(&a);
	a=a/M_PI*180.0;
	return a;
     }
  
     
    
     struct init
     {
       char *fname;
       double (*fnct)(double);
     };
     
     struct init arith_fncts[] =
     {
       {"sin",  sin},
       {"cos",  cos},
       {"asin",  asin},
       {"acos",  acos},
       {"tan", tan},
       { "atan", atan},
       {"ln",   log},
       {"exp",  exp},
       {"sqrt", sqrt},
       {"abs", fabs},
       {"normalizeanglerad", normalizeanglerad},
       {"normalizeangledeg", normalizeangledeg},
       {0, 0}
     };
     
     /* The symbol table: a chain of `struct symrec'.  */
  //   symrec *sym_table = (symrec *) 0;
     
     /* Put arithmetic functions in table. */
     void
     init_table (void)
     {
       int i;
       symrec *ptr;
       logvar1.n=0;
       logvar1.nmax=LOGVARMAX;
       logvar1.ref=(double **) malloc(LOGVARMAX*sizeof(double *));
       sysvar.nsym=0;
       sysvar.nsymmax=NSYSVARS;
       sysvar.ptable=sysvartab;
       symbols.nsym=0;
       symbols.nsymmax=NSYMBOLS;
       symbols.ptable=symtab1;   
       varpooltable.N=0;
       varpooltable.Nmax=VARPOOLSIZE;
       varpooltable.tab=varpool;
       programlinestack.p=0;
       condptr = putsym("condition",VAR,&sysvar);
       condptr->value.pvar[0]=-1;
       condptr_b->value.pvar=&cond_bvar;
       funcres[0]=putinternvar("res0",1);
       funcres[1]=putinternvar("res1",1);
       funcres[2]=putinternvar("res2",1);
       funcres[3]=putinternvar("res3",1);
       funcres[4]=putinternvar("res4",1);
       funcres[5]=putinternvar("res5",1);
       funcres[6]=putinternvar("res6",1); 
       funcres[7]=putinternvar("res7",1);
       funcres[8]=putinternvar("res8",1);
       funcres[9]=putinternvar("res9",1);

       for (i = 0; cmd_fncts[i].fname != 0; i++)
         {
           ptr = putsym (cmd_fncts[i].fname, CMD, &symbols);
           ptr->value.cmdptr = cmd_fncts[i].fnct;
           ptr->parstring=cmd_fncts[i].parstring;
         }
       for (i = 0; arith_fncts[i].fname != 0; i++)
         {
           ptr = putsym (arith_fncts[i].fname, FNCT, &symbols);
           ptr->value.fnctptr = arith_fncts[i].fnct;       
         }
       ptr = putsym("atan2",FNCT2,&symbols);
       ptr->value.fnct2ptr = atan2;
       ptr = putsym("hypot",FNCT2,&symbols);
       ptr->value.fnct2ptr = hypot;
       ptr = putsym("rem",FNCT2,&symbols);
       ptr->value.fnct2ptr = intp_rem;
       ptr = putsym("ntrue",FNCT2,&symbols);
       ptr->value.fnct2ptr = intp_ntrue;
     }
    
 
     symrec *
     putsym (char *sym_name, int sym_type, symboltable *tab)
     {
       symrec *ptr;
       ptr = &(tab->ptable[tab->nsym]);
       tab->nsym++;
       if (tab->nsym >=tab->nsymmax) return NULL;
       ptr->name = stringtabptr;
       stringtabptr+=strlen (sym_name) + 1;
       strcpy (ptr->name,sym_name);
       ptr->type = sym_type;
       if (sym_type== VAR){
         ptr->value.pvar =&((double *)varpooltable.tab)[varpooltable.N];
         varpooltable.N++; 
         ptr->value.pvar[0]=0;
	 ptr->varsize=1;
       }
       return ptr;
     }
     
     symrec *
     getsym (const char *sym_name, symboltable tab)
     {
       symrec *ptr;
       for (ptr = tab.ptable; ptr != &tab.ptable[tab.nsym]; ptr++)
         if (strcmp (ptr->name,sym_name) == 0)
           return ptr;
       return 0;
     }

     #include <ctype.h>
     
     int
     yylex (void)
     {
       static char symbuf[80];
       /* Ignore whitespace, get first nonwhite character.  */
       while (*bp == ' ' || *bp =='\t' || *bp=='\r' )bp++;
     
       if (*bp=='\0' || *bp=='%')
         return '\n';
     
       /* Char starts a number => parse the number.         */
       if (*bp== '.' || isdigit (*bp))
         {
          
          // sscanf (bp,"%lf", constptr);
	  *constptr=strtod(bp,&bp);
          yylval.cptr=constptr;
          constptr++;
          
          //while(*bp== '.' || isdigit (*bp))bp++;

           return NUM;
         }
       /* string constant */
       if (*bp=='"' || *bp==(char)92){
          char *start=conststringtabptr;
          bp++;
          while( *bp !='"' && *bp != (char) 92){
            if (*bp=='\0') return('\n');
            *conststringtabptr++=*bp;
            bp++;
          }
          *conststringtabptr++='\0';
          bp++;
          yylval.csptr=start;
          return (STRING);
       }
     
       /* Char starts an identifier => read the name.       */
       if (isalpha (*bp))
         {
           symrec *s;        
           int i;  
           i = 0;
           do
             {            
               /* Add this character to the buffer.         */
               symbuf[i++] = *bp;
               /* Get another character.                    */
               bp++;
             }
           while (*bp!='\0'  && isalnum (*bp));     
           symbuf[i] = '\0';
           if ( strcmp(symbuf,"if")==0) return IF;     
           if ( strcmp(symbuf,"eval")==0) return EVAL;
	   if ( strcmp(symbuf,"switch")==0) return SWITCH;
	   if ( strcmp(symbuf,"endswitch")==0) return ENDSWITCH;
	   if ( strcmp(symbuf,"case")==0) return CASE;
           s = getsym (symbuf,symbols);
           if (s != 0){  
             yylval.tptr = s;
             return s->type;
	   }
	   else {
              strncpy(newvarname,symbuf,255); 
	      return NEWVAR;
	   }
         }
         if (*bp=='$'){
           if (isalpha (*(bp+1)))
           {
             symrec *s;        
             int i;  
             i = 0;
             bp++;
             do
             {            
               /* Add this character to the buffer.         */
               symbuf[i++] = *bp;
               /* Get another character.                    */
               bp++;
             }
             while (*bp!='\0'  && isalnum (*bp));     
             symbuf[i] = '\0';
             s = getsym (symbuf,sysvar);
             if (s == NULL){
               printf("System variable \"$%s\" not found\n",symbuf);
               return ERR;
             }
             else {
               yylval.tptr = s;
               return s->type;
             }
           }
           else
           ;
         }
         if (*bp ==':')
           condstart=bp;
         if (*bp == '=' && *(bp+1)=='=') {
           bp+=2; return EQ;
         }
         if (*bp == '@' && *(bp+1)=='v') {
           bp+=2; return VEL;
         }
         if (*bp == '@' && *(bp+1)=='a') {
           bp+=2; return ACC;
         }
         if (*bp == '!' && *(bp+1)=='=') {
           bp+=2; return NEQ;
         }
         if (*bp == '>' && *(bp+1)=='=') {
           bp+=2; return GTE;
         }
         if (*bp == '<' && *(bp+1)=='=') {
           bp+=2; return LTE;
         }






       /* Any other character is a token by itself.        */
       return *bp++;
     }
