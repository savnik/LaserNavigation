  /* Functions type.                                   */
     typedef double (*func_t) (double);
     typedef double (*func_t2) (double , double);
     typedef int (*cmd_t)(void *);

     /* Data type for links in the chain of symbols.      */
     struct symrec
     {
       char *name;  /* name of symbol                     */
       int type;    /* type of symbol: either VAR or FNCT */
       int varsize;
       char *parstring;
       union
       {
         double *pvar;                  /* value of a VAR   */
         func_t fnctptr;              /* value of a FNCT  */
         func_t2 fnct2ptr;
         cmd_t cmdptr;
       } value;

     };

   typedef struct symrec symrec;
   void yyerror (const char *s);
   void pushvarptr(int typ,void * p);
   void pushcond(double v);
   void pushans(double ans);
   double getwatchres(void);

   void newcmd(void);
   typedef struct {
                   int nsym;
		   int nsymmax;
                   symrec *ptable;
                   } symboltable;

   typedef struct{
   		  int sampletime;
                  int n;
                  symrec *ref[20];
                  }symbolptrtabtype;

  typedef struct{
                  int n;
                  int nmax;
                  double **ref;
                }varptrtabtype;


   typedef struct {
   		   int N;
		   int Nmax;
		   void *tab;
		   }tabletype;


   typedef struct{
   		char name[32];
		int type;
		double val;
		char exp[256];
		}watchelement;

   typedef struct{
                  int typ;
		  double * pt;
		 }paramtype;
   enum {VARPAR,NUMPAR,STRINGPAR};

   void saveinterpcontext(void);
   void loadinterpcontext(void);
   void initinterp(void);

   enum {SW_ENDED,SW_STARTED,SW_INCASE,SW_AFTERCASE};
   struct{
   	  int actualcase,status;
	  }intp_switch;
#define WATCHTABSIZE	2
   struct{
   	   int p,max;
	   watchelement tab[WATCHTABSIZE];
	   }watchtab;
#define LOGVARMAX 50

#define CPI 0

enum {NUMTYP,VARTYP,OPTYP};
enum {O_GT,O_LT,O_AND,O_COND};

#define VARPOOLSIZE 10000
typedef struct{
                 int typ;
		 union {
		        double cst;
			symrec *tp;
			char op;
			}val;
	      }codeel;

  symrec * getsym (const char *sym_name, symboltable tab);





