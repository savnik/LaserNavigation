
typedef struct{
   double e;
   double k,tau,alfa,ts;
   double a,b,state;
   double o;
}pd_type;

void pd_init(pd_type *p);
void pd_out(pd_type *p);
void pd_update(pd_type *p);
