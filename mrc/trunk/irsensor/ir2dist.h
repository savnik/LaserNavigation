#define REFRESH_TIME 18 // it has to be a multiple of the cycle time 18*10ms

typedef struct  {
	double k1,k2;
	}irparamtype;

typedef struct {
	double lval;
	int newval;
	int repeated;
	}irsamptype;

void ir2dist(double dist[],unsigned char irval[],irparamtype par[],int N);
void ir2dist1(double dist[],unsigned char irval[],irparamtype par[],irsamptype samp[],int N);
int calc_ir_param(irparamtype param[],double d1[],double ir1[],double d2[],double ir2[],int N);
