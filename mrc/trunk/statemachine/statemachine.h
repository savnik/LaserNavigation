#include "mobilerobot.h"
enum{SM_RESET,SM_RUNNING,SM_FINISHED,SM_FAILED};

typedef struct sm_data{
	int state,oldstate,time,status;
	double startdist;
	posetype startpose;
	double p[10];
	struct sm_data *sm1;
	}sm_type;

sm_type * get_sm_data(void);
void sm_datapool_init(void);
int free_sm_data(sm_type * sm);
void sm_reset(sm_type *sm);
void sm_update(sm_type *sm);
void sm_userupdate(sm_type *sm);

int sm_resetmotors(sm_type *sm);
int sm_barcode(sm_type *sm);
