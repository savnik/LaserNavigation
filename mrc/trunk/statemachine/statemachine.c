#include "stdio.h"
#include "statemachine.h"

#define SM_POOL_LEN  10

 
struct {
	int stat;
        sm_type data;
	} sm_datapool[SM_POOL_LEN];

sm_type * get_sm_data(void){
  int i=0;
  while ( i < SM_POOL_LEN){
    if (sm_datapool[i].stat==0){
      sm_datapool[i].stat=1;
      return &sm_datapool[i].data;
    }
    i++;
  }
  return NULL;
}

void sm_datapool_init(void){
  int i;
  for (i=0;i<SM_POOL_LEN;i++)
    sm_datapool[i].stat=0;
}

int free_sm_data(sm_type * sm){
  int i=0;
  while ( i < SM_POOL_LEN){
    if (&sm_datapool[i].data == sm){
      sm_datapool[i].stat=0;
     // printf("datapool no %d \n",i);
      return 0;
    }
    i++;
  }
  return 1;
}

void sm_reset(sm_type *sm){
  sm->state=0;
  sm->oldstate=-1;
  sm->time=-1;
  sm->status=SM_RESET;
  sm->startdist=0;
}

void sm_update(sm_type *sm){
 if (sm->state != sm->oldstate){
    sm->time=-1;
    sm->oldstate=sm->state;
    sm_userupdate(sm);

 //  printf("State %d \n",sm->state);
 }
 sm->time++;
}
