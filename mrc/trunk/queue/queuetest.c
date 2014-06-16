/*
 *  FILE   : testQueue.c
 *  AUTHOR : Jeffrey Hunter
 *  WEB    : http://www.iDevelopment.info
 *  NOTES  : Example program that tests the
 *           Queue implementation.
 */

#include <stdio.h>
#include "queue.h"

main() {

  Queue Q;
  int i;
  double a;
  char buf[256];

  printf("\n");
  printf("Create Queue(15)...\n\n");
  Q = CreateQueue(15,256);

  printf("Enqueue 10 elements...\n");
  for (i=0; i<10; i++) {
    
    Enqueue( Q,"kuk");
  }

  printf("Print all 10 elements...\n");
  while (!IsEmpty(Q)) {
    Front(Q,buf);
    printf("%s ",buf );
    Dequeue(Q);
  }
  printf("\n\n");

  printf("Enqueue 10 more elements...\n");
  for (i=10; i<20; i++) {
   
    Enqueue(Q,"langstreng");
  }

  printf("Print the new queue...\n");
  while (!IsEmpty(Q)) {
    Front(Q,buf);
    printf("%s ",buf );
    Dequeue(Q);
  }
  printf("\n\n");

  printf("Dispose of the queue...\n");
  DisposeQueue(Q);
  printf("\n");

  return 0;

}
