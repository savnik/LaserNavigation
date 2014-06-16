/*
 *  FILE   : queue.c
 *  AUTHOR : Jeffrey Hunter
 *  WEB    : http://www.iDevelopment.info
 *  NOTES  : Implement all functions required
 *           for a Queue data structure.
 */

#include "queue.h"
#include <stdlib.h>

#define MinQueueSize (5)

struct QueueRecord {
  int Capacity;
  int Front;
  int Rear;
  int Size;
  int elementsize;
  void *Array;
};

int IsEmpty(Queue Q) {
  return Q->Size == 0;
}

int IsFull(Queue Q) {
  return Q->Size == Q->Capacity;
}

Queue CreateQueue(int MaxElements,int elementsize) {
  Queue Q;

  if (MaxElements < MinQueueSize) {
    Error("CreateQueue Error: Queue size is too small.");
  }

  Q = malloc (sizeof(struct QueueRecord));
  if (Q == NULL) {
    FatalError("CreateQueue Error: Unable to allocate more memory.");
    return Q;
  }
  Q->elementsize=elementsize;
  
  Q->Array = malloc( elementsize * MaxElements );
  if (Q->Array == NULL) {
    FatalError("CreateQueue Error: Unable to allocate more memory.");
  }

  Q->Capacity = MaxElements;
  MakeEmpty(Q);

  return Q;
}

void MakeEmpty(Queue Q) {

  Q->Size = 0;
  Q->Front = 1;
  Q->Rear = 0;

}

void DisposeQueue(Queue Q) {
  if (Q != NULL) {
    free(Q->Array);
    free(Q);
  }
}

static int Succ(int Value, Queue Q) {
  if (++Value == Q->Capacity) {
    Value = 0;
  }
  return Value;
}

int Enqueue( Queue Q,void * data) {

  if (IsFull(Q)) {
    Error("Enqueue Error: The queue is full.");
    return -1;
  } else {
    Q->Size++;
    Q->Rear = Succ(Q->Rear, Q);
    memcpy(Q->Array+(Q->Rear*Q->elementsize),data,Q->elementsize);
  }

}

int Front(Queue Q,void * data) {

  if (!IsEmpty(Q)) {
    memcpy(data,Q->Array+(Q->Front*Q->elementsize),Q->elementsize);
    return 0;
  }
  Error("Front Error: The queue is empty.");

  /* Return value to avoid warnings from the compiler */
  return 0;

}

void Dequeue(Queue Q) {

  if (IsEmpty(Q)) {
    Error("Dequeue Error: The queue is empty.");
  } else {
    Q->Size--;
    Q->Front = Succ(Q->Front, Q);
  }

}

int FrontAndDequeue(Queue Q,void * data) {

  if (IsEmpty(Q)) {
    Error("FrontAndDequeue Error: The queue is empty.");
    return -1;
  } else {
    Q->Size--;
    memcpy(data,Q->Array+(Q->Front*Q->elementsize),Q->elementsize);
    Q->Front = Succ(Q->Front, Q);
    return 0;
  }

}
