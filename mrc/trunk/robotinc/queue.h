/*
 *  FILE   : queue.h
 *  AUTHOR : Jeffrey Hunter
 *  Modified by Nils Axel Andersen
 *  WEB    : http://www.iDevelopment.info
 *  NOTES  : Define queue record structure and
 *           all forward declarations.
 */

#include <stdio.h>
#include <stdlib.h>

#define Error(Str)        FatalError(Str)
#define FatalError(Str)   fprintf(stderr, "%s\n", Str), exit(1)


#ifndef _Queue_h

  struct QueueRecord;
  typedef struct QueueRecord *Queue;

  int         IsEmpty(Queue Q);
  int         IsFull(Queue Q);
  Queue       CreateQueue(int MaxElements,int elementsize);
  void        DisposeQueue(Queue Q);
  void        MakeEmpty(Queue Q);
  int         Enqueue(Queue Q,void *data);
  int	      Front(Queue Q,void * data);
  void        Dequeue(Queue Q);
  int	      FrontAndDequeue(Queue Q, void *data);

#endif  /* _Queue_h */

