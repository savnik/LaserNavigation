#ifndef CLIENT_H
#define CLIENT_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>

#define STRING_LENGTH 1000


typedef struct
{
  // User input:
  char *hostname;
  int port;
    
  // Internal variables:
  char answer[STRING_LENGTH];
  char queue[STRING_LENGTH][STRING_LENGTH];
  int answer_len;
  int queue_cnt;

  // Error value:
  int error;   // Returns a number different from 0 if an error occurs

  // Output value:
  int sockfd;

} clienttype;

void cl_connect();
void cl_disconnect();
int cl_send(const char *cmd);
void cl_cmdwait(char *cmd);
void sig_pipe(int n);

#endif // KLIENT_DOT_H
