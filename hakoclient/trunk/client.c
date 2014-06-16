#include "client.h"
#include "functions.h"
#include <semaphore.h>
#define DEBUG 0
extern sem_t cmdsem;
void client_init();

void sig_pipe(int n) 
{
  fprintf(stderr, "Broken pipe signal\n");
  close(ct.sockfd);
  ct.sockfd = 0;
 	socket_busy = 0;
  exit_flag = 1;
}

void cl_connect()
{
  struct sockaddr_in serv_addr;
  struct hostent *host_param;

  if((ct.sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
      fprintf(stderr, "Can not create socket\n");
      ct.error = 1;
      return;
    }
  int x = 0;
  if (setsockopt(ct.sockfd, SOL_TCP, TCP_NODELAY, &x, sizeof(x)))
 	{
  	fprintf(stderr, "Error setting Socket options\n");
  }

  
  if((host_param = gethostbyname(ct.hostname)) == NULL)
    {
      fprintf(stderr, "Can not connect to %s\n", ct.hostname);
      ct.error = 2;
      return;
    }
  
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *) host_param->h_addr, (char *) &serv_addr.sin_addr.s_addr, host_param->h_length);
  serv_addr.sin_port = htons(ct.port);
  
  if(connect(ct.sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) != 0)
    {
      fprintf(stderr, "Error while trying to connect to socket on: %s, port: %d\n", ct.hostname, ct.port);
      ct.error = 3;
      return;
    }
  
  client_init(); // Initialize client
  ct.error = 0;
  return;
}

void cl_disconnect()
{
  send(ct.sockfd, "exit\n", 5, 0);
  
  usleep(100000);
  close(ct.sockfd);
  
  ct.error = 0;
  return;
}

int cl_send(const char *cmd)
{
  int len = 0;
   len = send(ct.sockfd, cmd, strlen(cmd), 0);
   printf(" cmd= %s \n",cmd);
  if(len != strlen(cmd))
    ct.error = 1;
  else
    ct.error = 0;

  return len;
}

void cl_cmdwait(char *cmd)
{
  
  cl_send(cmd);
  cl_send("syncevent \"wait\" \n");  
  sem_wait(&cmdsem);  
  printf("released \n");
  return;
}

void client_init()
{
  ct.answer_len = 0;
  ct.queue_cnt = 0;
   
  return;
}
