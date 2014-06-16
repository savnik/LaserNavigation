#ifndef SERVER1_H
#define SERVER1_H
typedef struct{
          int s,ls, port;
          struct sockaddr_in name,from;
        }servertype; 

int startserver(servertype *p);
int stopserver(servertype *p);

#endif
