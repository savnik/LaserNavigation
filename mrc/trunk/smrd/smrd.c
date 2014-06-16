
#include <sched.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#if 0
#define CONFIG_X86_TSC
#include <asm/timex.h>		/* cycles_t get_cycles(); */

#else
#define rdtsc(low,high) \
     __asm__ __volatile__("rdtsc" : "=a" (low), "=d" (high))

#define rdtscl(low) \
     __asm__ __volatile__("rdtsc" : "=a" (low) : : "edx")

#define rdtscll(val) \
     __asm__ __volatile__("rdtsc" : "=A" (val))


unsigned int get_cycles() {
  int x; rdtscl(x); return x;
}
#endif

#include <linux/serial.h>

#include "smr.h"

/*#define DEBUG*/
#define MS (get_cycles()/500000)

#define SERIAL1 "/dev/ttyS0"
#define SERIAL2 "/dev/ttyS1"

#define BLOCK_MAX 200
#define XMIT_BYTES 104		/* about 10ms at 115.2 Kbaud */

int set_serial(int fd);

void client_reset(void);
void client_update(unsigned char *p);
void xmit(void);

int rs485, rs232;		/* file descriptors */

void *rs485_task(void *), *rs232_task(void *);
void *server_task(void *), *client_task(void *); 
pthread_t rs485_thread, rs232_thread, server_thread, client_thread;
pthread_attr_t attr;

void print_sched()
{
  struct sched_param param;

  switch (sched_getscheduler(0))
    {
    case SCHED_OTHER:
      fprintf(stderr, "Scheduler: other\n");
      break;
    case SCHED_FIFO:
      sched_getparam(0, &param);
      fprintf(stderr, "Scheduler fifo, priority %d\n", param.sched_priority);
      break;
    case SCHED_RR:
      sched_getparam(0, &param);
      fprintf(stderr, "Scheduler rr, priority %d\n", param.sched_priority);
      break;
    default:
      perror("getscheduler");
      break;
    }
}

int main()
{
 int i;
 unsigned int clocks[10000][4];
#if 1
#include <sys/mman.h>
  
  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      perror("mlockall");
      exit(-1);
    }
  fprintf(stderr, "Running with memory locked\n");
#endif

  { /* use real-time (fixed priority) scheduler
     * set priority to one less than the maximum
     */
    struct sched_param param;

    param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
    if (sched_setscheduler(0, SCHED_RR, &param))
      {
	perror("setscheduler");
	exit(-1);
      }
#if 1
    print_sched();
#else
    fprintf(stderr, "Running with real-time priority %d\n",
	    param.sched_priority);
#endif
  }

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
    fprintf(stderr, "signal: can't ignore SIGPIPE.\n");


  {
    int ttyS0, ttyS1;

    if ((ttyS0 = open (SERIAL1, O_RDWR /*| O_NONBLOCK*/)) == -1)
      {
	perror("Can't open first serial port");
	exit(-1);
      }
    if (set_serial(ttyS0) == -1)
      {
	perror("Can't set first serial port parameters");
	exit(-1);
      }
	     
    if ((ttyS1 = open (SERIAL2, O_RDWR /*| O_NONBLOCK*/)) == -1)
      {
	perror("can't open second serial port");
      }
    else if (set_serial(ttyS1) == -1)
      {
	perror("Can't set second serial port parameters");
	close(ttyS1);
	ttyS1 = -1;
      }

    if (ttyS1 == -1)
      {
	rs485 = ttyS0;
	rs232 = -1;
      }
    else
      {
	rs485 = ttyS1;
	rs232 = ttyS0;
      }
  }

  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED); 
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&rs485_thread, &attr, rs485_task, 0))
    {
      perror("can't start receive thread");
      exit(-1);
    }

  if (rs232 != -1)
    if (pthread_create(&rs232_thread, &attr, rs232_task, 0))
      {
	perror("can't start linesensor receive thread");
	exit(-1);
      }

  if (pthread_create(&server_thread, &attr, server_task, 0))
    {
      perror("can't start server thread");
      exit(-1);
    }
  i=0;
  while (1)
    {
      static unsigned char tick[] = {ENET_TYPE_CONTROL | 9, ENET_CONTROL_START,
      				     0, 0, 0, 0, 0, 0, 0, 0};
   // i++; 
#ifdef DEBUG
      fprintf(stderr, "<%lu xmit: ", MS);
#endif
      client_reset();

      (*(unsigned int *)(tick + 2))++;
      clocks[i][0]= get_cycles();
      (*(unsigned int *)(tick + 6)) =clocks[i][0];
#ifdef DEBUG
      fprintf(stderr, "tick %d ", (*(unsigned int *)(tick + 2)));
#endif
      client_update(tick);
      clocks[i][1]= get_cycles();
      tcflush(rs485, TCIOFLUSH);
      xmit();
#ifdef DEBUG
      fprintf(stderr, " :xmit %lu>\n", MS);
#endif
      clocks[i][2]= get_cycles();
//      clocks[i][0]=tcdrain(rs485); /* waits until all output written has been transmitted.
//		       * under linux, also seems to wait a clock tick. */
       usleep(8000);
		        clocks[i][3]= get_cycles();
    }
    {
      FILE *f;
      f=fopen("timelog","w");
      for (i=0;i<1000;i++)
        fprintf(f,"%d  %d %d  %d \n",clocks[i][0],clocks[i][1],clocks[i][2],
	clocks[i][3]);
      fclose(f);
    }      

  return 0;
}



static int msg_read(int fd, unsigned char *p)
{
  int n, c, ret;

  n = read(fd, p, 1);
  if (n <= 0) return n;
  ret = ENET_BYTES(p);
  c = ret - 1;
  p++;
  while (c)
    {
      n = read(fd, p, c);
      if (n <= 0) return n;
      p += n;
      c -= n;
    }
  return ret;
}

static int msg_write(int fd, unsigned char *p)
{
  int c = ENET_BYTES(p);
  write(fd, p, c);
  return c;
}


int set_serial(int fd)
{
  struct termios tios;

  if (tcgetattr(fd, &tios))  return -1;

  tios.c_iflag = 
    IGNPAR;			/* ignore framing errors (read as 0) */
  tios.c_oflag = 0;
  tios.c_cflag = (CS8
		  | CREAD	/* enable receiver ? */
		  | CLOCAL	/* ignore modem control lines */
#if 0
		  | CRTSCTS	/* flow control */
#endif
		  );
  tios.c_lflag = 0;
  
  cfsetospeed (&tios, B115200);
  cfsetispeed (&tios, B115200); 

  if (tcsetattr(fd, TCSANOW, &tios))  return -1;

#if 1
  {
  /* request low-latency from kernel
   */
    struct serial_struct ss;
    ioctl(fd, TIOCGSERIAL, &ss);
    ss.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &ss);
  }
#endif

  return 0;
}


static int rs485_pad(int c)
{
  int i;
  char z = 0;
  for (i = 0; i < c; i++)
    write(rs485, &z, 1);
  return c;
}

void *rs485_task(void *not_used)
{
  fprintf(stderr, "rx_task running\n");
  print_sched();

  tcflush(rs485, TCIFLUSH);
  while (1)
    {
      unsigned char buf[ENET_BYTES_MAX];
      int n;

      if ((n = msg_read(rs485, buf)) < 0) perror("rs485");
#ifdef DEBUG
      fprintf(stderr, "<%lu rs485: len %d ", MS, n);
#endif

#if 0
      fprintf(stderr, "rs485 %d bytes (%2x %2x..)\n",
	      n, buf[0], buf[1]);
#endif
      client_update(buf);

#ifdef DEBUG
      fprintf(stderr, " :rs485 %lu>\n", MS);
#endif


#define POWER_OFF_COUNT 25

      if ((buf[0] == 0x08) && (buf[1] == 0x19))	/* power supply data message */
	{
	  static int power_off = 0;

	  if (power_off != -1)
	    {
	      if (buf[2] & 0x04)	/* power switch on */
		{
		  if (power_off > 0)
		    fprintf(stderr, "Reseting power_off (%d)!\n", power_off);

		  power_off = 0;
		}
	      else		/* power switch off */
		{
		  power_off++;
		  if (power_off == POWER_OFF_COUNT)
		    {
		      fprintf(stderr, "Power_off count reached limit (%d).\n",
			      POWER_OFF_COUNT);

		      power_off = -1;
		      system("/sbin/poweroff &");
		    }
		}
	    }
	}
    }
}


#define LS_READ_BYTES 20
char rs232_command = '?';

void rs232_send(unsigned char *p)
{
#ifdef DEBUG
  int c = ENET_BYTES(p);
  if (c != 2) fprintf(stderr, "funny line sensor command, length %d.\n", c);
#endif
  rs232_command = p[1];

  if (write(rs232, &rs232_command, 1) != 1)
    {
      perror("rs232_send");
    }
}

void *rs232_task(void *not_used)
{
  fprintf(stderr, "lsrx_task running\n");
  print_sched();

  while (1)
    {
      unsigned char buf[ENET_BYTES_MAX];
      int c = read(rs232, buf + 2, LS_READ_BYTES);

#ifdef DEBUG
      fprintf(stderr, "<%lu rs232: %d bytes ", MS, c);
#endif

      if (c <= 0)
	{
	  fprintf(stderr, "error reading rs232");
	  pthread_exit(0);
	}
      buf[0] = (c + 1) | ENET_TYPE_232;
      buf[1] = rs232_command;
      client_update(buf);

#ifdef DEBUG
      fprintf(stderr, " :rs232 %lu>\n", MS);
#endif
    }      
}


/*
 * Server
 */

#define SERVER_PORT 24901
#define CLIENTS_MAX 20
#define CLIENT_MSGS_MAX 10

struct client {
  pthread_t pt;
  int fd;
  unsigned char buf[CLIENT_MSGS_MAX][ENET_BYTES_MAX];
				/* messages from client */
  volatile int next;			/* next free message buffer */
  volatile int send;			/* next buffer to send (if != next) */
  volatile int ready;			/* >0 to enable data to client */
} clients[CLIENTS_MAX];

void *server_task(void *not_used)
{
  int s, i, client;

  fprintf(stderr, "server task running\n");
  print_sched();

  for (i = 0; i < CLIENTS_MAX; i++)
    {
      clients[i].fd = -1;
      clients[i].next = 0;
      clients[i].send = 0;
      clients[i].ready = 0;
    }

  if ((s = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("server: socket");
      pthread_exit(0);
    }

  {
    struct sockaddr_in sa;
    sa.sin_family =  AF_INET;
    sa.sin_port = htons(SERVER_PORT);
#if 0
    inet_aton("127.0.0.1", &sa.sin_addr);
#else
    sa.sin_addr.s_addr = INADDR_ANY;
#endif
    if (bind(s, &sa, sizeof(sa)))
      {
	perror("server: bind");
	pthread_exit(0);
      }
  }

  if (listen(s, 1))
      {
	perror("server: listen");
	pthread_exit(0);
      }

  while ((client = accept(s, 0, 0)) >= 0)
    {
#ifdef DEBUG
      fprintf(stderr, "<server: ");
#endif

      for (i = 0; i < CLIENTS_MAX; i++)
	if (clients[i].fd == -1)
	  {
	    clients[i].fd = client;
	    clients[i].ready = 0;
	    clients[i].next = 0;
	    clients[i].send = 0;
	    if (pthread_create(&clients[i].pt, &attr,
			       client_task, clients+i))
	      {
		perror("can't start client thread");
		clients[i].fd = -1;
		close(client);
	      }
	    break;
	  }
#ifdef DEBUG
      fprintf(stderr, " :server>\n");
#endif
      if (i == CLIENTS_MAX) close(client);
    }

  perror("server: accept");
  pthread_exit(0);
}


volatile unsigned char recv_buffer[BLOCK_MAX];
volatile unsigned char *recv_buffer_next = recv_buffer;

void *client_task(void *arg)
{
  struct client *p = arg;
  int c;

  {
    int x = 1;

    if (setsockopt(p->fd, SOL_TCP, TCP_NODELAY, &x, sizeof(x)))
      perror("setsockopt");
  }

  p->next = 0;
  p->send = 0;

  while((c = msg_read(p->fd, p->buf[p->next])) > 0)
    {
#ifdef DEBUG
      fprintf(stderr, "<%lu client %d: ", MS, p-clients);
#endif

#ifdef DEBUG
      fprintf(stderr, "client msg %d bytes (%2x %2x %2x..)\n", c,
	      p->buf[p->next][0], p->buf[p->next][1], p->buf[p->next][2]);
#endif
      if (ENET_TYPE(p->buf[p->next]) == ENET_TYPE_CONTROL)
	switch (p->buf[p->next][1])
	  {
	  case ENET_CONTROL_SEND:
	    if (!p->ready)
	      write(p->fd, recv_buffer, recv_buffer_next - recv_buffer);
	    p->ready++;
	    if (p->ready > 2) p->ready = 2;
#ifdef DEBUG
	    fprintf(stderr, "ready%d = %d\n", p-clients, p->ready);
#endif
	    break;
	  }
#if 0
      else if ((p->buf[p->next][0] == ENET_TYPE_485 | 2) &&
	       ((b->buf[p->next][1] == 0x11) || (b->buf[p->next][1] == 0x12)))
	{
	  if (b->buf[p->next][1] == 0x11)
	    motor_lr = b->buf[p->next][2];
	  else
	    motor_lr = b->buf[p->next][2];
	}
#endif
      else
	{
	  if (++p->next == CLIENT_MSGS_MAX) p->next = 0;
	}
#ifdef DEBUG
      fprintf(stderr, " :client %d %lu>\n", p-clients, MS);
#endif
    }
  if (c) perror("can't read from client");

  close(p->fd);
  p->fd = -1;
  p->next = 0;
  p->send = 0;
  p->ready = 0;
  pthread_exit(0);
}

void client_reset()
{
  int i;
  for (i = 0; i < CLIENTS_MAX; i++)
    if (clients[i].ready)
      {
	unsigned char msg[] = {ENET_TYPE_CONTROL|1, ENET_CONTROL_END};
	msg_write(clients[i].fd, msg);	
	clients[i].ready--;
#ifdef DEBUG
	fprintf(stderr, "decr ready%d: %d\n", i, clients[i].ready);
#endif
      }

  recv_buffer_next = recv_buffer;
}

void client_update(unsigned char *p)
{
  int i;

  memcpy(recv_buffer_next, p, ENET_BYTES(p));
  recv_buffer_next += ENET_BYTES(p);

  for (i = 0; i < CLIENTS_MAX; i++)
    if (clients[i].ready)
      {
#ifdef DEBUG
	fprintf(stderr, "client %d, %d bytes\n", i, ENET_BYTES(p));
#endif
	msg_write(clients[i].fd, p);
      }
}

void client_msg_sent(struct client *p)
{
  client_update(p->buf[p->send]);
  if (++p->send == CLIENT_MSGS_MAX) p->send = 0;
  if (p->send == p->next) 
    {
#ifdef DEBUG
      fprintf(stderr, "all sent\n");
#endif
    }
}

void xmit(void)
{
  static int index = 0;
  int rs232_flag = 0;
  int left_flag = 0;
  int right_flag = 0;
  int i = index;
  int xcount = 0;

  do				/* check for special priority messages */
    {
    again:
      if (clients[i].send != clients[i].next)
	switch (ENET_TYPE(clients[i].buf[clients[i].send]))
	  {
	  case ENET_TYPE_232:
	    if ((rs232 != -1) && !rs232_flag)
	      {
		rs232_send(clients[i].buf[clients[i].send]);
		client_msg_sent(clients + i);
		rs232_flag = 1;
		goto again;
	      }
	    break;
	    
	  case ENET_TYPE_485:
	    if (clients[i].buf[clients[i].send][1] == 0x11)
	      {
		if (!right_flag)
		  xcount += msg_write(rs485, clients[i].buf[clients[i].send]);
		client_msg_sent(clients + i);
		right_flag = 1;
		goto again;
	      }
	    else if (clients[i].buf[clients[i].send][1] == 0x12)
	      {
		if (!left_flag)
		  xcount += msg_write(rs485, clients[i].buf[clients[i].send]);
		client_msg_sent(clients + i);
		left_flag = 1;
		goto again;
	      }
	    break;

	  default:		/* just get rid of it */
	    client_msg_sent(clients + i);
	    goto again;
	  }

      if (++i == CLIENTS_MAX) i = 0;
    }
  while (i != index);

  if ((rs232 != -1) && !rs232_flag)
  {				/* get line sensor data */
    static unsigned char msg[] = {ENET_TYPE_232 | 1, 's'};
    rs232_send(msg);
  }

  {				/* get right encoder */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x21};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(10);
  }
  {				/* get left encoder */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x22};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(10);
  }
  {				/* get (rs485) linesensor */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x17};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(16);
  }
  {				/* get proximity encoders */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x88};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(16);
  }
  {				/* get power status */
    static unsigned char msg[] = {ENET_TYPE_485 | 1, 0x19};
    xcount += msg_write(rs485, msg);
    xcount += rs485_pad(16);
  }

  do
    {
      if (clients[i].send != clients[i].next)
	switch (ENET_TYPE(clients[i].buf[clients[i].send]))
	  {
	  case ENET_TYPE_485:
	    xcount += msg_write(rs485, clients[i].buf[clients[i].send]);
	    client_msg_sent(clients + i);
	    index = i;
	    if (++index == CLIENTS_MAX) index = 0;

	    goto alldone;
	  }

      if (++i == CLIENTS_MAX) i = 0;
    }
  while (i != index);

 alldone:
  {
    int x = XMIT_BYTES - xcount;
    if (x < 0)
      fprintf(stderr, "xmit overflow %d\n", -x);
    else
      rs485_pad(x);
  }

  return;
}
