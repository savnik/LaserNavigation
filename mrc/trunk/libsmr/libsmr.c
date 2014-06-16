#include "smr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>

#include <sys/types.h>

#include <sys/socket.h>
#include <netinet/in.h>        
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>

int enet_connect(char *hostname, int port)
{
  struct sockaddr_in sa;
  int s;

  if (!hostname) 
    {
      char *smr = getenv("SMR");

      if (smr)
	{
	  char *p;

	  hostname = strtok(smr, ":");
	  p = strtok(0, "");
	  if (p)  port = atoi(p);
	}
    }

  if (hostname)
    {
      struct hostent *h = gethostbyname(hostname);
      if (!h)
	{
	  fprintf(stderr, "gethostbyname: can't find %s.\n", hostname);
	  return -1;
	}
      sa.sin_family = h->h_addrtype;
      memcpy(&sa.sin_addr, h->h_addr_list[0], h->h_length);
    }
  else
    {
      sa.sin_family = AF_INET;
      sa.sin_addr.s_addr = INADDR_LOOPBACK;
      inet_aton("127.0.0.1", &sa.sin_addr);
    }

  sa.sin_port = htons(port ? port : SMR_PORT);

  if ((s = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("socket");
      return -1;
    }

  if (connect(s, (struct sockaddr *)&sa, sizeof(sa)))
    {
      perror("connect");
      close(s);
      return -1;
    }

  {
    int x = 1;

    if (setsockopt(s, SOL_TCP, TCP_NODELAY, &x, sizeof(x)))
      perror("setsockopt");
  }

  return s;
}

int enet_read(int fd, unsigned char *p)
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

int enet_write(int fd, unsigned char *p)
{
  return write(fd, p, ENET_BYTES(p));
}

void enet_fprint(FILE *f, unsigned char *msg)
{
  int len = msg[0] & 0x1f;
  int type = msg[0] >> 5;
  char *name[8] = {"rs485", "rs232", "?", "?", "?", "?", "?", "control"};
  fprintf(f, "[%s", name[type]);
  if (len)
    {
      int i;
      fprintf(f, ": %2x", msg[1]);
      for (i = 2; i <= len; i++)
	fprintf(f, " %2x", msg[i]);
    }
  fprintf(f, "]\n");
}

void enet_print(unsigned char *msg)
{
  enet_fprint(stdout, msg);
}

struct smr *smr_connect(char *hostname, int port)
{
  struct smr *smr = malloc(sizeof(struct smr));
  int i;

  if (!smr)
    {
      fprintf(stderr, "smr_connect: out of memory.\n");
      return 0;
    }
  smr->read_flags = 0;
  smr->wait_flags = SMR_FLAG_LE | SMR_FLAG_RE | SMR_FLAG_IR | SMR_FLAG_LS;
  smr->write_flags = SMR_FLAG_LV | SMR_FLAG_RV;
  smr->left.encoder = 0;
  smr->left.speed = 0;
  smr->right.encoder = 0;
  smr->right.speed = 0;
  for (i = 0; i < SMR_IR_N; i++) smr->ir[i] = 0;
  for (i = 0; i < SMR_LS_N; i++) smr->ls[i] = 0;
  smr->tick = 0;
  smr->ts = 0;
  smr->recv_hook = 0;
  smr->reverse_right = 1;
  if ((smr->enet = enet_connect(hostname, port)) < 0)
    {
      free(smr);
      return 0;
    }
  return smr;
}

void smr_disconnect(struct smr *smr)
{
  smr_read(smr);
  smr_read(smr);
  smr->left.speed = 0;
  smr->right.speed = 0;
  smr_write(smr);
  smr_read(smr);

  close(smr->enet);
  free(smr);
}

int smr_read(struct smr *smr)
{
  unsigned char *buf = smr->msg;
  int f232 = 1;

  {
    unsigned char msg[] = {ENET_MSG_CONTROL_SEND};
    enet_write(smr->enet, msg);
  }

  do				/* flush out any old messages */
    {
      enet_read(smr->enet, buf);
      if (smr->recv_hook && (ENET_TYPE(buf) == ENET_TYPE_485 || ENET_TYPE(buf) == ENET_TYPE_SIM )) smr->recv_hook(smr);
    }
  while ((ENET_TYPE(buf) != ENET_TYPE_CONTROL) ||
	 (buf[1] != ENET_CONTROL_START));

  smr->tick = (*(unsigned int *)(buf + 2));
  smr->ts = (*(unsigned int *)(buf + 6));

  smr->read_flags = 0;

  while (~smr->read_flags & smr->wait_flags)
    {
      enet_read(smr->enet, buf);
      switch (ENET_TYPE(buf))
	{
	case ENET_TYPE_CONTROL:
	  if (ENET_BYTES(buf) > 1)
	    switch (buf[1])
	      {
	      case ENET_CONTROL_START:
		/* not possible! */
		break;
	      case ENET_CONTROL_END:
		/* end of data, return with what we have */
		return 0;
	      }
	  break;

	case ENET_TYPE_485:
	  if (ENET_BYTES(buf) > 1)
	    switch (buf[1])
	      {
	      case SMR_LEFT_ENCODER_RET:
		smr->left.encoder = (buf[2]<<8) + buf[3];
		smr->read_flags |= SMR_FLAG_LE;
		if (ENET_BYTES(buf) >= 6)
		  {
		    smr->left.status = buf[4];
		    smr->left.pwm = buf[5];
		    smr->left.pwm -= 128;
		    smr->read_flags |= SMR_FLAG_LPS;
		  }
		break;
	      case SMR_RIGHT_ENCODER_RET:
		smr->right.encoder = (buf[2]<<8) + buf[3];
		if (smr->reverse_right)
		  smr->right.encoder = -smr->right.encoder;
		smr->read_flags |= SMR_FLAG_RE;
		if (ENET_BYTES(buf) >= 6)
		  {
		    smr->right.status = buf[4];
		    smr->right.pwm = buf[5];
		    smr->right.pwm -= 128;
		    if (smr->reverse_right)
		      smr->right.pwm = -smr->right.pwm;
		    smr->read_flags |= SMR_FLAG_RPS;
		  }
		break;
	      case SMR_LS_RET:
		//if (ENET_BYTES(buf) >= 1+SMR_LS_N)
		if (ENET_BYTES(buf) >= 9)
		  {
		    int len;
		    len=ENET_BYTES(buf);
		    if (len > 32) len=32;
		    memcpy(smr->ls, buf+2, len);
		    smr->read_flags |= SMR_FLAG_LS;
		  }
		break;
	      case SMR_IR_RET:
		if (ENET_BYTES(buf) >= 1+SMR_IR_N)
		  {
		    memcpy(smr->ir, buf+2, SMR_IR_N);
		    smr->read_flags |= SMR_FLAG_IR;
		  }
		break;
	      case SMR_POWER_RET:
		if (ENET_BYTES(buf) == 9)
		  {
		    smr->status = buf[2] >> 2;
		    smr->ad[0] = ((buf[2] << 8) & 0x300) | buf[4];
		    smr->ad[1] = ((buf[3] << 2) & 0x300) | buf[5];
		    smr->ad[2] = ((buf[3] << 4) & 0x300) | buf[6];
		    smr->ad[3] = ((buf[3] << 6) & 0x300) | buf[7];
		    smr->ad[4] = ((buf[3] << 8) & 0x300) | buf[8];
		    smr->read_flags |= SMR_FLAG_PW;
		  }
		break;
	      default:
		smr->read_flags |= SMR_FLAG_AM;
		if (smr->recv_hook) smr->recv_hook(smr);
		break;
	      }
	  break;
	  
	case ENET_TYPE_232:
	  if (f232)
	    {
	      f232 = 0;
	      if ((ENET_BYTES(buf) >= 2+SMR_LS_N) && (buf[1] == 's')
		  && !memchr(buf+2, 0, SMR_LS_N)) /* filter out zero data */
		{
		  memcpy(smr->ls, buf+2, SMR_LS_N);
		  smr->read_flags |= SMR_FLAG_LS;
		}
	    }
	  break;
	}
    }
  return 0;
}

int smr_write(struct smr *smr)
{
  unsigned char buf[6 + ENET_BYTES_MAX], *p = buf;

  if (smr->write_flags & SMR_FLAG_LV)
    {
      int x = (smr->left.speed == -128) ? -127 : smr->left.speed;
      unsigned char msg[] = {SMR_MSG_LEFT_SPEED(x)};
      memcpy(p, msg, sizeof(msg));
      p += sizeof(msg);
    }
  if (smr->write_flags & SMR_FLAG_RV)
    {
      int x = (smr->right.speed == -128) ? -127 : smr->right.speed;
      unsigned char msg[] = {SMR_MSG_RIGHT_SPEED(smr->reverse_right ? -x : x)};
      memcpy(p, msg, sizeof(msg));
      p += sizeof(msg);
    }
  if (smr->write_flags & SMR_FLAG_AM)
    {
      memcpy(p, smr->msg, ENET_BYTES(smr->msg));
      p += ENET_BYTES(smr->msg);
    }
  if (p == buf) return -1;

  write(smr->enet, buf, p - buf);
  return 0;
}
