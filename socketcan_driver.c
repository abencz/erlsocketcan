#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
 
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>

#include "erl_driver.h"

static int open(char* port)
{
  int s;
  struct sockaddr_can addr;
  struct ifreq ifr;

  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    // TODO
    return -1;
  }

  strcpy(ifr.ifr_name, port);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Error in socket bind");
    // TODO
    return -2;
  }

  return s;
}

static int twice(int n)
{
  return 2 * n;
}

static int sum(int n1, int n2)
{
  return n1 + n2;
}

typedef struct {
  ErlDrvPort port;
} example_data;

static ErlDrvData example_drv_start(ErlDrvPort port, char *buff)
{
  example_data *d = (example_data*)driver_alloc(sizeof(example_data));
  d->port = port;
  return (ErlDrvData) d;
}

static void example_drv_stop(ErlDrvData handle)
{
  driver_free((char*)handle);
}

static void example_drv_output(ErlDrvData handle, char *buff, int bufflen)
{
  example_data *d = (example_data*)handle;
  char fn = buff[0], arg = buff[1], res;

  if (fn == 1) {
    res = twice(arg);
  } else if (fn == 2) {
    res = sum(buff[1], buff[2]);
  } else if (fn == 3) {
    char *port = malloc(bufflen); /* -1 for first arg, +1 for terminator */
    strncpy(port, buff+1, bufflen-1);
    port[bufflen-1] = '\0';
    res = open(buff+1);
  }

  driver_output(d->port, &res, 1);
}

ErlDrvEntry example_driver_entry = {
  NULL,
  example_drv_start,
  example_drv_stop,
  example_drv_output,
  NULL,
  NULL,
  "socketcan_driver",
  NULL,
  NULL,
  NULL,
  NULL
};
  
DRIVER_INIT(socketcan_driver)
{
  return &example_driver_entry;
}
