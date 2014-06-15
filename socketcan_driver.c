#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
 
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>

#include "erl_driver.h"
#include "ei.h"

#define RESULT_BUF_MAX_LEN 100

typedef struct {
  char *buf;
  int len;
} port_msg;

typedef int (*port_fcn)(const port_msg, int, ei_x_buff*);

typedef struct {
  const char *name;
  int arity;
  port_fcn fcn;
} port_function;

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

static int can_send(int s, uint32_t can_id, uint8_t length, unsigned char data[8])
{
  struct can_frame frame;

  frame.can_id  = can_id;
  frame.can_dlc = length;
  memcpy(frame.data, data, length);

  return write(s, &frame, sizeof(struct can_frame));
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

static void set_error(ei_x_buff *msg, int result)
{
  ei_x_encode_atom(msg, "error");
  ei_x_encode_long(msg, result);
}

static int twice_fn(const port_msg in, int index, ei_x_buff *out)
{
  long value;
  if (ei_decode_long(in.buf, &index, &value)) {
    set_error(out, 7);
    return 0;
  }

  ei_x_encode_atom(out, "ok");
  ei_x_encode_long(out, value * 2);
  return 0;
}

port_function functions[] = {
  {"twice", 1, twice_fn},
  {"", 0, NULL} // guard
};

static void example_drv_output(ErlDrvData handle, char *buff, int bufflen)
{
  example_data *d = (example_data*)handle;
  port_msg in_msg = {buff, bufflen};

  char command[MAXATOMLEN];
  int index, version, arity;
  ei_x_buff result;

  index = 0;

  ei_x_new_with_version(&result);
  ei_x_encode_tuple_header(&result, 2);

  if (ei_decode_version(buff, &index, &version)) {
    set_error(&result, 1);
    goto stop_processing;
  }

  if (ei_decode_tuple_header(buff, &index, &arity)) {
    set_error(&result, 2);
    goto stop_processing;
  }

  if (ei_decode_atom(buff, &index, command)) {
    set_error(&result, 4);
    goto stop_processing;
  }

  port_function *current_function = &functions[0];

  while (1)
  {
    if (!current_function->fcn)
    {
      set_error(&result, 10);
      break;
    }

    if (!strcmp(current_function->name, command))
    {
      if (current_function->arity != arity-1) // tuple has additional "cmd" term
      {
        set_error(&result, 5);
        break;
      }
      current_function->fcn(in_msg, index, &result);
      break;
    }

    current_function++;
  }

  //char fn = buff[0], arg = buff[1], res;

  //if (fn == 1) {
  //  res = twice(arg);
  //} else if (fn == 2) {
  //  res = sum(buff[1], buff[2]);
  //} else if (fn == 3) {
  //  char *port = malloc(bufflen); /* -1 for first arg, +1 for terminator */
  //  strncpy(port, buff+1, bufflen-1);
  //  port[bufflen-1] = '\0';
  //  res = open(buff+1);
  //} else if (fn == 4) {
  //  if (bufflen < 4) {
  //    res = 255;
  //  } else {
  //    int data_len = bufflen - 4;
  //    data_len = min(data_len, 8);
  //    uint16_t can_id = *((uint16_t*) &buff[2]);
  //    res = can_send(arg, can_id, data_len, &buff[4]);
  //  }
  //}

stop_processing:
  {
    int out_size = result.index + 2;
    //int out_size = result.index;
    //ErlDrvBinary *bin_out = driver_alloc_binary(out_size);
    //char *bin_data = bin_out->orig_bytes;
    //memcpy(bin_data, &result.index, sizeof(result.index));
    //bin_data[0] = (result.index >> 8) & 0xff;
    //bin_data[1] = result.index & 0xff;
    //memcpy(bin_data + 2, result.buff, result.index);
    //driver_output_binary(d->port, NULL, 0, bin_out, 0, out_size);
    //free(buf_out);
    driver_output(d->port, result.buff, result.index);
    if (d->connecting) 
      driver_select(d->port, (ErlDrvEvent)d->socket, DO_WRITE, 0);
  }

  //ei_x_free(&result);
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
