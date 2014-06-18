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

typedef struct {
  char *buf;
  int len;
  ErlDrvPort *port;
} port_context;

typedef void (*port_fcn)(const port_context, int*, ei_x_buff*);

typedef struct {
  const char *name;
  int arity;
  port_fcn fcn;
} port_function;

typedef struct {
  long socket;
  struct can_frame frame;
} async_context;

typedef struct {
  ErlDrvPort port;
} socketcan_data;


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

static void set_error(ei_x_buff *msg, int result)
{
  ei_x_encode_atom(msg, "error");
  ei_x_encode_long(msg, result);
}

static void open_fn(const port_context in, int *index, ei_x_buff *out)
{
  char *portname;
  int type, size;

  // get string size
  ei_get_type(in.buf, index, &type, &size);
  portname = driver_alloc(size + 1); // +1 for null terminator

  if (ei_decode_string(in.buf, index, portname)) {
    set_error(out, 8);
    goto finish;
  }

  int port = open(portname);
  if (port < 0) {
    set_error(out, port);
  } else {
    ei_x_encode_atom(out, "ok");
    ei_x_encode_long(out, port);
  }

finish:
  driver_free(portname);
  driver_output(*in.port, out->buff, out->index);
}

static void send_fn(const port_context in, int *index, ei_x_buff *out)
{
  long socket, can_id, bin_size;
  int type, size;
  char *data;

  if (ei_decode_long(in.buf, index, &socket) ||
      ei_decode_long(in.buf, index, &can_id)) {
    set_error(out, 8);
    goto send;
  }

  ei_get_type(in.buf, index, &type, &size);
  if (size > 8) {
    set_error(out, 8);
    goto send;
  }

  data = driver_alloc(size);

  if (ei_decode_binary(in.buf, index, data, &bin_size)) {
    set_error(out, 8);
    goto finish;
  }

  int res = can_send(socket, can_id, size, data);
  ei_x_encode_atom(out, "ok");
  ei_x_encode_long(out, res);

finish:
  driver_free(data);
send:
  driver_output(*in.port, out->buff, out->index);
}

static void recv_async(void *async_data)
{
  async_context *data = async_data;
  read(data->socket, &data->frame, sizeof(data->frame));
}

static void free_async(void *async_data)
{
  async_context *data = async_data;
  driver_free(data);
}

static void recv_fn(const port_context in, int *index, ei_x_buff *out)
{
  async_context *async_data = driver_alloc(sizeof(async_context));
  if (ei_decode_long(in.buf, index, &async_data->socket)) {
    set_error(out, 8);
    driver_output(*in.port, out->buff, out->index);
    return;
  }

  driver_async(*in.port, NULL, recv_async, async_data, free_async);
}

/* functions that may be called from erlang */
static port_function functions[] = {
  {"open", 1, open_fn},
  {"send", 3, send_fn},
  {"recv", 1, recv_fn},
  {"", 0, NULL} // guard
};

static ErlDrvData socketcan_start(ErlDrvPort port, char *buff)
{
  socketcan_data *d = (socketcan_data*)driver_alloc(sizeof(socketcan_data));
  set_port_control_flags(port, PORT_CONTROL_FLAG_BINARY);
  d->port = port;
  return (ErlDrvData) d;
}

static void socketcan_stop(ErlDrvData handle)
{
  driver_free((char*)handle);
}

static void socketcan_output(ErlDrvData handle, char *buff, int bufflen)
{
  socketcan_data *d = (socketcan_data*)handle;
  port_context context = {buff, bufflen, &d->port};

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
      current_function->fcn(context, &index, &result);
      ei_x_free(&result);
      return;
    }

    current_function++;
  }

stop_processing:
  driver_output(d->port, result.buff, result.index);
  ei_x_free(&result);
}

static void can_recv_ready(ErlDrvData drv_data, ErlDrvThreadData async_data)
{
  async_context *data = (async_context*)async_data;

  socketcan_data *d = (socketcan_data*)drv_data;

  ei_x_buff result;

  ei_x_new_with_version(&result);
  ei_x_encode_tuple_header(&result, 2);
  ei_x_encode_atom(&result, "ok");

  ei_x_encode_tuple_header(&result, 2);
  ei_x_encode_long(&result, data->frame.can_id);
  ei_x_encode_binary(&result, data->frame.data, data->frame.can_dlc);

  driver_output(d->port, result.buff, result.index);
  ei_x_free(&result);
}

ErlDrvEntry socketcan_driver_entry = {
  NULL,                 /* init */
  socketcan_start,
  socketcan_stop,
  socketcan_output,
  NULL,                 /* ready_input */
  NULL,                 /* ready_output */
  "socketcan_drv",      /* driver name */
  NULL,                 /* finish */
  NULL,                 /* handle */
  NULL,                 /* control */
  NULL,                 /* timeout */
  NULL,                 /* outputv */
  can_recv_ready,
  NULL,                 /* flush */
  NULL,                 /* call */
  NULL                  /* event */
};
  
DRIVER_INIT(socketcan_driver)
{
  return &socketcan_driver_entry;
}
