#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include "erl_nif.h"

#define ASSERT(e) ((void)((e) ? 1 : (assert_error(#e, __func__, __FILE__, __LINE__), 0)))

static void assert_error(const char *expr, const char *func, const char *file, int line) {
  fflush(stdout);
  fprintf(stderr, "%s:%d:%s() Assertion failed: %s\r\n", file, line, func, expr);
  fflush(stderr);
  abort();
}

/** State of the nif */
typedef struct {
  int open;
  int fd;
} State;

/** Private data of the nif */
typedef struct {
  ERL_NIF_TERM atom_ok;
  ERL_NIF_TERM atom_undefined;
  ERL_NIF_TERM atom_error;
  ERL_NIF_TERM atom_nil;
} priv_data;

static void handler_rt_dtor(ErlNifEnv *env, void *obj) {
  enif_fprintf(stderr, "handler_rt_dtor called\r\n");
}

static void handler_rt_stop(ErlNifEnv *env, void *obj, int fd, int is_direct_call) {
  enif_fprintf(stderr, "handler_rt_stop called %s\r\n", (is_direct_call ? "DIRECT" : "LATER"));
}

static void handler_rt_down(ErlNifEnv* env, void* obj, ErlNifPid* pid, ErlNifMonitor* mon) {
  enif_fprintf(stderr, "handler_rt_down called\r\n");
  State *state = (State*)obj;
  if(state->open) {
    // No private data, so make an existing atom here.
    ERL_NIF_TERM undefined;
    enif_make_existing_atom(env, "undefined", &undefined, ERL_NIF_LATIN1);
    int rv = enif_select(env, state->fd, ERL_NIF_SELECT_STOP, state, NULL, undefined);
    ASSERT(rv >= 0);
  }
}

static ErlNifResourceType *handler_rt;
static ErlNifResourceTypeInit handler_rt_init = {handler_rt_dtor, handler_rt_stop, handler_rt_down};

// Called on compilation, not runtime.
static int load(ErlNifEnv* env, void** priv, ERL_NIF_TERM info) {
  // Allocate private data.
  priv_data* data = enif_alloc(sizeof(priv_data));
  if (data == NULL)
    return 1;

  // Assign data.
  data->atom_ok = enif_make_atom(env, "ok");
  data->atom_undefined = enif_make_atom(env, "undefined");
  data->atom_error = enif_make_atom(env, "error");
  data->atom_nil = enif_make_atom(env, "nil");

  // cast data, and return.
  *priv = (void*) data;
  handler_rt = enif_open_resource_type_x(env, "monitor", &handler_rt_init, ERL_NIF_RT_CREATE, NULL);
  return !handler_rt;
}

// Callbacks we don't need.
static int reload(ErlNifEnv* env, void** priv, ERL_NIF_TERM info) { return 0; }
static int upgrade(ErlNifEnv* env, void** priv, void** old_priv, ERL_NIF_TERM info) { return load(env, priv, info); }
static void unload(ErlNifEnv* env, void* priv) { enif_free(priv); }

// These two functions were stollen from a StackOverflow answer. Mostly Black magic.
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c/38318768

/** Set speed and parity of a serial port. */
int set_interface_attribs(int fd, int speed, int parity) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);

  if (tcgetattr (fd, &tty) != 0) {
    enif_fprintf(stderr, "Error %s from tcgetattr",  errno);
    return errno;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 5;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    enif_fprintf(stderr, "Error %s from tcsetattr", strerror(errno));
    return errno;
  }
  return 0;
}

int set_blocking(int fd, int should_block) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);

  if (tcgetattr (fd, &tty) != 0) {
    enif_fprintf(stderr, "error %s from tggetattr", strerror(errno));
    return errno;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    enif_fprintf(stderr, "error %s setting term attributes", strerror(errno));
    return errno;
  }
  return 0;
}

/** This is where the actual serial port is opened */
static ERL_NIF_TERM start_handler(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[]) {
  if(argc != 1)
    return enif_make_badarg(env);

  State *state;
  priv_data* priv = enif_priv_data(env);

  ERL_NIF_TERM res;
  ErlNifPid self;

  enif_self(env, &self);
  char portname[15];
  enif_get_string(env, argv[0], portname, 15, ERL_NIF_LATIN1);
  enif_fprintf(stderr, "Trying to open: %s\n", portname);
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    char *error_str = strerror(errno);
    enif_fprintf(stderr, "Failed to open: %s %s\n", portname, error_str);
    return enif_make_tuple2(env, priv->atom_error, enif_make_string(env, error_str, sizeof(error_str)));
  }

  int fail;
  fail = set_interface_attribs(fd, B115200, 0);
  if(fail < 0) {
    char *error_str = strerror(fail);
    enif_fprintf(stderr, "Failed to set interface attributes %s\n", error_str);
    return enif_make_tuple2(env, priv->atom_error, enif_make_string(env, error_str, sizeof(error_str)));
  }

  fail = set_blocking(fd, 0);
  if(fail < 0) {
    char *error_str = strerror(fail);
    enif_fprintf(stderr, "Failed to set blocking %s\n", error_str);
    return enif_make_tuple2(env, priv->atom_error, enif_make_string(env, error_str, sizeof(error_str)));
  }

  state = enif_alloc_resource(handler_rt, sizeof(State));
  state->fd = fd;
  state->open = 1;
  enif_monitor_process(env, state, &self, NULL);
  res = enif_make_resource(env, state);
  enif_release_resource(state);
  return enif_make_tuple2(env, priv->atom_ok, res);
}

// Called when we want to close the serial port.
static ERL_NIF_TERM stop_handler(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[]) {
  if(argc != 1)
    return enif_make_badarg(env);

  State *state;
  priv_data* priv = enif_priv_data(env);
  int do_stop;

  if(!enif_get_resource(env, argv[0], handler_rt, (void **)&state))
    return enif_make_badarg(env);

  do_stop = state->open;
  if(do_stop) {
    printf("Closing fd=%d\r\n", state->fd);
    close(state->fd);
    state->open = 0;
    int rv = enif_select(env, state->fd, ERL_NIF_SELECT_STOP, state, NULL, priv->atom_undefined);
    ASSERT(rv >= 0);
  }

  return priv->atom_ok;
}

static ERL_NIF_TERM poll(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[]) {
  State *state;
  int rv;
  priv_data* priv = enif_priv_data(env);
  if(!enif_get_resource(env, argv[0], handler_rt, (void **)&state)) {
    return enif_make_badarg(env);
  }
  rv = enif_select(env, state->fd, ERL_NIF_SELECT_READ, state, NULL, priv->atom_undefined);
  ASSERT(rv >= 0);
  return priv->atom_ok;
}

/** This is where the data is actually read in */
static ERL_NIF_TERM receive_input(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[]) {
  State *state;
  priv_data* priv = enif_priv_data(env);

  if(!enif_get_resource(env, argv[0], handler_rt, (void **)&state)) {
    return enif_make_badarg(env);
  }

  char buf[1];
  char line[100];
  int i = 0;
  int bytes_read;
  // Stollen from: https://github.com/nerves-project/nerves_uart/blob/c357c73c88bc025405faf271e452ddf6a89e63d7/src/uart_comm_unix.c#L613
  // reads one byte at a time until a newline character.
  do {
    bytes_read = read(state->fd, buf, 1);
    line[i] = buf[0];
    i++;
  } while(bytes_read < 0 || buf[0] != '\n');

  int line_size = sizeof(line[0]) * i;

  if(line_size > 0) {
    ErlNifBinary ibin;
    enif_alloc_binary(line_size, &ibin);
    strcpy(ibin.data, line);
    return enif_make_binary(env, &ibin);
    ERL_NIF_TERM ret_str = enif_make_string(env, buf, sizeof(buf));;
    return ret_str;
  } else if(bytes_read == 0 || (bytes_read < 0 && errno == EAGAIN)) {
    return priv->atom_nil;
  } else {
    char *error_str = strerror(errno);
    ERL_NIF_TERM err = enif_make_string(env, error_str, sizeof(error_str));
    return enif_make_tuple2(env, priv->atom_error, err);
  }
}

// Nif stuff.
static ErlNifFunc nif_funcs[] = {
  {"start_handler", 1, start_handler},
  {"stop_handler", 1, stop_handler},
  {"poll", 1, poll},
  {"receive_input", 1, receive_input},
};

ERL_NIF_INIT(Elixir.Farmbot.Firmware.UartHandler, nif_funcs, &load, &reload, &upgrade, &unload)
