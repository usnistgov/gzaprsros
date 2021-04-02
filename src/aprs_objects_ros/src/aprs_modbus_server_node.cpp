#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <modbus/modbus.h>
#include "ulapi.h"

static int debug = 0;

#define CONVEYOR_TOPIC "conveyor"

#define MODBUS_PORT 502		// requires admin privileges

#define MODBUS_OUTPUT_BITS 500
#define MODBUS_INPUT_BITS 500
#define MODBUS_OUTPUT_REGISTERS 500
#define MODBUS_INPUT_REGISTERS 500

#define MODBUS_COMMAND_INDEX 7
#define MODBUS_ADDRESS_INDEX 8
#define MODBUS_VALUE_INDEX 10

typedef struct {
  ulapi_task_struct *modbus_server_task_ptr;
  int modbus_port;
  ros::Publisher *pub_ptr;
} modbus_server_args_struct;

static void modbus_server_code(void *args)
{
  ulapi_task_struct *modbus_server_task_ptr;
  int modbus_port;
  ros::Publisher *pub_ptr = NULL;
  modbus_t *ctx = NULL;
  modbus_mapping_t *mb_mapping = NULL;
  int socket = -1;
  bool conveyor_on = false; // false = off, true = on
  bool conveyor_reverse = false; // false = forward, true = reverse
  double conveyor_speed = 0; // positive only, 0 = stopped, 32768 = max
  std_msgs::Float64 speed;

  /*
    We will use the C99 _Exit(int) call to prevent core dumps from unknown atexit() function registrations by libmodbus
  */
#define MYEXIT(x) _Exit(x)

  modbus_server_task_ptr = ((modbus_server_args_struct *) args)->modbus_server_task_ptr;
  modbus_port = ((modbus_server_args_struct *) args)->modbus_port;
  pub_ptr = ((modbus_server_args_struct *) args)->pub_ptr;
  
  mb_mapping = modbus_mapping_new(MODBUS_OUTPUT_BITS, MODBUS_INPUT_BITS, MODBUS_OUTPUT_REGISTERS, MODBUS_INPUT_REGISTERS);
  if (NULL == mb_mapping) {
    std::cerr << "Failed to allocate the mapping: " << strerror(errno) << std::endl;
    MYEXIT(errno);
  }

  for (;;) {
    if (-1 != socket) close(socket);
    if (NULL != ctx) {
      modbus_close(ctx);
      modbus_free(ctx);
    }

    ctx = modbus_new_tcp(
#if 0
      "127.0.0.1",
#else
      NULL,
#endif
      modbus_port);

    if (NULL == ctx) {
      std::cerr << "Failed to create a server socket: " << strerror(errno) << std::endl;
      MYEXIT(errno);
    }

    if (debug) std::cout << "Created server on port " << modbus_port << std::endl;

    modbus_set_debug(ctx, debug ? TRUE : FALSE);

    socket = modbus_tcp_listen(ctx, 2);
    if (-1 == socket) {
      std::cerr << "Failed to listen on server socket: " << strerror(errno) << std::endl;
      MYEXIT(errno);
    }

    if (debug) std::cout << "Waiting for connection on " << socket << "..." << std::endl;

    modbus_tcp_accept(ctx, &socket);

    if (debug) std::cout << "Got connection" << std::endl;

    for (;;) {
      uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
      int rc;

      rc = modbus_receive(ctx, query);
      if (rc > 0) {

	/*
	  A "write single register" command looks like this:
	  0 <seq #> 0 0 0 6 255 6 <hi addr> <lo addr> <hi byte> <lo byte>

	  e.g.,
	  0 23 0 0 0 6 255 6 0 39 1 52
	*/
	if (MODBUS_FC_WRITE_SINGLE_REGISTER == query[MODBUS_COMMAND_INDEX]) {
	  int address;
	  int value;

	  address = MODBUS_GET_INT16_FROM_INT8(query, MODBUS_ADDRESS_INDEX);
	  value = MODBUS_GET_INT16_FROM_INT8(query, MODBUS_VALUE_INDEX);
	  if (debug) std::cout << "write single register:" << " addr " << address << " value " << value << std::endl;
	  /*
	    The APRS PLC has register offsets of 0x8000, which is out of range
	    for this Modbus server, so we need to subtract off the offset and
	    put it back into the command
	  */
	  if (address >= 0x8000) {
	    address -= 0x8000;
	    MODBUS_SET_INT16_TO_INT8(query, MODBUS_ADDRESS_INDEX, address);
	  }
	  /*
	    Handle the register write for the conveyor

	    0x8000 (0) is on/off, 0 = off, 1 = on
	    0x8001 (1) is the direction, 0 = forward, 1 = reverse
	    0x8002 (2) is the speed, 0 = stopped, 32768 = max
	  */
	  if (0 == address) {
	    conveyor_on = (value ? true : false);
	    if (debug) std::cout << "conveyor " << (conveyor_on ? "on" : "off") << std::endl;
	  } else if (1 == address) {
	    conveyor_reverse = (value ? true : false);
	    if (debug) std::cout << "conveyor " << (conveyor_reverse ? "reverse" : "forward") << std::endl;
	  } else if (2 == address) {
	    conveyor_speed = value;
	    if (debug) std::cout << "conveyor speed " << conveyor_speed << std::endl;
	  } else {
	    // not handled
	  }

	  if (conveyor_on) {
	    speed.data = (conveyor_reverse ? -conveyor_speed : conveyor_speed);
	  } else {
	    speed.data = 0;
	  }
	  if (debug) std::cout << "set speed to " << speed.data << std::endl;
	  pub_ptr->publish(speed);
	  if (debug) std::cout << "published " << speed.data << std::endl;
	} else {
	  /*
	    Ignore other Modbus commands; we only write outputs
	  */
	}
	modbus_reply(ctx, query, rc, mb_mapping);
      } else {
	/* Connection closed by the client or error */
	break;
      }
    } // for (;;) on modbus_receive

    if (debug) std::cout << "modbus connection closed" << std::endl;
  } // for (;;) on connections

  MYEXIT(0);
}

int main(int argc, char *argv[])
{
  int option;
  int i1;
  int ros_argc;
  char **ros_argv;
  ulapi_integer modbus_port = MODBUS_PORT;
  std::string topic(CONVEYOR_TOPIC);
  ulapi_task_struct modbus_server_task;
  modbus_server_args_struct modbus_server_args;
  modbus_t *ctx;
  modbus_mapping_t *mb_mapping;

  if (ULAPI_OK != ulapi_init()) {
    return 1;
  } 

  opterr = 0;
  while (1) {
    option = ulapi_getopt(argc, argv, ":p:t:d:");
    if (option == -1)
      break;

    switch (option) {
    case 'p':
      if (1 != sscanf(optarg, "%i", &i1)) {
	std::cerr << "bad port value: " << optarg << std::endl;
	return 1;
      }
      modbus_port = i1;
      break;

    case 't':
      topic = optarg;
      break;

    case 'd':
      if (1 != sscanf(optarg, "%i", &i1)) {
	std::cerr << "bad debug value: " << optarg << std::endl;
	return 1;
      }
      debug = i1;
      break;

    case ':':
      std::cerr << "missing value for -" << ulapi_optopt << std::endl;
      return 1;
      break;

    default:			/* '?' */
      std::cerr << "unrecognized option -" <<  ulapi_optopt << std::endl;
      return 1;
      break;
    }
  }
  // everything else goes as args to ROS
  ros_argc = argc - optind;
  ros_argv = &argv[optind];

  ros::init(ros_argc, ros_argv, "aprs_modbus_server_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64>(topic, 1);

  // run the Modbus server task
  ulapi_task_init(&modbus_server_task);
  modbus_server_args.modbus_server_task_ptr = &modbus_server_task;
  modbus_server_args.modbus_port = modbus_port;
  modbus_server_args.pub_ptr = &pub;
  ulapi_task_start(&modbus_server_task, modbus_server_code, &modbus_server_args, ulapi_prio_lowest(), 0);

  ros::spin();

  return 0;
}
