#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include "ulapi.h"

static bool debug = false;

int main(int argc, char *argv[])
{
  int option;
  int i1;
  int ros_argc;
  char **ros_argv;

  opterr = 0;		      // supress getopt's built-in error print
  while (true) {
    option = ulapi_getopt(argc, argv, "d:");
    if (option == -1)
      break;

    switch (option) {
    case 'd':
      if (1 != sscanf(optarg, "%i", &i1)) {
	fprintf(stderr, "bad debug value: %s\n", optarg);
	return 1;
      }
      debug = i1 ? true : false;
      break;

    case ':':
      fprintf(stderr, "missing value for -%c\n", ulapi_optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf(stderr, "unrecognized option -%c\n", ulapi_optopt);
      return 1;
      break;
    }
  }
  // everything else goes as args to ROS
  ros_argc = argc - optind;
  ros_argv = &argv[optind];

  ros::init(ros_argc, ros_argv, "aprs_demo_node");

  if (ULAPI_OK != ulapi_init()) {
    std::cout << "can't initialize ulapi\n";
    return 1;
  }

  ros::spin();

  return 0;
}
