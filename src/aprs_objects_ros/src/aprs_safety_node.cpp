/*
  aprs_safety_node.cpp

  ROS node that reads beam sensors states from a Gazebo simulation, and flags
  an intrusion.
*/

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "ulapi.h"

// global debug flag
static int debug = 0;

bool getRosTopics(ros::master::V_TopicInfo& topics){
  XmlRpc::XmlRpcValue args, result, payload;

  args[0] = ros::this_node::getName();
  if (! ros::master::execute("getTopicTypes", args, result, payload, true)) {
    return false;
  }

  topics.clear();
  for (int i = 0; i < payload.size(); ++i) {
    topics.push_back(ros::master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
  }

  return true;
}

// we'll publish strings like "estop" and "clear" on this topic
static ros::Publisher pub;

// string comparison function for map with strings as the key
struct objcmp {
  bool operator() (std::string a, std::string b) const
  {
    return a.compare(b) < 0;
  }
};

// the map of names of sensors and their estopped state
typedef std::map<std::string, bool, objcmp> estopmap;

// the global object database, all objects and their latest pose
static estopmap estops;

// determine if we're in estop
bool estopped(estopmap &em)
{
  estopmap::iterator iter, start, end;

  start = em.begin(), end = em.end();
  for (iter = start; iter != end; iter++) {
    if (iter->second) return true;
  }
  return false;
}

// callback function for the sensor messages topic we subscribe to
static void sensorMsgsCB(const sensor_msgs::LaserScan& laser_scans)
{
  int size = laser_scans.ranges.size();
  float min = laser_scans.range_min;
  float max = laser_scans.range_max;
  std::string name = laser_scans.header.frame_id;
  bool myestop;

  for (int t = 0; t < size; t++) {
    float val = laser_scans.ranges[t];
    if (val > min && val < max) {
      myestop = true;
      if (debug) {
	std::cout << name << ": " << val << std::endl;
      }
    } else {
      myestop = false;
      if (debug) {
	std::cout << laser_scans.header.seq << ": inf" << std::endl;
      }
    }
  }

  estops[name] = myestop;

  std::stringstream ss;
  if (myestop) {
    ss << "estop";
  } else {
    if (estopped(estops)) {
      ss << "estop";
    } else {
      ss << "clear";
    }
  }

  std_msgs::String msg;
  msg.data = ss.str();
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  int option;
  std::string prefix;
  int i1;
  int ros_argc;
  char **ros_argv;

  if (ULAPI_OK != ulapi_init()) {
    return 1;
  } 

  opterr = 0;
  while (1) {
    option = ulapi_getopt(argc, argv, ":p:d:");
    if (option == -1)
      break;

    switch (option) {
    case 'p':
      prefix = optarg;
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
      std::cerr << "unrecognized option -" << ulapi_optopt << std::endl;
      return 1;
      break;
    }
  }
  // everything else goes as args to ROS
  ros_argc = argc - optind;
  ros_argv = &argv[optind];

  if (debug) {

  }

  ros::init(ros_argc, ros_argv, "aprs_safety_node");
  ros::NodeHandle n;
  ros::master::V_TopicInfo topics;
  std::vector<ros::Subscriber> subs;
  pub = n.advertise<std_msgs::String>("safety", 1);

  if (getRosTopics(topics)) {
    int size = topics.size();
    for (int t = 0; t < size; t++) {
      if (0 == topics[t].name.compare(0, prefix.size(), prefix)) {
	subs.push_back(n.subscribe(topics[t].name, 1, sensorMsgsCB));
	if (debug) {
	  std::cout << "subscribed to " << topics[t].name << std::endl;
	}
      }
    }
  }

  ros::spin();

  return 0;
}
