/*
  aprs_objects_node.cpp

  ROS node that reads model states from a Gazebo simulation, and sends
  location information for parts, part trays, and kit trays to the
  APRS control system. Serves two ports, one for each camera, with
  objects checked against camera field of view to decide which camera
  port to use.

  You can trigger a re-read of the configuration parameters for debug
  and others by changing them using 'rosparam set' and then publishing
  any string on the aprs_objects_cfg topic, e.g., 

  rosparam set /aprs_objects/debug 0xFF
  rostopic pub -1 /aprs_objects/aprs_objects_cfg std_msgs/String a
*/

/*
  To do:

  Create map of Gazebo object names to APRS object names
*/

#include <iostream>
#include <math.h>
#include <map>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include "ulapi.h"
#include "aprs_objects.h"

// global debug flag
static int debug = 0;

// global mutex on shared access to object info
static ulapi_mutex_struct objectMutex;

// transforms from simulation coordinates to Fanuc and Motoman camera coords
static tf::Transform simToFanuc;
static tf::Transform simToMotoman;

// fields of view of each camera, -x, -y, +x, +y

struct viewBox {
  double xmin, ymin, xmax, ymax;
  // check if a pose is in the field of view
  bool isIn(geometry_msgs::Pose p) {
    return p.position.x >= xmin && p.position.x <= xmax &&
      p.position.y >= ymin && p.position.y <= ymax;
  }
  viewBox() {
    xmin = -1, ymin = -1, xmax = 1, ymax = 1;
  }
  viewBox(double _xmin, double _ymin, double _xmax, double _ymax) {
    xmin = _xmin, ymin = _ymin, xmax = _xmax, ymax = _ymax;
  }
  void set(double _xmin, double _ymin, double _xmax, double _ymax) {
    xmin = _xmin, ymin = _ymin, xmax = _xmax, ymax = _ymax;
  }
};

static viewBox VB1;
static viewBox VB2;

/*
  Names of parts in the APRS:

  sku_part_large_gear
  sku_part_medium_gear
  sku_part_small_gear
  sku_large_gear_vessel
  sku_small_gear_vessel
  sku_medium_gear_vessel
  sku_kit_m2l1_vessel
  sku_kit_s2l2_vessel
*/

/*
  Names of parts in Gazebo:

  sku_kit_s2l2_vessel_1
  sku_kit_s2l2_vessel_2
  sku_large_gear_vessel_1
  sku_large_gear_vessel_2
  sku_part_large_gear_1
  sku_part_large_gear_2
  sku_part_large_gear_3
  sku_part_large_gear_4
  sku_part_small_gear_1
  sku_part_small_gear_2
  sku_part_small_gear_3
  sku_part_small_gear_4
  sku_small_gear_vessel
*/

// string comparison function for map with strings as the key
struct objcmp {
  bool operator() (std::string a, std::string b) const
  {
    return a.compare(b) < 0;
  }
};

// the map of names of objects and their Gazebo pose
typedef std::map<std::string, geometry_msgs::Pose, objcmp> objmap;

// the global object database, all objects and their latest pose
static objmap ObjectDB;

// utility function for printing the object database
void print_objects(objmap &objs)
{
  objmap::iterator iter, start, end;

  start = objs.begin(), end = objs.end();
  std::cout << "-" << objs.size() << "-" << std::endl;
  for (iter = start; iter != end; iter++) {
    std::cout << iter->first << ": " << iter->second.position.x << ", " << iter->second.position.y << std::endl;
  }
}

// callback function for requests to re-read our configuration
void cfgCB(const std_msgs::String& str)
{
  int i1;
  double d1;
  double xmin, ymin, xmax, ymax;

  // ignore the string, the occurance of a publish is all we need
  
  if (ros::param::get("debug", i1)) debug = i1;
  if (ros::param::get("cam1_x_min", d1)) xmin = d1;
  if (ros::param::get("cam1_y_min", d1)) ymin = d1;
  if (ros::param::get("cam1_x_max", d1)) xmax = d1;
  if (ros::param::get("cam1_y_max", d1)) ymax = d1;

  ulapi_mutex_take(&objectMutex);
  VB1.set(xmin, ymin, xmax, ymax);
  ulapi_mutex_give(&objectMutex);

  if (ros::param::get("cam2_x_min", d1)) xmin = d1;
  if (ros::param::get("cam2_y_min", d1)) ymin = d1;
  if (ros::param::get("cam2_x_max", d1)) xmax = d1;
  if (ros::param::get("cam2_y_max", d1)) ymax = d1;

  ulapi_mutex_take(&objectMutex);
  VB2.set(xmin, ymin, xmax, ymax);
  if (debug & DEBUG_CFG) {
    std::cout << VB1.xmin << " " << VB1.ymin << " " << VB1.xmax << " " << VB1.ymax << std::endl;
    std::cout << VB2.xmin << " " << VB2.ymin << " " << VB2.xmax << " " << VB2.ymax << std::endl;
  }
  if (debug & DEBUG_OBJ) print_objects(ObjectDB);
  ulapi_mutex_give(&objectMutex);
}

// callback function for the Gazebo model states topic we subscribe to
void modelStatesCB(const gazebo_msgs::ModelStates& model_states)
{
  static double now, last, delta;
  int n_models = model_states.name.size();

  now = ulapi_time();
  delta = now - last;
  last = now;

  for (int imodel = 0; imodel < n_models; imodel++) {
    geometry_msgs::Pose pose = model_states.pose[imodel];
    std::pair<double, double> p = xybidist();
    pose.position.x += p.first;
    pose.position.y += p.second;

    ulapi_mutex_take(&objectMutex);
    ObjectDB[model_states.name[imodel]] = pose;
    ulapi_mutex_give(&objectMutex);

    if (debug & DEBUG_TIME) {
      std::cout << delta << std::endl;
    }
    if (debug & DEBUG_POSE) {
      std::cout << model_states.name[imodel] << ": " << pose.position.x << ", " << pose.position.y << std::endl;
    }
  }
}

// arguments to pass to the socket server thread for each connection
typedef struct {
  ulapi_task_struct *server_task;
  ulapi_integer which_camera;
  ulapi_integer client_id;
} server_args_struct;

// arguments to the top-level socket connection listener thread
typedef struct {
  ulapi_task_struct *connection_task;
  ulapi_integer which_camera;
  ulapi_integer socket_id;
} connection_args_struct;

/*
  Handler thread for a client connection to read APRS object info.
  Used for both cameras, with a 'which_camera' argument to tell the
  thread which camera it is serving.
 */
void server_code(void *args)
{
  ulapi_task_struct *server_task;
  ulapi_integer which_camera;
  ulapi_integer client_id;
  int len;
  int nchars;

  server_task = ((server_args_struct *) args)->server_task;
  which_camera = ((server_args_struct *) args)->which_camera;
  client_id = ((server_args_struct *) args)->client_id;
  free(args);

  while (true) {
    /*
      write the objects that would appear in the field of view of the
      specified camera port, based on their X-Y location
     */
    std::ostringstream outstr;
    objmap::iterator iter, start, end;

    ulapi_mutex_take(&objectMutex);
    start = ObjectDB.begin(), end = ObjectDB.end();
    for (iter = start; iter != end; iter++) {
      if ((CAMERA_1_PORT == which_camera &&
	   VB1.isIn(iter->second)) ||
	  (CAMERA_2_PORT == which_camera &&
	   VB2.isIn(iter->second))) {
	outstr << iter->first << ",0," << iter->second.position.x << "," << iter->second.position.y << ",0.95P,";
      } else {
	// object not in range
	if (debug & DEBUG_OBJ) {
	  std::cout << iter->first << std::endl;
	}
      }
    }
    ulapi_mutex_give(&objectMutex);

    len = outstr.str().length();
    if (len > 0) {
      nchars = ulapi_socket_write(client_id, outstr.str().c_str(), len);
      if (-1 == nchars) {
	break;
      }
      if (debug & DEBUG_OUT) {
	std::cout << "wrote " << nchars << " chars: " << outstr.str() << std::endl;
      }
    }
    ulapi_sleep(1.0);
  }

  ulapi_socket_close(client_id);

  ulapi_task_delete(server_task);

  return;
}

/* 
   Handler for incoming client connections, which spawns a 'server_code'
   thread for each connection
*/
void connection_code(void *args)
{
  ulapi_task_struct *connection_task;
  ulapi_integer which_camera;
  ulapi_integer socket_id;
  ulapi_integer client_id;
  ulapi_task_struct *server_task;
  
  server_args_struct *server_args_ptr;

  connection_task = ((connection_args_struct *) args)->connection_task;
  which_camera = ((connection_args_struct *) args)->which_camera;
  socket_id = ((connection_args_struct *) args)->socket_id;
  free(args);

  while (true) {
    client_id = ulapi_socket_get_connection_id(socket_id);
    if (client_id < 0) {
      break;
    }

    if (debug & DEBUG_CONNECT) {
      std::cout << "got a connection on " <<  client_id << std::endl;
    }

    server_task = ulapi_task_new();
    server_args_ptr = reinterpret_cast<server_args_struct *>(malloc(sizeof(server_args_struct)));
    server_args_ptr->server_task = server_task;
    server_args_ptr->which_camera = which_camera;
    server_args_ptr->client_id = client_id;
    ulapi_task_start(server_task, server_code, server_args_ptr, ulapi_prio_lowest(), 0);
  }
}

int main(int argc, char **argv)
{
  int option;
  int i1;
  double xmin, ymin, xmax, ymax;
  int ros_argc;
  char **ros_argv;
  ulapi_integer socket_port;
  ulapi_integer camera_1_port = CAMERA_1_PORT;
  ulapi_integer camera_2_port = CAMERA_2_PORT;
  ulapi_integer socket_id;
  ulapi_task_struct *connection_task;
  connection_args_struct *connection_args_ptr;

  if (ULAPI_OK != ulapi_init()) {
    return 1;
  } 

  opterr = 0;
  while (1) {
    option = ulapi_getopt(argc, argv, ":1:2:d:");
    if (option == -1)
      break;

    switch (option) {
    case '1':
      if (1 != sscanf(optarg, "%i", &i1)) {
	std::cerr << "bad camera port value: " << optarg << std::endl;
	return 1;
      }
      camera_1_port = i1;
      break;

    case '2':
      if (1 != sscanf(optarg, "%i", &i1)) {
	std::cerr << "bad camera port value: " << optarg << std::endl;
	return 1;
      }
      camera_2_port = i1;
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

  ulapi_mutex_init(&objectMutex, 1);

  ros::init(ros_argc, ros_argv, "aprs_objects_node");
  ros::NodeHandle n;

  /*
    External programs can publish to this topic and cause us to
    re-read the configuration parameters. We create the topic, then
    subscribe to it. We won't publish on it.
  */
  ros::Publisher cfgpub = n.advertise<std_msgs::String>("aprs_objects_cfg", 1);
  ros::Subscriber cfgsub = n.subscribe("aprs_objects_cfg", 1, cfgCB);

  // this it the topic that Gazebo publishes object poses on
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1, modelStatesCB);

  // initialize the transforms

  tf::Vector3 v;
  tf::Quaternion q;

  v.setValue(1, 2, 3);
  q.setRPY(0, 0, 1.5708);
  simToFanuc.setOrigin(v);
  simToFanuc.setRotation(q);

  v.setValue(2, 3, 5);
  q.setRPY(0, 1, -1);
  simToMotoman.setOrigin(v);
  simToMotoman.setRotation(q);

  simToMotoman = simToMotoman.inverseTimes(simToFanuc);

  // load the bounding boxes on the cameras from the launch file params
  ros::param::param<double>("cam1_x_min", xmin, -1);
  ros::param::param<double>("cam1_y_min", ymin, -1);
  ros::param::param<double>("cam1_x_max", xmax, 1);
  ros::param::param<double>("cam1_y_max", ymax, 1);
  VB1.set(xmin, ymin, xmax, ymax);
  ros::param::param<double>("cam2_x_min", xmin, -1);
  ros::param::param<double>("cam2_y_min", ymin, -1);
  ros::param::param<double>("cam2_x_max", xmax, 1);
  ros::param::param<double>("cam2_y_max", ymax, 1);
  VB2.set(xmin, ymin, xmax, ymax);

  if (debug & DEBUG_CFG) {
    std::cout << VB1.xmin << " " << VB1.ymin << " " << VB1.xmax << " " << VB1.ymax << std::endl;
    std::cout << VB2.xmin << " " << VB2.ymin << " " << VB2.xmax << " " << VB2.ymax << std::endl;
  }

  // open the first camera server socket
  socket_port = camera_1_port;
  socket_id = ulapi_socket_get_server_id_on_interface(socket_port, NULL);
  if (socket_id < 0) {
    std::cerr << "can't serve port " << socket_port << std::endl;
    ulapi_exit();
    return 1;
  }

  // start the socket server connection listener task
  connection_task = ulapi_task_new();
  connection_args_ptr = reinterpret_cast<connection_args_struct *>(malloc(sizeof(connection_args_struct)));
  connection_args_ptr->connection_task = connection_task;
  connection_args_ptr->which_camera = CAMERA_1_PORT;
  connection_args_ptr->socket_id = socket_id;
  ulapi_task_start(connection_task, connection_code, connection_args_ptr, ulapi_prio_lowest(), 0);

  // open the second camera server socket
  socket_port = camera_2_port;
  socket_id = ulapi_socket_get_server_id_on_interface(socket_port, NULL);
  if (socket_id < 0) {
    std::cerr << "can't serve port " << socket_port << std::endl;
    ulapi_exit();
    return 1;
  }

  // start the socket server connection listener task
  connection_task = ulapi_task_new();
  connection_args_ptr = reinterpret_cast<connection_args_struct *>(malloc(sizeof(connection_args_struct)));
  connection_args_ptr->connection_task = connection_task;
  connection_args_ptr->which_camera = CAMERA_2_PORT;
  connection_args_ptr->socket_id = socket_id;
  ulapi_task_start(connection_task, connection_code, connection_args_ptr, ulapi_prio_lowest(), 0);

  ros::spin();

  return 0;
}
