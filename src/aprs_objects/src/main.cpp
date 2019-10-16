

#include <aprs_objects.h>
#include <ulapi.h>
#include <ros/ros.h>
#include "gazebo.h"

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
    fprintf(stderr, "bad camera port value: %s\n", optarg);
    return 1;
      }
      camera_1_port = i1;
      break;

    case '2':
      if (1 != sscanf(optarg, "%i", &i1)) {
    fprintf(stderr, "bad camera port value: %s\n", optarg);
    return 1;
      }
      camera_2_port = i1;
      break;

    case 'd':
      if (1 != sscanf(optarg, "%i", &i1)) {
    fprintf(stderr, "bad debug value: %s\n", optarg);
    return 1;
      }
      debug = i1;
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

  ulapi_mutex_init(&objectMutex, 1);

  int rosargc=1;
  char* rosargv[1];
  rosargv[0]= (char *) getexepath();


  //ros::init(rosargc, rosargv, "aprs_objects_node");
  ros::M_string remappings;
  remappings["__master"]=  "http://localhost:11311";
  remappings["__name"]= "aprs_objects_node";

  // Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault
  std::string rosname("aprs_objects_node");
  ros::init(remappings,rosname) ;


  ros::NodeHandle n;

  CGzModelReader gzmodelReader;
  gz.init();
  gzmodelReader.AssignUpdateCallback(modelStatesCB);
  gzmodelReader.start();


  /*
    External programs can publish to this topic and cause us to
    re-read the configuration parameters. We create the topic, then
    subscribe to it. We won't publish on it.
  */
 // ros::Publisher cfgpub = n.advertise<std_msgs::String>("aprs_objects_cfg", 1);
 // ros::Subscriber cfgsub = n.subscribe("aprs_objects_cfg", 1, cfgCB);

  // this it the topic that  ROS publishes Gazebo object poses on
  //ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1, modelStatesCB);

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

  // Defaults below are adjusted from roslaunch from mm to m units


  ros::param::param<int>("debug", debug, 0);
  ros::param::param<int>("camera_1_port", camera_1_port, 5001);
  ros::param::param<int>("camera_2_port", camera_2_port, 5002);

  // load the bounding boxes on the cameras from the launch file params
  ros::param::param<double>("cam1_x_min", xmin, -1);
  ros::param::param<double>("cam1_y_min", ymin, 0);
  ros::param::param<double>("cam1_x_max", xmax, 1);
  ros::param::param<double>("cam1_y_max", ymax, 1);
  VB1.set(xmin, ymin, xmax, ymax);
  ros::param::param<double>("cam2_x_min", xmin, -1);
  ros::param::param<double>("cam2_y_min", ymin, -1);
  ros::param::param<double>("cam2_x_max", xmax, 1);
  ros::param::param<double>("cam2_y_max", ymax, 0);
  VB2.set(xmin, ymin, xmax, ymax);

  if (debug & DEBUG_CFG) {
    printf("%f %f %f %f\n", VB1.xmin, VB1.ymin, VB1.xmax, VB1.ymax);
    printf("%f %f %f %f\n", VB2.xmin, VB2.ymin, VB2.xmax, VB2.ymax);
  }

  // open the first camera server socket
  socket_port = camera_1_port;
  socket_id = ulapi_socket_get_server_id_on_interface(socket_port, NULL);
  if (socket_id < 0) {
    fprintf(stderr, "can't serve port %d\n", (int) socket_port);
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
    fprintf(stderr, "can't serve port %d\n", (int) socket_port);
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

  //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
  ros::AsyncSpinner * _spinner = new ros::AsyncSpinner(1);
  _spinner->start();
  ros::waitForShutdown();

  gzmodelReader.stop();
  gz.stop();


  return 0;
}
