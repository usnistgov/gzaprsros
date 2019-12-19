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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <map>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
//#include <gazebo_msgs/ModelStates.h>
//#include <geometry_msgs/Pose.h>
#include "ulapi.h"
#include "aprs_objects.h"

// global debug flag
int debug = 0;

// global mutex on shared access to object info
ulapi_mutex_struct objectMutex;

tf::Transform simToFanuc;
tf::Transform simToMotoman;

viewBox VB1;
viewBox VB2;

// trim digits from end
static inline std::string rnumtrim(std::string s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isdigit))).base(), s.end());
    return s;
}

static  geometry_msgs::Pose  Convert(tf::Pose p)
{
    geometry_msgs::Pose m;
    m.orientation.x=p.getRotation().getX();
    m.orientation.y=p.getRotation().getY();
    m.orientation.z=p.getRotation().getZ();
    m.orientation.w=p.getRotation().getW();
    m.position.x = p.getOrigin().getX();
    m.position.y = p.getOrigin().getY();
    m.position.z = p.getOrigin().getZ();
    return m;
}

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

sku_part_large_gear,0.00,670.18,107.91,0.98,P,
sku_part_large_gear,0.00,432.81,112.46,0.96,P,
sku_part_large_gear,,,,,P,
sku_part_large_gear,,,,,P,
sku_part_medium_gear,0.00,640.46,-190.82,0.95,P,
sku_part_medium_gear,0.00,645.48,-116.34,0.94,P,
sku_part_medium_gear,0.00,565.24,-186.41,0.93,P,
sku_part_medium_gear,0.00,569.95,-111.98,0.91,P,
sku_part_medium_gear,,,,,P,
sku_part_small_gear,,,,,P,
sku_part_small_gear,,,,,P,
sku_part_small_gear,,,,,P,
sku_part_small_gear,,,,,P,
sku_kit_s2l2_vessel,,,,,KT,
sku_kit_s2l2_vessel,,,,,KT,
sku_large_gear_vessel,-3.12,427.79,-147.06,0.99,PT,
sku_small_gear_vessel,,,,,PT,
sku_medium_gear_vessel,-0.05,603.46,-149.80,0.86,PT,
sku_medium_gear_vessel,,,,,PT,
sku_kit_m2l1_vessel,-1.55,394.77,109.58,0.68,KT,
sku_kit_m2l1_vessel,-1.60,627.80,107.75,0.70,KT,
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
        printf("%f %f %f %f\n", VB1.xmin, VB1.ymin, VB1.xmax, VB1.ymax);
        printf("%f %f %f %f\n", VB2.xmin, VB2.ymin, VB2.xmax, VB2.ymax);
    }
    if (debug & DEBUG_OBJ) print_objects(ObjectDB);
    ulapi_mutex_give(&objectMutex);
}

// callback function for the Gazebo model states topic we subscribe to
void modelStatesCB(std::string name, tf::Pose tfp)
{
    static double now, last, delta;

    now = ulapi_time();
    delta = now - last;
    last = now;


    geometry_msgs::Pose pose =  Convert(tfp) ;

//    std::pair<double, double> p = xybidist();
//    pose.position.x += p.first;
//    pose.position.y += p.second;

    ulapi_mutex_take(&objectMutex);
    ObjectDB[name] = pose;
    ulapi_mutex_give(&objectMutex);

    if (debug & DEBUG_TIME) {
        std::cout << delta << std::endl;
    }
    if (debug & DEBUG_POSE) {
        std::cout << name << ": " << pose.position.x << ", " << pose.position.y << std::endl;
    }
}



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
                //
                std::string partType("P");

                // Skip non gear sku related parts
                if(iter->first.find("sku")==std::string::npos)
                    continue;
                if(iter->first.find("vessel")!=std::string::npos)
                    partType="PT";
                if(iter->first.find("kit")!=std::string::npos)
                    partType="KT";

                outstr << rnumtrim(iter->first) << ",0," << iter->second.position.x *1000.0
                       << "," << iter->second.position.y * 1000.0
                       << ",0.95," << partType << ",";
            } else {
                // object not in range
                if (debug & DEBUG_OBJ) {
                    std::cout << iter->first << std::endl;
                }
            }
        }
        outstr << "\r\n";
        ulapi_mutex_give(&objectMutex);

        len = outstr.str().length();
        if (len > 0) {
            nchars = ulapi_socket_write(client_id, outstr.str().c_str(), len);
            if (-1 == nchars) {
                break;
            }
            if (debug & DEBUG_OUT) {
                printf("wrote %d chars: %s\n", nchars, outstr.str().c_str());
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
            printf("got a connection on %d\n", (int) client_id);
        }

        server_task = ulapi_task_new();
        server_args_ptr = reinterpret_cast<server_args_struct *>(malloc(sizeof(server_args_struct)));
        server_args_ptr->server_task = server_task;
        server_args_ptr->which_camera = which_camera;
        server_args_ptr->client_id = client_id;
        ulapi_task_start(server_task, server_code, server_args_ptr, ulapi_prio_lowest(), 0);
    }
}

