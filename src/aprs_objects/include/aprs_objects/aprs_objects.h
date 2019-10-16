#ifndef APRS_OBJECTS_H
#define APRS_OBJECTS_H

#include <utility>
#include <ulapi.h>
#include <tf/tf.h>
#include <std_msgs/String.h>


// TCP ports for camera servers
enum {
  CAMERA_1_PORT = 5001,
  CAMERA_2_PORT = 5002,
};

// mean and standard deviation for pose X and Y values
#define NOISE_MEAN 0.0
#define NOISE_STD 0.003

extern std::pair<double, double> xybidist(void);

enum {
  DEBUG_TIME = 0x01,
  DEBUG_POSE = 0x02,
  DEBUG_CFG  = 0x04,
  DEBUG_OBJ  = 0x08,
  DEBUG_CONNECT = 0x10,
  DEBUG_OUT = 0x20,
};

// Global helpers
inline char * getexepath()
{
  static char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return result;
}
inline std::string getexefolder()
{
    std::string exepath = getexepath();
    return  exepath.substr(0, exepath.find_last_of('/') + 1);
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

// global debug flag
extern int debug;

// global mutex on shared access to object info
extern ulapi_mutex_struct objectMutex;

extern tf::Transform simToFanuc;
extern tf::Transform simToMotoman;


//extern void cfgCB(const std_msgs::String& str);
extern void modelStatesCB(std::string name, tf::Pose tfp);
void server_code(void *args);
void connection_code(void *args);

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

extern viewBox VB1;
extern viewBox VB2;
#endif	/* APRS_OBJECTS_H */
