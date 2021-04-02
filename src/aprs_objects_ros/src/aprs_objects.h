#ifndef APRS_OBJECTS_H
#define APRS_OBJECTS_H

#include <utility>

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

#endif	/* APRS_OBJECTS_H */
