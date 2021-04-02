#include <math.h>
#include <iostream>
#include <string>

struct xy {
  double x, y;
  xy(double _x, double _y) {x = _x, y = _y;};
  xy() {x = 0, y = 0;};
};

#define DIST(x,y) sqrt((x)*(x)+(y)*(y))

int main(int argc, char *argv[])
{
  enum {NUM_POINTS = 4};
  xy points[NUM_POINTS];
  xy deltas[NUM_POINTS];
  double dists[NUM_POINTS];
  double weights[NUM_POINTS];
  xy target;
  xy moved;
  double denom;

  /*
    Registration points from the Motoman:
  */
  points[0] = xy(0.5359, 0.1557);
  points[1] = xy(0.7231, -0.4003);
  points[2] = xy(0.5238, -0.10686);
  points[3] = xy(0.7473, 0.1554);

  deltas[0] = xy(0, 0.005);
  deltas[1] = xy(-0.009, -0.007);
  deltas[2] = xy(-0.004, 0);
  deltas[3] = xy(0.005, 0);

#ifdef USE_STDIN
  for (std::string line; std::getline(std::cin, line);) {
    double d1, d2;
    if (2 != sscanf(line.c_str(), "%lf %lf", &d1, &d2)) continue;
    target.x = d1, target.y = d2;
#else
    for (target.x = 0.5; target.x < 0.75; target.x += 0.001) {
      for (target.y = -0.45; target.y < 0.16; target.y += 0.001) {
#endif

    for (int i = 0; i < NUM_POINTS; i++) {
      dists[i] = DIST(points[i].x - target.x, points[i].y - target.y);
    }

    // create the products of n-1 weights
    denom = 0;
    for (int i = 0; i < NUM_POINTS; i++) {
      weights[i] = 1;
      for (int j = 0; j < NUM_POINTS; j++) {
	if (i != j) {
	  weights[i] *= dists[j];
	}
      }
      denom += weights[i];
    }

    moved = target;
    for (int i = 0; i < NUM_POINTS; i++) {
      moved.x += weights[i] * deltas[i].x / denom;
      moved.y += weights[i] * deltas[i].y / denom;
    }

#define SCALE 1000.0

    std::cout << SCALE*target.x << ", " << SCALE*target.y << ", " << SCALE*DIST(target.x - moved.x, target.y - moved.y) << std::endl;

#ifdef USE_STDIN
  } // for (line)
#else
  }}
#endif

  return 0;
}
