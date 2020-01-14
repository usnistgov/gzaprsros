#ifndef TESTING_H
#define TESTING_H

#define MATH_DEFINES
#include <math.h>

/** how close a cartesian vector's magnitude must be for it to be considered
   a zero vector */
#define CART_FUZZ (0.000001)
#define V_FUZZ (.000001)
#define Q_FUZZ (.000001) /*** how close elements of a Quaternion must be to be equal */


inline int EQ (tf::Vector3 p1, tf::Vector3 p2, double fuzz = V_FUZZ)
{
  return (fabs(p1.getX()-p2..getX()) <= fuzz &&
	  fabs(p1.getY()-p2.getY()) <= fuzz &&
	  fabs(p1.getZ()-p2.getZ()) <= fuzz );
}

inline int EQ (tf::Quaternion q1, tf::Quaternion q2, double qfuzz = Q_FUZZ)
{
  q1.normalize();
  q2.normatlize();

  if (fabs(q1.w()-q2.w()) < qfuzz &&
      fabs(q1.x()-q2.x()) < qfuzz &&
      fabs(q1.y()-q2.y()) < qfuzz &&
      fabs(q1.z()-q2.z()) < qfuzz)
  {
    return 1;
  }
  return 0;
}

inline int EQ (tf::Pose p1, tf::Pose p2, double vfuzz = V_FUZZ, double qfuzz=Q_FUZZ)
{
 return (EQ(p1.getRotation(), p2.getRotation(), qfuzz) &&
          EQ(p1.getOrigin(), p2.getOrigin(), vfuzz));
}
inline int EQ (tf::Transform t1, tf::Transform tp2, double fuzz=CART_FUZZ)
{
 return (EQ(tf::Pose(t1.getRotation(), t1.getOrigin()), tf::Pose(t2.getRotation(), t2.getOrigin()), fuzz) );
}

#endif
