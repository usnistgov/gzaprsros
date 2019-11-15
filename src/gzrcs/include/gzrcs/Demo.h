

#ifndef DEMO_H
#define DEMO_H


/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"
#include "aprs_headers/Conversions.h"
#include "gzrcs/CrclApi.h"

#include "gzrcs/Kinematics.h"
#include "gzrcs/Controller.h"
#include "gzrcs/Shape.h"
#include "gzrcs/gazebo.h"


// Gear Demo 
struct CGearDemo
{
    CGearDemo();
    int init(std::string robotName);
    int issueRobotCommands(int & state, CCrclApi & r);
    int isDone(int & state, CCrclApi & r);
    void start();
    void stop();
    CGzModelReader gzInstances;        // reads positions of gears and other instances from gazebo
protected:
    ShapeModel::CShape * _instance;
    ShapeModel::CShapes _shapes;
    std::string _path;
    tf::Pose _baseoffset;
 };

#endif
