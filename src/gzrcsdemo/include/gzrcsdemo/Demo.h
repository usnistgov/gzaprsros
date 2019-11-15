

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
#include "gzrcsdemo/CrclApi.h"

#include "gzrcsdemo/Shape.h"
#include "gzrcsdemo/gazebo.h"


// Gear Demo 
struct CGearDemo
{
    CGearDemo();
    int init(std::string robotName);
    int issueRobotCommands(int & state);
    int isDone(int & state);
    int isBusy(int & state);
    void start();
    void stop();
    CGzModelReader gzInstances;        // reads positions of gears and other instances from gazebo
    std::shared_ptr<CCrclApi> crclApi;
protected:
    ShapeModel::CShape * _instance;
    ShapeModel::CShapes _shapes;
    std::string _path;
    tf::Pose _baseoffset;

    // Must declare all variables beforehand
    tf::Pose pickpose;
    std::string gearname;
    tf::Pose affpose;
    tf::Pose gripperoffset;
    tf::Quaternion bend;
    tf::Vector3 offset;
    tf::Pose slotpose;
    tf::Pose slotoffset;
    tf::Pose placepose;

 };

#endif
