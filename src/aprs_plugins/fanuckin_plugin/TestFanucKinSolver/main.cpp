
#include <../include/fanuckin_plugin/fanuckin_plugin.h>
#include <aprs_headers/Debug.h>

#include <../include/fanuckin_plugin/fanuc_lrmate200id.h>


//#define __USE_GNU
//#include <math.h>

//#ifndef Deg2Rad
//#define Deg2Rad(Ang)    ( (double) ( Ang * M_PI / 180.0 ) )
//#define Rad2Deg(Ang)    ( (double) ( Ang * 180.0 / M_PI ) )
//#define MM2Meter(d)     ( (double) ( d / 1000.00 ) )
//#define Meter2MM(d)     ( (double) ( d * 1000.00 ) )
//#endif

int main(int argc, char *argv[])
{
    std::vector<double> thetas={0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    std::string path = getexepath().substr(0,getexepath().find_last_of('/') + 1);

    fanuc_lrmate200id gokin;
    tf::Pose p;

//    gokin.fanuc_lrmate200id_kin_fwd (&thetas[0], p);
 //   std::cout << "threekin FK answer"  << RCS::dumpPoseSimple(p) << "\n";



    RCS::Cfanuckin_plugin plugin;
    plugin.set("URDFFILE", path+"FanucLRMate200iD.urdf");
    plugin.set("baselink","fanuc_base_link");
    plugin.set("tiplink","fanuc_link_6");
    plugin.init(); // really ignoring urdf except for names


    plugin.FK(thetas, p);
    std::cout << "threekin FK answer"  << RCS::dumpPoseSimple(p) << "\n";

    std::vector<double> newjoints;
    plugin.IK(p, newjoints);
    std::cout << "threekin IK answer"  << RCS::dumpstdVector(newjoints)<<"\n";


    std::vector<double> calibrationjts= {0.001,0.001,0.001,0.001,0.001,0.001};
    std::vector<double> ds = { 0.47,   0.00,   0.70,0.706753,0.00035373,0.70746,-0.000354084 };
    tf::Pose calibrationpose ( tf::Quaternion(ds[3], ds[4], ds[5], ds[6]), tf::Vector3(ds[0], ds[1], ds[2]));

    plugin.calibrate(calibrationjts, calibrationpose);


    tf::Pose goalpose;
    std::vector<double> joints= thetas;

    // 0.47,   0.00,   0.70,0.706753,0.00035373,0.70746,-0.000354084
    //std::cout << "answer 384.987, 0.040, -0.007, -179.992, 0.001, 0.006\n";
    std::transform(joints.begin(), joints.end(), joints.begin(),
                    [](double d) -> double { return d * M_PI / 180.0; });



    plugin.FK(joints, goalpose);
    std::cout << "FK answer"  << RCS::dumpPoseSimple(goalpose) << "\n";

    plugin.IK(goalpose, newjoints);
    std::cout << "FK answer"  << RCS::dumpstdVector(newjoints)<<"\n";

    return 0;
}
