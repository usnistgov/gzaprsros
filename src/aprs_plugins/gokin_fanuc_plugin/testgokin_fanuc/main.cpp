#include "gokin_fanuc_plugin/gokin_fanuc_plugin.h"
#include <boost/format.hpp>
#include <tf/tf.h>

inline std::ostream & operator << (std::ostream & os,  std::vector<double> jts)
{
    for (std::vector<double>::const_iterator i = jts.begin(); i != jts.end(); ++i)
        os << *i << ',';
    os<< "\n";
    return os;
}

inline std::ostream & operator << (std::ostream & os,  tf::Pose & pose)
{
    os << boost::format("%11.4f") % ( 1000.0 * pose.getOrigin().getX() ) << ":" << boost::format("%11.4f") % ( 1000.0 *  pose.getOrigin().getY()  ) << ":"
       << boost::format("%11.4f") % ( 1000.0 *  pose.getOrigin().getZ()  ) << std::endl;
    return os;
}

int main(int argc, char *argv[])
{
    RCS::gokin_fanuc   fanuc;

    // in angles
    std::vector<double> goaljts = { -0.0005, 29.930, -29.947, 0.12, -59.935, 0.062 };

    // transform angles from degree to radians
    std::transform(goaljts.begin( ), goaljts.end( ), goaljts.begin( ),
                   std::bind1st(std::multiplies<double>( ), M_PI / 180.0) );

    std::cerr << "Current Joints: " << std::endl;
    std::cerr << goaljts  << std::endl;

    tf::Pose pose ;
    fanuc.FK(goaljts, pose);
    std::cerr << "Pose = \n";
    std::cerr << pose;
    std::vector<double> joints ;
    fanuc.IK(pose,joints);

    std::cerr << "Computed Joints: " << std::endl;
    std::cerr << joints  << std::endl;

    return 0;
}
