#ifndef IKFAST_FANUC_H
#define IKFAST_FANUC_H



#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <string>


#include <aprs_headers/IKinematic.h>
#include <aprs_headers/seriallinkrobot.h>

#include <boost/config.hpp>
#include <boost/dll/alias.hpp>

namespace RCS
{
class IKFAST_FanucKin :  public IKinematic, public  CSerialLinkRobot, public TestingKinematics<IKFAST_FanucKin>
{
public:

    IKFAST_FanucKin();

    int init();

    int FK(std::vector<double> jv, tf::Pose &pose);
    int IK(tf::Pose pose, std::vector<double>&) ;

    const  std::string & getName(void){ return robot_name; }
    int debug(bool flag);
    int debugStream(std::ostream&);
    size_t numJoints() ;
    std::string set(std::string param,  std::string value);
    std::string set(std::string param,  void * value);
    std::string get(std::string param);
    bool isError(){ return errmsg.empty(); }
    int calibrate(const std::vector<double>& joints, const tf::Pose pose)
    {
        // no need to calibrate
        return 0;
    }

    std::string  runtests(std::string filepath)
    {
        return TestingKinematics<IKFAST_FanucKin>::runtests(filepath);
    }

    /**
     * @brief create boost dll factory method. Creates IKFAST_FanucKin instances.
     * @return  wrapped shared pointer to new IKFAST_FanucKin instance.
     */
    static boost::shared_ptr<IKFAST_FanucKin> create()
    {
        return boost::shared_ptr<IKFAST_FanucKin>( new IKFAST_FanucKin());
    }

private:
    bool bDebug;
    bool bHandleExceptions;
    std::ofstream out;
    std::string errmsg;
    std::string _urdf;
    std::string _urdffile;
    std::string _baselink;
    std::string  _tiplink;
    std::string _inifilename;


    int allIK(tf::Pose & pose, std::vector<std::vector<double>> &joints);
    std::vector<double> nearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints);

    /*!
     * \brief Compute distance between seed state and solution joint state.
     * ROS routine. Copied from ROS.
     * First, normalize all solution joint values to (-2pi,+2pi).
     * \param  ik_seed_state contains original joint value
     * \param  solution is candidate joint values.
     * \return distance between joint vectors
     */
    double Harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
        double dist_sqr = 0;
        std::vector<double> ss = ik_seed_state;
        for (size_t i = 0; i < ik_seed_state.size(); ++i) {
#if 1
            while (ss[i] > 2 * M_PI) {
                ss[i] -= 2 * M_PI;
            }
            while (ss[i] < -2 * M_PI) {
                ss[i] += 2 * M_PI;
            }
            while (solution[i] > 2 * M_PI) {
                solution[i] -= 2 * M_PI;
            }
            while (solution[i] < -2 * M_PI) {
                solution[i] += 2 * M_PI;
            }
#endif
            dist_sqr += fabs(ik_seed_state[i] - solution[i]);
        }
        return dist_sqr;
    }
    void GetClosestSolution(const std::vector<std::vector<double>> &solutions,
                            const std::vector<double> &ik_seed_state, std::vector<double> &solution)
    {
        double mindist = DBL_MAX;
        int minindex = -1;
        std::vector<double> sol;

        for (size_t i = 0; i < solutions.size(); ++i) {
            sol = solutions[i];
            double dist = Harmonize(ik_seed_state, sol);
            if (minindex == -1 || dist < mindist) {
                minindex = i;
                mindist = dist;
            }
        }
        if (minindex >= 0) {
            solution = solutions[minindex];
            Harmonize(ik_seed_state, solution);
        }
    }
};


//extern "C" BOOST_SYMBOL_EXPORT  IKFAST_FanucKin ikfast_fanuc_kin;
BOOST_DLL_ALIAS(
            IKFAST_FanucKin::create, create_plugin
        )

}

#endif
