#ifndef IKINEMATIC_H
#define IKINEMATIC_H

#include <tf/tf.h>
#include <vector>
#include <aprs_headers/seriallinkrobot.h>
namespace RCS
{


/**
 * @brief The IKinematic class
 */
class IKinematic : public ISerialLinkRobot
{
public:
    // Errors so far....
    static const int Parsing_Error = -2;
    static const int Bad_Parameter = -3;
    static const int Initialization_Failed = -4;
    static const int Ini_File_Error = -5;
    static const int Robot_IK_Singularity = -6;
    static const int Not_configured = -7;
    static const int Bad_URDF_String = -8;
    static const int Bad_Conversion = -9;
    static const int Robot_IK_Problem = -10;

    virtual int init(std::string urdf, std::string baselink, std::string tiplink)=0;
    /**
     * @brief get_name This function gets a descriptive and hopefully unique name so
     *    that the controller can adjust the meaning of the parameters passed
     *    to go_kin_set_parameters()
     * @return string containing robot name
     */
    virtual  const  std::string & getName(void)=0;

    /**
     * @brief num_joints returns the actual number of joints, possibly less than the max
     *    supported depending on how many links were present in the call
     *    to go_kin_set_parameters
     * @return positive integer number
     */
    virtual  size_t numJoints()=0;

    virtual  int FK(const std::vector<double> jv, tf::Pose &pose)=0;

    /**
     * @brief inv inverse kinematics take world coordinates and determine joint values,
     *   given the inverse kinematics flags to resolve any ambiguities. The forward
     *   flags are set to indicate their value appropriate to the world coordinates
     *   passed in.
     * @param world world pose
     * @param motors joint values
     * @return
     */
    virtual int IK(const tf::Pose pose, std::vector<double>&)=0;

    virtual int debug(bool flag)=0;
    virtual int debugStream(std::ostream&)=0;

    virtual std::string set(std::string param,  std::string value)=0;

    virtual std::string get(std::string param)=0;
    virtual bool isError()=0;

    // Need to set configuration flags for solution: e.g., elbow up/down
#if 0
    int SetWristOffset(double x);

    int set_parameters(go_link *params, int num){ return 0; }

    int get_parameters(go_link *params, int num){ return 0; }
#endif


};
}
#endif // IKINEMATIC_H
