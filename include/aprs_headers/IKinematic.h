#ifndef IKINEMATIC_H
#define IKINEMATIC_H

#include <tf/tf.h>
#include <vector>
#include <aprs_headers/seriallinkrobot.h>
namespace RCS
{


/**
 * @brief The IKinematic class abstract interface to define a kinematic plugin.
 */
class IKinematic : public ISerialLinkRobot
{
public:
    // Errors so far....
    static const int Kinematics_Ok=0;
    static const int Parsing_Error = -2;
    static const int Bad_Parameter = -3;
    static const int Initialization_Failed = -4;
    static const int Ini_File_Error = -5;
    static const int Robot_IK_Singularity = -6;
    static const int Not_configured = -7;
    static const int Bad_URDF_String = -8;
    static const int Bad_Conversion = -9;
    static const int Robot_IK_Problem = -10;
    static const int Bad_Joints_Size = -11;
    static const int Not_Implemented = -11;

    virtual int init()=0;
    //virtual int init(std::string urdf, std::string baselink, std::string tiplink)=0;
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

    /**
     * @brief FK calculate the forward kinematics to derive a pose from the given joints.
     * @param jv input joint values of the robot
     * @param pose reference to pose to store FK in
     * @return  0 if successful. Otherwise return error enumeration and error message
     * to be retrieved by get("help");
     */
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

    /**
     * @brief calibrate take an input set of joints, with given FK pose and compute
     * use as ground truth to compute the difference from locate plugin
     * calculations to robot centric coordinates. THis arose after kdl amont other
     * plugins caculated a n orientation as 1,0,0,0 when actually points
     * at 1,0,0, 0 (axis 1,0,0, angle 0).
     * @param jo joint valuoes (possible non-zero as this may be singulatority)ints
     * @param pose  expected FK from joints.
     * @return 0 if successful.
     */
    virtual int calibrate(const std::vector<double>& joints, const tf::Pose pose)=0;

    /**
     * @brief debug set the debug flat
     * @param flag true to output debug informjatoin
     * @return 0 if successful. Otherwise return error enumeration and error message
     * to be retrieved by get("help");
     */
    virtual int debug(bool flag)=0;

    /**
     * @brief debugStream is a pointer to a iostream in which to
     * output debug information.
     * @return 0 if successful. Otherwise return error enumeration and error message
     * to be retrieved by get("help");
     */
    virtual int debugStream(std::ostream&)=0;

    /**
     * @brief set assigns the given param with the given value string. The string
     * will be converted into the internal format.
     * @param param string containing name of parameter.
     * @param value  string  containing new value.
     * @return 0 if successful. Otherwise return error enumeration and error message
     * to be retrieved by get("help");
     */
    virtual std::string set(std::string param,  std::string value)=0;

    /**
     * @brief set assigns the given param with the given void pointer.
     * @param param string containing name of parameter.
     * @param value  pointer to void to assign to parameter.
     * @return 0 if successful. Otherwise return error enumeration and error message
     * to be retrieved by get("help");
     */
    virtual std::string set(std::string param,  void * value)=0;

    /**
     * @brief get returns a string containing the value of the given parameter. If get
     * parameter is "help" it returns a list of all gets/sets allowed by this kinetmic
     * plugin.
     * @param param string containing name of parameter.
     * @return 0 if successful. Otherwise return error enumeration and error message
     * to be retrieved by get("help");
     */
    virtual std::string get(std::string param)=0;

    /**
     * @brief isError did the last method produce an error.
     * @return true if error, otherwise false.
     */
    virtual bool isError()=0;

    // FIXME: Need to set configuration flags for solution: e.g., elbow up/down
    // maybe with set parameter...


    /**
     * @brief runtests accepts a test file path containing fk/ik commands and solutions
     * and runs tests to see if the kinsolver passes.
     * @param filename full path of test file
     * @return string containing failed tests. Empty string if successful.
     */
    virtual std::string runtests(std::string filename)=0;
};
}
#endif // IKINEMATIC_H
