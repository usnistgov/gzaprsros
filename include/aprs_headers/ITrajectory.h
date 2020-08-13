#ifndef ITRAJECOTY_H
#define ITRAJECOTY_H

#include <tf/tf.h>
#include <vector>
#include <aprs_headers/seriallinkrobot.h>


namespace RCS
{


/**
 * @brief The ITrajectory class abstract interface to define a kinematic plugin.
 */
class ITrajectory
{
public:
    // Errors
    static const int Trajectory_Ok=0;
    static const int Parsing_Error = -2;
    static const int Bad_Parameter = -3;
    static const int Initialization_Failed = -4;
    static const int Ini_File_Error = -5;
    static const int Not_configured = -6;
    static const int Bad_Conversion = -7;
    static const int Robot_Trajectory_Problem = -8;

    // Control
    static const int Position=1;
    static const int MixedMode = 2;


    virtual int init()=0;

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


};
}
#endif // IKINEMATIC_H
