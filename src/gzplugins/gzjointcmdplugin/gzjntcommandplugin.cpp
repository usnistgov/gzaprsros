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

#include "gzjntcommandplugin.h"


#define DEFAULT_UPDATE_RATE 10


namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
gzJntCmdPlugin::gzJntCmdPlugin()
{
    update_period_=0.005;
    b_debug=false;
    _nstatus=100;

}
////////////////////////////////////////////////////////////////////////////////
gzJntCmdPlugin::~gzJntCmdPlugin()
{
    // hope this kills the thread xx (crossing fingers)
    bFlag=false;
}
////////////////////////////////////////////////////////////////////////////////
void gzJntCmdPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::cout << "gzJntCmdPlugin: Compiled " << __DATE__ << " " << __TIME__ << "\n" << std::flush;

    // Store the model pointer for convenience.
    this->model = _model;
    this->world = model->GetWorld();

    model_name= model->GetName();

    robotCmd_topicName= "~/" + model_name +"/jnt_cmd";
    if (_sdf->HasElement("cmdtopic"))
        robotCmd_topicName= _sdf->Get<std::string>("cmdtopic");


    robotStatus_topicName= "~/" + model_name +"/jnt_status";
    if (_sdf->HasElement("statustopic"))
        robotStatus_topicName= _sdf->Get<std::string>("statustopic");


    b_debug=false;
    if (_sdf->HasElement("debug"))
    {
        b_debug= _sdf->Get<bool>("debug");
    }


  //  if (_sdf->HasElement("update_rate"))
   // {
  //      int _rate = _sdf->Get<int>("update_rate");
  //      double _updateRate = _rate;
  //      update_rate_ = 1.0/_updateRate;
  //  }


    while( _sdf->HasElement("finger") )
    {
        auto my_finger_sdf = _sdf->GetElement("finger");
        //auto finger_name = my_finger_sdf->GetAttribute("name");
        fingers.push_back(model_name+"::"+_sdf->Get<std::string>("finger"));
        if(b_debug)
            std::cout << " Finger added:" << fingers.back() <<  "\n";
        _sdf->RemoveChild(my_finger_sdf);
    }


    std::cout << "JointPlugin: Topic name Jnts Comm is "<< robotCmd_topicName<<std::endl;
    std::cout << "JointPlugin: Topic name Jnts status is "<< robotStatus_topicName<<std::endl;

    // Safety check
    if (_model->GetJointCount() == 0)
    {
        gzerr << "Invalid joint count, JointPlugin not loaded\n";
        return;
    }

    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    // this->joint = _model->GetJoints()[0];
    size_t n = _model->GetJoints().size();
    size_t num=0;
    for(size_t i=0; i< n; i++)
    {
        std::string joint_name = model_name+"::"+_model->GetJoints()[i]->GetName();
        if(b_debug)
            std::cout << "joint_name=|" << joint_name << "|\n";
        std::vector<std::string>::iterator it;
        if((it=std::find(fingers.begin(), fingers.end(), joint_name))!=fingers.end())
        {
            std::cout << " Found joint matching finger " << joint_name << " removed\n";
            num++;
            continue;
        }

        joints_ptr.push_back(_model->GetJoints()[i]);
        joint_names.push_back( joint_name);
        joint_pos.push_back(0.0);
        joint_vel.push_back(0.0);
        joint_effort.push_back(0.0);
        joint_typeControl.push_back(POSITION_CONTROL);
    }

    if(num==0)
        gzdbg << " Warning! No joint matching fingers\n";

    // Create our node for communication
    this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
    this->node->Init(this->model->GetWorld()->GetName());
#else
    this->node->Init(this->model->GetWorld()->Name());
#endif


    // Subscribe to the joints communication topic, and register a callback
    if(!robotCmd_topicName.empty())
        sub = node->Subscribe(robotCmd_topicName,
                              &gzJntCmdPlugin::onJointsCommMsg,
                              this);

    // advertise status topic
    if(b_debug)
        std::cout << "Advertise joint status on " << robotStatus_topicName << "\n";

    pub = node->Advertise<message::JointsComm>(robotStatus_topicName);


#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world->SimTime();
#else
    last_update_time_ = this->world->GetSimTime();
#endif

    // Update joint model
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&gzJntCmdPlugin::onUpdate, this, _1));

}


////////////////////////////////////////////////////////////////////////////////
void gzJntCmdPlugin::onJointsCommMsg(const ConstJointsCommPtr &msg)
{
    boost::mutex::scoped_lock lock(m);
    for(size_t i=0; i< msg->name_size(); i++)
    {
        // only update those joint names that are given. assuming full joint names?
        std::vector<std::string>::iterator iter = std::find(joint_names.begin(), joint_names.end(), msg->name(i));
        if( iter != joint_names.end()  )
        {
            if(b_debug)
                std::cout << "Update JntsComm msg for " << msg->name(i) <<":" << msg->position(i) << "\n";
            size_t index = std::distance(joint_names.begin(), iter);
            joint_typeControl[index]=POSITION_CONTROL;
            joint_pos[index]=msg->position(i);
        }
    }

}



////////////////////////////////////////////////////////////////////////////////
void gzJntCmdPlugin::onUpdate ( const common::UpdateInfo & /*_info*/ )
{
    // Apply a small linear velocity to the model.
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = this->world->SimTime();
#else
    common::Time current_time = this->world->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        publishJointStates();
        updateJointStatus();

        last_update_time_+= common::Time ( update_period_ );
    }
}
void gzJntCmdPlugin::updateJointStatus()
{
    boost::mutex::scoped_lock lock(m);
    message::JointsComm jnt_readings;
    for(size_t i=0; i< joint_names.size(); i++)
    {
        jnt_readings.add_name(joint_names[i]);
        double pos,vel,acc;
#if GAZEBO_MAJOR_VERSION >= 8
        pos = this->model->GetJoint(joint_names[i])->Position();
        jnt_readings.add_position(pos);
        vel = this->model->GetJoint(joint_names[i])->GetVelocity(0);
        jnt_readings.add_velocity(vel);
        acc = this->model->GetJoint(joint_names[i])->GetForce(0);
        jnt_readings.add_effort(acc);
#else
        pos = this->model->GetJoint(joint_names[i])->GetAngle(0).Radian();
        jnt_readings.add_position(pos);
        vel = this->model->GetJoint(joint_names[i])->GetVelocity(0);
        jnt_readings.add_velocity(vel);
        acc = this->model->GetJoint(joint_names[i])->GetForce(0);
        jnt_readings.add_effort(acc);
#endif
    }
#ifdef STATUS_ECHO
    if(b_debug && _nstatus==0)
        std::cerr << jnt_readings.DebugString() << "\n";
    _nstatus--;
    if(_nstatus<0)
        _nstatus=100;
#endif
    pub->Publish(jnt_readings);

}

////////////////////////////////////////////////////////////////////////////////
void gzJntCmdPlugin::publishJointStates()
{
    // you need to continually update joints or gravity could effect position
    {
        boost::mutex::scoped_lock lock(m);
        myjoint_postarget=joint_pos;
        myjoint_veltarget=joint_vel;
        myjoint_efftarget=joint_effort;
    }

    for (size_t i = 0; i < joint_names.size(); i++ )
    {
        if(joint_typeControl[i]==VELOCITY_CONTROL)
        {
            this->model->GetJoint(joint_names[i])->SetParam("fmax", 0, myjoint_efftarget[i]);
            this->model->GetJoint(joint_names[i])->SetParam("vel", 0, myjoint_veltarget[i]);
            std::cerr << "Set Velocity" << myjoint_veltarget[i] << " force=" << myjoint_efftarget[i] << "\n";
            continue;
        }
        else if (joint_typeControl[i]==POSITION_CONTROL)
        {
            // otherwise just use position control
            setJntPosition(joint_names[i], myjoint_postarget[i]);
        }
        else if (joint_typeControl[i]==FORCE_TORQUE_CONTROL)
        {
            // otherwise just use position control
            this->model->GetJoint(joint_names[i])->SetForce(0, myjoint_efftarget[i]);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void gzJntCmdPlugin::setJntPosition(const std::string &_jointName, const double &_target)
{
    this->model->SetJointPosition(_jointName, _target);
}
}
