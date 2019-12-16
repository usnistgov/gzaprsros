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

#include "gzparallelgripperplugin.h"
#include <boost/algorithm/string.hpp>

#define DEFAULT_UPDATE_RATE 10

#include <mutex>
namespace gazebo
{
static std::mutex mymutex;
////////////////////////////////////////////////////////////////////////////////
gzParallelGripperPlugin::VirtualJoint::VirtualJoint(gzParallelGripperPlugin* p,
                                                  physics::JointPtr j,
                                                  physics::CollisionPtr c1,
                                                  physics::CollisionPtr c2,
                                                  ignition::math::Vector3d f)
    : parent(p), joint(j), collision1(c1), collision2(c2), contactForce(f)
{
    //gzdbg << "Got joint: " << this->joint->GetScopedName() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
bool gzParallelGripperPlugin::VirtualJoint::CheckValidity(VirtualJoint* v)
{
#if 0
    //Joint Deletion Callback here
    auto savedForce = v->contactForce;
    auto currentForce = -v->joint->GetForceTorque(0).body2Force;
    auto error = (savedForce - currentForce).Length();

    //gzdbg << "Saved Force is: " << savedForce << std::endl;
    //gzdbg << "Current Force is: " << currentForce << std::endl;
    //gzdbg << "Error is: " << error << std::endl;

    if (error > v->parent->detachThreshold)
    {
        //gzdbg << "Error is: " << error << ", Deleting virtual joint: " << v->joint->GetName() << std::endl;

        v->collision1->SetCollideBits(0xffff);
        v->collision2->SetCollideBits(0xffff);
        v->parent->model->RemoveJoint(v->joint->GetName());

        return true;
    }
#endif
    return false;
}

////////////////////////////////////////////////////////////////////////////////
gzParallelGripperPlugin::gzParallelGripperPlugin()
{
    update_period_=0.005;
    bDebug=false;
    grip_enabled = false;
    jointId = 0;
    bGrasping=0;

}
////////////////////////////////////////////////////////////////////////////////
gzParallelGripperPlugin::~gzParallelGripperPlugin()
{

}
////////////////////////////////////////////////////////////////////////////////
void gzParallelGripperPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::cout << "GzParallelGripperPlugin: Compiled " << __DATE__ << " " << __TIME__ << "\n" << std::flush;

    //Get SDF Params

    grip_force_close = 10;
    if (_sdf->HasElement("grip_force_close"))
        grip_force_close = _sdf->Get<double>("grip_force_close");

    grip_force_open = -0.5 * grip_force_close;
    if (_sdf->HasElement("grip_force_open"))
        grip_force_open = _sdf->Get<double>("grip_force_open");

    grip_kp = 1000;
    if (_sdf->HasElement("grip_kp"))
        grip_kp = _sdf->Get<double>("grip_kp");

    std::string joint1_name = "joint1";
    if (_sdf->HasElement("joint1"))
        joint1_name = _sdf->Get<std::string>("joint1");

    std::string joint2_name = "joint2";
    if (_sdf->HasElement("joint2"))
        joint2_name = _sdf->Get<std::string>("joint2");

    std::string control_topic = "~/gripper/control";
    if (_sdf->HasElement("control_topic"))
        control_topic = _sdf->Get<std::string>("control_topic");

    std::string state_topic = control_topic;
    if(control_topic.find("control") != std::string::npos)
        boost::replace_all(state_topic, "control", "state") ;
    else
        state_topic = "~/gripper/state";

    if (_sdf->HasElement("state_topic"))
        state_topic = _sdf->Get<std::string>("state_topic");

    bDebug=0;
    if (_sdf->HasElement("debug"))
        bDebug = _sdf->Get<int>("debug");

    bFullDebug=0;
    if (_sdf->HasElement("fulldebug"))
        bFullDebug = _sdf->Get<int>("fulldebug");


    bCollisions=0;
    if (_sdf->HasElement("collisions"))
    {
        bCollisions= _sdf->Get<bool>("collisions");
    }

    // Store the pointer to the model
    this->model = _parent;
    this->world = model->GetWorld();

    // Save model name for fully scoping joint, link and collision names
    std::string    model_name= model->GetName();

    //get pointers to each gripper joint, link
    joint1 = this->model->GetJoint(joint1_name);
    joint2 = this->model->GetJoint(joint2_name);

    std::string link1_name = joint1->GetJointLink(0)->GetName();
    std::string link2_name = joint2->GetJointLink(0)->GetName();


    link1 = this->model->GetJoint(joint1_name)->GetJointLink(0);
    link2 = this->model->GetJoint(joint2_name)->GetJointLink(0);

    // save gripper finger joint objects
    gzjoints.push_back(joint1);
    gzjoints.push_back(joint2);

    // save gripper finger link objects
    gzlinks.push_back(link1);
    gzlinks.push_back(link2);

    // save joint names
    joint_names.push_back(model_name+"::"+ joint1_name);
    joint_names.push_back(model_name+"::"+ joint2_name);

    std::cout  << "gzParallelGripperPlugin Model = " << model_name<< "\n";
    std::cout  << "gzParallelGripperPlugin Joint1 = " << joint1_name << "\n";
    std::cout  << "gzParallelGripperPlugin Joint2 = " << joint2_name << "\n";
    std::cout  << "gzParallelGripperPlugin Link1 = " << link1_name << "\n";
    std::cout  << "gzParallelGripperPlugin Link2 = " << link2_name << "\n";
    std::cout  << "gzParallelGripperPlugin Control Topic = " << control_topic << "\n";
    std::cout  << "gzParallelGripperPlugin State Topic = " << state_topic << "\n";

    if (!joint1 || !joint2)
    {
        gzerr << "Gripper joints not found, check specified joint names.\n";
    }

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&gzParallelGripperPlugin::onUpdate, this, _1));

    // Create our node for communication
    this->node = transport::NodePtr(new transport::Node());

#if GAZEBO_MAJOR_VERSION < 8
    this->node->Init(this->model->GetWorld()->GetName());
#else
    this->node->Init(this->model->GetWorld()->Name());
#endif

    // Publish to a Gazebo topic
    this->pub = node->Advertise<message::GripCommand>(state_topic);

    // Subscribe to the control topic, and register a callback
    this->sub = this->node->Subscribe(control_topic,
                                      &gzParallelGripperPlugin::onMsg,
                                      this);

    // setup contact manager
    this->contactManager = _parent->GetWorld()->Physics()->GetContactManager();

    // If set to true, SetNeverDropContacts will always add contacts even if there are no subscribers.
    this->contactManager->SetNeverDropContacts(true);

    // determine collision link for each finger
    if(bCollisions)
    {
        //std::vector<int> v = {1, 2, 3, 4};
        for (unsigned int j = 0; j < this->gzlinks.size(); ++j)
        {
            physics::CollisionPtr collision = gzlinks[j]->GetCollision((unsigned int) 0);
            std::string collName = collision->GetScopedName();
            //collision->SetContactsEnabled(true);
            std::map<std::string, physics::CollisionPtr>::iterator collIter = collisionElems.find(collName);
            if (collIter != this->collisionElems.end()) { //this collision was already added before
                gzwarn <<"GazeboGraspGripper: Adding Gazebo collision link element "<<collName<<" multiple times, the gazebo grasp handler may not work properly"<<std::endl;
                continue;
            }
            gzdbg << "Add collision = " << collName << "\n";
            this->collisionElems[collName] = collision;
           // _collisionElems[collName] = collision;
            collisionNames.push_back(collName);
        }


        contacttopic= contactManager->CreateFilter(model->GetScopedName(), collisionNames);
        std::cout<<"Subscribing contact manager to topic " << contacttopic << std::endl;
        bool latching=false;
        this->contactSub = this->node->Subscribe(contacttopic,&gzParallelGripperPlugin::OnContact, this, latching);

    }
    // Fix...
    this->model->SetGravityMode(false);
    linear_velocity = ignition::math::Vector3d(0,0,0);
    angular_velocity = ignition::math::Vector3d(0,0,0);

}
////////////////////////////////////////////////////////////////////////////////
void gzParallelGripperPlugin::OnContact(const ConstContactsPtr& _msg)
{
    std::lock_guard<std::mutex> lockGuard(mymutex);

    try {
        this->contacts.clear();

        for (int i = 0; i < _msg->contact_size(); ++i)
        {
            contacts.push_back(_msg->contact(i));
            if(bDebug && _msg->contact_size()==2)
                gzdbg << "Contact msg received num contacts =" << _msg->contact_size() << std::endl;
        }
    }
    catch(...)
    {

    }
}

////////////////////////////////////////////////////////////////////////////////
void gzParallelGripperPlugin::onMsg(const ConstGripCommandPtr &msg)
{
    if(bDebug)
        gzdbg << "Grip command requested: " << (msg->enable() ? "Close" : "Open") << std::endl;
    grip_enabled = msg->enable();

}

////////////////////////////////////////////////////////////////////////////////
void gzParallelGripperPlugin::onUpdate ( const common::UpdateInfo & /*_info*/ )
{
#ifdef TIMED_UPDATE
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = this->world->SimTime();
#else
    common::Time current_time = this->world->GetSimTime();
#endif

    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update < update_period_ )
        return;
    last_update_time_+= common::Time ( update_period_ );
#endif

    force1=joint1->GetForce(0);
    force2=joint2->GetForce(0);

    if(bFullDebug)
        gzdbg << "Gripper force: joint1=" << force1 << " joint2= "<< force2 << std::endl;

    if (!grip_enabled)
    {
        if(bFullDebug)
            gzdbg << "Gripper start opening " << std::endl;

        joint1->SetForce(0, grip_force_open);
        joint2->SetForce(0, grip_force_open);
        this->model->SetLinearVel(linear_velocity);
        this->model->SetAngularVel(angular_velocity);

        // check if force open threshold execeeded.
        // if so, remove vitual joints and turn on gravity for object
        if(bCollisions  &&  (force1+force2 > (2 * grip_force_open)-(0.01*grip_force_open)))
        {
            // FIXME: set object link's gravity to false -  allow easily dropping
            for (auto v : this->virtualJoints)
            {
                SetLinksGravityFlag(v->collision2->GetModel(), true);
                if(bDebug)
                    gzdbg << "Set " << v->collision2->GetModel() << " restore gravity " << std::endl;

            }
            Detach();
        }
        if(this->virtualJoints.size() == 0)
            bGrasping=0;
    }
    else
    {
        if(bFullDebug)
            gzdbg << "Gripper closing " << std::endl;

        //Additional force modifier to ensure symmetrical finger positions
        double force_modifier = (joint1->Position(0) - joint2->Position(0)) * grip_kp;

        joint1->SetForce(0, grip_force_close - force_modifier);
        joint2->SetForce(0, grip_force_close + force_modifier);
        this->model->SetLinearVel(linear_velocity);
        this->model->SetAngularVel(angular_velocity);

        // check if force threshold execeeded.
        // if so collision with object, if so attach with vitual joint to gripper fingers
        // and turn off gravity for object
        if(bCollisions  && ! CheckAttach() && (force1+force2 > (2 * grip_force_close)-(0.01*grip_force_close)))
        {
            for (auto v : this->virtualJoints)
            {
                // assuming not mid air...
                SetLinksGravityFlag(v->collision2->GetModel(), false);
                if(bDebug)
                    gzdbg << "Set " << v->collision2->GetModel() << " no gravity " << std::endl;
                bGrasping=1;
            }
        }
        if(this->virtualJoints.size() > 0)
            bGrasping=1;
        if(bDebug && (force1+force2 < (2 * grip_force_close)-(0.01*grip_force_close)))
            gzdbg << "Foce modifier="  << force_modifier << " Closing forces= " << force1 << ":" << force2 << std::endl;
    }

    publishGripperState();
}
////////////////////////////////////////////////////////////////////////////////
void gzParallelGripperPlugin::publishGripperState()
{
    message::GripCommand status;
    status.set_enable(grip_enabled);
    status.set_state(bGrasping);

    status.add_force(force1);
    status.add_force(force2);
    pub->Publish(status);
}

////////////////////////////////////////////////////////////////////////////////
void gzParallelGripperPlugin::Detach()
{
    for (auto v : this->virtualJoints)
    {
        if(bDebug)
            gzdbg << "Detach " << v->joint->GetName() << " virtual joint" << std::endl;
        v->collision1->SetCollideBits(0xffff);
        v->collision2->SetCollideBits(0xffff);
        v->parent->model->RemoveJoint(v->joint->GetName());
    }
    this->virtualJoints.clear();
}

////////////////////////////////////////////////////////////////////////////////
int gzParallelGripperPlugin::CheckAttach()
{
    std::lock_guard<std::mutex> lockGuard(mymutex);

    for (auto _contact : this->contacts)
    {
        physics::CollisionPtr internalCollision = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->EntityByName(_contact.collision1()));
        physics::CollisionPtr externalCollision = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->EntityByName(_contact.collision2()));

        if(internalCollision.get() == NULL)
            continue;
        if(externalCollision.get() == NULL)
            continue;
        std::string collision1 = internalCollision->GetScopedName();
        std::string collision2 = externalCollision->GetScopedName();

        bool foundCollision = false;

        if (std::find(this->collisionNames.begin(), this->collisionNames.end(), collision1) != this->collisionNames.end())
        {
            internalCollision = boost::dynamic_pointer_cast<physics::Collision>(
                   this->world->EntityByName(_contact.collision1()));
            externalCollision = boost::dynamic_pointer_cast<physics::Collision>(
                   this->world->EntityByName(_contact.collision2()));
            foundCollision = true;
        }
        else if (std::find(this->collisionNames.begin(), this->collisionNames.end(), collision2) != this->collisionNames.end())
        {
            internalCollision = boost::dynamic_pointer_cast<physics::Collision>(
                   this->world->EntityByName(_contact.collision2()));
            externalCollision = boost::dynamic_pointer_cast<physics::Collision>(
                   this->world->EntityByName(_contact.collision1()));
            foundCollision = true;
        }
        if (foundCollision)
        {

            auto jointName = this->model->GetName() + "_virtual_joint_" + std::to_string(this->jointId++);
            auto joint = this->model->CreateJoint(jointName, "fixed",
                                                  externalCollision->GetLink(),
                                                  internalCollision->GetLink());
            if(bDebug)
                gzdbg << "Create virtual joint " << jointName << std::endl;

            joint->SetProvideFeedback(true);
            auto offset = internalCollision->WorldPose() - externalCollision->WorldPose();
            joint->Load(externalCollision->GetLink(), internalCollision->GetLink(), offset);
            joint->Init();

            this->virtualJoints.push_back(new VirtualJoint(this, joint, internalCollision, externalCollision, ignition::math::Vector3d(0,0,0)));

            internalCollision->SetCollideBits(0x00ff);
            externalCollision->SetCollideBits(0xff00);
        }
    }

    this->contacts.clear();
    return this->virtualJoints.size();
}


////////////////////////////////////////////////////////////////////////////////
void gzParallelGripperPlugin::SetLinksGravityFlag(physics::ModelPtr objmodel, bool bFlag)
{
    physics::Link_V links = objmodel->GetLinks();

    ///  Set whether gravity affects a link.
    ///  True to enable gravity.
    for (size_t l = 0; l< links.size(); l++)
    {
        links[l]->SetGravityMode(bFlag);

    }
}
}



