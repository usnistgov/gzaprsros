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

#include "gzmodelplugin.h"
//using namespace gazebo::math;

/////////////////////////////////////////////////////////////////////////////
static std::string replaceAll(std::string subject, const std::string& search,
                              const std::string& replace)
{
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
        subject.replace(pos, search.length(), replace);
        pos += replace.length();
    }
    return subject;
}

static std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}
/**
* @brief Tokenize takes a string and delimiters and parses into vector
* @param str string to tokenize
* @param delimiters string containing delimiters
 * @return  std vector of tokens from parsed string
 */
static std::vector<std::string> tokenize (const std::string & str,
                                          const std::string & delimiters)
{
    std::vector<std::string> tokens;
    std::string::size_type   delimPos = 0, tokenPos = 0, pos = 0;

    if ( str.length( ) < 1 )
    {
        return tokens;
    }

    while ( 1 )
    {
        delimPos = str.find_first_of(delimiters, pos);
        tokenPos = str.find_first_not_of(delimiters, pos);

        if ( std::string::npos != delimPos )
        {
            if ( std::string::npos != tokenPos )
            {
                if ( tokenPos < delimPos )
                {
                    tokens.push_back(str.substr(pos, delimPos - pos));
                }
                else
                {
                    tokens.push_back("");
                }
            }
            else
            {
                tokens.push_back("");
            }
            pos = delimPos + 1;
        }
        else
        {
            if ( std::string::npos != tokenPos )
            {
                tokens.push_back(str.substr(pos));
            }
            else
            {
                tokens.push_back("");
            }
            break;
        }
    }

    return tokens;
}

/**
                * @brief TrimmedTokenize takes a string and delimiters and parses into
                * vector,
                * but Trims tokens of leading and trailing spaces before saving
                * @param str string to tokenize
                * @param delimiters string containing delimiters
                * @return  std vector of tokens from parsed string Trimmed
                *  tokens of leading and trailing spaces
                */
static std::vector<std::string> trimmedTokenize (std::string value,
                                                 std::string delimiter)
{
    std::vector<std::string> tokens = tokenize(value, delimiter);

    for ( size_t i = 0; i < tokens.size( ); i++ )
    {
        if ( tokens[i].empty( ) )
        {
            tokens.erase(tokens.begin( ) + i);
            i--;
            continue;
        }
        tokens[i] = trim(tokens[i]);
    }
    return tokens;
}

/////////////////////////////////////////////////////////////////////////////
static bool findEnvFile(std::vector<std::string> envs, std::string path, std::string & fullpath)
{
    struct stat buffer;
    fullpath.clear();
    std::string file ;
    for(size_t i=0; i< envs.size(); i++)
    {
        const char *v = std::getenv( envs[i].c_str() );

        if( v == NULL )
            continue;

        std::vector<std::string> dirs=trimmedTokenize(std::string(v), ":");
        for(size_t j=0; j< dirs.size(); j++)
        {
            file = dirs[j];
            if(dirs[j].back()!= '/')
                file += "/";
            file += path;
            if (stat (file.c_str(), &buffer) == 0)
            {
                if(!fullpath.empty())
                    file+=";";
                fullpath+=file;
            }
        }
    }
    return !fullpath.empty();
}

namespace gazebo
{
/////////////////////////////////////////////////////////////////////////////
gzModelPlugin::gzModelPlugin() :  WorldPlugin()
{
    b_debug=false;
    // Update interval of 50 milliseconds. Hope fast enough.
    d_update_period=0.05;
}
/////////////////////////////////////////////////////////////////////////////
void gzModelPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    std::cout << "gzModelPlugin: Compiled " << __DATE__ << " " << __TIME__ << "\n" << std::flush;

    world=_world;
    Connect();

    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&gzModelPlugin::onWorldUpdate, this));

    if (_sdf->HasElement("debug"))
    {
        b_debug= _sdf->Get<bool>("debug");
    }
    if (_sdf->HasElement("updateRate"))
    {
        d_update_period = 1./(_sdf->Get<double>("updateRate"));
    }


}
/////////////////////////////////////////////////////////////////////////////
void gzModelPlugin::Connect()
{
    // Create our node for communication
    node=gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    // Publish to a Gazebo topic
    pub = node->Advertise<gazebo::msgs::Model>("~/ariac/model");
    // Wait for a subscriber to connect
    //pub->WaitForConnection();
}
/////////////////////////////////////////////////////////////////////////////
void gzModelPlugin::onWorldUpdate()
{


    // If no one is listening, don't publish.
    if (!pub || !pub->HasConnections())
    {
        n_readers=0;
        return;
    }

    // Determine time period since last update.
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = this->world->SimTime();
#else
    common::Time current_time = this->world->GetSimTime();
#endif
    double seconds_since_last_clear = ( current_time - last_update_time_ ).Double();

    // If not enough time has elapsed since last update return
    if ( seconds_since_last_clear > 1.0 )
    {
        last_location.clear();
    }

    if( n_readers==0)
    {
        last_location.clear();
        n_readers++;
    }

    // Determine seconds since last update.
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    // If not enough time has elapsed since last update return
    if ( seconds_since_last_update < d_update_period )
        return;

    // Update time
    last_update_time_+= common::Time ( d_update_period );

    //  Model_V === std::vector<ModelPtr>
#if GAZEBO_MAJOR_VERSION >= 8
    physics::Model_V models=world->Models();
#else
      physics::Model_V models=world->GetModels();
#endif
    physics::Model_V::iterator m_it;
    std::string model_name;

    for (size_t m = 0; m< models.size(); m++)
    {
        try {
            physics::ModelPtr model = models.at(m);
            model_name = model->GetName();

            if(b_debug)
                std::cout << "Plugin Model " << model_name <<"\n";

            // Get first link as the physics tweaking effects model pose
            // assignment of pose in world is reflected in world pose - generally unless bouncing around
            Pose pose;
            Pose mpose, wpose;

            physics::Link_V links = model->GetLinks(); // (*m_it)->GetLinks();
            if(links.size() > 0 )
#if GAZEBO_MAJOR_VERSION >= 8
                mpose = links[0]->WorldPose();
#else
                mpose = links[0]->GetWorldPose().Ign();
#endif
            else
                mpose=Pose(Vector3d(0,0,0), Quaterniond(1,0,0,0));

            if(b_debug)
                std::cout << "Compute pose " << model_name <<"\n";

#if GAZEBO_MAJOR_VERSION >= 8
            wpose= model->WorldPose();
#else
            wpose= model->GetWorldPose().Ign();
#endif
            pose=mpose;
            pose.CoordPositionAdd(wpose);

#if 0
            // Check to see if significant pose change - only linear for now
            // This is a pose difference - not sure what it means
            double mag = pose.pos.Distance(last_location[model_name].pos);
            if(fabs(mag) < 0.001)
            {
                continue;
            }

            // update pose if changed significantly
            last_location[model_name]=pose;
#endif

            if(b_debug)
                std::cout << "Set msg pose " << model_name <<"\n";

            gazebo::msgs::Model msg;
            msg.set_name(model_name.c_str());
            msg.set_id(model->GetId());
#if GAZEBO_MAJOR_VERSION >= 8
            msg.set_allocated_pose(new gazebo::msgs::Pose(gazebo::msgs::Convert (pose)) );
#else
            msg.set_allocated_pose(new gazebo::msgs::Pose(gazebo::msgs::Convert (pose)) );
#endif


            // At one point this worked but is so convoluted and prone to yavc (yet
            // another version change) not worth it.
#ifdef BOUNDING_BOX

            // Yet another way to return bounding box. Below is done using mesh.
            // return bounding box of model inside visual[0]
            ignition::math::Box bbox = model->BoundingBox();
            ignition::math::Vector3d vbox( bbox.Max()-bbox.Min());
            ::gazebo::msgs::Visual* visual = msg.add_visual();
            std::string * vname= new std::string("bbox");
            std::string * vmodel_name=new std::string(model_name);
            visual->set_allocated_name(vname);
            visual->set_allocated_parent_name(vmodel_name);
            gazebo::msgs::Geometry *g= new gazebo::msgs::Geometry();
            gazebo::msgs::BoxGeom *bg = new  gazebo::msgs::BoxGeom();
            gazebo::msgs::Vector3d * v = new gazebo::msgs::Vector3d();
            v->set_x(vbox.X());
            v->set_y(vbox.Y());
            v->set_z(vbox.Z());
            visual->set_allocated_geometry(g);
            g->set_allocated_box(bg);
            bg->set_allocated_size(v);
#if 0
            try {
                while(1)
                {

                    // assume all model have at least one link. What is its unique ID?
                    if(links.size() == 0 )
                        break;

                    const std::map<uint32_t, msgs::Visual> & visuals = links[0]->Visuals();
                    if(visuals.size() == 0)
                        break;
                    std::map<uint32_t, msgs::Visual>::const_iterator vit= visuals.begin();
                    std::string meshfile;
                    if(!(*vit).second.has_geometry())
                        break;
                    const ::gazebo::msgs::Geometry & geo = (*vit).second.geometry();
                    if(!geo.has_type() || geo.type() != ::gazebo::msgs::Geometry::MESH)
                        break;
                    const ::gazebo::msgs::MeshGeom & mesh =  geo.mesh();
                    if(!mesh.has_filename())
                    {
                        break;
                    }

                    std::string modelfile= mesh.filename();
                    //meshfile= mesh.filename();
                    if(modelfile.find("model:/")!=std::string::npos)
                    {
                        modelfile=replaceAll(modelfile,"model:/","");
                        std::vector<std::string> envs={"GAZEBO_MODEL_PATH"};
                        findEnvFile(envs,modelfile,meshfile);
                    }
                    else  if(modelfile.find("file:/")!=std::string::npos)
                    {
                        meshfile=replaceAll(modelfile,"file:/","");
                    }
                    else
                    {
                        // Model::instances.storeInstance(name, tfpose, meshfile, scale);


                        //    const google::protobuf::Message *prototype =
                        //      google::protobuf::MessageFactory::generated_factory()->GetPrototype(
                        //          _msg->GetDescriptor());
                        //    std::unique_ptr<google::protobuf::Message> payload(prototype->New());
                        // //   payload->ParseFromString(response->serialized_data());
                        //    payload->ParseFromString(_msg->SerializeAsString());
                        //    std::cout << payload->DebugString() << std::endl;
                        //   std::cout << _msg->DebugString() << std::endl;
                        meshfile= modelfile;
                    }


                    // Build protobuf message
                    ::gazebo::msgs::Link * link = msg.add_link();
                    link->set_id(links[0]->GetId());
                    link->set_name(links[0]->GetName());
                    ::gazebo::msgs::Visual* v = link->add_visual();
                    v->set_name((*vit).second.name());
                    v->set_parent_name((*vit).second.parent_name());
                    ::gazebo::msgs::Geometry * g (new ::gazebo::msgs::Geometry());
                    v->set_allocated_geometry(g);
                    g->set_type(::gazebo::msgs::Geometry::MESH);
                    ::gazebo::msgs::MeshGeom * msgmesh = new ::gazebo::msgs::MeshGeom(geo.mesh());
                    msgmesh->set_filename(meshfile);
                    g->set_allocated_mesh(msgmesh);
                    ::gazebo::msgs::Vector3d scale = msgmesh->scale();

                    pub->Publish(msg);
                    // exit bogus loop there to help with error checking breaks.
                    break; // since no more links

                }

                continue;
            }
            catch(...)
            {
                std::cout << "Model plugin error in determining mesh\n";
            }


#endif
#endif


#if 0
            for(size_t j=0; j< links.size(); j++)
            {
                ::gazebo::msgs::Link* link = msg.add_link();
                link->set_id(links[j]->GetId());
                link->set_name(links[j]->GetName());
            }
#endif

            pub->Publish(msg);

            if(b_debug)
                std::cerr << "Publish Model = " << (*m_it)->GetName()<< " at: [" <<  pose << "]\n";
        }
        catch(...)
        {
            gzerr << "Model  " << (*m_it)->GetName()<< " Execption thrown\n";

        }

    }
}


}
