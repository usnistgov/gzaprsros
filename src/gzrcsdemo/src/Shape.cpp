
#include <memory>
#include <sstream>
#include <string>

#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>

#include "gzrcsdemo/Shape.h"

#include <aprs_headers/Conversions.h>
#include "aprs_headers/Debug.h"
#include "gzrcsdemo/assimp.h"

using namespace tf;
namespace ShapeModel {

CShapes definitions;
CInstances instances;
std::mutex shapemutex;


static Vector3 comp_pt;
static double distFcn(const Vector3& lp, const Vector3& rp)
{
     return comp_pt.distance(lp) < comp_pt.distance(rp);
}

static Vector3 now_pt;
static double shapeDistFcn(const CShape * lp, const CShape *  rp)
{
     return now_pt.distance(lp->_location.getOrigin()) < now_pt.distance(rp->_location.getOrigin());
}

////////////////////////////////////////////////////////////////////////////////
CShape::CShape(std::string name, std::string type, tf::Pose pose, CShape * parent)
{
    this->_name=name;
    this->_type=type;
    this->_location=pose;
    this->_bEmpty=true;
    this->_parent=parent;
}

std::string CShape::name() const
{
    return _name;
}

void CShape::setName(const std::string &name)
{
    _name = name;
}

std::string CShape::type() const
{
    return _type;
}

void CShape::setType(const std::string &type)
{
    _type = type;
}

tf::Pose CShape::centroid() const
{
    return _location;
}

void CShape::setCentroid(const tf::Pose &centroid)
{
    _location = centroid;
}

////////////////////////////////////////////////////////////////////////////////
void CShapes::initDefinitions()
{
    tf::Pose dimensions;

    definitions.push_back(CShape("definition", "sku_part_small_gear", tf::Identity()));
    definitions.push_back(CShape("definition", "sku_part_medium_gear", tf::Identity()));
    definitions.push_back(CShape("definition", "sku_part_large_gear", tf::Identity()));

    /// Vessels
    definitions.push_back(CShape("definition", "sku_small_gear_vessel", tf::Identity()));
    CShape &vessel(definitions.back());

    // Need dimensions to understand if within container
    dimensions=tf::Pose (tf::Quaternion(0,0,0,1),tf::Vector3(.22,0.165,0));
    vessel._dimensions=dimensions;

    // The defines the slots and offsets from the centroid
    vessel._contains.push_back(CShape("slot1",
                                    "sku_part_small_gear",
                                    tf::Pose( QIdentity(), tf::Vector3(0.028575,0.028575, 0.0)),
                                    &vessel));

    vessel._contains.push_back(CShape("slot2",
                                    "sku_part_small_gear",
                                    tf::Pose( QIdentity(), tf::Vector3(0.028575,-0.028575, 0.0)),
                                    &vessel));
    vessel._contains.push_back(CShape("slot3",
                                    "sku_part_small_gear",
                                    tf::Pose( QIdentity(), tf::Vector3(-0.028575,0.028575, 0.0)),
                                    &vessel));
    vessel._contains.push_back(CShape("slot4",
                                    "sku_part_small_gear",
                                    tf::Pose( QIdentity(), tf::Vector3(-0.028575,-0.028575, 0.0)),
                                    &vessel));

    definitions.push_back(CShape("definition", "sku_large_gear_vessel", tf::Identity()));
    CShape &vessel1(definitions.back());

    // Need dimensions to understand if within container
    dimensions=tf::Pose (tf::Quaternion(0,0,0,1),tf::Vector3(.22,0.165,0));
    vessel1._dimensions=dimensions;

    // The defines the slots and offsets from the centroid
    vessel1._contains.push_back(CShape("slot1",
                                     "sku_part_large_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(-.055,0.0,  0.0)),
                                     &vessel));

    vessel1._contains.push_back(CShape("slot2",
                                     "sku_part_large_gear",
                                     tf::Pose( QIdentity(), tf::Vector3( .055, 0.0, 0.0)),
                                     &vessel));


    definitions.push_back(CShape("definition", "sku_medium_gear_vessel", tf::Identity()));
    CShape &vessel2(definitions.back());

    // Need dimensions to understand if within container
    dimensions=tf::Pose (tf::Quaternion(0,0,0,1),tf::Vector3(.22,0.165,0));
    vessel2._dimensions=dimensions;

    // The defines the slots and offsets from the centroid
    vessel2._contains.push_back(CShape("slot1",
                                     "sku_part_medium_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(0.0396,0.0396, 0.0)),
                                     &vessel));

    vessel2._contains.push_back(CShape("slot2",
                                     "sku_part_medium_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(-0.0396,0.0396, 0.0)),
                                     &vessel));
    vessel2._contains.push_back(CShape("slot3",
                                     "sku_part_medium_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(0.0396,-0.0396, 0.0)),
                                     &vessel));
    vessel2._contains.push_back(CShape("slot4",
                                     "sku_part_medium_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(-0.0396,-0.0396, 0.0)),
                                     &vessel));


    /// Kits

    definitions.push_back(CShape("definition", "sku_kit_s2l2_vessel", tf::Identity()));
    CShape &vessel3(definitions.back());

    // Need dimensions to understand if within container
    dimensions=tf::Pose (tf::Quaternion(0,0,0,1),tf::Vector3(.22,0.165,0));
    vessel3._dimensions=dimensions;

    // The defines the slots and offsets from the centroid
    vessel3._contains.push_back(CShape("slot1",
                                     "sku_part_small_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(0.03, -0.054, 0.0)),
                                     &vessel));

    vessel3._contains.push_back(CShape("slot2",
                                     "sku_part_small_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(-0.03, -0.054, 0.0)),
                                     &vessel));
    vessel3._contains.push_back(CShape("slot3",
                                     "sku_part_large_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(-.055, 0.0375, 0.0)),
                                     &vessel));
    vessel3._contains.push_back(CShape("slot4",
                                     "sku_part_large_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(.055, 0.0375, 0.0)),
                                     &vessel));

    definitions.push_back(CShape("definition", "sku_kit_m2l1_vessel", tf::Identity()));
    CShape &vessel4(definitions.back());

    // Need dimensions to understand if within container
    dimensions=tf::Pose (tf::Quaternion(0,0,0,1),tf::Vector3(0.16,0.19,0));
    vessel4._dimensions=dimensions;

    // The defines the slots and offsets from the centroid
    vessel4._contains.push_back(CShape("slot1",
                                     "sku_part_medium_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(-0.04, 0.055, 0.0)),
                                     &vessel));

    vessel4._contains.push_back(CShape("slot2",
                                     "sku_part_medium_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(0.04, 0.055, 0.0)),
                                     &vessel));
    vessel4._contains.push_back(CShape("slot3",
                                     "sku_part_large_gear",
                                     tf::Pose( QIdentity(), tf::Vector3(0.0, -0.04, 0.0)),
                                     &vessel));

}
////////////////////////////////////////////////////////////////////////////////
void CInstances::storeInstance(std::string name,
                               tf::Pose centroid,
                               std::string meshfile,
                               tf::Vector3 scale)
{
    // only store relevant gear related objects
    if(name.find("sku")==std::string::npos)
    {
        return;
    }


    std::unique_lock<std::mutex> lock(shapemutex);

    CShape * definition;

    // Find out if instance has already been defined
    auto it = find_if(instances.begin(), instances.end(), [&name](const CShape& obj) {return obj._name == name;});
    size_t index;
    if (it != instances.end())
    {
        index = std::distance(instances.begin(), it);
        Globals.bReadAllInstances=true;
    }
    else
    {
        std::string type (name);
        type = type.erase(type.find_last_not_of("0123456789")+1);
        instances.push_back(CShape(name, type, centroid));
        index=instances.size() -1;
        if((definition=definitions.findDefinition(type))!=NULL)
            instances.back()._contains = definition->_contains;
    }

    // if not all read and just updating pose then save mesh file bounding box
    if(!Globals.bReadAllInstances)
    {
        aiHandler ai;

        // Zero out mesh based dimensions
        instances[index]._height=0;
        instances[index]._bounding_box[0]=tf::Vector3(0,0,0);
        instances[index]._bounding_box[1]=tf::Vector3(0,0,0);

        // Read mesh file and fill in mesh based dimensions
        if(!meshfile.empty() && ai.import(meshfile))
        {
            instances[index]._height=ai.height()*scale.z();
            ai.bounding_box((tf::BoundingBox&)instances[index]._bounding_box)*scale;
        }
    }


    instances[index]._location=centroid;
}
////////////////////////////////////////////////////////////////////////////////
CShape * CInstances::findInstance(std::string name)
{
    // @todo this finds global instance name, not class instance name
    auto it = find_if(instances.begin(), instances.end(), [&name](const CShape& obj) {return obj._name == name;});
    if (it == instances.end())
        return NULL;
    return &(*it);
}
////////////////////////////////////////////////////////////////////////////////
CShape * CInstances::findSubstrInstance(std::string name)
{
    // @todo this finds global instance name, not class instance name
    auto it = find_if(instances.begin(), instances.end(), [&name](const CShape& obj) {return obj._name.find(name);});
    if (it == instances.end())
        return NULL;
    return &(*it);
}
////////////////////////////////////////////////////////////////////////////////
CShape * CInstances::getDefinition(std::string type)
{
    CShape * definition;
    if((definition=definitions.findDefinition(type))!=NULL)
        return definition;
    return NULL;
}
////////////////////////////////////////////////////////////////////////////////
CShape * CShapes::findDefinition(std::string type)
{
    for(size_t i=0; i< this->size(); i++)
    {
        if(this->at(i)._type == type)
            return &this->at(i);
    }
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
std::string CInstances::dumpLocations()
{
    std::stringstream ss;
    for(size_t i=0; i< this->size(); i++)
    {
        ss <<  this->at(i)._name << " at " << RCS::dumpPoseSimple( this->at(i)._location ) <<
            "height" << this->at(i)._height << "\n";
    }
    return ss.str();
}
std::string CInstances::dumpLocations(std::function<tf::Pose(tf::Pose)> Arg2)
{
    std::stringstream ss;
    for(size_t i=0; i< this->size(); i++)
    {
        tf::Pose pose = this->at(i)._location;
        pose=Arg2(pose);
        ss <<  this->at(i)._name << " at " << RCS::dumpPoseSimple(pose) <<
               "|height=" << this->at(i)._height << "\n";
    }
    return ss.str();
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Instances::FindFreeGear - find gear (using all instances) in vessel.
 * Gears in vessel are "free".
 * Note, vessels hold all the same part type.
 * Kits hold combinations of part types.
 * @param myparts list of instances (find if instance inside of vessesl).
 * @return
 */
CShape * CInstances::findFreeGear(std::vector<std::string> myparts) // , tf::Pose now_pose)
{

//    tf::Vector3 now_xyz=now_pose.getOrigin();
    std::vector<CShape *> potential_shapes;
    // mutex copy of latest positions of gears, vessels, and kits
    // not sure we shouldn't just muutex acccess.
    ShapeModel::CInstances now_instances;
    {
        std::unique_lock<std::mutex> lock(shapemutex);
        now_instances=instances;
    }
#ifdef DEBUG_ALL
    for(size_t i=0; i< this->size(); i++)
    {
        std::cout <<  now_instances[i].name << " at " << RCS::DumpPoseSimple( now_instances[i].centroid )<< "\n";
    }
#endif
    // Problem with this is that instances get all mixed up.
    // Make global ini switch
    //std::random_shuffle(this->begin(), this->end());
    for(size_t i=0; i< now_instances.size(); i++)
    {
        // Don't want a kit
        if(now_instances[i]._name.find("kit") != std::string::npos)
            continue;
        // don't vant a vessesl
        if(now_instances[i]._name.find("vessel") != std::string::npos)
            continue;
        // If not a sku skip
        if(now_instances[i]._name.find("sku") == std::string::npos)
            continue;

        // now_instances should be a part if we are here.

        // search if one of my robot parts, if not continue
        // Each robot has a list of parts it can access, so entire list is bigger.
        std::string gearname = now_instances[i]._name;
        if(std::find(myparts.begin(), myparts.end(), gearname)==myparts.end())
            continue;

        // If we are here, found a gear
        // Now find a gear IN a vessel NOT inside a kit.
        CShape * vessel = insideContainer(now_instances,
                                         &now_instances[i],
                                         Globals.trimmedTokenize("vessel", ","),
                                         Globals.trimmedTokenize("kit", ",")
                                         );

        // Should be inside a vessel, NOT A KIT

        // Wasn't inside any vessel - ignore most likely tipped over
        if(vessel==NULL)
        {
            continue;
            // return &this->at(i);
        }


        // check to make sure inside of container slot
        bool bInside = isInsideContainerSlot(vessel, &now_instances[i]);

        // What if now_instances[i] !=  this->at(i) ??
        // So do lookup of part based on instance name
        if(bInside)
        {
            potential_shapes.push_back(this->findInstance(instances[i]._name));
            //return this->FindInstance(instances[i].name);
        }

    }
    if(potential_shapes.size()==0)
        return NULL;

#ifdef FIXME
    // Sort by distance to current position
    now_pt = RCS::Convert<tf::Pose,tf::Vector3> (now_pose);

    // sort free gears by closet to current position
    // if we skip sorting it is as is now - by alphabetical order.
    if(Globals.bClosestFree)
        std::sort (potential_shapes.begin(), potential_shapes.end(), shapeDistFcn);
#endif
    // return shape
    return potential_shapes[0];
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Instances::InsideContainerSlot
/// \param container vessel holding part. Should be in a container slot
/// \param part pointer to part shape definition
/// \return  true if inside slot, return false if not in slot.
/// Doesn't detect sideways.
///??
///
bool CInstances::isInsideContainerSlot(CShape * container,   CShape * part)
{

    // If we are here, found a gear vessel, now find its definition
    CShape * vessel = definitions.findDefinition(container->_type);
    if(vessel==NULL)
    {
        // @todo this is a big time error
        return false;
    }

    // No slots must not be container
    if(vessel->_contains.size() < 1)
        return false;

    // Now look for a matching slot in this vessel
    // If not in vessel slot most likely physically challenged
    CShape * slot;

    for(size_t j=0; j< vessel->_contains.size(); j++)
    {
        // vessels only contain all the same part types
        if(vessel->_contains[j]._type != part->_type)
        {
            assert(0);
        }

        slot=&vessel->_contains[j];

        // FIXME: this should be post multiplied by rotation.
        // position of slot - container instance centroid plus slot offset
        Vector3 slot_location = container->_location.getOrigin() +
                slot->_location.getOrigin();

        // Where the gear instance is located
        Vector3 part_location = part->_location.getOrigin();

        // Fixme: maybe should check if z's near, as well as part with 0,0,1 orientation - pointing up.
        if((slot_location - part_location).length() < 0.1 )
            return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////
CShape * CInstances::insideContainer(std::vector<CShape> & now_instances,
                                   CShape * part,
                                   std::vector<std::string> types_container,
                                   std::vector<std::string> not_types_container)
{
    for(size_t i=0; i< now_instances.size(); i++)
    {
        CShape & shape( now_instances[i]);
        if(shape._name.find("sku") == std::string::npos)
            continue;

        // THis does a complete match.
        std::string name = shape._name;
        // Not kit has vessel attached at end of name!



        // Now look for container types
        size_t j;
        for(j=0; j< types_container.size(); j++)
        {
            if( name.find(types_container[j]) != std::string::npos)
                break;
        }

        if(j == types_container.size())
            continue;

        // Now look for non container types
        for(j=0; j< not_types_container.size(); j++)
        {
            if( name.find(not_types_container[j]) != std::string::npos)
                break;
        }
        if(j != types_container.size())
            continue;



        // If we are here, found a gear vessel, now find its definition
        CShape * vessel = definitions.findDefinition(shape._type);
        if(vessel==NULL)
        {
            // @todo this is a big time error
            continue;
        }

        // make sure containuer type matches part type
        if(vessel->_contains[j]._type != part->_type)
        {
            // vessels only contain like part types
            continue;
        }

        // Assume square defintion - although large vessel rectangle and unsure orientation
        tf::Vector3 dimensions = vessel->_dimensions.getOrigin()/2.0;  // this contains dimensions
        double minx = now_instances[i]._location.getOrigin().x() - dimensions.x();
        double maxx = now_instances[i]._location.getOrigin().x() + dimensions.x();
        double miny = now_instances[i]._location.getOrigin().y() - dimensions.y();
        double maxy = now_instances[i]._location.getOrigin().y() + dimensions.y();

        // for this test phase, containers don't move
        if(part->_location.getOrigin().x() > minx &&
                part->_location.getOrigin().x() < maxx &&
                part->_location.getOrigin().y() > miny &&
                part->_location.getOrigin().y() < maxy )
            return &now_instances[i];

    }
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Cycle through all instances - look for gear part, see if in slot, if so return false;
// Othwerwise nothing found in slot, return true
bool CInstances::findEmptyContainerSlot(std::vector<CShape> &now_instances,
                                   CShape * kit,
                                   CShape * slot)
{
    // Position of slot in actual kit - rotation included
    // slot= kit_centroid + (kit_rotation * slotoffset)
    tf::Matrix3x3 m(kit->_location.getRotation());
    Vector3 pos_slot = kit->_location.getOrigin() +  (m*slot->_location.getOrigin());

    // Cycle through all instance - look for gear part, see if in slot, if so return true;
    for(size_t i=0; i< now_instances.size(); i++)
    {
        // Must be a demo part model with sku tag, else continue
        if(now_instances[i]._name.find("sku") == std::string::npos)
            continue;

        // Must be a part to test if in container slot, else continue
        if(now_instances[i]._name.find("part") == std::string::npos)
            continue;

        // Determine if part contained in this slot - some error sphere
        CShape & part(  now_instances[i] );

        // make sure containuer type matches part type
        if(slot->_type != part._type)
        {
            // slots should only contain like part types
            continue;
        }

        // Position of part
        Vector3 pos_part = part._location.getOrigin();

        // Compute distance between slot and part
        // if close = then return false - NOT empty
        double mag = (pos_slot - pos_part).length();
        if(fabs(mag) < 0.05)
            return false;
    }
    return true;
}


////////////////////////////////////////////////////////////////////////////////
int CInstances::findFreeGearKitSlot(CShape* gear,
                                   tf::Pose &pos_slot,
                                    std::vector<std::string> myparts)
{

    std::string type = gear->_type;
    std::vector<Vector3> pos_slots;
    ShapeModel::CInstances now_instances;
    {
        std::unique_lock<std::mutex> lock(shapemutex);
        now_instances=instances;
    }

    for(size_t i=0; i< now_instances.size(); i++)
    {
        // if type not kit type try next
        if(now_instances[i]._name.find("kit") == std::string::npos)
            continue;

        // search if one of my robot kits, if not continue
        if(std::find(myparts.begin(), myparts.end(), now_instances[i]._name)==myparts.end())
            continue;

        // Find matching instance definition
        CShape * kit = definitions.findDefinition(now_instances[i]._type);
        if(kit==NULL)
            continue;


        // Now search for matching slot
        // @todo assume empty for now
        for(size_t j=0; j< kit->_contains.size(); j++)
        {
            if(kit->_contains[j]._type == type)
            {
                // If not empty slot, continue
                if(!this->findEmptyContainerSlot(now_instances,     // current instance position
                                             &now_instances[i], // kit
                                             &kit->_contains[j]))  // slot
                    continue;
                // Position of slot in kit= kit_centroid + (kit_rotation * slotoffset)
                tf::Matrix3x3 m(now_instances[i]._location.getRotation());
                Vector3 pos_slot = (now_instances[i]._location.getOrigin()) +  m*kit->_contains[j]._location.getOrigin();
                pos_slots.push_back(pos_slot);
               // pos_slots.push_back( now_instances[i]._location.getOrigin() +  kit->_contains[j]._location.getOrigin());

            }
        }
    }

    if(pos_slots.size()>0 )
    {
        comp_pt = gear->_location.getOrigin();
        // find closest open slot to gear
        if(Globals.bClosestOpenSlot)
            std::sort (pos_slots.begin(), pos_slots.end(), distFcn);
        pos_slot=RCS::Convert<tf::Vector3, tf::Pose>(pos_slots[0]);
    }
    return pos_slots.size();
}
}

