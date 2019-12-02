#ifndef ASSIMP_H
#define ASSIMP_H

//assimp.h
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

// tf math
#include <tf/tf.h>
namespace tf
{
typedef tf::Vector3 BoundingBox[2];
}
/**
 * @brief The aiHandler class
 * Example:
 * std::string stlfile = "/home/isd/michalos/src/robot-agility/gz/models/gear_support/sku_peter_gear/new_big_gear_Rotatex_Centered_ZeroZmin.stl";
 * aiHandler ai;
 * ai.import( stlfile);
 * double h = ai.height( );
 * std::cout << "height=" << h << "\n";
 */
class aiHandler
{
public:
    bool import( const std::string& pFile);
    aiVector3D dimensions( );
    bool bounding_box(tf::BoundingBox & bounds);
    double height( );
private:
    void get_bounding_box (aiVector3D* min, aiVector3D* max);
    void get_bounding_box_for_node (const aiNode* nd,
        aiVector3D* min,
        aiVector3D* max);
    const aiScene* scene;
    // Create an instance of the Importer class
    Assimp::Importer importer;
};


#endif // ASSIMP_H
