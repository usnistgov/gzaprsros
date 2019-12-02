#include "gzrcs/assimp.h"
#include <iostream>
#include <fstream>

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

void aiHandler::get_bounding_box_for_node (const aiNode* nd,
                                           aiVector3D* min,
                                           aiVector3D* max)

{
    aiMatrix4x4 prev;
    unsigned int n = 0, t;

    for (; n < nd->mNumMeshes; ++n) {
        const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
        for (t = 0; t < mesh->mNumVertices; ++t) {

            aiVector3D tmp = mesh->mVertices[t];

            min->x = aisgl_min(min->x,tmp.x);
            min->y = aisgl_min(min->y,tmp.y);
            min->z = aisgl_min(min->z,tmp.z);

            max->x = aisgl_max(max->x,tmp.x);
            max->y = aisgl_max(max->y,tmp.y);
            max->z = aisgl_max(max->z,tmp.z);
        }
    }

    for (n = 0; n < nd->mNumChildren; ++n) {
        get_bounding_box_for_node(nd->mChildren[n],min,max);
    }
}


void aiHandler::get_bounding_box (aiVector3D* min, aiVector3D* max)
{

    min->x = min->y = min->z =  1e10f;
    max->x = max->y = max->z = -1e10f;
    get_bounding_box_for_node(scene->mRootNode,min,max);
}

aiVector3D aiHandler::dimensions( )
{
    aiVector3D scene_min, scene_max; //, scene_center;
    get_bounding_box(&scene_min, &scene_max);
    aiVector3D dimension;
    dimension.x = scene_max.x-scene_min.x;
    dimension.y = scene_max.y - scene_min.y ;
    dimension.z = scene_max.z - scene_min.z ;
    return dimension;
}
bool aiHandler::bounding_box(tf::BoundingBox & bb )
{
    aiVector3D scene_min, scene_max; //, scene_center;
    get_bounding_box(&scene_min, &scene_max);
 ///   bb[0].;
    bb[0].setX(scene_min.x);
    bb[0].setX(scene_min.y);
    bb[0].setX(scene_min.z);
    bb[1].setX(scene_max.x);
    bb[1].setX(scene_max.y);
    bb[1].setX(scene_max.z);
    return 0;
}

double aiHandler::height( )
{
    aiVector3D scene_min, scene_max; //, scene_center;
    get_bounding_box(&scene_min, &scene_max);
    return fabs(scene_max.z - scene_min.z );
}

bool aiHandler::import( const std::string& pFile)
{
//    //check if file exists
//    std::ifstream fin(pFile.c_str());
//    if(!fin.fail()) {
//        fin.close();
//    }
//    else{
//        printf("Couldn't open file: %s\n", pFile.c_str());
//        printf("%s\n", importer.GetErrorString());
//        return false;
//    }

    scene = importer.ReadFile( pFile,
                               aiProcess_CalcTangentSpace       |
                               aiProcess_Triangulate            |
                               aiProcess_JoinIdenticalVertices  |
                               aiProcess_SortByPType);
    //, aiProcessPreset_TargetRealtime_Quality);

    // If the import failed, report it
    if( !scene)
    {
        printf("%s\n", importer.GetErrorString());
        return false;
    }

    // Now we can access the file's contents.
    printf("Import of scene %s succeeded.",pFile.c_str());

    aiVector3D scene_min, scene_max;
    get_bounding_box(&scene_min, &scene_max);
    float tmp,scaleFactor;
    tmp = scene_max.x-scene_min.x;
    tmp = scene_max.y - scene_min.y > tmp?scene_max.y - scene_min.y:tmp;
    tmp = scene_max.z - scene_min.z > tmp?scene_max.z - scene_min.z:tmp;
    scaleFactor = 1.f / tmp;

    // We're done. Everything will be cleaned up by the importer destructor
    return true;

}
