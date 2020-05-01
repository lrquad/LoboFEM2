#pragma once
#include "LoboMesh/LoboMesh.h"
#include "Shaders/LoboShader.h"
#include "Functions/deleteSTDvector.h"
#include <imgui.h>
#include "Utils/pugixml/pugixml.hpp"

#include "Collision/CollisionDector/CollisionWorld.h"

class BVHCollisionDetector;

namespace Lobo {
    
class LoboScene {
public:
    LoboScene(){};
    ~LoboScene(){
        deleteStdvectorPointer(mesh_list);
        deleteStdvectorPointer(bvh_list);
    };

    virtual void runXMLscript(pugi::xml_node &scene_node);

    virtual void addMesh(const char* filename, bool uniform = true);

    virtual LoboMesh* getMesh(int meshid);

    virtual void drawImGui(bool* p_open = NULL);

    virtual void initialGL();

    virtual void paintGL(LoboShader* shader);

    virtual void deleteGL();

    std::vector<LoboMesh*> mesh_list;
    //LoboMesh boudning box bvh

    std::vector<BVHCollisionDetector*> bvh_list;
    CollisionWorld collision_world;

protected:

    virtual void addMeshBVH(LoboMesh* trimesh,bool isstatic,bool selfcollision);


};
}  // namespace Lobo