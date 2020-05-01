#include "DynamicSimulator.h"
#include "LoboDynamic/LoboDynamicScene.h"
#include "imgui.h"
#include "LoboVolumtricMesh/LoboTetMesh.h"
#include "LoboDynamic/LoboDynamic.h"

Lobo::DynamicSimulator::DynamicSimulator(LoboDynamicScene *parent_scene_)
    : parent_scene(parent_scene_)
{
    bind_tetMesh = NULL;
    target_tetmesh_id = -1;
    skipsteps = 1;
    
}

Lobo::DynamicSimulator::~DynamicSimulator() {
    
}

void Lobo::DynamicSimulator::drawImGui()
{
    ImGui::Text("num dynamic models: %d", models.size());

    if (bind_tetMesh != NULL)
    {
        ImGui::Text("Target Tet mesh id: %d", target_tetmesh_id);
        ImGui::Text("Tet mesh name %s", bind_tetMesh->filebase.c_str());
    }
    else
    {
        ImGui::Text("Target Tet mesh id: %d", target_tetmesh_id);
    }
}


void Lobo::DynamicSimulator::setBindTetmeshFromScene(int tetmesh_id)
{   
    bind_tetMesh = this->parent_scene->tetmesh_in_scene[tetmesh_id];
}

void Lobo::DynamicSimulator::runXMLscript(pugi::xml_node &solver_node)
{
    if (solver_node.child("TargetTetMesh"))
    {
        target_tetmesh_id = solver_node.child("TargetTetMesh").attribute("id").as_int();
        setBindTetmeshFromScene(target_tetmesh_id);
    }

    

}

void Lobo::DynamicSimulator::precompute()
{
    
}