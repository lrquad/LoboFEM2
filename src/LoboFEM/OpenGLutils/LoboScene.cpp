#include "LoboScene.h"
#include "Functions/parseString.h"
#include "Collision/CollisionDector/BVHCollisionDetector.h"

void Lobo::LoboScene::addMesh(const char *filename, bool uniform)
{
    LoboMesh *mesh = new LoboMesh(filename, uniform);
    mesh_list.push_back(mesh);
}

Lobo::LoboMesh *Lobo::LoboScene::getMesh(int meshid)
{
    return mesh_list[meshid];
}

void Lobo::LoboScene::runXMLscript(pugi::xml_node &scene_node)
{
    //clear scene
    deleteStdvectorPointer(mesh_list);

    for (pugi::xml_node tri_mesh_node : scene_node.children("TriMesh"))
    {
        //required Path
        if (tri_mesh_node.child("Path"))
        {
            std::string tri_path = tri_mesh_node.child("Path").text().as_string();
            bool uniform = false;
            bool smooth = true;

            if(tri_mesh_node.attribute("uniform"))
            {
                uniform = tri_mesh_node.attribute("uniform").as_bool();
            }

            if(tri_mesh_node.attribute("smooth"))
            {
                smooth = tri_mesh_node.attribute("smooth").as_bool();
            }

            this->addMesh(tri_path.c_str(),uniform);
            
            if (tri_mesh_node.child("Position"))
            {
                std::vector<float> position = Lobo::parseString<float>(tri_mesh_node.child("Position").text().as_string());
                this->mesh_list.back()->setPosition(position);
            }

            if (tri_mesh_node.child("Scale"))
            {
                std::vector<float> scales = Lobo::parseString<float>(tri_mesh_node.child("Scale").text().as_string());
                this->mesh_list.back()->setScale(scales);
            }

            this->mesh_list.back()->shader_config.flat_mode = !smooth;

            if(tri_mesh_node.child("Collision"))
            {
                bool isstatic = tri_mesh_node.child("Collision").attribute("static").as_bool();
                this->addMeshBVH(this->mesh_list.back(),isstatic,false);
            }
        }
    }
}

void Lobo::LoboScene::drawImGui(bool *p_open)
{
    ImGui::Begin("Scene",
                 p_open); // Create a window called "Hello, world!" and
                          // append into it.
    static int selected = 0;
    ImGui::BeginChild("left pane", ImVec2(150, 0), true);
    for (int i = 0; i < mesh_list.size(); i++)
    {
        ImGui::PushID(i);
        char label[128];
        sprintf(label, "%s", mesh_list[i]->obj_file_name.c_str());
        if (ImGui::Selectable(label, selected == i))
            selected = i;
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::BeginChild("item view", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()));
    if (selected < mesh_list.size() && selected >= 0)
    {
        if(mesh_list[selected]->bvh_dectector!=NULL)
        {
            ImGui::Text("bvh on.");
        }
        mesh_list[selected]->drawImGui();
    }
    ImGui::EndChild();
    ImGui::EndGroup();
    ImGui::End();
}

void Lobo::LoboScene::initialGL()
{
    for (int i = 0; i < mesh_list.size(); i++)
    {
        mesh_list[i]->initialGL();
    }
}

void Lobo::LoboScene::paintGL(LoboShader *shader)
{
    for (int i = 0; i < mesh_list.size(); i++)
    {
        mesh_list[i]->paintGL(shader);
    }
}

void Lobo::LoboScene::deleteGL()
{
    for (int i = 0; i < mesh_list.size(); i++)
    {
        mesh_list[i]->deleteGL();
    }
}

void Lobo::LoboScene::addMeshBVH(LoboMesh* trimesh,bool isstatic,bool selfcollision)
{
    BVHCollisionDetector* bvh = new BVHCollisionDetector(trimesh);
    bvh->setIsstatic(isstatic);
    bvh->setSelfCollisionTest(selfcollision);
    this->bvh_list.push_back(bvh);    
    trimesh->bvh_dectector = bvh_list.back();
    //add to world
    this->collision_world.addCollisionDetector(bvh);
}